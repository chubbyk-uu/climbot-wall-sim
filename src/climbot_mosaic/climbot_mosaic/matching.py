# Copyright 2026 jerry
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""SIFT extraction and bidirectional robust matching in predicted overlap regions."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

from climbot_mosaic.candidates import OverlapCandidate
from climbot_mosaic.mosaic_inputs import FrameKey
from climbot_mosaic.projection import FrameProjection
import cv2
import numpy as np


class MatchingError(ValueError):
    """Feature or matching data violates the deterministic local-match contract."""


@dataclass(frozen=True)
class FeatureSet:
    """Stable SIFT locations and descriptors for one processed image."""

    key: FrameKey
    points_px: np.ndarray
    descriptors: np.ndarray


@dataclass(frozen=True)
class MatchConfig:
    """Recorded baseline parameters; these are algorithm settings, not acceptance limits."""

    ratio_test: float = 0.75
    ransac_threshold_m: float = 0.005
    minimum_mutual_matches: int = 4
    ransac_max_iterations: int = 2000
    ransac_confidence: float = 0.999


@dataclass(frozen=True)
class LocalMatch:
    """Explicit accepted or rejected evidence for one spatial candidate."""

    first: FrameKey
    second: FrameKey
    status: str
    reason: str
    first_overlap_features: int
    second_overlap_features: int
    ratio_matches_first_to_second: int
    ratio_matches_second_to_first: int
    mutual_matches: int
    ransac_inliers: int
    transform_second_to_first_xy: tuple[float, ...] | None
    overlap_center_correction_m: float | None
    residual_median_m: float | None
    residual_p95_m: float | None


def extract_sift(key: FrameKey, image_path: Path, use_clahe: bool = False) -> FeatureSet:
    """Extract deterministic SIFT descriptors from one rectified mono8 image."""
    image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
    if image is None or image.dtype != np.uint8 or image.ndim != 2:
        raise MatchingError(f'feature input is not mono8: {image_path}.')
    cv2.setNumThreads(1)
    cv2.setRNGSeed(0)
    detection = image
    if use_clahe:
        detection = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(image)
    keypoints, descriptors = cv2.SIFT_create().detectAndCompute(detection, None)
    if not keypoints or descriptors is None:
        return FeatureSet(key, np.empty((0, 2), np.float32), np.empty((0, 128), np.float32))
    points = np.asarray([item.pt for item in keypoints], dtype=np.float32)
    values = np.asarray(descriptors, dtype=np.float32)
    order = np.lexsort((points[:, 1], points[:, 0]))
    return FeatureSet(key, points[order], values[order])


def _inside_overlap(features: FeatureSet, projection: FrameProjection,
                    polygon_xy_m: tuple[tuple[float, float], ...]) -> np.ndarray:
    if features.points_px.size == 0:
        return np.empty(0, dtype=np.int64)
    homography = np.asarray(projection.homography_image_to_wall, dtype=np.float64).reshape(3, 3)
    wall_points = cv2.perspectiveTransform(features.points_px[None, :, :], homography)[0]
    polygon = np.asarray(polygon_xy_m, dtype=np.float32)
    indices = [index for index, point in enumerate(wall_points)
               if cv2.pointPolygonTest(polygon, tuple(map(float, point)), False) >= 0.0]
    return np.asarray(indices, dtype=np.int64)


def _ratio_matches(first: np.ndarray, second: np.ndarray,
                   ratio: float) -> dict[int, int]:
    if len(first) < 1 or len(second) < 2:
        return {}
    pairs = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False).knnMatch(first, second, k=2)
    return {pair[0].queryIdx: pair[0].trainIdx for pair in pairs
            if len(pair) == 2 and pair[0].distance < ratio * pair[1].distance}


def _rejected(candidate: OverlapCandidate, reason: str, counts: tuple[int, ...]) -> LocalMatch:
    return LocalMatch(candidate.first, candidate.second, 'rejected', reason, *counts,
                      0, None, None, None, None)


def _fit_rigid(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Least-squares SE(2) transform with fixed unit scale."""
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    covariance = (source - source_center).T @ (target - target_center)
    left, _, right = np.linalg.svd(covariance)
    rotation = right.T @ left.T
    if np.linalg.det(rotation) < 0.0:
        right[-1, :] *= -1.0
        rotation = right.T @ left.T
    translation = target_center - rotation @ source_center
    return np.column_stack((rotation, translation))


def match_candidate(candidate: OverlapCandidate,
                    first_features: FeatureSet, second_features: FeatureSet,
                    first_projection: FrameProjection, second_projection: FrameProjection,
                    config: MatchConfig = MatchConfig()) -> LocalMatch:
    """Match one predicted overlap and estimate a robust metric SE(2)-like correction."""
    if not 0.0 < config.ratio_test < 1.0:
        raise MatchingError('ratio_test must lie strictly between zero and one.')
    if not math.isfinite(config.ransac_threshold_m) or config.ransac_threshold_m <= 0.0:
        raise MatchingError('ransac_threshold_m must be finite and positive.')
    first_indices = _inside_overlap(first_features, first_projection, candidate.intersection_xy_m)
    second_indices = _inside_overlap(
        second_features, second_projection, candidate.intersection_xy_m)
    first_to_second = _ratio_matches(
        first_features.descriptors[first_indices], second_features.descriptors[second_indices],
        config.ratio_test)
    second_to_first = _ratio_matches(
        second_features.descriptors[second_indices], first_features.descriptors[first_indices],
        config.ratio_test)
    mutual = sorted(
        (first_index, second_index)
        for first_index, second_index in first_to_second.items()
        if second_to_first.get(second_index) == first_index)
    counts = (len(first_indices), len(second_indices), len(first_to_second),
              len(second_to_first), len(mutual))
    if len(mutual) < config.minimum_mutual_matches:
        return _rejected(candidate, 'insufficient_mutual_matches', counts)
    first_pixels = np.asarray(
        [first_features.points_px[first_indices[item[0]]] for item in mutual], np.float32)
    second_pixels = np.asarray(
        [second_features.points_px[second_indices[item[1]]] for item in mutual], np.float32)
    first_h = np.asarray(first_projection.homography_image_to_wall).reshape(3, 3)
    second_h = np.asarray(second_projection.homography_image_to_wall).reshape(3, 3)
    first_wall = cv2.perspectiveTransform(first_pixels[None, :, :], first_h)[0]
    second_wall = cv2.perspectiveTransform(second_pixels[None, :, :], second_h)[0]
    cv2.setRNGSeed(0)
    transform, inliers = cv2.estimateAffinePartial2D(
        second_wall, first_wall, method=cv2.RANSAC,
        ransacReprojThreshold=config.ransac_threshold_m,
        maxIters=config.ransac_max_iterations, confidence=config.ransac_confidence,
        refineIters=10)
    if transform is None or inliers is None or not np.all(np.isfinite(transform)):
        return _rejected(candidate, 'ransac_failed', counts)
    mask = inliers.reshape(-1).astype(bool)
    for _ in range(2):
        if int(mask.sum()) < config.minimum_mutual_matches:
            return _rejected(candidate, 'insufficient_ransac_inliers', counts)
        transform = _fit_rigid(second_wall[mask], first_wall[mask])
        predicted = cv2.transform(second_wall[None, :, :], transform)[0]
        mask = np.linalg.norm(predicted - first_wall, axis=1) <= config.ransac_threshold_m
    inlier_count = int(mask.sum())
    if inlier_count < config.minimum_mutual_matches:
        return _rejected(candidate, 'insufficient_ransac_inliers', counts)
    transform = _fit_rigid(second_wall[mask], first_wall[mask])
    predicted = cv2.transform(second_wall[None, :, :], transform)[0]
    residuals = np.linalg.norm(predicted[mask] - first_wall[mask], axis=1)
    overlap_center = np.asarray(candidate.intersection_xy_m, np.float64).mean(axis=0)
    corrected_center = transform[:, :2] @ overlap_center + transform[:, 2]
    center_correction = float(np.linalg.norm(corrected_center - overlap_center))
    return LocalMatch(
        candidate.first, candidate.second, 'accepted', '', *counts, inlier_count,
        tuple(float(value) for value in transform.reshape(-1)),
        center_correction,
        float(np.median(residuals)), float(np.percentile(residuals, 95.0)))
