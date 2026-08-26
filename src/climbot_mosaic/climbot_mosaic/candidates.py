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

"""Stable spatial-overlap candidates for planar wall-image matching."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

from climbot_mosaic.mosaic_inputs import FrameKey
from climbot_mosaic.projection import FrameProjection
import numpy as np


class CandidateError(ValueError):
    """Projected image footprints cannot define a valid candidate graph."""


@dataclass(frozen=True)
class OverlapCandidate:
    """One unordered pair whose initial wall-plane footprints overlap in area."""

    first: FrameKey
    second: FrameKey
    overlap_area_m2: float
    smaller_footprint_overlap_ratio: float
    intersection_xy_m: tuple[tuple[float, float], ...]


def _polygon_array(points: Iterable[tuple[float, float]]) -> np.ndarray:
    values = np.asarray(tuple(points), dtype=np.float64)
    if values.ndim != 2 or values.shape[0] < 3 or values.shape[1] != 2:
        raise CandidateError('a footprint must contain at least three XY points.')
    if not np.all(np.isfinite(values)):
        raise CandidateError('a footprint contains non-finite coordinates.')
    return values


def polygon_area_m2(points: Iterable[tuple[float, float]]) -> float:
    """Return the non-negative area of one simple, ordered planar polygon."""
    polygon = _polygon_array(points)
    signed = 0.5 * float(np.dot(polygon[:, 0], np.roll(polygon[:, 1], -1)) - np.dot(
        polygon[:, 1], np.roll(polygon[:, 0], -1)))
    area = abs(signed)
    if not math.isfinite(area) or area <= 1e-12:
        raise CandidateError('a footprint must have non-zero finite area.')
    return area


def _signed_area(polygon: np.ndarray) -> float:
    return 0.5 * float(np.dot(polygon[:, 0], np.roll(polygon[:, 1], -1)) - np.dot(
        polygon[:, 1], np.roll(polygon[:, 0], -1)))


def _cross(left: np.ndarray, right: np.ndarray) -> float:
    return float(left[0] * right[1] - left[1] * right[0])


def _line_intersection(start: np.ndarray, end: np.ndarray,
                       clip_start: np.ndarray, clip_end: np.ndarray) -> np.ndarray:
    clip_edge = clip_end - clip_start
    start_side = _cross(clip_edge, start - clip_start)
    end_side = _cross(clip_edge, end - clip_start)
    denominator = start_side - end_side
    if abs(denominator) < 1e-15:
        raise CandidateError('polygon clipping encountered coincident crossing lines.')
    result = start + (start_side / denominator) * (end - start)
    if not np.all(np.isfinite(result)):
        raise CandidateError('polygon clipping produced a non-finite point.')
    return result


def convex_intersection(first: Iterable[tuple[float, float]],
                        second: Iterable[tuple[float, float]]) -> tuple[tuple[float, float], ...]:
    """Clip one convex footprint by another, returning an ordered intersection polygon."""
    subject = _polygon_array(first)
    clip = _polygon_array(second)
    orientation = 1.0 if _signed_area(clip) > 0.0 else -1.0
    output = subject
    for index, clip_start in enumerate(clip):
        clip_end = clip[(index + 1) % len(clip)]
        if len(output) == 0:
            break
        input_polygon = output
        points: list[np.ndarray] = []
        previous = input_polygon[-1]
        previous_inside = (
            orientation * _cross(clip_end - clip_start, previous - clip_start) >= -1e-12)
        for current in input_polygon:
            current_inside = (
                orientation * _cross(clip_end - clip_start, current - clip_start) >= -1e-12)
            if current_inside != previous_inside:
                points.append(_line_intersection(previous, current, clip_start, clip_end))
            if current_inside:
                points.append(current)
            previous, previous_inside = current, current_inside
        output = np.asarray(points, dtype=np.float64)
    return tuple((float(point[0]), float(point[1])) for point in output)


def build_overlap_candidates(projections: Iterable[FrameProjection],
                             min_overlap_area_m2: float = 0.0) -> tuple[OverlapCandidate, ...]:
    """Use sweep-line bounding boxes then exact convex clipping in stable key order."""
    minimum = float(min_overlap_area_m2)
    if not math.isfinite(minimum) or minimum < 0.0:
        raise CandidateError('min_overlap_area_m2 must be finite and non-negative.')
    entries = []
    for projection in projections:
        polygon = _polygon_array(projection.footprint_xy_m)
        entries.append((
            float(polygon[:, 0].min()), float(polygon[:, 0].max()),
            float(polygon[:, 1].min()), float(polygon[:, 1].max()),
            projection, polygon_area_m2(projection.footprint_xy_m)))
    entries.sort(key=lambda entry: (entry[0], entry[4].key))
    active: list[tuple[float, float, float, float, FrameProjection, float]] = []
    candidates: list[OverlapCandidate] = []
    for entry in entries:
        min_x, max_x, min_y, max_y, projection, area = entry
        active = [previous for previous in active if previous[1] > min_x]
        for previous in active:
            if previous[3] <= min_y or max_y <= previous[2]:
                continue
            intersection = convex_intersection(
                previous[4].footprint_xy_m, projection.footprint_xy_m)
            if len(intersection) < 3:
                continue
            intersection_area = polygon_area_m2(intersection)
            if intersection_area <= minimum:
                continue
            first, second = sorted((previous[4].key, projection.key))
            candidates.append(OverlapCandidate(
                first, second, intersection_area,
                intersection_area / min(previous[5], area), intersection))
        active.append(entry)
    candidates.sort(key=lambda candidate: (candidate.first, candidate.second))
    return tuple(candidates)
