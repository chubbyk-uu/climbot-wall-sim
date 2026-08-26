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

"""Bounded parallel feature extraction, content-addressed cache and local matches."""

from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import time
from typing import Iterable
from uuid import uuid4

from climbot_mosaic.candidates import OverlapCandidate
from climbot_mosaic.matching import (
    extract_sift,
    FeatureSet,
    LocalMatch,
    match_candidate,
    MatchConfig,
)
from climbot_mosaic.mosaic_inputs import FrameKey, MosaicInputs, ProcessedFrame
from climbot_mosaic.projection import FrameProjection
import cv2
import numpy as np


class MatchPipelineError(RuntimeError):
    """Feature cache or bounded pipeline execution is incomplete or corrupt."""


def automatic_jobs(requested: int | None) -> int:
    """Resolve an explicit positive worker count or a conservative auto count."""
    if requested is not None:
        if isinstance(requested, bool) or requested <= 0:
            raise MatchPipelineError('jobs must be a positive integer or auto.')
        return requested
    return max(1, min(os.cpu_count() or 1, 8))


def _cache_key(frame: ProcessedFrame, use_clahe: bool) -> str:
    document = {
        'feature_format_version': 1,
        'image_sha256': frame.image_sha256,
        'opencv': cv2.__version__,
        'sift': 'opencv-default',
        'use_clahe': use_clahe,
    }
    return hashlib.sha256(json.dumps(
        document, allow_nan=False, separators=(',', ':'), sort_keys=True
    ).encode('utf-8')).hexdigest()


def _read_cache(path: Path, key: FrameKey) -> FeatureSet:
    try:
        with np.load(path, allow_pickle=False) as archive:
            points = np.asarray(archive['points_px'], dtype=np.float32)
            descriptors = np.asarray(archive['descriptors'], dtype=np.float32)
    except (OSError, ValueError, KeyError) as error:
        raise MatchPipelineError(f'feature cache is corrupt: {path.name}: {error}') from error
    if (points.ndim != 2 or points.shape[1:] != (2,) or descriptors.ndim != 2 or
            descriptors.shape[1:] != (128,) or len(points) != len(descriptors) or
            not np.all(np.isfinite(points)) or not np.all(np.isfinite(descriptors))):
        raise MatchPipelineError(f'feature cache has invalid arrays: {path.name}.')
    return FeatureSet(key, points, descriptors)


def _extract_task(task: tuple[FrameKey, str, bool]) -> FeatureSet:
    key, path, use_clahe = task
    return extract_sift(key, Path(path), use_clahe)


def extract_all_features(inputs: MosaicInputs, work_dir: Path, jobs: int | None = None,
                         use_clahe: bool = False) -> tuple[dict[FrameKey, FeatureSet], dict]:
    """Load valid content-addressed entries and compute cache misses in stable order."""
    if not work_dir.is_absolute():
        raise MatchPipelineError('work_dir must be an absolute path.')
    feature_dir = work_dir / 'features'
    feature_dir.mkdir(parents=True, exist_ok=True)
    resolved_jobs = automatic_jobs(jobs)
    features: dict[FrameKey, FeatureSet] = {}
    misses: list[ProcessedFrame] = []
    paths: dict[FrameKey, Path] = {}
    for frame in inputs.frames:
        path = feature_dir / f'{_cache_key(frame, use_clahe)}.npz'
        paths[frame.key] = path
        if path.exists():
            features[frame.key] = _read_cache(path, frame.key)
        else:
            misses.append(frame)
    started = time.perf_counter()
    tasks = [(frame.key, str(frame.image_path), use_clahe) for frame in misses]
    if tasks:
        if resolved_jobs == 1:
            extracted = map(_extract_task, tasks)
        else:
            executor = ProcessPoolExecutor(max_workers=resolved_jobs)
            extracted = executor.map(_extract_task, tasks, chunksize=1)
        try:
            for frame, feature in zip(misses, extracted, strict=True):
                temporary = paths[frame.key].with_name(
                    f'.{paths[frame.key].name}.tmp-{uuid4().hex}.npz')
                np.savez_compressed(
                    temporary, points_px=feature.points_px, descriptors=feature.descriptors)
                temporary.replace(paths[frame.key])
                features[frame.key] = feature
        finally:
            if resolved_jobs != 1:
                executor.shutdown(wait=True, cancel_futures=True)
    if set(features) != {frame.key for frame in inputs.frames}:
        raise MatchPipelineError('feature extraction did not return every input frame.')
    return features, {
        'jobs': resolved_jobs,
        'cache_hits': len(inputs.frames) - len(misses),
        'cache_misses': len(misses),
        'elapsed_s': time.perf_counter() - started,
        'use_clahe': use_clahe,
        'opencv': cv2.__version__,
        'sift': 'opencv-default',
    }


def match_all_candidates(candidates: Iterable[OverlapCandidate], inputs: MosaicInputs,
                         projections: Iterable[FrameProjection],
                         features: dict[FrameKey, FeatureSet],
                         config: MatchConfig = MatchConfig()) -> tuple[
                             tuple[LocalMatch, ...], dict]:
    """Match every candidate in stable order and account for every result."""
    frame_map = {frame.key: frame for frame in inputs.frames}
    projection_map = {projection.key: projection for projection in projections}
    values = tuple(sorted(candidates, key=lambda item: (item.first, item.second)))
    started = time.perf_counter()
    results = tuple(match_candidate(
        candidate, features[candidate.first], features[candidate.second],
        projection_map[candidate.first], projection_map[candidate.second], config)
        for candidate in values)
    accepted = sum(result.status == 'accepted' for result in results)
    if len(results) != len(values) or any(key not in frame_map for key in features):
        raise MatchPipelineError('local matching did not account for every candidate or frame.')
    reasons: dict[str, int] = {}
    for result in results:
        if result.status != 'accepted':
            reasons[result.reason] = reasons.get(result.reason, 0) + 1
    return results, {
        'candidate_count': len(values),
        'accepted_count': accepted,
        'rejected_count': len(values) - accepted,
        'rejection_reasons': dict(sorted(reasons.items())),
        'elapsed_s': time.perf_counter() - started,
        'config': asdict(config),
    }
