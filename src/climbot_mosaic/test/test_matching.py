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

"""Synthetic local feature matching tests."""

from climbot_mosaic.candidates import OverlapCandidate
from climbot_mosaic.match_pipeline import extract_all_features, MatchPipelineError
from climbot_mosaic.matching import FeatureSet, match_candidate, MatchConfig
from climbot_mosaic.mosaic_inputs import (
    CameraModel,
    FrameKey,
    MosaicInputs,
    ProcessedFrame,
    ProcessedRun,
)
from climbot_mosaic.projection import FrameProjection
import cv2
import numpy as np
import pytest


def test_mutual_matching_recovers_known_metric_translation():
    """Descriptor identity plus shifted projection produces the expected correction."""
    first_key, second_key = FrameKey('run', 0), FrameKey('run', 1)
    points = np.asarray(((10, 10), (30, 10), (10, 30), (30, 30), (20, 20)), np.float32)
    descriptors = np.zeros((5, 128), np.float32)
    for index in range(5):
        descriptors[index, index] = 1.0
    first = FeatureSet(first_key, points, descriptors)
    second = FeatureSet(second_key, points, descriptors)
    identity = (0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 1.0)
    shifted = (0.01, 0.0, 0.02, 0.0, 0.01, -0.01, 0.0, 0.0, 1.0)
    footprint = ((0.0, 0.0), (0.5, 0.0), (0.5, 0.5), (0.0, 0.5))
    first_projection = FrameProjection(first_key, 'a' * 64, identity, footprint, (0, 0), 0.3)
    second_projection = FrameProjection(second_key, 'b' * 64, shifted, footprint, (0, 0), 0.3)
    candidate = OverlapCandidate(first_key, second_key, 0.25, 1.0, footprint)

    result = match_candidate(candidate, first, second, first_projection, second_projection,
                             MatchConfig(ransac_threshold_m=0.001))

    assert result.status == 'accepted'
    transform = np.asarray(result.transform_second_to_first_xy).reshape(2, 3)
    np.testing.assert_allclose(transform[:, 2], (-0.02, 0.01), atol=1e-6)
    assert result.overlap_center_correction_m == pytest.approx(
        np.hypot(0.02, 0.01), abs=1e-6)


def test_candidate_with_no_features_is_explicitly_rejected():
    """An empty overlap is represented as evidence rather than silently omitted."""
    key_a, key_b = FrameKey('run', 0), FrameKey('run', 1)
    empty = np.empty((0, 2), np.float32)
    descriptors = np.empty((0, 128), np.float32)
    features_a = FeatureSet(key_a, empty, descriptors)
    features_b = FeatureSet(key_b, empty, descriptors)
    footprint = ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0))
    homography = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    projection_a = FrameProjection(key_a, 'a' * 64, homography, footprint, (0, 0), 0.3)
    projection_b = FrameProjection(key_b, 'b' * 64, homography, footprint, (0, 0), 0.3)

    result = match_candidate(
        OverlapCandidate(key_a, key_b, 1.0, 1.0, footprint),
        features_a, features_b, projection_a, projection_b)

    assert result.status == 'rejected'
    assert result.reason == 'insufficient_mutual_matches'


def test_feature_cache_is_reused_and_corruption_is_not_silently_recomputed(tmp_path):
    """Content-addressed features are reusable, while malformed entries stop the run."""
    image = np.zeros((64, 64), np.uint8)
    cv2.circle(image, (32, 32), 12, 255, 2)
    frames = []
    for index in range(2):
        path = tmp_path / f'{index}.png'
        assert cv2.imwrite(str(path), image)
        frames.append(ProcessedFrame(
            FrameKey('run', index), path, f'{index + 1}' * 64,
            tmp_path / f'{index}.json', {}))
    camera = CameraModel(64, 64, (1.0,) * 9, (0.0,) * 3, (0.0,) * 3, 'camera')
    run = ProcessedRun('run', 'a' * 64, tmp_path / 'input', camera, tuple(frames))
    inputs = MosaicInputs(camera, (run,), tuple(frames))
    work_dir = tmp_path / 'work'

    _, first = extract_all_features(inputs, work_dir, jobs=1)
    _, second = extract_all_features(inputs, work_dir, jobs=1)

    assert first['cache_misses'] == 2
    assert second['cache_hits'] == 2
    cache_file = sorted((work_dir / 'features').glob('*.npz'))[0]
    cache_file.write_bytes(b'not an npz')
    with pytest.raises(MatchPipelineError, match='corrupt'):
        extract_all_features(inputs, work_dir, jobs=1)
