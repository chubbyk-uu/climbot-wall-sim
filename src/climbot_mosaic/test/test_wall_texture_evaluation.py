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

"""Truth scoring must detect coherent false correspondences accepted by RANSAC."""

from pathlib import Path
import sys

from climbot_mosaic import matching
from climbot_mosaic.candidates import OverlapCandidate
from climbot_mosaic.matching import FeatureSet
from climbot_mosaic.mosaic_inputs import FrameKey
from climbot_mosaic.projection import FrameProjection
import numpy as np
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools'))
from evaluate_wall_texture_sift import score_pair  # noqa: E402,I100
from evaluate_wall_texture_sift import evaluate_actual_overlaps  # noqa: E402,I100


@pytest.mark.parametrize('true_shift,expected_wrong', [(0., 0), (2., 5)])
def test_more_ransac_inliers_does_not_make_a_false_wall_location_correct(
        true_shift, expected_wrong):
    a, b = FrameKey('probe', 0), FrameKey('probe', 1)
    points = np.array([[10, 10], [30, 10], [10, 30], [30, 30], [20, 20]], np.float32)
    desc = np.eye(5, 128, dtype=np.float32)
    features = {a: FeatureSet(a, points, desc), b: FeatureSet(b, points, desc)}
    footprint = ((0., 0.), (.5, 0.), (.5, .5), (0., .5))
    homography = (.01, 0., 0., 0., .01, 0., 0., 0., 1.)
    priors = {key: FrameProjection(key, 'a' * 64, homography, footprint, (0, 0), .275)
              for key in (a, b)}
    shifted = (.01, 0., true_shift, 0., .01, 0., 0., 0., 1.)
    truth = {**priors, b: FrameProjection(b, 'b' * 64, shifted, footprint, (0, 0), .275)}
    original = matching._fit_rigid
    result = score_pair(OverlapCandidate(a, b, .25, 1., footprint), features, priors, truth)
    assert matching._fit_rigid is original
    assert result['match']['status'] == 'accepted'
    assert result['truth']['inlier_count'] == 5
    assert result['truth']['wrong_inliers_over_1mm'] == expected_wrong
    assert result['truth']['pose_centre_error_mm'] == pytest.approx(true_shift * 1000, abs=1e-6)


def test_actual_overlap_protocol_does_not_force_far_duplicate_descriptors_to_match(tmp_path):
    points = np.array([[10, 10], [30, 10], [10, 30], [30, 30], [20, 20]], np.float32)
    desc = np.eye(5, 128, dtype=np.float32)
    features, projections, texture = {}, {}, {}
    for i in range(4):
        key = FrameKey('probe', i)
        x = 10. if i >= 2 else 0.
        footprint = ((x, 0.), (x + .5, 0.), (x + .5, .5), (x, .5))
        homography = (.01, 0., x, 0., .01, 0., 0., 0., 1.)
        projections[key] = FrameProjection(key, 'a' * 64, homography, footprint, (x, 0.), .275)
        features[key] = FeatureSet(key, points, desc)
        texture[key] = 1.
    capture = tmp_path / 'capture.json'
    capture.write_text('{}')
    data = {'frames': [{'view': {'index': i}} for i in range(4)], 'cleanup_exit_codes': [0, 0]}
    result = evaluate_actual_overlaps(capture, tmp_path / 'result.json', data, projections,
                                      projections, features, texture)
    assert result['summary']['actual_candidate_pairs'] == 2
    assert result['summary']['accepted_candidate_pairs'] == 2
    assert result['summary']['excluded_nonoverlap_pairs'] == 4
    assert result['summary']['wrong_inliers_over_1mm'] == 0
    assert 'far_false_acceptance' not in result['gates']
