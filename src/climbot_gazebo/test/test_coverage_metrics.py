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

"""Verify actual two-dimensional inspection-footprint coverage metrics."""

import math

from climbot_gazebo.coverage_metrics import footprint_coverage
import pytest


def test_dense_horizontal_scan_covers_the_requested_rectangle():
    polygon = [(0.0, -0.25), (1.0, -0.25), (1.0, 0.25), (0.0, 0.25)]
    result = footprint_coverage(
        polygon, [[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]],
        width=0.5, length=0.1, resolution=0.01)
    assert result['ratio'] == pytest.approx(1.0)
    assert result['missed_ratio'] == pytest.approx(0.0)


def test_metric_detects_a_narrower_actual_footprint():
    polygon = [(0.0, -0.25), (1.0, -0.25), (1.0, 0.25), (0.0, 0.25)]
    result = footprint_coverage(
        polygon, [[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]],
        width=0.4, length=0.1, resolution=0.01)
    assert result['ratio'] == pytest.approx(0.8)


def test_separate_scan_paths_do_not_cover_the_transition_between_them():
    polygon = [(0.0, -0.1), (1.0, -0.1), (1.0, 1.1), (0.0, 1.1)]
    paths = [
        [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)],
        [(1.0, 1.0, math.pi), (0.0, 1.0, math.pi)],
    ]
    result = footprint_coverage(
        polygon, paths, width=0.2, length=0.1, resolution=0.01)
    assert result['ratio'] == pytest.approx(1.0 / 3.0)


def test_front_camera_offset_moves_the_evaluated_footprint():
    polygon = [(0.3, -0.25), (1.3, -0.25), (1.3, 0.25), (0.3, 0.25)]
    result = footprint_coverage(
        polygon, [[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]],
        width=0.5, length=0.1, resolution=0.01, forward_offset=0.3)
    assert result['ratio'] == pytest.approx(1.0)


@pytest.mark.parametrize(
    'width,length,resolution',
    [(0.0, 0.1, 0.01), (0.5, -0.1, 0.01), (0.5, 0.1, 0.0)])
def test_metric_rejects_invalid_dimensions(width, length, resolution):
    polygon = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    with pytest.raises(ValueError):
        footprint_coverage(
            polygon, [], width=width, length=length, resolution=resolution)
