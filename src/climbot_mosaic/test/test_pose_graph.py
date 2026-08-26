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

"""Synthetic recovery and rejection tests for the sparse SE(2) graph."""

from climbot_mosaic.mosaic_inputs import (
    CameraModel,
    FrameKey,
    MosaicInputs,
    ProcessedFrame,
    ProcessedRun,
)
from climbot_mosaic.pose_graph import optimize_pose_graph, PoseGraphError, VisualEdge
import numpy as np
import pytest


def _inputs(tmp_path, count=3):
    camera = CameraModel(10, 10, (1.0,) * 9, (0.0,) * 3, (0.0,) * 3, 'camera')
    frames = []
    for index in range(count):
        covariance = np.zeros((6, 6), np.float64)
        covariance[0, 0] = covariance[1, 1] = 1.0
        covariance[5, 5] = 1.0
        frames.append(ProcessedFrame(
            FrameKey('run', index), tmp_path / f'{index}.png', f'{index + 1}' * 64,
            tmp_path / f'{index}.json', {
                'wall_heading_rad': 0.0,
                'camera_pose': {'pose': {'position': {'x': float(index), 'y': 0.0, 'z': 0.3}},
                                'covariance': covariance.reshape(-1).tolist()},
            }))
    run = ProcessedRun('run', 'a' * 64, tmp_path, camera, tuple(frames))
    return MosaicInputs(camera, (run,), tuple(frames))


def _translation(first, second, tx):
    return VisualEdge(first, second, (1.0, 0.0, tx, 0.0, 1.0, 0.0),
                      (0.5 * (first + second), 0.0), 0.001, 0.2, 20)


def test_pose_graph_recovers_consistent_zero_mean_corrections(tmp_path):
    inputs = _inputs(tmp_path)
    result = optimize_pose_graph(inputs, (_translation(0, 1, 0.01),
                                          _translation(1, 2, 0.01)))
    np.testing.assert_allclose(result.corrections[:, 0], (-0.01, 0.0, 0.01), atol=2e-4)
    np.testing.assert_allclose(result.corrections[:, 1:], 0.0, atol=2e-4)
    assert result.report['connected_component_sizes'] == [3]
    assert result.report['final_edge_error_m']['max'] < 1e-4


def test_pose_graph_rejects_invalid_recheck_floor(tmp_path):
    with pytest.raises(PoseGraphError, match='recheck_floor'):
        optimize_pose_graph(_inputs(tmp_path), (_translation(0, 1, 0.0),), float('nan'))


def test_pose_graph_is_deterministic(tmp_path):
    inputs = _inputs(tmp_path)
    edges = (_translation(0, 1, 0.004), _translation(1, 2, -0.003))
    first = optimize_pose_graph(inputs, edges)
    second = optimize_pose_graph(inputs, edges)
    np.testing.assert_array_equal(first.corrections, second.corrections)
    assert first.report == second.report
