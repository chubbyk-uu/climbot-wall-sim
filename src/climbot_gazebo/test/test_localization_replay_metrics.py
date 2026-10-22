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

"""Keep replay covariance evaluation correlated and interpolation bounded."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest


def _metrics():
    path = Path(__file__).resolve().parents[3] / 'tools/score_localization_replay.py'
    spec = importlib.util.spec_from_file_location('replay_metrics', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_nees_keeps_cross_axis_covariance():
    error = np.array([[0.001, 0.001], [-0.001, -0.001]])
    covariance = np.repeat(np.array([[[1e-6, 0.5e-6], [0.5e-6, 1e-6]]]), 2, axis=0)
    report = _metrics().error_summary(error, covariance)
    assert report['nees_xy_mean'] == pytest.approx(4 / 3)
    assert report['max_mm'] == pytest.approx(np.sqrt(2))
    assert report['fraction_inside_95pct_xy_ellipse'] == 1


def test_interpolation_refuses_to_extrapolate():
    module = _metrics()
    rows = np.array([[1, 10], [2, 20]])
    np.testing.assert_allclose(module.interpolate(rows, np.array([1.25, 1.75])), [[12.5], [17.5]])
    with pytest.raises(ValueError, match='extrapolate'):
        module.interpolate(rows, np.array([0.99, 1.5]))


def test_republished_stamp_keeps_the_last_estimate():
    rows = np.array([[2, 20], [1, 10], [1, 11], [3, 30]])
    np.testing.assert_array_equal(_metrics().ordered_unique(rows), [[1, 11], [2, 20], [3, 30]])


def test_nonpositive_covariance_is_an_error():
    with pytest.raises(ValueError, match='nonpositive'):
        _metrics().error_summary(np.zeros((1, 2)), np.zeros((1, 2, 2)))


def _tool(name):
    path = Path(__file__).resolve().parents[3] / 'tools' / (name + '.py')
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_sensor_audit_does_not_hide_bias_in_centered_variance():
    module = _tool('analyze_localization_noise')
    error = np.array([[0.001], [0.003], [0.005]])
    report = module.residual_summary(error, np.full_like(error, 1e-6))
    assert report['bias'] == pytest.approx([0.003])
    assert report['std'] == pytest.approx([np.sqrt(8 / 3) * 0.001])
    assert report['mean_square_over_reported_variance'] == pytest.approx([35 / 3])


def test_motion_audit_uniform_velocity_has_no_process_residual():
    module = _tool('analyze_localization_noise')
    times = np.arange(0, 3, 0.01)
    rows = np.zeros((len(times), 8))
    rows[:, 0] = times
    rows[:, 1] = 0.3 * times
    rows[:, 2] = 0.04 * times  # Preserve actual nonzero lateral motion.
    rows[:, 7] = 1
    report = module.motion_residuals(rows, 0.08)
    assert np.max(report['all']['effective_second_moment_per_second']) < 1e-18


def test_replay_camera_pose_interpolates_yaw_wrap_and_lever_covariance():
    from scipy.spatial.transform import Rotation
    module = _tool('derive_replay_processed_run')
    rows = np.zeros((2, 44))
    rows[:, 0] = [1, 1.02]
    rows[:, 4:8] = Rotation.from_euler('z', [179, -179], degrees=True).as_quat()
    covariance = np.eye(6) * 1e-6
    covariance[5, 5] = 1e-4
    rows[:, 8:44] = covariance.ravel()
    pose, heading = module.camera_pose_at(rows, 1.01, (0.34, 0, 0.275), (0, 0, 0))
    assert abs(heading) == pytest.approx(np.pi)
    assert pose['pose']['position']['x'] == pytest.approx(-0.34)
    transformed = np.array(pose['covariance']).reshape(6, 6)
    assert transformed[1, 1] == pytest.approx(1e-6 + 0.34 ** 2 * 1e-4)
    assert transformed[2, 2] == 0  # Frozen planar standoff, as in capture node.
    with pytest.raises(ValueError, match='outside'):
        module.camera_pose_at(rows, 0.99, (0.34, 0, 0.275), (0, 0, 0))
