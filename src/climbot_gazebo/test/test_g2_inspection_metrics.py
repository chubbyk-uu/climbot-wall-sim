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

"""Keep the reported exposure-time P95 definition deterministic."""

import importlib.util
from pathlib import Path

import pytest


def _evaluator_module():
    path = Path(__file__).resolve().parents[1] / 'scripts' / 'evaluate_g2_inspection.py'
    spec = importlib.util.spec_from_file_location('g2_inspection_evaluator', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_p95_uses_nearest_rank_so_small_capture_runs_are_not_interpolated():
    percentile = _evaluator_module().nearest_rank_percentile
    assert percentile([0.001, 0.002, 0.003, 0.004], 0.95) == pytest.approx(0.004)
    assert percentile([0.001, 0.002, 0.003, 0.004, 0.005], 0.80) == pytest.approx(0.004)


def test_percentile_rejects_invalid_fraction_and_empty_sample_set_is_explicit():
    percentile = _evaluator_module().nearest_rank_percentile
    assert percentile([], 0.95) is None
    with pytest.raises(ValueError):
        percentile([0.001], 0.0)


def test_exposure_truth_interpolates_full_attitude_across_pi():
    from types import SimpleNamespace
    from scipy.spatial.transform import Rotation
    import numpy as np

    evaluator = _evaluator_module().G2InspectionEvaluator
    first = Rotation.from_euler('xyz', [0.01, -0.02, np.pi - 0.01]).as_quat()
    second = Rotation.from_euler('xyz', [0.01, -0.02, -np.pi + 0.01]).as_quat()
    fake = SimpleNamespace(truth=[(100, (1, 2, 0.05), first),
                                  (200, (3, 4, 0.05), second)])
    position, rotation = evaluator._truth_at(fake, 150)
    np.testing.assert_allclose(position, [2, 3, 0.05])
    expected = Rotation.from_euler('xyz', [0.01, -0.02, np.pi]).as_matrix()
    np.testing.assert_allclose(rotation, expected, atol=1e-12)
    assert evaluator._truth_at(fake, 201) is None


def test_exposure_score_uses_optical_footprint_instead_of_tilted_camera_center():
    from types import SimpleNamespace as NS
    from scipy.spatial.transform import Rotation
    import numpy as np

    evaluator = _evaluator_module().G2InspectionEvaluator
    pitch = 0.02
    rotation = Rotation.from_euler('y', pitch).as_matrix()
    # A pitch about the drive axle moves the camera centre by millimetres,
    # while its optical ray lands at x = mount_x / cos(pitch).
    x = 1 + 0.34 / np.cos(pitch)
    capture = NS(header=NS(stamp=NS(sec=0, nanosec=150)),
                 camera_pose=NS(pose=NS(position=NS(x=x, y=2, z=0.275))),
                 wall_heading_rad=0.0)
    fake = NS(pending_truth=[capture], _truth_at=lambda stamp: ((1, 2, 0.05), rotation),
              camera_offset=(0.34, 0, 0.275), optical_axis_body=np.array([0, 0, -1]),
              surface_offset=0.05, surface=None, legacy_planar_errors=[],
              position_errors=[], position_error_components=[], heading_errors=[])
    evaluator._resolve_truth(fake)
    assert fake.position_errors[0] == pytest.approx(0, abs=1e-12)
    assert fake.legacy_planar_errors[0] > 5e-5
    assert not fake.pending_truth
