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

"""Verify wall-slip trajectories record each truth message exactly once."""

import importlib.util
import os

import pytest

_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'scripts', 'calibrate_wall_slip.py')
_SPEC = importlib.util.spec_from_file_location('calibrate_wall_slip', _PATH)
calibration = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(calibration)


def test_first_truth_sample_is_recorded():
    assert calibration.is_new_truth_sample(1_000_000, None)


def test_repeated_truth_timestamp_is_not_recorded_again():
    assert not calibration.is_new_truth_sample(1_000_000, 1_000_000)


def test_next_truth_timestamp_is_recorded():
    assert calibration.is_new_truth_sample(1_010_000, 1_000_000)


def test_reset_truth_timestamp_is_recorded():
    """A simulation reset must not suppress the first new-world sample."""
    assert calibration.is_new_truth_sample(0, 1_000_000)


def test_coefficient_of_variation_is_zero_for_identical_runs():
    assert calibration.coefficient_of_variation(
        [0.09, 0.09, 0.09]) == pytest.approx(0.0, abs=1e-15)


def test_coefficient_of_variation_reports_relative_spread():
    result = calibration.coefficient_of_variation([0.08, 0.10])
    assert result == pytest.approx(1.0 / 9.0)


def test_coefficient_of_variation_rejects_undefined_inputs():
    with pytest.raises(ValueError):
        calibration.coefficient_of_variation([])
    with pytest.raises(ValueError):
        calibration.coefficient_of_variation([-1.0, 1.0])
