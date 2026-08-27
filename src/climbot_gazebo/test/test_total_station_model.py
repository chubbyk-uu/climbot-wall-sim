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

"""Unit tests for the deterministic realistic total-station model."""

import random

from climbot_gazebo.total_station_model import (
    resolve_component_enabled,
    rotate_robot_residual_to_wall,
    timestamp_with_clock_error_ns,
)
import pytest


def test_precision_profile_leaves_both_realistic_components_off_by_default():
    assert not resolve_component_enabled('precision', 'auto')


def test_realistic_profile_enables_both_components_but_each_can_be_overridden():
    assert resolve_component_enabled('realistic', 'auto')
    assert not resolve_component_enabled('realistic', 'disabled')
    assert resolve_component_enabled('precision', 'enabled')


def test_profile_and_component_mode_are_validated():
    with pytest.raises(ValueError):
        resolve_component_enabled('unknown', 'auto')
    with pytest.raises(ValueError):
        resolve_component_enabled('precision', 'sometimes')


def test_robot_prism_residual_rotates_with_truth_yaw_and_reverses_projection():
    residual = (0.012, -0.006, 0.003)
    assert rotate_robot_residual_to_wall(residual, 0.0) == pytest.approx(
        (0.012, -0.006, 0.003))
    assert rotate_robot_residual_to_wall(
        residual, 1.5707963267948966) == pytest.approx((0.006, 0.012, 0.003))
    assert rotate_robot_residual_to_wall(
        residual, 3.141592653589793) == pytest.approx((-0.012, 0.006, 0.003))


def test_timestamp_error_is_deterministic_and_does_not_affect_delivery_clock():
    source = 12_000_000_000
    first = timestamp_with_clock_error_ns(source, 0.020, 0.002, random.Random(7))
    second = timestamp_with_clock_error_ns(source, 0.020, 0.002, random.Random(7))
    assert first == second
    assert first != source


def test_timestamp_is_clamped_only_when_negative_clock_error_crosses_zero():
    assert timestamp_with_clock_error_ns(
        1_000_000, -0.020, 0.0, random.Random(1)) == 0
