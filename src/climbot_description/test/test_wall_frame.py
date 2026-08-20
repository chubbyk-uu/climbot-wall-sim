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

"""Verify the wall work frame reproduces the wall axis convention exactly."""

import math
import os

from climbot_description.geometry import quaternion_from_rpy
from climbot_description.wall_frame import WallFrame
import pytest

CONFIG = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'config', 'wall.yaml')


def default_frame():
    """Return the wall frame described by the package's wall.yaml."""
    return WallFrame.from_yaml(CONFIG)


def test_maps_world_axes_onto_wall_axes():
    """World +Y, +Z and +X become wall +X, +Y and +Z respectively."""
    # The axis convention is the rotation alone, so this takes the rotation the
    # description carries and drops its translation; where the origin sits is
    # the next two tests' subject.
    frame = WallFrame((0.0, 0.0, 0.0), default_frame().roll_pitch_yaw)
    assert frame.position_from_world((0.0, 1.0, 0.0)) == pytest.approx(
        (1.0, 0.0, 0.0), abs=1e-12)
    assert frame.position_from_world((0.0, 0.0, 1.0)) == pytest.approx(
        (0.0, 1.0, 0.0), abs=1e-12)
    assert frame.position_from_world((1.0, 0.0, 0.0)) == pytest.approx(
        (0.0, 0.0, 1.0), abs=1e-12)


def test_origin_is_the_wall_lower_left_corner():
    """The whole surface is the first quadrant, with no negative coordinate."""
    frame = default_frame()
    width = float(frame.surface['width_m'])
    height = float(frame.surface['height_m'])
    # The wall is centred on world Y = 0 and stands on world Z = 0, so these
    # two world points are its bottom-left and top-right corners.
    assert frame.position_from_world((0.0, -0.5 * width, 0.0)) == pytest.approx(
        (0.0, 0.0, 0.0), abs=1e-12)
    assert frame.position_from_world((0.0, 0.5 * width, height)) == pytest.approx(
        (width, height, 0.0), abs=1e-12)


def test_robot_spawns_at_the_middle_of_the_wall():
    """lateral_m is a world offset, so a centred spawn is x = width / 2."""
    frame = default_frame()
    width = float(frame.surface['width_m'])
    assert frame.position_from_world((0.051, 0.0, 2.0)) == pytest.approx(
        (0.5 * width, 2.0, 0.051), abs=1e-12)


def test_robot_spawn_orientation_is_identity_in_the_wall_frame():
    """This is what lets the IMU's initial identity attitude mean wall zero."""
    frame = default_frame()
    spawn = quaternion_from_rpy(math.pi / 2.0, 0.0, math.pi / 2.0)
    assert frame.orientation_from_world(spawn) == pytest.approx(
        (0.0, 0.0, 0.0, 1.0), abs=1e-12)


def test_translation_is_applied_before_rotation():
    """A wall origin offset shifts the measured wall coordinates."""
    frame = WallFrame((1.0, 2.0, 3.0), (math.pi / 2.0, 0.0, math.pi / 2.0))
    assert frame.position_from_world((1.0, 2.0, 3.0)) == pytest.approx(
        (0.0, 0.0, 0.0), abs=1e-12)


def test_rejects_incomplete_descriptions():
    """Malformed input fails loudly instead of silently mis-transforming."""
    with pytest.raises(ValueError):
        WallFrame((0.0, 0.0), (0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        WallFrame((0.0, 0.0, 0.0), (0.0, 0.0))
