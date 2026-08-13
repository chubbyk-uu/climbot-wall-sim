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
    frame = default_frame()
    assert frame.position_from_world((0.0, 1.0, 0.0)) == pytest.approx(
        (1.0, 0.0, 0.0), abs=1e-12)
    assert frame.position_from_world((0.0, 0.0, 1.0)) == pytest.approx(
        (0.0, 1.0, 0.0), abs=1e-12)
    assert frame.position_from_world((1.0, 0.0, 0.0)) == pytest.approx(
        (0.0, 0.0, 1.0), abs=1e-12)


def test_matches_the_previous_hardcoded_permutation():
    """The robot spawn pose keeps the wall coordinates it had before."""
    frame = default_frame()
    world = (0.051, 0.0, 2.0)
    assert frame.position_from_world(world) == pytest.approx(
        (world[1], world[2], world[0]), abs=1e-12)


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
