"""Hold the wall world to the description it is supposed to be rendering."""

# Two failure modes motivate this file, and neither shows up as an error.
#
# The world xacro carries a default for every value the launch injects, so it
# can be rendered on its own. A default left behind after the YAML moved
# renders a different wall than the one every planner and evaluator believes
# in - the suction_force default was still 220 long after the authoritative
# value became 400.
#
# And simulation.yaml places the wall body in the Gazebo world with a centre
# that is not free: it has to put the same surface where
# climbot_description/config/wall.yaml says the work frame's origin is. A stale
# centre_z lifts the wall off the ground, and nothing else reports it.

from pathlib import Path
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
XACRO_NAMESPACE = 'http://www.ros.org/wiki/xacro'


def _documents():
    description = Path(get_package_share_directory('climbot_description'))
    wall = yaml.safe_load(
        (description / 'config' / 'wall.yaml').read_text())['wall']
    simulation = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'simulation.yaml').read_text())['simulation']
    return wall, simulation


def _world_defaults():
    root = ElementTree.parse(
        PACKAGE_ROOT / 'worlds' / 'climbot_wall.sdf.xacro').getroot()
    return {
        element.attrib['name']: float(element.attrib['default'])
        for element in root.findall(f'{{{XACRO_NAMESPACE}}}arg')
    }


def test_world_defaults_match_the_values_the_launch_injects():
    """A standalone render must produce the wall the launch would."""
    wall, simulation = _documents()
    defaults = _world_defaults()
    simulated = simulation['wall']
    spawn = simulation['spawn']
    expected = {
        'centre_x': simulated['centre_xyz'][0],
        'centre_y': simulated['centre_xyz'][1],
        'centre_z': simulated['centre_xyz'][2],
        'thickness': simulated['thickness_m'],
        'width': wall['surface']['width_m'],
        'height': wall['surface']['height_m'],
        'mu': simulated['mu'],
        'grid_spacing': wall['reference_grid']['spacing_m'],
        'suction_force': simulation['suction']['force_n'],
        'spawn_gap': spawn['surface_gap_m'],
        'spawn_lateral': spawn['lateral_m'],
        'spawn_height': spawn['height_m'],
        'spawn_roll': wall['origin_rpy'][0],
        'spawn_pitch': wall['origin_rpy'][1],
        'spawn_yaw': wall['origin_rpy'][2],
    }
    for name, value in expected.items():
        assert defaults[name] == pytest.approx(float(value)), name


def test_simulated_wall_body_carries_the_described_surface():
    """The Gazebo body and the work frame must describe one wall, not two."""
    wall, simulation = _documents()
    origin = wall['origin_xyz']
    surface = wall['surface']
    centre = simulation['wall']['centre_xyz']
    # Work +X is world +Y and work +Y is world +Z, and the origin is the
    # surface's lower-left corner, so the centre sits half a span along each.
    assert centre[1] == pytest.approx(
        origin[1] + 0.5 * float(surface['width_m']))
    assert centre[2] == pytest.approx(
        origin[2] + 0.5 * float(surface['height_m']))


def test_the_wall_stands_on_the_ground_plane():
    """Work y = 0 is the foot of the wall, which every region assumes."""
    wall, _ = _documents()
    assert wall['origin_xyz'][2] == pytest.approx(0.0)


def test_the_work_frame_origin_is_the_lower_left_corner():
    """No region, result or log should ever carry a negative wall coordinate."""
    wall, _ = _documents()
    # The wall is centred on world Y = 0, so its left edge - work x = 0 - is
    # half a width to the world's -Y side.
    assert wall['origin_xyz'][1] == pytest.approx(
        -0.5 * float(wall['surface']['width_m']))
