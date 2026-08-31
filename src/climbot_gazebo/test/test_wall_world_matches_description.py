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
import xacro
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
        if element.attrib['name'] not in {
            'inspection_target', 'inspection_flat_field_target',
            'wall_textured', 'moonlight_cast_shadows'}
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


def _wall_visual_size(wall_textured):
    """Expand the world and read the box the camera would actually photograph."""
    document = xacro.process_file(
        str(PACKAGE_ROOT / 'worlds' / 'climbot_wall.sdf.xacro'),
        mappings={'wall_textured': wall_textured})
    root = ElementTree.fromstring(document.toxml())
    size = root.find(".//visual[@name='wall_visual']/geometry/box/size")
    assert size is not None, 'the world has no wall_visual box'
    return [float(value) for value in size.text.split()]


def test_the_plain_wall_face_sits_on_the_plane_the_wheels_ride_on():
    """
    Without texture blocks there is nothing to z-fight, so nothing to step back for.

    A recess left switched on unconditionally puts the visible surface 1 mm
    behind the collision face: the same class of error, in the opposite
    direction, as the texture blocks that once stood 1.25 mm proud of it and
    scaled every mosaic by +4566 ppm. Nothing measures this surface today,
    which is exactly why an unconditional offset would sit here unnoticed.

    The condition is the texture and only the texture. Asserting it against
    inspection_target instead is what let two whole acquisitions come back
    striped with z-fighting while this file stayed green.
    """
    _, simulation = _documents()
    thickness = float(simulation['wall']['thickness_m'])
    assert _wall_visual_size('false')[0] == pytest.approx(thickness)
    # With the blocks loaded the face has to step back or it fights every one
    # of them; 1 mm each side is what that costs.
    assert _wall_visual_size('true')[0] == pytest.approx(thickness - 0.002)


def test_a_textured_wall_recesses_no_matter_what_the_calibration_target_does():
    """The G1 target flag must not decide whether the photographed face z-fights."""
    document = xacro.process_file(
        str(PACKAGE_ROOT / 'worlds' / 'climbot_wall.sdf.xacro'),
        mappings={'wall_textured': 'true', 'inspection_target': 'false'})
    root = ElementTree.fromstring(document.toxml())
    size = root.find(".//visual[@name='wall_visual']/geometry/box/size")
    _, simulation = _documents()
    thickness = float(simulation['wall']['thickness_m'])
    assert float(size.text.split()[0]) == pytest.approx(thickness - 0.002)


def test_optional_inspection_target_uses_shared_camera_geometry():
    """Keep the acceptance target centred under the initial camera view."""
    description = Path(get_package_share_directory('climbot_description'))
    camera = yaml.safe_load(
        (description / 'config' / 'inspection_camera.yaml').read_text()
    )['inspection_camera']
    _, simulation = _documents()
    defaults = _world_defaults()
    assert defaults['target_lateral'] == pytest.approx(
        simulation['spawn']['lateral_m'] +
        camera['optical_mount']['center_xyz_m'][0])
    assert defaults['target_height'] == pytest.approx(
        simulation['spawn']['height_m'])
    assert defaults['target_length'] == pytest.approx(
        camera['footprint']['effective_length_m'])
    assert defaults['target_width'] == pytest.approx(
        camera['footprint']['effective_width_m'])


def test_camera_noise_is_large_enough_for_multi_frame_statistics():
    """Calibration frames need independent signal, not byte-identical copies."""
    _, simulation = _documents()
    standard_deviation = float(
        simulation['inspection_camera']['noise_stddev'])
    assert standard_deviation * 255.0 >= 0.75
    assert standard_deviation * 255.0 <= 2.0
    assert isinstance(simulation['inspection_camera']['noise_seed'], int)


def test_moonlight_is_one_explicit_shadow_casting_scene_light():
    """Moonlight remains calibrated scene input, not GUI UI or duplicate lamps."""
    _, simulation = _documents()
    fill = simulation['lighting']['moonlight']
    defaults = _world_defaults()
    assert defaults['moonlight_intensity'] == pytest.approx(
        float(fill['intensity']))
    for axis, value in zip('xyz', fill['direction_xyz']):
        assert defaults[f'moonlight_direction_{axis}'] == pytest.approx(
            float(value))
    for channel, value in zip('rgb', fill['diffuse_rgb']):
        assert defaults[f'moonlight_diffuse_{channel}'] == pytest.approx(
            float(value))
    world = (PACKAGE_ROOT / 'worlds' / 'climbot_wall.sdf.xacro').read_text()
    assert '<light type="directional" name="moonlight">' in world
    assert '<cast_shadows>$(arg moonlight_cast_shadows)</cast_shadows>' in world
    assert 'name="night_environment"' not in world
    assert 'name="operator_fill"' not in world


def test_ground_is_below_the_wall_contact_plane():
    """The observation ground must not introduce a second wall contact."""
    world = (PACKAGE_ROOT / 'worlds' / 'climbot_wall.sdf.xacro').read_text()
    assert '<model name="ground">' in world
    assert '<pose>0 0 -0.02 0 0 0</pose>' in world
    assert '<plane><normal>0 0 1</normal><size>30 30</size></plane>' in world


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
