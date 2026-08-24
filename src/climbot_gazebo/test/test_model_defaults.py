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

"""Verify standalone SDF defaults match authoritative YAML settings."""

from pathlib import Path
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
XACRO_NAMESPACE = 'http://www.ros.org/wiki/xacro'


def test_sdf_defaults_match_shared_and_simulation_yaml():
    """Keep independent model rendering consistent with standard launch."""
    description = Path(get_package_share_directory('climbot_description'))
    robot = yaml.safe_load(
        (description / 'config' / 'robot.yaml').read_text())['robot']
    simulation = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'simulation.yaml').read_text())['simulation']
    root = ElementTree.parse(
        PACKAGE_ROOT / 'models' / 'climbot' / 'model.sdf.xacro').getroot()
    arguments = {
        element.attrib['name']: float(element.attrib['default'])
        for element in root.findall(f'{{{XACRO_NAMESPACE}}}arg')
    }
    assert arguments['base_x'] == pytest.approx(robot['base']['centre_xyz'][0])
    assert arguments['base_z'] == pytest.approx(robot['base']['centre_xyz'][2])
    assert arguments['wheel_axle_x'] == pytest.approx(
        robot['drive_wheel']['axle_x_m'])
    assert arguments['caster_x'] == pytest.approx(robot['caster']['centre_x_m'])
    camera = yaml.safe_load(
        (description / 'config' / 'inspection_camera.yaml').read_text()
    )['inspection_camera']
    body = robot['inspection_payload']['camera_body']
    bracket = robot['inspection_payload']['bracket']
    assert arguments['camera_body_mass'] == pytest.approx(body['mass_kg'])
    assert arguments['camera_body_z'] == pytest.approx(body['centre_xyz_m'][2])
    assert arguments['camera_bracket_mass'] == pytest.approx(bracket['mass_kg'])
    assert arguments['camera_bracket_x'] == pytest.approx(
        bracket['centre_xyz_m'][0])
    assert arguments['camera_optical_x'] == pytest.approx(
        camera['optical_mount']['center_xyz_m'][0])
    assert arguments['camera_optical_z'] == pytest.approx(
        camera['optical_mount']['center_xyz_m'][2])
    assert arguments['contact_rate'] == pytest.approx(
        simulation['contact']['update_rate_hz'])


def test_truth_odometry_is_explicitly_labelled_as_world_frame():
    """Keep Gazebo world truth distinct from wall-frame odometry."""
    root = ElementTree.parse(
        PACKAGE_ROOT / 'models' / 'climbot' / 'model.sdf.xacro').getroot()
    plugins = {
        plugin.attrib['name']: plugin
        for plugin in root.findall('model/plugin')
    }
    assert plugins['gz::sim::systems::DiffDrive'].findtext('frame_id') == 'odom'
    assert (plugins['gz::sim::systems::OdometryPublisher'].findtext('odom_frame')
            == 'world')
