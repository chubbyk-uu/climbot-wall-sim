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

import math
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
        element.attrib['name']: element.attrib['default']
        for element in root.findall(f'{{{XACRO_NAMESPACE}}}arg')
    }
    assert float(arguments['base_x']) == pytest.approx(
        robot['base']['centre_xyz'][0])
    assert float(arguments['base_z']) == pytest.approx(
        robot['base']['centre_xyz'][2])
    assert float(arguments['wheel_axle_x']) == pytest.approx(
        robot['drive_wheel']['axle_x_m'])
    assert float(arguments['caster_x']) == pytest.approx(
        robot['caster']['centre_x_m'])
    camera = yaml.safe_load(
        (description / 'config' / 'inspection_camera.yaml').read_text()
    )['inspection_camera']
    body = robot['inspection_payload']['camera_body']
    bracket = robot['inspection_payload']['bracket']
    assert float(arguments['camera_body_mass']) == pytest.approx(body['mass_kg'])
    assert float(arguments['camera_body_z']) == pytest.approx(
        body['centre_xyz_m'][2])
    assert float(arguments['camera_bracket_mass']) == pytest.approx(
        bracket['mass_kg'])
    assert float(arguments['camera_bracket_x']) == pytest.approx(
        bracket['centre_xyz_m'][0])
    assert float(arguments['camera_optical_x']) == pytest.approx(
        camera['optical_mount']['center_xyz_m'][0])
    assert float(arguments['camera_optical_z']) == pytest.approx(
        camera['optical_mount']['center_xyz_m'][2])
    image = camera['image']
    intrinsics = camera['calibration']['intrinsics']
    assert int(arguments['camera_image_width']) == image['width_px']
    assert int(arguments['camera_image_height']) == image['height_px']
    simulated_camera = simulation['inspection_camera']
    render_scale = simulated_camera['render_overscan_focal_scale']
    assert float(arguments['camera_render_fx']) == pytest.approx(
        intrinsics['fx_px'] * render_scale)
    assert float(arguments['camera_render_fy']) == pytest.approx(
        intrinsics['fy_px'] * render_scale)
    assert float(arguments['camera_render_hfov']) == pytest.approx(
        2.0 * math.atan(
            image['width_px'] / (2.0 * intrinsics['fx_px'] * render_scale)))
    assert arguments['camera_triggered'] == str(
        simulated_camera['triggered']).lower()
    assert arguments['camera_image_format'] == simulated_camera['image_format']
    assert float(arguments['camera_update_rate']) == pytest.approx(
        simulated_camera['update_rate_hz'])
    assert float(arguments['camera_noise_stddev']) == pytest.approx(
        simulated_camera['noise_stddev'])
    assert float(arguments['contact_rate']) == pytest.approx(
        simulation['contact']['update_rate_hz'])


def test_camera_sensor_is_triggered_and_publishes_a_wider_ideal_source():
    """Pin private transport topics and leave distortion to the Ogre2 adapter."""
    root = ElementTree.parse(
        PACKAGE_ROOT / 'models' / 'climbot' / 'model.sdf.xacro').getroot()
    sensor = root.find(
        "model/link[@name='inspection_camera_link']/sensor"
        "[@name='inspection_camera']")
    assert sensor is not None
    camera = sensor.find('camera')
    assert camera.findtext('triggered') == '$(arg camera_triggered)'
    assert camera.findtext('trigger_topic') == (
        '/simulation/inspection_camera/trigger')
    assert camera.findtext('camera_info_topic') == (
        '/simulation/inspection_camera/ideal_camera_info')
    assert sensor.findtext('topic') == '/simulation/inspection_camera/ideal_image'
    assert camera.find('distortion') is None
    intrinsics = camera.find('lens/intrinsics')
    assert intrinsics.findtext('fx') == '$(arg camera_render_fx)'
    assert intrinsics.findtext('fy') == '$(arg camera_render_fy)'
    assert camera.findtext('horizontal_fov') == '$(arg camera_render_hfov)'


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
