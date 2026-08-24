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

"""Validate the shared inspection-camera calibration and payload proxy."""

import math
from pathlib import Path
from xml.etree import ElementTree

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
XACRO_NAMESPACE = 'http://www.ros.org/wiki/xacro'


def load_configs():
    camera = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'inspection_camera.yaml').read_text()
    )['inspection_camera']
    robot = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'robot.yaml').read_text())['robot']
    return camera, robot


def finite_vector(values, length):
    return len(values) == length and all(
        isinstance(value, (int, float)) and math.isfinite(value)
        for value in values)


def test_nominal_calibration_is_finite_and_inside_the_image():
    camera, _ = load_configs()
    image = camera['image']
    calibration = camera['calibration']
    intrinsics = calibration['intrinsics']
    assert image == {'width_px': 1920, 'height_px': 1080}
    assert calibration['distortion_model'] == 'plumb_bob'
    assert finite_vector(calibration['distortion'], 5)
    assert any(value != 0.0 for value in calibration['distortion'])
    assert intrinsics['fx_px'] > 0.0
    assert intrinsics['fy_px'] > 0.0
    assert 0.0 <= intrinsics['cx_px'] < image['width_px']
    assert 0.0 <= intrinsics['cy_px'] < image['height_px']
    assert all(math.isfinite(float(value)) for value in intrinsics.values())


def test_pinhole_field_contains_the_effective_footprint():
    camera, _ = load_configs()
    image = camera['image']
    intrinsics = camera['calibration']['intrinsics']
    distance = camera['optical_mount']['center_xyz_m'][2]
    raw_width = image['width_px'] * distance / intrinsics['fx_px']
    raw_length = image['height_px'] * distance / intrinsics['fy_px']
    footprint = camera['footprint']
    assert raw_width == pytest.approx(0.550)
    assert raw_length == pytest.approx(0.309375)
    assert footprint['effective_width_m'] <= raw_width
    assert footprint['effective_length_m'] <= raw_length


def test_camera_and_bracket_proxy_have_physical_mass_and_clear_the_view():
    camera, robot = load_configs()
    body = robot['inspection_payload']['camera_body']
    bracket = robot['inspection_payload']['bracket']
    assert body['mass_kg'] > 0.0
    assert bracket['mass_kg'] > 0.0
    for key in ('size_xyz_m', 'inertia_box_size_xyz_m',
                'upright_size_xyz_m', 'top_arm_size_xyz_m'):
        source = body if key == 'size_xyz_m' else bracket
        assert finite_vector(source[key], 3)
        assert all(value > 0.0 for value in source[key])

    optical = camera['optical_mount']['center_xyz_m']
    raw_half_length = (
        camera['image']['height_px'] * optical[2] /
        camera['calibration']['intrinsics']['fy_px'] / 2.0)
    chassis_front = (
        robot['base']['centre_xyz'][0] + robot['base']['size_xyz'][0] / 2.0)
    chassis_top = (
        robot['base']['centre_xyz'][2] + robot['base']['size_xyz'][2] / 2.0)
    rear_ray_at_chassis = (
        optical[0] - (1.0 - chassis_top / optical[2]) * raw_half_length)
    assert rear_ray_at_chassis - chassis_front >= 0.020

    upright_front = (
        bracket['upright_centre_xyz_m'][0] +
        bracket['upright_size_xyz_m'][0] / 2.0)
    assert upright_front < rear_ray_at_chassis
    arm_wall_side = (
        bracket['top_arm_centre_xyz_m'][2] -
        bracket['top_arm_size_xyz_m'][2] / 2.0)
    assert arm_wall_side > optical[2]


def test_proxy_total_mass_and_centre_of_mass_are_reproducible():
    _, robot = load_configs()
    components = [
        (robot['base']['mass_kg'], robot['base']['centre_xyz']),
        (robot['drive_wheel']['mass_kg'], [
            robot['drive_wheel']['axle_x_m'],
            robot['drive_wheel']['separation_m'] / 2.0,
            robot['drive_wheel']['radius_m']]),
        (robot['drive_wheel']['mass_kg'], [
            robot['drive_wheel']['axle_x_m'],
            -robot['drive_wheel']['separation_m'] / 2.0,
            robot['drive_wheel']['radius_m']]),
        (robot['caster']['mass_kg'], [
            robot['caster']['centre_x_m'], 0.0, robot['caster']['radius_m']]),
        (robot['inspection_payload']['camera_body']['mass_kg'],
         robot['inspection_payload']['camera_body']['centre_xyz_m']),
        (robot['inspection_payload']['bracket']['mass_kg'],
         robot['inspection_payload']['bracket']['centre_xyz_m']),
    ]
    total_mass = sum(mass for mass, _ in components)
    centre = [
        sum(mass * xyz[axis] for mass, xyz in components) / total_mass
        for axis in range(3)
    ]
    assert total_mass == pytest.approx(16.5)
    assert centre == pytest.approx([-0.0566666667, 0.0, 0.1091515152])


def test_urdf_camera_defaults_match_shared_configs():
    camera, robot = load_configs()
    root = ElementTree.parse(
        PACKAGE_ROOT / 'urdf' / 'climbot.urdf.xacro').getroot()
    arguments = {
        element.attrib['name']: float(element.attrib['default'])
        for element in root.findall(f'{{{XACRO_NAMESPACE}}}arg')
    }
    body = robot['inspection_payload']['camera_body']
    bracket = robot['inspection_payload']['bracket']
    assert arguments['camera_body_mass'] == pytest.approx(body['mass_kg'])
    assert arguments['camera_body_x'] == pytest.approx(body['centre_xyz_m'][0])
    assert arguments['camera_bracket_mass'] == pytest.approx(bracket['mass_kg'])
    assert arguments['camera_bracket_z'] == pytest.approx(
        bracket['centre_xyz_m'][2])
    assert arguments['camera_optical_x'] == pytest.approx(
        camera['optical_mount']['center_xyz_m'][0])
    assert arguments['camera_optical_z'] == pytest.approx(
        camera['optical_mount']['center_xyz_m'][2])
    assert arguments['camera_optical_roll'] == pytest.approx(
        camera['optical_mount']['rpy_rad'][0])
