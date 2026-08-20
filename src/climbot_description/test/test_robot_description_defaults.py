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

"""Verify standalone URDF defaults match the shared robot description."""

from pathlib import Path
from xml.etree import ElementTree

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
XACRO_NAMESPACE = 'http://www.ros.org/wiki/xacro'


def test_urdf_geometry_defaults_match_robot_yaml():
    """Keep independent xacro rendering on the authoritative geometry."""
    robot = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'robot.yaml').read_text())['robot']
    root = ElementTree.parse(
        PACKAGE_ROOT / 'urdf' / 'climbot.urdf.xacro').getroot()
    arguments = {
        element.attrib['name']: float(element.attrib['default'])
        for element in root.findall(f'{{{XACRO_NAMESPACE}}}arg')
    }
    assert arguments['base_x'] == pytest.approx(robot['base']['centre_xyz'][0])
    assert arguments['base_z'] == pytest.approx(robot['base']['centre_xyz'][2])
    assert arguments['wheel_axle_x'] == pytest.approx(
        robot['drive_wheel']['axle_x_m'])
    assert arguments['caster_x'] == pytest.approx(robot['caster']['centre_x_m'])
