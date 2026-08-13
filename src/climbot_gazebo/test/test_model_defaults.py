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
    assert arguments['contact_rate'] == pytest.approx(
        simulation['contact']['update_rate_hz'])
