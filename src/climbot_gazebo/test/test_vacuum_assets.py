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

"""Contracts for the candidate runtime and its single source of geometry."""

from copy import deepcopy
import math
from xml.etree import ElementTree as ET

from climbot_gazebo.vacuum_assets import build_assets
import pytest


def test_b_c_only_change_the_pressure_source(tmp_path):
    """Hold geometry, skirt, starting state and actuator models identical."""
    roots = []
    for mode in ['fixed_pressure', 'fan_ode']:
        root = tmp_path / mode
        build_assets(root, mode=mode)
        tree = ET.parse(root / 'world.sdf')
        plugin = tree.find('.//plugin[@name="climbot_gazebo::VacuumSystem"]')
        assert plugin is not None
        assert plugin.findtext('pressure_mode') == mode
        plugin.find('pressure_mode').text = 'same-source'
        roots.append(ET.tostring(tree.getroot()))
    assert roots[0] == roots[1]


def test_native_contacts_and_skirt_have_one_force_path(tmp_path):
    """Require native wheel contacts and an exclusively analytic skirt."""
    result = build_assets(tmp_path)
    world = ET.parse(tmp_path / 'world.sdf')
    assert not any('apply-link-wrench' in p.get('filename', '')
                   for p in world.findall('.//plugin'))
    model = world.find('.//model[@name="climbot"]')
    skirt = model.find('link[@name="skirt_link"]')
    assert not skirt.findall('collision')
    for name in ['left_wheel_link', 'right_wheel_link', 'caster_ball_link']:
        link = model.find(f'link[@name="{name}"]')
        assert len(link.findall('collision')) == 1
        assert link.findtext('sensor/contact/collision') == link.find('collision').get('name')
    mass = sum(float(x.text) for x in model.findall('link/inertial/mass'))
    assert mass == pytest.approx(result['model']['mass_kg'], abs=1e-11)
    assert result['model']['caster_x_m'] == pytest.approx(-.3, abs=1e-14)
    assert model.find('.//light[@name="inspection_led"]') is not None


@pytest.mark.parametrize('heading', [0, 90, 180, 270])
def test_wall_surface_and_pressure_normal_have_one_transform(tmp_path, heading):
    """Yaw changes the drive heading, preserving the physical wall normal."""
    result = build_assets(tmp_path, heading_deg=heading)
    world = ET.parse(tmp_path / 'world.sdf')
    plugin = world.find('.//plugin[@name="climbot_gazebo::VacuumSystem"]')
    surface = [float(v) for v in plugin.findtext('wall_pose').split()]
    assert surface == pytest.approx(result['wall_surface_pose'])
    assert surface[0] == pytest.approx(.05)
    assert result['nominal_startup_minimum_pa'] > 0
    assert not math.isnan(result['model']['mass_kg'])


@pytest.mark.parametrize('kwargs', [
    {'mode': 'legacy'}, {'step_s': .002}, {'heading_deg': float('nan')},
    {'runtime_overrides': {'typo': 1}},
])
def test_invalid_runtime_selection_is_explicit(tmp_path, kwargs):
    """A rejected choice never silently becomes the legacy force model."""
    with pytest.raises(ValueError):
        build_assets(tmp_path, **deepcopy(kwargs))


@pytest.mark.parametrize('io', [
    {'sensor': {'rate_hz': 70}}, {'sensor': {'delay_s': .0005}},
    {'sensor': {'seed': -1}}, {'sensor': {'noise_pa': float('nan')}},
    {'sensor': {'minimum_pa': 130000}}, {'sensor': {'typo': 1}},
    {'events': [{'time_s': -1, 'kind': 0, 'index': 0, 'value': 1}]},
    {'events': [{'time_s': 1, 'kind': 0, 'index': 3, 'value': 1}]},
    {'events': [{'time_s': 1, 'kind': 2, 'index': 0, 'value': 3}]},
])
def test_sensor_and_fault_configuration_reject_invalid_contracts(tmp_path, io):
    """Reject invalid grids and commands before starting physics."""
    with pytest.raises(ValueError):
        build_assets(tmp_path, io_overrides=io)
