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
import hashlib
import json
import math
from pathlib import Path
from urllib.parse import unquote, urlparse
from xml.etree import ElementTree as ET

from climbot_gazebo.vacuum_assets import build_assets
import numpy as np
from PIL import Image
import pytest
import yaml


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
        # Generated assets live beside each world. Compare their actual bytes,
        # preserving the B/C geometry contract across different output paths.
        for uri in tree.findall('.//visual/geometry/mesh/uri'):
            path = Path(unquote(urlparse(uri.text).path))
            assert path.parent == root / 'visual_meshes'
            uri.text = 'sha256:' + hashlib.sha256(path.read_bytes()).hexdigest()
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
    {'opening_schedule': [{'time_s': .1, 'area_m2': 0}]},
    {'opening_schedule': [{'time_s': 0, 'area_m2': -.001}]},
    {'opening_schedule': [{'time_s': 0, 'area_m2': .03}]},
    {'opening_schedule': [{'time_s': 0, 'area_m2': 0}, {'time_s': 0, 'area_m2': .001}]},
])
def test_sensor_and_fault_configuration_reject_invalid_contracts(tmp_path, io):
    """Reject invalid grids and commands before starting physics."""
    with pytest.raises(ValueError):
        build_assets(tmp_path, io_overrides=io)


def test_candidate_tf_tree_contains_all_shared_sensor_frames(tmp_path):
    """Pressure/IMU headers and moving wheels need a connected actual URDF."""
    build_assets(tmp_path, runtime_overrides={'camera_enabled': True})
    urdf = ET.parse(tmp_path / 'robot.urdf')
    parent = {j.find('child').get('link'): j.find('parent').get('link')
              for j in urdf.findall('joint')}
    for child in ['chassis_link', 'imu_link', 'inspection_camera_optical_frame',
                  'left_wheel_link', 'right_wheel_link']:
        seen = set()
        while child != 'base_link':
            assert child not in seen
            seen.add(child)
            child = parent[child]
    world = ET.parse(tmp_path / 'world.sdf')
    assert world.find('.//frame[@name="imu_link"]') is not None


def test_collapsed_skirt_has_native_frictionless_stops(tmp_path):
    """The stop starts at the declared compression, without sealing a groove."""
    resolved = build_assets(tmp_path)
    cfg = resolved['model']['config']
    limit = resolved['physics']['skirt']['maximum_compression_m']
    stop_gap = cfg['skirt']['free_height_m'] - limit
    tree = ET.parse(tmp_path / 'model.sdf')
    stops = [c for c in tree.findall('.//link[@name="chassis_link"]/collision')
             if c.get('name', '').startswith('skirt_stop_')]
    assert len(stops) == 4
    for collision in stops:
        pose = list(map(float, collision.findtext('pose').split()))
        size = list(map(float, collision.findtext('geometry/box/size').split()))
        assert pose[2] - size[2]/2 == pytest.approx(cfg['body']['bottom_z_m']-stop_gap)
        assert float(collision.findtext('surface/friction/ode/mu')) == 0
        assert float(collision.findtext('surface/friction/ode/mu2')) == 0


def test_opt_in_caster_proxy_preserves_mass_visuals_and_scales_rolling(tmp_path):
    """Real and proxy rolling must have equal I/r² and opposing wall masks."""
    trees = []
    for enabled in [False, True]:
        out = tmp_path / str(enabled)
        resolved = build_assets(out, caster_proxy=enabled)
        trees.append(ET.parse(out / 'world.sdf'))
        urdf = ET.parse(out / 'robot.urdf')
        ball = urdf.find('link[@name="caster_ball_link"]')
        assert float(ball.find('collision/geometry/sphere').get('radius')) == pytest.approx(
            .00625 if enabled else .0125)
        assert float(ball.find('visual/geometry/sphere').get('radius')) == .0125
        assert ('caster_proxy' in resolved) == enabled
    a, b = [t.find('.//model[@name="climbot"]/link[@name="caster_ball_link"]')
            for t in trees]
    assert a.findtext('inertial/mass') == b.findtext('inertial/mass')
    assert a.findtext('pose') == b.findtext('pose')
    assert ET.tostring(a.find('visual')) == ET.tostring(b.find('visual'))
    for key in ['ixx', 'iyy', 'izz']:
        assert float(b.findtext('inertial/inertia/' + key)) == pytest.approx(
            float(a.findtext('inertial/inertia/' + key)) / 4)
    for collision in trees[1].findall('.//model[@name="climbot"]/link/collision'):
        assert collision.findtext('surface/contact/collide_bitmask') == (
            '2' if collision.get('name') == 'caster_collision' else '1')
    assert trees[1].findtext('.//model[@name="wall"]//collide_bitmask') == '1'
    assert trees[1].findtext('.//model[@name="caster_proxy_wall"]//collide_bitmask') == '2'
    assert trees[1].findtext('.//plugin/caster_radius_m') == '0.00625'
    assert trees[0].find('.//model[@name="caster_proxy_wall"]') is None


@pytest.mark.parametrize('proxy', [False, True])
def test_compliant_wheels_preserve_mass_inertia_and_publish_moving_tf_chain(tmp_path, proxy):
    """Axle splitting preserves the assembly and every moving TF has a state source."""
    reference = tmp_path / 'rigid'
    build_assets(reference, caster_proxy=proxy)
    out = tmp_path / 'soft'
    build_assets(out, wheel_compliance=True, caster_proxy=proxy)
    rigid = ET.parse(reference / 'model.sdf')
    soft = ET.parse(out / 'model.sdf')
    urdf = ET.parse(out / 'robot.urdf')
    published = soft.findall('.//plugin[@name="gz::sim::systems::JointStatePublisher"]/joint_name')
    assert {n.text for n in published} == {
        'left_wheel_joint', 'right_wheel_joint', 'left_tyre_joint', 'right_tyre_joint'}
    assert sum(float(n.text) for n in rigid.findall('.//inertial/mass')) == pytest.approx(
        sum(float(n.text) for n in soft.findall('.//inertial/mass')), abs=1e-12)
    for side in ['left', 'right']:
        original = rigid.find(f'.//link[@name="{side}_wheel_link"]')
        wheel = soft.find(f'.//link[@name="{side}_wheel_link"]')
        axle = soft.find(f'.//link[@name="{side}_wheel_axle_link"]')
        assert original.findtext('pose') == wheel.findtext('pose') == axle.findtext('pose')
        for key in ['ixx', 'iyy', 'izz']:
            assert float(original.findtext('inertial/inertia/' + key)) == pytest.approx(
                float(wheel.findtext('inertial/inertia/' + key)) +
                float(axle.findtext('inertial/inertia/' + key)), abs=1e-14)
        spring = urdf.find(f'joint[@name="{side}_tyre_joint"]')
        rotation = urdf.find(f'joint[@name="{side}_wheel_joint"]')
        assert spring.find('parent').get('link') == 'base_link'
        assert spring.find('child').get('link') == rotation.find('parent').get('link')
        assert spring.find('axis').get('xyz') == '0 0 1'
        assert rotation.find('origin').get('xyz') == '0 0 0'
        assert float(spring.find('limit').get('lower')) == 0
    assert rigid.find('.//joint[@name="left_tyre_joint"]') is None


@pytest.mark.parametrize('representation,amplitude,radius', [
    ('mesh', 0, .0475), ('native_heightmap', 0, .05),
    ('native_heightmap', .002, .0475)])
def test_surface_patch_keeps_one_source_and_excludes_hidden_flat_support(
        tmp_path, monkeypatch, representation, amplitude, radius):
    """Serialize through SDF-safe flow text and partition the two contact meshes."""
    from climbot_gazebo import vacuum_assets
    original_read = vacuum_assets._read

    def configured_radius(path, key):
        result = original_read(path, key)
        if key == 'vacuum_chassis':
            result['wheel']['radius_m'] = radius
        return result

    monkeypatch.setattr(vacuum_assets, '_read', configured_radius)
    config = yaml.safe_load((Path(__file__).parents[1] /
                            'config/m2_surface_fixture.yaml').read_text())
    config['roughness']['amplitude_m'] = amplitude
    config['contact_representation'] = representation
    config['grooves'] = config['grooves'][:2]
    for index, groove in enumerate(config['grooves']):
        groove['start_m'] = [-.02, .6] if index == 0 else [.6, -.02]
        groove['end_m'] = [1.22, .6] if index == 0 else [.6, 1.22]
    resolved = build_assets(tmp_path, mode='fixed_pressure', caster_proxy=True,
                            surface_patch=config)
    tree = ET.parse(tmp_path / 'world.sdf')
    assert tree.find('.//model[@name="wall"]') is None
    assert tree.find('.//model[@name="caster_proxy_wall"]') is None
    real = tree.find('.//model[@name="m2_real_wall"]')
    proxy = tree.find('.//model[@name="m2_caster_wall"]')
    assert real.findtext('.//collide_bitmask') == '1'
    assert proxy.findtext('.//collide_bitmask') == '2'
    assert proxy.find('.//visual') is None
    text = tree.findtext('.//plugin/wall_surface_yaml')
    assert '\n' not in text
    assert json.loads(text) == yaml.safe_load(
        (tmp_path / 'surface_patch/source.yaml').read_text())
    assert yaml.safe_load(' '.join(text.split())) == json.loads(text)
    assert text == ET.parse(tmp_path / 'model.sdf').findtext('.//plugin/wall_surface_yaml')
    assert resolved['surface_patch']['proxy_centroid_error_m'] < .00001
    if representation == 'native_heightmap':
        from climbot_gazebo.wall_surface import WallSurface
        source = WallSurface(config)
        assert source.groove_depth(.6, .6) >= .003
        assert real.findtext('.//visual/geometry/mesh/uri').endswith('/real.obj')
        assert proxy.findtext('.//collision/geometry/mesh/uri').endswith('/proxy.obj')
        patch = resolved['surface_patch']
        assert patch['driver_radius_m'] == resolved['model']['config']['wheel']['radius_m']
        assert patch['ideal_driver_omitted_sag_m'] == pytest.approx({
            'single': radius - math.sqrt(radius**2 - .0025**2),
            'cross_diagonal': radius - math.sqrt(radius**2 - 2*.0025**2)})
        assert patch['driver_surface_policy'] == 'roughness_only'
        if amplitude:
            grid = np.linspace(0, 1.2, 257)
            u, v = np.meshgrid(grid, grid)
            expected = source.roughness(u, v)
            low, high = expected.min(), expected.max()
            shape = real.find('.//collision/geometry/heightmap')
            assert shape is not None
            assert list(map(float, shape.findtext('size').split())) == pytest.approx(
                [1.2*257/256, 1.2*257/256, high-low])
            pixels = np.flipud(np.asarray(Image.open(tmp_path / 'surface_patch/height.png')))
            decoded = low + pixels/65535*(high-low)
            assert np.max(np.abs(decoded-expected)) <= (high-low)/65535
            assert abs(decoded[128, 128] - source.height(.6, .6)) > .003
            assert patch['driver_collision_geometry'] == 'heightmap'
        else:
            # Zero-range roughness is exactly a plane, while the full source
            # still contains grooves for the skirt, caster and visual.
            assert real.find('.//collision/geometry/heightmap') is None
            assert real.findtext('.//collision/geometry/box/size') == '1.2 1.2 0.05'
            assert real.findtext('.//collision/pose') == '0 0 -0.025 0 0 0'
            assert patch['driver_collision_geometry'] == 'box'


def test_surface_patch_rejects_unvalidated_fan_coupling(tmp_path):
    """The contact prototype cannot silently pass as a fan/leak acceptance case."""
    with pytest.raises(ValueError, match='fixed_pressure'):
        build_assets(tmp_path, caster_proxy=True, surface_patch={})
