# Copyright 2026 jerry
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Check source geometry independently of collision resolution and sampling phase."""

from copy import deepcopy
from pathlib import Path

from climbot_gazebo.wall_surface import WallSurface
import numpy as np
import pytest
import yaml


def config():
    return yaml.safe_load((Path(__file__).parents[1] /
                           'config/m2_surface_fixture.yaml').read_text())


def test_seed_and_amplitude_bound_are_independent_of_query_resolution():
    """Evaluation at shared points agrees exactly without reseeding per tile."""
    a = WallSurface(config())
    b = WallSurface(config())
    u, v = np.meshgrid(np.linspace(0, .8, 201), np.linspace(0, .8, 203))
    assert np.array_equal(a.roughness(u, v), b.roughness(u, v))
    assert np.max(np.abs(a.roughness(u, v))) <= .002
    assert a.roughness(.31, .72) == b.roughness(np.array([.31]), np.array([.72]))[0]
    changed = config()
    changed['roughness']['seed'] += 1
    assert not np.array_equal(a.roughness(u, v), WallSurface(changed).roughness(u, v))


def test_real_joint_width_depth_and_no_double_cut_at_intersection():
    """Depth below the local surface is 3–5 mm, regardless of base roughness."""
    cfg = config()
    cfg['grooves'] = cfg['grooves'][:1]
    surface = WallSurface(cfg)
    u = np.linspace(0, .8, 301)
    cut = surface.roughness(u, .4) - surface.height(u, .4)
    assert np.min(cut) >= .003 - 1e-12
    assert np.max(cut) <= .005 + 1e-12
    for side in [-1, 1]:
        assert surface.groove_depth(.3, .4 + side * .0025001) == 0
        assert surface.groove_depth(.3, .4 + side * .00249) > 0
    duplicate = deepcopy(cfg['grooves'][0])
    cfg['grooves'].append(duplicate)
    assert np.array_equal(surface.height(u, .4), WallSurface(cfg).height(u, .4))


def test_crack_tapers_to_closed_tip_in_both_width_and_depth():
    """A crack terminates instead of creating a constant-width rectangular hole."""
    cfg = config()
    cfg['grooves'] = [cfg['grooves'][2]]
    surface = WallSurface(cfg)
    start = np.array(cfg['grooves'][0]['start_m'])
    end = np.array(cfg['grooves'][0]['end_m'])
    delta = (end - start) / np.linalg.norm(end - start)
    assert surface.groove_depth(*start) == 0
    assert surface.groove_depth(*(start - .001 * delta)) == 0
    assert surface.groove_depth(*(start + .03 * delta)) > .001
    assert 0 < surface.groove_depth(*(start + .001 * delta)) < .0001
    normal = surface.normal(.6, .6)
    assert np.linalg.norm(normal) == pytest.approx(1)
    assert normal[2] > 0


@pytest.mark.parametrize('change', ['amplitude', 'scale', 'zero_length', 'negative_depth'])
def test_invalid_source_geometry_is_rejected(change):
    """Do not let malformed fixtures turn into unbounded forces or invalid assets."""
    cfg = config()
    if change == 'amplitude':
        cfg['roughness']['amplitude_m'] = float('nan')
    elif change == 'scale':
        cfg['roughness']['scale_range_m'] = [.8, .2]
    elif change == 'zero_length':
        cfg['grooves'][0]['end_m'] = cfg['grooves'][0]['start_m']
    else:
        cfg['grooves'][0]['depth_range_m'] = [-.001, .003]
    with pytest.raises(ValueError):
        WallSurface(cfg)


def test_joint_mesh_keeps_open_section_and_valid_normals(tmp_path):
    """Rim-aligned cells resolve the actual 5 mm opening without a top cap."""
    from climbot_gazebo.wall_surface import write_tensor_mesh
    cfg = config()
    cfg['grooves'] = cfg['grooves'][:1]
    surface = WallSurface(cfg)
    u = np.linspace(0, .8, 81)
    v = np.unique(np.r_[np.linspace(0, .8, 81),
                        np.linspace(.3975, .398, 5), np.linspace(.402, .4025, 5)])
    target = tmp_path / 'surface.obj'
    mesh = write_tensor_mesh(target, surface, u, v)
    vertices, normals, faces = [], [], []
    for line in target.read_text().splitlines():
        if line.startswith('v '):
            vertices.append([float(x) for x in line.split()[1:]])
        elif line.startswith('vn '):
            normals.append([float(x) for x in line.split()[1:]])
        elif line.startswith('f '):
            faces.append([int(x.split('//')[0])-1 for x in line.split()[1:]])
    vertices = np.array(vertices)
    triangles = vertices[np.array(faces)]
    centroid = triangles.mean(axis=1)
    expected = surface.height(centroid[:, 0] + .4, centroid[:, 1] + .4)
    assert np.max(np.abs(expected - centroid[:, 2])) < .000005
    assert np.all(np.cross(triangles[:, 1] - triangles[:, 0],
                           triangles[:, 2] - triangles[:, 0])[:, 2] > 0)
    assert len(normals) == len(vertices) == mesh['vertices']
    assert np.allclose(np.linalg.norm(normals, axis=1), 1)
    assert mesh['triangles'] == len(faces) < 15000
    at_bottom = np.isclose(vertices[:, 1], 0, atol=1e-12)
    cut = (surface.roughness(vertices[at_bottom, 0] + .4, .4) -
           vertices[at_bottom, 2])
    assert np.min(cut) >= .003 - 1e-12


def test_light_outside_probe_checks_damped_flight_not_fixed_drop(tmp_path):
    """A tiny gauge can leave the wall correctly without dropping 50 mm."""
    import csv
    import sys
    sys.path.insert(0, str(Path(__file__).parents[3] / 'tools'))
    from probe_m2_wall_surface import evaluate
    cfg = config()
    cfg['roughness']['amplitude_m'] = 0
    cfg['grooves'] = []
    native = tmp_path / 'native'
    native.mkdir()
    (native / 'run.yaml').write_text('iterations: 350\n')
    (native / 'contacts.csv').write_text('time_s,name\n')
    fixture = {'orientation': 'vertical', 'gauge_mass_kg': .0001,
               'joint_damping_ns_m': .02, 'radius_m': .0001,
               'probes': [{'name': 'probe_00', 'outside': True,
                           'initial_height_m': .000101}]}
    for stuck in [False, True]:
        with (native / 'frames.csv').open('w') as stream:
            writer = csv.writer(stream)
            writer.writerow(['time_s', 'name', 'x', 'contacts'])
            for i in range(100):
                time = .25 + i * .001
                # Independently tabulated terminal-speed trajectory: tau=5 ms,
                # v_terminal=49.05 mm/s; exp(-t/tau) is negligible in this window.
                position = .000101 if stuck else .000101 - .04905 * (time - .005)
                writer.writerow([time, 'probe_00', position, 0])
        result = evaluate(tmp_path, cfg, fixture)
        assert result['assertions']['probe_00_outside'] is not stuck


@pytest.mark.parametrize('change', ['no_groove', 'width', 'oblique'])
def test_fixed_mesh_probe_rejects_a_different_wall_before_generation(tmp_path, change):
    """A flat or differently placed groove must not pass as this declared fixture."""
    import sys
    sys.path.insert(0, str(Path(__file__).parents[3] / 'tools'))
    from probe_m2_wall_surface import make_fixture
    cfg = config()
    cfg['grooves'] = cfg['grooves'][:1]
    if change == 'no_groove':
        cfg['grooves'] = []
    elif change == 'width':
        cfg['grooves'][0]['width_m'] = .008
    else:
        cfg['grooves'][0]['end_m'][1] = .5
    target = tmp_path / 'fixture'
    with pytest.raises(ValueError, match='declared horizontal'):
        make_fixture(target, cfg, 257, .001, 'vertical', representation='joint_mesh')
    assert not target.exists()


@pytest.mark.parametrize('samples', [129, 257])
def test_heightmap_edge_fixture_resolves_intended_extent_at_each_density(tmp_path, samples):
    """Probe intended edges, rather than shrinking the reference to match the engine."""
    import sys
    from xml.etree import ElementTree as ET
    sys.path.insert(0, str(Path(__file__).parents[3] / 'tools'))
    from probe_m2_wall_surface import make_fixture
    cfg = config()
    cfg['grooves'] = []
    target = tmp_path / 'fixture'
    fixture = make_fixture(target, cfg, samples, .001, 'vertical', boundary_probes=True)
    world = ET.parse(target / 'world.sdf')
    span = [float(x) for x in world.find('.//heightmap/size').text.split()]
    # Engine convention checked natively in Subway and this project's T0:
    # N points occupy N-1 intervals, each spaced SDF size / N.
    loaded_span = np.array(span[:2]) / samples * (samples - 1)
    assert np.allclose(loaded_span, [.8, .8], rtol=0, atol=1e-14)
    assert world.find('.//heightmap/pos').text == '0 0 0'
    probes = fixture['probes']
    assert len(probes) == 24
    assert sum(p['outside'] for p in probes) == 12
    radius = fixture['radius_m']
    for probe in probes:
        u, v = probe['u_m'], probe['v_m']
        if probe['outside']:
            assert max(-u, u - .8, -v, v - .8) > radius
        else:
            assert min(u, .8-u, v, .8-v) > radius
        edge_distance = min(abs(u), abs(.8-u), abs(v), abs(.8-v))
        assert edge_distance + radius <= fixture['boundary_support_tolerance_m'] + 1e-15


@pytest.mark.parametrize('samples', [1, 2, 128, 256, 257.0])
def test_heightmap_invalid_sampling_rejected_before_native_crash(tmp_path, samples):
    """The native heightmap loader requires a square 2^n+1 lattice."""
    import sys
    sys.path.insert(0, str(Path(__file__).parents[3] / 'tools'))
    from probe_m2_wall_surface import make_fixture
    target = tmp_path / 'fixture'
    with pytest.raises(ValueError, match='sample count'):
        make_fixture(target, config(), samples, .001, 'vertical')
    assert not target.exists()


def test_contact_approximation_retains_joints_and_does_not_change_leak_source():
    """Only explicitly classified narrow cracks may disappear from wheel collision."""
    source = WallSurface(config())
    contact = source.contact_view('bridge_narrow_cracks')
    assert [g['id'] for g in contact.config['grooves']] == [
        'horizontal_joint', 'vertical_joint']
    assert len(contact.config['omitted_cracks']) == 2
    assert contact.config['surface_role'] == 'wheel_contact'
    assert len(source.config['grooves']) == 4
    assert source.groove_depth(.225, .22) > .001
    assert contact.groove_depth(.225, .22) == 0
    assert source.height(.7, .4) == contact.height(.7, .4)
    assert source.height(.45, .65) == contact.height(.45, .65)
    assert source.height(.225, .22) < contact.height(.225, .22)
    # No label means retain the geometry, rather than guess from its name.
    unclassified = config()
    unclassified['grooves'][2].pop('kind')
    view = WallSurface(unclassified).contact_view('bridge_narrow_cracks')
    assert 'main_crack' in [g['id'] for g in view.config['grooves']]


@pytest.mark.parametrize('change', ['width', 'depth', 'policy'])
def test_contact_approximation_rejects_unapproved_geometry(change):
    """Do not silently flatten a broad crack or an unknown contact policy."""
    cfg = config()
    if change == 'width':
        cfg['grooves'][2]['width_m'] = .0021
    elif change == 'depth':
        cfg['grooves'][2]['depth_range_m'][1] = .0031
    with pytest.raises(ValueError):
        WallSurface(cfg).contact_view('unknown' if change == 'policy' else 'bridge_narrow_cracks')


def test_sampled_ball_and_cylinder_support_obey_isolated_slot_bound():
    """Use wheel support, rather than raw groove-bottom height, for this proxy."""
    import sys
    sys.path.insert(0, str(Path(__file__).parents[3] / 'tools'))
    from check_m2_crack_bridge import sag, support_difference
    cfg = config()
    cfg['roughness']['amplitude_m'] = 0
    cfg['grooves'] = cfg['grooves'][:1]
    cfg['grooves'][0]['width_m'] = .002
    cfg['grooves'][0]['depth_range_m'] = [.003, .003]
    source = WallSurface(cfg)
    for radius, width in [(.0125, 0), (.0475, .047)]:
        difference = support_difference(source, [.4, .4], radius, width, 90, .00005)
        assert .7*sag(radius, .002) <= difference <= sag(radius, .002)+.000002
    assert sag(.0125, .002) < .000041


def test_proxy_fixture_deduplicates_branch_start_and_preserves_role(tmp_path):
    """The branch origin and main midpoint must not spawn colliding gauges."""
    import sys
    from xml.etree import ElementTree as ET
    sys.path.insert(0, str(Path(__file__).parents[3] / 'tools'))
    from probe_m2_wall_surface import make_fixture
    cfg = config()
    cfg['grooves'] = cfg['grooves'][2:]
    fixture = make_fixture(tmp_path/'fixture', cfg, 257, .001, 'vertical',
                           representation='bridged_crack_heightmap')
    assert len(fixture['probes']) == 13
    assert fixture['contact_config']['contact_policy'] == 'bridge_narrow_cracks'
    assert not fixture['contact_config']['grooves']
    assert len(fixture['contact_config']['omitted_cracks']) == 2
    poses = [np.array([float(x) for x in model.find('pose').text.split()[:3]])
             for model in ET.parse(tmp_path/'fixture/world.sdf').findall('.//world/model')
             if model.get('name').startswith('probe_')]
    assert all(np.linalg.norm(a-b) > 2*fixture['radius_m']
               for i, a in enumerate(poses) for b in poses[i+1:])


def test_evaluator_rejects_contacts_between_gauges(tmp_path):
    """A fixture interaction must not be interpreted as a wall geometry defect."""
    import sys
    sys.path.insert(0, str(Path(__file__).parents[3] / 'tools'))
    from probe_m2_wall_surface import evaluate
    native = tmp_path/'native'
    native.mkdir()
    (native/'run.yaml').write_text('iterations: 350\n')
    (native/'frames.csv').write_text('time_s,name\n')
    fixture = {'orientation': 'vertical', 'probes': []}
    for other in ['surface::surface::surface_collision', 'probe_01::body::collision']:
        (native/'contacts.csv').write_text(
            'time_s,name,collision1_name,collision2_name\n'
            f'.3,probe_00,probe_00::body::collision,{other}\n')
        result = evaluate(tmp_path, config(), fixture)
        assert result['assertions']['only_surface_contacts'] is other.startswith('surface::')


def test_cross_joint_mesh_preserves_manifold_and_actual_depth_equality(tmp_path):
    """Share crease vertices without bridging a trough or adding overlapping patches."""
    from collections import Counter
    from climbot_gazebo.surface_mesh import audit_mesh_sections, write_cross_joint_mesh
    cfg = config()
    cfg['grooves'] = cfg['grooves'][:2]
    source = WallSurface(cfg)
    target = tmp_path/'cross.obj'
    result = write_cross_joint_mesh(target, source)
    assert result['vertices'] < 10000
    assert result['triangles'] < 20000
    assert result['explicit_crease_vertices'] > 0
    vertices, faces = [], []
    for line in target.read_text().splitlines():
        if line.startswith('v '):
            vertices.append([float(x) for x in line.split()[1:]])
        elif line.startswith('f '):
            faces.append([int(x.split('//')[0])-1 for x in line.split()[1:]])
    vertices = np.array(vertices)
    triangles = vertices[np.array(faces)]
    centroid = triangles.mean(axis=1)
    assert np.max(np.abs(source.height(centroid[:, 0]+.4, centroid[:, 1]+.4) -
                         centroid[:, 2])) < .000005
    edges = Counter(tuple(sorted((face[i], face[(i+1) % 3])))
                    for face in faces for i in range(3))
    for (a, b), count in edges.items():
        assert count in (1, 2)
        if count == 1:
            shared_boundary = np.isclose(vertices[a, :2], vertices[b, :2], atol=1e-12)
            at_boundary = np.isclose(np.abs(vertices[a, :2]), .4, atol=1e-12)
            assert np.any(shared_boundary & at_boundary)
    assert audit_mesh_sections(target, source)['passed']


def test_normal_cone_accepts_real_crease_but_rejects_unrelated_edge_normal():
    """A sharp crease has a bounded normal set, not an arbitrary normal exemption."""
    from climbot_gazebo.surface_mesh import normal_cone_angle
    first, second = np.array([1., 0, .1]), np.array([0., 1, .1])
    first /= np.linalg.norm(first)
    second /= np.linalg.norm(second)
    assert normal_cone_angle(first+second, first, second) < 1e-5
    assert normal_cone_angle(first, first, second) < 1e-5
    assert normal_cone_angle([0, 0, 1], first, second) > 50


def test_joint_fillets_preserve_aperture_depth_and_cross_section_area():
    """Round the lip without narrowing the joint or inventing a reduced leak area."""
    cfg = config()
    cfg['grooves'] = cfg['grooves'][:1]
    sharp = WallSurface(cfg)
    cfg['grooves'][0]['rim_radius_m'] = .0001
    rounded = WallSurface(cfg)
    offsets = np.linspace(-.0026, .0026, 20001)
    for along in [.1, .3, .7]:
        plain_depth = sharp.groove_depth(along, .4+offsets)
        round_depth = rounded.groove_depth(along, .4+offsets)
        assert np.max(round_depth) == pytest.approx(np.max(plain_depth))
        assert np.all(round_depth[np.abs(offsets) >= .0025] == 0)
        assert np.all(round_depth[np.abs(offsets) < .0025] > 0)
        assert abs(np.trapz(round_depth-plain_depth, offsets)) < 1e-10
        eps = 1e-8
        slope_at_rim = rounded.groove_depth(along, .3975+eps)/eps
        assert slope_at_rim < .001


@pytest.mark.parametrize('radius', [.00025, -.0001, float('nan')])
def test_joint_fillets_reject_radius_that_does_not_fit(radius):
    """A large fillet must not consume the declared sidewall or opening."""
    cfg = config()
    cfg['grooves'][0]['rim_radius_m'] = radius
    with pytest.raises(ValueError, match='radius'):
        WallSurface(cfg)


def test_native_joint_patches_do_not_cover_the_groove_with_old_support(tmp_path):
    """A local heightmap diagnostic must lower underlying support and preserve scale."""
    from xml.etree import ElementTree as ET
    from climbot_gazebo.surface_heightmap import add_panel_heightmaps
    from PIL import Image
    cfg = config()
    cfg['grooves'] = cfg['grooves'][:2]
    source = WallSurface(cfg)
    world = ET.Element('world')
    result = add_panel_heightmaps(world, tmp_path, source)
    assert result['samples'] == 257*257+3*129*129
    for name in ['vertical', 'intersection']:
        model = world.find(f"model[@name='surface_{name}']")
        shape = model.find('.//heightmap')
        span = [float(x) for x in shape.findtext('size').split()]
        lo = float(model.findtext('pose').split()[0])
        image = np.array(Image.open(shape.findtext('uri')))
        count = len(image)
        loaded_u_span = span[0]/count*(count-1)
        assert loaded_u_span == pytest.approx(.025)
        assert 0 <= image.min() < image.max() <= 65535
        height = lo+image[count//2, count//2]/65535*span[2]
        if name == 'vertical':
            assert height < float(source.height(.45, .4))-.005
        else:
            assert height == pytest.approx(float(source.height(.45, .4)), abs=1e-6)


def test_local_45_degree_chamfer_keeps_opening_depth_and_declares_section_change():
    """A short lip bevel changes its section explicitly, without widening the slot."""
    cfg = config()
    cfg['grooves'] = cfg['grooves'][:1]
    cfg['grooves'][0]['top_chamfer_m'] = .00025
    surface = WallSurface(cfg)
    u = .31
    assert surface.groove_depth(u, .3975+.000125) == pytest.approx(.000125, abs=1e-12)
    assert surface.groove_depth(u, .3975-.000001) == 0
    assert surface.groove_depth(u, .4025+.000001) == 0
    depth = float(surface.groove_depth(u, .4))
    assert .003 <= depth <= .005
    offsets = np.linspace(-.0025, .0025, 20001)
    actual_area = np.trapz(surface.groove_depth(u, .4+offsets), offsets)
    expected_area = depth*(.005-.0005)-.00025*(depth-.0005)
    assert actual_area == pytest.approx(expected_area, abs=1e-11)


@pytest.mark.parametrize('change', ['oversize', 'negative', 'rounded'])
def test_chamfer_rejects_undefined_or_overlapping_profiles(change):
    """Do not silently stack a fillet and bevel or consume the whole sidewall."""
    cfg = config()
    cfg['grooves'][0]['top_chamfer_m'] = .00025
    if change == 'oversize':
        cfg['grooves'][0]['top_chamfer_m'] = .0005
    elif change == 'negative':
        cfg['grooves'][0]['top_chamfer_m'] = -.0001
    else:
        cfg['grooves'][0]['rim_radius_m'] = .0001
    with pytest.raises(ValueError, match='chamfer'):
        WallSurface(cfg)


def test_equal_chamfer_faces_do_not_create_zero_area_crease_slivers(tmp_path):
    """A grid node on two equal 45-degree lips must remain one shared vertex."""
    from climbot_gazebo.surface_mesh import audit_mesh_sections, write_cross_joint_mesh
    cfg = config()
    cfg['grooves'] = cfg['grooves'][:2]
    for groove in cfg['grooves']:
        groove['top_chamfer_m'] = .00025
    source = WallSurface(cfg)
    path = tmp_path/'chamfer_cross.obj'
    result = write_cross_joint_mesh(path, source)
    assert result['projected_area_m2'] == pytest.approx(.64, abs=1e-10)
    assert result['triangles'] < 20000
    assert audit_mesh_sections(path, source)['passed']


def test_native_port_matches_frozen_pre_port_reference():
    """Independent pre-port samples include roughness, branches, fillets and bevels."""
    import json
    data = json.loads((Path(__file__).parent / 'data/wall_surface_pre_native.json').read_text())
    from _climbot_surface import WallSurface as NativeSurface
    for case in data['cases']:
        surface = WallSurface(case['config'])
        u, v = np.array(case['u']), np.array(case['v'])
        np.testing.assert_allclose(surface.height(u, v), case['height'], atol=2e-14, rtol=0)
        np.testing.assert_allclose(surface.groove_depth(u, v), case['depth'], atol=2e-14, rtol=0)
        # The serialized realization is directly consumable by runtime C++.
        restored = NativeSurface(surface.resolved_yaml)
        np.testing.assert_allclose(restored.evaluate(u, v, 0), case['height'], atol=2e-14)
        assert surface.height(.1, .2).shape == ()
        assert surface.height(u[:, None], v).shape == (len(u), len(v))


def test_diagnostic_heightmap_underlays_meet_on_shared_boundaries(tmp_path):
    """Reject the old 6--8 mm duplicate support band; shared rims encode identically."""
    from xml.etree import ElementTree as ET
    from climbot_gazebo.surface_heightmap import add_panel_heightmaps
    from PIL import Image
    cfg = config()
    cfg['grooves'] = cfg['grooves'][:2]
    world = ET.Element('world')
    report = add_panel_heightmaps(world, tmp_path, WallSurface(cfg))
    patches = {p['name']: p for p in report['patches']}
    base = np.flipud(np.asarray(Image.open(tmp_path/'base.png')))
    vertical = np.flipud(np.asarray(Image.open(tmp_path/'vertical.png')))
    horizontal = np.flipud(np.asarray(Image.open(tmp_path/'horizontal.png')))
    intersection = np.flipud(np.asarray(Image.open(tmp_path/'intersection.png')))
    u0, u1 = patches['vertical']['u_range_m']
    v0, v1 = patches['horizontal']['v_range_m']
    cols = np.rint(np.array([u0, u1])/(.8/256)).astype(int)
    # Keep samples away from the crossing, which is covered by the third patch.
    assert np.array_equal(base[:100:2, cols[0]], vertical[:50, 0])
    assert np.array_equal(base[:100:2, cols[1]], vertical[:50, -1])
    cross_rows = np.rint(np.array([v0, v1])/(.8/128)).astype(int)
    cross_cols = np.rint(np.array([u0, u1])/(.8/128)).astype(int)
    assert np.array_equal(vertical[cross_rows[0]], intersection[0])
    assert np.array_equal(vertical[cross_rows[1]], intersection[-1])
    assert np.array_equal(horizontal[:, cross_cols[0]], intersection[:, 0])
    assert np.array_equal(horizontal[:, cross_cols[1]], intersection[:, -1])
    # Former duplicate band (+/- 6--8 mm) is now strictly below the source.
    for offset in [-.00625, .00625]:
        column = round((.45+offset)/(.8/256))
        assert base[20, column] < base[20, cols[0]]-10000
