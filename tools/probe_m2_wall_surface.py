#!/usr/bin/env python3
"""T0-A: independently observe native vertical heightmap geometry.

This isolates smooth roughness only, not wheel crossing or skirt sealing.
Raw fixtures and native time series belong outside Git. No renderer is loaded.
"""
import argparse
from copy import deepcopy
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time
import uuid
from xml.etree import ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from climbot_gazebo.ode_runtime import environment as ode_environment
from climbot_gazebo.surface_mesh import (
    audit_mesh_sections, normal_cone_angle, write_cross_joint_mesh, write_feature_mesh)
from climbot_gazebo.wall_surface import WallSurface, write_tensor_mesh
import numpy as np
from PIL import Image
import yaml


def element(parent, tag, text=None, **attributes):
    node = ET.SubElement(parent, tag, **attributes)
    if text is not None:
        node.text = str(text)
    return node


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def make_fixture(out, config, samples, step, orientation, representation='heightmap',
                 boundary_probes=False):
    """Place h along world X, u along Y, v along Z; no heightmap <pos> offset."""
    if (type(samples) is not int or samples < 3
            or (samples - 1) & (samples - 2)):
        raise ValueError('heightmap sample count must be 2^n+1')
    if representation not in ('heightmap', 'joint_mesh', 'cross_mesh', 'crack_mesh',
                              'bridged_crack_heightmap'):
        raise ValueError('unknown collision representation')
    if representation == 'cross_mesh' and [g['id'] for g in config['grooves']] != [
            'horizontal_joint', 'vertical_joint']:
        raise ValueError('cross_mesh requires the horizontal and vertical joint pair')
    if representation == 'crack_mesh' and [g['id'] for g in config['grooves']] != [
            'main_crack', 'branch_crack']:
        raise ValueError('crack_mesh requires the declared main and branch crack pair')
    if representation == 'joint_mesh':
        grooves = config['grooves']
        if (len(grooves) != 1 or grooves[0].get('cross_section') != 'trapezoid'
                or not np.allclose([grooves[0]['start_m'][1], grooves[0]['end_m'][1]],
                                   [.4, .4], rtol=0, atol=1e-12)
                or grooves[0]['start_m'][0] > 0 or grooves[0]['end_m'][0] < .8
                or grooves[0]['width_m'] != .005 or grooves[0]['shoulder_fraction'] != .1
                or grooves[0]['depth_range_m'] != [.003, .005]):
            raise ValueError('joint_mesh requires the declared horizontal 5 mm / 3-5 mm fixture')
    start = time.monotonic()
    out.mkdir()
    surface = WallSurface(config)
    if representation == 'bridged_crack_heightmap':
        surface = surface.contact_view('bridge_narrow_cracks')
        if surface.config['grooves'] or not surface.config['omitted_cracks']:
            raise ValueError('bridged crack fixture requires only explicitly classified cracks')
    mesh = representation in ('joint_mesh', 'cross_mesh', 'crack_mesh')
    features = representation in ('cross_mesh', 'crack_mesh')
    size, radius = .8, (.0001 if mesh else .0025)
    mass = .0001 if mesh else .1
    initial_gap = .000001 if mesh else .00005
    if representation == 'bridged_crack_heightmap':
        radius, mass, initial_gap = .0001, .0001, .000001
    if boundary_probes:
        radius, mass, initial_gap = .00005, .0001, .000001
    axis = np.linspace(0, size, samples)
    u, v = np.meshgrid(axis, axis)
    heights = surface.roughness(u, v)
    low, high = float(heights.min()), float(heights.max())
    if high - low < 1e-10:
        low, high = 0., .001
    encoded = np.rint((heights - low) / (high - low) * 65535).astype(np.uint16)
    mesh_info = None
    if features:
        mesh_info = (write_cross_joint_mesh(out / 'surface.obj', surface)
                     if representation == 'cross_mesh' else
                     write_feature_mesh(out / 'surface.obj', surface))
        mesh_info['section_audit'] = audit_mesh_sections(out / 'surface.obj', surface)
        if not mesh_info['section_audit']['passed']:
            raise ValueError('independent mesh sections failed; do not run native acceptance')
    elif mesh:
        mesh_info = write_tensor_mesh(
            out / 'surface.obj', surface, np.linspace(0, size, 81),
            np.unique(np.r_[np.linspace(0, size, 81),
                            np.linspace(.3975, .398, 5), np.linspace(.402, .4025, 5)]))
    else:
        Image.fromarray(np.flipud(encoded)).save(out / 'height.png')
    root = ET.Element('sdf', version='1.9')
    world = element(root, 'world', name='m2_surface_probe')
    # Diagnostic normal preload only; these are independent gauges, not the robot.
    vertical = orientation == 'vertical'
    element(world, 'gravity', '-9.81 0 0' if vertical else '0 0 -9.81')
    physics = element(world, 'physics', name='physics', type='ignored')
    element(physics, 'max_step_size', step)
    element(physics, 'real_time_factor', 0)
    element(element(physics, 'dart'), 'collision_detector', 'ode')
    element(world, 'plugin', filename='gz-sim-physics-system', name='gz::sim::systems::Physics')
    element(world, 'plugin', filename='gz-sim-contact-system', name='gz::sim::systems::Contact')
    model = element(world, 'model', name='surface')
    element(model, 'static', 'true')
    placement = 0. if mesh else low
    pose = (f'{placement} 0 1.4 {math.pi / 2} 0 {math.pi / 2}' if vertical
            else f'0 0 {placement} 0 0 0')
    element(model, 'pose', pose)
    geometry = element(element(element(model, 'link', name='surface'),
                               'collision', name='surface_collision'), 'geometry')
    if mesh:
        element(element(geometry, 'mesh'), 'uri', str((out / 'surface.obj').resolve()))
    elif config['roughness']['amplitude_m'] == 0:
        # A real finite flat control, not an ambiguous constant-intensity heightmap.
        element(model.find('link/collision'), 'pose', '0 0 -.025 0 0 0')
        element(element(geometry, 'box'), 'size', f'{size} {size} .05')
    else:
        shape = element(geometry, 'heightmap')
        element(shape, 'uri', str((out / 'height.png').resolve()))
        # gz-physics divides size by N; ODE spans N-1 sample intervals.
        # Compensate in SDF, keeping source coordinates and image unchanged.
        sdf_span = size * samples / (samples - 1)
        element(shape, 'size', f'{sdf_span:.17g} {sdf_span:.17g} {high - low:.17g}')
        element(shape, 'pos', '0 0 0')
    probes = []
    locations = [(x, y) for x in [.025, .19, .43, .775] for y in [.025, .27, .59, .775]]
    if mesh:
        locations = [(x, y) for x in [.15, .45, .7]
                     for y in [.396, .39775, .4, .40225, .404]]
    if representation == 'cross_mesh':
        locations = [(x, y) for x in [.2, .7]
                     for y in [.396, .39775, .4, .40225, .404]]
        locations += [(x, y) for y in [.2, .65]
                      for x in [.446, .44775, .45, .45225, .454]]
        locations += [(.45, .4), (.449, .399), (.451, .401),
                      (.44775, .39775), (.45225, .40225)]
    if representation in ('crack_mesh', 'bridged_crack_heightmap'):
        locations = []
        for groove in config['grooves']:
            a, b = np.array(groove['start_m']), np.array(groove['end_m'])
            direction = (b-a)/np.linalg.norm(b-a)
            across = np.array([-direction[1], direction[0]])
            for fraction in [.2, .5, .8]:
                center = a + fraction*(b-a)
                for offset in [-.6, -.4, 0, .4, .6]:
                    locations.append(tuple(center + across * offset * groove['width_m']))
            locations.extend([tuple(a), tuple(a-.001*direction), tuple(b),
                              tuple(b+.001*direction)])
        if representation == 'bridged_crack_heightmap':
            locations = [tuple(np.array(g['start_m']) + fraction * (
                np.array(g['end_m'])-g['start_m'])) for g in config['grooves']
                for fraction in [0, .25, .5, .75, 1]]
    locations += [(-.025, .3), (.825, .3), (.3, -.025), (.3, .825)]
    if boundary_probes:
        # Probe both sides of each intended edge, independent of the image grid.
        # 50 um spheres at +/-150 um bound support placement to 0.2 mm.
        locations = []
        for along in [.08, .4, .72]:
            for offset in [.00015, -.00015]:
                locations.extend([(offset, along), (size - offset, along),
                                  (along, offset), (along, size - offset)])
    locations = list(dict.fromkeys((round(u, 12), round(v, 12)) for u, v in locations))
    centers = []
    for index, (u, v) in enumerate(locations):
        outside = not 0 <= u <= size or not 0 <= v <= size
        probes.append({'name': f'probe_{index:02}', 'u_m': u, 'v_m': v,
                       'outside': outside})
        model = element(world, 'model', name=f'probe_{index:02}')
        support = radius / float(surface.normal(u, v)[2]) if mesh or boundary_probes else radius
        initial = float(surface.height(u, v)) + support + initial_gap
        probes[-1]['initial_height_m'] = initial
        xyz = (initial, u - size / 2, 1 + v) if vertical else (
            u - size / 2, v - size / 2, initial)
        if any(np.linalg.norm(np.array(xyz)-center) < 2*radius+1e-8 for center in centers):
            raise ValueError('overlapping geometry gauges; use independent positions or cases')
        centers.append(np.array(xyz))
        element(model, 'pose', ' '.join(map(str, (*xyz, 0, 0, 0))))
        body = element(model, 'link', name='body')
        inertia = element(body, 'inertial')
        element(inertia, 'mass', mass)
        tensor = element(inertia, 'inertia')
        for tag in ['ixx', 'iyy', 'izz']:
            element(tensor, tag, .4 * mass * radius**2)
        sphere = element(element(element(body, 'collision', name='collision'),
                                 'geometry'), 'sphere')
        element(sphere, 'radius', radius)
        joint = element(model, 'joint', name='gauge_slide', type='prismatic')
        element(joint, 'parent', 'world')
        element(joint, 'child', 'body')
        axis_node = element(joint, 'axis')
        element(axis_node, 'xyz', '1 0 0' if vertical else '0 0 1')
        limit = element(axis_node, 'limit')
        element(limit, 'lower', -.8)
        element(limit, 'upper', .8)
        element(element(axis_node, 'dynamics'), 'damping', .02)
    # Geometry gauges isolate normal contact. Tangential friction belongs in the
    # later wheel tests; mixing it with the constrained gauges masks penetration.
    for collision in world.iter('collision'):
        ode = element(element(element(collision, 'surface'), 'friction'), 'ode')
        element(ode, 'mu', 0)
        element(ode, 'mu2', 0)
    ET.ElementTree(root).write(out / 'world.sdf', encoding='unicode')
    (out / 'source.yaml').write_text(yaml.safe_dump(config))
    request = {'world': 'world.sdf', 'iterations': round(.35 / step)}
    (out / 'request.yaml').write_text(yaml.safe_dump(request))
    return {'samples': samples, 'step_s': step, 'orientation': orientation,
            'cell_size_m': size / (samples - 1), 'probes': probes,
            'source_height_range_m': [low, high],
            'scope': representation if features or representation == 'bridged_crack_heightmap' else (
                'one_horizontal_joint' if mesh else 'smooth_roughness_only'),
            'representation': representation, 'mesh': mesh_info,
            'contact_config': surface.config,
            'boundary_probes': boundary_probes,
            'boundary_support_tolerance_m': .0002 if boundary_probes else None,
            'height_tolerance_m': .0001 if representation == 'crack_mesh' else .0002,
            'generation_seconds': time.monotonic() - start,
            'gauge_mass_kg': mass, 'gauge_friction': 0., 'initial_gap_m': initial_gap,
            'joint_damping_ns_m': .02,
            'incident_speed_bound_mps': math.sqrt(2 * 9.81 * initial_gap),
            'request': request, 'radius_m': radius}


def evaluate(out, config, fixture):
    surface = WallSurface(fixture.get('contact_config', config))
    frames = rows(out / 'native/frames.csv')
    contacts = rows(out / 'native/contacts.csv')
    vertical = fixture['orientation'] == 'vertical'
    errors, angles, depths, minima = [], [], [], []
    raw_angles, crease_angles = [], []
    crease_sources = ([WallSurface(dict(surface.config, grooves=[g]))
                       for g in surface.config['grooves']]
                      if fixture.get('representation') == 'cross_mesh' else [])
    assertions = {}
    assertions['only_surface_contacts'] = all(
        ((r.get('collision1_name', '').startswith('probe_') and
          r.get('collision2_name') == 'surface::surface::surface_collision') or
         (r.get('collision2_name', '').startswith('probe_') and
          r.get('collision1_name') == 'surface::surface::surface_collision'))
        for r in contacts)
    for probe in fixture['probes']:
        name = probe['name']
        steady = [r for r in frames if r['name'] == name and float(r['time_s']) >= .25]
        points = [r for r in contacts if r['name'] == name and float(r['time_s']) >= .25]
        if probe['outside']:
            coord = 'x' if vertical else 'z'
            initial = probe['initial_height_m']
            tau = fixture['gauge_mass_kg'] / fixture['joint_damping_ns_m']

            def free_position(time_s):
                return initial - 9.81 * tau * (time_s - tau * (1 - math.exp(-time_s / tau)))
            assertions[name + '_outside'] = (
                len(steady) >= 99 and not points and
                all(int(r['contacts']) == 0 and abs(float(r[coord]) - free_position(
                    float(r['time_s']))) <= .0001 + .02 * abs(initial - free_position(
                        float(r['time_s']))) for r in steady))
        else:
            assertions[name + '_continuous_contact'] = (
                len(steady) >= 99 and all(int(r['contacts']) > 0 for r in steady))
            for point in points:
                x, y, z = [float(point[k]) for k in ['x', 'y', 'z']]
                u, v, height = (y + .4, z - 1, x) if vertical else (x + .4, y + .4, z)
                errors.append(abs(float(surface.height(u, v)) - height))
                # Source derivative independent of image / collision interpolation.
                n = surface.normal(u, v)
                expected = np.array([n[2], n[0], n[1]]) if vertical else n
                measured = np.array([float(point[k]) for k in ['nx', 'ny', 'nz']])
                cosine = abs(float(np.dot(expected, measured))) / np.linalg.norm(measured)
                angle = math.degrees(math.acos(np.clip(cosine, 0, 1)))
                raw_angles.append(angle)
                if crease_sources:
                    values = [float(s.groove_depth(u, v)) for s in crease_sources]
                    normals = [s.normal(u, v) for s in crease_sources]
                    gradient = np.linalg.norm(normals[0][:2]/normals[0][2] -
                                              normals[1][:2]/normals[1][2])
                    # Geometric distance to the positive-depth equality crease;
                    # depth difference alone depends on the steep wall slope.
                    if (min(values) > 0 and gradient > 1e-6 and
                            abs(values[0]-values[1])/gradient <= 2e-8):
                        local = measured[[1, 2, 0]] if vertical else measured.copy()
                        if local[2] < 0:
                            local = -local
                        angle = normal_cone_angle(local, *normals)
                        crease_angles.append(angle)
                angles.append(angle)
                depths.append(float(point['depth_m']))
            if steady:
                center = [float(steady[-1][k]) for k in ['x', 'y', 'z']]
                source = float(surface.height(probe['u_m'], probe['v_m']))
                minima.append(abs(center[0 if vertical else 2] - fixture['radius_m'] - source))
    assertions['has_independent_contacts'] = bool(errors)
    assertions['max_height_error'] = bool(errors) and max(errors) <= fixture.get(
        'height_tolerance_m', .0002)
    assertions['normal_error'] = bool(angles) and max(angles) <= 2
    assertions['penetration'] = bool(depths) and max(depths) <= .0002
    return {'passed': all(assertions.values()), 'assertions': assertions,
            'height_error_max_m': max(errors) if errors else None,
            'normal_error_max_deg': max(angles) if angles else None,
            'normal_raw_single_derivative_max_deg': max(raw_angles) if raw_angles else None,
            'physical_crease_contacts': len(crease_angles),
            'physical_crease_cone_error_max_deg': max(crease_angles) if crease_angles else None,
            'penetration_max_m': max(depths) if depths else None,
            'center_minus_radius_error_max_m': max(minima) if minima else None,
            'native_run': yaml.safe_load((out / 'native/run.yaml').read_text())}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--joint-mesh', action='store_true',
                        help='include the first real horizontal joint fixture, no other grooves')
    parser.add_argument('--boundary-only', action='store_true',
                        help='only finite-support checks near all four heightmap edges')
    parser.add_argument('--features-only', action='store_true',
                        help='only horizontal/vertical joint intersection and branched crack meshes')
    parser.add_argument('--bridged-cracks-only', action='store_true',
                        help='explicit wheel-contact approximation; visual/leak source keeps cracks')
    parser.add_argument('--ode-prefix', type=Path,
                        help='validated private ODE install, M2 probe only')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output already exists; preserve evidence')
    if sum([args.boundary_only, args.joint_mesh, args.features_only,
            args.bridged_cracks_only]) > 1:
        parser.error('select only one optional fixture set')
    config_path = args.config or Path(get_package_share_directory(
        'climbot_gazebo')) / 'config/m2_surface_fixture.yaml'
    config = yaml.safe_load(config_path.read_text())
    # Do not silently claim grooves were tested by a smooth-heightmap experiment.
    smooth = deepcopy(config)
    smooth['grooves'] = []
    args.output.mkdir(parents=True)
    prefix = Path(get_package_prefix('climbot_gazebo'))
    binary = prefix / 'lib/climbot_gazebo/wall_surface_probe'
    env = dict(os.environ, GZ_PARTITION='climbot_t0_' + uuid.uuid4().hex)
    ode = None
    if args.ode_prefix:
        env, ode = ode_environment(args.ode_prefix, env)
    summary = {'ode_runtime': ode,
               'scope': 'surface gauges with explicit contact views; no robot, wheel or sealing acceptance',
               'source_config_sha256': hashlib.sha256(config_path.read_bytes()).hexdigest(),
               'git_commit': subprocess.check_output(
                   ['git', 'rev-parse', 'HEAD'], text=True).strip(),
               'source_modified': bool(subprocess.check_output(
                   ['git', 'status', '--porcelain'], text=True)), 'cases': {}}
    root = Path(__file__).resolve().parents[1]
    sources = [Path(__file__).resolve(), root / 'src/climbot_gazebo/src/wall_surface_probe.cpp',
               root / 'src/climbot_gazebo/climbot_gazebo/wall_surface.py',
               root / 'src/climbot_gazebo/climbot_gazebo/surface_mesh.py']
    summary['source_sha256'] = {
        str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sources}
    summary['binary_sha256'] = hashlib.sha256(binary.read_bytes()).hexdigest()
    cases = [
            ('flat_vertical_box_control', 0., 'vertical', 257, .001),
            ('rough_horizontal_control', .002, 'horizontal', 257, .001),
            ('rough_vertical', .002, 'vertical', 257, .001),
            ('rough_vertical_half_step', .002, 'vertical', 257, .0005)]
    if args.joint_mesh:
        cases.append(('vertical_joint_mesh', .002, 'vertical', 257, .001))
    if args.boundary_only:
        cases = [('boundary_flat_control', 0., 'vertical', 257, .001),
                 ('boundary_vertical_129', .002, 'vertical', 129, .001),
                 ('boundary_vertical_257', .002, 'vertical', 257, .001),
                 ('boundary_horizontal_257', .002, 'horizontal', 257, .001)]
    if args.features_only:
        cases = [('cross_mesh', .002, 'vertical', 257, .001),
                 ('crack_mesh', .002, 'vertical', 257, .001)]
    if args.bridged_cracks_only:
        cases = [('bridged_crack_heightmap', .002, 'vertical', 257, .001)]
    for name, amplitude, orientation, samples, step in cases:
        case_config = deepcopy(smooth)
        case_config['roughness']['amplitude_m'] = amplitude
        is_mesh = name == 'vertical_joint_mesh'
        if is_mesh:
            case_config['grooves'] = config['grooves'][:1]
        if args.features_only:
            case_config['grooves'] = config['grooves'][:2] if name == 'cross_mesh' else (
                config['grooves'][2:])
        if args.bridged_cracks_only:
            case_config['grooves'] = config['grooves'][2:]
        out = args.output / name
        fixture = make_fixture(out, case_config, samples, step, orientation,
                               representation=name if args.features_only or
                               args.bridged_cracks_only else (
                                   'joint_mesh' if is_mesh else 'heightmap'),
                               boundary_probes=args.boundary_only)
        fixture['world_sha256'] = hashlib.sha256((out / 'world.sdf').read_bytes()).hexdigest()
        with (out / 'native.log').open('w') as stream:
            result = subprocess.run([str(binary), str(out / 'request.yaml'), str(out / 'native')],
                                    stdout=stream, stderr=subprocess.STDOUT, env=env, timeout=40)
        if result.returncode:
            raise RuntimeError(f'native probe failed: {out / "native.log"}')
        evaluation = evaluate(out, case_config, fixture)
        if ode:
            loaded = evaluation['native_run']['ode_libraries']
            if len(loaded) != 1 or Path(loaded[0]).resolve() != Path(ode['library']):
                raise RuntimeError('native observer did not load the selected private ODE')
        summary['cases'][name] = {'fixture': fixture, 'evaluation': evaluation}
    summary['passed'] = all(case['evaluation']['passed'] for case in summary['cases'].values())
    summary['vertical_feasibility_passed'] = all(
        case['evaluation']['passed'] for case in summary['cases'].values()
        if case['fixture']['orientation'] == 'vertical')
    summary['t0_complete'] = False
    report = json.dumps(summary, indent=2, allow_nan=False) + '\n'
    (args.output / 'summary.json').write_text(report)
    print(json.dumps({'passed': summary['passed'], 'cases': {
        name: case['evaluation'] for name, case in summary['cases'].items()}}, indent=2))
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
