#!/usr/bin/env python3
"""Bounded T0-F gate: nonzero masks, then flat/single/cross at one fixed setting."""
import argparse
import csv
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import subprocess
import time
import uuid
from xml.etree import ElementTree as ET

from _climbot_surface import SphereEnvelope
from ament_index_python.packages import get_package_prefix
from climbot_gazebo.mesa_runtime import rendering_environment
from climbot_gazebo.ode_runtime import environment
from climbot_gazebo.wall_surface import WallSurface
import numpy as np
import yaml

from probe_m2_wall_surface import element as node

ROOT = Path(__file__).resolve().parents[1]
RADIUS, PROXY, LOAD, SPEED, STEP, SETTLE = .0125, .00625, 500., .2, .001, .4
CARRIAGE_KG = 5.
# Discrete-step allowances (contract revision 2026-10-19). A one-step contact miss lets the
# loaded carriage fall a*dt^2 under semi-implicit Euler, and a crease can only be reached
# to within one step of travel. Neither is a proxy-geometry error.
STEP_SINK_M = LOAD/CARRIAGE_KG*STEP**2
# Loaded normals: at most 5% of contact force beyond 5 deg, none beyond 10 deg.
NORMAL_GATE_DEG, NORMAL_MAX_DEG, NORMAL_OUTLIER_SHARE = 5., 10., .05
CREASE_WINDOW_M = max(.00001, SPEED*STEP)


def write(path, obj):
    path.write_text(json.dumps(obj, indent=2, allow_nan=False)+'\n')


def mask(collision, value):
    surface = node(collision, 'surface')
    node(node(surface, 'contact'), 'collide_bitmask', value)
    ode = node(node(surface, 'friction'), 'ode')
    node(ode, 'mu', 0)
    node(ode, 'mu2', 0)


def world():
    root = ET.Element('sdf', version='1.9')
    w = node(root, 'world', name='caster_proxy_gate')
    node(w, 'gravity', '-9.81 0 0')
    p = node(w, 'physics', name='physics', type='ignored')
    node(p, 'max_step_size', STEP)
    node(p, 'real_time_factor', 0)
    node(node(p, 'dart'), 'collision_detector', 'ode')
    for kind in ['physics', 'contact']:
        node(w, 'plugin', filename=f'gz-sim-{kind}-system',
             name='gz::sim::systems::'+kind.title())
    return root, w


def plane(w, name, height, bits):
    m = node(w, 'model', name=name)
    node(m, 'static', 'true')
    node(m, 'pose', f'{height-.025} 0 1.4 0 0 0')
    c = node(node(m, 'link', name='surface'), 'collision', name='collision')
    node(node(node(c, 'geometry'), 'box'), 'size', '.05 .8 .8')
    mask(c, bits)


def body(w, name, xyz, radius, bits, sweep=False, heading=0.):
    model = node(w, 'model', name=name)
    node(model, 'pose', ' '.join(map(str, [*xyz, 0, 0, 0])))
    for link_name, mass in ([('guide', .01), ('body', CARRIAGE_KG)] if sweep else [('body', .1)]):
        link = node(model, 'link', name=link_name)
        inertial = node(link, 'inertial')
        node(inertial, 'mass', mass)
        tensor = node(inertial, 'inertia')
        for axis in ['ixx', 'iyy', 'izz']:
            node(tensor, axis, .4*mass*radius**2)
        if link_name == 'body':
            c = node(link, 'collision', name='collision')
            node(node(node(c, 'geometry'), 'sphere'), 'radius', radius)
            mask(c, bits)
    joints = [('scan_slide', 'world', 'guide', f'0 {math.cos(heading)} {math.sin(heading)}'),
              ('normal_slide', 'guide', 'body', '1 0 0')] if sweep else [
                  ('normal_slide', 'world', 'body', '1 0 0')]
    for name, parent, child, direction in joints:
        j = node(model, 'joint', name=name, type='prismatic')
        node(j, 'parent', parent)
        node(j, 'child', child)
        axis = node(j, 'axis')
        node(axis, 'xyz', direction)
        limits = node(axis, 'limit')
        node(limits, 'lower', -.1)
        node(limits, 'upper', .1)
        node(limits, 'effort', 1e6)


def run(out, root, binary, env, request):
    out.mkdir()
    ET.ElementTree(root).write(out/'world.sdf')
    (out/'request.yaml').write_text(yaml.safe_dump(dict(world='world.sdf', **request)))
    child = dict(env, GZ_PARTITION='climbot_proxy_'+uuid.uuid4().hex)
    with (out/'native.log').open('w') as log:
        completed = subprocess.run([str(binary), str(out/'request.yaml'), str(out/'native')],
                                   env=child, stdout=log, stderr=subprocess.STDOUT, timeout=60)
    if completed.returncode:
        raise RuntimeError(f'native gate exit {completed.returncode}: {out}/native.log')
    observed = yaml.safe_load((out/'native/run.yaml').read_text())['ode_libraries']
    expected = str((Path(env['LD_LIBRARY_PATH'].split(':')[0])/'libode.so.8').resolve())
    if observed != [expected]:
        raise RuntimeError(f'wrong native ODE mapping: {observed}')
    with (out/'native/frames.csv').open() as stream:
        frames = list(csv.DictReader(stream))
    with (out/'native/contacts.csv').open() as stream:
        contacts = list(csv.DictReader(stream))
    return frames, contacts


def masks_gate(out, binary, env):
    root, w = world()
    plane(w, 'real_wall', 0, 1)
    plane(w, 'proxy_wall', RADIUS-PROXY, 2)
    specs = [('probe_00', PROXY, 2, 'proxy_wall', RADIUS),
             ('probe_01', RADIUS, 1, 'real_wall', RADIUS),
             ('probe_02', .002, 1, 'real_wall', .002)]
    for i, (name, radius, bits, _, _) in enumerate(specs):
        body(w, name, [.013, -.2+i*.2, 1.4], radius, bits)
    frames, contacts = run(out, root, binary, env, {'iterations': 400})
    checks = {}
    for name, _, _, wall, expected in specs:
        f = [r for r in frames if r['name'] == name and float(r['time_s']) > .3]
        c = [r for r in contacts if r['name'] == name and float(r['time_s']) > .3]
        checks[name] = bool(f and c and all(int(r['contacts']) > 0 and
                                           abs(float(r['x'])-expected) < .00001 for r in f) and
                            all(wall in r['collision1_name']+' '+r['collision2_name'] and
                                ('proxy_wall' if wall == 'real_wall' else 'real_wall') not in
                                r['collision1_name']+' '+r['collision2_name'] for r in c))
    return {'passed': all(checks.values()), 'checks': checks,
            'scope': 'Nonzero bits 1 and 2: real wheel and collapsed-stop analogue exclude proxy; proxy ball excludes real wall'}


def axes(center, across_joint):
    """
    Whole-wall budget: 10 mm along joints and on panels, 0.1 mm only over the opening.

    0.2 mm across failed the loaded-normal gate; along-joint spacing is set by roughness.
    """
    values = np.linspace(center-.02, center+.02, 5)
    if across_joint:
        values = np.r_[values, center+np.linspace(-.0026, .0026, 53),
                       center+np.array([-.0025, 0, .0025])]
    return np.unique(np.round(values, 12))


def source(config, kind):
    config = deepcopy(config)
    config['roughness']['amplitude_m'] = 0
    config['grooves'] = config['grooves'][:(2 if kind == 'cross' else 1)]
    if kind == 'flat':
        config['grooves'] = []
    return WallSurface(config)


def analytic(u, v, radius, kind):
    """Independent ideal deep-strip reference; not used to construct the mesh."""
    dv = np.maximum(0, .0025-np.abs(v-.4)) if kind != 'flat' else 0.
    du = np.maximum(0, .0025-np.abs(u-.45)) if kind == 'cross' else 0.
    return np.sqrt(radius**2-du**2-dv**2)


def trajectory_normal_angle(frame, contact, kind):
    """Angle to E's one-sided normal cone within the frozen 10um geometry budget."""
    u, v = float(frame['before_y'])+.4, float(frame['before_z'])-1
    h = float(analytic(u, v, RADIUS, kind))
    limits = []
    for value, center, active in [(u, .45, kind == 'cross'), (v, .4, kind != 'flat')]:
        offset = value-center
        gradient = max(0, .0025-abs(offset))/h if active else 0.
        if active and abs(offset) <= CREASE_WINDOW_M:
            limits.append((-gradient, gradient))
        else:
            slope = -math.copysign(gradient, offset)
            limits.append((slope, slope))
    normal = np.array([float(contact['n'+k]) for k in 'xyz'])
    if normal[0] < 0:
        normal = -normal
    # Section of the cone at nx=1 is an interval or rectangle. Projection here
    # gives a conservative angle upper bound (and zero for every cone member).
    if normal[0] <= 0:
        return 90.
    q = [1., *[float(np.clip(normal[i+1]/normal[0], *bounds))
                for i, bounds in enumerate(limits)]]
    return math.degrees(math.acos(np.clip(np.dot(normal, q)/(
        np.linalg.norm(normal)*np.linalg.norm(q)), 0, 1)))


def mesh(path, u_axis, v_axis, heights):
    u, v = np.meshgrid(u_axis, v_axis)
    vertices = np.column_stack([u.ravel()-.45, v.ravel()-.4, heights.ravel()])
    faces = []
    n = len(u_axis)
    for j in range(len(v_axis)-1):
        for i in range(n-1):
            a = j*n+i
            faces.extend([(a, a+1, a+n+1), (a, a+n+1, a+n)])
    faces = np.asarray(faces)
    normals = np.zeros_like(vertices)
    face_normals = np.cross(vertices[faces[:, 1]]-vertices[faces[:, 0]],
                            vertices[faces[:, 2]]-vertices[faces[:, 0]])
    for corner in range(3):
        np.add.at(normals, faces[:, corner], face_normals)
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
    with path.open('w') as f:
        for p in vertices:
            f.write('v '+' '.join(f'{x:.17g}' for x in p)+'\n')
        for p in normals:
            f.write('vn '+' '.join(f'{x:.17g}' for x in p)+'\n')
        for p in faces:
            f.write('f '+' '.join(f'{x+1}//{x+1}' for x in p)+'\n')
    return vertices, faces


def sweep(out, config, kind, binary, env):
    out.mkdir()
    surface = source(config, kind)
    (out/'surface_resolved.yaml').write_text(surface.resolved_yaml)
    # Horizontal joint at v=.4 (single and cross); vertical joint at u=.45 (cross only).
    pu, pv = axes(.45, kind == 'cross'), axes(.4, kind != 'flat')
    su = np.unique(np.r_[np.linspace(.405, .495, 91), pu])
    sv = np.unique(np.r_[np.linspace(.355, .445, 91), pv])
    u, v = np.meshgrid(su, sv)
    envelope = SphereEnvelope(su, sv, surface.height(u, v))
    u, v = np.meshgrid(pu, pv)
    start = time.monotonic()
    heights = envelope.evaluate(u, v, RADIUS-PROXY)
    vertices, faces = mesh(out/'proxy.obj', pu, pv, heights)
    geometric_error = float(np.max(np.abs(heights-analytic(u, v, RADIUS-PROXY, kind))))
    triangle = vertices[faces]
    center = triangle.mean(axis=1)
    interpolation = np.abs(center[:, 2]-analytic(center[:, 0]+.45, center[:, 1]+.4,
                                                RADIUS-PROXY, kind))
    geometry = {'vertices': len(vertices), 'triangles': len(faces),
                'vertex_error_m': geometric_error, 'centroid_error_m': float(interpolation.max()),
                'generation_seconds': time.monotonic()-start}
    if max(geometric_error, geometry['centroid_error_m']) > .00001:
        write(out/'geometry.json', geometry)
        return {'passed': False, 'geometry': geometry, 'reason': 'offline geometry gate failed'}
    root, w = world()
    plane(w, 'real_wall_masked_control', 0, 1)
    model = node(w, 'model', name='proxy_wall')
    node(model, 'static', 'true')
    node(model, 'pose', f'0 .05 1.4 {math.pi/2} 0 {math.pi/2}')
    c = node(node(model, 'link', name='surface'), 'collision', name='collision')
    node(node(node(c, 'geometry'), 'mesh'), 'uri', str((out/'proxy.obj').resolve()))
    mask(c, 2)
    heading = math.pi/4 if kind == 'cross' else math.pi/2
    start_uv = np.array([.45, .4])-.016*np.array([math.cos(heading), math.sin(heading)])
    initial = float(analytic(*start_uv, RADIUS, kind))+.00005
    body(w, 'probe_00', [initial, start_uv[0]-.4, start_uv[1]+1], PROXY, 2, True, heading)
    frames, contacts = run(out/'run', root, binary, env, {'iterations': round((SETTLE+.032/SPEED)/STEP),
        'sweep': {'speed_mps': SPEED, 'settle_s': SETTLE,
                  'extra_normal_force_n': LOAD-5*9.81}})
    return evaluate_saved(out, kind, geometry, frames, contacts)


def evaluate_saved(out, kind, geometry, frames=None, contacts=None):
    """Re-evaluate native records offline; never re-run physics to fix reporting."""
    if frames is None:
        with (out/'run/native/frames.csv').open() as f:
            frames = list(csv.DictReader(f))
        with (out/'run/native/contacts.csv').open() as f:
            contacts = list(csv.DictReader(f))
    frames = [r for r in frames if float(r['time_s']) >= SETTLE]
    contacts = [r for r in contacts if float(r['time_s']) >= SETTLE]
    by_time = {float(r['time_s']): r for r in frames}
    errors = [float(r['x'])-float(analytic(float(r['y'])+.4, float(r['z'])-1,
                                         RADIUS, kind)) for r in frames]
    gap = max_gap = 0
    for r in frames:
        gap = gap+1 if int(r['contacts']) == 0 else 0
        max_gap = max(gap, max_gap)
    angles = []
    trajectory_angles = []
    trajectory_forces = []
    all_angles = []
    owned = True
    force_tuples = 0
    for r in contacts:
        owned &= 'proxy_wall' in r['collision1_name']+' '+r['collision2_name']
        f = by_time.get(float(r['time_s']))
        if not f:
            continue
        radial = np.array([float(f['before_'+k])-float(r[k]) for k in 'xyz'])
        normal = np.array([float(r['n'+k]) for k in 'xyz'])
        angle = math.degrees(math.acos(np.clip(abs(radial@normal)/(
            np.linalg.norm(radial)*np.linalg.norm(normal)), 0, 1)))
        all_angles.append(angle)
        force = np.linalg.norm([float(r['force_'+k+'_n']) for k in 'xyz'])
        if force > .01:
            angles.append(angle)
            trajectory_angles.append(trajectory_normal_angle(f, r, kind))
            trajectory_forces.append(force)
            force_tuples += 1
    displacement = math.hypot(float(frames[-1]['y'])-float(frames[0]['y']),
                              float(frames[-1]['z'])-float(frames[0]['z']))
    total = sum(trajectory_forces)
    outlier_share = (sum(f for a, f in zip(trajectory_angles, trajectory_forces)
                         if a > NORMAL_GATE_DEG)/total if total else 1.)
    result = {'geometry': geometry, 'support_error_min_m': min(errors),
              'loaded_normal_outlier_force_share': float(outlier_share),
              'support_error_max_m': max(errors), 'contact_gap_max_steps': max_gap,
              'loaded_radial_diagnostic_max_deg': max(angles) if angles else None,
              'loaded_trajectory_normal_max_deg': max(trajectory_angles) if trajectory_angles else None,
              'all_tuple_normal_max_deg': max(all_angles) if all_angles else None,
              'force_bearing_tuples': force_tuples, 'travel_m': displacement}
    result['checks'] = {'support': max(map(abs, errors)) <= .00005+STEP_SINK_M,
                        'contact_gap': max_gap <= 2, 'ownership': owned,
                        'loaded_trajectory_normals': bool(trajectory_angles) and bool(
                            max(trajectory_angles) <= NORMAL_MAX_DEG and
                            outlier_share <= NORMAL_OUTLIER_SHARE),
                        'travel': .0305 <= displacement <= .0325}
    result['passed'] = all(result['checks'].values())
    write(out/'summary.json', result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--ode-prefix', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    binary = Path(get_package_prefix('climbot_gazebo'))/'lib/climbot_gazebo/wall_surface_probe'
    env, mesa = rendering_environment()
    env, ode = environment(args.ode_prefix, env)
    report = {'passed': False, 'scope': 'Planar source panels; one diagnostic setting, no full vehicle acceptance',
              'contract': {'normal_load_n': LOAD, 'speed_mps': SPEED, 'step_s': STEP,
                           'support_gate_m': .00005+STEP_SINK_M,
                           'step_sink_allowance_m': STEP_SINK_M,
                           'crease_window_m': CREASE_WINDOW_M,
                           'loaded_normal_gate': {'deg': NORMAL_GATE_DEG, 'max_deg': NORMAL_MAX_DEG,
                                                  'outlier_force_share': NORMAL_OUTLIER_SHARE},
                           'real_radius_m': RADIUS, 'proxy_radius_m': PROXY,
                           'force_bearing_threshold_n': .01,
                           'normal_reference': 'E one-sided normals; 10um crease-location budget, radial error diagnostic only'},
              'mesa': mesa, 'ode': ode, 'cases': {},
              'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest()}
    report['masks'] = masks_gate(args.output/'masks', binary, env)
    write(args.output/'summary.json', report)
    if not report['masks']['passed']:
        print('Nonzero mask gate failed; stopped before constructing proxy meshes')
        return 1
    config = yaml.safe_load((ROOT/'src/climbot_gazebo/config/m2_surface_fixture.yaml').read_text())
    for kind in ['flat', 'single', 'cross']:
        report['cases'][kind] = sweep(args.output/kind, config, kind, binary, env)
        write(args.output/'summary.json', report)
        if not report['cases'][kind]['passed']:
            print(json.dumps({'stopped_at': kind, 'result': report['cases'][kind]}))
            return 1
    report['passed'] = True
    write(args.output/'summary.json', report)
    print(json.dumps({'passed': True, 'cases': report['cases']}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
