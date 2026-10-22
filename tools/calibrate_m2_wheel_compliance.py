#!/usr/bin/env python3
"""Calibrate one normal tyre joint with independently measured wheel contact load."""
import argparse
from copy import deepcopy
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import time
import uuid
from xml.etree import ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from climbot_description.vacuum_layout import build
from climbot_gazebo.mesa_runtime import rendering_environment
from climbot_gazebo.ode_runtime import environment
import numpy as np
from probe_m2_wall_surface import element as node
import yaml

ROOT = Path(__file__).resolve().parents[1]
TARGET, STEP, ZETA, AXLE_MASS, TRAVEL = .001, .001, .3, .02, .004
PROFILE = [(0, 0), (.1, 0), (.2, 50), (1., 50), (1.1, 150), (1.9, 150),
           (2., 300), (2.8, 300), (2.9, 150), (3.7, 150), (3.8, 0), (4.6, 0)]
WINDOWS = [('50_up', .7, .95, 50), ('150_up', 1.6, 1.85, 150),
           ('300_up', 2.5, 2.75, 300), ('150_down', 3.4, 3.65, 150),
           ('unloaded', 4.3, 4.55, 0)]
GATES = {'target_relative_error': .05, 'tail_peak_to_peak_m': .00001,
         'unloaded_residual_m': .00002, 'contact_load_error_n': 1.,
         'linearity_relative_error': .1, 'repeat_150_error_m': .00002,
         'stop_clearance_m': .0001, 'negative_compression_tolerance_m': .00001}


def write(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')


def mechanical_inputs():
    """Use the current shared wheel geometry, tensor, mass and body mass."""
    share = Path(get_package_share_directory('climbot_description'))
    cfg = yaml.safe_load((share / 'config/vacuum_chassis.yaml').read_text())['vacuum_chassis']
    camera = yaml.safe_load((share / 'config/inspection_camera.yaml').read_text())[
        'inspection_camera']
    robot = yaml.safe_load((share / 'config/robot.yaml').read_text())['robot']
    model = build(cfg, camera, robot)
    gazebo = Path(get_package_share_directory('climbot_gazebo'))
    simulation = yaml.safe_load((gazebo/'config/simulation.yaml').read_text())['simulation']
    wheel = deepcopy(next(p for p in model['parts'] if p['name'] == 'left_wheel_link'))
    return {'wheel': wheel, 'whole_mass_kg': model['mass_kg'],
            'wall_mu': simulation['wall']['mu'],
            'drive_wheel_mu': simulation['drive_wheel']['mu'],
            'sprung_mass_kg': (model['mass_kg'] - 2 * wheel['mass_kg']) / 2,
            'actuator_damping_nm_s': cfg['wheel']['joint_damping_nm_s']}


def fixture(inputs, stiffness, damping):
    """Normal-only free chassis, spring axle and freely rotating real cylinder."""
    root = ET.Element('sdf', version='1.9')
    w = node(root, 'world', name='tyre_calibration')
    node(w, 'gravity', '0 0 0')
    p = node(w, 'physics', name='physics', type='ignored')
    node(p, 'max_step_size', STEP)
    node(p, 'real_time_factor', 0)
    node(node(p, 'dart'), 'collision_detector', 'ode')
    for kind in ['physics', 'contact']:
        node(w, 'plugin', filename=f'gz-sim-{kind}-system',
             name='gz::sim::systems::' + kind.title())
    wall = node(w, 'model', name='wall')
    node(wall, 'static', 'true')
    node(wall, 'pose', '-.025 0 1 0 0 0')
    c = node(node(wall, 'link', name='wall'), 'collision', name='wall_collision')
    node(node(node(c, 'geometry'), 'box'), 'size', '.05 1 1')
    friction = node(node(node(c, 'surface'), 'friction'), 'ode')
    node(friction, 'mu', inputs['wall_mu'])
    node(friction, 'mu2', inputs['wall_mu'])
    model = node(w, 'model', name='probe_00')
    wheel = inputs['wheel']
    shape = wheel['shapes'][0]
    node(model, 'pose', f'{shape["radius"]} 0 1 0 0 0')
    axle_inertia = np.eye(3) * .4 * AXLE_MASS * .01**2
    for name, mass, tensor in [
            ('sprung', inputs['sprung_mass_kg'], np.eye(3) * .1),
            ('axle', AXLE_MASS, axle_inertia),
            ('body', wheel['mass_kg'] - AXLE_MASS,
             np.array(wheel['inertia_kg_m2']) - axle_inertia)]:
        link = node(model, 'link', name=name)
        inertial = node(link, 'inertial')
        node(inertial, 'mass', mass)
        inertia = node(inertial, 'inertia')
        for i, axis in enumerate(['ixx', 'iyy', 'izz']):
            node(inertia, axis, tensor[i, i])
        if name == 'body':
            c = node(link, 'collision', name='collision')
            node(c, 'pose', f'0 0 0 {math.pi / 2} 0 0')
            cylinder = node(node(c, 'geometry'), 'cylinder')
            node(cylinder, 'radius', shape['radius'])
            node(cylinder, 'length', shape['length'])
            friction = node(node(node(c, 'surface'), 'friction'), 'ode')
            node(friction, 'mu', inputs['drive_wheel_mu'])
            node(friction, 'mu2', inputs['drive_wheel_mu'])
    for name, parent, child, kind, direction, lower, upper in [
            ('normal_guide', 'world', 'sprung', 'prismatic', '1 0 0', -.05, .05),
            ('tyre_compression', 'sprung', 'axle', 'prismatic', '1 0 0', 0, TRAVEL),
            ('wheel_spin', 'axle', 'body', 'revolute', '0 1 0', -1e16, 1e16)]:
        joint = node(model, 'joint', name=name, type=kind)
        node(joint, 'parent', parent)
        node(joint, 'child', child)
        axis = node(joint, 'axis')
        node(axis, 'xyz', direction)
        limits = node(axis, 'limit')
        node(limits, 'lower', lower)
        node(limits, 'upper', upper)
        if name == 'tyre_compression':
            dynamics = node(axis, 'dynamics')
            for key, value in [('spring_reference', 0), ('spring_stiffness', stiffness),
                               ('damping', damping)]:
                node(dynamics, key, value)
        if name == 'wheel_spin':
            node(node(axis, 'dynamics'), 'damping', inputs['actuator_damping_nm_s'])
    return root


def analyze(out):
    """Compare measured joint displacement and independent collision force."""
    with (out / 'native/frames.csv').open() as f:
        rows = list(csv.DictReader(f))
    values = {key: np.array([float(r[key]) for r in rows])
              for key in ['time_s', 'compression_m', 'compression_velocity_mps', 'contacts']}
    with (out / 'native/contacts.csv').open() as f:
        contacts = list(csv.DictReader(f))
    forces = {}
    for c in contacts:
        t = float(c['time_s'])
        forces[t] = forces.get(t, 0.) + float(c['force_x_n'])
    t, q = values['time_s'], values['compression_m']
    force = np.array([forces.get(x, 0.) for x in t])
    windows = {}
    for name, start, end, load in WINDOWS:
        ids = (t >= start) & (t <= end)
        windows[name] = {'compression_mean_m': float(q[ids].mean()),
                         'compression_peak_to_peak_m': float(np.ptp(q[ids])),
                         'contact_load_mean_n': float(force[ids].mean()),
                         'maximum_abs_compression_velocity_mps': float(np.max(
                             np.abs(values['compression_velocity_mps'][ids]))),
                         'expected_load_n': load}
    nominal = windows['150_up']['compression_mean_m']
    checks = {'finite': all(bool(np.isfinite(v).all()) for v in values.values()),
              'target': abs(nominal / TARGET - 1) <= GATES['target_relative_error'],
              'settled': all(v['compression_peak_to_peak_m'] <= GATES['tail_peak_to_peak_m']
                             for v in windows.values()),
              'independent_contact_load': all(
                  abs(v['contact_load_mean_n'] - v['expected_load_n']) <=
                  GATES['contact_load_error_n'] for v in windows.values()),
              'unloaded': abs(windows['unloaded']['compression_mean_m']) <=
              GATES['unloaded_residual_m'],
              'no_upper_stop': float(q.max()) < TRAVEL - GATES['stop_clearance_m'],
              'no_tensile_extension': float(q.min()) >=
              -GATES['negative_compression_tolerance_m'],
              'repeat_150': abs(windows['150_down']['compression_mean_m'] - nominal) <=
              GATES['repeat_150_error_m'],
              'linear': all(abs(windows[name]['compression_mean_m'] /
                                (nominal * load / 150) - 1) <= GATES['linearity_relative_error']
                            for name, load in [('50_up', 50), ('300_up', 300)])}
    result = {'windows': windows, 'compression_min_m': float(q.min()),
              'compression_max_m': float(q.max()), 'checks': checks,
              'passed': all(checks.values())}
    write(out / 'metrics.json', result)
    return result


def trial(out, inputs, stiffness, binary, env):
    """Run one load cycle in a fresh native process with explicit ODE mapping."""
    out.mkdir()
    damping = 2 * ZETA * math.sqrt(stiffness * inputs['sprung_mass_kg'])
    ET.ElementTree(fixture(inputs, stiffness, damping)).write(out / 'world.sdf')
    request = {'world': 'world.sdf', 'iterations': round(4.6 / STEP),
               'normal_load_profile': [{'time_s': t, 'force_n': f} for t, f in PROFILE]}
    (out / 'request.yaml').write_text(yaml.safe_dump(request))
    log = Path('/tmp') / f'climbot_m2_t1_{out.name}_{time.time_ns()}.log'
    with log.open('w') as stream:
        run = subprocess.run([str(binary), str(out / 'request.yaml'), str(out / 'native')],
                             env=dict(env, GZ_PARTITION='climbot_t1_' + uuid.uuid4().hex),
                             stdout=stream, stderr=subprocess.STDOUT, timeout=60)
    if run.returncode:
        raise RuntimeError(f'native exit {run.returncode}: {log}')
    loaded = yaml.safe_load((out / 'native/run.yaml').read_text())['ode_libraries']
    expected = str((Path(env['LD_LIBRARY_PATH'].split(':')[0]) / 'libode.so.8').resolve())
    if loaded != [expected]:
        raise RuntimeError(f'wrong ODE mapping: {loaded}')
    return dict(analyze(out), stiffness_n_m=stiffness, damping_n_s_m=damping,
                log_path=str(log))


def discrete_target_stiffness(inputs):
    """Invert the measured implicit-contact response, then verify in Gazebo."""
    mass = inputs['wheel']['mass_kg']
    b = 2 * ZETA * math.sqrt(inputs['sprung_mass_kg']) * STEP / mass
    c = STEP**2 / mass - TARGET / 150
    if c >= 0:
        raise ValueError('target is below the discrete contact displacement floor')
    inverse_root = -2 * c / (b + math.sqrt(b * b - 4 * c))
    return 1 / inverse_root**2


def main():
    """Calibrate with bounded trials, preserving failed candidates and fixed gates."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--ode-prefix', type=Path, required=True)
    parser.add_argument('--resume', action='store_true', help='reuse recorded failed candidates')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=args.resume)
    inputs = mechanical_inputs()
    protocol = {'target_m': TARGET, 'step_s': STEP, 'gates': GATES,
                'zeta': ZETA, 'axle_mass_kg': AXLE_MASS,
                'compression_limit_m': TRAVEL, 'profile': PROFILE,
                'windows': WINDOWS, 'mechanical_inputs': inputs}
    if args.resume:
        if json.loads((args.output / 'protocol.json').read_text()) != json.loads(
                json.dumps(protocol)):
            raise ValueError('resume protocol differs from the frozen loading trial')
    else:
        write(args.output / 'protocol.json', protocol)
    env, mesa = rendering_environment()
    env, ode = environment(args.ode_prefix, env)
    binary = Path(get_package_prefix('climbot_gazebo')) / 'lib/climbot_gazebo/wall_surface_probe'
    stiffness = 150 / TARGET
    trials = []
    prior = json.loads((args.output / 'summary.json').read_text()) if args.resume else None
    if prior and (prior['binary_sha256'] != hashlib.sha256(binary.read_bytes()).hexdigest() or
                  prior['ode']['library_sha256'] != ode['library_sha256']):
        raise ValueError('resume binary or ODE differs from the recorded trial')
    for i in range(4):
        current = (prior['trials'][i] if prior and i < len(prior['trials']) else
                   trial(args.output / f'trial_{i}', inputs, stiffness, binary, env))
        trials.append(current)
        if current['passed']:
            break
        # A too-soft initial guess can clip at 300 N. Correct from its unclipped
        # 150 N sample; the final candidate must still pass every original gate.
        nominal = current['windows']['150_up']['compression_mean_m']
        if (nominal >= TRAVEL - GATES['stop_clearance_m'] or
                any(not current['checks'][k] for k in
                    ['finite', 'settled', 'independent_contact_load', 'unloaded',
                     'no_tensile_extension', 'repeat_150'])):
            break
        measured = current['windows']['150_up']['compression_mean_m']
        prediction = 150 / current['stiffness_n_m'] * (
            1 + current['damping_n_s_m'] * STEP / inputs['wheel']['mass_kg']) + (
            150 * STEP**2 / inputs['wheel']['mass_kg'])
        if abs(prediction - measured) < .000001:
            # All three exploratory samples agreed with this discrete relation
            # within 0.001 um. New runs need one guess and one confirmation;
            # resume retains the older scale/secant trials without repeating them.
            stiffness = discrete_target_stiffness(inputs)
        elif i == 0:
            stiffness *= measured / TARGET
        else:
            a, b = trials[-2:]
            qa = a['windows']['150_up']['compression_mean_m']
            qb = b['windows']['150_up']['compression_mean_m']
            slope = (qb - qa) / (1 / b['stiffness_n_m'] - 1 / a['stiffness_n_m'])
            inverse = 1 / stiffness + (TARGET - measured) / slope
            if inverse <= 0:
                break
            stiffness = 1 / inverse
    dirty = bool(subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT))
    result = {'scope': 'Single-wheel normal-only flat-wall calibration; no whole-robot coupling',
              'passed': trials[-1]['passed'], 'protocol': 'protocol.json', 'trials': trials,
              'mesa': mesa, 'ode': ode, 'source_commit': subprocess.check_output(
                  ['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
              'source_dirty': dirty,
              'source_sha256': {str(p.relative_to(ROOT)):
                                hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in [Path(__file__).resolve(), ROOT / 'src/climbot_gazebo/'
                                          'src/wall_surface_probe.cpp']},
              'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest()}
    write(args.output / 'summary.json', result)
    print(json.dumps({'passed': result['passed'], 'trials': len(trials),
                      'last': trials[-1], 'output': str(args.output)}))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
