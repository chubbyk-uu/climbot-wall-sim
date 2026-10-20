#!/usr/bin/env python3
"""One paired full-robot flat-wall rolling calibration; no S6 matrix or GUI claim."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import uuid

from ament_index_python.packages import get_package_prefix
from climbot_gazebo.mesa_runtime import rendering_environment
from climbot_gazebo.ode_runtime import environment
from climbot_gazebo.vacuum_assets import build_assets
import numpy as np
from validate_vacuum_motion import frame_data
import yaml

ROOT = Path(__file__).resolve().parents[1]
PROFILE = [(0, 0, 0), (1, 0, 0), (1.6, .2, 0), (3.2, .2, 0),
           (3.8, 0, 0), (4.2, 0, 0), (4.8, 0, .35), (6.4, 0, .35),
           (7, 0, 0), (7.5, 0, 0)]
# Freeze practical tolerances before the paired simulation. No GUI performance claim.
GATES = {'position_difference_m': .005, 'orientation_difference_deg': .2,
         'mean_load_difference_n': 5., 'rolling_force_relative_difference': .05,
         'rolling_power_relative_difference': .05, 'speed_ratio_error': .05,
         'contact_slip_mps': .005}


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def run(out, proxy, binary, env, caster_compliance=False, wheel_compliance=False,
        tyre_lateral=False):
    resolved = build_assets(out, caster_proxy=proxy, heading_deg=90, realtime_factor=0,
                            caster_compliance=caster_compliance,
                            wheel_compliance=wheel_compliance, tyre_lateral=tyre_lateral)
    request = {'world': 'world.sdf', 'step_s': .001, 'duration_s': 11.,
               'track_m': resolved['model']['track_m'],
               'radius_m': resolved['model']['config']['wheel']['radius_m'],
               'motion_profile': [dict(zip(
                   ['after_release_s', 'linear_mps', 'angular_rps'], p)) for p in PROFILE]}
    (out / 'request.yaml').write_text(yaml.safe_dump(request))
    child = dict(env, GZ_PARTITION='climbot_m2_roll_' + uuid.uuid4().hex)
    log = Path('/tmp') / f'climbot_m2_roll_{out.name}_{time.time_ns()}.log'
    with log.open('w') as stream:
        completed = subprocess.run([str(binary), str(out / 'request.yaml'), str(out / 'native')],
                                   env=child, stdout=stream, stderr=subprocess.STDOUT, timeout=90)
    if completed.returncode:
        raise RuntimeError(f'native exit {completed.returncode}; inspect {log}')
    libraries = json.loads((out / 'native/runtime_libraries.json').read_text())
    expected = str((Path(env['LD_LIBRARY_PATH'].split(':')[0]) / 'libode.so.8').resolve())
    if expected not in libraries['physics_libraries']:
        raise RuntimeError('selected private ODE was not loaded')
    c = frame_data(out / 'native/frames.csv')
    release = float(c['vac_release_time_s'][-1])
    if release < 0 or release + 7.5 > c['sim_time_s'][-1]:
        raise RuntimeError('startup did not release in time to complete the paired action')
    radius = .00625 if proxy else .0125
    velocity = np.column_stack([c['ball_v' + a] for a in 'xyz'])
    spin = np.column_stack([c['ball_w' + a] for a in 'xyz'])
    torque = np.column_stack([c['vac_rolling_m' + a] for a in 'xyz'])
    slip = velocity + np.cross(spin, np.array([-radius, 0, 0]))
    phases = {}
    for name, start, end in [('straight', 2.2, 3.1), ('turn', 5.4, 6.3)]:
        ids = (c['sim_time_s'] >= release + start) & (c['sim_time_s'] <= release + end)
        phases[name] = {'mean_ball_load_n': float(c['ball_fx'][ids].mean()),
                        'mean_spin_rps': float(np.linalg.norm(spin[ids, 1:], axis=1).mean()),
                        'mean_equivalent_rolling_force_n': float(
                            np.linalg.norm(torque[ids], axis=1).mean() / radius),
                        'mean_rolling_dissipation_w': float(
                            (-np.sum(torque[ids] * spin[ids], axis=1)).mean()),
                        'maximum_contact_slip_mps': float(
                            np.linalg.norm(slip[ids], axis=1).max())}
    active = c['sim_time_s'] > release + 1
    checks = {'released': True, 'no_runtime_failure': bool(np.all(c['vac_runtime_failed'] == 0)),
              'no_fixture_reintroduced': bool(np.all(c['vac_fixture_active'][active] == 0)),
              'no_skirt_stop': bool(np.all(c['vac_skirt_stop_previous_normal_n'][active] == 0)),
              'finite': all(bool(np.isfinite(x).all()) for x in c.values())}
    metrics = {'checks': checks, 'phases': phases, 'release_s': release,
               'log_path': str(log), 'ode_verified': True}
    write(out / 'case_summary.json', metrics)
    return c, metrics


def compare(a, b, ma, mb, expected_spin_ratio=2):
    # Compare matching times relative to each run's own physical fixture release.
    t = np.arange(1., 7.5, .001)
    ca = {k: np.interp(t + ma['release_s'], a['sim_time_s'], a[k])
          for k in ['pose_x', 'pose_y', 'pose_z', 'qx', 'qy', 'qz', 'qw']}
    cb = {k: np.interp(t + mb['release_s'], b['sim_time_s'], b[k])
          for k in ca}
    position = np.column_stack([cb[k] - ca[k] for k in ['pose_x', 'pose_y', 'pose_z']])
    qa = np.column_stack([ca[k] for k in ['qx', 'qy', 'qz', 'qw']])
    qb = np.column_stack([cb[k] for k in ['qx', 'qy', 'qz', 'qw']])
    dot = np.abs(np.sum(qa * qb, axis=1)) / (
        np.linalg.norm(qa, axis=1) * np.linalg.norm(qb, axis=1))
    metrics = {'maximum_position_difference_m': float(np.linalg.norm(position, axis=1).max()),
               'maximum_orientation_difference_deg': float(np.degrees(
                   2 * np.arccos(np.clip(dot, 0, 1))).max())}
    checks = {'position': metrics['maximum_position_difference_m'] <=
              GATES['position_difference_m'],
              'orientation': metrics['maximum_orientation_difference_deg'] <=
              GATES['orientation_difference_deg']}
    phases = {}
    for name in ma['phases']:
        x, y = ma['phases'][name], mb['phases'][name]
        p = {'load_difference_n': abs(x['mean_ball_load_n'] - y['mean_ball_load_n']),
             'spin_ratio': y['mean_spin_rps'] / x['mean_spin_rps']}
        for label, field in [('force', 'mean_equivalent_rolling_force_n'),
                             ('power', 'mean_rolling_dissipation_w')]:
            p[label + '_relative_difference'] = abs(y[field] / x[field] - 1)
        phases[name] = p
        checks.update({name + '/load': p['load_difference_n'] <= GATES['mean_load_difference_n'],
                       name + '/spin': abs(p['spin_ratio'] / expected_spin_ratio - 1) <=
                       GATES['speed_ratio_error'],
                       name + '/force': p['force_relative_difference'] <=
                       GATES['rolling_force_relative_difference'],
                       name + '/power': p['power_relative_difference'] <=
                       GATES['rolling_power_relative_difference'],
                       name + '/slip': max(
                           x['maximum_contact_slip_mps'], y['maximum_contact_slip_mps']) <=
                       GATES['contact_slip_mps']})
    return dict(metrics, phases=phases, checks=checks)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--ode-prefix', required=True, type=Path)
    parser.add_argument('--caster-compliance-pair', action='store_true',
                        help='same proxy and soft tyres; compare fixed versus floating housing')
    parser.add_argument('--tyre-lateral-pair', action='store_true',
                        help='same proxy and soft tyres; compare without/with lateral tyre spring')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    protocol = {'gates': GATES, 'profile': PROFILE, 'step_s': .001, 'heading_deg': 90,
                'caster_compliance_pair': args.caster_compliance_pair,
                'tyre_lateral_pair': args.tyre_lateral_pair}
    write(args.output / 'protocol.json', protocol)
    prefix = Path(get_package_prefix('climbot_gazebo'))
    env, mesa = rendering_environment()
    env, ode = environment(args.ode_prefix, env)
    env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = os.pathsep.join(filter(None, [
        str(prefix / 'lib'), env.get('GZ_SIM_SYSTEM_PLUGIN_PATH')]))
    binary = prefix / 'lib/climbot_gazebo/vacuum_sim_probe'
    same_ball = args.caster_compliance_pair or args.tyre_lateral_pair
    if args.tyre_lateral_pair:
        a, ma = run(args.output / 'fixed', True, binary, env, wheel_compliance=True)
        b, mb = run(args.output / 'lateral', True, binary, env,
                    wheel_compliance=True, tyre_lateral=True)
    elif args.caster_compliance_pair:
        a, ma = run(args.output / 'fixed', True, binary, env, wheel_compliance=True)
        b, mb = run(args.output / 'floating', True, binary, env,
                    caster_compliance=True, wheel_compliance=True)
    else:
        a, ma = run(args.output / 'real', False, binary, env)
        b, mb = run(args.output / 'proxy', True, binary, env)
    paired = compare(a, b, ma, mb, expected_spin_ratio=1 if same_ball else 2)
    result = {'scope': 'Full robot flat-wall straight/turn pair; no rough wall or GUI acceptance',
              'passed': all(paired['checks'].values()) and
              all(v for m in [ma, mb] for v in m['checks'].values()),
              'gates': GATES, 'paired': paired,
              ('fixed' if same_ball else 'real'): ma,
              ('lateral' if args.tyre_lateral_pair else
               'floating' if args.caster_compliance_pair else 'proxy'): mb,
              'mesa': mesa, 'ode': ode,
              'source_commit': subprocess.check_output(
                  ['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
              'source_sha256': {str(p.relative_to(ROOT)):
                                hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in [Path(__file__).resolve(), ROOT / 'src/climbot_gazebo/'
                                          'climbot_gazebo/vacuum_assets.py', ROOT /
                                          'src/climbot_gazebo/src/vacuum_sim_probe.cpp',
                                          ROOT / 'src/climbot_gazebo/climbot_gazebo/'
                                          'caster_compliance.py', ROOT /
                                          'src/climbot_gazebo/config/m2_caster_compliance.yaml',
                                          ROOT / 'src/climbot_gazebo/climbot_gazebo/'
                                          'wheel_compliance.py', ROOT /
                                          'src/climbot_gazebo/config/m2_tyre_lateral.yaml']},
              'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest()}
    write(args.output / 'summary.json', result)
    print(json.dumps({'passed': result['passed'], 'paired': paired, 'output': str(args.output)}))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
