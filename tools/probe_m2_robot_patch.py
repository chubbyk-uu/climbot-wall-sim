#!/usr/bin/env python3
"""One full-robot B-mode rough/jointed patch crossing, before fan/leak coupling."""
import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import uuid

from ament_index_python.packages import get_package_prefix
from climbot_description.geometry import quaternion_from_rpy, rotate_vector
from climbot_gazebo.mesa_runtime import rendering_environment
from climbot_gazebo.ode_runtime import environment
from climbot_gazebo.vacuum_assets import build_assets
import numpy as np
from validate_vacuum_motion import frame_data
import yaml

ROOT = Path(__file__).resolve().parents[1]
PROFILE = [(0, 0, 0), (1, 0, 0), (1.6, .2, 0), (4.6, .2, 0), (5.2, 0, 0), (6, 0, 0)]


def write(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')


def config():
    """Extend the existing crossed-joint source to a 1.2 m representative patch."""
    cfg = yaml.safe_load((ROOT / 'src/climbot_gazebo/config/m2_surface_fixture.yaml').read_text())
    cfg = deepcopy(cfg)
    cfg['grooves'] = cfg['grooves'][:2]
    for index, g in enumerate(cfg['grooves']):
        g['start_m'] = [-.02, .6] if index == 0 else [.6, -.02]
        g['end_m'] = [1.22, .6] if index == 0 else [.6, 1.22]
    return cfg


def gap_steps(values):
    longest = streak = 0
    for value in values:
        streak = streak + 1 if value == 0 else 0
        longest = max(longest, streak)
    return longest


def evaluate(c, resolved):
    """Measure actual wheel crossings and contact continuity; no fan-margin claim."""
    t = c['sim_time_s']
    released = (c['vac_fixture_active'] == 0) & (c['paused'] == 0)
    if not released.any():
        return {'passed': False, 'checks': {'released': False}}
    release = float(c['vac_release_time_s'][released][0])
    ids = released & (t >= release + 1) & (t <= release + 6)
    q = quaternion_from_rpy(*resolved['wall_surface_pose'][3:])
    normal, forward = [np.array(rotate_vector(q, a)) for a in [(0, 0, 1), (0, 1, 0)]]
    origin = np.array(resolved['wall_surface_pose'][:3])
    positions = {side: np.column_stack([c[side + '_' + a] for a in 'xyz']) - origin
                 for side in ['left', 'right', 'ball']}
    crossing = {side: bool((p[ids] @ forward).min() < .6 and
                           (p[ids] @ forward).max() > .6) for side, p in positions.items()}
    gaps = {side: gap_steps(c[side + '_points'][ids]) for side in positions}
    travel = float((np.array([c['pose_' + a][-1] - c['pose_' + a][0]
                              for a in 'xyz']) @ forward))
    pressure = c['vac_delta_pa'][ids]
    normals = np.column_stack([c[s + '_fx'][ids] for s in positions])
    # Freeze modest diagnostic gates before running; detailed friction, impulses
    # and pressure demand belong to T2. One local contact-miss step is permitted.
    checks = {'released': True, 'complete_action': bool(t[-1] >= release + 6),
              'no_runtime_failure': bool(np.all(c['vac_runtime_failed'] == 0)),
              'no_startup_failure': bool(np.all(c['vac_startup_failed'] == 0)),
              'all_wheels_crossed': all(crossing.values()),
              'bounded_travel': .45 <= travel <= .7,
              'contact_gaps_at_most_two_steps': max(gaps.values()) <= 2,
              'no_skirt_stop': bool(np.all(c['vac_skirt_stop_previous_normal_n'][ids] == 0)),
              'finite': all(bool(np.isfinite(v).all()) for v in c.values())}
    patch = resolved['surface_patch']
    crossing_scope = 'virtual line in a smooth control; no joint is present'
    if patch['joint_count']:
        crossing_scope = ('source seam position; caster uses full joint proxy; '
                          'drivers bridge joints' if
                          patch['driver_surface_policy'] == 'roughness_only' else 'physical joint')
    return {'passed': all(checks.values()), 'checks': checks, 'release_s': release,
            'crossing_scope': crossing_scope,
            'travel_m': travel, 'wheels_crossed': crossing, 'max_contact_gap_steps': gaps,
            'normal_load_range_n': [normals.min(axis=0).tolist(), normals.max(axis=0).tolist()],
            'ball_outward_height_range_m': [float((positions['ball'][ids] @ normal).min()),
                                            float((positions['ball'][ids] @ normal).max())],
            'pressure_range_pa': [float(pressure.min()), float(pressure.max())],
            'plugin_cpu_us_mean': float(c['vac_cpu_ns'][ids].mean() / 1000)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--ode-prefix', required=True, type=Path)
    parser.add_argument('--soft', action='store_true', help='paired compliant-wheel diagnostic')
    parser.add_argument('--smooth-control', action='store_true',
                        help='same roughness without joints')
    parser.add_argument('--native-heightmap', action='store_true',
                        help='roughness-only driver contact; caster and skirt retain joints')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    write(args.output / 'protocol.json', {
        'scope': 'B contact diagnostics; not T2', 'step_s': .001, 'profile': PROFILE,
        'max_contact_gap_steps': 2, 'travel_range_m': [.45, .7], 'soft': args.soft,
        'smooth_control': args.smooth_control, 'native_heightmap': args.native_heightmap})
    source = config()
    if args.smooth_control:
        source['grooves'] = []
    if args.native_heightmap:
        source['contact_representation'] = 'native_heightmap'
    resolved = build_assets(args.output, mode='fixed_pressure', caster_proxy=True,
                            wheel_compliance=args.soft, surface_patch=source,
                            heading_deg=90, realtime_factor=0)
    request = {'world': 'world.sdf', 'step_s': .001, 'duration_s': 9.,
               'track_m': resolved['model']['track_m'],
               'radius_m': resolved['model']['config']['wheel']['radius_m'],
               'motion_profile': [dict(zip(['after_release_s', 'linear_mps', 'angular_rps'], p))
                                  for p in PROFILE]}
    (args.output / 'request.yaml').write_text(yaml.safe_dump(request))
    prefix = Path(get_package_prefix('climbot_gazebo'))
    binary = prefix / 'lib/climbot_gazebo/vacuum_sim_probe'
    env, mesa = rendering_environment()
    env, ode = environment(args.ode_prefix, env)
    env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = os.pathsep.join(filter(None, [
        str(prefix / 'lib'), env.get('GZ_SIM_SYSTEM_PLUGIN_PATH')]))
    env['GZ_PARTITION'] = 'climbot_m2_patch_' + uuid.uuid4().hex
    log = Path('/tmp') / f'climbot_m2_patch_native_{time.time_ns()}.log'
    with log.open('w') as stream:
        run = subprocess.run([str(binary), str(args.output / 'request.yaml'),
                              str(args.output / 'native')], env=env, stdout=stream,
                             stderr=subprocess.STDOUT, timeout=90)
    if run.returncode:
        raise RuntimeError(f'native exit {run.returncode}; inspect {log}')
    libraries = json.loads((args.output / 'native/runtime_libraries.json').read_text())
    expected = str((args.ode_prefix / 'lib/libode.so.8').resolve())
    if expected not in libraries['physics_libraries']:
        raise RuntimeError('wrong native ODE mapping')
    metrics = evaluate(frame_data(args.output / 'native/frames.csv'), resolved)
    result = {'scope': 'B-mode real rough/jointed patch, not fan/leak or T2 acceptance',
              'passed': metrics['passed'], 'metrics': metrics, 'patch': resolved['surface_patch'],
              'soft': args.soft, 'smooth_control': args.smooth_control,
              'mesa': mesa, 'ode': ode, 'log_path': str(log),
              'source_commit': subprocess.check_output(
                  ['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
              'source_dirty': True,
              'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest()}
    write(args.output / 'summary.json', result)
    print(json.dumps({'passed': result['passed'], 'metrics': metrics}))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
