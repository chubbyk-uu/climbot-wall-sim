#!/usr/bin/env python3
"""One explicitly configured M2 patch motion; detailed T2 checks are independent."""
import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import uuid
from xml.etree import ElementTree as ET

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
    right = np.array(rotate_vector(q, [1, 0, 0]))
    axes = {'horizontal': forward, 'vertical': right}
    crossing_axes = {side: [name for name, axis in axes.items()
                            if (p[ids] @ axis).min() < .6 < (p[ids] @ axis).max()]
                     for side, p in positions.items()}
    crossing = {side: bool(names) for side, names in crossing_axes.items()}
    gaps = {side: gap_steps(c[side + '_points'][ids]) for side in positions}
    angle = np.deg2rad(resolved['heading_deg'])
    direction = np.array(rotate_vector(q, [np.cos(angle), np.sin(angle), 0]))
    travel = float((np.array([c['pose_' + a][-1] - c['pose_' + a][0]
                              for a in 'xyz']) @ direction))
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
            'travel_m': travel, 'wheels_crossed': crossing, 'crossed_axes': crossing_axes,
            'max_contact_gap_steps': gaps,
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
    parser.add_argument('--caster-compliance', action='store_true')
    parser.add_argument('--tyre-lateral', action='store_true',
                        help='opt-in tyre lateral spring; requires --soft')
    parser.add_argument('--smooth-control', action='store_true',
                        help='same roughness without joints')
    parser.add_argument('--native-heightmap', action='store_true',
                        help='roughness-only driver contact; caster and skirt retain joints')
    parser.add_argument('--mode', choices=['fixed_pressure', 'fan_ode'], default='fixed_pressure')
    parser.add_argument('--channels', action='store_true')
    parser.add_argument('--flat', action='store_true')
    parser.add_argument('--heading-deg', type=float, default=90)
    parser.add_argument('--seed', type=int, default=20261019)
    parser.add_argument('--drape-m', type=float)
    parser.add_argument('--refinement', type=int)
    parser.add_argument('--duration-s', type=float, default=9)
    parser.add_argument('--diagnostic-step-s', type=float, choices=[.001, .0005], default=.001,
                        help='isolated step comparison; does not extend tyre calibration')
    parser.add_argument('--patch-size-m', type=float, default=1.2)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    source = config()
    source['patch_size_m'] = args.patch_size_m
    source['start_u_m'] = .6
    source['grooves'][0]['end_m'][0] = args.patch_size_m + .02
    source['grooves'][1]['end_m'][1] = args.patch_size_m + .02
    source['roughness']['seed'] = args.seed
    if args.flat:
        source['roughness']['amplitude_m'] = 0
    if args.channels:
        channel_path = ROOT / 'src/climbot_gazebo/config/m2_seal_channels.yaml'
        source['channel_model'] = yaml.safe_load(channel_path.read_text())['m2_seal_channels']
        if args.drape_m is not None:
            source['channel_model']['drape_m'] = args.drape_m
        if args.refinement is not None:
            source['channel_model']['refinement'] = args.refinement
    if args.smooth_control:
        source['grooves'] = []
    if args.native_heightmap:
        source['contact_representation'] = 'native_heightmap'
    write(args.output / 'protocol.json', {
        'scope': 'M2 patch motion; detailed T2 checks independent',
        'step_s': args.diagnostic_step_s, 'tyre_calibration_step_s': .001,
        'profile': PROFILE, 'duration_s': args.duration_s, 'source': source,
        'mode': args.mode, 'heading_deg': args.heading_deg,
        'max_contact_gap_steps': 2, 'travel_range_m': [.45, .7], 'soft': args.soft,
        'smooth_control': args.smooth_control, 'native_heightmap': args.native_heightmap,
        'caster_compliance': args.caster_compliance, 'tyre_lateral': args.tyre_lateral})
    resolved = build_assets(args.output, mode=args.mode, caster_proxy=True,
                            wheel_compliance=args.soft, surface_patch=source,
                            heading_deg=args.heading_deg, realtime_factor=0,
                            caster_compliance=args.caster_compliance,
                            tyre_lateral=args.tyre_lateral)
    if args.diagnostic_step_s != .001:
        # Production generation still rejects uncalibrated tyre timesteps.
        # Only this explicitly labelled numerical diagnostic alters the world.
        world_path = args.output / 'world.sdf'
        world = ET.parse(world_path)
        world.find('.//physics/max_step_size').text = str(args.diagnostic_step_s)
        world.write(world_path, encoding='unicode', xml_declaration=True)
        resolved['step_s'] = args.diagnostic_step_s
        resolved['diagnostic_uncalibrated_tyre_step'] = True
        write(args.output / 'resolved.json', resolved)
    request = {'world': 'world.sdf', 'step_s': args.diagnostic_step_s,
               'duration_s': args.duration_s,
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
                             stderr=subprocess.STDOUT, timeout=max(90, args.duration_s * 4))
    if run.returncode:
        raise RuntimeError(f'native exit {run.returncode}; inspect {log}')
    libraries = json.loads((args.output / 'native/runtime_libraries.json').read_text())
    expected = str((args.ode_prefix / 'lib/libode.so.8').resolve())
    if expected not in libraries['physics_libraries']:
        raise RuntimeError('wrong native ODE mapping')
    metrics = evaluate(frame_data(args.output / 'native/frames.csv'), resolved)
    result = {'scope': 'M2 patch motion; detailed fan/leak and T2 checks independent',
              'passed': metrics['passed'], 'metrics': metrics, 'patch': resolved['surface_patch'],
              'soft': args.soft, 'smooth_control': args.smooth_control,
              'mesa': mesa, 'ode': ode, 'log_path': str(log),
              'source_commit': subprocess.check_output(
                  ['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
              'source_dirty': bool(subprocess.check_output(
                  ['git', 'status', '--porcelain'], cwd=ROOT, text=True).strip()),
              'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest()}
    write(args.output / 'summary.json', result)
    print(json.dumps({'passed': result['passed'], 'metrics': metrics}))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
