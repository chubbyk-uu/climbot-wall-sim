#!/usr/bin/env python3
"""S5 native motion and independently declared port/fan failure experiments."""
import argparse
import csv
import gzip
import hashlib
import itertools
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
import uuid

from ament_index_python.packages import get_package_prefix
from climbot_description.geometry import quaternion_from_rpy, rotate_vector
from climbot_description.vacuum_loads import minimum_pressure
from climbot_gazebo.mesa_runtime import rendering_environment
from climbot_gazebo.vacuum_assets import build_assets
from freeze_suction_baseline import INPUTS
import numpy as np
from validate_vacuum_physics import reference_pressure, reference_rate, response_time
import yaml

ROOT = Path(__file__).resolve().parents[1]
STOP_FIELD = 'vac_skirt_stop_previous_normal_n'


def write(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')


def frame_data(path):
    with path.open() as stream:
        names = next(csv.reader(stream))
        values = np.loadtxt(stream, delimiter=',', ndmin=2)
    return {name: values[:, i] for i, name in enumerate(names)}


def profiles():
    """Build controller cruise/catch-up/turn values below shared actuator limits."""
    control = yaml.safe_load((ROOT / 'src/climbot_control/config/control.yaml').read_text())[
        'line_tracker']['ros__parameters']
    speed = float(control['catch_up_max_linear_speed'])
    ramp = speed / float(control['catch_up_max_linear_acceleration'])
    turn = float(control['max_turn_angular_speed'])
    tr = turn / float(control['max_turn_angular_acceleration'])
    return {
        'straight': [(0, 0, 0), (2, 0, 0), (2 + ramp, speed, 0), (4.5, speed, 0),
                     (4.5 + ramp, 0, 0), (6, 0, 0), (6 + ramp, -speed, 0),
                     (8.5, -speed, 0), (8.5 + ramp, 0, 0), (12, 0, 0)],
        'turn': [(0, 0, 0), (2, 0, 0), (2 + tr, 0, turn), (5.6, 0, turn),
                 (5.6 + tr, 0, 0), (7, 0, 0), (7 + tr, 0, -turn),
                 (10.6, 0, -turn), (10.6 + tr, 0, 0), (12, 0, 0)],
        'mixed': [(0, 0, 0), (2, 0, 0), (3, .2, .35), (6, .2, .35),
                  (7, 0, 0), (8, 0, 0), (9, -.2, -.35), (12, -.2, -.35),
                  (13, 0, 0), (14, 0, 0)],
    }


def cases(stage):
    result = []
    if stage in ('all', 'motion'):
        for profile, heading, mode in itertools.product(
                profiles(), range(0, 360, 45), ['fixed_pressure', 'fan_ode']):
            result.append({'name': f'{profile}_{heading:03}_{mode}', 'kind': 'motion',
                           'profile': profile, 'heading': heading, 'mode': mode, 'duration': 16})
    if stage in ('all', 'bench'):
        # Identical curves in a single lumped chamber: fan permutations are symmetric.
        for reverse, area_mm2, stopped in itertools.product(
                [0., .05, .2], [0, 40, 160, 400], [1, 2, 3]):
            result.append({'name': f'bench_r{reverse:g}_a{area_mm2}_off{stopped}', 'kind': 'bench',
                           'reverse': reverse, 'port_mm2': area_mm2, 'stopped': stopped,
                           'mode': 'fan_ode', 'heading': 90, 'duration': 9})
    if stage in ('all', 'free'):
        for stopped, hold in [(1, 1.), (2, .15), (3, .02), (3, 1.)]:
            result.append({'name': f'free_off{stopped}_hold{hold:g}', 'kind': 'free',
                           'stopped': stopped, 'hold': hold, 'mode': 'fan_ode',
                           'heading': 90, 'duration': 7})
        result.append({'name': 'free_port_pulse', 'kind': 'port', 'mode': 'fan_ode',
                       'heading': 90, 'duration': 7})
    if stage in ('all', 'convergence'):
        for step in [.001, .0005]:
            result.append({'name': f'numeric_{step:g}', 'kind': 'numeric', 'mode': 'fan_ode',
                           'heading': 90, 'duration': 9, 'step': step,
                           'port_mm2': 160, 'stopped': 1, 'reverse': .05})
    return result


def motion_metrics(c, resolved, directory, profile_name):
    t = c['sim_time_s']
    released = (c['paused'] == 0) & (c['vac_fixture_active'] == 0)
    if not released.any():
        return {'checks': {'released': False}}
    release = float(c['vac_release_time_s'][released][0])
    end = release + profiles()[profile_name][-1][0]
    mask = released & (t >= release + 2) & (t <= end + 1e-9)
    ids = np.flatnonzero(mask)
    if np.any(c['vac_runtime_failed']):
        return {'checks': {'no_runtime_failure': False},
                'reason': 'normal motion left pressure model validity domain'}
    normal = np.column_stack([c[side + '_fx'][ids] for side in ['left', 'right', 'ball']])
    tangent = np.column_stack([np.hypot(c[side + '_fy'][ids], c[side + '_fz'][ids])
                               for side in ['left', 'right']])
    required = np.maximum(5, 1.5 * tangent / 1.1)
    qx, qy, qz, qw = [c[k] for k in ['qx', 'qy', 'qz', 'qw']]
    forward = np.column_stack((
        1 - 2 * (qy**2 + qz**2), 2 * (qx * qy + qw * qz), 2 * (qx * qz - qw * qy)))
    wall_q = quaternion_from_rpy(*resolved['wall_surface_pose'][3:])
    wall_x, wall_y, wall_z = [
        np.array(rotate_vector(wall_q, axis)) for axis in [(1, 0, 0), (0, 1, 0), (0, 0, 1)]]
    heading = np.degrees(np.arctan2(forward @ wall_y, forward @ wall_x))
    velocity = np.column_stack([c[k] for k in ['vx', 'vy', 'vz']])
    acceleration = np.gradient(velocity, t, axis=0)
    v = np.sum(velocity * forward, axis=1)
    a = np.sum(acceleration * forward, axis=1)
    w = np.column_stack([c[k] for k in ['wx', 'wy', 'wz']]) @ wall_z
    alpha = np.gradient(w, t)
    state = np.column_stack((heading[ids], v[ids], a[ids], w[ids], alpha[ids]))
    pressure_min = np.concatenate([minimum_pressure(
        resolved['model'], state[i:i + 256], lateral='conservative')
        for i in range(0, len(state), 256)])
    actual_pressure = c['vac_delta_pa'][ids]
    flow_reserve = np.array([-reference_rate(
        p, [c[f'vac_fan_speed_{j}'][i] for j in range(3)],
        c['vac_leak_area_m2'][i], resolved['physics']) for i, p in zip(ids, pressure_min)])
    margin = 1.1 * normal[:, :2] / np.maximum(tangent, 1e-12)
    checks = {
        'released': True,
        'no_runtime_failure': not bool(np.any(c['vac_runtime_failed'])),
        'all_native_contacts_present': all(
            np.all(c[s + '_valid'][ids] == 1) and np.all(c[s + '_points'][ids] > 0)
            for s in ['left', 'right', 'ball']),
        'each_drive_normal_meets_demand': bool(np.all(normal[:, :2] >= required)),
        'ball_normal_meets_floor': bool(np.min(normal[:, 2]) >= 10),
        'pressure_above_inverse_dynamic_requirement': bool(
            np.all(actual_pressure >= pressure_min)),
        'fixture_removed_in_scored_window': bool(np.all(c['vac_fixture_fx'][ids] == 0)),
        'flow_capacity_at_required_pressure': bool(np.min(flow_reserve) >= 0),
        'nonpositive_skirt_drag_power': True,
    }
    # Evaluate planar sliding/yaw work; normal elastic work may be positive
    # during decompression and must not be mistaken for friction supplying energy.
    skirt_force = np.column_stack([c['vac_skirt_f' + axis][ids] for axis in 'xyz'])
    skirt_moment = np.column_stack([c['vac_skirt_m' + axis][ids] for axis in 'xyz'])
    input_velocity = np.column_stack(
        [c['vac_force_input_v' + axis + '_mps'][ids] for axis in 'xyz'])
    input_omega = np.column_stack([c['vac_force_input_w' + axis + '_rps'][ids] for axis in 'xyz'])
    tangent_velocity = input_velocity - (input_velocity @ wall_z)[:, None] * wall_z
    skirt_power = np.sum(skirt_force * tangent_velocity, axis=1) + (
        skirt_moment @ wall_z) * (input_omega @ wall_z)
    checks['nonpositive_skirt_drag_power'] = bool(np.max(skirt_power) <= 1e-5)
    # Normal motion must never bottom the seal; frames without the field cannot prove it.
    stop = c.get(STOP_FIELD)
    checks['skirt_stop_not_engaged'] = stop is not None and bool(np.all(stop[ids] == 0))
    with gzip.open(directory / 'inverse_dynamics.csv.gz', 'wt', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['time_s', 'heading_deg', 'v_mps', 'a_mps2', 'omega_rps', 'alpha_rps2',
                         'minimum_delta_pa', 'actual_delta_pa', 'left_required_n',
                         'right_required_n'])
        writer.writerows(np.column_stack((t[ids], state, pressure_min, actual_pressure, required)))
    return {'checks': checks, 'release_s': release, 'scored_samples': len(ids),
            'minimum_normals_n': normal.min(axis=0).tolist(),
            'minimum_friction_margin': float(margin.min()),
            'minimum_pressure_slack_pa': float(np.min(actual_pressure - pressure_min)),
            'maximum_inverse_requirement_pa': float(pressure_min.max()),
            'minimum_pressure_pa': float(actual_pressure.min()),
            'minimum_flow_reserve_at_required_pressure_kg_s': float(flow_reserve.min()),
            'minimum_head_margin_pa': float(np.min(
                c['vac_maximum_fan_head_pa'][ids] - actual_pressure)),
            'maximum_skirt_power_w': float(skirt_power.max()),
            'maximum_skirt_stop_normal_n': None if stop is None else float(stop[ids].max()),
            'maximum_abs_actual_a_mps2': float(np.max(np.abs(a[ids]))),
            'maximum_abs_actual_alpha_rps2': float(np.max(np.abs(alpha[ids]))),
            'end_pose_m': [float(c['pose_' + axis][ids[-1]]) for axis in 'xyz'],
            'plugin_cpu_us_mean': float(np.mean(c['vac_cpu_ns'][ids]) / 1000)}


def fault_metrics(c, resolved, spec):
    t = c['sim_time_s']
    final = (t >= 5.4) & (t < 5.9) if spec['kind'] in ('bench', 'numeric') else t > 6.5
    pressure = float(np.mean(c['vac_delta_pa'][final]))
    speeds = [float(np.mean(c[f'vac_fan_speed_{i}'][final])) for i in range(3)]
    area = float(np.mean(c['vac_leak_area_m2'][final]))
    expected = reference_pressure(speeds, area, resolved['physics'])
    failed = np.flatnonzero(c['vac_runtime_failed'] > 0)
    valid = (c['vac_fixture_active'] == 0) & (c['paused'] == 0) & (c['vac_runtime_failed'] == 0)
    normals = np.column_stack([c[s + '_fx'] for s in ['left', 'right', 'ball']])
    tangents = np.column_stack([np.hypot(c[s + '_fy'], c[s + '_fz']) for s in ['left', 'right']])
    margin = 1.1 * normals[:, :2] / np.maximum(tangents, 1e-12)
    violation = valid & ((normals < [5, 5, 10]).any(axis=1) | (margin < 1.5).any(axis=1))
    first = np.flatnonzero(violation & (t >= 3))
    checks = {'finite_observations': all(np.isfinite(v).all() for v in c.values())}
    if spec['kind'] in ('bench', 'numeric'):
        checks.update({
            'explicitly_constrained': bool(
                np.all(c['vac_fixture_bench_hold'] == 1) and np.all(c['vac_fixture_active'] == 1)),
            'no_runtime_failure': not len(failed),
            'equilibrium_matches_independent_reference': abs(pressure - expected) <= .01,
            'mass_flow_residual_small': bool(
                np.max(np.abs(c['vac_mass_flow_residual_kg_s'][final])) < 1e-8),
            'recovery': abs(float(c['vac_delta_pa'][-1]) - reference_pressure(
                [1, 1, 1], float(c['vac_leak_area_m2'][-1]), resolved['physics'])) < .02,
        })
    else:
        checks['fixture_not_reintroduced'] = bool(np.all(c['vac_fixture_active'][t > 3] == 0))
        checks['failure_boundary_or_complete_observation_recorded'] = bool(
            len(first) or not len(failed))
    trace = [{'time_s': float(x), 'delta_pa': float(y)} for x, y in zip(t, c['vac_delta_pa'])]
    response = None
    if spec['kind'] in ('bench', 'numeric'):
        response = response_time([row for row in trace if row['time_s'] <= 5.9],
                                 event=3, final_delta=pressure)
    invalid_tail = bool(len(failed))
    last_valid = int(failed[0]) - 1 if invalid_tail else len(t) - 1
    return {'checks': checks, 'steady_delta_pa': None if invalid_tail else pressure,
            'reference_delta_pa': None if invalid_tail else expected,
            'equilibrium_error_pa': None if invalid_tail else pressure - expected,
            'steady_speeds': None if invalid_tail else speeds,
            'steady_leak_area_m2': area,
            'first_requirement_violation_s': float(t[first[0]]) if len(first) else None,
            'first_runtime_domain_exit_s': float(t[failed[0]]) if len(failed) else None,
            'response': response,
            'end_delta_pa': None if invalid_tail else float(c['vac_delta_pa'][-1]),
            'last_valid_delta_pa': float(c['vac_delta_pa'][last_valid]),
            'invalid_tail_reason': ('model validity domain exited; pressure snapshot stale'
                                    if invalid_tail else None),
            'minimum_free_normals_n': normals[valid].min(axis=0).tolist() if valid.any() else None,
            'released_s': float(np.max(c['vac_release_time_s'])),
            'constrained_bench_not_free_holding_evidence': spec['kind'] in ('bench', 'numeric')}


def run_case(root, spec, binary, env, identity):
    directory = root / spec['name']
    runtime = {}
    io = {}
    extra = {}
    if spec['kind'] in ('bench', 'numeric'):
        runtime = {'fixture_bench_hold': True, 'controlled_opening_multiplier': 0.}
        io['opening_schedule'] = [{'time_s': 0, 'area_m2': spec['port_mm2'] * 1e-6}]
        path = ROOT / 'src/climbot_gazebo/config/vacuum_physics.yaml'
        physical = yaml.safe_load(path.read_text())['vacuum_physics']
        for fan in physical['fans']:
            fan['reverse_fraction'] = spec['reverse']
        extra['physics_overrides'] = {'fans': physical['fans']}
    if spec['kind'] in ('bench', 'numeric', 'free'):
        end = 6 if spec['kind'] in ('bench', 'numeric') else 3 + spec['hold']
        io['events'] = [{'time_s': t, 'kind': 0, 'index': i, 'value': value}
                        for t, value in [(3, 0), (end, 1)] for i in range(spec['stopped'])]
    if spec['kind'] == 'port':
        io['opening_schedule'] = [{'time_s': t, 'area_m2': area} for t, area in [
            (0, 0), (3, 0), (3.1, .00015), (3.6, .00015), (3.7, 0)]]
    resolved = build_assets(directory, mode=spec['mode'], heading_deg=spec['heading'],
                            realtime_factor=0, step_s=spec.get('step', .001),
                            runtime_overrides=runtime, io_overrides=io, **extra)
    request = {'world': 'world.sdf', 'step_s': spec.get('step', .001),
               'duration_s': spec['duration']}
    if spec['kind'] == 'motion':
        request.update(track_m=resolved['model']['track_m'],
                       radius_m=resolved['model']['config']['wheel']['radius_m'],
                       motion_profile=[
                           dict(zip(['after_release_s', 'linear_mps', 'angular_rps'], p))
                           for p in profiles()[spec['profile']]])
    (directory / 'request.yaml').write_text(yaml.safe_dump(request))
    env = env.copy()
    env['GZ_PARTITION'] = 'climbot_s5_' + uuid.uuid4().hex
    log = Path('/tmp') / f'climbot_s5_{spec["name"]}_{time.time_ns()}.log'
    with log.open('w') as stream:
        result = subprocess.run([str(binary), str(directory / 'request.yaml'),
                                 str(directory / 'native')], env=env, stdout=stream,
                                stderr=subprocess.STDOUT, timeout=180)
    write(directory / 'execution.json', {'exit_code': result.returncode,
                                         'log_name': log.name, 'rendering': identity})
    if result.returncode:
        raise RuntimeError(f'{spec["name"]} native observer failed; inspect {log}')
    libraries = json.loads((directory / 'native/runtime_libraries.json').read_text())
    if identity['mesa'] == 'private' and (
            not libraries['private_mapping_complete'] or libraries['system_gallium_mapped']):
        raise RuntimeError('Incorrect Mesa mapping')
    c = frame_data(directory / 'native/frames.csv')
    metrics = (motion_metrics(c, resolved, directory, spec['profile']) if spec['kind'] == 'motion'
               else fault_metrics(c, resolved, spec))
    write(directory / 'case_summary.json', metrics)
    for path in (directory / 'native').glob('*.csv'):
        with path.open('rb') as src, gzip.open(str(path) + '.gz', 'wb', compresslevel=3) as dst:
            shutil.copyfileobj(src, dst)
        path.unlink()
    return metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--stage', choices=['all', 'motion', 'bench', 'free', 'convergence'],
                        default='all')
    parser.add_argument('--limit', type=int, default=0,
                        help='diagnostic subset, never full acceptance')
    parser.add_argument('--names', nargs='+', help='explicit diagnostic case subset')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    specs = cases(args.stage)
    if args.limit:
        specs = specs[:args.limit]
    if args.names:
        specs = [s for s in specs if s['name'] in args.names]
        if {s['name'] for s in specs} != set(args.names):
            raise ValueError('unknown diagnostic case name')
    write(args.output / 'cases.json', specs)
    prefix = Path(get_package_prefix('climbot_gazebo'))
    binary = prefix / 'lib/climbot_gazebo/vacuum_sim_probe'
    env, identity = rendering_environment()
    env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = os.pathsep.join(filter(None, [
        str(prefix / 'lib'), env.get('GZ_SIM_SYSTEM_PLUGIN_PATH')]))
    # The S0 legacy freeze ended at S7; former inputs are live sources, recorded by hash only.
    source_paths = sorted({p for p in INPUTS if (ROOT / p).exists()} | {
        'tools/validate_vacuum_motion.py', 'tools/validate_vacuum_physics.py',
        'tools/freeze_suction_baseline.py', 'tools/vacuum_layout.py', 'tools/vacuum_loads.py',
        'docs/SUCTION_S5_METHOD.md',
        'src/climbot_gazebo/src/vacuum_system.cpp',
        'src/climbot_gazebo/src/vacuum_io.cpp',
        'src/climbot_gazebo/src/vacuum_core.cpp',
        'src/climbot_gazebo/src/vacuum_sim_probe.cpp',
        'src/climbot_gazebo/include/climbot_gazebo/vacuum_state.hpp',
        'src/climbot_gazebo/config/vacuum_runtime.yaml',
        'src/climbot_gazebo/config/vacuum_io.yaml',
        'src/climbot_gazebo/config/vacuum_physics.yaml',
        'src/climbot_description/config/vacuum_chassis.yaml',
        'src/climbot_gazebo/climbot_gazebo/vacuum_assets.py'})
    revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    write(args.output / 'source_manifest.json', {
        'revision': revision,
        'source_sha256': {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
                          for name in source_paths},
        'legacy_freeze': {'status': 'ended at S7; old chassis removed in merge ea7de8f',
                          'historical_check': 'tools/freeze_suction_baseline.py'},
        'tracked_dirty': bool(subprocess.check_output(
            ['git', 'status', '--porcelain', '--untracked-files=no'], cwd=ROOT)),
        'probe_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
        'plugin_sha256': hashlib.sha256(
            (prefix / 'lib/libclimbot_vacuum_system.so').read_bytes()).hexdigest(),
        'stage': args.stage, 'diagnostic_subset': bool(args.limit or args.names)})
    results = {}
    for spec in specs:
        results[spec['name']] = run_case(args.output, spec, binary, env, identity)
        write(args.output / 'progress.json', {'completed': len(results), 'total': len(specs)})
    checks = {name + '/' + key: value for name, result in results.items()
              for key, value in result['checks'].items()}
    if {s['name'] for s in cases('bench')} <= set(results):
        for reverse, stopped in itertools.product([0., .05, .2], [1, 2, 3]):
            pressure = [results[f'bench_r{reverse:g}_a{area}_off{stopped}']['steady_delta_pa']
                        for area in [0, 40, 160, 400]]
            checks[f'port_monotonic_r{reverse:g}_off{stopped}'] = bool(
                np.all(np.diff(pressure) <= .01))
        for area, stopped in itertools.product([0, 40, 160, 400], [1, 2, 3]):
            pressure = [results[f'bench_r{r:g}_a{area}_off{stopped}']['steady_delta_pa']
                        for r in [0., .05, .2]]
            checks[f'reverse_monotonic_a{area}_off{stopped}'] = bool(
                np.all(np.diff(pressure) <= .01))
    if 'numeric_0.001' in results and 'numeric_0.0005' in results:
        a = results['numeric_0.001']
        b = results['numeric_0.0005']
        checks['steady_step_convergence'] = abs(
            a['steady_delta_pa'] - b['steady_delta_pa']) <= max(
                .01, .02 * abs(b['steady_delta_pa']))
        if a['response'] and b['response']:
            checks['fan_response_step_convergence'] = abs(
                a['response']['t10_90_s'] - b['response']['t10_90_s']
            ) <= .02 * abs(b['response']['t10_90_s'])
    write(args.output / 'summary.json', {
        'cases': results, 'checks': checks, 'all_passed': all(checks.values())})
    print(json.dumps({'cases': len(results), 'checks': len(checks),
                     'failed': [k for k, v in checks.items() if not v]}))
    return 0 if all(checks.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
