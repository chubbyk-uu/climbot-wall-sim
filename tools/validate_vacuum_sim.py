#!/usr/bin/env python3
"""S3 native flat-wall integration evidence; no camera or motion acceptance."""
import argparse
import csv
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
import uuid

import numpy as np
import yaml

from ament_index_python.packages import get_package_prefix
from climbot_gazebo.mesa_runtime import rendering_environment
from climbot_gazebo.vacuum_assets import build_assets
from design_vacuum_chassis import inputs, profiles
from freeze_suction_baseline import legacy_input_path, verify_legacy_inputs
from vacuum_layout import build
from vacuum_loads import evaluate, minimum_pressure, states

ROOT = Path(__file__).resolve().parents[1]


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def recalculate_design(out):
    """Keep historical allocation numbers and the revised envelope explicit."""
    config, robot, camera, control, _, gravity = inputs()
    model = build(config, camera, robot)
    cases = profiles(control, robot, model['track_m'])
    rows = []
    worst = {}
    with gzip.open(out / 'revised_design_rows.csv.gz', 'wt', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['profile', 'allocation', 'heading_deg', 'minimum_pressure_pa'])
        for allocation in ['normal_proportional', 'equal', 'conservative']:
            best = None
            for profile in cases:
                kwargs = {key: profile[key] for key in ['v', 'a', 'omega', 'alpha']}
                state = states(np.arange(360), **kwargs)
                pressure = minimum_pressure(model, state, gravity=gravity, lateral=allocation)
                for h, p in zip(state[:, 0], pressure):
                    writer.writerow([profile['name'], allocation, h, p])
                peak = state[int(np.argmax(pressure)), 0]
                fine = states(np.arange(peak-1, peak+1.001, .1) % 360, **kwargs)
                refined = minimum_pressure(model, fine, gravity=gravity, lateral=allocation)
                index = int(np.argmax(refined))
                item = {'profile': profile['name'], 'state': fine[index].tolist(),
                        'minimum_pressure_pa': float(refined[index]),
                        'pressure_force_n': float(refined[index] * model['effective_area_m2'])}
                rows.append({'allocation': allocation, **item})
                if best is None or item['minimum_pressure_pa'] > best['minimum_pressure_pa']:
                    best = item
            worst[allocation] = best
    result = {'profiles': len(cases), 'gravity_mps2': gravity,
              'area_m2': model['effective_area_m2'], 'worst': worst,
              'interpretation': 'Equal and proportional allocation envelope; motion still requires S5 validation.'}
    write(out / 'revised_design.json', result)
    write(out / 'profile_peaks.json', rows)
    return result


def summarize(directory, resolved):
    with (directory / 'frames.csv').open() as stream:
        names = next(csv.reader(stream))
        data = np.loadtxt(stream, delimiter=',', ndmin=2)
    columns = {name: data[:, i] for i, name in enumerate(names)}
    c = columns
    active = c['paused'] == 0
    released = active & (c['vac_fixture_active'] == 0)
    if not released.any():
        return {'released': False, 'startup_failed': bool(c['vac_startup_failed'][-1]),
                'runtime_failed': bool(c['vac_runtime_failed'][-1])}
    release = float(c['vac_release_time_s'][released][0])
    steady = released & (c['sim_time_s'] >= release + 2) & (c['sim_time_s'] < release + 62)
    model = resolved['model']
    state = states(np.full(released.sum(), resolved['heading_deg']))
    predicted = evaluate(model, state, c['vac_delta_pa'][released], lateral='conservative')
    normals = np.column_stack([c[s + '_fx'] for s in ['left', 'right', 'ball']])
    tangent = np.column_stack([np.hypot(c[s + '_fy'], c[s + '_fz'])
                              for s in ['left', 'right']])
    required = 1.5 * predicted['drive_demand_n'] / 1.1
    force = np.zeros((len(data), 3))
    moment = np.zeros_like(force)
    for source in ['pressure', 'skirt', 'fixture']:
        force += np.column_stack([c[f'vac_{source}_f{x}'] for x in 'xyz'])
        moment += np.column_stack([c[f'vac_{source}_m{x}'] for x in 'xyz'])
    origin = np.column_stack([c['pose_' + x] for x in 'xyz'])
    moment += np.cross(origin, force)
    for side in ['left', 'right', 'ball']:
        f = np.column_stack([c[f'{side}_f{x}'] for x in 'xyz'])
        force += f
        moment += np.column_stack([c[f'{side}_m{x}'] for x in 'xyz']) + np.cross(origin, f)
    gravity = np.zeros_like(force);gravity[:, 2] = -model['mass_kg'] * 9.81
    force += gravity
    cg = np.column_stack([c['cg_' + x] for x in 'xyz'])
    moment += np.cross(cg, gravity)
    moment += np.column_stack([c['vac_rolling_m' + x] for x in 'xyz'])
    # Native P/H are independently summed over all loaded inertial links.
    # Differences have cache quantization; report both instantaneous and
    # integrated balances, never tune a contact-force scale to fit them.
    momentum = np.column_stack([c['p' + x] for x in 'xyz'])
    angular = np.column_stack([c['h' + x] for x in 'xyz'])
    indices = np.flatnonzero(steady)
    dt = resolved['step_s']
    impulse = force[indices[1:]].sum(axis=0) * dt
    angular_impulse = moment[indices[1:]].sum(axis=0) * dt
    dp = momentum[indices[-1]] - momentum[indices[0]]
    dh = angular[indices[-1]] - angular[indices[0]]
    expected = round(60 / dt)
    floor = np.array([5., 5., 10.])
    delta_pose = origin[steady] - origin[steady][0]
    result = {
        'released': True, 'release_s': release, 'native_samples': int(active.sum()),
        'steady_samples': int(steady.sum()), 'expected_steady_samples': expected,
        'steady_normal_mean_n': normals[steady].mean(axis=0).tolist(),
        'normal_min_since_release_n': normals[released].min(axis=0).tolist(),
        'normal_prediction_max_error_n': float(np.abs(
            normals[released] - predicted['normals_n']).max()),
        'drive_requirement_min_slack_n': float((normals[released, :2] - required).min()),
        'native_friction_margin_min': float((1.1 * normals[released, :2] /
                                            np.maximum(tangent[released], 1e-20)).min()),
        'steady_delta_pa': float(c['vac_delta_pa'][steady].mean()),
        'minimum_pressure_pa': float(minimum_pressure(
            model, states([resolved['heading_deg']]), lateral='conservative')[0]),
        'steady_force_residual_mean_n': force[steady].mean(axis=0).tolist(),
        'steady_world_moment_residual_mean_nm': moment[steady].mean(axis=0).tolist(),
        'steady_force_residual_max_n': float(np.linalg.norm(force[steady], axis=1).max()),
        'linear_impulse_residual_ns': (impulse - dp).tolist(),
        'angular_impulse_residual_nms': (angular_impulse - dh).tolist(),
        'steady_translation_range_m': float(np.linalg.norm(delta_pose, axis=1).max()),
        'maximum_tilt_gap_spread_m': float((c['vac_maximum_gap_m'][released] -
                                           c['vac_minimum_gap_m'][released]).max()),
        'minimum_body_gap_m': float(c['vac_minimum_gap_m'][released].min()),
        'plugin_cpu_p99_us': float(np.quantile(c['vac_cpu_ns'][released], .99) / 1000),
        'plugin_cpu_mean_us': float(c['vac_cpu_ns'][released].mean() / 1000),
        'pressure_substeps_max': float(c['vac_substeps'].max()),
        'raw_contact_header_stamp_all_zero': all(np.all(c[s + '_stamp_s'] == 0)
                                                for s in ['left', 'right', 'ball']),
    }
    pressure_force = c['vac_delta_pa'][steady] * c['vac_area_m2'][steady]
    fixture = [c[f'vac_fixture_{kind}{axis}'][released] for kind in 'fm' for axis in 'xyz']
    result['checks'] = {
        'all_physics_samples_present': int(active.sum()) == round(64 / dt),
        'unique_physics_timestamps': len(np.unique(c['sim_time_s'][active])) == int(active.sum()),
        'full_60s_steady_window': abs(int(steady.sum()) - expected) <= 1,
        'no_invalid_or_missing_contact': all(np.all(c[s + '_valid'][released] == 1) and
                                             np.all(c[s + '_points'][released] > 0)
                                             for s in ['left', 'right', 'ball']),
        'support_floors': bool(np.all(normals[released] >= floor)),
        'per_wheel_requirement': result['drive_requirement_min_slack_n'] >= 0,
        'native_traction_margin': result['native_friction_margin_min'] >= 1.5,
        'normal_balance_2pct': bool(abs(force[steady, 0].mean()) <= .02 * pressure_force.mean()),
        'moment_balance_2pct': bool(np.linalg.norm(moment[steady].mean(axis=0)) <=
                                   .02 * pressure_force.mean() * max(model['track_m'],
                                                                    abs(model['caster_x_m']))),
        'no_fixture_after_release': all(np.all(value == 0) for value in fixture),
        'pressure_above_static_requirement': bool(np.all(c['vac_delta_pa'][released] >=
                                                         result['minimum_pressure_pa'])),
        'body_clearance': result['minimum_body_gap_m'] >= .01,
        'no_runtime_or_startup_failure': not bool(np.any(c['vac_runtime_failed']) or
                                                 np.any(c['vac_startup_failed'])),
    }
    return result


def run_case(binary, out, tag, mode, heading, dt=.001, duration=64., overrides=None,
             events=None):
    case = out / tag;case.mkdir()
    resolved = build_assets(case, mode=mode, heading_deg=heading, step_s=dt,
                            realtime_factor=0, runtime_overrides=overrides)
    request = dict(world='world.sdf', step_s=dt, duration_s=duration, **(events or {}))
    (case / 'request.yaml').write_text(yaml.safe_dump(request))
    env, identity = rendering_environment()
    env['GZ_PARTITION'] = 'climbot_s3_' + uuid.uuid4().hex
    env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = str(Path(get_package_prefix('climbot_gazebo')) / 'lib') + ':' + env.get(
        'GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    log = Path('/tmp') / f'climbot_s3_{tag}_{time.time_ns()}.log'
    with log.open('w') as stream:
        completed = subprocess.run([str(binary), str(case / 'request.yaml'), str(case / 'native')],
                                   env=env, stdout=stream, stderr=subprocess.STDOUT,
                                   timeout=max(90, duration * 10))
    write(case / 'execution.json', {'exit_code': completed.returncode,
                                   'log_name': log.name, 'rendering': identity})
    if completed.returncode:
        raise RuntimeError(f'{tag} failed; diagnostic log {log}')
    libraries = json.loads((case / 'native/runtime_libraries.json').read_text())
    if identity['mesa'] == 'private' and (not libraries['private_mapping_complete'] or
                                         libraries['system_gallium_mapped']):
        raise RuntimeError('native process did not load the exclusive selected patched Mesa')
    if duration == 64:
        result = summarize(case / 'native', resolved)
    else:
        lifecycle = json.loads((case / 'native/run.json').read_text())
        result = {'lifecycle': lifecycle}
        if tag == 'lifecycle':
            result['checks'] = {
                'pause_state_frozen': lifecycle['pause_state_equal'],
                'paused_backend_pose_frozen': lifecycle['pause_pose_equal'],
                'reset_reinitializes': lifecycle['reset_restored'],
                'observer_survives_reset': lifecycle['reset_snapshots'] == 1 and
                    lifecycle['native_frames'] == round(duration / dt),
                'no_lifecycle_runtime_failure': not lifecycle['last_runtime_failed'],
            }
        else:
            with (case / 'native/frames.csv').open() as stream:
                rows = csv.DictReader(stream)
                held, never_released = True, True
                for row in rows:
                    held &= float(row['vac_fixture_active']) == 1
                    never_released &= float(row['vac_release_time_s']) == -1
            result['checks'] = {'timeout_explicit': lifecycle['last_startup_failed'],
                                'fixture_held_on_timeout': held and never_released,
                                'timeout_is_not_model_failure': not lifecycle['last_runtime_failed']}
    write(case / 'case_summary.json', result)
    for path in (case / 'native').glob('*.csv'):
        with path.open('rb') as source, gzip.open(str(path) + '.gz', 'wb', compresslevel=3) as target:
            shutil.copyfileobj(source, target)
        path.unlink()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, default=ROOT / 'install/climbot_gazebo/lib/climbot_gazebo/vacuum_sim_probe')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--headings', type=float, nargs='+', default=list(range(0, 360, 45)))
    args = parser.parse_args();out = args.output.resolve()
    if out.exists():raise ValueError('evidence directory already exists')
    out.mkdir(parents=True)
    summary = {}
    binary = args.binary.resolve()
    sources = ['tools/validate_vacuum_sim.py', 'tools/vacuum_loads.py',
               'tools/freeze_suction_baseline.py',
               'src/climbot_gazebo/src/vacuum_system.cpp',
               'src/climbot_gazebo/src/vacuum_sim_probe.cpp',
               'src/climbot_gazebo/climbot_gazebo/vacuum_assets.py',
               'src/climbot_gazebo/climbot_gazebo/mesa_runtime.py',
               'src/climbot_gazebo/config/vacuum_runtime.yaml',
               'src/climbot_gazebo/config/vacuum_physics.yaml',
               'src/climbot_description/config/vacuum_chassis.yaml',
               'src/climbot_gazebo/include/climbot_gazebo/vacuum_state.hpp',
               'src/climbot_gazebo/include/climbot_gazebo/vacuum_core.hpp',
               'src/climbot_gazebo/src/vacuum_core.cpp',
               'src/climbot_gazebo/launch/climbot_vacuum.launch.py',
               'tools/vacuum_layout.py', 'tools/vacuum_preview.py',
               'tools/design_vacuum_chassis.py', 'docs/SUCTION_S3_METHOD.md']
    legacy_acceptance = verify_legacy_inputs(ROOT)
    frozen = legacy_acceptance['byte_identical']
    sources += list(frozen)
    write(out / 'source_manifest.json', {
        'revision': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'dirty': bool(subprocess.check_output(['git', 'status', '--porcelain', '--untracked-files=no'], cwd=ROOT)),
        'sha256': {p: hashlib.sha256(legacy_input_path(ROOT, p).read_bytes()).hexdigest() for p in sources},
        'native_binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
        'legacy_inputs_byte_identical': frozen,
        'legacy_source_acceptance': legacy_acceptance,
    })
    recalculate_design(out)
    for h in args.headings:
        for mode in ['fixed_pressure', 'fan_ode']:
            tag = f'{mode}_h{h:g}'
            summary[tag] = run_case(binary, out, tag, mode, h)
            write(out / 'progress.json', summary)
    summary['half_step'] = run_case(binary, out, 'half_step', 'fan_ode', 45, dt=.0005)
    coarse = summary.get('fan_ode_h45')
    if coarse:
        fine = summary['half_step']
        fine['checks']['step_pressure_converges_2pct'] = abs(
            fine['steady_delta_pa'] / coarse['steady_delta_pa'] - 1) <= .02
        fine['checks']['step_wheel_loads_converge_2pct'] = bool(np.all(np.abs(
            np.asarray(fine['steady_normal_mean_n']) /
            np.asarray(coarse['steady_normal_mean_n']) - 1) <= .02))
    summary['lifecycle'] = run_case(binary, out, 'lifecycle', 'fan_ode', 0, duration=5,
                                  events={'pause_at_s': .3, 'pause_iterations': 100,
                                          'reset_at_s': 2.})
    summary['startup_timeout'] = run_case(binary, out, 'startup_timeout', 'fixed_pressure', 0,
                                         duration=10.5, overrides={'fixed_delta_pa': 100.})
    write(out / 's3_summary.json', summary)
    checks = [v for case in summary.values() for v in case.get('checks', {}).values()]
    write(out / 'validation.json', {'checks': len(checks), 'passed': sum(checks),
                                   'all_passed': bool(checks) and all(checks)})
    write(out / 'artifact_manifest.json', {
        str(p.relative_to(out)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted(out.rglob('*')) if p.is_file()})
    print(json.dumps({'cases': len(summary), 'checks': len(checks), 'passed': sum(checks)}))
    return 0 if checks and all(checks) else 2


if __name__ == '__main__':
    raise SystemExit(main())
