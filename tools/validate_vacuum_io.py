#!/usr/bin/env python3
"""S4 native sensor and fault acceptance, preserving the S3 evidence untouched."""
import argparse
import csv
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import uuid

from ament_index_python.packages import get_package_prefix
from climbot_gazebo.mesa_runtime import rendering_environment
from climbot_gazebo.vacuum_assets import build_assets
import numpy as np
import yaml


def write(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')


def read_frames(path):
    with path.open() as stream:
        names = next(csv.reader(stream))
        data = np.loadtxt(stream, delimiter=',', ndmin=2)
    return {name: data[:, i] for i, name in enumerate(names)}


def delivered(c):
    return np.flatnonzero(
        (c['paused'] == 0) & (c['vac_sensor_stamp_s'] >= 0) &
        np.r_[True, np.diff(c['vac_sensor_delivery_s']) > 1e-8])


def sensor_metrics(c):
    indices = delivered(c)
    truth = {(int(g), round(t, 8)): p for g, t, p in zip(
        c['vac_generation'], c['vac_time_s'], c['vac_cavity_pa'])}
    errors = [c['vac_sensor_value_pa'][i] - truth[
        (int(c['vac_generation'][i]), round(c['vac_sensor_stamp_s'][i], 8))]
        for i in indices]
    return {'delivered': len(indices), 'error_mean_pa': float(np.mean(errors)),
            'error_variance_pa2': float(np.var(errors)),
            'maximum_delay_error_s': float(np.max(np.abs(
                c['vac_sensor_delivery_s'][indices] - c['vac_sensor_stamp_s'][indices] - .02))),
            'all_stamps_on_10ms_grid': bool(np.all(np.abs(
                c['vac_sensor_stamp_s'][indices] * 100 -
                np.round(c['vac_sensor_stamp_s'][indices] * 100)) < 1e-7)),
            'end_delta_pa': float(c['vac_delta_pa'][-1]),
            'end_fan_speeds': [float(c[f'vac_fan_speed_{i}'][-1]) for i in range(3)],
            'command_revision_max': int(np.max(c['vac_command_revision'])),
            'sensor_fault_modes': sorted({int(x) for x in c['vac_sensor_fault']}),
            'runtime_failed': bool(np.any(c['vac_runtime_failed']))}


def plot_evidence(out, frames):
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt

    figure, axes = plt.subplots(3, 1, figsize=(10, 10), constrained_layout=True)
    c = frames['normal']
    d = delivered(c)
    axes[0].plot(c['vac_time_s'], c['vac_delta_pa'] / 1000, label='physical truth')
    axes[0].plot(c['vac_sensor_delivery_s'][d],
                 (101325 - c['vac_sensor_value_pa'][d]) / 1000,
                 '.', ms=2, label='delivered measurement (20 ms latency)')
    axes[0].set(xlim=(0, 1.4), ylabel='pressure difference (kPa)',
                title='S4: simulated sensor, three fixed-setting fans, flat wall')
    c = frames['fans']
    for i in range(3):
        axes[1].plot(c['vac_time_s'], c[f'vac_fan_speed_{i}'], label=f'fan {i}')
    axes[1].set(xlim=(0, 1.4), ylabel='normalized fan speed',
                title='Independent stop/failure and recovery; port backflow retained')
    c = frames['sensor_faults']
    d = delivered(c)
    axes[2].plot(c['vac_time_s'], c['vac_cavity_pa'], label='physical truth')
    axes[2].plot(c['vac_sensor_delivery_s'][d], c['vac_sensor_value_pa'][d],
                 '.', ms=3, label='measurement')
    axes[2].axvspan(.3, .6, alpha=.15, color='red', label='dropout')
    axes[2].axvspan(.9, 1.2, alpha=.15, color='orange', label='stuck value')
    axes[2].set(xlim=(.2, 1.4), ylim=(92500, 98000),
                ylabel='cavity absolute pressure (Pa)', xlabel='simulation time (s)')
    for axis in axes:
        axis.grid(alpha=.2)
        axis.legend(loc='best')
    figure.savefig(out / 'sensor_and_fans.png', dpi=160)
    figure.savefig(out / 'sensor_and_fans.svg', metadata={'Date': None})
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    out = args.output
    revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip()
    dirty = bool(subprocess.check_output(['git', 'status', '--porcelain', '--untracked-files=no']))
    prefix = Path(get_package_prefix('climbot_gazebo'))
    env, mesa = rendering_environment()
    env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = os.pathsep.join(filter(None, [
        str(prefix / 'lib'), env.get('GZ_SIM_SYSTEM_PLUGIN_PATH')]))
    fan_events = [{'time_s': t, 'kind': kind, 'index': i, 'value': value}
                  for t, kind, i, value in [
                      (.15, 1, 0, 1), (.30, 1, 0, 0),
                      (.45, 0, 1, 0), (.60, 0, 1, 1),
                      (.75, 1, 2, 1), (.90, 1, 2, 0)]]
    faults = [{'time_s': t, 'kind': 2, 'index': 0, 'value': value}
              for t, value in [(.3, 1), (.6, 0), (.9, 2), (1.2, 0)]]
    specs = [
        ('normal', {}, {}), ('repeat', {}, {}),
        ('zero_error', {'sensor': {'noise_pa': 0, 'bias_pa': 0}}, {}),
        ('sensor_faults', {'events': faults}, {}),
        ('fans', {'events': fan_events}, {}),
        ('fixed_fans', {'events': fan_events}, {'mode': 'fixed_pressure'}),
        ('lifecycle', {'events': fan_events + [
            {'time_s': 1.35, 'kind': 2, 'index': 0, 'value': 1},
            {'time_s': 1.7, 'kind': 2, 'index': 0, 'value': 0}]}, {'lifecycle': True}),
        ('saturation', {'sensor': {'minimum_pa': 95000, 'maximum_pa': 100000}}, {}),
        ('half_step', {}, {'step_s': .0005}),
    ]
    frames = {}
    summaries = {}
    checks = {}
    for name, io, options in specs:
        directory = out / name
        opts = {k: v for k, v in options.items() if k != 'lifecycle'}
        build_assets(directory, realtime_factor=0, io_overrides=io, **opts)
        step = opts.get('step_s', .001)
        request = {'world': 'world.sdf', 'step_s': step, 'duration_s': 4}
        if options.get('lifecycle'):
            request.update(pause_at_s=.3, pause_iterations=20, reset_at_s=1.5)
        (directory / 'request.yaml').write_text(yaml.safe_dump(request))
        env['GZ_PARTITION'] = 'climbot_s4_' + uuid.uuid4().hex
        log = Path('/tmp') / ('climbot_s4_' + name + '_' + uuid.uuid4().hex + '.log')
        with log.open('w') as stream:
            result = subprocess.run([str(prefix / 'lib/climbot_gazebo/vacuum_sim_probe'),
                                     str(directory / 'request.yaml'), str(directory / 'native')],
                                    env=env, stdout=stream, stderr=subprocess.STDOUT, timeout=90)
        if result.returncode:
            raise RuntimeError(f'{name} failed: {log}')
        c = read_frames(directory / 'native/frames.csv')
        frames[name] = c
        metrics = sensor_metrics(c)
        summaries[name] = metrics
        checks[name + '_no_runtime_failure'] = not metrics['runtime_failed']
        checks[name + '_timestamp_grid'] = metrics['all_stamps_on_10ms_grid']
        checks[name + '_delay'] = metrics['maximum_delay_error_s'] < 1e-8
        libraries = json.loads((directory / 'native/runtime_libraries.json').read_text())
        checks[name + '_mesa'] = (mesa['mesa'] != 'private' or (
            libraries['private_mapping_complete'] and not libraries['system_gallium_mapped']))
        if options.get('lifecycle'):
            events = json.loads((directory / 'native/run.json').read_text())
            checks['pause_unchanged'] = events['pause_state_equal'] and events['pause_pose_equal']
            checks['reset_restored'] = events['reset_restored']
            for generation in [0, 1]:
                same = (c['vac_generation'] == generation) & (c['paused'] == 0)
                first = np.flatnonzero(same)[0]
                checks[f'reset_generation_{generation}_clears_commands'] = bool(
                    c['vac_command_revision'][first] == 0 and c['vac_sensor_sequence'][first] == 0)
            a = delivered(c)
            zero = a[c['vac_generation'][a] == 0]
            one = a[c['vac_generation'][a] == 1]
            n = min(len(zero), len(one))
            # Compare random error after subtracting independently recorded physical input.

            def errors(indices):
                lookup = {(int(g), round(t, 8)): p for g, t, p in zip(
                    c['vac_generation'], c['vac_time_s'], c['vac_cavity_pa'])}
                return np.array([c['vac_sensor_value_pa'][i] - lookup[(
                    int(c['vac_generation'][i]), round(c['vac_sensor_stamp_s'][i], 8))]
                    for i in indices])
            checks['reset_random_sequence_replays'] = bool(np.allclose(
                errors(zero[:n]), errors(one[:n]), rtol=0, atol=1e-9))
            checks['reset_scheduled_commands_replay'] = bool(
                np.max(c['vac_command_revision'][c['vac_generation'] == 0]) == 7 and
                np.max(c['vac_command_revision'][c['vac_generation'] == 1]) == 8)
            before_reset = np.flatnonzero((c['vac_generation'] == 0) & (c['paused'] == 0))[-1]
            after_reset = np.flatnonzero((c['vac_generation'] == 1) & (c['paused'] == 0))[0]
            checks['reset_clears_active_sensor_fault'] = bool(
                c['vac_sensor_fault'][before_reset] == 1 and
                c['vac_sensor_fault'][after_reset] == 0)
        write(directory / 'metrics.json', metrics)
        for path in (directory / 'native').glob('*.csv'):
            with path.open('rb') as src, gzip.open(str(path) + '.gz', 'wb') as dst:
                shutil.copyfileobj(src, dst)
            path.unlink()
    base = frames['normal']
    checks['fixed_seed_identical_sensor'] = bool(np.array_equal(
        base['vac_sensor_value_pa'], frames['repeat']['vac_sensor_value_pa']))
    checks['bias_mean_within_2Pa'] = abs(summaries['normal']['error_mean_pa'] - 25) < 2
    checks['variance_within_15percent'] = abs(summaries['normal']['error_variance_pa2'] - 100) < 15
    checks['zero_error_matches_physics'] = abs(summaries['zero_error']['error_mean_pa']) < 1e-9
    # All native mechanics/pressure fields must be identical when only sensor behaviour changes.
    excluded = {'vac_cpu_ns', 'vac_command_revision'}
    physical = [k for k in base if not k.startswith('vac_sensor_') and k not in excluded]
    for name in ['repeat', 'zero_error', 'sensor_faults', 'saturation']:
        checks[name + '_physics_isolated'] = all(
            np.array_equal(base[k], frames[name][k]) for k in physical)
    c = frames['sensor_faults']
    d = delivered(c)
    time = c['vac_time_s']
    checks['dropout_suppresses_and_flushes'] = not np.any((time[d] >= .3) & (time[d] < .62))
    stuck = d[(time[d] >= .92) & (time[d] < 1.2)]
    checks['stuck_value_fresh_stamps'] = bool(
        len(stuck) > 10 and np.ptp(c['vac_sensor_value_pa'][stuck]) == 0 and
        np.min(np.diff(c['vac_sensor_stamp_s'][stuck])) > .0099)
    checks['sensor_recovers'] = bool(len(d[time[d] > 1.22]) > 100)
    c = frames['fans']
    time = c['vac_time_s']
    for i, start in [(0, .15), (1, .45), (2, .75)]:
        before = np.flatnonzero(time < start)[-1]
        end = np.flatnonzero(time < start + .15)[-1]
        checks[f'fan_{i}_spins_down_independently'] = bool(
            c[f'vac_fan_speed_{i}'][end] < c[f'vac_fan_speed_{i}'][before] and
            all(c[f'vac_fan_speed_{j}'][end] > c[f'vac_fan_speed_{j}'][before]
                for j in range(3) if j != i))
    checks['fan_commands_change_cavity'] = bool(np.max(np.abs(
        c['vac_cavity_pa'] - base['vac_cavity_pa'])) > 100)
    checks['fan_recovery'] = bool(np.min([c[f'vac_fan_speed_{i}'][-1] for i in range(3)]) > .999)
    c = frames['fixed_fans']
    checks['B_ideal_pressure_ignores_fan_fault_as_declared'] = bool(
        np.ptp(c['vac_delta_pa']) < 1e-8)
    c = frames['saturation']
    d = delivered(c)
    checks['range_saturation_reported'] = bool(
        np.any(c['vac_sensor_saturated'][d]) and
        np.min(c['vac_sensor_value_pa'][d]) >= 95000 and
        np.max(c['vac_sensor_value_pa'][d]) <= 100000)
    a = delivered(base)
    c = frames['half_step']
    b = delivered(c)
    checks['half_step_same_acquisition_times'] = bool(np.allclose(
        base['vac_sensor_stamp_s'][a], c['vac_sensor_stamp_s'][b], rtol=0, atol=1e-10))

    def noise_errors(data, indices):
        lookup = {round(t, 8): p for t, p in zip(data['vac_time_s'], data['vac_cavity_pa'])}
        return [data['vac_sensor_value_pa'][i] - lookup[round(data['vac_sensor_stamp_s'][i], 8)]
                for i in indices]

    checks['half_step_same_random_errors'] = bool(np.allclose(
        noise_errors(base, a), noise_errors(c, b), rtol=0, atol=1e-9))
    plot_evidence(out, frames)
    write(out / 'summary.json', {
        'cases': summaries, 'checks': checks,
        'all_passed': all(checks.values()), 'mesa': mesa})
    binaries = [('plugin', 'lib/libclimbot_vacuum_system.so'),
                ('probe', 'lib/climbot_gazebo/vacuum_sim_probe')]
    write(out / 'source_manifest.json', {
        'revision': revision, 'tracked_dirty': dirty,
        'binary_sha256': {name: hashlib.sha256((prefix / rel).read_bytes()).hexdigest()
                          for name, rel in binaries}})
    print(json.dumps({'cases': len(specs), 'checks': len(checks),
                     'failed': [k for k, v in checks.items() if not v]}))
    return 0 if all(checks.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
