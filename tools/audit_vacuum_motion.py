#!/usr/bin/env python3
"""Independently audit S5 recorded contacts, phase windows and pressure states."""
import argparse
import csv
import gzip
import hashlib
import json
from pathlib import Path

import matplotlib
import numpy as np
import yaml


def write(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')


def audit(directory, spec):
    with gzip.open(directory / 'native/frames.csv.gz', 'rt') as stream:
        names = next(csv.reader(stream))
        data = np.loadtxt(stream, delimiter=',', ndmin=2)
    c = {name: data[:, i] for i, name in enumerate(names)}
    result = {
        'frames': len(data), 'all_finite': bool(np.isfinite(data).all()),
        'maximum_absolute_pressure_identity_error_pa': float(np.max(np.abs(
            c['vac_cavity_pa'] + c['vac_delta_pa'] - c['vac_ambient_pa']))),
        'all_samples_1_generation': bool(np.all(c['vac_generation'] == 0)),
        'all_loaded_mass_kg': [float(c['mass_kg'].min()), float(c['mass_kg'].max())],
    }
    if spec['kind'] == 'motion':
        request = yaml.safe_load((directory / 'request.yaml').read_text())
        released = c['vac_release_time_s'].max()
        time = c['sim_time_s']
        end = released + request['motion_profile'][-1]['after_release_s']
        mask = (time >= released + 2) & (time <= end + 1e-9)
        normal = np.column_stack([c[s + '_fx'][mask] for s in ['left', 'right', 'ball']])
        tangent = np.column_stack([np.hypot(c[s + '_fy'][mask], c[s + '_fz'][mask])
                                   for s in ['left', 'right']])
        with gzip.open(directory / 'inverse_dynamics.csv.gz', 'rt') as stream:
            inverse = np.loadtxt(stream, delimiter=',', skiprows=1, ndmin=2)
        profile = request['motion_profile']
        knots = [p['after_release_s'] for p in profile]
        expected_v = np.interp(time - released, knots, [p['linear_mps'] for p in profile])
        expected_w = np.interp(time - released, knots, [p['angular_rps'] for p in profile])
        result.update({
            'scored_samples': int(mask.sum()),
            'minimum_native_friction_margin': float(np.min(
                1.1 * normal[:, :2] / np.maximum(tangent, 1e-12))),
            'native_drive_requirement_pass': bool(np.all(
                normal[:, :2] >= np.maximum(5, 1.5 * tangent / 1.1))),
            'native_ball_floor_pass': bool(np.all(normal[:, 2] >= 10)),
            'all_fixture_wrench_zero': all(
                np.all(c['vac_fixture_' + kind + axis][mask] == 0)
                for kind in 'fm' for axis in 'xyz'),
            'inverse_rows_same_native_time': bool(
                len(inverse) == mask.sum() and
                np.allclose(inverse[:, 0], time[mask], rtol=0, atol=1e-10)),
            'raw_inverse_pressure_slack_pa': float(np.min(inverse[:, 7] - inverse[:, 6])),
            'maximum_command_error_mps_rps': float(max(
                np.max(np.abs(c['command_v_mps'] - expected_v)),
                np.max(np.abs(c['command_omega_rps'] - expected_w)))),
            'maximum_head_margin_pa': float(np.max(
                c['vac_maximum_fan_head_pa'][mask] - c['vac_delta_pa'][mask])),
        })
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batch', type=Path)
    args = parser.parse_args()
    root = args.batch
    summary = json.loads((root / 'summary.json').read_text())
    specs = json.loads((root / 'cases.json').read_text())
    audits = {}
    checks = {}
    for spec in specs:
        value = audit(root / spec['name'], spec)
        audits[spec['name']] = value
        checks[spec['name'] + '/finite_and_pressure_identity'] = (
            value['all_finite'] and value['maximum_absolute_pressure_identity_error_pa'] < 1e-9)
        if spec['kind'] == 'motion':
            checks[spec['name'] + '/raw_phase_contact_and_command_audit'] = all([
                value['native_drive_requirement_pass'], value['native_ball_floor_pass'],
                value['all_fixture_wrench_zero'], value['inverse_rows_same_native_time'],
                value['raw_inverse_pressure_slack_pa'] >= 0,
                value['maximum_command_error_mps_rps'] < 1e-9])
    write(root / 'independent_audit.json', {
        'audit_script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'native_source_revision': json.loads(
            (root / 'source_manifest.json').read_text())['revision'],
        'cases': audits, 'checks': checks, 'all_passed': all(checks.values())})
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(13, 5), constrained_layout=True)
    for profile in ['straight', 'turn', 'mixed']:
        headings = list(range(0, 360, 45))
        for mode, style in [('fixed_pressure', '--'), ('fan_ode', '-')]:
            margins = [summary['cases'][f'{profile}_{h:03}_{mode}']['minimum_friction_margin']
                       for h in headings]
            axes[0].plot(headings, margins, style, marker='.', label=profile + '/' + mode)
    axes[0].axhline(1.5, color='red', label='required 1.5')
    axes[0].set(xlabel='initial heading (deg)', ylabel='minimum measured wheel margin',
                title='S5 normal motion: flat wall, rigid wheels')
    for reverse in [0., .05, .2]:
        for stopped in [1, 2, 3]:
            areas = [0, 40, 160, 400]
            pressure = [summary['cases'][f'bench_r{reverse:g}_a{a}_off{stopped}'][
                'steady_delta_pa'] / 1000 for a in areas]
            axes[1].plot(areas, pressure, marker='.', label=f'reverse={reverse:g}, off={stopped}')
    axes[1].set(xlabel='independent port area (mm²); base=20 mm²',
                ylabel='steady pressure difference (kPa)', title='Constrained pressure bench')
    for ax in axes:
        ax.grid(alpha=.2)
        ax.legend(fontsize=7)
    fig.savefig(root / 'motion_and_failure_boundary.png', dpi=160)
    fig.savefig(root / 'motion_and_failure_boundary.svg', metadata={'Date': None})
    plt.close(fig)
    print(json.dumps({'audited_cases': len(audits), 'checks': len(checks),
                      'failed': [k for k, v in checks.items() if not v]}))
    return 0 if all(checks.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
