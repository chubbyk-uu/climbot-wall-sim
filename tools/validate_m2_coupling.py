#!/usr/bin/env python3
"""Independent M2 flow and per-wheel checks; raw failures are never averaged away."""

import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path

from climbot_description.vacuum_loads import minimum_pressure
from m2_load_criterion import evaluate as evaluate_loads
from m2_load_criterion import resolved_mu
import numpy as np
from probe_m2_robot_patch import gap_steps
from validate_vacuum_motion import frame_data
from validate_vacuum_physics import reference_pressure, reference_rate


def case_metrics(root):
    resolved = json.loads((root / 'resolved.json').read_text())
    c = frame_data(root / 'native/frames.csv')
    mu = resolved_mu(resolved)
    t = c['sim_time_s']
    release = float(c['vac_release_time_s'][c['vac_fixture_active'] == 0][0])
    ids = np.flatnonzero((t >= release + 1) & (t <= release + 6))
    normals = np.column_stack([c[s + '_fx'] for s in ['left', 'right', 'ball']])
    tangent = np.column_stack([np.hypot(c[s + '_fy'], c[s + '_fz'])
                               for s in ['left', 'right']])
    required = np.maximum(5, 1.5 * tangent / mu)
    deficient = normals[:, :2] < required
    native = {}
    for i, s in enumerate(['left', 'right']):
        lost = c[s + '_points'] == 0
        transient = lost.copy()
        for offset in [-2, -1, 1, 2]:
            transient |= np.roll(lost, offset)
        native[s] = {
            'raw_minimum_normal_n': float(normals[ids, i].min()),
            'raw_minimum_margin': float((mu * normals[ids, i] /
                                        np.maximum(tangent[ids, i], 1e-12)).min()),
            'raw_requirement_violation_frames': int(deficient[ids, i].sum()),
            'longest_requirement_violation_steps': gap_steps(~deficient[ids, i]),
            'violation_frames_outside_loss_and_two_neighbour_steps': int(
                (deficient[ids, i] & ~transient[ids]).sum()),
        }
    # Recompute an explicitly conservative envelope, without changing fan curves
    # or replacing it with the slower commanded motion after inspecting results.
    source = Path('results/m2_review_2026-10-19/physical_model/summary.json')
    state = np.array(json.loads(source.read_text())['nominal']['worst']['state'])
    model = deepcopy(resolved['model'])
    # +/-2 mm covers nominal tyre displacement in its measured 0..300 N range.
    # The unshifted rigid model is retained as one of the bounding inputs.
    pressure_bounds = []
    for height_shift in [-.002, 0, .002]:
        model['centre_xyz_m'][2] = resolved['model']['centre_xyz_m'][2] + height_shift
        pressure_bounds.append(float(minimum_pressure(
            model, state[None, :], mu=mu, lateral='conservative')[0]))
    minimum = max(pressure_bounds)
    pressure = c['vac_delta_pa'][ids]
    sample = ids[::50]
    physics = resolved['physics']
    independent = np.array([reference_rate(
        c['vac_delta_pa'][i], [c[f'vac_fan_speed_{j}'][i] for j in range(3)],
        c['vac_leak_area_m2'][i], physics) for i in sample])
    flow_error = np.max(np.abs(independent - c['vac_mass_flow_residual_kg_s'][sample]))
    reserve = np.array([-reference_rate(
        minimum, [c[f'vac_fan_speed_{j}'][i] for j in range(3)],
        c['vac_leak_area_m2'][i], physics) for i in sample])
    hold = np.flatnonzero(t >= release + 7)
    hold_metrics = None
    if len(hold):
        h = hold[::100]
        equilibrium = np.array([reference_pressure(
            [c[f'vac_fan_speed_{j}'][i] for j in range(3)],
            c['vac_leak_area_m2'][i], physics) for i in h])
        hold_metrics = {
            'duration_s': float(t[hold[-1]] - t[hold[0]]),
            'maximum_equilibrium_error_pa': float(np.max(np.abs(
                equilibrium - c['vac_delta_pa'][h]))),
            'minimum_normals_n': normals[hold].min(axis=0).tolist(),
            'each_drive_meets_raw_demand': bool(np.all(~deficient[hold])),
            'maximum_stop_reaction_n': float(c['vac_skirt_stop_previous_normal_n'][hold].max()),
        }
    channels = c['vac_surface_channel_area_m2'][ids]
    leak = c['vac_leak_area_m2'][ids]
    load_acceptance = evaluate_loads(c, release, mu=mu)
    raw_checks = {
        'each_drive_raw_frame_meets_demand': bool(np.all(~deficient[ids])),
        'ball_raw_floor': bool(np.all(normals[ids, 2] >= 10)),
    }
    checks = {
        'no_runtime_failure': not bool(np.any(c['vac_runtime_failed'])),
        'no_startup_failure': not bool(np.any(c['vac_startup_failed'])),
        'pressure_meets_conservative_envelope': bool(np.all(pressure >= minimum)),
        'flow_reserve_at_required_pressure': bool(reserve.min() >= 0),
        'independent_flow_equation': bool(flow_error <= 1e-10),
        'load_criterion_v2': load_acceptance['passed'],
        'no_stop_contact': bool(np.all(c['vac_skirt_stop_previous_normal_n'][ids] == 0)),
    }
    cpu_us = c['vac_cpu_ns'][ids] / 1000
    cpu_summary = [float(cpu_us.mean()), *np.percentile(cpu_us, [95, 99]).tolist()]
    return {
        'checks': checks, 'release_s': release, 'native_loads': native,
        'load_acceptance': load_acceptance, 'raw_checks': raw_checks,
        'strict_raw_load_checks_pass': all(raw_checks.values()),
        'pressure_range_pa': [float(pressure.min()), float(pressure.max())],
        'conservative_pressure_bounds_pa': pressure_bounds,
        'conservative_required_pressure_pa': minimum,
        'minimum_pressure_slack_pa': float(pressure.min() - minimum),
        'channel_area_range_mm2': (np.array([channels.min(), channels.max()]) * 1e6).tolist(),
        'total_leak_area_range_mm2': (np.array([leak.min(), leak.max()]) * 1e6).tolist(),
        'independent_mass_rate_error_kg_s': float(flow_error),
        'minimum_flow_reserve_at_requirement_kg_s': float(reserve.min()),
        'plugin_cpu_us_mean_p95_p99': cpu_summary,
        'channel_query_cpu_us_mean': float(c['vac_surface_query_ns'][ids].mean() / 1000),
        'hold': hold_metrics,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-root', required=True, type=Path)
    parser.add_argument('--case', action='append', required=True)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--loads-only', action='store_true',
                        help='reassess saved forces without recomputing pressure subchecks')
    args = parser.parse_args()
    if args.loads_only:
        cases = {}
        for name in args.case:
            path = args.data_root / name / 'native/frames.csv'
            c = frame_data(path)
            released = c['vac_fixture_active'] == 0
            if not released.any():
                raise ValueError(f'{name}: fixture never released')
            release = float(c['vac_release_time_s'][released][0])
            cases[name] = evaluate_loads(c, release, mu=resolved_mu(
                json.loads((args.data_root/name/'resolved.json').read_text())))
            cases[name]['frames_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
        result = {'scope': 'M2 load subcheck only; original samples retained', 'cases': cases}
    else:
        result = {'scope': 'T2 subchecks; strict per-frame failures remain explicit',
                  'cases': {name: case_metrics(args.data_root / name) for name in args.case}}
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    for name, c in result['cases'].items():
        summary = {'case': name, 'checks': c['checks'],
                   'load_passed': c.get('passed', c.get('load_acceptance', {}).get('passed'))}
        if not args.loads_only:
            summary.update(pressure_slack_pa=c['minimum_pressure_slack_pa'],
                           loads=c['native_loads'], raw_checks=c['raw_checks'], hold=c['hold'])
        print(json.dumps(summary))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
