#!/usr/bin/env python3
"""Combine T2 evidence and check complete scored traces at the new pressure bound."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from validate_vacuum_motion import frame_data
from validate_vacuum_physics import read_physics, reference_rate


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def metrics(directory, required, physics):
    path = directory/'native/frames.csv'
    c = frame_data(path)
    release = c['vac_release_time_s'][c['vac_fixture_active'] == 0][0]
    scored = np.flatnonzero(c['sim_time_s'] >= release+1)
    reserve = [-reference_rate(required, [c[f'vac_fan_speed_{j}'][i] for j in range(3)],
                               c['vac_leak_area_m2'][i], physics) for i in scored]
    pressure = c['vac_delta_pa'][scored]
    hold = c['sim_time_s'] >= release+7
    common = hold & (c['sim_time_s'] <= release+8)
    force = sum(np.column_stack([c[s+'_f'+a] for a in 'xyz'])
                for s in ['left', 'right', 'ball', 'vac_pressure', 'vac_skirt'])
    force[:, 2] -= 9.81*c['mass_kg']
    moment = sum(np.column_stack([c[s+'_m'+a] for a in 'xyz'])
                 for s in ['left', 'right', 'ball', 'vac_pressure', 'vac_skirt'])
    moment += np.column_stack([c['vac_rolling_m'+a] for a in 'xyz'])
    gravity = np.zeros_like(moment)
    gravity[:, 2] = -9.81*c['mass_kg']
    moment += np.cross(np.column_stack([c['cg_'+a]-c['pose_'+a] for a in 'xyz']), gravity)
    pressure_force = np.linalg.norm(np.column_stack([
        c['vac_pressure_f'+a] for a in 'xyz']), axis=1)
    result = {
        'frames_sha256': sha(path), 'scored_samples': len(scored),
        'pressure_range_pa': [float(pressure.min()), float(pressure.max())],
        'minimum_pressure_slack_pa': float(pressure.min()-required),
        'minimum_flow_reserve_at_requirement_kg_s': float(min(reserve)),
        'minimum_shutoff_headroom_pa': float(physics['fans'][0]['shutoff_pa']-pressure.max()),
        'hold_duration_s': float(np.ptp(c['sim_time_s'][hold])),
        'hold_mean_force_residual_n': force[hold].mean(axis=0).tolist(),
        'hold_mean_moment_residual_nm': moment[hold].mean(axis=0).tolist(),
        'hold_normal_balance_relative': float(
            abs(force[hold, 0].mean())/pressure_force[hold].mean()),
        'hold_position_drift_m': float(np.linalg.norm([
            np.ptp(c['pose_'+a][hold]) for a in 'xyz'])),
        'common_hold_mean_pressure_pa': float(c['vac_delta_pa'][common].mean()),
        'common_hold_mean_pressure_force_n': float(pressure_force[common].mean()),
        'checks': {'whole_trace_pressure_meets_screen': bool(np.all(pressure >= required)),
                   'whole_trace_flow_reserve': bool(min(reserve) >= 0),
                   'whole_trace_no_stop': bool(np.all(
                        c['vac_skirt_stop_previous_normal_n'][scored] == 0))}}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ['audit', 'coupling', 'pressure', 'data-root', 'output']:
        parser.add_argument('--'+key, type=Path, required=True)
    args = parser.parse_args()
    audit = json.loads(args.audit.read_text())
    coupling = json.loads(args.coupling.read_text())
    pressure = json.loads(args.pressure.read_text())
    physics = read_physics()
    required = audit['capability_screen']['required_pressure_pa']
    native = {name: metrics(args.data_root/name, required, physics)
              for name in coupling['cases']}
    normal, half = [native[n] for n in ['rough_soft_c', 'rough_soft_c_half_step']]
    comparison = {key: abs(half[key]-normal[key])/normal[key] for key in [
        'common_hold_mean_pressure_pa', 'common_hold_mean_pressure_force_n']}
    checks = {
        'saved_pose_quadrature': all(
            all(c['checks'].values()) for c in audit['saved_poses'].values()),
        'seal_thresholds': all(audit['threshold']['checks'].values()),
        'native_coupling_and_loads': all(
            all(c['checks'].values()) for c in coupling['cases'].values()),
        'updated_pressure_and_flow': all(all(c['checks'].values()) for c in native.values()),
        'nominal_hold_at_least_60s': normal['hold_duration_s'] >= 60,
        'nominal_normal_balance': normal['hold_normal_balance_relative'] <= .02,
        'physical_step_pressure_and_force': all(v <= .02 for v in comparison.values()),
        'fan_response_steps': all(pressure['checks'].values())}
    # Do not turn an observed fault boundary into a fault-tolerance requirement.
    result = {
        'scope': 'Representative-patch T2 closure only; M2 defaults and T3 remain pending',
        't2_representative_accepted': all(checks.values()), 'm2_all_accepted': False,
        'checks': checks,
        'protocol': audit['protocol'], 'source': audit['source'],
        'finalizer_sha256': sha(Path(__file__)),
        'input_sha256': {name: sha(path) for name, path in [
            ('audit', args.audit), ('coupling', args.coupling), ('pressure', args.pressure)]},
        'saved_pose_audit': audit['saved_poses'], 'threshold': audit['threshold'],
        'capability_screen': audit['capability_screen'], 'fault_budget': audit['fault_budget'],
        'native_coupling': coupling['cases'], 'complete_trace_checks': native,
        'physical_step_relative_difference': comparison, 'pressure_time_integration': pressure,
        'pending': ['T3 large complete capture with Gazebo GUI and RViz, mean RTF >=0.95',
                    'T3 control/slip/TF checks and independent load criterion'
                    ' verification',
                    'Full-wall geometry, startup/performance and default adoption'
                    ' after acceptance']}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'t2_representative_accepted': result['t2_representative_accepted'],
                      'checks': checks, 'required_pressure_pa': required}))
    return 0 if result['t2_representative_accepted'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
