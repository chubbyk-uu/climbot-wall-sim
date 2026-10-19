#!/usr/bin/env python3
"""Isolate pressure time integration at saved M2 leak/volume bounds."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

from ament_index_python.packages import get_package_prefix
import numpy as np
from validate_vacuum_physics import probe, read_physics, reference_pressure, response_time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--closure', type=Path, required=True)
    parser.add_argument('--data-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    data = json.loads(args.closure.read_text())
    physics = read_physics()
    areas = [a for c in data['saved_poses'].values() for a in c['whole_scored_leak_range_m2']]
    areas = [min(areas), max(areas)]
    volumes = []
    for name in data['saved_poses']:
        path = args.data_root/name/'native/frames.csv'
        with path.open() as stream:
            names = next(csv.reader(stream))
            values = np.loadtxt(stream, delimiter=',', usecols=[names.index('vac_volume_m3')])
        volumes.extend([float(values.min()), float(values.max())])
    volume = max(volumes)
    steps = [.001, .0005, .00005, .000025]
    args.output.mkdir(parents=True, exist_ok=False)
    protocol = {'areas_m2': areas, 'volume_m3': volume,
                'observed_volume_bounds_m3': [min(volumes), volume], 'step_s': steps,
                'steady_relative_limit': .02, 'response_relative_limit': .02,
                'response_absolute_limit_s': .0001,
                'scope': 'Pressure-core isolation, no change to tyre 1ms calibration '
                'and no claim of full mechanical 0.5ms convergence'}
    (args.output/'protocol.json').write_text(json.dumps(protocol, indent=2)+'\n')
    binary = Path(get_package_prefix('climbot_gazebo'))/'lib/climbot_gazebo/vacuum_core_probe'
    rows = []
    for area in areas:
        equilibrium = reference_pressure([1, 1, 1], area, physics)
        for action in ['startup', 'one_fan_off']:
            metrics = []
            for dt in steps:
                case = {'id': f'{action}_{area:.9f}_{dt:g}', 'kind': 'transient', 'mode': 'ode',
                        'volume_m3': volume, 'dt_s': dt, 'duration_s': 1., 'area_m2': area,
                        'initial_speeds': [0, 0, 0] if action == 'startup' else [1, 1, 1],
                        'targets': [1, 1, 1] if action == 'startup' else [0, 1, 1],
                        'previous_pa': physics['gas']['ambient_pa']-(
                                0 if action == 'startup' else equilibrium)}
                result = probe(binary, physics, [case], args.output/'requests', case['id'])[0]
                if result['status'] != 'ok':
                    raise ValueError(result)
                trace = result['trace']
                response = response_time(trace)
                metrics.append({'dt_s': dt, 'final_pressure_pa': trace[-1]['delta_pa'],
                                'response': response})
            ref = metrics[-1]
            for m in metrics:
                m['relative_final_pressure_error'] = abs(
                    m['final_pressure_pa']-ref['final_pressure_pa'])/ref['final_pressure_pa']
                error = abs(m['response']['t10_90_s']-ref['response']['t10_90_s'])
                m['response_absolute_error_s'] = error
                m['response_relative_error'] = error/ref['response']['t10_90_s']
            rows.append({'area_m2': area, 'action': action, 'metrics': metrics})
    checks = {
        'pressure_converged': all(m['relative_final_pressure_error'] <= .02
                                  for r in rows for m in r['metrics']),
        'response_converged': all(
            m['response_relative_error'] <= .02 or m['response_absolute_error_s'] <= .0001
            for r in rows for m in r['metrics'])}
    result = {'protocol': protocol, 'checks': checks, 'cases': rows,
              'probe_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
              'closure_sha256': hashlib.sha256(args.closure.read_bytes()).hexdigest()}
    (args.output/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(checks))


if __name__ == '__main__':
    main()
