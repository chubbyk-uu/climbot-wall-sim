#!/usr/bin/env python3
"""Four bounded caster-size/lip comparisons; retain failed physics results verbatim."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--ode-prefix', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output must be new')
    args.output.mkdir(parents=True)
    repo = Path(__file__).resolve().parents[1]
    protocol = {'scope': 'four isolated worst-intersection sweeps; not vehicle acceptance',
                'diameters_mm': [25, 40], 'top_45_degree_chamfer_mm': [0, .25],
                'path': 'cross', 'load_n': 80, 'normal_carriage_mass_kg': 5,
                'speed_mps': .02, 'step_s': .001, 'opening_m': .005,
                'depth_range_m': [.003, .005], 'gates_unchanged': True,
                'native_detector': 'ode', 'no_robot_default_change': True}
    (args.output/'protocol.json').write_text(json.dumps(protocol, indent=2)+'\n')
    results, identities = {}, []
    for diameter in [25, 40]:
        for chamfer in [0, .25]:
            name = f'd{diameter}_ch{chamfer}'
            out = args.output/name
            command = [sys.executable, str(repo/'tools/probe_m2_joint_sweep.py'),
                       '--caster-only', '--skip-static', '--paths', 'cross',
                       '--caster-diameter-mm', str(diameter), '--top-chamfer-mm', str(chamfer),
                       '--ode-prefix', str(args.ode_prefix), '--output', str(out)]
            with (args.output/(name+'.log')).open('w') as log:
                result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=75)
            if not (out/'summary.json').exists():
                raise RuntimeError(f'{name} did not produce a result; inspect its separate log')
            data = json.loads((out/'summary.json').read_text())
            identities.append(data['source_sha256'])
            results[name] = {'exit_code': result.returncode, 'diameter_mm': diameter,
                             'top_chamfer_mm': chamfer, **data['cases']['caster_cross'],
                             'producer_summary_sha256': hashlib.sha256(
                                 (out/'summary.json').read_bytes()).hexdigest()}
    if any(identity != identities[0] for identity in identities):
        raise RuntimeError('source changed during the comparison')
    for relative, expected in identities[0].items():
        source = repo/relative
        if hashlib.sha256(source.read_bytes()).hexdigest() != expected:
            raise RuntimeError(f'source changed: {relative}')
        target = args.output/'source_snapshot'/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    comparison = {'protocol': protocol, 'cases': results, 'source_sha256': identities[0],
                  'completed': True, 'all_physics_checks_passed': all(
                      case['passed'] for case in results.values())}
    (args.output/'comparison.json').write_text(json.dumps(comparison, indent=2,
                                                         allow_nan=False)+'\n')
    print(json.dumps({'completed': True, 'cases': results}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
