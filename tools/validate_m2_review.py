#!/usr/bin/env python3
"""Bounded system/private ODE comparison on the accepted flat, rigid M1 chassis."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_prefix
from climbot_gazebo.mesa_runtime import rendering_environment
from climbot_gazebo.ode_runtime import environment
import numpy as np
from validate_vacuum_motion import cases, run_case


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--ode-prefix', type=Path, required=True)
    parser.add_argument('--summarize-existing', action='store_true',
                        help='recheck saved cases without rerunning simulation')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=args.summarize_existing)
    prefix = Path(get_package_prefix('climbot_gazebo'))
    binary = prefix / 'lib/climbot_gazebo/vacuum_sim_probe'
    env, identity = rendering_environment()
    env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = str(prefix / 'lib')
    private, selected = environment(args.ode_prefix, env)
    names = ['straight_270_fan_ode', 'turn_000_fan_ode', 'mixed_270_fan_ode']
    specs = [s for s in cases('motion') if s['name'] in names]
    records = {}
    for variant, child in [('system', env), ('private', private)]:
        records[variant] = {}
        for spec in specs:
            root = args.output / variant
            root.mkdir(exist_ok=True)
            metrics = (json.loads((root / spec['name'] / 'case_summary.json').read_text())
                       if args.summarize_existing else
                       run_case(root, spec, binary, child, identity))
            libraries = json.loads((root / spec['name'] / 'native/runtime_libraries.json').read_text())
            expected = selected['library'] if variant == 'private' else str(
                Path('/usr/lib/x86_64-linux-gnu/libode.so.8').resolve())
            mapped = libraries['physics_libraries']
            assert expected in mapped, (expected, mapped)
            assert any(Path(p).name.startswith('libccd.so.2') for p in mapped)
            assert all(metrics['checks'].values()), metrics
            records[variant][spec['name']] = metrics
            (args.output / 'progress.json').write_text(json.dumps(records, indent=2)+'\n')
    comparisons = {}
    for name in names:
        system, private = records['system'][name], records['private'][name]
        load_error = float(np.max(np.abs(np.array(private['minimum_normals_n']) /
                                        system['minimum_normals_n'] - 1)))
        pressure_error = abs(private['minimum_pressure_pa'] - system['minimum_pressure_pa'])
        margin_error = abs(private['minimum_friction_margin'] / system['minimum_friction_margin']-1)
        comparisons[name] = dict(load_relative_error=load_error,
                                 pressure_error_pa=pressure_error,
                                 friction_margin_relative_error=margin_error,
                                 passed=load_error <= .02 and pressure_error <= .1 and margin_error <= .05)
    summary = dict(scope='Three representative M1 motion cases, including initial static hold; not full M1/M2 acceptance',
                   passed=all(c['passed'] for c in comparisons.values()),
                   comparisons=comparisons, metrics=records, selected_ode=selected,
                   source_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
                   probe_sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps({'passed': summary['passed'], 'comparisons': comparisons}))
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
