#!/usr/bin/env python3
"""Bracket native wheel margin with B, then verify the pressure on all S5 motions."""
import argparse
import gzip
import hashlib
import itertools
import json
from pathlib import Path
import shutil
import subprocess
import time
import uuid

from ament_index_python.packages import get_package_prefix
from climbot_gazebo.mesa_runtime import rendering_environment
from climbot_gazebo.vacuum_assets import build_assets
from validate_vacuum_motion import frame_data, motion_metrics, profiles, write
import yaml

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    prefix = Path(get_package_prefix('climbot_gazebo'))
    binary = prefix / 'lib/climbot_gazebo/vacuum_sim_probe'
    env, mesa = rendering_environment()
    env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = str(prefix / 'lib')
    specs = [{'name': f'scan_B_{pressure:g}', 'pressure_pa': pressure, 'heading': 270,
              'profile': 'mixed', 'purpose': 'boundary_scan'}
             for pressure in [5800., 6500., 7400., 7450., 7475., 7500., 8000., 8404.442641162896]]
    specs += [{'name': f'verify_{profile}_{heading:03}_7500', 'pressure_pa': 7500.,
               'heading': heading, 'profile': profile, 'purpose': 'profile_verification'}
              for profile, heading in itertools.product(profiles(), range(0, 360, 45))]
    write(args.output / 'cases.json', specs)
    sources = ['tools/probe_vacuum_pressure_boundary.py', 'tools/validate_vacuum_motion.py',
               'src/climbot_gazebo/climbot_gazebo/vacuum_assets.py',
               'src/climbot_gazebo/config/vacuum_runtime.yaml',
               'src/climbot_gazebo/config/vacuum_physics.yaml', 'docs/SUCTION_S5_METHOD.md']
    write(args.output / 'source_manifest.json', {
        'revision': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        'tracked_dirty': bool(subprocess.check_output(
            ['git', 'status', '--porcelain', '--untracked-files=no'])),
        'source_sha256': {p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest() for p in sources},
        'probe_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
        'plugin_sha256': hashlib.sha256(
            (prefix / 'lib/libclimbot_vacuum_system.so').read_bytes()).hexdigest()})
    results = {}
    for spec in specs:
        directory = args.output / spec['name']
        resolved = build_assets(directory, mode='fixed_pressure', heading_deg=spec['heading'],
                                realtime_factor=0,
                                runtime_overrides={'fixed_delta_pa': spec['pressure_pa']})
        request = {'world': 'world.sdf', 'step_s': .001, 'duration_s': 16,
                   'track_m': resolved['model']['track_m'], 'radius_m': .0475,
                   'motion_profile': [dict(zip(
                       ['after_release_s', 'linear_mps', 'angular_rps'], p))
                                      for p in profiles()[spec['profile']]]}
        (directory / 'request.yaml').write_text(yaml.safe_dump(request))
        env['GZ_PARTITION'] = 'climbot_s5_pressure_' + uuid.uuid4().hex
        log = Path('/tmp') / f'climbot_s5_pressure_{spec["name"]}_{time.time_ns()}.log'
        with log.open('w') as stream:
            result = subprocess.run([str(binary), str(directory / 'request.yaml'),
                                     str(directory / 'native')], env=env, stdout=stream,
                                    stderr=subprocess.STDOUT, timeout=120)
        if result.returncode:
            raise RuntimeError(f'Pressure boundary probe failed; inspect {log}')
        c = frame_data(directory / 'native/frames.csv')
        metrics = motion_metrics(c, resolved, directory, spec['profile'])
        libraries = json.loads((directory / 'native/runtime_libraries.json').read_text())
        if mesa['mesa'] == 'private' and (
                not libraries['private_mapping_complete'] or libraries['system_gallium_mapped']):
            raise RuntimeError('pressure probe did not load selected private Mesa')
        results[spec['name']] = metrics
        write(directory / 'case_summary.json', metrics)
        write(directory / 'execution.json', {
            'exit_code': result.returncode, 'log_name': log.name, 'mesa': mesa})
        for path in (directory / 'native').glob('*.csv'):
            target = Path(str(path) + '.gz')
            with path.open('rb') as src, gzip.open(target, 'wb', compresslevel=3) as dst:
                shutil.copyfileobj(src, dst)
            path.unlink()
        # Failed low-pressure wheel criteria are boundary data, never a C retune.
        verified = [s['name'] for s in specs[:len(results)]
                    if s['purpose'] == 'profile_verification']
        write(args.output / 'summary.json', {
            'cases': results, 'completed': len(results), 'total': len(specs),
            'verification_passed': bool(len(verified) == 24 and all(
                all(results[name]['checks'].values()) for name in verified)),
            'interpretation': ('B pressure boundary; unchanged C fan curves; '
                               'includes expected failures')})
    summary = json.loads((args.output / 'summary.json').read_text())
    print(json.dumps({'completed': len(results),
                      'verification_passed': summary['verification_passed'],
                      'margins': {name: r.get('minimum_friction_margin')
                                  for name, r in results.items() if name.startswith('scan_')}}))
    return 0 if summary['verification_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
