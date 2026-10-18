#!/usr/bin/env python3
"""Run paired legacy/candidate GUI control cases serially with immutable evidence."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
CASES = ['horizontal', 'vertical', 'trapezoid_horizontal', 'trapezoid_vertical',
         'bigH', 'bigTH', 'bigV', 'bigTV']


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--case', action='append', choices=CASES)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip()
    if subprocess.check_output(['git', 'status', '--porcelain', '--', 'src', 'tools']):
        raise RuntimeError('commit runtime sources and tools before formal GUI acceptance')
    write(args.output / 'source_manifest.json', {
        'revision': revision,
        'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'source_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in map(Path, subprocess.check_output(
                              ['git', 'ls-files', '-z', 'src', 'tools']).decode().split('\0'))
                          if p.is_file()}})
    results = {}
    token = str(time.time_ns())
    for case in args.case or CASES:
        for geometry in ['legacy', 'vacuum_chassis']:
            name = case + '_' + geometry
            directory = args.output / name
            directory.mkdir()
            tag = f's6_gui_{geometry}_{token}'
            log = Path('/tmp') / f'climbot_{name}_{token}.log'
            env = dict(os.environ, ROBOT_GEOMETRY=geometry, ADHESION_MODE='auto',
                       COVERAGE_HEADLESS='false', COVERAGE_RVIZ='true',
                       COVERAGE_GUI_GPU_BACKEND='shared')
            with log.open('w') as stream:
                result = subprocess.run(['tools/run_coverage_regression.sh', '-j', '1', '-t', tag,
                                         case], cwd=ROOT, env=env, stdout=stream,
                                        stderr=subprocess.STDOUT, timeout=1800)
            text = log.read_text()
            match = re.search(r'^logs\s*:\s*(.+)$', text, re.M)
            if match:
                lane = Path(match.group(1))
                observer = lane / case / 'observation'
                if observer.exists():
                    shutil.copytree(observer, directory / 'observation')
                native = lane / case / 'native_observation'
                if native.exists():
                    shutil.copytree(native, directory / 'native_observation')
                cleanup = (lane / 'teardown').read_text() if (lane / 'teardown').exists() else ''
            else:
                cleanup = 'runner did not report a log directory'
            prefix = ROOT / 'results' / f'coverage_{case}_{tag}'
            for suffix in ['_summary.json', '_trajectory.csv.gz']:
                path = Path(str(prefix) + suffix)
                if path.exists():
                    shutil.move(path, directory / path.name)
            observation_path = directory / 'observation/summary.json'
            observation = (json.loads(observation_path.read_text())
                           if observation_path.exists() else {})
            summaries = list(directory.glob('*_summary.json'))
            summary = json.loads(summaries[0].read_text()) if summaries else {}
            native_path = directory / 'native_observation/summary.json'
            native = json.loads(native_path.read_text()) if native_path.exists() else {}
            checks = {
                'evaluator_passed': result.returncode == 0 and summary.get('passed') is True,
                'visible_gui_recorded': bool(observation.get('gui')),
                'visible_rviz_recorded': bool(observation.get('rviz')),
                'native_clock_integrity': bool(native.get('checks')) and all(
                    native.get('checks', {}).values()),
                'task_average_rtf_at_least_095': (
                    native.get('native_clock_ratio') is not None and
                    native['native_clock_ratio'] >= .95),
                'imu_localization_and_joint_streams_present': all(
                    observation.get('counts', {}).get(k, 0) > 0
                    for k in ['imu', 'imu_wall', 'filtered', 'joints', 'truth', 'clock']),
                'all_sensor_tf_connected': bool(observation.get('tf_connected')) and all(
                    observation.get('tf_connected', {}).values()),
                'private_mesa_actual_server_and_gui': (
                    any(not p['gui'] and p['private_mapping_complete'] for p in
                        observation.get('runtime_libraries', [])) and
                    any(p['gui'] and p['private_mapping_complete'] for p in
                        observation.get('runtime_libraries', [])) and
                    not any(p['system_gallium_mapped'] for p in
                            observation.get('runtime_libraries', []))),
                'candidate_pressure_released_without_failure': geometry == 'legacy' or (
                    observation.get('fixture_released_observed') is True
                    and not observation.get('observed_startup_failure', True)
                    and not observation.get('observed_runtime_failure', True)
                    and observation.get('counts', {}).get('pressure', 0) > 0
                    and observation.get('counts', {}).get('vacuum', 0) > 0),
                'no_observation_error': observation.get('error') is None,
                'no_cleanup_failure': not cleanup.strip(),
            }
            results[name] = {'exit_code': result.returncode, 'log_name': log.name,
                             'checks': checks, 'rtf': observation.get('measured_realtime_factor'),
                             'native_rtf': native.get('native_clock_ratio')}
            write(args.output / 'progress.json', results)
            print(json.dumps({name: results[name]}), flush=True)
            if not all(checks.values()):
                return 1
    comparisons = {}
    for case in args.case or CASES:
        old = results[case + '_legacy']['native_rtf']
        new = results[case + '_vacuum_chassis']['native_rtf']
        comparisons[case] = {'legacy_rtf': old, 'candidate_rtf': new,
                             'candidate_minus_legacy': new - old}
    write(args.output / 'summary.json', {
        'cases': results, 'paired_rtf': comparisons, 'all_case_checks_passed': True})
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
