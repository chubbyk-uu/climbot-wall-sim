#!/usr/bin/env python3
"""Fixed M2 friction screening; no retuning, sensor changes or default switch."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import uuid
from xml.etree import ElementTree as ET

from ament_index_python.packages import get_package_prefix
from climbot_gazebo.mesa_runtime import rendering_environment
from climbot_gazebo.ode_runtime import environment
from climbot_gazebo.vacuum_assets import build_assets
from m2_load_criterion import evaluate
import numpy as np
from probe_m2_robot_patch import config
from validate_vacuum_motion import frame_data
import yaml

ROOT = Path(__file__).resolve().parents[1]
# Same acceleration and cruise bounds as production. Braking, reverse travel,
# opposite turns and final hold are scored, including every low-load sample.
STRAIGHT = [(0, 0, 0), (2, 0, 0), (3.5, .3, 0), (3.75, .3, 0),
            (4.95, 0, 0), (7, 0, 0), (8.5, -.3, 0), (8.75, -.3, 0),
            (9.95, 0, 0), (18, 0, 0)]
TURN = [(0, 0, 0), (2, 0, 0), (2.6, 0, .6), (5.6, 0, .6),
        (6.2, 0, 0), (8, 0, 0), (8.6, 0, -.6), (11.6, 0, -.6),
        (12.2, 0, 0), (18, 0, 0)]
CASES = {'horizontal': (0, STRAIGHT), 'vertical': (90, STRAIGHT),
         'turn': (0, TURN), 'diagonal': (47, STRAIGHT)}


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--ode-prefix', type=Path, required=True)
    parser.add_argument('--mu', type=float, default=.7)
    parser.add_argument('--case', action='append', choices=CASES)
    args = parser.parse_args()
    if not np.isfinite(args.mu) or args.mu <= 0:
        raise ValueError('mu must be finite and positive')
    control = yaml.safe_load((ROOT/'src/climbot_control/config/control.yaml').read_text())['line_tracker']['ros__parameters']
    for name, (_, profile) in CASES.items():
        for a, b in zip(profile, profile[1:]):
            acceleration = (b[1]-a[1])/(b[0]-a[0])
            speeding_up = abs(b[1]) > abs(a[1])
            bound = control['max_linear_acceleration' if speeding_up else 'max_linear_deceleration']
            assert abs(acceleration) <= bound+1e-12, name
            assert abs((b[2]-a[2])/(b[0]-a[0])) <= control['max_turn_angular_acceleration']+1e-12, name
    args.output.mkdir(parents=True, exist_ok=False)
    paths = subprocess.check_output(['git', 'ls-files', '-z', 'src', 'tools'], cwd=ROOT).decode().split('\0')
    # Include this runner when not yet tracked.
    paths = sorted(set(paths + ['tools/validate_m2_friction.py']))
    write(args.output/'source.json', {
        'head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'dirty': bool(subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT)),
        'sha256': {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
                   for name in paths if (ROOT/name).is_file()}})
    prefix = Path(get_package_prefix('climbot_gazebo'))
    binary = prefix/'lib/climbot_gazebo/vacuum_sim_probe'
    env, mesa = rendering_environment()
    env, ode = environment(args.ode_prefix, env)
    env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = os.pathsep.join(filter(None, [
        str(prefix/'lib'), env.get('GZ_SIM_SYSTEM_PLUGIN_PATH')]))
    results = {}
    for name in args.case or CASES:
        heading, profile = CASES[name]
        out = args.output/name
        source = config()
        source['contact_representation'] = 'native_heightmap'
        source['channel_model'] = yaml.safe_load((ROOT/
            'src/climbot_gazebo/config/m2_seal_channels.yaml').read_text())['m2_seal_channels']
        resolved = build_assets(out, traction_mu=args.mu, heading_deg=heading,
                                caster_proxy=True, wheel_compliance=True,
                                tyre_lateral=True, tyre_crown=True,
                                surface_patch=source, realtime_factor=0)
        world = ET.parse(out/'world.sdf')
        # Audit actual generated inputs, including startup demand, before launch.
        mu = resolved['friction']['effective_drive_mu']
        robot = world.find('.//model[@name="climbot"]')
        assert float(robot.findtext('plugin/drive_mu')) == mu
        for model in ['m2_real_wall', 'm2_caster_wall']:
            assert all(float(world.findtext(f'.//model[@name="{model}"]/link/collision/surface/friction/ode/{tag}')) == args.mu
                       for tag in ['mu', 'mu2'])
        request = {'world': 'world.sdf', 'step_s': .001, 'duration_s': 22,
                   'track_m': resolved['model']['track_m'],
                   'radius_m': resolved['model']['config']['wheel']['radius_m'],
                   'motion_profile': [dict(zip(['after_release_s', 'linear_mps', 'angular_rps'], row))
                                      for row in profile]}
        (out/'request.yaml').write_text(yaml.safe_dump(request))
        write(out/'protocol.json', {'scope': 'Open-loop physical screening; no ROS/EKF/capture acceptance',
                                  'mu': mu, 'case': name, 'request': request,
                                  'pressure_or_slip_retuning': False,
                                  'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
                                  'mesa': mesa, 'ode': ode})
        env['GZ_PARTITION'] = 'climbot_mu_' + uuid.uuid4().hex
        log = Path('/tmp')/f'climbot_mu_{name}_{time.time_ns()}.log'
        with log.open('w') as stream:
            run = subprocess.run([str(binary), str(out/'request.yaml'), str(out/'native')],
                                 env=env, stdout=stream, stderr=subprocess.STDOUT, timeout=180)
        result = {'native_exit_code': run.returncode, 'log': str(log), 'passed': False}
        if run.returncode == 0:
            libraries = json.loads((out/'native/runtime_libraries.json').read_text())
            assert str((args.ode_prefix/'lib/libode.so.8').resolve()) in libraries['physics_libraries']
            c = frame_data(out/'native/frames.csv')
            release = float(c['vac_release_time_s'].max())
            result.update(release_s=release,
                          startup_failed=bool(c['vac_startup_failed'].max()),
                          runtime_failed=bool(c['vac_runtime_failed'].max()))
            if release >= 0:
                load = evaluate(c, release, mu=mu)
                ids = c['sim_time_s'] >= release+1
                result.update(load_acceptance=load,
                              pressure_range_pa=[float(c['vac_delta_pa'][ids].min()), float(c['vac_delta_pa'][ids].max())],
                              maximum_stop_normal_n=float(c['vac_skirt_stop_previous_normal_n'][ids].max()),
                              maximum_drive_normal_n=float(max(c[s+'_fx'][ids].max() for s in ['left', 'right'])))
                result['passed'] = (load['passed'] and not result['startup_failed'] and
                                    not result['runtime_failed'] and result['maximum_stop_normal_n'] == 0)
        write(out/'screen.json', result)
        results[name] = result
        write(args.output/'summary.json', {'scope': 'Fixed physical screen; not default acceptance',
                                          'cases': results,
                                          'passed': all(r['passed'] for r in results.values())})
        print(json.dumps({'case': name, 'exit_code': run.returncode, 'passed': result['passed'],
                          'log': str(log)}), flush=True)
    return 0 if all(r['passed'] for r in results.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
