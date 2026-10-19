#!/usr/bin/env python3
"""Small imposed-tangent wheel-geometry sweeps; normal displacement remains physical."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import uuid
from xml.etree import ElementTree as ET

from ament_index_python.packages import get_package_prefix
from climbot_gazebo.ode_runtime import environment
from climbot_gazebo.surface_heightmap import add_panel_heightmaps
from climbot_gazebo.wall_surface import WallSurface
import numpy as np
import yaml

from probe_m2_wall_surface import element, evaluate, make_fixture, rows


def support_height(surface, u, v, radius, width, heading):
    """Independent sampled static support, using the source rather than the OBJ."""
    along = np.linspace(-.009, .009, 91)
    across = np.linspace(-width/2, width/2, 121) if width else along
    x, y = np.meshgrid(along, across)
    radial = x*x if width else x*x+y*y
    profile = np.where(radial < radius*radius,
                       np.sqrt(np.maximum(0, radius*radius-radial)), -np.inf)
    return float(np.max(surface.height(u+x*math.cos(heading)-y*math.sin(heading),
                                      v+x*math.sin(heading)+y*math.cos(heading))+profile))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--ode-prefix', type=Path, required=True)
    parser.add_argument('--rounded-rims', action='store_true',
                        help='0.1 mm source fillets, same opening, depth and isolated section area')
    parser.add_argument('--native-heightmaps', action='store_true',
                        help='native joint strips/intersection; no fine whole-wall heightmap')
    parser.add_argument('--caster-only', action='store_true', help='selected failure diagnosis only')
    parser.add_argument('--collision-detector', choices=['ode', 'fcl'], default='ode')
    parser.add_argument('--caster-diameter-mm', type=float,
                        help='isolated size override; does not modify the robot configuration')
    parser.add_argument('--top-chamfer-mm', type=float, default=0.,
                        help='45 degree lip: horizontal and vertical extent in mm')
    parser.add_argument('--paths', nargs='+', choices=['single', 'cross'],
                        default=['single', 'cross'])
    parser.add_argument('--skip-static', action='store_true',
                        help='only moving-wheel comparison; no micro-gauge acceptance claim')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output must be new')
    if args.native_heightmaps and args.collision_detector != 'ode':
        parser.error('native heightmap diagnostic requires ODE; no silent backend substitution')
    if args.caster_diameter_mm is not None and not 5 <= args.caster_diameter_mm <= 100:
        parser.error('caster diameter override must be 5-100 mm')
    if not 0 <= args.top_chamfer_mm < .5 or (args.rounded_rims and args.top_chamfer_mm):
        parser.error('top chamfer must be below 0.5 mm and cannot combine with rounded rims')
    root = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((root/'src/climbot_gazebo/config/m2_surface_fixture.yaml').read_text())
    config['grooves'] = config['grooves'][:2]
    if args.rounded_rims:
        for groove in config['grooves']:
            groove['rim_radius_m'] = .0001
    if args.top_chamfer_mm:
        for groove in config['grooves']:
            groove['top_chamfer_m'] = args.top_chamfer_mm/1000
    robot = yaml.safe_load((root/'src/climbot_description/config/robot.yaml').read_text())['robot']
    surface = WallSurface(config)
    args.output.mkdir(parents=True)
    base = args.output/'base'
    fixture = make_fixture(base, config, 257, .001, 'vertical', 'cross_mesh')
    if args.collision_detector != 'ode':
        tree = ET.parse(base/'world.sdf')
        tree.find('.//collision_detector').text = args.collision_detector
        tree.write(base/'world.sdf', encoding='unicode')
    heightmaps = None
    if args.native_heightmaps:
        tree = ET.parse(base/'world.sdf')
        world = tree.find('world')
        world.remove(world.find("model[@name='surface']"))
        heightmaps = add_panel_heightmaps(world, base, surface)
        # Static controls away from joints; crossing is checked with actual wheel geometry below.
        for model in list(world.findall('model')):
            if model.get('name').startswith('probe_'):
                world.remove(model)
        tree.write(base/'world.sdf', encoding='unicode')
    binary = Path(get_package_prefix('climbot_gazebo'))/'lib/climbot_gazebo/wall_surface_probe'
    env, ode = environment(args.ode_prefix, dict(
        os.environ, GZ_PARTITION='climbot_joint_sweep_'+uuid.uuid4().hex))
    report = {'scope': 'isolated 5 kg normal carriage, imposed frictionless tangent translation; '
              'actual wheel dimensions, no traction/controller/vehicle acceptance',
              'normal_axis_is_free': True, 'tangent_speed_mps': .02, 'step_s': .001,
              'ode_runtime': ode, 'collision_detector': args.collision_detector,
              'caster_diameter_override_mm': args.caster_diameter_mm,
              'top_chamfer_mm': args.top_chamfer_mm,
              'fixture': fixture, 'heightmaps': heightmaps, 'cases': {},
              'source_sha256': {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in [Path(__file__).resolve(),
                                          root/'tools/probe_m2_wall_surface.py',
                                          root/'src/climbot_gazebo/src/wall_surface_probe.cpp',
                                          root/'src/climbot_gazebo/climbot_gazebo/surface_mesh.py',
                                          root/'src/climbot_gazebo/climbot_gazebo/surface_heightmap.py',
                                          root/'src/climbot_gazebo/climbot_gazebo/wall_surface.py']},
              'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest()}
    contract = {'normal_carriage_mass_kg': 5., 'speed_mps': .02, 'settle_s': .4,
                'duration_s': 2., 'loads_n': {'drive': 150., 'caster': 80.},
                'support_error_gate_m': .0002, 'penetration_gate_m': .0002,
                'max_contact_gap_steps': 2, 'sphere_radial_normal_gate_deg': 5.,
                'normal_peak_force_multiple': 10.}
    (args.output/'contract.json').write_text(json.dumps(contract, indent=2)+'\n')

    def native(out):
        with (out/'native.log').open('w') as log:
            subprocess.run([str(binary), str(out/'request.yaml'), str(out/'native')],
                           env=env, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=60)
    if not args.native_heightmaps and not args.skip_static:
        native(base)
        report['static_cross'] = evaluate(base, config, fixture)
    else:
        report['static_cross'] = {'passed': None, 'scope': 'not run; selected moving-wheel diagnosis only'}
    for kind in (['caster'] if args.caster_only else ['drive', 'caster']):
        radius = robot['drive_wheel' if kind == 'drive' else 'caster']['radius_m']
        if kind == 'caster' and args.caster_diameter_mm is not None:
            radius = args.caster_diameter_mm/2000
        width = robot['drive_wheel']['width_m'] if kind == 'drive' else 0.
        load = contract['loads_n'][kind]
        for path in args.paths:
            name = kind+'_'+path
            out = args.output/name
            out.mkdir()
            tree = ET.parse(base/'world.sdf')
            world = tree.find('world')
            for model in list(world.findall('model')):
                if model.get('name').startswith('probe_'):
                    world.remove(model)
            heading = 0. if path == 'single' else math.pi/4
            target = np.array([.45, .65 if path == 'single' else .4])
            start = target-.016*np.array([math.cos(heading), math.sin(heading)])
            initial = support_height(surface, *start, radius, width, heading)+.00005
            model = element(world, 'model', name='probe_00')
            element(model, 'pose', f'{initial} {start[0]-.4} {start[1]+1} 0 0 0')
            for link_name, mass in [('guide', .01), ('body', 5.)]:
                link = element(model, 'link', name=link_name)
                if link_name == 'body':
                    element(link, 'pose', f'0 0 0 {heading} 0 0')
                    geometry = element(element(link, 'collision', name='collision'), 'geometry')
                    shape = element(geometry, 'cylinder' if width else 'sphere')
                    element(shape, 'radius', radius)
                    if width:
                        element(shape, 'length', width)
                    ode_surface = element(element(element(link.find('collision'), 'surface'),
                                                  'friction'), 'ode')
                    element(ode_surface, 'mu', 0)
                    element(ode_surface, 'mu2', 0)
                inertial = element(link, 'inertial')
                element(inertial, 'mass', mass)
                tensor = element(inertial, 'inertia')
                for axis in ['ixx', 'iyy', 'izz']:
                    element(tensor, axis, max(1e-8, mass*radius*radius*.4))
            for joint_name, parent, child, axis in [
                    ('scan_slide', 'world', 'guide',
                     f'0 {math.cos(heading)} {math.sin(heading)}'),
                    ('normal_slide', 'guide', 'body', '1 0 0')]:
                joint = element(model, 'joint', name=joint_name, type='prismatic')
                element(joint, 'parent', parent)
                element(joint, 'child', child)
                node = element(joint, 'axis')
                element(node, 'xyz', axis)
                limit = element(node, 'limit')
                element(limit, 'lower', -.1)
                element(limit, 'upper', .1)
            tree.write(out/'world.sdf', encoding='unicode')
            request = {'world': 'world.sdf', 'iterations': 2000,
                       'sweep': {'speed_mps': .02, 'settle_s': .4,
                                 'extra_normal_force_n': load-5*9.81}}
            (out/'request.yaml').write_text(yaml.safe_dump(request))
            native(out)
            frames = [r for r in rows(out/'native/frames.csv') if float(r['time_s']) >= .4]
            contacts = [r for r in rows(out/'native/contacts.csv') if float(r['time_s']) >= .4]
            by_time = {float(r['time_s']): r for r in frames}
            support_errors = [abs(float(r['x'])-support_height(
                surface, float(r['y'])+.4, float(r['z'])-1, radius, width, heading))
                for r in frames[::20]]
            gap, max_gap = 0, 0
            for frame in frames:
                gap = gap+1 if int(frame['contacts']) == 0 else 0
                max_gap = max(max_gap, gap)
            angles, forces = [], {}
            for row in contacts:
                t = float(row['time_s'])
                forces[t] = forces.get(t, 0.)+float(row['force_x_n'])
                if not width and t in by_time:
                    center = np.array([float(by_time[t][k]) for k in ['x', 'y', 'z']])
                    point = np.array([float(row[k]) for k in ['x', 'y', 'z']])
                    normal = np.array([float(row[k]) for k in ['nx', 'ny', 'nz']])
                    radial = center-point
                    cosine = abs(radial@normal)/(np.linalg.norm(radial)*np.linalg.norm(normal))
                    angles.append(math.degrees(math.acos(np.clip(cosine, 0, 1))))
            penetration = max([float(r['depth_m']) for r in contacts], default=math.inf)
            displacement = math.hypot(float(frames[-1]['y'])-float(frames[0]['y']),
                                      float(frames[-1]['z'])-float(frames[0]['z']))
            checks = {'support': max(support_errors) <= .0002, 'contact_gap': max_gap <= 2,
                      'penetration': penetration <= .0002, 'tangent_motion': .031 < displacement < .033,
                      'sphere_radial_normal': not angles or max(angles) <= 5,
                      'peak_normal_force': bool(forces) and max(abs(v) for v in forces.values()) < 10*load}
            report['cases'][name] = {'passed': all(checks.values()), 'checks': checks,
                                    'support_error_max_m': max(support_errors),
                                    'contact_gap_max_steps': max_gap,
                                    'penetration_max_m': penetration,
                                    'sphere_radial_normal_max_deg': max(angles) if angles else None,
                                    'normal_force_peak_n': max(map(abs, forces.values())) if forces else None,
                                    'world_sha256': hashlib.sha256((out/'world.sdf').read_bytes()).hexdigest()}
    report['passed'] = (args.native_heightmaps or args.skip_static or
                        report['static_cross']['passed']) and all(
        c['passed'] for c in report['cases'].values())
    (args.output/'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'passed': report['passed'], 'cases': report['cases']}, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
