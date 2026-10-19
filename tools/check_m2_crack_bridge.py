#!/usr/bin/env python3
"""Check static circular wheel support for the explicit narrow-crack approximation."""
import argparse
import hashlib
import json
import math
from pathlib import Path

from climbot_gazebo.wall_surface import WallSurface
import numpy as np
import yaml


def sag(radius, width):
    """Stable circular sag for an isolated slot on a locally flat surface."""
    if not 0 < width < 2*radius:
        raise ValueError('slot width must be positive and smaller than the diameter')
    half = width/2
    return half*half/(radius+math.sqrt(radius*radius-half*half))


def support_difference(surface, center, radius, width, heading, spacing):
    """Compare equal-grid maxima; this is a static sampled check, not dynamics."""
    extent = .009
    along = np.linspace(-extent, extent, round(2*extent/spacing)+1)
    across = (np.linspace(-width/2, width/2, math.ceil(width/spacing)+1) if width else along)
    x, y = np.meshgrid(along, across)
    radial_squared = x*x if width else x*x+y*y
    shape = np.sqrt(np.maximum(0, radius*radius-radial_squared))
    shape = np.where(radial_squared < radius*radius, shape, -np.inf)
    angle = math.radians(heading)
    u = center[0]+x*math.cos(angle)-y*math.sin(angle)
    v = center[1]+x*math.sin(angle)+y*math.cos(angle)
    base = surface.roughness(u, v)+shape
    actual = base-surface.groove_depth(u, v)
    return float(base.max()-actual.max())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('preserve prior evidence; output must be new')
    root = Path(__file__).resolve().parents[1]
    cfg_path = root/'src/climbot_gazebo/config/m2_surface_fixture.yaml'
    robot_path = root/'src/climbot_description/config/robot.yaml'
    config = yaml.safe_load(cfg_path.read_text())
    config['grooves'] = [g for g in config['grooves'] if g.get('kind') == 'crack']
    surface = WallSurface(config)
    robot = yaml.safe_load(robot_path.read_text())['robot']
    wheels = {'drive': (robot['drive_wheel']['radius_m'], robot['drive_wheel']['width_m']),
              'caster': (robot['caster']['radius_m'], 0)}
    centers = []
    for groove in config['grooves']:
        a, b = np.array(groove['start_m']), np.array(groove['end_m'])
        centers.extend((groove['id'], fraction, a+fraction*(b-a))
                       for fraction in [0, .25, .5, .75, 1])
    rows = []
    for name, (radius, width) in wheels.items():
        for groove, fraction, center in centers:
            for heading in ([0, 45, 90, 135] if width else [0]):
                coarse = support_difference(surface, center, radius, width, heading, .0001)
                fine = support_difference(surface, center, radius, width, heading, .00005)
                rows.append({'wheel': name, 'groove': groove, 'fraction': fraction,
                             'heading_deg': heading, 'coarse_difference_m': coarse,
                             'fine_difference_m': fine, 'resolution_delta_m': abs(coarse-fine)})
    report = {'scope': 'sampled static support on isolated cracked rough fixture; no tyre deformation, '
              'impact, traction, panel-joint intersection or GUI acceptance',
              'window_half_extent_m': .009, 'sample_spacings_m': [.0001, .00005],
              'source_sha256': {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in [cfg_path, robot_path, Path(__file__).resolve(),
                                          root/'src/climbot_gazebo/climbot_gazebo/wall_surface.py']},
              'wheel_geometry': wheels,
              'flat_isolated_slot_sag_m': {name: {str(w): sag(radius, w)
                                                 for w in [.0005, .001, .002]}
                                           for name, (radius, _) in wheels.items()},
              'rows': rows,
              'max_support_difference_m': max(r['fine_difference_m'] for r in rows),
              'max_resolution_delta_m': max(r['resolution_delta_m'] for r in rows),
              'support_error_gate_m': .0001, 'resolution_delta_gate_m': .00002}
    report['passed'] = (report['max_support_difference_m'] <= .0001 and
                        report['max_resolution_delta_m'] <= .00002)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: report[k] for k in ['passed', 'max_support_difference_m',
                                           'max_resolution_delta_m']}))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
