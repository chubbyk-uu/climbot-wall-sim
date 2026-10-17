#!/usr/bin/env python3
"""Screen three-contact static suction layouts; this is not a Gazebo result.

The mass/CG inputs come from a frozen legacy description. Changing the two
support coordinates isolates their effect; it does not predict a new chassis.
The passive ball and skirt provide no static tangential holding force.
"""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


def loads(force, weight, height, track, caster_x, pressure_x, heading_deg):
    """Wall reactions (left, right, ball), +Z outward; +heading is CCW."""
    if not (caster_x < pressure_x < 0 and track > 0 and height >= 0):
        raise ValueError('need caster_x < pressure_x < axle_x=0, track > 0, h >= 0')
    if not all(math.isfinite(v) for v in (
            force, weight, height, track, caster_x, pressure_x, heading_deg)):
        raise ValueError('inputs must be finite')
    if force < 0 or weight <= 0:
        raise ValueError('force must be nonnegative and weight positive')
    theta = math.radians(heading_deg)
    fx, fy = -weight * math.sin(theta), -weight * math.cos(theta)
    ball = (-pressure_x * force - height * fx) / -caster_x
    drive_sum = force - ball
    difference = 2 * height * fy / track
    return ((drive_sum + difference) / 2, (drive_sum - difference) / 2, ball)


def tangential(weight, cg_x, cg_y, track, heading_deg):
    """Required drive longitudinal forces and their combined lateral force."""
    theta = math.radians(heading_deg)
    fx, fy = -weight * math.sin(theta), -weight * math.cos(theta)
    tx, ty = -fx, -fy
    yaw = -cg_x * fy + cg_y * fx
    # -b*Tx_left + b*Tx_right balances the static CG yaw moment.
    return ((tx - 2 * yaw / track) / 2,
            (tx + 2 * yaw / track) / 2, ty)


def feasible(normal, tangent, mu, margin, lateral_mode, ball_floor_n):
    left, right, ball = normal
    lx, rx, ty = tangent
    if min(left, right) <= 0 or ball < ball_floor_n:
        return False
    caps = [mu * left / margin, mu * right / margin]
    if abs(lx) > caps[0] or abs(rx) > caps[1]:
        return False
    if lateral_mode == 'feasible':
        # Necessary/sufficient friction-circle feasibility; says nothing about
        # which lateral split the actual rigid contact solver will select.
        return abs(ty) <= (math.sqrt(max(0, caps[0] ** 2 - lx ** 2)) +
                           math.sqrt(max(0, caps[1] ** 2 - rx ** 2)))
    if lateral_mode != 'normal_proportional':
        raise ValueError('unknown lateral allocation')
    ly = ty * left / (left + right)
    ry = ty - ly
    return math.hypot(lx, ly) <= caps[0] and math.hypot(rx, ry) <= caps[1]


def minimum_force(weight, cg, track, caster_x, pressure_x, heading_deg,
                  mu=1.1, margin=1.5, lateral_mode='normal_proportional',
                  ball_floor_n=10.0):
    if (not all(math.isfinite(v) for v in (*cg, mu, margin, ball_floor_n)) or
            mu <= 0 or margin < 1 or ball_floor_n < 0):
        raise ValueError('invalid friction/margin/support criterion')
    tangent = tangential(weight, cg[0], cg[1], track, heading_deg)

    def check(force):
        return feasible(loads(force, weight, cg[2], track, caster_x,
                              pressure_x, heading_deg),
                        tangent, mu, margin, lateral_mode, ball_floor_n)

    lower, upper = 0.0, weight
    while not check(upper):
        upper *= 2
        if upper > 1e6:
            raise ValueError('no feasible static solution below 1 MN')
    for _ in range(55):
        midpoint = (lower + upper) / 2
        if check(midpoint):
            upper = midpoint
        else:
            lower = midpoint
    return upper


def compare_static(frozen, observations):
    """Compare predicted means with an independent legacy measurement archive."""
    centres = {p['link']: p['centre_xyz_m'] for p in frozen['robot']['parts']}
    track = centres['left_wheel_link'][1] - centres['right_wheel_link'][1]
    names = ['left_wheel', 'right_wheel', 'caster']
    records, seen = [], set()
    for observed in observations['records']:
        if observed['category'] != 'static':
            continue
        heading, name = observed['heading_deg'], observed['contact']
        if (heading, name) in seen:
            raise ValueError('duplicate static observation')
        seen.add((heading, name))
        if not math.isfinite(observed['mean']):
            raise ValueError('non-finite observation')
        prediction = loads(
            -frozen['world']['persistent_force_world_n'][0],
            frozen['world']['weight_n'], frozen['robot']['centre_xyz_m'][2],
            track, centres['caster_ball_link'][0],
            frozen['world']['pressure_point_in_base_m'][0], heading)
        predicted = prediction[names.index(name)]
        records.append({'heading_deg': heading, 'contact': name,
                        'predicted_n': predicted, 'measured_mean_n': observed['mean'],
                        'error_n': predicted - observed['mean']})
    if not records or any(
            {name for h, name in seen if h == heading} != set(names)
            for heading, _ in seen):
        raise ValueError('need all three contacts at each static heading')
    return {
        'record_count': len(records),
        'rmse_n': math.sqrt(sum(r['error_n'] ** 2 for r in records) / len(records)),
        'maximum_absolute_error_n': max(abs(r['error_n']) for r in records),
        'heading_basis': 'requested headings; actual settled pose is not in the archive',
        'no_parameters_fitted': True,
        'records': records,
    }


def screen(baseline, output, observations=None):
    frozen = json.loads(baseline.read_text())
    robot = frozen['robot']
    weight = frozen['world']['weight_n']
    cg = robot['centre_xyz_m']
    inputs = frozen['input_sha256']
    # Recover the actual axle track from rendered wheel centres, not a second
    # copy of robot.yaml constants.
    centres = {part['link']: part['centre_xyz_m'] for part in robot['parts']}
    track = centres['left_wheel_link'][1] - centres['right_wheel_link'][1]
    mu = frozen['contact']['drive_mu']
    cases = [
        ('repository_legacy', -0.110, -0.330),
        ('legacy_centre_inner_ball', -0.110, -0.240),
        ('cad_centre_external_ball', -0.141, -0.410),
        ('cad_centre_inner_ball', -0.141, -0.240),
        ('cad_centre_rearward_reference', -0.141, -0.310),
    ]
    output.mkdir(parents=True, exist_ok=False)
    rows, summaries = [], []
    for name, pressure_x, caster_x in cases:
        records = []

        def record_at(heading):
            proportional = minimum_force(weight, cg, track, caster_x, pressure_x,
                                         heading, mu=mu)
            optimal = minimum_force(weight, cg, track, caster_x, pressure_x,
                                    heading, mu=mu, lateral_mode='feasible')
            return {
                'layout': name, 'heading_deg': heading,
                'minimum_force_normal_proportional_n': proportional,
                'minimum_force_feasible_split_n': optimal,
                **dict(zip(('left_at_400_n', 'right_at_400_n', 'ball_at_400_n'),
                           loads(400, weight, cg[2], track, caster_x,
                                 pressure_x, heading))),
            }

        records.extend(record_at(heading) for heading in range(360))
        refinement = set()
        for key in ['minimum_force_normal_proportional_n',
                    'minimum_force_feasible_split_n']:
            peak = max(records, key=lambda r: r[key])['heading_deg']
            refinement.update(round((peak + delta / 10) % 360, 6)
                              for delta in range(-10, 11))
        records.extend(record_at(heading) for heading in sorted(refinement)
                       if heading % 1 != 0)
        rows.extend(records)
        worst = max(records, key=lambda r: r['minimum_force_normal_proportional_n'])
        summaries.append({
            'layout': name, 'pressure_x_m': pressure_x, 'caster_x_m': caster_x,
            'drive_share': 1 - pressure_x / caster_x,
            'axial_s1_n': minimum_force(weight, cg, track, caster_x, pressure_x,
                                       90, mu=mu, margin=1),
            'axial_s1_5_n': minimum_force(weight, cg, track, caster_x, pressure_x,
                                         90, mu=mu),
            'all_heading_s1_5_proportional_n': worst[
                'minimum_force_normal_proportional_n'],
            'limiting_heading_deg': worst['heading_deg'],
            'all_heading_s1_5_feasible_split_lower_bound_n': max(
                r['minimum_force_feasible_split_n'] for r in records),
            'axial_at_400_n': loads(400, weight, cg[2], track, caster_x,
                                   pressure_x, 90),
        })
    with (output / 'headings.csv').open('w') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    summary = {
        'schema_version': 1,
        'kind': 'analytical_static_geometry_diagnostic_not_new_chassis_design',
        'baseline_sha256': hashlib.sha256(baseline.read_bytes()).hexdigest(),
        'generator_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'legacy_robot_config_sha256': inputs['src/climbot_description/config/robot.yaml'],
        'weight_n': weight, 'mass_kg': robot['mass_kg'], 'cg_xyz_m': cg,
        'track_m': track, 'mu': mu, 'required_margin': 1.5, 'ball_floor_n': 10.0,
        'heading_step_deg': 1, 'extremum_refinement_step_deg': 0.1,
        'lateral_allocation': 'normal_proportional; feasible split is a lower bound',
        'exclusions': ['skirt', 'new_chassis_mass_or_geometry', 'dynamics',
                       'cable_weight_and_tension', 'real_fan_performance'],
        'cases': summaries,
    }
    if observations:
        summary['legacy_static_comparison'] = {
            'observations_sha256': hashlib.sha256(observations.read_bytes()).hexdigest(),
            **compare_static(frozen, json.loads(observations.read_text())),
        }
    (output / 'screening_summary.json').write_text(
        json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + '\n')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--observations', type=Path,
                        help='independent legacy normal-load summary JSON')
    args = parser.parse_args()
    result = screen(args.baseline, args.output_dir, args.observations)
    for case in result['cases']:
        print('%s: axial=%.1f N all_heading=%.1f N heading=%g deg' % (
            case['layout'], case['axial_s1_5_n'],
            case['all_heading_s1_5_proportional_n'], case['limiting_heading_deg']))


if __name__ == '__main__':
    main()
