#!/usr/bin/env python3
"""Offline T2 closure from preserved poses; never alters fan or friction curves."""

import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import subprocess

from climbot_gazebo.wall_surface import WallSurface
from design_vacuum_chassis import inputs, profiles, scan
from m2_load_criterion import resolved_mu
import numpy as np
from scipy.spatial.transform import Rotation
from vacuum_layout import skirt_points
from validate_vacuum_motion import frame_data
from validate_vacuum_physics import read_physics, reference_pressure, reference_rate
import yaml

ROOT = Path(__file__).resolve().parents[1]
CASES = ['tyre_lateral_rough_soft_c', 'tyre_lateral_holdout_clearance']
PROTOCOL = {
    'version': 'm2_t2_close_20261019',
    'scope': 'Offline saved-pose quadrature, seal threshold and capability screening',
    'pose_stride_s': .1, 'refinements': [1, 2, 4, 8],
    'skirt_samples_per_side': [32, 64, 128, 256],
    'quadrature_relative_limit': .05,
    'area_absolute_floor_m2': 1e-6,
    'pressure_absolute_floor_pa': 1.,
    'threshold_gap_range_m': [.016, .024],
    'threshold_gap_intervals': [1000, 2000, 4000, 8000],
    'threshold_refinement_note': 'Initial 1000/2000-interval broad-gap scans '
    'under-resolved the smooth 0.1mm transition. Refine resolution; keep 1% limit.',
    'transition_maximum_relative_increment': .01,
    'roundtrip_absolute_area_m2': 1e-14,
    'reverse_fractions': [0., .01, .05, .2, 1.],
    'force_safety_factor': 1.5, 'drive_mu': 1.1,
    'geometry_note': 'Same nominal projected support geometry; CG height bounds '
    'include +/-2mm wall heights and 4mm normal joint travel. Lateral travel '
    'and noncoplanar contact effects remain native/T3 checks, not analytic guarantees.',
}


def write(path, value):
    def scalar(value):
        if isinstance(value, np.generic):
            return value.item()
        raise TypeError(f'unsupported JSON value: {type(value)}')
    path.write_text(json.dumps(value, indent=2, allow_nan=False, default=scalar)+'\n')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def geometry(model, count):
    model = deepcopy(model)
    model['config']['skirt']['samples_per_side'] = count
    points, weights = skirt_points(model)
    points[:, 2] = model['config']['body']['bottom_z_m']
    return points, weights


def native_query(surface, corners, model, channel, refinement, size=1.2):
    cfg = model['config']['skirt']
    return surface._native.seal_leak(
        corners, cfg['contact_band_m'], cfg['free_height_m'], channel['drape_m'],
        channel['transition_m'], size, size, refinement)['area_m2']


def snapshot(c, index, resolved, surface, count):
    model = resolved['model']
    points, weights = geometry(model, count)
    pos = np.array([c['pose_'+a][index] for a in 'xyz'])
    rotation = Rotation.from_quat([c['q'+a][index] for a in 'xyzw'])
    wall = Rotation.from_euler('xyz', resolved['wall_surface_pose'][3:])
    origin = np.array(resolved['wall_surface_pose'][:3])
    world_points = rotation.apply(points)+pos
    wall_points = wall.inv().apply(world_points-origin)
    gap = wall_points[:, 2]-surface.height(wall_points[:, 0], wall_points[:, 1])
    speed = np.array([c['v'+a][index] for a in 'xyz'])+np.cross(
        [c['w'+a][index] for a in 'xyz'], world_points-pos)
    speed = wall.inv().apply(speed)
    normal = surface.normal(wall_points[:, 0], wall_points[:, 1])
    rate = speed[:, 2]+(normal[:, :2]*speed[:, :2]).sum(axis=1)/normal[:, 2]
    sk = model['config']['skirt']
    physics = resolved['physics']['skirt']
    compression = sk['free_height_m']-gap
    t = np.clip(compression/physics['transition_m'], 0, 1)
    damping = t*t*(3-2*t)
    mechanical = np.maximum(0, sk['stiffness_n_m']*np.minimum(
        np.maximum(compression, 0), physics['maximum_compression_m']) -
        physics['damping_ns_m']*rate*damping)*weights
    contact = np.clip(-compression/physics['transition_m'], 0, 1)
    contact = 1-contact*contact*(3-2*contact)
    perimeter = 2*(-2*sk['contact_band_m']+model['config']['body']['length_m'] +
                   model['config']['body']['width_m'])
    lip = sk['lip_pressure_weight']*sk['contact_band_m']*perimeter*weights*contact
    normal_force = mechanical+c['vac_delta_pa'][index]*lip
    # Force first moment, as an independent quadrature-convergence observable.
    first_moment = (normal_force[:, None]*points[:, :2]).sum(axis=0)
    return {'mechanical': mechanical, 'lip': lip, 'force': normal_force.sum(),
            'moment': first_moment, 'minimum_gap': gap.min(), 'maximum_gap': gap.max()}


def saved_pose_audit(root, name):
    directory = root/name
    resolved = json.loads((directory/'resolved.json').read_text())
    surface = WallSurface(yaml.safe_load((directory/'surface_patch/source.yaml').read_text()))
    c = frame_data(directory/'native/frames.csv')
    release = c['vac_release_time_s'][c['vac_fixture_active'] == 0][0]
    valid = np.flatnonzero(c['sim_time_s'] >= release+1)
    step = c['sim_time_s'][1]-c['sim_time_s'][0]
    selected = set(valid[::round(PROTOCOL['pose_stride_s']/step)].tolist())
    for key in ['vac_surface_channel_area_m2', 'vac_minimum_gap_m',
                'vac_maximum_gap_m', 'vac_skirt_mechanical_n', 'vac_delta_pa']:
        selected.update([int(valid[np.argmin(c[key][valid])]),
                         int(valid[np.argmax(c[key][valid])])])
    indices = sorted(selected)
    model = resolved['model']
    x0, x1, y0, y1 = model['body_bounds_m']
    band = model['config']['skirt']['contact_band_m']
    corners = np.array([[x0+band/2, y0+band/2], [x1-band/2, y0+band/2],
                        [x1-band/2, y1-band/2], [x0+band/2, y1-band/2]])
    corners = np.column_stack((corners, np.full(4, model['config']['body']['bottom_z_m'])))
    wall = Rotation.from_euler('xyz', resolved['wall_surface_pose'][3:])
    origin = np.array(resolved['wall_surface_pose'][:3])
    channel = resolved['surface_patch']['channel_model']
    area_errors = {str(r): [] for r in [1, 2, 4]}
    force_errors = {str(n): [] for n in [32, 64, 128]}
    moment_errors = {str(n): [] for n in [32, 64, 128]}
    recompute = []
    mechanical = []
    for i in indices:
        # Telemetry contains the PreUpdate force query and PostUpdate pose.
        # The preceding saved pose is the input to this force query.
        j = i-1
        rotation = Rotation.from_quat([c['q'+a][j] for a in 'xyzw'])
        pos = np.array([c['pose_'+a][j] for a in 'xyz'])
        local = wall.inv().apply(rotation.apply(corners)+pos-origin)
        areas = [native_query(surface, local, model, channel, r,
                              resolved['surface_patch']['size_m'])
                 for r in PROTOCOL['refinements']]
        recompute.append(abs(areas[0]-c['vac_surface_leak_area_m2'][i]))
        for r, a in zip([1, 2, 4], areas):
            area_errors[str(r)].append(abs(a-areas[-1])/max(
                areas[-1], PROTOCOL['area_absolute_floor_m2']))
        snaps = [snapshot(c, j, resolved, surface, n)
                 for n in PROTOCOL['skirt_samples_per_side']]
        ref = snaps[-1]
        # Retain actual spatial mechanical preload; no uniform-force substitution.
        mechanical.append(snaps[0]['mechanical'])
        for n, s in zip([32, 64, 128], snaps):
            force_errors[str(n)].append(abs(s['force']-ref['force'])/max(ref['force'], 1))
            moment_errors[str(n)].append(
                float(np.max(abs(s['moment']-ref['moment'])))/max(ref['force']*.4265, 1))
    report = {
        'poses': len(indices), 'pose_offset_steps': -1,
        'time_range_s': [float(c['sim_time_s'][indices[0]]),
                         float(c['sim_time_s'][indices[-1]])],
        'maximum_runtime_recompute_area_error_m2': float(max(recompute)),
        'area_maximum_relative_error': {r: float(max(v)) for r, v in area_errors.items()},
        'skirt_force_maximum_relative_error': {
            n: float(max(v)) for n, v in force_errors.items()},
        'skirt_first_moment_normalized_error': {
            n: float(max(v)) for n, v in moment_errors.items()},
        'whole_scored_pressure_range_pa': [
            float(c['vac_delta_pa'][valid].min()), float(c['vac_delta_pa'][valid].max())],
        'whole_scored_leak_range_m2': [
            float(c['vac_leak_area_m2'][valid].min()), float(c['vac_leak_area_m2'][valid].max())],
        'source_sha256': {p: sha(directory/p) for p in [
            'resolved.json', 'surface_patch/source.yaml', 'native/frames.csv']}}
    report['checks'] = {
        'runtime_channel_reproduced': bool(max(recompute) < 1e-10),
        'channel_quadrature_converged': bool(max(area_errors['1']) <= .05),
        'skirt_force_converged': bool(max(force_errors['32']) <= .05),
        'skirt_first_moment_converged': bool(max(moment_errors['32']) <= .05)}
    return report, resolved, np.array(mechanical)


def threshold_audit(model, physics):
    source = yaml.safe_load((ROOT/'src/climbot_gazebo/config/m2_surface_fixture.yaml').read_text())
    source['roughness']['amplitude_m'] = 0
    source['grooves'] = source['grooves'][:1]
    source['grooves'][0]['start_m'] = [-.1, .6]
    source['grooves'][0]['end_m'] = [1.3, .6]
    source['grooves'][0]['depth_range_m'] = [.004, .004]
    surface = WallSurface(source)
    cfg = model['config']['skirt']
    results = []
    for intervals in PROTOCOL['threshold_gap_intervals']:
        gaps = np.linspace(*PROTOCOL['threshold_gap_range_m'], intervals+1)
        values = []
        # The same prescribed gap is scanned in both directions. This is an
        # algebraic unique-branch check, not a claim of dynamic fabric hysteresis.
        for gap in np.r_[gaps, gaps[::-1]]:
            half_x = (model['config']['body']['length_m']-cfg['contact_band_m'])/2
            half_y = (model['config']['body']['width_m']-cfg['contact_band_m'])/2
            corners = np.array([[.6-half_x, .6-half_y, gap], [.6+half_x, .6-half_y, gap],
                                [.6+half_x, .6+half_y, gap], [.6-half_x, .6+half_y, gap]])
            area = native_query(surface, corners, model, {'drape_m': .001,
                                'transition_m': .0001}, 1)+.00018
            dp = reference_pressure([1, 1, 1], area, physics)
            compression = max(0, cfg['free_height_m']-gap)
            t = np.clip((gap-cfg['free_height_m'])/physics['skirt']['transition_m'], 0, 1)
            lip = model['lip_weighted_area_m2']*(1-t*t*(3-2*t))
            force = cfg['stiffness_n_m']*min(compression, .01)+dp*lip
            values.append([area, dp, force])
        values = np.array(values)
        up, down = values[:len(gaps)], values[len(gaps):][::-1]
        increments = np.max(abs(np.diff(up, axis=0)), axis=0)/np.maximum(
            np.ptp(up, axis=0), [1e-6, 1, 1])
        results.append({'intervals': intervals,
                        'maximum_normalized_increment': increments.tolist(),
                        'roundtrip_maximum_error': np.max(abs(up-down), axis=0).tolist()})
    return {'scope': 'Algebraic gap sweep: residual-channel closure and gross seal lift; '
            'pressure/fan steady reference, no claim of dynamic hysteresis',
            'gap_range_m': PROTOCOL['threshold_gap_range_m'], 'scans': results,
            'checks': {
                    'roundtrip_identical': all(
                        max(v['roundtrip_maximum_error']) == 0 for v in results),
                    'no_threshold_jump': max(results[-1]['maximum_normalized_increment']) <= .01,
                    'increments_refine': all(
                        bool(np.all(np.array(b['maximum_normalized_increment']) <=
                                    np.array(a['maximum_normalized_increment'])*.55))
                        for a, b in zip(results[:-1], results[1:]))}}


def capability(model, preload, physics, *, mu=1.1):
    # Re-scan the entire old capability envelope (all headings, all 149 profiles),
    # retaining the actuator limit combinations rather than substituting .2m/s.
    _, robot, _, control, _, gravity = inputs()
    cases = profiles(control, robot, model['track_m'])
    points, _ = skirt_points(model)
    # Representative extrema chosen deterministically, not by pressure outcome.
    moments = preload@points[:, :2]
    selected = sorted({int(np.argmax(preload.sum(axis=1))),
                       *np.argmin(moments, axis=0).tolist(),
                       *np.argmax(moments, axis=0).tolist()})
    results = []
    for i in selected:
        variant = deepcopy(model)
        variant['skirt_mechanical_samples_n'] = preload[i]
        # 4mm normal travel + two endpoints of +/-2mm wall range can increase
        # support-to-CG height by at most 8mm. Use that whole bound upward;
        # do not reduce the design demand by the actual tyre compression.
        variant['centre_xyz_m'][2] += .008
        result, _ = scan(variant, cases, gravity, mu=mu)
        results.append({'preload_index': i, 'mechanical_n': float(preload[i].sum()),
                        'mechanical_first_moment_nm': moments[i].tolist(),
                        'required_pressure_pa': result['worst']['minimum_pressure_pa'],
                        'rounded_pressure_pa': result['rounded_pressure_pa'],
                        'worst': result['worst'],
                        'minimum_normals_n': result['minimum_normals_at_design_n'],
                        'maximum_motor_torque_nm': result['maximum_motor_torque_at_design_nm'],
                        'minimum_drive_margin': result['minimum_drive_margin_at_design']})
    required = max(v['rounded_pressure_pa'] for v in results)
    return {'scope': 'Capability screening at observed preload-moment extrema, '
            'CG height +8mm bound; not an exhaustive noncoplanar/lateral-travel guarantee',
            'cg_height_upward_bound_m': .008, 'profile_count': len(cases),
            'configurations': results, 'required_pressure_pa': required}


def fault_budget(required, areas, physics):
    rows = []
    for fraction in PROTOCOL['reverse_fractions']:
        p = deepcopy(physics)
        for f in p['fans']:
            f['reverse_fraction'] = fraction
        for count in [3, 2, 1, 0]:
            speeds = [1]*count+[0]*(3-count)
            for area in sorted(set(areas)):
                dp = reference_pressure(speeds, area, p)
                reserve = -reference_rate(required, speeds, area, p)
                rows.append({'running_fans': count, 'reverse_fraction': fraction,
                             'area_mm2': area*1e6, 'equilibrium_pa': dp,
                             'flow_reserve_at_requirement_kg_s': reserve,
                             'above_requirement': dp >= required})
    return {'required_pressure_pa': required, 'cases': rows,
            'interpretation': 'Fault boundaries, not a universal fault-tolerance pass. '
            'Parallel fan heads do not add; stopped ports retain specified backflow.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-root', type=Path, required=True)
    parser.add_argument('--case', action='append')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    source = {'commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
              'dirty': bool(subprocess.check_output(['git', 'status', '--porcelain'], text=True)),
              'tool_sha256': sha(Path(__file__)),
              'loads_sha256': sha(ROOT/'tools/vacuum_loads.py'),
              'input_sha256': {p: sha(ROOT/p) for p in [
                      'src/climbot_gazebo/src/seal_channels.cpp',
                      'src/climbot_gazebo/config/vacuum_physics.yaml',
                      'src/climbot_gazebo/config/m2_wheel_compliance.yaml',
                      'src/climbot_gazebo/config/m2_tyre_lateral.yaml',
                      'tools/probe_m2_robot_patch.py', 'tools/m2_load_criterion.py']}}
    saved, preloads, coefficients = {}, [], set()
    for name in args.case or CASES:
        saved[name], resolved, preload = saved_pose_audit(args.data_root, name)
        preloads.append(preload)
        coefficients.add(resolved_mu(resolved))
        write(args.output/(name+'.json'), saved[name])
    if len(coefficients) != 1:
        raise ValueError('capability inputs have mixed friction coefficients')
    write(args.output/'protocol.json', dict(PROTOCOL, drive_mu=resolved_mu(resolved)))
    physics = read_physics()
    threshold = threshold_audit(resolved['model'], physics)
    write(args.output/'quadrature.json', {'saved_poses': saved, 'threshold': threshold})
    design = capability(resolved['model'], np.vstack(preloads), physics, mu=resolved_mu(resolved))
    areas = [a for case in saved.values() for a in case['whole_scored_leak_range_m2']]
    areas += [.00018, .00027]
    faults = fault_budget(design['required_pressure_pa'], areas, physics)
    result = {'protocol': dict(PROTOCOL, drive_mu=resolved_mu(resolved)), 'source': source, 'saved_poses': saved, 'threshold': threshold,
              'capability_screen': design, 'fault_budget': faults,
              'pending': ['Independent v2 load validation in T3',
                          'Same final configuration B/C and coupled step comparison',
                          'Full-wall dual-GUI performance/capture in T3'],
              'all_t2_accepted': False}
    write(args.output/'summary.json', result)
    print(json.dumps({'saved_checks': {n: c['checks'] for n, c in saved.items()},
                      'threshold_checks': threshold['checks'],
                      'required_pressure_pa': design['required_pressure_pa'],
                      'all_t2_accepted': False}))


if __name__ == '__main__':
    main()
