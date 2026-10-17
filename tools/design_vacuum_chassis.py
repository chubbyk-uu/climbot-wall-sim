#!/usr/bin/env python3
"""Reproduce S1 layout, mass and instantaneous load sizing (not Gazebo evidence)."""
import argparse
import csv
from copy import deepcopy
import hashlib
import itertools
import json
import math
from pathlib import Path
import subprocess
import time
from xml.etree import ElementTree as ET

import numpy as np
import yaml

from vacuum_layout import build
from vacuum_loads import evaluate, minimum_pressure, states
from vacuum_preview import export

ROOT = Path(__file__).resolve().parents[1]
INPUTS = ['src/climbot_description/config/'+p for p in
          ['vacuum_chassis.yaml', 'robot.yaml', 'inspection_camera.yaml']]+[
    'src/climbot_control/config/control.yaml',
    'src/climbot_gazebo/config/simulation.yaml',
    'src/climbot_gazebo/worlds/climbot_wall.sdf.xacro',
    'tools/vacuum_layout.py', 'tools/vacuum_loads.py', 'tools/vacuum_preview.py',
    'tools/design_vacuum_chassis.py', 'tools/test_vacuum_design.py',
    'docs/SUCTION_CONTRACT.md']


def inputs():
    def read(path, key):
        return yaml.safe_load((ROOT/path).read_text())[key]
    cfg = read(INPUTS[0], 'vacuum_chassis')
    robot = read(INPUTS[1], 'robot')
    camera = read(INPUTS[2], 'inspection_camera')
    control = read(INPUTS[3], 'line_tracker')['ros__parameters']
    sim = read(INPUTS[4], 'simulation')
    # Gravity is a literal in the current world, not a simulation.yaml knob.
    world = ET.parse(ROOT/INPUTS[5])
    gravity = abs(float(world.find('.//gravity').text.split()[2]))
    return cfg, robot, camera, control, sim, gravity


def profiles(control, robot, track):
    result = [{'name': 'static', 'group': 'static', 'v': 0., 'a': 0., 'omega': 0., 'alpha': 0.}]
    def add(prefix, group, velocities, accelerations, omegas, alphas):
        for v, a, w, al in itertools.product(velocities, accelerations, omegas, alphas):
            result.append({'name': prefix+'_%03d'%len(result), 'group': group,
                           'v': v, 'a': a, 'omega': w, 'alpha': al})
    # All signs: these are conservative instantaneous envelope combinations,
    # not promised trajectories or a claim that the control eight cases ran.
    signs = lambda x: [-x, x]
    add('cruise', 'operation', signs(control['cruise_speed']), [0], [0], [0])
    add('catchup', 'operation', signs(control['catch_up_max_linear_speed']),
        signs(control['catch_up_max_linear_acceleration']), [0], [0])
    add('correct', 'operation', signs(control['catch_up_max_linear_speed']),
        signs(control['catch_up_max_linear_acceleration']),
        signs(control['max_angular_speed']), signs(control['max_angular_acceleration']))
    add('turn', 'operation', [0], [0], signs(control['max_turn_angular_speed']),
        signs(control['max_turn_angular_acceleration']))
    add('turn_start', 'operation', [0], [0], [0], signs(control['max_turn_angular_acceleration']))
    d = robot['drive']
    add('rated_straight', 'actuator', signs(d['max_linear_velocity_mps']),
        signs(d['max_linear_acceleration_mps2']), [0], [0])
    add('rated_turn', 'actuator', [0], [0], signs(d['max_angular_velocity_rps']),
        signs(d['max_angular_acceleration_rps2']))
    # Controller rate limiting bounds each wheel's linear speed/acceleration.
    # Mixed outer corners consume both budgets, not each full body limit twice.
    v = d['max_linear_velocity_mps']-track/2*d['max_angular_velocity_rps']
    a = d['max_linear_acceleration_mps2']-track/2*d['max_angular_acceleration_rps2']
    add('rated_mixed', 'actuator', signs(v), signs(a),
        signs(d['max_angular_velocity_rps']), signs(d['max_angular_acceleration_rps2']))
    # Include zero speed/acceleration and independent straight/turn budgets;
    # neither straight-only nor simultaneous-corner-only sampling covers these.
    seen = {tuple(p[k] for k in ['v','a','omega','alpha']) for p in result}
    for vel, acc, om, al in itertools.product([-d['max_linear_velocity_mps'], -v, 0, v, d['max_linear_velocity_mps']],
            [-d['max_linear_acceleration_mps2'], -a, 0, a, d['max_linear_acceleration_mps2']],
            [-d['max_angular_velocity_rps'], 0, d['max_angular_velocity_rps']],
            [-d['max_angular_acceleration_rps2'], 0, d['max_angular_acceleration_rps2']]):
        if (abs(vel)+track/2*abs(om) > d['max_linear_velocity_mps']+1e-12 or
                abs(acc)+track/2*abs(al) > d['max_linear_acceleration_mps2']+1e-12):
            continue
        if (vel,acc,om,al) not in seen:
            result.append(dict(name='rated_grid_%03d'%len(result),group='actuator',v=vel,a=acc,omega=om,alpha=al))
            seen.add((vel,acc,om,al))
    return result


def scan(model, cases, gravity, mu=1.1, margin=1.5, caster_floor=10., step=1., lateral='normal_proportional', detailed=False):
    if not model['geometry_valid']:
        return {'status': 'geometry_rejected', 'clearances_m': model['clearances_m']}, []
    kwargs = dict(gravity=gravity, mu=mu, margin=margin, caster_floor=caster_floor, lateral=lateral)
    records, peaks = [], []
    for case in cases:
        def state(h):
            return states(h, **{k: case[k] for k in ['v','a','omega','alpha']})
        coarse = state(np.arange(0, 360, step))
        p = minimum_pressure(model, coarse, **kwargs)
        # Refine every local coarse maximum, including symmetric equal peaks.
        indices = np.flatnonzero((p >= np.roll(p, 1)) & (p >= np.roll(p, -1)))
        fine_h = np.unique(np.round(np.concatenate([
            (np.arange(-step, step+.05, .1)+coarse[i, 0]) % 360 for i in indices]), 8))
        fine = state(fine_h)
        pfine = minimum_pressure(model, fine, **kwargs)
        s = np.vstack((coarse, fine))
        pressures = np.concatenate((p, pfine))
        i = int(np.argmax(pressures))
        peak = {'profile': case['name'], 'group': case['group'],
                'state': s[i].tolist(), 'minimum_pressure_pa': float(pressures[i]),
                'minimum_pressure_force_n': float(pressures[i]*model['effective_area_m2'])}
        peaks.append(peak)
        if detailed:
            values = evaluate(model, s, pressures, **kwargs)
            for j, row in enumerate(s):
                records.append({'profile': case['name'], 'group': case['group'],
                    'heading_deg': row[0], 'v_mps': row[1], 'a_mps2': row[2],
                    'omega_rps': row[3], 'alpha_rps2': row[4],
                    'minimum_pressure_pa': pressures[j],
                    'minimum_pressure_force_n': values['pressure_force_n'][j],
                    **{name: values['normals_n'][j, k] for k, name in enumerate(['left_n','right_n','ball_n'])},
                    **{name: values['drive_demand_n'][j, k] for k, name in enumerate(['left_t_n','right_t_n'])},
                    'max_motor_torque_nm': np.max(np.abs(values['motor_torque_nm'][j])),
                    'force_residual_n': np.max(np.abs(values['force_residual_n'][j])),
                    'moment_residual_nm': np.max(np.abs(values['moment_residual_nm'][j]))})
    worst = max(peaks, key=lambda x:x['minimum_pressure_pa'])
    rounding = model['config']['analysis']['pressure_rounding_pa']
    design = math.ceil(worst['minimum_pressure_pa']/rounding)*rounding
    # Evaluate chosen constant pressure over the complete sampled envelope,
    # including every refined local peak. No pressure ramp is simulated here.
    ns, margins, torques, speeds, fr, mr = [], [], [], [], [], []
    for case, peak in zip(cases, peaks):
        h = np.unique(np.r_[np.arange(0, 360, step), peak['state'][0]])
        e = evaluate(model, states(h, **{k: case[k] for k in ['v','a','omega','alpha']}), design, **kwargs)
        ns.append(np.min(e['normals_n'], axis=0)); margins.append(np.min(e['drive_margin']))
        torques.append(float(np.max(np.abs(e['motor_torque_nm']))))
        speeds.append(float(np.max(np.abs(e['wheel_speed_rps']))))
        fr.append(float(np.max(np.abs(e['force_residual_n']))))
        mr.append(float(np.max(np.abs(e['moment_residual_nm']))))
    at_worst = evaluate(model, [worst['state']], design, **kwargs)
    result = {'status': 'analytical_candidate', 'mu': mu, 'margin_required': margin,
        'caster_floor_n': caster_floor, 'lateral_allocation': lateral,
        'coarse_heading_step_deg': step, 'refined_heading_step_deg': .1,
        'worst': worst, 'profile_peaks': peaks, 'rounded_pressure_pa': design,
        'rounded_pressure_force_n': design*model['effective_area_m2'],
        'minimum_normals_at_design_n': np.min(ns, axis=0).tolist(),
        'minimum_drive_margin_at_design': min(margins), 'maximum_motor_torque_at_design_nm': max(torques),
        'maximum_wheel_speed_at_design_rps': max(speeds),
        'maximum_force_residual_n': max(fr), 'maximum_moment_residual_nm': max(mr),
        'worst_state_normals_at_design_n': at_worst['normals_n'][0].tolist(),
        'worst_state_drive_tangent_at_design_n': np.stack((at_worst['drive_longitudinal_n'][0], at_worst['drive_lateral_n'][0]), axis=1).tolist(),
        'worst_state_skirt_normal_at_design_n': float(at_worst['skirt_normal_n'][0]),
        'worst_state_skirt_drag_at_design_n': at_worst['skirt_drag_n'][0].tolist()}
    return result, records


def write_json(path, obj):
    path.write_text(json.dumps(obj, indent=2, allow_nan=False)+'\n')


def write_csv(path, rows):
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader(); writer.writerows(rows)


def figures(out, model, rows):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Circle
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for ax, indices, title in [(axes[0], (0,1), 'Wall projection (+x forward, +y left)'),
                                (axes[1], (0,2), 'Side view (wall z=0)')]:
        for part in model['parts']:
            for shape in part['shapes']:
                c = np.array(shape['centre'])
                color = {'drive':'tab:blue','ball':'tab:orange','skirt':'tab:green'}.get(part['role'], 'gray')
                if shape['type'] == 'box':
                    size = np.array(shape['size'])
                elif shape['type'] == 'sphere':
                    ax.add_patch(Circle(c[list(indices)], shape['radius'], color=color, alpha=.45));continue
                else:
                    # Cylinder projected bounding boxes, not perspective art.
                    r, h = shape['radius'], shape['length']
                    size = np.array([2*r,h,2*r] if shape['axis']=='y' else [2*r,2*r,h])
                p, size = c[list(indices)], size[list(indices)]
                ax.add_patch(Rectangle(p-size/2, *size, facecolor=color, edgecolor='black', lw=.35, alpha=.35))
        ax.plot(*np.array(model['centre_xyz_m'])[list(indices)], 'rx', label='CG')
        ax.plot(*np.array(model['pressure_centre_xyz_m'])[list(indices)], 'm+', label='pressure centre')
        ax.autoscale();ax.set_aspect('equal');ax.set_title(title);ax.set_xlabel('x [m]');ax.grid(alpha=.2);ax.legend()
    axes[0].set_ylabel('y [m]');axes[1].set_ylabel('z [m]')
    fig.tight_layout();fig.savefig(out/'layout.png', dpi=160);plt.close(fig)
    static = sorted([r for r in rows if r['profile']=='static'], key=lambda r:r['heading_deg'])
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    x = [r['heading_deg'] for r in static]
    axes[0].plot(x, [r['minimum_pressure_force_n'] for r in static]);axes[0].set_ylabel('Minimum pressure force [N]')
    for key in ['left_n','right_n','ball_n']:
        axes[1].plot(x, [r[key] for r in static], label=key)
    axes[1].set_ylabel('Reactions at minimum force [N]');axes[1].set_xlabel('Heading [deg]');axes[1].legend()
    for ax in axes:ax.grid(alpha=.3)
    fig.tight_layout();fig.savefig(out/'static_loads.png', dpi=160);plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output
    if out.exists():
        raise SystemExit('output already exists; preserve previous evidence')
    cfg, robot, camera, control, sim, gravity = inputs()
    source_status = subprocess.check_output(['git','-C',str(ROOT),'status','--porcelain','--untracked-files=normal'], text=True)
    mu = min(sim['wall']['mu'],sim['drive_wheel']['mu'])
    model = build(cfg, camera, robot)
    cases = profiles(control, robot, model['track_m'])
    started = time.monotonic()
    out.mkdir(parents=True)
    write_json(out/'layout.json', model)
    write_json(out/'motion_profiles.json', {'state_columns':['heading_deg','v_mps','a_mps2','omega_rps','alpha_rps2'],
        'profiles': cases, 'meaning': 'conservative instantaneous kinematic samples, not controller trajectory runs'})
    export(model, camera, out/'candidate.sdf')
    export(model, camera, out/'candidate.urdf', urdf=True)
    result, rows = scan(model, cases, gravity, mu=mu, detailed=True)
    # Also solve the friction-circle feasible-domain lower bound and S=1.
    lower, _ = scan(model, cases, gravity, mu=mu, lateral='feasible')
    unity, _ = scan(model, cases, gravity, mu=mu, margin=1.)
    write_csv(out/'per_state_loads.csv', rows)
    write_json(out/'nominal.json', {'nominal':result, 'feasible_lateral_lower_bound': lower, 'margin_one_boundary': unity})
    reference=[]
    for case in cases:
        e=evaluate(model,states(np.arange(360),**{k:case[k] for k in ['v','a','omega','alpha']}),
                   400/model['effective_area_m2'],gravity=gravity,mu=mu)
        reference.append({'profile':case['name'],'group':case['group'],
            'minimum_normals_n':e['normals_n'].min(axis=0).tolist(),
            'minimum_drive_margin':float(np.min(e['drive_margin'])),
            'passes_sampled_traction_support':bool(np.min(e['slack_n'])>=0)})
    write_json(out/'400n_diagnostic.json',{'force_n':400,'pressure_pa':400/model['effective_area_m2'],
        'meaning':'rejected historical force on new geometry; never a design target','profiles':reference})
    # Ready-pressure table for the final settled wall pose at each heading.
    # Actual launch orientation, surface gap and release transient remain S3.
    start=states(np.arange(0,360,15));start_p=minimum_pressure(model,start,gravity=gravity,mu=mu)
    write_csv(out/'startup_ready.csv',[{'heading_deg':s[0],'static_minimum_pressure_pa':p,
        'ready_pressure_pa':1.05*p,'continuous_ready_s':.5,'timeout_s':10.}
        for s,p in zip(start,start_p)])
    # A reproducible interior sample audit supplements the boundary grids. It
    # is not a mathematical proof over the continuous operating envelope.
    rng = np.random.default_rng(20261017)
    sample = rng.uniform(-1,1,(5000,5))
    sample[:,0] = (sample[:,0]+1)*180
    d = robot['drive'];b=model['track_m']/2
    sample[:,3] *= d['max_angular_velocity_rps'];sample[:,4] *= d['max_angular_acceleration_rps2']
    sample[:,1] *= d['max_linear_velocity_mps']-b*np.abs(sample[:,3])
    sample[:,2] *= d['max_linear_acceleration_mps2']-b*np.abs(sample[:,4])
    interior_p = minimum_pressure(model,sample,gravity=gravity,mu=mu)
    j=int(np.argmax(interior_p))
    write_json(out/'interior_audit.json', {'seed':20261017,'samples':len(sample),
        'maximum_required_pressure_pa':float(interior_p[j]),'worst_state':sample[j].tolist(),
        'within_design':bool(np.max(interior_p)<=result['rounded_pressure_pa'])})
    write_csv(out/'interior_states.csv', [dict(heading_deg=s[0],v_mps=s[1],a_mps2=s[2],omega_rps=s[3],alpha_rps2=s[4],minimum_pressure_pa=p)
        for s,p in zip(sample,interior_p)])
    layout_rows = []
    for extension, bx in itertools.product(cfg['analysis']['front_extensions_m'], cfg['analysis']['caster_reference_x_m']):
        m = build(cfg, camera, robot, extension=extension, caster_x=bx)
        v, _ = scan(m, cases, gravity, mu=mu, step=5.)
        layout_rows.append({'front_extension_m':extension,'caster_x_m':bx,'mass_kg':m['mass_kg'],
                           'pressure_centre_x_m': m['pressure_centre_xyz_m'][0], 'clearances_m':m['clearances_m'],
                           'geometry_valid': m['geometry_valid'], 'sizing':v})
    write_json(out/'layout_screening.json', layout_rows)
    # One-at-a-time sensitivities + crossed adverse endpoints. Geometry-rejected
    # combinations remain in the table; increasing pressure never repairs them.
    scenarios = []
    for path, values in [
        (('body','internal_mass_kg'),cfg['analysis']['internal_mass_scan_kg']),
        (('skirt','contact_band_m'),cfg['analysis']['skirt_band_scan_m']),
        (('skirt','lip_pressure_weight'),cfg['analysis']['lip_weight_scan']),
        (('skirt','sliding_mu'),cfg['analysis']['skirt_mu_scan']),
        (('wheel','radius_m'),[.045,.0475,.050]),
        (('caster','mount_offset'),[-.002,0,.002])]:
        for value in values:
            scenarios.append({'label':'.'.join(path)+'='+str(value), 'changes':{'.'.join(path):value}})
    for key, values in [('mu',cfg['analysis']['drive_mu_scan']),('caster_floor',[5.,10.,20.]),
                         ('body_offset',cfg['analysis']['body_offset_scan_m'])]:
        for value in values:
            scenarios.append({'label':key+'='+str(value), 'changes':{key:value}})
    # All corners of coupled major uncertainties, keeping CAD/body and caster
    # coordinates correlated. Skirt band/pressure share effects need not be
    # monotonic: scan both, rather than name a presumed worst endpoint.
    for mass, band, phi, skmu, offset in itertools.product([6.,10.],[.015,.025],[.25,.75],[.05,.20],[-.005,.005]):
        scenarios.append({'label':'joint_corner_%02d'%len(scenarios),'changes':{
            'body.internal_mass_kg':mass,'skirt.contact_band_m':band,
            'skirt.lip_pressure_weight':phi,'skirt.sliding_mu':skmu,'body_offset':offset,'mu':.8}})
    sensitivity = []
    for scenario in scenarios:
        c = deepcopy(cfg);offset=0.;muv=mu;floor=10.;mount=0.
        for key,value in scenario['changes'].items():
            if key=='mu':muv=value
            elif key=='caster_floor':floor=value
            elif key=='body_offset':offset=value
            elif key=='caster.mount_offset':mount=value
            else:
                section, field=key.split('.');c[section][field]=value
        m = build(c,camera,robot,body_offset=offset,
                  caster_x=cfg['body']['centre_x_m']-cfg['body']['length_m']/2+cfg['caster']['rear_offset_m']+offset+mount)
        v,_ = scan(m,cases,gravity,mu=muv,caster_floor=floor,step=5.)
        sensitivity.append({**scenario,'mass_kg':m['mass_kg'],'centre_xyz_m':m['centre_xyz_m'],
                            'effective_area_m2':m['effective_area_m2'],'sizing':v})
    write_json(out/'sensitivity.json', sensitivity)
    worst_sensitivity = max((s for s in sensitivity if s['sizing']['status']=='analytical_candidate'),
                            key=lambda s:s['sizing']['worst']['minimum_pressure_pa'])
    # Refine the complete worst endpoint on a 1-degree base grid, not only 5 deg.
    # Its coarse/refined result is retained above; independently regenerate.
    changes=worst_sensitivity['changes'];c=deepcopy(cfg)
    for key,value in changes.items():
        if '.' in key and key!='caster.mount_offset':
            section,field=key.split('.');c[section][field]=value
    m = build(c,camera,robot,body_offset=changes.get('body_offset',0),
              caster_x=cfg['body']['centre_x_m']-cfg['body']['length_m']/2+cfg['caster']['rear_offset_m']+
                       changes.get('body_offset',0)+changes.get('caster.mount_offset',0))
    refined,_ = scan(m,cases,gravity,mu=changes.get('mu',mu),caster_floor=changes.get('caster_floor',10.),step=1.)
    write_json(out/'sensitivity_worst_refined.json', {**worst_sensitivity,'refined_sizing':refined})
    figures(out,model,rows)
    hashes = {p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in INPUTS}
    summary = {'stage':'S1 analytical design only', 'source_commit':subprocess.check_output(
        ['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip(),
        'source_dirty':bool(source_status.strip()),'input_sha256':hashes,
        'gravity_mps2':gravity,'drive_mu_nominal':mu,'mass_kg':model['mass_kg'],
        'centre_xyz_m':model['centre_xyz_m'],'profile_count':len(cases), 'state_count':len(rows),
        'nominal':result,'worst_sensitivity_label':worst_sensitivity['label'],
        'worst_sensitivity_refined':refined,'elapsed_seconds':time.monotonic()-started,
        'gazebo_dynamic_validation':False,'fan_curve_validated':False}
    effort = robot['drive']['joint_effort_nm'];speed=robot['drive']['joint_velocity_rps']
    summary['nominal_actuator_limits_satisfied']=(result['maximum_motor_torque_at_design_nm']<effort and result['maximum_wheel_speed_at_design_rps']<speed)
    summary['sensitivity_actuator_limits_satisfied']=all(s['sizing']['maximum_motor_torque_at_design_nm']<effort and s['sizing']['maximum_wheel_speed_at_design_rps']<speed
        for s in sensitivity if s['sizing']['status']=='analytical_candidate')
    write_json(out/'summary.json', summary)
    write_json(out/'artifact_manifest.json', {p.name:hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted(out.iterdir()) if p.is_file()})
    print(json.dumps({k:summary[k] for k in ['source_commit','source_dirty','mass_kg','profile_count','state_count','nominal_actuator_limits_satisfied','elapsed_seconds']}))
    print('nominal pressure',result['rounded_pressure_pa'],'force',result['rounded_pressure_force_n'],
          'sensitivity pressure',refined['rounded_pressure_pa'])


if __name__ == '__main__':
    main()
