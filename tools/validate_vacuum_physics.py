#!/usr/bin/env python3
"""S2 offline evidence: exercise the C++ core against independent gas references.

No Gazebo robot or real fan is validated by this calculation. Equivalent curves
are frozen in vacuum_physics.yaml; holdouts never change those curves.
"""
import argparse
from copy import deepcopy
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
import subprocess
import tempfile

import numpy as np
import yaml
from scipy.integrate import solve_ivp
from scipy.optimize import brentq

ROOT=Path(__file__).resolve().parents[1]


def orifice(a,b,area,gas):
    """Independent isentropic nozzle expression, upstream-density convention."""
    if a==b or area==0:return 0.
    high,low=max(a,b),min(a,b);gamma=gas['gamma'];d=abs(a-b)
    r=max(low/high,(2/(gamma+1))**(gamma/(gamma-1)))
    q=gas['discharge_coefficient']*area*high*math.sqrt(max(0.,2*gamma/(
        gas['specific_r']*gas['temperature_k']*(gamma-1))*(r**(2/gamma)-r**((gamma+1)/gamma))))
    q*=math.sqrt(d)/(d*d+gas['regularization_pa']**2)**.25
    return math.copysign(q,a-b)


def reference_rate(dp,speeds,area,physics):
    gas=physics['gas'];pa=gas['ambient_pa'];pc=pa-dp
    value=orifice(pa,pc,area,gas)
    for n,f in zip(speeds,physics['fans']):
        head=n*n*f['shutoff_pa']
        if n>0 and dp<head:
            q=n*f['free_flow_m3_s']*(1-max(0.,dp)/head)
            value-=pc/(gas['specific_r']*gas['temperature_k'])*q
        if dp>head:
            value+=orifice(pa-head,pc,f['port_area_m2']*f['reverse_fraction'],gas)
    return value


def reference_pressure(speeds,area,physics):
    pa=physics['gas']['ambient_pa']
    return brentq(lambda dp:reference_rate(dp,speeds,area,physics),0.,pa-1e-5,xtol=1e-7)


def read_physics():
    return yaml.safe_load((ROOT/'src/climbot_gazebo/config/vacuum_physics.yaml').read_text())['vacuum_physics']


def probe(binary,physics,cases,directory,tag):
    directory.mkdir(parents=True,exist_ok=True)
    request=directory/(tag+'.yaml')
    request.write_text(yaml.safe_dump({'physics':physics,'cases':cases},sort_keys=False))
    with tempfile.TemporaryDirectory(prefix='climbot_s2_') as temporary:
        output=Path(temporary)/'response.json'
        subprocess.run([str(binary.resolve()),str(request),str(output)],check=True,
                       stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
        raw=output.read_bytes()
    values=json.loads(raw,parse_constant=lambda x:(_ for _ in ()).throw(ValueError(x)))
    (directory.parent/(tag+'.json.gz')).write_bytes(gzip.compress(raw,mtime=0))
    return values


def response_time(trace,event=0.,final_delta=None):
    t=np.array([v['time_s'] for v in trace]);p=np.array([v['delta_pa'] for v in trace])
    start=int(np.searchsorted(t,event));initial=p[start];final=p[-1] if final_delta is None else final_delta
    if abs(final-initial)<1:return None
    progress=(p-initial)/(final-initial)
    times=[]
    for fraction in [.1,.9]:
        indexes=np.flatnonzero((t>event)&(progress>=fraction))
        if not len(indexes):raise ValueError('response threshold was not reached')
        i=int(indexes[0]);times.append(float(t[i-1]+(fraction-progress[i-1])*(t[i]-t[i-1])/(progress[i]-progress[i-1])))
    return {'t10_s':times[0]-event,'t90_s':times[1]-event,'t10_90_s':times[1]-times[0]}


def write_json(path,obj):path.write_text(json.dumps(obj,indent=2,allow_nan=False)+'\n')


def validate(binary,out):
    if out.exists():raise ValueError('output exists; preserve previous evidence')
    source=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    dirty=bool(subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip())
    physics=read_physics();cal=physics['calibration'];acc=physics['acceptance'];gas=physics['gas']
    src=ROOT/cal['requirement_source'];s1=json.loads(src.read_text());layout=json.loads((src.parent/'layout.json').read_text())
    required=s1['nominal']['worst']['minimum_pressure_pa'];volume=layout['volume_m3']
    out.mkdir(parents=True);request_dir=out/'requests';request_dir.mkdir()
    (out/'frozen_physics.yaml').write_text(yaml.safe_dump(physics,sort_keys=False))
    assertions={};metrics={};base=cal['residual_area_m2'];aref=cal['reference_area_m2']
    steady_cases=[dict(id=f'{group}_{v}',kind='steady',speeds=[1.,1.,1.],area_m2=base+v*aref,volume_m3=volume)
        for group,values in [('calibration',cal['opening_multipliers']),('holdout',cal['holdout_multipliers'])] for v in values]
    # Calibration and allowed normal range were frozen in config, before holdout.
    results=probe(binary,physics,steady_cases,request_dir,'calibration_holdout')
    table=[]
    for c,r in zip(steady_cases,results):
        if r['status']!='ok':raise ValueError(r)
        v=r['value'];ref=reference_pressure(c['speeds'],c['area_m2'],physics)
        table.append({'id':c['id'],'additional_opening_m2':c['area_m2']-base,
            'pressure_pa':v['delta_pa'],'force_n':v['delta_pa']*layout['effective_area_m2'],
            'required_pressure_pa':required,'meets_nominal_requirement':v['delta_pa']>=required,
            'shutoff_headroom_pa':physics['fans'][0]['shutoff_pa']-v['delta_pa'],
            'mass_flow_residual_kg_s':v['residual_kg_s'],'independent_pressure_error_pa':v['delta_pa']-ref,
            'linear_pressure_time_constant_s':volume/(gas['specific_r']*gas['temperature_k']*v['pressure_slope_kg_s_pa'])})
    metrics['calibration_and_holdout']=table
    assertions['mass_balance']=max(abs(r['mass_flow_residual_kg_s']) for r in table)<=acc['mass_flow_residual_kg_s']
    assertions['independent_pressure']=max(abs(r['independent_pressure_error_pa']) for r in table)<=acc['independent_pressure_error_pa']
    allowed=base+cal['maximum_normal_opening_multiplier']*aref
    assertions['allowed_normal_opening']=all(row['meets_nominal_requirement'] for c,row in zip(steady_cases,table) if c['area_m2']<=allowed+1e-12)
    # Lower friction and higher skirt drag are a separate parameter version;
    # the nominal curve is not silently certified against that entire envelope.
    metrics['s1_uncertainty_pressure_pa']=s1['worst_sensitivity_refined']['worst']['minimum_pressure_pa']
    metrics['uncertainty_requirement_at_allowed_opening_passes']=next(r['pressure_pa'] for r in table if r['id']=='calibration_2.0')>=metrics['s1_uncertainty_pressure_pa']
    # Fault matrix: all one/two/three-fan combinations and each stopped channel.
    faults=[]
    for fraction in [0.,.01,.05,.2,1.]:
        for mask in range(8):
            for opening in [0.,1.,2.,3.,6.,10.]:
                speeds=[float(bool(mask&(1<<i))) for i in range(3)]
                faults.append((fraction,dict(id=f'backflow_{fraction}_mask_{mask}_opening_{opening}',kind='steady',
                    speeds=speeds,area_m2=base+opening*aref,volume_m3=volume)))
    fault_metrics=[]
    for fraction in [0.,.01,.05,.2,1.]:
        config=deepcopy(physics)
        for f in config['fans']:f['reverse_fraction']=fraction
        cases=[c for value,c in faults if value==fraction]
        output=probe(binary,config,cases,request_dir,f'faults_{fraction}')
        for c,r in zip(cases,output):
            if r['status']!='ok':raise ValueError(r)
            v=r['value'];fault_metrics.append({'id':c['id'],'running':sum(c['speeds']),
                'reverse_fraction':fraction,'total_area_m2':c['area_m2'],'pressure_pa':v['delta_pa'],
                'meets_nominal_requirement':v['delta_pa']>=required,'reverse_kg_s':v['reverse_kg_s']})
    metrics['fault_matrix']=fault_metrics
    # Zero leak: reverse paths make running shutoff unique; ideal check valve
    # all-stop retains mass, running flat branch must be explicitly rejected.
    zero=[dict(id='all_stopped_closed',kind='steady',speeds=[0.,0.,0.],area_m2=0.,previous_pa=95000.,previous_volume_m3=volume,volume_m3=volume/2),
          dict(id='running_closed_check',kind='steady',speeds=[1.,1.,1.],area_m2=0.)]
    checked=deepcopy(physics)
    for f in checked['fans']:f['reverse_fraction']=0.
    zeros=probe(binary,checked,zero,request_dir,'closed_check_valve')
    assertions['closed_mass_retained']=zeros[0]['status']=='ok' and zeros[0]['value']['sealed_retained'] and abs(zeros[0]['value']['pressure_pa']-190000)<1e-6
    assertions['nonunique_rejected']=zeros[1]['status']=='error'
    # Isolated compression/lip area and continuous opening, at three spacings.
    cfg=layout['config'];sk=cfg['skirt'];body=cfg['body']
    geom=dict(length_m=body['length_m'],width_m=body['width_m'],centre_x_m=body['centre_x_m'],band_m=sk['contact_band_m'],
        samples_per_side=sk['samples_per_side'],lip_weight=sk['lip_pressure_weight'],stiffness_n_m=sk['stiffness_n_m'],
        sliding_mu=sk['sliding_mu'],free_height_m=sk['free_height_m'],**physics['skirt'])
    isolated=[dict(id=f'compression_{gap}_{dp}',kind='skirt',geometry=geom,gap_m=gap,delta_pa=dp)
        for gap in [.014,.018,.021,.022,.023,.025] for dp in [0.,3000.,6000.]]
    isolated_results=probe(binary,physics,isolated,request_dir,'skirt_isolated')
    assert all(r['status']=='ok' for r in isolated_results)
    e=next(r for r in isolated_results if r['id']=='compression_0.018_6000.0')
    assertions['s1_area_and_preload']=abs(e['area_m2']-layout['effective_area_m2'])<1e-12 and abs(e['mechanical_n']-layout['skirt_mechanical_n'])<1e-10
    # Sweep entire lift transition, including compression and fully open lip.
    sweeps={};free=sk['free_height_m'];width=physics['skirt']['transition_m']
    baseline_resolution=int(acc['transition_samples_per_width'])
    for resolution in [baseline_resolution,2*baseline_resolution]:
        gaps=np.linspace(free-width,free+2*width,3*resolution+1)
        cases=[dict(id=f'gap_{i}',kind='skirt',geometry=geom,gap_m=float(gap),speeds=[1.,1.,1.],base_area_m2=base,volume_m3=volume) for i,gap in enumerate(gaps)]
        data=probe(binary,physics,cases,request_dir,f'transition_{resolution}')
        if not all(r['status']=='ok' for r in data):raise ValueError('invalid transition sample')
        values=np.array([[r['leak_area_m2'],r['balance']['leakage_kg_s'],r['delta_pa'],r['normal_n']] for r in data])
        limits=acc['transition_relative_increment']*np.ptp(values,axis=0)+np.array([1e-9,1e-9,.1,.01])
        assertions[f'continuous_transition_{resolution}']=bool(np.all(np.max(np.abs(np.diff(values,axis=0)),axis=0)<=limits))
        sweeps[resolution]=values
    assertions['transition_spatial_refinement']=bool(np.max(np.abs(sweeps[baseline_resolution]-sweeps[2*baseline_resolution][::2]))<1e-8)
    # Reverse sweep explicitly exercises deterministic branch selection.
    reverse_cases=list(reversed(cases));reverse=probe(binary,physics,reverse_cases,request_dir,'transition_reverse')
    rv=np.array([[r['leak_area_m2'],r['balance']['leakage_kg_s'],r['delta_pa'],r['normal_n']] for r in reverse])[::-1]
    assertions['transition_reverse_unique']=bool(np.max(np.abs(rv-sweeps[2*baseline_resolution]))<1e-8)
    metrics['transition_max_adjacent_change']=np.max(np.abs(np.diff(sweeps[2*baseline_resolution],axis=0)),axis=0).tolist()
    metrics['transition_columns']=['area_m2','flow_kg_s','pressure_pa','skirt_n']
    # Quadrature refinement under nonuniform tilt and combined translation/yaw.
    quadrature=[]
    for count in [32,64,128]:
        geo={**geom,'samples_per_side':count,'tilt_rad':.002,'v_mps':.2,'omega_rps':.6}
        case=dict(id=f'quadrature_{count}',kind='skirt',geometry=geo,gap_m=.022,delta_pa=6000.)
        quadrature.append(probe(binary,physics,[case],request_dir,f'quadrature_{count}')[0])
    qarrays=np.array([[r['normal_n'],r['leak_area_m2'],*r['contact_force_n'],*r['contact_moment_nm']] for r in quadrature])
    assertions['skirt_sampling_refinement']=bool(np.max(np.abs(qarrays[1]-qarrays[2])/np.maximum(np.abs(qarrays[2]),1e-5))<=.05)
    assertions['skirt_drag_dissipative']=all(r['drag_power_w']<=0 for r in quadrature)
    metrics['quadrature']=quadrature
    # Actual volume/pressure dynamics during a slow bidirectional opening,
    # at the two physical steps. Dynamic lag and viscous loading are reported
    # separately from the zero-rate branch-uniqueness scan above.
    ramps={}
    for mode in ['quasi_static','ode']:
        for dt in [.001,.0005]:
            case=dict(id=f'ramp_{mode}_{dt}',kind='skirt_ramp',mode=mode,dt_s=dt,travel_s=2.,
                gap_start_m=free-width,gap_end_m=free+2*width,reference_gap_m=body['bottom_z_m'],
                geometry=geom,base_area_m2=base,volume_m3=volume,speeds=[1.,1.,1.],
                previous_pa=results[0]['value']['pressure_pa'])
            data=probe(binary,physics,[case],request_dir,case['id'])[0]
            if data['status']!='ok':raise ValueError(data)
            ramps[(mode,dt)]=data['trace']
        coarse=ramps[(mode,.001)];fine=ramps[(mode,.0005)][::2]
        dp_error=max(abs(a['delta_pa']-b['delta_pa']) for a,b in zip(coarse,fine))
        force_error=max(abs(a['normal_n']-b['normal_n']) for a,b in zip(coarse,fine))
        scale=max(v['normal_n'] for v in fine)
        assertions[f'ramp_{mode}_time_refinement']=dp_error/physics['fans'][0]['shutoff_pa']<=.02 and force_error/max(scale,1e-9)<=.02
        metrics[f'ramp_{mode}_pressure_refinement_error_pa']=dp_error
        metrics[f'ramp_{mode}_force_refinement_error_n']=force_error
    metrics['ramp_physical_lag_pa']=max(abs(a['delta_pa']-b['delta_pa']) for a,b in zip(
        ramps[('quasi_static',.0005)],ramps[('ode',.0005)]))
    # Finite zero slope, operating-domain regularization and epsilon sensitivity.
    reg=[]
    for epsilon in [.01,.1,1.]:
        config=deepcopy(physics);config['gas']['regularization_pa']=epsilon
        cases=[dict(id='operating',kind='steady',speeds=[1.,1.,1.],area_m2=allowed)]
        cases += [dict(id=f'zero_{dp}',kind='orifice',a_pa=gas['ambient_pa'],b_pa=gas['ambient_pa']-dp,area_m2=aref) for dp in [-.001,-.0001,0,.0001,.001]]
        data=probe(binary,config,cases,request_dir,f'regularization_{epsilon}')
        reg.append({'epsilon_pa':epsilon,'pressure_pa':data[0]['value']['delta_pa'],
                    'zero_flows_kg_s':[r['flow_kg_s'] for r in data[1:]]})
    metrics['regularization']=reg
    assertions['regularization_operating_error']=max(r['pressure_pa'] for r in reg)-min(r['pressure_pa'] for r in reg)<.01
    # Motor and fast-opening responses. Integration step is explicitly refined
    # below the physical millisecond step to resolve the absolute time budget.
    responses=[];traces={}
    steady0=results[0]['value']['pressure_pa']
    definitions=[('fan_start','quasi_static',0.,[0.,0.,0.],base,1.5),
                 ('fan_start','ode',0.,[0.,0.,0.],base,1.5),
                 ('fan_stop','quasi_static',0.,[1.,1.,1.],base,1.5),
                 ('fan_stop','ode',0.,[1.,1.,1.],base,1.5),
                 ('fast_open','ode',.01,[1.,1.,1.],base,.08)]
    for name,mode,event,initial,area,duration in definitions:
        for dt in ([.001,.0005,.00005,.000025] if name.startswith('fan_') else [.0001,.00005,.000025]):
            case=dict(id=f'{name}_{mode}_{dt}',kind='transient',mode=mode,dt_s=dt,duration_s=duration,
                initial_speeds=initial,targets=[0.,0.,0.] if name=='fan_stop' else [1.,1.,1.],area_m2=area,volume_m3=volume,
                previous_pa=gas['ambient_pa'] if name=='fan_start' else steady0,
                area_events=[] if name!='fast_open' else [{'time_s':event,'area_m2':base+6*aref}])
            data=probe(binary,physics,[case],request_dir,case['id'])[0]
            if data['status']!='ok':raise ValueError(data)
            trace=data['trace']
            final_speeds=[0.,0.,0.] if name=='fan_stop' else [1.,1.,1.]
            final_area=base+6*aref if name=='fast_open' else base
            final_steady=reference_pressure(final_speeds,final_area,physics)
            time_metric=response_time(trace,event,final_delta=final_steady)
            responses.append({'name':name,'mode':mode,'dt_s':dt,'response':time_metric,
                'initial_delta_pa':trace[int(round(event/dt))]['delta_pa'],
                'final_steady_delta_pa':final_steady,'final_delta_pa':trace[-1]['delta_pa']})
            traces[(name,mode,dt)]=trace
    metrics['responses']=responses
    for name,mode,_,_,_,_ in definitions:
        values=[r for r in responses if r['name']==name and r['mode']==mode]
        a,b=values[-2:];err=abs(a['response']['t10_90_s']-b['response']['t10_90_s'])
        assertions[f'{name}_{mode}_time_convergence']=err<=acc['response_absolute_error_s'] and err/b['response']['t10_90_s']<=acc['response_relative_error']
        metrics[f'{name}_{mode}_time_error_s']=err
        if name.startswith('fan_'):
            assertions[f'{name}_{mode}_steady_step_convergence']=abs(values[0]['final_delta_pa']-values[1]['final_delta_pa'])/max(1.,values[1]['final_delta_pa'])<=acc['steady_relative_error']
    # Independent high-accuracy adaptive ODE for a declared fast opening. It
    # uses a different integrator and explicit mass/ideal-gas equation.
    trace=traces[('fast_open','ode',.000025)];ts=np.array([r['time_s'] for r in trace]);ps=np.array([r['pressure_pa'] for r in trace])
    event=.01;ix=np.searchsorted(ts,event);pa=gas['ambient_pa'];rt=gas['specific_r']*gas['temperature_k']
    solution=solve_ivp(lambda t,y:[rt/volume*reference_rate(pa-y[0],[1.,1.,1.],base+6*aref,physics)],
        (event,ts[-1]),[steady0],method='Radau',t_eval=ts[ix:],rtol=1e-10,atol=1e-6)
    if not solution.success:raise ValueError(solution.message)
    error=np.max(np.abs(ps[ix:]-solution.y[0]))
    metrics['fast_open_independent_ode_maximum_pressure_error_pa']=float(error)
    excursion=abs(solution.y[0,-1]-steady0)
    assertions['independent_ode_response']=bool(error/excursion<.02)
    # Also show that quasi-static predicts the slow fan response, without
    # inventing finite dynamics for its instantaneous opening jumps.
    qs=traces[('fan_start','quasi_static',.000025)];od=traces[('fan_start','ode',.000025)]
    metrics['fan_start_quasi_vs_ode_max_pressure_difference_pa']=max(abs(a['delta_pa']-b['delta_pa']) for a,b in zip(qs,od))
    # Plot standard standalone evidence, no runtime UI or synthetic artwork.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    for label,trace in [('quasi_static',qs),('ODE',od)]:axes[0].plot([r['time_s'] for r in trace],[r['delta_pa']/1000 for r in trace],label=label)
    axes[0].set_xlabel('Time [s]');axes[0].set_ylabel('Fan start pressure difference [kPa]');axes[0].legend()
    axes[1].plot(np.linspace(-1,2,len(sweeps[2*baseline_resolution])),sweeps[2*baseline_resolution][:,2]/1000)
    axes[1].set_xlabel('(Gap - free height) / transition width');axes[1].set_ylabel('Pressure difference [kPa]')
    for ax in axes:ax.grid(alpha=.3)
    fig.tight_layout();fig.savefig(out/'pressure_and_transition.png',dpi=150);plt.close(fig)
    sources=['src/climbot_gazebo/'+v for v in ['config/vacuum_physics.yaml','include/climbot_gazebo/vacuum_core.hpp','src/vacuum_core.cpp','src/vacuum_core_probe.cpp','test/test_vacuum_core.cpp']]+['tools/validate_vacuum_physics.py',str(src.relative_to(ROOT)),str((src.parent/'layout.json').relative_to(ROOT))]
    summary={'stage':'S2 isolated C++ core, not Gazebo integration or hardware validation','source_commit':source,'source_dirty':dirty,
        'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),'input_sha256':{v:hashlib.sha256((ROOT/v).read_bytes()).hexdigest() for v in sources},
        'assertions':assertions,'passed':all(assertions.values()),'metrics':metrics}
    write_json(out/'summary.json',summary)
    write_json(out/'artifact_manifest.json',{str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob('*')) if p.is_file()})
    print(json.dumps({'passed':summary['passed'],'assertions':assertions,'steady_pressure_pa':[r['pressure_pa'] for r in table],
        'ode_error_pa':float(error),'responses':responses},allow_nan=False))
    if not summary['passed']:raise SystemExit(1)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--binary',type=Path,default=ROOT/'build/climbot_gazebo/vacuum_core_probe')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();validate(a.binary,a.output)
