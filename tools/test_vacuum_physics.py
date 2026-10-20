"""Independent C++ adapter/physics and shared S1 geometry contract checks."""
from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest

from design_vacuum_chassis import inputs
from vacuum_layout import build, skirt_points
from vacuum_loads import evaluate, states
from validate_vacuum_physics import probe, read_physics, reference_pressure, response_time

ROOT=Path(__file__).resolve().parents[1]
BINARY=ROOT/'build/climbot_gazebo/vacuum_core_probe'


@pytest.fixture
def run(tmp_path):
    if not BINARY.is_file():pytest.skip('build vacuum_core_probe before running its adapter checks')
    def call(cases,physics=None):
        return probe(BINARY,physics or read_physics(),cases,tmp_path/'requests','case')
    return call


@pytest.mark.parametrize('area',[2e-5,6e-5,1.4e-4,2.6e-4])
def test_cpp_pressure_matches_independent_compressible_flow_reference(run,area):
    row=run([dict(id='independent',kind='steady',area_m2=area,speeds=[1.,1.,1.])])[0]
    assert row['status']=='ok'
    value=row['value']
    assert isinstance(value['pressure_pa'],float)
    assert isinstance(value['sealed_retained'],bool)
    assert value['delta_pa']==pytest.approx(reference_pressure([1.,1.,1.],area,read_physics()),abs=.01)
    assert abs(value['residual_kg_s'])<1e-9


def test_adapter_preserves_string_ids_and_reports_errors_without_force_fallback(run):
    name='12\n"quoted"'
    rows=run([dict(id=name,kind='steady',area_m2=-1.,speeds=[1.,1.,1.]),
              dict(id='bad_mode',kind='transient',mode='typo',dt_s=.001,duration_s=.1)])
    assert rows[0]['id']==name
    assert all(row['status']=='error' and 'value' not in row for row in rows)


def test_cpp_skirt_reproduces_s1_load_path_and_dissipation(run):
    cfg,robot,camera,*_=inputs();model=build(cfg,camera,robot)
    s=cfg['skirt'];b=cfg['body']
    geometry=dict(length_m=b['length_m'],width_m=b['width_m'],centre_x_m=b['centre_x_m'],
        band_m=s['contact_band_m'],samples_per_side=s['samples_per_side'],lip_weight=s['lip_pressure_weight'],
        stiffness_n_m=s['stiffness_n_m'],free_height_m=s['free_height_m'],sliding_mu=s['sliding_mu'],v_mps=.2,omega_rps=.6)
    dp=6000.
    row=run([dict(id='shared_s1',kind='skirt',geometry=geometry,gap_m=.018,delta_pa=dp)])[0]
    assert row['status']=='ok'
    expected=evaluate(model,states([0],v=.2,omega=.6),dp)
    assert row['area_m2']==pytest.approx(model['effective_area_m2'])
    assert row['normal_n']==pytest.approx(expected['skirt_normal_n'][0])
    np.testing.assert_allclose(row['contact_force_n'][:2],expected['skirt_drag_n'][0,:2],atol=1e-10)
    assert row['drag_power_w']<=0
    pts,weights=skirt_points(model)
    velocity=np.cross([0,0,.6],pts)+[.2,0,0]
    normal=expected['skirt_normal_n'][0]*weights
    drag=-s['sliding_mu']*normal[:,None]*velocity/np.sqrt(np.sum(velocity**2,axis=1)[:,None]+.001**2)
    torque=np.cross(pts,drag).sum(axis=0)
    np.testing.assert_allclose(row['contact_moment_nm'],torque+np.cross([b['centre_x_m'],0,0],[0,0,row['normal_n']]),atol=1e-10)


def test_closed_zero_leak_explicitly_retains_mass_instead_of_restoring_ambient(run):
    physics=deepcopy(read_physics())
    for f in physics['fans']:f['reverse_fraction']=0.
    row=run([dict(id='closed',kind='steady',area_m2=0.,speeds=[0.,0.,0.],previous_pa=95000.,previous_volume_m3=.003,volume_m3=.002)],physics)[0]
    assert row['status']=='ok' and row['value']['sealed_retained']
    assert row['value']['pressure_pa']==pytest.approx(142500.)


def test_implicit_trace_matches_independent_initial_derivative(run):
    physics=read_physics();gas=physics['gas'];dt=1e-6;volume=.00235
    case=dict(id='initial_derivative',kind='transient',mode='ode',dt_s=dt,duration_s=dt,
              area_m2=2e-5,initial_speeds=[1.,1.,1.],targets=[1.,1.,1.],volume_m3=volume)
    row=run([case])[0];assert row['status']=='ok'
    final=row['trace'][-1]['pressure_pa']
    mass_out=gas['ambient_pa']/(gas['specific_r']*gas['temperature_k'])*sum(f['free_flow_m3_s'] for f in physics['fans'])
    expected=-gas['specific_r']*gas['temperature_k']/volume*mass_out
    assert (final-gas['ambient_pa'])/dt==pytest.approx(expected,rel=.005)


def test_response_thresholds_use_final_equilibrium_not_unfinished_trace_endpoint():
    trace=[{'time_s':float(t),'delta_pa':float(1000*(1-np.exp(-t)))} for t in np.arange(0,4.001,.001)]
    result=response_time(trace,final_delta=1000.)
    assert result['t10_90_s']==pytest.approx(np.log(9),abs=1e-6)
