"""S1 independent mass, wrench, geometry and exported-description checks."""
from copy import deepcopy
from xml.etree import ElementTree as ET

import numpy as np
import pytest

from design_vacuum_chassis import inputs, profiles, scan
from freeze_suction_baseline import inertial_summary
from screen_suction_layouts import loads, tangential
from vacuum_layout import build, box_inertia, combine, hollow_beam, plate_cells, skirt_points
from vacuum_loads import evaluate, minimum_pressure, states
from vacuum_preview import export


@pytest.fixture
def data():
    return inputs()


@pytest.fixture
def model(data):
    c,r,cam,*_=data
    return build(c,cam,r)


@pytest.mark.parametrize('heading', [0., 41.8, 90., 180., 270.])
def test_revised_lateral_envelope_preserves_both_allocation_hypotheses(model, heading):
    state = states([heading], v=.189, a=.7, omega=-1.2)
    proportional = minimum_pressure(model, state, lateral='normal_proportional')
    equal = minimum_pressure(model, state, lateral='equal')
    envelope = minimum_pressure(model, state, lateral='conservative')
    np.testing.assert_allclose(envelope, np.maximum(equal, proportional), atol=1e-5)
    for allocation in ['normal_proportional', 'equal', 'conservative']:
        assert evaluate(model, state, envelope+1e-3, lateral=allocation)['slack_n'][0] >= 0
    assert evaluate(model, state, envelope-1e-3, lateral='conservative')['slack_n'][0] < 0


def test_hollow_beam_matches_independent_outer_minus_inner_volume():
    outer=np.array([.02,.04,.3]);inner=np.array([.016,.036,.3]);rho=2700
    p=hollow_beam('beam',outer,[.03,.01,.2],.002,2,rho)
    assert p['mass_kg']==pytest.approx(rho*(np.prod(outer)-np.prod(inner)))
    np.testing.assert_allclose(p['inertia_kg_m2'], box_inertia(rho*np.prod(outer),outer)-box_inertia(rho*np.prod(inner),inner),atol=1e-14)


def test_plate_overlapping_holes_removed_as_union():
    cells=plate_cells((0,4,0,3),0,.2,[(1,3,1,2),(2,4,1,2)])
    assert sum(np.prod(s) for s,_ in cells)==pytest.approx((12-3)*.2)
    with pytest.raises(ValueError):plate_cells((0,1,0,1),0,.1,[(-1,.2,0,.1)])


def test_cad_geometry_clearances_and_single_cavity(model):
    assert model['geometry_valid']
    assert model['caster_x_m']==pytest.approx(-.300)
    assert model['effective_area_m2']==pytest.approx(.131293)
    assert model['clearances_m']['wheel_to_body']==pytest.approx(.023)
    assert model['clearances_m']['cutout_to_seal_after_mount_tolerance']==pytest.approx(.01125)
    assert model['clearances_m']['cup_to_duct_after_mount_tolerance']==pytest.approx(.0275)
    assert model['clearances_m']['flange_overlap_after_mount_tolerance']==pytest.approx(.002)
    assert model['clearances_m']['cup_fit_after_mount_tolerance']==pytest.approx(.001)
    assert .002 < model['volume_m3'] < .0025  # not the ambient body interior
    assert len([p for p in model['parts'] if p['name'].startswith('fan_') and p['name'].endswith('_link')])==3


@pytest.mark.parametrize('x,valid',[(-.240,False),(-.280,True),(-.290,True),(-.295,True),(-.300,True),(-.310,False)])
def test_layout_rejects_seal_and_inlet_obstructions(data,x,valid):
    c,r,cam,*_=data
    assert build(c,cam,r,caster_x=x)['geometry_valid']==valid


def test_wider_band_and_mounting_tolerance_limit_rearward_ball(data):
    c,r,cam,*_=data;c['skirt']['contact_band_m']=.025
    assert build(c,cam,r,caster_x=-.290)['geometry_valid']
    assert build(c,cam,r,caster_x=-.295)['geometry_valid']
    assert not build(c,cam,r,caster_x=-.300)['geometry_valid']


def test_cup_fit_and_flange_overlap_include_mounting_tolerance(data):
    c,r,cam,*_=data;c['caster']['flange_width_m']=.045
    assert not build(c,cam,r)['geometry_valid']
    c['caster']['flange_width_m']=.050;c['caster']['floor_cutout_m']=.039
    assert not build(c,cam,r)['geometry_valid']


def test_shared_cad_offset_moves_caster_but_front_extension_changes_outline(data):
    c,r,cam,*_=data;m=build(c,cam,r);moved=build(c,cam,r,body_offset=.005)
    assert moved['caster_x_m']-m['caster_x_m']==pytest.approx(.005)
    ext=build(c,cam,r,extension=.04)
    assert ext['body_bounds_m'][0]==pytest.approx(m['body_bounds_m'][0])
    assert ext['body_bounds_m'][1]-m['body_bounds_m'][1]==pytest.approx(.04)
    assert ext['pressure_centre_xyz_m'][0]-m['pressure_centre_xyz_m'][0]==pytest.approx(.02)
    assert ext['caster_x_m']==pytest.approx(m['caster_x_m'])
    assert ext['mass_kg']>m['mass_kg']
    assert ext['effective_area_m2']>m['effective_area_m2']


def test_mass_inertia_and_midline_quadrature(model):
    assert 14 < model['mass_kg'] < 16
    inertia=np.array(model['inertia_kg_m2']);eig=np.linalg.eigvalsh(inertia)
    assert min(eig)>0 and max(eig)<sum(eig)-max(eig)
    points,weights=skirt_points(model)
    np.testing.assert_allclose(weights@points,[-.141,0,0],atol=1e-14)
    assert sum(weights)==pytest.approx(1)
    # Independent parallel-axis aggregation at a common origin, then shift back.
    originI=np.zeros((3,3));mass=0;first=np.zeros(3)
    for part in model['parts']:
        m=part['mass_kg'];p=np.array(part['centre_xyz_m'])
        mass+=m;first+=m*p
        originI+=np.array(part['inertia_kg_m2'])+m*(np.dot(p,p)*np.eye(3)-np.outer(p,p))
    cg=first/mass
    np.testing.assert_allclose(inertia,originI-mass*(np.dot(cg,cg)*np.eye(3)-np.outer(cg,cg)),atol=1e-14)


@pytest.mark.parametrize('heading',range(0,360,30))
def test_static_reactions_and_yaw_match_independent_closed_form(model,heading):
    p=6000.;m=model['mass_kg'];cg=model['centre_xyz_m']
    e=evaluate(model,states([heading]),p)
    # Lip pressure and reaction cancel in the wheel budget. Mechanical skirt
    # preload remains at the same x as pressure; no lip gets counted twice.
    net=p*model['inner_area_m2']-model['skirt_mechanical_n']
    expected=loads(net,m*9.81,cg[2],model['track_m'],model['caster_x_m'],-.141,heading)
    np.testing.assert_allclose(e['normals_n'][0],expected,atol=1e-11)
    tangent=tangential(m*9.81,cg[0],cg[1],model['track_m'],heading)
    np.testing.assert_allclose(e['drive_longitudinal_n'][0],tangent[:2],atol=1e-11)
    assert sum(e['drive_lateral_n'][0])==pytest.approx(tangent[2],abs=1e-11)
    np.testing.assert_allclose(e['skirt_drag_n'],0,atol=1e-14)


def test_lip_weight_cancels_static_wheel_load_but_changes_moving_drag(data):
    c,r,cam,*_=data;m1=build(c,cam,r);c['skirt']['lip_pressure_weight']=.75;m2=build(c,cam,r)
    e1=evaluate(m1,states([20]),6000);e2=evaluate(m2,states([20]),6000)
    np.testing.assert_allclose(e1['normals_n'],e2['normals_n'],atol=1e-10)
    e1=evaluate(m1,states([20],v=.2),6000);e2=evaluate(m2,states([20],v=.2),6000)
    assert abs(e2['skirt_drag_n'][0,0])>abs(e1['skirt_drag_n'][0,0])


def test_drags_dissipate_and_spin_inertias_enter_balance(model):
    s=states([0,40,180,270],v=.15,a=.5,omega=-.6,alpha=.8)
    e=evaluate(model,s,6500)
    assert np.all(e['skirt_drag_power_w']<0)
    np.testing.assert_allclose(e['force_residual_n'],0,atol=1e-10)
    np.testing.assert_allclose(e['moment_residual_nm'],0,atol=1e-10)
    # Independent dynamic Newton/Euler reconstruction, including wheel spin,
    # sphere spin and the equivalent rolling-resistance couple at the ball.
    c=model['config'];m=model['mass_kg'];cg=np.array(model['centre_xyz_m'])
    ball=next(p for p in model['parts'] if p['role']=='ball');Ib=ball['inertia_kg_m2'][0][0]
    I=np.array(model['inertia_kg_m2'])-np.eye(3)*Ib;r=c['wheel']['radius_m'];Iw=.5*c['wheel']['mass_each_kg']*r*r
    b=model['track_m']/2;L=-model['caster_x_m'];rb=c['caster']['radius_m']
    pts,weights=skirt_points(model)
    for j,(heading,v,a,w,al) in enumerate(s):
        om=np.array([0,0,w]);alpha=np.array([0,0,al]);a0=np.array([a,v*w,0])
        acg=a0+np.cross(alpha,cg)+np.cross(om,np.cross(om,cg))
        grav=m*9.81*np.array([-np.sin(np.deg2rad(heading)),-np.cos(np.deg2rad(heading)),0])
        pressure=np.array([0,0,-6500*model['effective_area_m2']])
        forces=[grav,pressure];moments=[np.cross(cg,grav),np.cross(model['pressure_centre_xyz_m'],pressure)]
        for k,point in enumerate([[0,b,0],[0,-b,0],[model['caster_x_m'],0,0]]):
            force=np.array([0.,0.,e['normals_n'][j,k]])
            if k<2:force[:2]=[e['drive_longitudinal_n'][j,k],e['drive_lateral_n'][j,k]]
            else:force+=e['ball_tangent_n'][j]
            forces.append(force);moments.append(np.cross(point,force))
        for pt,weight in zip(pts,weights):
            vel=np.array([v,0,0])+np.cross(om,pt)
            fn=e['skirt_normal_n'][j]*weight
            force=-c['skirt']['sliding_mu']*fn*vel/np.sqrt(vel@vel+.001**2)+np.array([0,0,fn])
            forces.append(force);moments.append(np.cross(pt,force))
        vb=np.array([v,-L*w,0]);rr=-c['analysis']['caster_rolling_coefficient']*e['normals_n'][j,2]*vb/np.sqrt(vb@vb+.001**2)
        moments.append(np.cross([0,0,rb],rr))
        ab=a0+np.cross(alpha,[-L,0,0])+np.cross(om,np.cross(om,[-L,0,0]))
        hdot=I@alpha+np.cross(om,I@om)+np.array([-w*2*Iw*v/r,2*Iw*a/r,0])+Ib/rb*np.cross([0,0,1],ab)
        np.testing.assert_allclose(sum(forces),m*acg,atol=1e-10)
        np.testing.assert_allclose(sum(moments),np.cross(cg,m*acg)+hdot,atol=1e-10)


@pytest.mark.parametrize('mode',['normal_proportional','feasible'])
def test_pressure_root_is_valid_boundary_and_affine_cache_matches_full_equations(model,mode):
    s=states(np.arange(0,360,7),v=-.189,a=.2215,omega=1.2,alpha=-2.2)
    p=minimum_pressure(model,s,lateral=mode)
    assert np.min(evaluate(model,s,p+.001,lateral=mode)['slack_n'])>0
    assert np.max(evaluate(model,s,p-.001,lateral=mode)['slack_n'])<0
    if mode=='feasible':assert np.all(p<=minimum_pressure(model,s)+1e-5)


def test_search_ceiling_rejects_without_force_fallback(model):
    model['config']['analysis']['maximum_search_pressure_pa']=100
    with pytest.raises(ValueError,match='ceiling'):minimum_pressure(model,states([90]))


@pytest.mark.parametrize('mu',[float('nan'),float('inf'),0,-1])
def test_invalid_friction_is_rejected(model,mu):
    with pytest.raises(ValueError):evaluate(model,states([0]),6000,mu=mu)


def test_invalid_mass_is_rejected_before_inertia_assembly(data):
    c,r,cam,*_=data;c['body']['internal_mass_kg']=-1
    with pytest.raises(ValueError,match='positive'):build(c,cam,r)


def test_static_and_dynamic_requirements_are_separate(model,data):
    c,r,cam,ctrl,sim,g=data;cases=profiles(ctrl,r,model['track_m'])
    static,_=scan(model,cases[:1],g)
    full,_=scan(model,cases,g)
    assert full['worst']['minimum_pressure_pa']>static['worst']['minimum_pressure_pa']
    assert full['minimum_drive_margin_at_design']>=1.5
    assert full['maximum_motor_torque_at_design_nm']<r['drive']['joint_effort_nm']
    # Independent actuator and shared-camera preservation contracts.
    for case in cases:
        assert abs(case['v'])+model['track_m']/2*abs(case['omega'])<=.45+1e-12
        assert abs(case['a'])+model['track_m']/2*abs(case['alpha'])<=.7+1e-12


def test_preview_sdf_inertia_and_urdf_camera_tree(model,data,tmp_path):
    camera=data[2];sdf=tmp_path/'model.sdf';urdf=tmp_path/'robot.urdf'
    export(model,camera,sdf);export(model,camera,urdf,urdf=True)
    xml=ET.parse(sdf).getroot();summary=inertial_summary(xml.find('model'))
    assert summary['mass_kg']==pytest.approx(model['mass_kg'])
    np.testing.assert_allclose(summary['centre_xyz_m'],model['centre_xyz_m'],atol=1e-12)
    np.testing.assert_allclose(summary['inertia_at_centre_kg_m2'],model['inertia_kg_m2'],atol=1e-12)
    assert not xml.findall('.//plugin')
    assert not xml.findall('.//link[@name="skirt_link"]/collision')
    ur=ET.parse(urdf).getroot();links={l.get('name') for l in ur.findall('link')}
    children=[j.find('child').get('link') for j in ur.findall('joint')]
    assert links-set(children)=={'base_link'} and len(children)==len(set(children))
    assert len(links)==len(children)+1
    origins={j.find('child').get('link'):np.fromstring(j.find('origin').get('xyz'),sep=' ') for j in ur.findall('joint')}
    urparts=[]
    for link in ur.findall('link'):
        inertial=link.find('inertial')
        if inertial is None:continue
        offset=np.fromstring(inertial.find('origin').get('xyz'),sep=' ')
        pos=origins.get(link.get('name'),np.zeros(3))+offset
        inertia=inertial.find('inertia').attrib
        urparts.append(dict(mass_kg=float(inertial.find('mass').get('value')),centre_xyz_m=pos,
            inertia_kg_m2=[[float(inertia['ixx']),float(inertia['ixy']),float(inertia['ixz'])],
             [float(inertia['ixy']),float(inertia['iyy']),float(inertia['iyz'])],
             [float(inertia['ixz']),float(inertia['iyz']),float(inertia['izz'])]]))
    combined=combine(urparts)
    assert combined['mass_kg']==pytest.approx(model['mass_kg'])
    np.testing.assert_allclose(combined['inertia_kg_m2'],model['inertia_kg_m2'],atol=1e-12)
    optical=ur.find('joint[@name="optical_mount"]/origin')
    np.testing.assert_allclose(np.fromstring(optical.get('xyz'),sep=' '),camera['optical_mount']['center_xyz_m'])
    np.testing.assert_allclose(np.fromstring(optical.get('rpy'),sep=' '),camera['optical_mount']['rpy_rad'])
