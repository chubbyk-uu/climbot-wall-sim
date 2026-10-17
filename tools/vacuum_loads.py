"""Instantaneous inverse dynamics for the S1 three-contact candidate.

Planar motion is prescribed, not simulated. Isotropic drive friction circles,
normal-proportional lateral allocation and point wheel contacts are hypotheses
that require independent Gazebo validation in S3. Units are SI throughout.
"""
import numpy as np

from vacuum_layout import skirt_points


def states(headings, v=0., a=0., omega=0., alpha=0.):
    h = np.asarray(headings, dtype=float).reshape(-1)
    return np.column_stack((h, np.full((len(h), 4), [v, a, omega, alpha])))


def evaluate(model, state, pressure, gravity=9.81, mu=1.1, margin=1.5,
             caster_floor=10., lateral='normal_proportional'):
    """Return reactions and drive demand for rows [heading_deg,v,a,omega,alpha]."""
    s = np.atleast_2d(np.asarray(state, dtype=float))
    if (s.shape[1] != 5 or not np.isfinite(s).all() or
            not np.isfinite([mu,margin,gravity,caster_floor]).all() or mu <= 0 or margin <= 0 or
            gravity <= 0 or caster_floor < 0):
        raise ValueError('invalid state/traction parameters')
    p = np.broadcast_to(np.asarray(pressure, dtype=float), (len(s),))
    if not np.isfinite(p).all() or (p < 0).any():
        raise ValueError('pressure must be finite and nonnegative')
    m, cg = model['mass_kg'], np.asarray(model['centre_xyz_m'])
    cfg = model['config']
    b, L = model['track_m']/2, -model['caster_x_m']
    if not (model['caster_x_m'] < model['pressure_centre_xyz_m'][0] < 0):
        raise ValueError('pressure centre outside drive/caster support interval')
    v, a, omega, alpha = s[:, 1:].T
    omega_vec = np.column_stack((v*0, v*0, omega))
    alpha_vec = np.column_stack((v*0, v*0, alpha))
    a0 = np.column_stack((a, v*omega, v*0))
    acg = a0+np.cross(alpha_vec, cg)+np.cross(omega_vec, np.cross(omega_vec, cg))
    theta = np.deg2rad(s[:, 0])
    g = np.column_stack((-gravity*np.sin(theta), -gravity*np.cos(theta), v*0))
    # The free ball's spin is not rigidly locked to chassis yaw.
    ball = next(part for part in model['parts'] if part['role'] == 'ball')
    Ib = ball['inertia_kg_m2'][0][0]
    inertia = np.asarray(model['inertia_kg_m2'])-np.eye(3)*Ib
    wheel = cfg['wheel']
    r, Iw = wheel['radius_m'], wheel['mass_each_kg']*wheel['radius_m']**2/2
    hdot_wheel = np.column_stack((-omega*2*Iw*v/r, 2*Iw*a/r, v*0))
    aball = a0+np.cross(alpha_vec, [-L, 0, 0])+np.cross(omega_vec, np.cross(omega_vec, [-L, 0, 0]))
    hdot_ball = Ib/cfg['caster']['radius_m']*np.cross([0, 0, 1], aball)
    angular = (alpha_vec@inertia.T+np.cross(omega_vec, omega_vec@inertia.T) +
               hdot_wheel+hdot_ball)
    # Effective external gravity minus translational/angular inertia.
    f = m*(g-acg)
    moment = np.cross(cg, f)-angular
    pressure_force = np.column_stack((p*0, p*0, -p*model['effective_area_m2']))
    f += pressure_force
    moment += np.cross(model['pressure_centre_xyz_m'], pressure_force)
    nskirt = model['skirt_mechanical_n']+p*model['lip_weighted_area_m2']
    points, weights = skirt_points(model)
    skirt_centre = weights@points
    fn = np.column_stack((p*0, p*0, nskirt))
    f += fn
    moment += np.cross(skirt_centre, fn)
    vball = np.column_stack((v, -L*omega, v*0))
    rr_unit = (-cfg['analysis']['caster_rolling_coefficient']*
          vball/np.sqrt(np.sum(vball**2, axis=1, keepdims=True)+
                        cfg['analysis']['caster_rolling_regularization_mps']**2))
    # Equivalent rolling resistance acts at ball centre and therefore also
    # changes overturning moments. Its normal-dependent couple belongs in
    # the normal solve, rather than being appended after the solve.
    matrix = np.tile(np.array([[1., 1., 1.], [b, -b, 0], [0, 0, L]]), (len(s),1,1))
    couple = np.cross([0, 0, cfg['caster']['radius_m']], rr_unit)
    matrix[:, 1:, 2] += couple[:, :2]
    normals = np.linalg.solve(matrix, -np.column_stack((f[:, 2], moment[:, :2]))[:, :, None])[:, :, 0]
    # Continuous, dissipative drag with exactly zero static holding credit.
    unique, index = np.unique(s[:, [1, 3]], axis=0, return_inverse=True)
    velocities = (np.cross(np.column_stack((unique[:, 0]*0, unique[:, 0]*0, unique[:, 1]))[:, None, :], points)+
                  np.column_stack((unique[:, 0], unique[:, 0]*0, unique[:, 0]*0))[:, None, :])
    eps = cfg['skirt']['velocity_regularization_mps']
    drag = (-cfg['skirt']['sliding_mu']*weights[None, :, None]*
            velocities/np.sqrt(np.sum(velocities**2, axis=2, keepdims=True)+eps**2))
    drag_force = nskirt[:, None]*drag.sum(axis=1)[index]
    f += drag_force
    moment += nskirt[:, None]*np.cross(points, drag).sum(axis=1)[index]
    # Signed continuation is used only while searching. Nonpositive contact
    # trials always fail support slack and are never accepted as physical.
    rr = normals[:, 2, None]*rr_unit
    ftball = -Ib/cfg['caster']['radius_m']**2*aball
    # Rolling resistance is an equivalent drag at the sphere centre. This
    # includes the contact couple r*n x rr needed by sphere angular balance.
    f += ftball+rr
    moment += np.cross([-L, 0, 0], ftball)+np.cross(ball['centre_xyz_m'], rr)
    tx, ty, mz = -f[:, 0], -f[:, 1], -moment[:, 2]
    longs = np.column_stack(((tx-mz/b)/2, (tx+mz/b)/2))
    dn = normals[:, :2]
    denominator = np.maximum(dn.sum(axis=1), 1e-12)
    lat = ty[:, None]*dn/denominator[:, None]
    demand = np.sqrt(longs**2+lat**2)
    capacity = mu*dn/margin
    if lateral == 'normal_proportional':
        traction = np.min(capacity-demand, axis=1)
    elif lateral == 'feasible':
        room = np.sqrt(np.maximum(capacity**2-longs**2, 0)).sum(axis=1)
        traction = np.minimum(np.min(capacity-np.abs(longs), axis=1), room-np.abs(ty))
    else:
        raise ValueError('unknown lateral allocation')
    slack = np.minimum(traction, np.min(dn, axis=1)-cfg['analysis']['drive_floor_n'])
    slack = np.minimum(slack, normals[:, 2]-caster_floor)
    speeds = np.column_stack((v-b*omega, v+b*omega))/r
    accelerations = np.column_stack((a-b*alpha, a+b*alpha))/r
    torques = longs*r+Iw*accelerations+wheel['joint_damping_nm_s']*speeds
    # Independent assembled wrench residual for the chosen proportional split.
    contact = np.stack((longs, lat, dn), axis=2)
    ballnormal = np.column_stack((p*0, p*0, normals[:, 2]))
    residual_f = f+contact.sum(axis=1)+ballnormal
    residual_m = (moment+np.cross([0, b, 0], contact[:, 0])+np.cross([0, -b, 0], contact[:, 1])+
                  np.cross([-L, 0, 0], ballnormal))
    actual_margin = mu*dn/np.maximum(demand, 1e-30)
    return {'normals_n': normals, 'drive_longitudinal_n': longs, 'drive_lateral_n': lat,
            'drive_demand_n': demand, 'drive_margin': actual_margin, 'slack_n': slack,
            'pressure_force_n': p*model['effective_area_m2'], 'skirt_normal_n': nskirt,
            'skirt_drag_n': drag_force,
            'skirt_drag_power_w': nskirt*np.sum(drag*velocities, axis=(1, 2))[index],
            'ball_tangent_n': ftball+rr, 'motor_torque_nm': torques, 'wheel_speed_rps': speeds,
            'force_residual_n': residual_f, 'moment_residual_nm': residual_m}


def minimum_pressure(model, state, **kwargs):
    """Traction/support boundary by bisection; motor feasibility checked separately.

No negative contact is accepted. Search is bounded; no force fallback. Motor
limits are not folded into a monotonic pressure search (drag can raise effort).
"""
    s = np.atleast_2d(state)
    hi = np.full(len(s), model['config']['analysis']['maximum_search_pressure_pa'])
    high = evaluate(model, s, hi, **kwargs)
    if (high['slack_n'] < 0).any():
        raise ValueError('no traction/support solution below search ceiling')
    anchor = evaluate(model, s, hi*.75, **kwargs)
    # In the positive-contact domain N, longitudinal force and total lateral
    # force are affine in pressure. Cache these, retaining the nonlinear
    # normal-proportional split and friction circle in every root iteration.
    # Below that domain support slack rejects the extrapolation. If the
    # anchor has lost contact use the full piecewise evaluation instead.
    affine = np.all(anchor['normals_n'] > 0)
    coefficients = {}
    for key in ['normals_n', 'drive_longitudinal_n', 'drive_lateral_n']:
        upper, lower = high[key], anchor[key]
        if key == 'drive_lateral_n':
            upper, lower = upper.sum(axis=1)[:, None], lower.sum(axis=1)[:, None]
        slope = (upper-lower)/(hi[:, None]*.25)
        coefficients[key] = (slope, upper-slope*hi[:, None])
    mu, margin = kwargs.get('mu', 1.1), kwargs.get('margin', 1.5)
    floor = kwargs.get('caster_floor', 10.)
    lo = np.zeros(len(s))
    for _ in range(34):
        mid = (lo+hi)/2
        if affine:
            normal, longitudinal, total_lat = [slope*mid[:, None]+intercept
                for slope, intercept in coefficients.values()]
            dn = normal[:, :2]
            capacity = mu*dn/margin
            if kwargs.get('lateral', 'normal_proportional') == 'feasible':
                traction = ((capacity >= np.abs(longitudinal)).all(axis=1) &
                    (np.sqrt(np.maximum(capacity**2-longitudinal**2, 0)).sum(axis=1)
                     >= np.abs(total_lat[:, 0])))
            else:
                lat = total_lat*dn/np.maximum(dn.sum(axis=1)[:, None], 1e-12)
                traction = (capacity >= np.hypot(longitudinal, lat)).all(axis=1)
            ok = (traction & (dn >= model['config']['analysis']['drive_floor_n']).all(axis=1)
                  & (normal[:, 2] >= floor))
        else:
            ok = evaluate(model, s, mid, **kwargs)['slack_n'] >= 0
        hi = np.where(ok, mid, hi)
        lo = np.where(ok, lo, mid)
    return hi
