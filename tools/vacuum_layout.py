"""Offline S1 geometry and mass budget, independent of ROS and Gazebo.

This is a candidate descriptor, not a second active robot model. All derived
positions, inertia, pressure areas and preview geometry share this descriptor.
"""

from copy import deepcopy
import math

import numpy as np


def box_inertia(mass, size):
    x, y, z = size
    return np.diag([mass * (y*y + z*z) / 12,
                    mass * (x*x + z*z) / 12,
                    mass * (x*x + y*y) / 12])


def combine(parts):
    mass = sum(p['mass_kg'] for p in parts)
    centre = sum(p['mass_kg'] * np.array(p['centre_xyz_m']) for p in parts) / mass
    inertia = np.zeros((3, 3))
    for p in parts:
        d = np.array(p['centre_xyz_m']) - centre
        inertia += np.array(p['inertia_kg_m2']) + p['mass_kg'] * (
            np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return {'mass_kg': mass, 'centre_xyz_m': centre.tolist(),
            'inertia_kg_m2': inertia.tolist()}


def box_part(name, size, centre, mass, role='fixed'):
    return {'name': name, 'role': role, 'mass_kg': float(mass),
            'centre_xyz_m': list(centre),
            'inertia_kg_m2': box_inertia(mass, size).tolist(),
            'shapes': [{'type': 'box', 'size': list(size), 'centre': list(centre)}]}


def cells_part(name, cells, density=None, mass=None):
    volumes = [math.prod(size) for size, _ in cells]
    rho = density if density is not None else mass / sum(volumes)
    pieces = [box_part(name, size, centre, rho * volume)
              for (size, centre), volume in zip(cells, volumes)]
    return {'name': name, 'role': 'fixed', **combine(pieces),
            'shapes': [p['shapes'][0] for p in pieces]}


def plate_cells(bounds, z0, height, holes=()):
    """Exact disjoint rectangular cells for a plate minus rectangular holes."""
    x0, x1, y0, y1 = bounds
    if not (x0 < x1 and y0 < y1 and height > 0):
        raise ValueError('nonpositive plate')
    for a, b, c, d in holes:
        if not (x0 <= a < b <= x1 and y0 <= c < d <= y1):
            raise ValueError('hole outside plate')
    xs = sorted({x0, x1, *(v for h in holes for v in h[:2])})
    ys = sorted({y0, y1, *(v for h in holes for v in h[2:])})
    cells = []
    for a, b in zip(xs, xs[1:]):
        for c, d in zip(ys, ys[1:]):
            x, y = (a+b)/2, (c+d)/2
            if any(h[0] < x < h[1] and h[2] < y < h[3] for h in holes):
                continue
            cells.append(([b-a, d-c, height], [x, y, z0+height/2]))
    return cells


def ring_cells(cx, length, width, thickness, z0, height):
    return plate_cells((cx-length/2, cx+length/2, -width/2, width/2), z0, height,
                       [(cx-length/2+thickness, cx+length/2-thickness,
                         -width/2+thickness, width/2-thickness)])


def hollow_beam(name, size, centre, wall, axis, density):
    outer = np.array(size)
    inner = outer - 2*wall
    inner[axis] = outer[axis]  # open ends
    if min(inner) <= 0:
        raise ValueError('beam wall too thick')
    mo, mi = density * np.prod(outer), density * np.prod(inner)
    inertia = box_inertia(mo, outer) - box_inertia(mi, inner)
    # Four wall boxes, never a solid visual volume used as a second mass.
    cross = [i for i in range(3) if i != axis]
    cells = []
    for index, dimension in enumerate(cross):
        for sign in [-1, 1]:
            dims = outer.copy()
            dims[dimension] = wall
            if index == 1:
                dims[cross[0]] = inner[cross[0]]
            pos = np.array(centre)
            pos[dimension] += sign * (outer[dimension]-wall)/2
            cells.append((dims.tolist(), pos.tolist()))
    result = cells_part(name, cells, density=density)
    if not np.allclose(result['inertia_kg_m2'], inertia, atol=1e-14, rtol=1e-12):
        raise ValueError('inconsistent hollow beam inertia')
    return result


def build(config, camera, legacy_robot, extension=0.0, body_offset=0.0,
          caster_x=None):
    c = deepcopy(config)
    b, w, ball, fans, seal = (c[k] for k in ('body', 'wheel', 'caster', 'fans', 'skirt'))
    positives = [b['length_m']+extension,b['width_m'],b['floor_thickness_m'],
                 b['floor_density_kg_m3'],b['cover_thickness_m'],b['cover_density_kg_m3'],
                 b['internal_mass_kg'],w['radius_m'],w['mass_each_kg'],w['separation_m'],
                 ball['radius_m'],ball['ball_density_kg_m3'],ball['housing_mass_kg'],
                 fans['radius_m'],fans['height_m'],fans['mass_each_kg'],
                 seal['mass_kg'],seal['stiffness_n_m'],seal['contact_band_m']]
    if not np.isfinite(positives+[body_offset]).all() or min(positives) <= 0:
        raise ValueError('geometry, mass and material inputs must be positive and finite')
    if fans['count'] != 3 or len(fans['x_offsets_m']) != 3:
        raise ValueError('this candidate requires three fans')
    length, width = b['length_m']+extension, b['width_m']
    cx = b['centre_x_m']+body_offset+extension/2
    x0, x1 = cx-length/2, cx+length/2
    z0, z1 = b['bottom_z_m'], b['top_z_m']
    t, cover_t = b['floor_thickness_m'], b['cover_thickness_m']
    bx = x0 + ball['rear_offset_m'] if caster_x is None else caster_x
    cut = ball['floor_cutout_m']/2
    fan_x = [cx+offset for offset in fans['x_offsets_m']]
    port = fans['port_width_m']/2
    holes = [(x-port, x+port, -port, port) for x in fan_x]
    floor_holes = holes + [(bx-cut, bx+cut, -cut, cut)]
    parts = [cells_part('chassis_link', plate_cells(
        (x0, x1, -width/2, width/2), z0, t, floor_holes), b['floor_density_kg_m3'])]
    cover_cells = (ring_cells(cx, length, width, cover_t, z0+t, z1-z0-t-cover_t) +
                   plate_cells((x0, x1, -width/2, width/2), z1-cover_t, cover_t, holes))
    parts.append(cells_part('cover_link', cover_cells, b['cover_density_kg_m3']))
    for sign, name in [(1, 'left'), (-1, 'right')]:
        parts.append(box_part('internal_'+name, b['internal_bay_size_xyz_m'],
                              [cx, sign*b['internal_bay_y_m'], b['internal_bay_z_m']],
                              b['internal_mass_kg']/2))
        r, track = w['radius_m'], w['separation_m']
        wheel_width = w['outer_width_m']-track
        if wheel_width <= 0:
            raise ValueError('nonpositive wheel width')
        mass = w['mass_each_kg']
        transverse = mass*(3*r*r+wheel_width*wheel_width)/12
        parts.append({'name': name+'_wheel_link', 'role': 'drive', 'mass_kg': mass,
                      'centre_xyz_m': [0, sign*track/2, r],
                      'inertia_kg_m2': np.diag([transverse, mass*r*r/2, transverse]).tolist(),
                      'shapes': [{'type': 'cylinder', 'radius': r, 'length': wheel_width,
                                  'axis': 'y', 'centre': [0, sign*track/2, r]}]})
    for index, x in enumerate(fan_x):
        r, h, mass = fans['radius_m'], fans['height_m'], fans['mass_each_kg']
        parts.append({'name': 'fan_%d_link'%index, 'role': 'fixed', 'mass_kg': mass,
                      'centre_xyz_m': [x, 0, fans['centre_z_m']],
                      'inertia_kg_m2': np.diag([mass*(3*r*r+h*h)/12]*2+[mass*r*r/2]).tolist(),
                      'shapes': [{'type': 'cylinder', 'radius': r, 'length': h,
                                  'axis': 'z', 'centre': [x, 0, fans['centre_z_m']]}]})
        duct_bottom, duct_top = z0+t, fans['centre_z_m']-h/2
        duct_cells = ring_cells(x, fans['port_width_m']+2*fans['duct_wall_m'],
                                fans['port_width_m']+2*fans['duct_wall_m'],
                                fans['duct_wall_m'], duct_bottom, duct_top-duct_bottom)
        parts.append(cells_part('fan_%d_duct'%index, duct_cells, b['cover_density_kg_m3']))
    # The cup is closed towards the ambient body interior. Its flange covers
    # the complete floor cutout; the ball opens only into the common cavity.
    hw, bore = ball['housing_width_m'], ball['housing_bore_m']
    bottom, top = ball['housing_bottom_z_m'], ball['housing_top_z_m']
    cap = ball['cap_thickness_m']
    cup = ring_cells(bx, hw, hw, (hw-bore)/2, bottom, top-bottom-cap)
    cup.append(([hw, hw, cap], [bx, 0, top-cap/2]))
    fw = ball['flange_width_m']
    cup += ring_cells(bx, fw, fw, (fw-hw)/2, z0, t)
    parts.append(cells_part('caster_housing_link', cup, mass=ball['housing_mass_kg']))
    r = ball['radius_m']
    mass = ball['ball_density_kg_m3']*4*math.pi*r**3/3
    parts.append({'name': 'caster_ball_link', 'role': 'ball', 'mass_kg': mass,
                  'centre_xyz_m': [bx, 0, r],
                  'inertia_kg_m2': (np.eye(3)*.4*mass*r*r).tolist(),
                  'shapes': [{'type': 'sphere', 'radius': r, 'centre': [bx, 0, r]}]})
    parts.append(cells_part('skirt_link', ring_cells(cx, length, width,
                           seal['contact_band_m'], 0, z0), mass=seal['mass_kg']))
    parts[-1]['role'] = 'skirt'  # visual only: plugin is the sole contact path
    payload = legacy_robot['inspection_payload']['camera_body']
    optical = camera['optical_mount']['center_xyz_m']
    camera_size = payload['size_xyz_m']
    camera_centre = [optical[0], optical[1], optical[2]+camera_size[2]/2]
    parts.append(box_part('inspection_camera_link', camera_size, camera_centre, payload['mass_kg']))
    bracket = c['bracket']
    beam_y, beam_t = bracket['width_y_m'], bracket['thickness_m']
    mount_x = x1-bracket['attachment_front_inset_m']
    arm_x0, arm_x1 = mount_x-beam_t/2, optical[0]+camera_size[0]/2
    arm_z0 = camera_centre[2]+camera_size[2]/2
    parts += [hollow_beam('camera_upright', [beam_t, beam_y, arm_z0-z1],
                          [mount_x, 0, (arm_z0+z1)/2], bracket['wall_m'], 2,
                          bracket['density_kg_m3']),
              hollow_beam('camera_arm', [arm_x1-arm_x0, beam_y, beam_t],
                          [(arm_x0+arm_x1)/2, 0, arm_z0+beam_t/2], bracket['wall_m'], 0,
                          bracket['density_kg_m3'])]
    band = seal['contact_band_m']
    inner_area = (length-2*band)*(width-2*band)
    lip_area = length*width-inner_area
    phi = seal['lip_pressure_weight']
    effective = inner_area+phi*lip_area
    # Flat-wall mechanics only. Positive compression reacts at the wall;
    # pressure-weighted lip area is the same term in pressure and skirt loads.
    preload = seal['stiffness_n_m']*(seal['free_height_m']-z0)
    inner = [x0+band, x1-band, -width/2+band, width/2-band]
    clearance = min(bx-cut-inner[0], inner[1]-bx-cut,
                    -cut-inner[2], inner[3]-cut)-ball['mount_tolerance_m']
    wheel_gap = (w['separation_m']-wheel_width)/2-width/2
    fan_gap = min(abs(a-d)-2*fans['radius_m'] for a, d in zip(fan_x, fan_x[1:]))
    cup_fan_gap = min(max(abs(x-bx)-fans['radius_m']-hw/2,
                          fans['centre_z_m']-fans['height_m']/2-top) for x in fan_x)
    # Keep the entire inlet free, not merely the duct wall solid. Otherwise
    # a cup inside a duct would pass an external fan-cylinder clearance test.
    duct_half = port+fans['duct_wall_m']
    cup_duct_gap = min(abs(x-bx)-duct_half-hw/2 for x in fan_x)
    cup_duct_gap -= ball['mount_tolerance_m']
    fan_body_gap = min(min(x-x0, x1-x)-fans['radius_m'] for x in fan_x)
    bay = b['internal_bay_size_xyz_m']
    bay_gap = min((length-bay[0])/2-cover_t,
                  width/2-b['internal_bay_y_m']-bay[1]/2-cover_t,
                  b['internal_bay_y_m']-bay[1]/2-fans['radius_m'],
                  b['internal_bay_z_m']-bay[2]/2-z0-t,
                  z1-cover_t-b['internal_bay_z_m']-bay[2]/2)
    ball_cup_gap = min(bore/2-r, top-cap-2*r)
    housing_inside_cutout = ball['flange_width_m'] >= ball['floor_cutout_m']
    valid = (clearance >= ball['clearance_to_seal_m'] and wheel_gap > 0 and
             fan_gap > 0 and cup_fan_gap > 0 and cup_duct_gap >= .003 and
             fan_body_gap > cover_t and bay_gap > 0 and ball_cup_gap > 0 and
             housing_inside_cutout)
    # Common bottom cavity + inlet ducts, excluding cup walls and spherical cap.
    ball_height = min(z0, 2*r)
    ball_displaced = math.pi*ball_height**2*(r-ball_height/3)
    cup_displaced = (hw*hw-bore*bore)*max(0, z0-bottom)
    volume = inner_area*z0-ball_displaced-cup_displaced
    volume += fans['count']*fans['port_width_m']**2*(fans['centre_z_m']-fans['height_m']/2-z0)
    if not (min(length-2*band, width-2*band, preload, volume) > 0 and 0 <= phi <= 1):
        raise ValueError('invalid skirt/cavity geometry')
    return {'config': c, 'extension_m': extension, 'body_offset_m': body_offset,
            'parts': parts, **combine(parts), 'body_bounds_m': [x0, x1, -width/2, width/2],
            'body_z_m': [z0, z1], 'caster_x_m': bx, 'track_m': w['separation_m'],
            'pressure_centre_xyz_m': [cx, 0, z0], 'inner_area_m2': inner_area,
            'lip_area_m2': lip_area, 'lip_weighted_area_m2': phi*lip_area,
            'effective_area_m2': effective, 'volume_m3': volume,
            'skirt_mechanical_n': preload, 'wheel_width_m': wheel_width,
            'geometry_valid': bool(valid),
            'clearances_m': {'cutout_to_seal_after_mount_tolerance': clearance,
                             'wheel_to_body': wheel_gap, 'adjacent_fans': fan_gap,
                             'cup_to_fan_separating_axis': cup_fan_gap,
                             'cup_to_duct_after_mount_tolerance': cup_duct_gap,
                             'fan_to_body': fan_body_gap, 'internal_bays': bay_gap,
                             'ball_to_cup': ball_cup_gap},
            'camera_optical_xyz_m': list(optical),
            'actuator_limits': deepcopy(legacy_robot['drive']),
            'recommended_footprint': {'length_m': 2*max(abs(x0), abs(arm_x1)),
                                      'width_m': w['outer_width_m'],
                                      'edge_clearance_m': legacy_robot['footprint']['edge_clearance_m']}}


def skirt_points(model):
    b = model['config']['skirt']['contact_band_m']
    x0, x1, y0, y1 = model['body_bounds_m']
    x0, x1, y0, y1 = x0+b/2, x1-b/2, y0+b/2, y1-b/2
    n = model['config']['skirt']['samples_per_side']
    points, lengths = [], []
    for start, end in [((x0,y0),(x1,y0)), ((x1,y0),(x1,y1)),
                       ((x1,y1),(x0,y1)), ((x0,y1),(x0,y0))]:
        start, end = np.array(start), np.array(end)
        for i in range(n):
            points.append([*(start+(end-start)*(i+.5)/n), 0])
            lengths.append(np.linalg.norm(end-start)/n)
    weights = np.array(lengths)/sum(lengths)
    return np.array(points), weights
