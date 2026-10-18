"""Small deterministic display meshes; never used for inertia or contact.

The photographic references inform silhouettes, not measured dimensions.
Mechanical geometry remains in vacuum_layout.py. OBJ triangles are generated
beside each exported model so the inspected SDF and URDF share their assets.
"""
import math
from pathlib import Path

import numpy as np


def _mesh(directory, name, vertices, faces, centre, color, smooth=False):
    path = Path(directory) / (name + '.obj')
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = ['# Climbot display geometry, metres; no collision or mass.']
    lines += ['v ' + ' '.join(format(float(x), '.12g') for x in v) for v in vertices]
    normals = np.zeros((len(vertices) if smooth else len(faces), 3))
    for n, face in enumerate(faces):
        a, b, c = [np.asarray(vertices[i]) for i in face]
        normal = np.cross(b - a, c - a)
        if np.linalg.norm(normal) <= 1e-16:
            raise ValueError('degenerate display triangle')
        if smooth:
            for i in face:
                normals[i] += normal
        else:
            normals[n] = normal
    for normal in normals:
        normal /= max(np.linalg.norm(normal), 1e-30)
        lines.append('vn ' + ' '.join(format(float(x), '.12g') for x in normal))
    for n, face in enumerate(faces, 1):
        lines.append('f ' + ' '.join(f'{i + 1}//{i + 1 if smooth else n}' for i in face))
    path.write_text('\n'.join(lines) + '\n')
    return {'type': 'mesh', 'uri': path.resolve().as_uri(), 'centre': list(centre),
            'color': color, 'display_name': name}


def _ring(vertices, faces, radius, width, bottom, height, count=48):
    start = len(vertices)
    for z, r in [(bottom, radius), (bottom + height, radius),
                 (bottom + height, radius - width), (bottom, radius - width)]:
        vertices.extend([(r * math.cos(i * 2 * math.pi / count),
                          r * math.sin(i * 2 * math.pi / count), z) for i in range(count)])
    for level in range(4):
        for i in range(count):
            a = start + level * count + i
            b = start + level * count + (i + 1) % count
            c = start + ((level + 1) % 4) * count + (i + 1) % count
            d = start + ((level + 1) % 4) * count + i
            faces.extend([(a, b, c), (a, c, d)])


def _panel(directory, name, size, centre, color, bevel=.004):
    # An extruded chamfered x/z outline, giving rails and camera a clean edge.
    x, y, z = [float(v) / 2 for v in size]
    b = min(bevel, x * .6, z * .6)
    outline = [(-x+b, -z), (x-b, -z), (x, -z+b), (x, z-b),
               (x-b, z), (-x+b, z), (-x, z-b), (-x, -z+b)]
    vertices = [(u, s*y, v) for s in [-1, 1] for u, v in outline]
    faces = []
    for i in range(1, 7):
        faces.extend([(0, i, i+1), (8, 8+i+1, 8+i)])
    for i in range(8):
        j = (i+1) % 8
        faces.extend([(i, 8+i, 8+j), (i, 8+j, j)])
    return _mesh(directory, name, vertices, faces, centre, color)


def _round_tube(directory, name, samples, radius, centre, color):
    """Sweep a circular section along a smooth path, including its bends."""
    vertices, faces = [], []
    count = 20
    normal = np.asarray([0., 0., 1.])
    first = np.asarray(samples[0][1], dtype=float)
    if abs(np.dot(first/np.linalg.norm(first), normal)) > .9:
        normal = np.asarray([0., 1., 0.])
    for point, direction in samples:
        tangent = np.asarray(direction, dtype=float)
        tangent /= np.linalg.norm(tangent)
        # Transport the section through bends without switching reference axes.
        normal = normal-tangent*np.dot(normal, tangent)
        normal /= np.linalg.norm(normal)
        side = np.cross(tangent, normal)
        for angle in np.linspace(0, 2*math.pi, count, endpoint=False):
            p = (np.asarray(point)+radius*(math.cos(angle)*normal+math.sin(angle)*side)
                 -np.asarray(centre))
            vertices.append(p.tolist())
    for i in range(len(samples)-1):
        for j in range(count):
            a, b = i*count+j, i*count+(j+1)%count
            c, d = (i+1)*count+(j+1)%count, (i+1)*count+j
            faces.extend([(a, b, c), (a, c, d)])
    last = (len(samples)-1)*count
    for i in range(1, count-1):
        faces.extend([(0, i+1, i), (last, last+i, last+i+1)])
    return _mesh(directory, name, vertices, faces, centre, color, smooth=True)


def display_shapes(part, model, camera, directory):
    """Return visual overrides only; caller always exports original collisions."""
    name = part['name']
    centre = part['centre_xyz_m']
    if name == 'chassis_link':
        shapes = list(part['shapes'])
        body_half = model['config']['body']['width_m']/2
        wheel_half = model['wheel_width_m']/2
        axle_y = model['track_m']/2-wheel_half
        z = model['config']['wheel']['radius_m']
        for side in [-1, 1]:
            label = 'left' if side == 1 else 'right'
            shapes.extend([
                {'type': 'box', 'size': [.064, .008, .052],
                 'centre': [0, side*(body_half+.003), z],
                 'color': [.24, .27, .30, 1], 'display_name': label+'_bearing_mount'},
                {'type': 'cylinder', 'radius': .011, 'length': axle_y-body_half+.012,
                 'axis': 'y', 'centre': [0, side*(body_half+axle_y)/2, z],
                 'color': [.52, .55, .58, 1], 'display_name': label+'_drive_axle'},
            ])
        return shapes
    if part['role'] == 'drive':
        shapes = list(part['shapes'])
        side = 1 if centre[1] > 0 else -1
        y = centre[1]+side*(model['wheel_width_m']/2-.003)
        shapes.extend([
            {'type': 'cylinder', 'radius': .029, 'length': .003, 'axis': 'y',
             'centre': [centre[0], y, centre[2]],
             'color': [.47, .50, .53, 1], 'display_name': name+'_hub'},
            {'type': 'cylinder', 'radius': .012, 'length': .004, 'axis': 'y',
             'centre': [centre[0], y+side*.001, centre[2]],
             'color': [.15, .17, .19, 1], 'display_name': name+'_axle_cap'},
        ])
        return shapes
    if name == 'camera_arm':
        # One continuous curved display mesh is attached to camera_upright.
        return []
    if name == 'camera_upright':
        support = model['support_geometry']
        path = [(np.asarray(p), np.asarray(d)) for p, d in support['main']]
        z0 = model['config']['body']['top_z_m']
        vertices, faces = [], []

        def tube(samples, width, height, end_width=None, end_height=None, rounding=.26):
            first = len(vertices)
            count = 16
            for i, (p, tangent) in enumerate(samples):
                t = i/(len(samples)-1)
                if end_width is None:
                    scale = 1-.15*t
                    w, h = width*scale, height*scale
                else:
                    blend = t*t*(3-2*t)
                    w = width+(end_width-width)*blend
                    h = height+(end_height-height)*blend
                r = min(w, h)*rounding
                cross = []
                for cy, cn, start in [(w/2-r, h/2-r, 0), (-w/2+r, h/2-r, 90),
                                      (-w/2+r, -h/2+r, 180), (w/2-r, -h/2+r, 270)]:
                    for angle in np.linspace(start, start+90, 4):
                        a = math.radians(angle)
                        cross.append((cy+r*math.cos(a), cn+r*math.sin(a)))
                normal = np.asarray([-tangent[1], tangent[0]])
                for y, n in cross:
                    point = p+normal*n
                    vertices.append((point[0]-centre[0], y, point[1]-centre[2]))
            for i in range(len(samples)-1):
                for j in range(count):
                    a = first+i*count+j
                    b = first+i*count+(j+1)%count
                    c = first+(i+1)*count+(j+1)%count
                    d = first+(i+1)*count+j
                    faces.extend([(a, b, c), (a, c, d)])
            last = first+(len(samples)-1)*count
            for i in range(1, count-1):
                faces.extend([(first, first+i+1, first+i), (last, last+i, last+i+1)])

        tube(path, .060, .042)
        silver = [.76, .79, .82, 1]
        shapes = [_mesh(directory, 'camera_main_arm', vertices, faces, centre,
                        silver, smooth=True)]
        # A rear-mounted carry handle replaces the local diagonal brace.
        # The narrow round rails leave an open centre, with some outlet-rim
        # overlap in top view. This display mesh does not model airflow loss.
        height = support['height_m']
        spread = model['config']['bracket']['handle_spacing_m']/2
        rear = support['rear_foot_xyz_m'][0]
        for sign, label, samples in zip([1, -1], ['left', 'right'], support['handles']):
            shapes.append(_round_tube(directory, label+'_carry_handle', samples,
                                      .011, centre, silver))
            shapes.append(_round_tube(
                directory, label+'_handle_grip',
                [([v, sign*spread, height], [-1, 0, 0]) for v in [-.070, -.200]],
                .012, centre, [.12, .14, .16, 1]))
        # A single sculpted rear support receives both rails in a broad head,
        # then slopes rearward into a thicker foot, including at its lower end.
        vertices, faces = [], []
        rear_path = [(np.asarray(p), np.asarray(d)) for p, d in support['rear']]
        tube(rear_path, .106, .028, end_width=.060, end_height=.042, rounding=.42)
        shapes.append(_mesh(directory, 'handle_rear_support', vertices, faces,
                            centre, silver, smooth=True))
        shapes.append(_panel(
            directory, 'handle_rear_foot', [.065, .075, .010],
            [rear, 0, z0+.004], [.27, .30, .33, 1], bevel=.004))
        return shapes
    if name == 'inspection_camera_link':
        optical = camera['optical_mount']['center_xyz_m']
        body_centre = [optical[0], optical[1], optical[2]+.054]
        shapes = [
            _panel(directory, 'camera_housing', [.055, .065, .044], body_centre,
                   [.05, .20, .32, 1], bevel=.007),
            {'type': 'cylinder', 'radius': .020, 'length': .032,
             'centre': [optical[0], optical[1], optical[2]+.016],
             'color': [.055, .06, .065, 1], 'display_name': 'lens_barrel'},
            {'type': 'cylinder', 'radius': .0215, 'length': .003,
             'centre': [optical[0], optical[1], optical[2]+.004],
             'color': [.30, .32, .34, 1], 'display_name': 'lens_focus_ring'},
            {'type': 'cylinder', 'radius': .016, 'length': .001,
             'centre': [optical[0], optical[1], optical[2]+.001],
             'color': [.025, .065, .09, 1], 'display_name': 'lens_glass'},
        ]
        shapes.append({'type': 'box', 'size': [.055, .130, .012],
                       'centre': [optical[0], optical[1], optical[2]+.079],
                       'color': [.52, .55, .58, 1], 'display_name': 'camera_light_crossbar'})
        for side in [-1, 1]:
            label = 'left' if side == 1 else 'right'
            p = [optical[0], optical[1]+side*.050, optical[2]+.039]
            shapes.append(_panel(directory, label+'_lamp_housing', [.037, .029, .028],
                                 p, [.10, .12, .14, 1], bevel=.004))
            shapes.append({'type': 'box', 'size': [.009, .012, .025],
                           'centre': [p[0], p[1], optical[2]+.064],
                           'color': [.40, .43, .46, 1], 'display_name': label+'_lamp_mount'})
            shapes.append({'type': 'box', 'size': [.031, .023, .002],
                           'centre': [p[0], p[1], p[2]-.014],
                           'color': [.66, .69, .71, 1], 'display_name': label+'_reflector'})
            shapes.append({'type': 'box', 'size': [.023, .015, .001],
                           'centre': [p[0], p[1], p[2]-.0155],
                           'color': [.92, .95, 1, 1], 'emissive': [.65, .70, .8, 1],
                           'display_name': label+'_led_face'})
        return shapes
    if name.startswith('fan_') and name.endswith('_link'):
        top = model['config']['body']['top_z_m'] - centre[2]
        vertices, faces = [], []
        _ring(vertices, faces, .042, .004, top-.001, .009)
        rim = _mesh(directory, name+'_outlet', vertices, faces, centre,
                    [.48, .51, .54, 1])
        vertices, faces = [], []
        # Seven swept blades behind a light guard; GUI-only animation, no joints.
        for i in range(7):
            angle = i*2*math.pi/7
            blade = [(r*math.cos(angle+a), r*math.sin(angle+a), top+.002)
                     for r, a in [(.010, -.10), (.034, .02), (.035, .36), (.012, .48)]]
            j = len(vertices)
            vertices.extend(blade)
            faces.extend([(j, j+1, j+2), (j, j+2, j+3)])
        blades = _mesh(directory, name+'_blades', vertices, faces, centre,
                       [.10, .12, .14, 1])
        blades['animated_rotor'] = True
        vertices, faces = [], []
        for r in [.021, .031, .038]:
            _ring(vertices, faces, r, .0008, top+.006, .0012)
        for angle in [0, 2*math.pi/3, 4*math.pi/3]:
            j = len(vertices)
            for z in [top+.006, top+.0072]:
                for r, side in [(.009, -.0006), (.038, -.0006),
                                (.038, .0006), (.009, .0006)]:
                    vertices.append((r*math.cos(angle)-side*math.sin(angle),
                                     r*math.sin(angle)+side*math.cos(angle), z))
            faces.extend(tuple(j+i for i in f) for f in
                         [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
                          (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
                          (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)])
        guard = _mesh(directory, name+'_guard', vertices, faces, centre,
                      [.36, .39, .42, 1])
        return [rim, blades, guard,
                {'type': 'cylinder', 'radius': .038, 'length': .001,
                 'centre': [centre[0], centre[1], centre[2]+top],
                 'color': [.025, .03, .035, 1], 'display_name': name+'_throat'},
                {'type': 'cylinder', 'radius': .0105, 'length': .005,
                 'centre': [centre[0], centre[1], centre[2]+top+.003],
                 'color': [.23, .25, .27, 1], 'display_name': name+'_hub'}]
    if part['role'] == 'skirt':
        body = model['config']['body']
        x0, x1, y0, y1 = model['body_bounds_m']
        # A shallow outward drape makes the cloth visible outside the side rail.
        # This is a visual proxy, not an added pressure area or collision lip.
        vertices, faces = [], []
        count = 32
        edges = [((x0, y0), (x1, y0), (0, -1)),
                 ((x1, y0), (x1, y1), (1, 0)),
                 ((x1, y1), (x0, y1), (0, 1)),
                 ((x0, y1), (x0, y0), (-1, 0))]
        for start, end, normal in edges:
            first = len(vertices)
            for height, flare in [(body['bottom_z_m']+.006, 0), (.011, .008), (.002, .016)]:
                for i in range(count+1):
                    t = i/count
                    ripple = .0012*math.sin(12*math.pi*t)*math.sin(math.pi*t)
                    mid = [(x0+x1)/2, (y0+y1)/2]
                    p = [(1-t)*(start[k]+math.copysign(flare, start[k]-mid[k]))
                         + t*(end[k]+math.copysign(flare, end[k]-mid[k]))
                         + normal[k]*ripple for k in range(2)]
                    vertices.append((p[0]-centre[0], p[1]-centre[1], height-centre[2]))
            for row in range(2):
                for i in range(count):
                    a = first+row*(count+1)+i
                    b, c, d = a+1, a+count+2, a+count+1
                    faces.extend([(a, c, b), (a, d, c), (a, b, c), (a, c, d)])
        shapes = [_mesh(directory, 'skirt_outward_cloth', vertices, faces, centre,
                        [.055, .06, .065, 1])]
        for i, (size, pos) in enumerate([
                ([x1-x0+.004, .002, .006], [(x0+x1)/2, y0-.001, .024]),
                ([x1-x0+.004, .002, .006], [(x0+x1)/2, y1+.001, .024]),
                ([.002, y1-y0, .006], [x0-.001, 0, .024]),
                ([.002, y1-y0, .006], [x1+.001, 0, .024])]):
            shapes.append({'type': 'box', 'size': size, 'centre': pos,
                           'color': [.48, .50, .52, 1],
                           'display_name': 'skirt_retaining_strip_'+str(i)})
        return shapes
    return part['shapes']
