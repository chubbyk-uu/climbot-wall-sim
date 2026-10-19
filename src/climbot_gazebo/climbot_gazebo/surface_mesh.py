# Copyright 2026 jerry
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Offline feature-seeded mesh refinement; never called in a physics update."""

from pathlib import Path

from climbot_gazebo.wall_surface import smoothstep, WallSurface
import numpy as np
from scipy.spatial import Delaunay


def normal_cone_angle(measured, first, second):
    """Angle to positive combinations of two independently defined face normals."""
    measured = np.asarray(measured)/np.linalg.norm(measured)
    first, second = np.asarray(first), np.asarray(second)
    correlation = np.clip(first@second, -1, 1)
    a, b = first@measured, second@measured
    best = max(a, b)
    if 1-correlation*correlation > 1e-12:
        alpha = (a-correlation*b)/(1-correlation*correlation)
        beta = (b-correlation*a)/(1-correlation*correlation)
        if alpha >= 0 and beta >= 0:
            projection = alpha*first+beta*second
            length = np.linalg.norm(projection)
            if length > 0:
                best = max(best, measured@projection/length)
    return float(np.degrees(np.arccos(np.clip(best, -1, 1))))


def write_cross_joint_mesh(path, surface, size=.8):
    """Share joint grids and split facets along the actual depth-equality crease."""
    grooves = surface.config['grooves']
    if (len(grooves) != 2 or
            [g['id'] for g in grooves] != ['horizontal_joint', 'vertical_joint']):
        raise ValueError('cross mesh requires the declared horizontal/vertical pair')
    axes = []
    for axis, groove in [(0, grooves[1]), (1, grooves[0])]:
        if (groove.get('cross_section') != 'trapezoid' or groove['end_taper_m'] != 0
                or groove['start_m'][axis] != groove['end_m'][axis]):
            raise ValueError('cross mesh needs axis-aligned untapered trapezoids')
        center, width = groove['start_m'][axis], groove['width_m']
        half, shoulder = width/2, width*groove['shoulder_fraction']
        nodes = np.r_[np.linspace(center-half, center-half+shoulder, 5),
                      np.linspace(center+half-shoulder, center+half, 5)]
        chamfer = groove.get('top_chamfer_m', 0.)
        if chamfer:
            nodes = np.r_[nodes, center-half+chamfer, center+half-chamfer]
        radius = groove.get('rim_radius_m', 0.)
        if radius:
            # Angle-spaced arcs bound normal changes without uniform tiny cells.
            arc = radius*np.sin(np.linspace(0, np.pi/2, 31))
            for side in [-1, 1]:
                nodes = np.r_[nodes, center+side*(half-arc),
                              center+side*(half-shoulder+arc)]
        cells = round(size/.01)
        axes.append(np.unique(np.r_[np.linspace(0, size, cells+1), nodes]))
    u, v = np.meshgrid(*axes)
    points = list(map(tuple, np.column_stack([u.ravel(), v.ravel()])))
    singles = [WallSurface(dict(surface.config, grooves=[g])) for g in grooves]
    values = list(singles[0].groove_depth(u, v).ravel() -
                  singles[1].groove_depth(u, v).ravel())
    # Equal 45-degree faces can differ by floating-point roundoff at shared
    # grid nodes. Canonicalize picometre depth ties before clipping polygons.
    values = [0. if abs(value) <= 1e-12 else value for value in values]
    crossings = {}

    def intersection(a, b):
        if values[a] == 0:
            return a
        if values[b] == 0:
            return b
        key = tuple(sorted([a, b]))
        if key not in crossings:
            start, end = np.array(points[key[0]]), np.array(points[key[1]])
            lo, hi = 0., 1.
            sign = values[key[0]] > 0
            for _ in range(32):
                mid = (lo+hi)/2
                x, y = start+mid*(end-start)
                value = singles[0].groove_depth(x, y)-singles[1].groove_depth(x, y)
                if abs(value) <= 1e-14:
                    lo = hi = mid
                    break
                if (value > 0) == sign:
                    lo = mid
                else:
                    hi = mid
            point = start+(lo+hi)/2*(end-start)
            crossings[key] = len(points)
            points.append(tuple(point))
            values.append(0.)
        return crossings[key]

    faces = []
    columns = len(axes[0])
    for row in range(len(axes[1])-1):
        for column in range(columns-1):
            a = row*columns+column
            for triangle in [[a, a+1, a+columns+1], [a, a+columns+1, a+columns]]:
                signs = [values[i] for i in triangle]
                if min(signs) >= 0 or max(signs) <= 0:
                    faces.append(triangle)
                    continue
                for side in [1, -1]:
                    polygon = []
                    for i, b in enumerate(triangle):
                        a = triangle[i-1]
                        inside_a, inside_b = side*values[a] >= 0, side*values[b] >= 0
                        if inside_a != inside_b:
                            polygon.append(intersection(a, b))
                        if inside_b:
                            polygon.append(b)
                    polygon = list(dict.fromkeys(polygon))
                    faces.extend([polygon[0], polygon[i], polygon[i+1]]
                                 for i in range(1, len(polygon)-1))
    points, faces = np.asarray(points), np.asarray(faces)
    triangle = points[faces]
    area = ((triangle[:, 1, 0]-triangle[:, 0, 0]) *
            (triangle[:, 2, 1]-triangle[:, 0, 1]) -
            (triangle[:, 1, 1]-triangle[:, 0, 1]) *
            (triangle[:, 2, 0]-triangle[:, 0, 0]))
    if not np.all(area > 0) or abs(area.sum()/2-size*size) > 1e-10:
        raise ValueError('cross mesh must cover the patch once with positive faces')
    height = surface.height(points[:, 0], points[:, 1])
    with Path(path).open('x') as stream:
        stream.write('# Shared-grid joints with explicit depth-equality crease\n')
        for (x, y), h in zip(points, height):
            stream.write(f'v {x-size/2:.17g} {y-size/2:.17g} {h:.17g}\n')
        for normal in surface.normal(points[:, 0], points[:, 1]):
            stream.write('vn '+' '.join(f'{x:.17g}' for x in normal)+'\n')
        for face in faces+1:
            stream.write('f '+' '.join(f'{i}//{i}' for i in face)+'\n')
    return {'vertices': len(points), 'triangles': len(faces),
            'explicit_crease_vertices': len(crossings), 'projected_area_m2': area.sum()/2,
            'representation': 'shared_joint_grid_with_depth_equality_split'}


def write_feature_mesh(path, surface, size=.8, tolerance=.00002,
                       max_vertices=80000, max_rounds=16):
    """Resolve grooves locally, rejecting unresolved or over-budget exports."""
    if not 0 < tolerance <= .0001 or size <= 0 or max_vertices < 4 or max_rounds < 1:
        raise ValueError('invalid feature mesh limits')
    axis = np.linspace(0, size, round(size/.01) + 1)
    u, v = np.meshgrid(axis, axis)
    seeds = [np.column_stack([u.ravel(), v.ravel()])]
    for groove in surface.config['grooves']:
        a, b = np.array(groove['start_m']), np.array(groove['end_m'])
        length = np.linalg.norm(b-a)
        direction = (b-a)/length
        normal = np.array([-direction[1], direction[0]])
        spacing = min(.002, groove['width_m'] * .5)
        along = np.linspace(0, length, int(np.ceil(length/spacing)) + 1)
        taper = groove['end_taper_m']
        envelope = (smoothstep(along/taper)*smoothstep((length-along)/taper)
                    if taper else np.ones_like(along))
        shoulder = groove['shoulder_fraction']
        fractions = np.unique(np.r_[-.65, np.linspace(-.5, -.5+shoulder, 17),
                                    0, np.linspace(.5-shoulder, .5, 17), .65])
        points = (a + along[:, None, None]*direction +
                  envelope[:, None, None]*groove['width_m'] *
                  fractions[None, :, None]*normal)
        seeds.append(points.reshape(-1, 2))
    points = np.concatenate(seeds)
    # Nanometre quantization coalesces almost coincident feature / base nodes;
    # picometre slivers otherwise survive Qhull around tapered crack tips.
    points = np.unique(np.round(points[np.all((points >= 0) & (points <= size), axis=1)],
                                9), axis=0)
    # Edge and interior samples detect both groove caps and inaccurate facets.
    weights = np.array([[1/3, 1/3, 1/3], [.5, .5, 0], [.5, 0, .5], [0, .5, .5],
                        [.6, .2, .2], [.2, .6, .2], [.2, .2, .6]])
    history = []
    for iteration in range(max_rounds):
        if len(points) > max_vertices:
            raise ValueError(f'feature mesh vertex budget exceeded; no export written: {history}')
        triangulation = Delaunay(points)
        faces = triangulation.simplices
        height = surface.height(points[:, 0], points[:, 1])
        locations = np.einsum('kj,fjd->fkd', weights, points[faces])
        interpolated = np.einsum('kj,fj->fk', weights, height[faces])
        error = np.abs(surface.height(locations[:, :, 0], locations[:, :, 1])-interpolated)
        maximum = float(error.max())
        history.append({'round': iteration, 'vertices': len(points),
                        'triangles': len(faces), 'sampled_height_error_max_m': maximum})
        if maximum <= tolerance:
            break
        added = locations[error > tolerance]
        updated = np.unique(np.round(np.concatenate([points, added]), 9), axis=0)
        if len(updated) == len(points):
            raise ValueError('feature mesh refinement stalled; no export written')
        points = updated
    else:
        raise ValueError(f'feature mesh did not converge; no export written: {history}')
    triangles = points[faces]
    cross = ((triangles[:, 1, 0]-triangles[:, 0, 0]) *
             (triangles[:, 2, 1]-triangles[:, 0, 1]) -
             (triangles[:, 1, 1]-triangles[:, 0, 1]) *
             (triangles[:, 2, 0]-triangles[:, 0, 0]))
    if not np.all(cross > 0):
        raise ValueError('feature mesh contains reversed or degenerate faces')
    with Path(path).open('x') as stream:
        stream.write('# M2 feature-refined source mesh; no underlying collision plane\n')
        for (u, v), h in zip(points, height):
            stream.write(f'v {u-size/2:.17g} {v-size/2:.17g} {h:.17g}\n')
        for normal in surface.normal(points[:, 0], points[:, 1]):
            stream.write('vn ' + ' '.join(f'{value:.17g}' for value in normal) + '\n')
        for face in faces + 1:
            stream.write('f ' + ' '.join(f'{index}//{index}' for index in face) + '\n')
    return {'vertices': len(points), 'triangles': len(faces),
            'sampled_height_error_max_m': maximum, 'height_tolerance_m': tolerance,
            'min_projected_double_area_m2': float(cross.min()),
            'refinement': history, 'vertex_budget': max_vertices,
            'extent_m': [0., size], 'runtime_refinement': False}


def audit_mesh_sections(path, surface, size=.8):
    """Measure the exported faces at independent dense cuts through each groove."""
    from matplotlib.tri import LinearTriInterpolator, Triangulation

    vertices, faces = [], []
    for line in Path(path).read_text().splitlines():
        if line.startswith('v '):
            vertices.append([float(x) for x in line.split()[1:]])
        elif line.startswith('f '):
            faces.append([int(x.split('//')[0])-1 for x in line.split()[1:]])
    vertices = np.asarray(vertices)
    triangulation = Triangulation(vertices[:, 0]+size/2, vertices[:, 1]+size/2,
                                  np.asarray(faces))
    interpolator = LinearTriInterpolator(triangulation, vertices[:, 2])
    sections = []
    for groove in surface.config['grooves']:
        a, b = np.array(groove['start_m']), np.array(groove['end_m'])
        direction = (b-a)/np.linalg.norm(b-a)
        across = np.array([-direction[1], direction[0]])
        for fraction in [.02, .25, .5, .75, .98]:
            offsets = np.linspace(-groove['width_m'], groove['width_m'], 1001)
            positions = a + fraction*(b-a) + offsets[:, None]*across
            if not np.all((positions >= 0) & (positions <= size)):
                continue
            u, v = positions.T
            actual = interpolator(u, v)
            if np.ma.getmaskarray(actual).any():
                raise ValueError('mesh has unsupported section samples')
            expected_depth = surface.groove_depth(u, v)
            actual_depth = np.maximum(0, surface.roughness(u, v)-actual)
            expected_area = float(np.trapz(expected_depth, offsets))
            actual_area = float(np.trapz(actual_depth, offsets))
            area_error = abs(actual_area-expected_area)
            height_error = float(np.max(np.abs(actual-surface.height(u, v))))
            sections.append({'groove': groove['id'], 'fraction': fraction,
                             'source_area_m2': expected_area, 'mesh_area_m2': actual_area,
                             'area_error_m2': area_error,
                             'height_error_max_m': height_error,
                             'passed': height_error <= .0001 and area_error <= (
                                 .05*expected_area if expected_area > 1e-8 else 1e-8)})
    return {'passed': bool(sections) and all(s['passed'] for s in sections),
            'samples_per_section': 1001, 'sections': sections,
            'scope': 'exported OBJ faces, not a native collision or leak-flow measurement'}
