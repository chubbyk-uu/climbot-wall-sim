# Copyright 2026 jerry
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Resolve one robot variant for planning, control and independent evaluators."""

from copy import deepcopy
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml

GEOMETRIES = ('vacuum_chassis',)


def resolve_adhesion(geometry, adhesion):
    """Validate the pair before starting any robot process."""
    if geometry not in GEOMETRIES:
        raise ValueError('robot_geometry must be vacuum_chassis')
    if adhesion == 'auto':
        adhesion = 'vacuum'
    allowed = ('fixed_pressure', 'vacuum')
    if adhesion not in allowed:
        raise ValueError(f'invalid geometry/adhesion pair: {geometry}/{adhesion}')
    return adhesion


def _bounds(parts):
    low = [math.inf] * 3
    high = [-math.inf] * 3
    for part in parts:
        for shape in part['shapes']:
            kind = shape['type']
            if kind == 'box':
                half = [v / 2 for v in shape['size']]
            elif kind == 'sphere':
                half = [shape['radius']] * 3
            elif kind == 'cylinder':
                half = [shape['radius']] * 3
                half[1 if shape.get('axis') == 'y' else 2] = shape['length'] / 2
            else:
                raise ValueError(f'unsupported robot envelope shape: {kind}')
            for axis in range(3):
                low[axis] = min(low[axis], shape['centre'][axis] - half[axis])
                high[axis] = max(high[axis], shape['centre'][axis] + half[axis])
    return low, high


def load_robot(geometry='vacuum_chassis', description_share=None):
    """Derive the accepted robot from the shared mechanical source."""
    if geometry not in GEOMETRIES:
        raise ValueError('robot_geometry must be vacuum_chassis')
    share = Path(description_share or get_package_share_directory('climbot_description'))
    robot = yaml.safe_load((share / 'config/robot.yaml').read_text())['robot']
    from climbot_description.vacuum_layout import build, combine
    config = yaml.safe_load((share / 'config/vacuum_chassis.yaml').read_text())['vacuum_chassis']
    camera_file = share / 'config/inspection_camera.yaml'
    camera = yaml.safe_load(camera_file.read_text())['inspection_camera']
    model = build(config, camera, robot)
    if not model['geometry_valid']:
        raise ValueError('candidate geometry rejected')
    selected = deepcopy(robot)
    parts = {part['name']: part for part in model['parts']}
    bracket = [part for name, part in parts.items() if name in ('camera_upright', 'camera_arm')]
    excluded = {'left_wheel_link', 'right_wheel_link', 'caster_ball_link',
                'inspection_camera_link', 'camera_upright', 'camera_arm'}
    base_parts = [part for name, part in parts.items() if name not in excluded]
    base = combine(base_parts)
    body = config['body']
    selected['base'] = {'size_xyz': [
        body['length_m'], body['width_m'], body['top_z_m'] - body['bottom_z_m']],
                        'mass_kg': base['mass_kg'], 'centre_xyz': base['centre_xyz_m'],
                        'inertia_kg_m2': base['inertia_kg_m2']}
    wheel = config['wheel']
    selected['drive_wheel'].update(
        radius_m=wheel['radius_m'], separation_m=wheel['separation_m'],
        width_m=wheel['outer_width_m'] - wheel['separation_m'], mass_kg=wheel['mass_each_kg'])
    selected['caster'].update(radius_m=config['caster']['radius_m'],
                              mass_kg=parts['caster_ball_link']['mass_kg'],
                              centre_x_m=model['caster_x_m'])
    bracket_mass = combine(bracket)
    low, high = _bounds(bracket)
    selected['inspection_payload']['bracket'].update(
        mass_kg=bracket_mass['mass_kg'], centre_xyz_m=bracket_mass['centre_xyz_m'],
        inertia_box_size_xyz_m=[high[i] - low[i] for i in range(3)])
    for name, prefix in [('camera_upright', 'upright'), ('camera_arm', 'top_arm')]:
        low, high = _bounds([parts[name]])
        selected['inspection_payload']['bracket'].update({
            prefix + '_size_xyz_m': [high[i] - low[i] for i in range(3)],
            prefix + '_centre_xyz_m': [(high[i] + low[i]) / 2 for i in range(3)]})
    low, high = _bounds(model['parts'])
    selected['footprint'] = {
        'length_m': 2 * max(abs(low[0]), abs(high[0])),
        'width_m': 2 * max(abs(low[1]), abs(high[1])),
        'edge_clearance_m': robot['footprint']['edge_clearance_m'],
        'bounds_xy_m': [low[0], high[0], low[1], high[1]],
    }
    selected['mass_properties'] = {key: model[key] for key in [
        'mass_kg', 'centre_xyz_m', 'inertia_kg_m2']}
    selected['geometry_name'] = geometry
    return selected
