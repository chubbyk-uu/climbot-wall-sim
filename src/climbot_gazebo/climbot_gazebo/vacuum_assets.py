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

"""Generate the explicit S3 candidate from the shared S1/S2 sources."""

from copy import deepcopy
import importlib.util
import json
import math
from pathlib import Path
from xml.etree import ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from climbot_description.geometry import quaternion_from_rpy, quaternion_multiply, rotate_vector
from climbot_description.vacuum_layout import build, skirt_points
from climbot_description.vacuum_loads import minimum_pressure, states
from climbot_description.vacuum_preview import export, node, numbers
import yaml


def _read(path, key):
    return yaml.safe_load(path.read_text())[key]


def _validate_io(io, step_s):
    if set(io) != {'schema_version', 'truth_rate_hz', 'sensor', 'events'}:
        raise ValueError('invalid vacuum IO keys')
    sensor = io['sensor']
    if set(sensor) != {'rate_hz', 'noise_pa', 'bias_pa', 'delay_s', 'minimum_pa',
                       'maximum_pa', 'seed'} or io['schema_version'] != 1:
        raise ValueError('invalid sensor schema')
    for value in [io['truth_rate_hz'], *sensor.values()]:
        if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                not math.isfinite(value)):
            raise ValueError('nonfinite/non-numeric sensor parameter')
    if (not isinstance(sensor['seed'], int) or not 0 <= sensor['seed'] <= 2**32 - 1 or
            sensor['noise_pa'] < 0 or not 0 <= sensor['delay_s'] <= 10 or
            not 0 < sensor['minimum_pa'] < sensor['maximum_pa']):
        raise ValueError('invalid sensor domain')
    for rate in [io['truth_rate_hz'], sensor['rate_hz']]:
        if not 0 < rate <= 1000 or abs(1 / rate / step_s - round(1 / rate / step_s)) > 1e-8:
            raise ValueError('publication rate must align with physical step')
    if abs(sensor['delay_s'] / step_s - round(sensor['delay_s'] / step_s)) > 1e-8:
        raise ValueError('sensor delay must align with physical step')
    previous = -1
    for event in io['events']:
        if set(event) != {'time_s', 'kind', 'index', 'value'}:
            raise ValueError('invalid scheduled command keys')
        time = event['time_s']
        if (isinstance(time, bool) or not isinstance(time, (int, float)) or
                not math.isfinite(time) or time < 0 or time < previous):
            raise ValueError('scheduled commands must be ordered by simulation time')
        kind, index, value = [event[key] for key in ['kind', 'index', 'value']]
        if (any(type(x) is not int for x in [kind, index, value]) or
                kind not in (0, 1, 2) or index not in range(3) or
                (kind == 2 and (index != 0 or value not in range(3))) or
                (kind != 2 and value not in (0, 1))):
            raise ValueError('invalid scheduled command domain')
        previous = time


def _rpy(quaternion):
    columns = [rotate_vector(quaternion, axis) for axis in [(1, 0, 0), (0, 1, 0), (0, 0, 1)]]
    r = [[columns[j][i] for j in range(3)] for i in range(3)]
    pitch = math.atan2(-r[2][0], math.hypot(r[0][0], r[1][0]))
    if abs(math.cos(pitch)) < 1e-10:
        return [math.atan2(-r[1][2], r[1][1]), pitch, 0.0]
    return [math.atan2(r[2][1], r[2][2]), pitch, math.atan2(r[1][0], r[0][0])]


def _legacy_module(gazebo):
    path = gazebo / 'launch' / 'climbot_wall.launch.py'
    spec = importlib.util.spec_from_file_location('frozen_climbot_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def build_assets(directory, mode='fan_ode', heading_deg=0.0, step_s=0.001,
                 realtime_factor=1.0, runtime_overrides=None, io_overrides=None):
    """Generate an inspectable runtime world, model, URDF and resolved inputs."""
    if mode not in ('fan_ode', 'fixed_pressure'):
        raise ValueError('pressure mode must be fan_ode or fixed_pressure')
    if not math.isfinite(heading_deg) or not 0 < step_s <= 0.001:
        raise ValueError('invalid heading or physical step')
    out = Path(directory)
    out.mkdir(parents=True, exist_ok=True)
    description = Path(get_package_share_directory('climbot_description'))
    gazebo = Path(get_package_share_directory('climbot_gazebo'))
    cfg = _read(description / 'config/vacuum_chassis.yaml', 'vacuum_chassis')
    camera = _read(description / 'config/inspection_camera.yaml', 'inspection_camera')
    robot = _read(description / 'config/robot.yaml', 'robot')
    wall = _read(description / 'config/wall.yaml', 'wall')
    simulation = _read(gazebo / 'config/simulation.yaml', 'simulation')
    physics = _read(gazebo / 'config/vacuum_physics.yaml', 'vacuum_physics')
    runtime = _read(gazebo / 'config/vacuum_runtime.yaml', 'vacuum_runtime')
    io = _read(gazebo / 'config/vacuum_io.yaml', 'vacuum_io')
    if io_overrides:
        if set(io_overrides) - set(io):
            raise ValueError('unknown vacuum IO override')
        for key, value in io_overrides.items():
            if key == 'sensor':
                io[key].update(value)
            else:
                io[key] = value
    _validate_io(io, step_s)
    if runtime_overrides:
        if set(runtime_overrides) - set(runtime):
            raise ValueError('unknown runtime override')
        runtime.update(runtime_overrides)
    model = build(cfg, camera, robot)
    if not model['geometry_valid']:
        raise ValueError('candidate geometry rejected')
    export(model, camera, out / 'model.sdf')
    export(model, camera, out / 'robot.urdf', urdf=True)
    sdf = ET.parse(out / 'model.sdf').getroot()
    new = sdf.find('model')
    new.set('name', 'climbot')
    urdf = ET.parse(out / 'robot.urdf').getroot()
    urdf.set('name', 'climbot')
    for tree in (new, urdf):
        for side in ['left', 'right']:
            tree.find(f'joint[@name="{side}_wheel_link_joint"]').set('name', side + '_wheel_joint')
    node(new, 'self_collide', 'false')
    # Read the existing generated sensor and actuator definitions. Their
    # numeric inputs remain the shared configuration, not a copied template.
    legacy = _legacy_module(gazebo)
    mappings = legacy.robot_mappings(str(gazebo), str(description))
    mappings.update(wheel_radius=str(cfg['wheel']['radius_m']),
                    wheel_separation=str(cfg['wheel']['separation_m']))
    import xacro
    old = ET.fromstring(xacro.process_file(
        str(gazebo / 'models/climbot/model.sdf.xacro'), mappings=mappings).toxml()).find('model')
    for link_name in ['chassis_link', 'inspection_camera_link']:
        source = old.find(f'link[@name="{link_name}"]')
        target = new.find(f'link[@name="{link_name}"]')
        old_pose = [float(v) for v in source.findtext('pose', '0 0 0 0 0 0').split()]
        new_pose = [float(v) for v in target.findtext('pose', '0 0 0 0 0 0').split()]
        for sensor in source.findall('sensor'):
            if sensor.get('type') == 'camera' and not runtime['camera_enabled']:
                continue
            item = deepcopy(sensor)
            position = item.find('pose')
            text = position.text if position is not None else '0 0 0 0 0 0'
            local = [float(v) for v in text.split()]
            if position is None:
                position = node(item, 'pose')
            position.text = numbers(
                [local[i] + old_pose[i] - new_pose[i] for i in range(3)] + local[3:])
            target.append(item)
    # Keep the shared optical lighting contract when the camera is enabled.
    source = old.find('link[@name="inspection_camera_bracket_link"]')
    if source is not None:
        old_pose = [float(v) for v in source.findtext('pose').split()]
        target = new.find('link[@name="chassis_link"]')
        new_pose = [float(v) for v in target.findtext('pose').split()]
        for light in source.findall('light'):
            item = deepcopy(light)
            position = item.find('pose')
            local = [float(v) for v in position.text.split()]
            position.text = numbers(
                [local[i] + old_pose[i] - new_pose[i] for i in range(3)] + local[3:])
            target.append(item)
    for side, link_name in [('left', 'left_wheel_link'), ('right', 'right_wheel_link'),
                            ('caster', 'caster_ball_link')]:
        link = new.find(f'link[@name="{link_name}"]')
        collision = link.find('collision')
        collision.set('name', side + '_collision')
        friction = node(node(collision, 'surface'), 'friction')
        ode = node(friction, 'ode')
        mu = simulation['caster']['mu'] if side == 'caster' else simulation['drive_wheel']['mu']
        node(ode, 'mu', mu)
        node(ode, 'mu2', mu)
        sensor = node(link, 'sensor', name=side + '_contact', type='contact')
        node(sensor, 'always_on', 'true')
        node(sensor, 'update_rate', 0)  # physics-step ECM data, no decimated observer
        node(node(sensor, 'contact'), 'collision', side + '_collision')
    for plugin in old.findall('plugin'):
        new.append(deepcopy(plugin))
    # Preserve the original compliances / nominal load ratio. S3 measures
    # actual load and slip, rather than silently changing this tyre model.
    sp = node(new, 'plugin', filename='climbot_vacuum_system', name='climbot_gazebo::VacuumSystem')
    node(sp, 'pressure_mode', mode)
    io_element = node(sp, 'io')
    node(io_element, 'truth_rate_hz', io['truth_rate_hz'])
    sensor = node(io_element, 'sensor')
    for key, value in io['sensor'].items():
        node(sensor, key, value)
    for event in io['events']:
        element = node(io_element, 'event')
        for key, value in event.items():
            node(element, key, value)
    for key, value in runtime.items():
        if key not in ('schema_version', 'pressure_mode', 'camera_enabled'):
            node(sp, key, str(value).lower() if isinstance(value, bool) else value)
    node(sp, 'model_mass_kg', model['mass_kg'])
    node(sp, 'model_cg', numbers(model['centre_xyz_m']))
    node(sp, 'track_m', model['track_m'])
    node(sp, 'caster_x_m', model['caster_x_m'])
    node(sp, 'caster_radius_m', cfg['caster']['radius_m'])
    for key in ['caster_rolling_coefficient', 'caster_rolling_regularization_mps']:
        node(sp, key, cfg['analysis'][key])
    node(sp, 'inner_area_m2', model['inner_area_m2'])
    node(sp, 'nominal_volume_m3', model['volume_m3'])
    node(sp, 'centre_x_m', cfg['body']['centre_x_m'])
    node(sp, 'nominal_gap_m', cfg['body']['bottom_z_m'])
    node(sp, 'drive_mu', min(simulation['drive_wheel']['mu'], simulation['wall']['mu']))
    for key in ['margin', 'drive_floor_n', 'caster_floor_n']:
        node(sp, key, cfg['analysis'][key])
    node(sp, 'reference_area_m2', physics['calibration']['reference_area_m2'])
    node(sp, 'residual_area_m2', physics['calibration']['residual_area_m2'])
    nominal_minimum = float(minimum_pressure(
        model, states([heading_deg]), lateral='conservative')[0])
    node(sp, 'nominal_startup_minimum_pa', nominal_minimum)
    gas = node(sp, 'gas')
    for key, value in physics['gas'].items():
        node(gas, key, value)
    for fan in physics['fans']:
        element = node(sp, 'fan')
        for key, value in fan.items():
            node(element, key, value)
    skirt = node(sp, 'skirt')
    for key, value in {**cfg['skirt'], **physics['skirt']}.items():
        node(skirt, key, value)
    points, weights = skirt_points(model)
    perimeter = 2 * (cfg['body']['length_m'] + cfg['body']['width_m'] -
                     2 * cfg['skirt']['contact_band_m'])
    for point, weight in zip(points, weights):
        element = node(sp, 'sample')
        node(element, 'attachment', numbers(
            [point[0], point[1], cfg['body']['bottom_z_m']]))
        node(element, 'length_m', weight * perimeter)
    wall_q = quaternion_from_rpy(*wall['origin_rpy'])
    normal = rotate_vector(wall_q, [0, 0, 1])
    surface = [wall['origin_xyz'][i] + normal[i] * simulation['wall']['thickness_m'] / 2
               for i in range(3)]
    node(sp, 'wall_pose', numbers(surface + wall['origin_rpy']))
    node(sp, 'wall_size', numbers([wall['surface']['width_m'], wall['surface']['height_m']]))
    world_file = Path(legacy.render_world(str(gazebo), str(description), '0'))
    try:
        world_root = ET.parse(world_file).getroot()
    finally:
        world_file.unlink(missing_ok=True)
    world = world_root.find('world')
    old_wrench = [p for p in world.findall('plugin')
                  if 'apply-link-wrench' in p.get('filename', '')]
    if len(old_wrench) != 1:
        raise ValueError('expected exactly one legacy wrench to remove')
    world.remove(old_wrench[0])
    includes = [p for p in world.findall('include') if p.findtext('uri') == 'model://climbot']
    if len(includes) != 1:
        raise ValueError('expected one robot include')
    position = [float(v) for v in includes[0].findtext('pose').split()[:3]]
    for i in range(3):
        position[i] += normal[i] * (runtime['spawn_gap_m'] - simulation['spawn']['surface_gap_m'])
    orientation = quaternion_multiply(wall_q, quaternion_from_rpy(0, 0, math.radians(heading_deg)))
    node(new, 'pose', numbers(position + _rpy(orientation)))
    world.remove(includes[0])
    world.append(deepcopy(new))
    world.find('physics/max_step_size').text = str(step_s)
    world.find('physics/real_time_factor').text = str(realtime_factor)
    if not runtime['camera_enabled']:
        for plugin in list(world.findall('plugin')):
            if plugin.get('name') == 'gz::sim::systems::Sensors':
                world.remove(plugin)
    for path, root in [(out / 'model.sdf', sdf), (out / 'robot.urdf', urdf),
                       (out / 'world.sdf', world_root)]:
        ET.indent(root)
        ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)
    resolved = {'mode': mode, 'heading_deg': heading_deg, 'step_s': step_s,
                'runtime': runtime, 'model': model, 'physics': physics, 'io': io,
                'wall_surface_pose': surface + wall['origin_rpy'],
                'nominal_startup_minimum_pa': nominal_minimum,
                'camera_enabled': runtime['camera_enabled']}
    (out / 'resolved.json').write_text(json.dumps(resolved, indent=2, allow_nan=False) + '\n')
    return resolved
