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

"""Run the pressure chassis with the accepted localization, camera and GUI pipeline."""

from copy import deepcopy
import os
from pathlib import Path
import shutil
import tempfile
from xml.etree import ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from climbot_common.dds import local_dds_environment
from climbot_description.robot_geometry import resolve_adhesion
from climbot_description.vacuum_preview import node
from climbot_gazebo.calibration_target import prepare_emissive_g1_target
from climbot_gazebo.vacuum_assets import (
    _stack_module, add_fan_visuals_gui, build_assets, drop_contact_observer)
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    """Adapt only generated robot/world inputs; retain the stack stack itself."""
    gazebo = Path(get_package_share_directory('climbot_gazebo'))
    description = Path(get_package_share_directory('climbot_description'))
    stack = _stack_module(gazebo)
    original_setup = stack.launch_setup
    original_world = stack.render_world

    def setup(context, *args, **kwargs):
        geometry = LaunchConfiguration('robot_geometry').perform(context)
        mode = resolve_adhesion(geometry, LaunchConfiguration('adhesion_mode').perform(context))
        if geometry != 'vacuum_chassis':
            raise ValueError('candidate stack requires vacuum_chassis')
        root = Path(tempfile.mkdtemp(prefix='climbot_vacuum_stack_'))
        try:
            io_path = LaunchConfiguration('io_config').perform(context)
            io = yaml.safe_load(Path(io_path).read_text())['vacuum_io'] if io_path else None
            surface = LaunchConfiguration('wall_surface').perform(context)
            soft = LaunchConfiguration('wheel_compliance').perform(context) == 'true'
            # The lateral tyre spring follows the normal one unless a diagnostic
            # run separates them; it cannot be enabled on its own.
            lateral = LaunchConfiguration('tyre_lateral').perform(context)
            lateral = soft if lateral == 'auto' else lateral == 'true'
            # Crowned tyre contact is part of the compliant wheel; 'false' keeps
            # the former flat-edged cylinder for comparison runs.
            crown = LaunchConfiguration('tyre_crown').perform(context)
            crown = soft if crown == 'auto' else crown == 'true'
            texture = LaunchConfiguration('wall_texture').perform(context) or None
            # 'auto' follows simulation.yaml; observers are off by default.
            contact = LaunchConfiguration('contact_topics').perform(context)
            contact = None if contact == 'auto' else contact == 'true'
            resolved = build_assets(
                root, mode='fan_ode' if mode == 'vacuum' else 'fixed_pressure',
                runtime_overrides={'camera_enabled': True}, io_overrides=io,
                wheel_compliance=soft, tyre_lateral=lateral,
                tyre_crown=crown, contact_topics=contact,
                caster_proxy=surface == 'undulating', wall_surface=surface, wall_texture=texture)
            contact = resolved['contact_observer_topics']
            # Use the same actual scene options as the frozen launch. The generated
            # robot and its single force path are the only replacement here.
            world_path = Path(original_world(
                str(gazebo), str(description),
                LaunchConfiguration('wall_grid_spacing').perform(context),
                LaunchConfiguration('wall_texture').perform(context) or None,
                LaunchConfiguration('inspection_target').perform(context).lower() in
                ('true', '1', 'yes'),
                LaunchConfiguration('inspection_flat_field_target').perform(context).lower() in
                ('true', '1', 'yes')))
            try:
                world = ET.parse(world_path)
            finally:
                world_path.unlink(missing_ok=True)
            # The optional G1 reference is emissive: do not add incident LED
            # reflection to its reference colors. No ordinary wall is changed.
            prepare_emissive_g1_target(world)
            parent = world.getroot().find('world')
            if not contact:
                drop_contact_observer(parent)
            includes = [p for p in parent.findall('include')
                        if p.findtext('uri') == 'model://climbot']
            forces = [p for p in parent.findall('plugin')
                      if 'apply-link-wrench' in p.get('filename', '')]
            if len(includes) != 1 or forces:
                raise ValueError('scene must contain one robot include and no constant wrench')
            parent.remove(includes[0])
            if surface == 'undulating':
                generated = ET.parse(root/'world.sdf').getroot().find('world')
                for item in list(parent.findall('model')):
                    if item.get('name') in ('wall', 'caster_proxy_wall'):
                        parent.remove(item)
                for item in generated.findall('model'):
                    if item.get('name', '').startswith('m2_'):
                        parent.append(deepcopy(item))
            model_tree = ET.parse(root / 'model.sdf')
            candidate = model_tree.getroot().find('model')
            trace = LaunchConfiguration('m2_load_trace').perform(context)
            if trace:
                recorder = node(candidate, 'plugin', filename='libclimbot_load_recorder.so',
                                name='climbot_gazebo::LoadRecorder')
                node(recorder, 'path', str(Path(trace).expanduser().resolve()))
            parent.append(deepcopy(candidate))
            add_fan_visuals_gui(parent)
            ET.indent(world.getroot())
            world.write(root / 'world.sdf', encoding='utf-8', xml_declaration=True)
            snapshot = LaunchConfiguration('resolved_assets_directory').perform(context)
            if snapshot:
                shutil.copytree(root, Path(snapshot))
            stack.render_world = lambda *_a, **_kw: str(root / 'world.sdf')
            stack.render_robot = lambda *_a, **_kw: (str(root), (root / 'robot.urdf').read_text())
            prefix = '/world/climbot_wall/model/climbot/link/'
            stack.CONTACT_TOPIC_LEFT = prefix + 'left_wheel_link/sensor/left_contact/contact'
            stack.CONTACT_TOPIC_RIGHT = prefix + 'right_wheel_link/sensor/right_contact/contact'
            stack.CONTACT_OBSERVER = contact
            actions = original_setup(context, *args, **kwargs)
            if surface == 'undulating':
                from .mesa_runtime import rendering_environment
                from .ode_runtime import environment
                env, _ = rendering_environment(
                    LaunchConfiguration('gpu_backend').perform(context),
                    LaunchConfiguration('mesa').perform(context))
                env, identity = environment(
                    LaunchConfiguration('ode_prefix').perform(context), env)
                index = next(i for i, a in enumerate(actions) if isinstance(a, ExecuteProcess))
                actions[index:index] = [
                    SetEnvironmentVariable('LD_LIBRARY_PATH', env['LD_LIBRARY_PATH']),
                    LogInfo(msg=f'M2 private ODE: {identity}')]
            actions.insert(0, LogInfo(msg=f'Robot geometry={geometry}; adhesion={mode}'))
            routes = [
                '/suction/pressure@sensor_msgs/msg/FluidPressure[gz.msgs.FluidPressure',
                '/suction/ambient_pressure@sensor_msgs/msg/FluidPressure[gz.msgs.FluidPressure',
                '/suction/ground_truth/state@std_msgs/msg/String[gz.msgs.StringMsg',
                '/suction/sensor/fault@std_msgs/msg/Int32]gz.msgs.Int32',
                *[f'/suction/fans/fan_{i}/{field}@std_msgs/msg/Bool]gz.msgs.Boolean'
                  for i in range(3) for field in ['enabled', 'failed']]]
            actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
                                name='vacuum_observation_bridge', arguments=routes,
                                additional_env=local_dds_environment(
                                    Path(get_package_share_directory('climbot_common')) /
                                    'config/fastdds_control_local.xml', context.environment)))
            return actions
        except Exception:
            shutil.rmtree(root, ignore_errors=True)
            raise

    # Use a private module instance so independent launch descriptions cannot interfere.
    stack.launch_setup = setup
    launch = stack.generate_launch_description()
    for action in [
        DeclareLaunchArgument('robot_geometry', default_value='vacuum_chassis'),
        DeclareLaunchArgument('adhesion_mode', default_value='vacuum'),
        DeclareLaunchArgument('io_config', default_value=''),
        DeclareLaunchArgument('resolved_assets_directory', default_value=''),
        # M2 defaults (2026-10-20): the undulating wall with panel seams and the
        # preloaded, crowned compliant tyres. 'flat' and 'false' keep the M1
        # flat wall and rigid wheels for baselines.
        DeclareLaunchArgument('wall_surface', default_value='undulating',
                              choices=['flat', 'undulating']),
        DeclareLaunchArgument('tyre_crown', default_value='auto',
                              choices=['auto', 'false', 'true']),
        DeclareLaunchArgument('tyre_lateral', default_value='auto',
                              choices=['auto', 'false', 'true']),
        DeclareLaunchArgument('wheel_compliance', default_value='true',
                              choices=['false', 'true']),
        DeclareLaunchArgument('m2_load_trace', default_value=''),
        DeclareLaunchArgument('contact_topics', default_value='auto',
                              choices=['auto', 'false', 'true']),
        DeclareLaunchArgument('ode_prefix', default_value=os.environ.get(
            'CLIMBOT_ODE_PREFIX', str(Path.home()/'opt/climbot-ode-0.16.2-hashfix-ccd/install'))),
    ]:
        launch.entities.insert(0, action)
    return launch
