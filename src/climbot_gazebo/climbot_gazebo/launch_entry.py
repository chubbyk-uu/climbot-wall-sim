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
from pathlib import Path
import shutil
import tempfile
from xml.etree import ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from climbot_common.dds import local_dds_environment
from climbot_description.robot_geometry import resolve_adhesion
from climbot_gazebo.calibration_target import prepare_emissive_g1_target
from climbot_gazebo.vacuum_assets import _stack_module, add_fan_visuals_gui, build_assets
from launch.actions import DeclareLaunchArgument, LogInfo
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
            build_assets(
                root, mode='fan_ode' if mode == 'vacuum' else 'fixed_pressure',
                runtime_overrides={'camera_enabled': True}, io_overrides=io)
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
            includes = [p for p in parent.findall('include')
                        if p.findtext('uri') == 'model://climbot']
            forces = [p for p in parent.findall('plugin')
                      if 'apply-link-wrench' in p.get('filename', '')]
            if len(includes) != 1 or forces:
                raise ValueError('scene must contain one robot include and no constant wrench')
            parent.remove(includes[0])
            model_tree = ET.parse(root / 'model.sdf')
            candidate = model_tree.getroot().find('model')
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
            actions = original_setup(context, *args, **kwargs)
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
    ]:
        launch.entities.insert(0, action)
    return launch
