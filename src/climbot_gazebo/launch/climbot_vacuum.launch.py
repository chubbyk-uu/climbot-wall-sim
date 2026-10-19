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

"""S3 physics entry: candidate geometry, B/C pressure source, ROS actuator bridge."""

import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from climbot_gazebo.mesa_runtime import KEYS, rendering_environment, SNAPSHOT
from climbot_gazebo.vacuum_assets import build_assets
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler,
    SetEnvironmentVariable, Shutdown, UnsetEnvironmentVariable,
)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _setup(context):
    """Keep this experimental entry separate from the frozen legacy pipeline."""
    env, identity = rendering_environment(
        LaunchConfiguration('gpu_backend').perform(context),
        LaunchConfiguration('mesa').perform(context))
    mesa_actions = []
    for key in (*KEYS, SNAPSHOT, 'GALLIUM_DRIVER', 'MESA_D3D12_DEFAULT_ADAPTER_NAME'):
        if env.get(key) != os.environ.get(key):
            mesa_actions.append(SetEnvironmentVariable(key, env[key]) if key in env
                                else UnsetEnvironmentVariable(key))
    directory = tempfile.TemporaryDirectory(prefix='climbot_vacuum_')
    root = Path(directory.name)
    io_config = LaunchConfiguration('io_config').perform(context)
    io = yaml.safe_load(Path(io_config).read_text())['vacuum_io'] if io_config else None
    build_assets(root, mode=LaunchConfiguration('pressure_mode').perform(context),
                 heading_deg=float(LaunchConfiguration('heading_deg').perform(context)),
                 io_overrides=io,
                 wheel_compliance=(
                     LaunchConfiguration('wheel_compliance').perform(context) == 'true'))
    prefix = Path(get_package_prefix('climbot_gazebo'))
    share = Path(get_package_share_directory('climbot_gazebo'))
    control = Path(get_package_share_directory('climbot_control'))
    supervisor = str(prefix / 'lib/climbot_gazebo/gz_sim_supervisor.py')
    return mesa_actions + [
        SetEnvironmentVariable('GZ_GUI_PLUGIN_PATH', os.pathsep.join(filter(None, [
            str(prefix / 'lib'), os.environ.get('GZ_GUI_PLUGIN_PATH')]))),
        LogInfo(msg=f'Mesa runtime: {identity}'),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', os.pathsep.join(filter(None, [
            str(prefix / 'lib'), os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH')]))),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(filter(None, [
            str(share / 'models'), str(share), os.environ.get('GZ_SIM_RESOURCE_PATH')]))),
        RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=lambda _: directory.cleanup())])),
        ExecuteProcess(cmd=[supervisor, 'server', '--world', str(root / 'world.sdf')],
                       name='gazebo_server', output='screen', on_exit=Shutdown()),
        ExecuteProcess(cmd=[supervisor, 'gui', '--wait-for-render-ready'],
                       name='gazebo_gui', output='screen',
                       condition=UnlessCondition(LaunchConfiguration('headless'))),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'use_sim_time': True,
                          'robot_description': (root / 'robot.urdf').read_text()}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='vacuum_physics_bridge', arguments=[
                 '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                 '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                 '/model/climbot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
                 '/model/climbot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                 '/suction/pressure@sensor_msgs/msg/FluidPressure[gz.msgs.FluidPressure',
                 '/suction/ambient_pressure@sensor_msgs/msg/FluidPressure[gz.msgs.FluidPressure',
                 '/suction/ground_truth/state@std_msgs/msg/String[gz.msgs.StringMsg',
                 '/suction/sensor/fault@std_msgs/msg/Int32]gz.msgs.Int32',
                 *[f'/suction/fans/fan_{i}/{field}@std_msgs/msg/Bool]gz.msgs.Boolean'
                   for i in range(3) for field in ['enabled', 'failed']]],
             remappings=[('/model/climbot/joint_state', '/joint_states')],
             parameters=[{'qos_overrides./cmd_vel.subscriber.reliability': 'reliable'}]),
        Node(package='climbot_control', executable='cmd_vel_watchdog_node',
             name='cmd_vel_watchdog',
             parameters=[str(control / 'config/control.yaml'), {'use_sim_time': True}],
             remappings=[('cmd_vel_in', '/control/cmd_vel'), ('cmd_vel_out', '/cmd_vel')]),
    ]


def generate_launch_description():
    """Launch flat-wall rigid-wheel physics, measured pressure and fault inputs."""
    return LaunchDescription([
        DeclareLaunchArgument('pressure_mode', default_value='fan_ode',
                              choices=['fan_ode', 'fixed_pressure']),
        DeclareLaunchArgument('heading_deg', default_value='0'),
        DeclareLaunchArgument('wheel_compliance', default_value='false',
                              choices=['false', 'true']),
        DeclareLaunchArgument('io_config', default_value=''),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('gpu_backend', default_value='auto',
                              choices=['auto', 'wsl_d3d12', 'native', 'software']),
        DeclareLaunchArgument('mesa', default_value=os.environ.get('CLIMBOT_MESA', 'private'),
                              choices=['private', 'system']),
        OpaqueFunction(function=_setup),
    ])
