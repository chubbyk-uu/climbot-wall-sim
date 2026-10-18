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

"""Launch the straight-line tracker with shared robot hard limits."""

import os

from ament_index_python.packages import get_package_share_directory
from climbot_common.dds import local_dds_environment
from climbot_description.robot_geometry import load_robot
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _tracker_node(context):
    """Resolve the selected robot before constructing the controller."""
    geometry = LaunchConfiguration('robot_geometry').perform(context)
    robot = load_robot(geometry)
    wheel = robot['drive_wheel']
    drive = robot['drive']

    tracker = Node(
        package='climbot_control',
        executable='line_tracker_node',
        name='line_tracker',
        additional_env=local_dds_environment(
            os.path.join(get_package_share_directory('climbot_common'),
                         'config', 'fastdds_control_local.xml'), context.environment),
        parameters=[LaunchConfiguration('config_file'), {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'standalone_mode': LaunchConfiguration('standalone_mode'),
            'tracking_mode': LaunchConfiguration('tracking_mode'),
            'robot_geometry': geometry,
            'wheel_separation': float(wheel['separation_m']),
            'wheel_speed_limit': float(drive['max_linear_velocity_mps']),
            'wheel_acceleration_limit': float(
                drive['max_linear_acceleration_mps2']),
        }],
        output='screen',
    )

    return [tracker]


def generate_launch_description():
    """Inject robot geometry and hard limits into the controller."""
    control_share = get_package_share_directory('climbot_control')
    default_config = os.path.join(control_share, 'config', 'control.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('robot_geometry', default_value='vacuum_chassis'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('standalone_mode', default_value='true'),
        # Deliberately an argument and not a config entry: an A/B run switches
        # it per launch, and a value in control.yaml would be overridden here
        # without saying so.
        DeclareLaunchArgument('tracking_mode', default_value='time'),
        DeclareLaunchArgument('config_file', default_value=default_config),
        OpaqueFunction(function=_tracker_node),
    ])
