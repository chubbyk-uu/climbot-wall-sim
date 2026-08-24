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

"""Launch simulator-agnostic manual and position-triggered inspection."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Expose topic/config overrides for simulation and future real cameras."""
    package_share = get_package_share_directory('climbot_inspection')
    default_config = os.path.join(package_share, 'config', 'inspection.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('config_file', default_value=default_config),
        Node(
            package='climbot_inspection',
            executable='capture_once_node',
            name='capture_once_node',
            parameters=[LaunchConfiguration('config_file'), {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            output='screen',
        ),
        DeclareLaunchArgument('automatic_capture', default_value='true'),
        Node(
            package='climbot_inspection',
            executable='automatic_capture_node',
            name='automatic_capture_node',
            parameters=[LaunchConfiguration('config_file'), {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            condition=IfCondition(LaunchConfiguration('automatic_capture')),
            output='screen',
        ),
    ])
