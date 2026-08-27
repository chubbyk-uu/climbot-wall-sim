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

"""Launch the multi-segment coverage executor."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Reuse the tracker configuration with standalone mode disabled."""
    tracker_launch = PathJoinSubstitution([
        FindPackageShare('climbot_control'), 'launch', 'line_tracker.launch.py'])
    default_config = PathJoinSubstitution([
        FindPackageShare('climbot_control'), 'config', 'control.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # Deliberately not named config_file. Included launch files inherit the
        # parent scope, and a declared default is skipped for a name the parent
        # already set, so a combined launch that declares config_file for its
        # planner would hand the planner's file to the tracker. The tracker
        # would then start on built-in defaults, including no slip
        # compensation, without reporting anything. A distinct name plus the
        # explicit pass below makes that collision impossible.
        DeclareLaunchArgument('control_config_file', default_value=default_config),
        DeclareLaunchArgument('tracking_mode', default_value='time'),
        DeclareLaunchArgument('inspection_default_enabled', default_value='true'),
        DeclareLaunchArgument(
            'inspection_output_root',
            default_value=EnvironmentVariable('CLIMBOT_DATA_ROOT', default_value='')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tracker_launch),
            launch_arguments={
                'standalone_mode': 'false',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'config_file': LaunchConfiguration('control_config_file'),
                'tracking_mode': LaunchConfiguration('tracking_mode'),
            }.items(),
        ),
        Node(
            package='climbot_control',
            executable='coverage_manager_node',
            name='coverage_manager',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'inspection_default_enabled': LaunchConfiguration('inspection_default_enabled'),
                'inspection_output_root': LaunchConfiguration('inspection_output_root'),
            }],
            output='screen',
        ),
    ])
