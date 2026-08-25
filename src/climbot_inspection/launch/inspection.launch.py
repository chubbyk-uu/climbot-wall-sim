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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    """Expose topic/config overrides for simulation and future real cameras."""
    package_share = get_package_share_directory('climbot_inspection')
    description_share = get_package_share_directory('climbot_description')
    default_config = os.path.join(package_share, 'config', 'inspection.yaml')
    with open(os.path.join(
            description_share, 'config', 'inspection_camera.yaml')) as handle:
        camera = yaml.safe_load(handle)['inspection_camera']
    mount = camera['optical_mount']
    geometry = {
        'effective_length_m': camera['footprint']['effective_length_m'],
        'image_overlap_ratio': camera['capture_policy']['nominal_overlap_ratio'],
        'camera_mount_x_m': mount['center_xyz_m'][0],
        'camera_mount_y_m': mount['center_xyz_m'][1],
        'camera_mount_z_m': mount['center_xyz_m'][2],
        'camera_mount_roll_rad': mount['rpy_rad'][0],
        'camera_mount_pitch_rad': mount['rpy_rad'][1],
        'camera_mount_yaw_rad': mount['rpy_rad'][2],
    }
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('flat_field_file', default_value=''),
        DeclareLaunchArgument('archive_recorder', default_value='true'),
        DeclareLaunchArgument('inspection_output_root', default_value='~/climbot_data'),
        Node(
            package='climbot_inspection',
            executable='capture_once_node',
            name='capture_once_node',
            parameters=[LaunchConfiguration('config_file'), {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            output='screen',
        ),
        Node(
            package='climbot_inspection',
            executable='flat_field_node',
            name='flat_field_node',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'calibration_file': LaunchConfiguration('flat_field_file'),
            }],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('flat_field_file'), "' != ''"])),
            output='screen',
        ),
        DeclareLaunchArgument('automatic_capture', default_value='true'),
        Node(
            package='climbot_inspection',
            executable='automatic_capture_node',
            name='automatic_capture_node',
            parameters=[LaunchConfiguration('config_file'), geometry, {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            condition=IfCondition(LaunchConfiguration('automatic_capture')),
            output='screen',
        ),
        Node(
            package='climbot_inspection',
            executable='archive_recorder_node',
            name='archive_recorder_node',
            parameters=[LaunchConfiguration('config_file'), geometry, {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'output_root': LaunchConfiguration('inspection_output_root'),
                'flat_field_file': LaunchConfiguration('flat_field_file'),
            }],
            condition=IfCondition(LaunchConfiguration('archive_recorder')),
            output='screen',
        ),
    ])
