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

"""Launch coverage planning and optional RViz visualization."""

import os

from ament_index_python.packages import get_package_share_directory
from climbot_description.wall_frame import reference_grid_spacing
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    """Create the coverage-planning launch description."""
    package_share = get_package_share_directory('climbot_coverage')
    description_share = get_package_share_directory('climbot_description')
    default_config = os.path.join(
        package_share, 'config', 'coverage_rectangle.yaml')
    rviz_config = os.path.join(package_share, 'rviz', 'coverage.rviz')
    with open(os.path.join(description_share, 'config', 'robot.yaml')) as handle:
        footprint = yaml.safe_load(handle)['robot']['footprint']
    with open(os.path.join(description_share, 'config', 'wall.yaml')) as handle:
        wall_surface = yaml.safe_load(handle)['wall']['surface']

    planner = Node(
        package='climbot_coverage',
        executable='coverage_planner_node',
        name='coverage_planner',
        parameters=[LaunchConfiguration('config_file'), {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_mode': LaunchConfiguration('input_mode'),
            'region_type': LaunchConfiguration('region_type'),
            'sweep_direction': LaunchConfiguration('sweep_direction'),
            'robot_length': float(footprint['length_m']),
            'robot_width': float(footprint['width_m']),
            'edge_clearance': float(footprint['edge_clearance_m']),
            'wall_width': float(wall_surface['width_m']),
            'wall_height': float(wall_surface['height_m']),
            # Not a launch argument. The wall_grid_spacing argument switches
            # the grid painted on the wall face in Gazebo, which is the one
            # that ends up in photographs; this overlay is only ever looked at
            # by a person, and the switch for it is unticking the display in
            # RViz. Tying the two together would take the operator's grid away
            # on exactly the runs where they still want it.
            'wall_grid_spacing': reference_grid_spacing(),
        }],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('input_mode', default_value='parameters'),
        DeclareLaunchArgument('region_type', default_value='rectangle'),
        DeclareLaunchArgument('sweep_direction', default_value='horizontal'),
        planner,
        rviz,
    ])
