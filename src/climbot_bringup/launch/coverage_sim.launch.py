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

"""Launch the wall simulation, coverage planner, and RViz together."""

import os

from ament_index_python.packages import get_package_share_directory
from climbot_description.wall_frame import reference_grid_spacing
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Create the combined stage-five launch description."""
    gazebo_share = get_package_share_directory('climbot_gazebo')
    coverage_share = get_package_share_directory('climbot_coverage')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'climbot_wall.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'headless': LaunchConfiguration('headless'),
            'gpu_backend': LaunchConfiguration('gpu_backend'),
            'wall_grid_spacing': LaunchConfiguration('wall_grid_spacing'),
            'wall_texture': LaunchConfiguration('wall_texture'),
        }.items(),
    )
    coverage = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                coverage_share, 'launch', 'coverage_planner.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'rviz': 'true',
            'config_file': LaunchConfiguration('config_file'),
            'input_mode': LaunchConfiguration('input_mode'),
            'region_type': LaunchConfiguration('region_type'),
            'sweep_direction': LaunchConfiguration('sweep_direction'),
        }.items(),
    )

    default_config = os.path.join(
        coverage_share, 'config', 'coverage_rectangle.yaml')
    return LaunchDescription([
        # The wall launch this includes supports both, and coverage_mission
        # declares both. Without them the two combined entry points differ in
        # what they can do for no reason an operator can see.
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo without its GUI; RViz still opens.',
        ),
        DeclareLaunchArgument(
            'gpu_backend',
            default_value='auto',
            description='Rendering backend: auto, wsl_d3d12, or native.',
        ),
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('input_mode', default_value='parameters'),
        DeclareLaunchArgument('region_type', default_value='rectangle'),
        DeclareLaunchArgument('sweep_direction', default_value='horizontal'),
        # The grid painted on the wall face in Gazebo, which is the one that
        # ends up in photographs. Set it to 0 for a run that photographs the
        # wall. It deliberately does not reach the RViz overlay: that one is
        # only ever looked at by a person, and its switch is unticking the
        # Wall Reference Grid display, which needs no restart.
        DeclareLaunchArgument(
            'wall_grid_spacing',
            default_value=repr(reference_grid_spacing()),
            description='Reference grid pitch on the Gazebo wall face in '
                        'metres; 0 paints none. Does not affect RViz.',
        ),
        DeclareLaunchArgument(
            'wall_texture',
            default_value='',
            description='Bake manifest from tools/bake_wall_texture.py; empty '
                        'leaves the wall its flat colour.',
        ),
        simulation,
        coverage,
    ])
