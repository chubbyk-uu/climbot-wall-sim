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

"""Launch the complete click-plan-start-execute chain in one command."""

import os

from ament_index_python.packages import get_package_share_directory
from climbot_description.wall_frame import reference_grid_spacing
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Bring up simulation, planner, RViz, tracker and manager together."""
    gazebo_share = get_package_share_directory('climbot_gazebo')
    coverage_share = get_package_share_directory('climbot_coverage')
    control_share = get_package_share_directory('climbot_control')
    inspection_share = get_package_share_directory('climbot_inspection')

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
    planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                coverage_share, 'launch', 'coverage_planner.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'rviz': LaunchConfiguration('rviz'),
            'config_file': LaunchConfiguration('planner_config_file'),
            'input_mode': LaunchConfiguration('input_mode'),
            'region_type': LaunchConfiguration('region_type'),
            'sweep_direction': LaunchConfiguration('sweep_direction'),
        }.items(),
    )
    # The executor is what coverage_sim.launch.py leaves out: without it the
    # planned task is previewed but nothing can ever run it. Both parameter
    # files are passed explicitly and carry distinct argument names, because
    # one shared config_file would reach whichever include is expanded and
    # leave the other node running on built-in defaults.
    execution = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                control_share, 'launch', 'coverage_executor.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'control_config_file': LaunchConfiguration('control_config_file'),
            'tracking_mode': LaunchConfiguration('tracking_mode'),
        }.items(),
    )
    inspection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                inspection_share, 'launch', 'inspection.launch.py')),
        condition=IfCondition(LaunchConfiguration('inspection')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # This launch runs the RViz click workflow by default, where the shape is
    # a runtime choice, so it needs a config whose task_id does not name one.
    default_planner_config = os.path.join(
        coverage_share, 'config', 'coverage_interactive.yaml')
    default_control_config = os.path.join(
        control_share, 'config', 'control.yaml')
    return LaunchDescription([
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
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Open RViz, which provides the Publish Point tool.',
        ),
        DeclareLaunchArgument(
            'planner_config_file',
            default_value=default_planner_config,
            description='Planner parameter file; region corners are ignored '
                        'in rviz input mode.',
        ),
        DeclareLaunchArgument(
            'control_config_file',
            default_value=default_control_config,
            description='Line-tracker parameter file.',
        ),
        DeclareLaunchArgument(
            'input_mode',
            default_value='rviz',
            description='Region input: rviz clicks or configured parameters.',
        ),
        DeclareLaunchArgument('region_type', default_value='rectangle'),
        DeclareLaunchArgument('sweep_direction', default_value='horizontal'),
        DeclareLaunchArgument(
            'inspection',
            default_value='true',
            description='Start G1 manual and G2 position-triggered inspection.',
        ),
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
        DeclareLaunchArgument(
            'tracking_mode',
            default_value='time',
            description='Straight-line control: distance or time. The panel '
                        'can also switch it while no task is running.',
        ),
        simulation,
        inspection,
        planning,
        execution,
    ])
