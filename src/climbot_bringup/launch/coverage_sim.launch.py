"""Launch the wall simulation, coverage planner, and RViz together."""

import os

from ament_index_python.packages import get_package_share_directory
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
        launch_arguments={'use_sim_time': 'true'}.items(),
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
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('input_mode', default_value='parameters'),
        DeclareLaunchArgument('region_type', default_value='rectangle'),
        DeclareLaunchArgument('sweep_direction', default_value='horizontal'),
        simulation,
        coverage,
    ])
