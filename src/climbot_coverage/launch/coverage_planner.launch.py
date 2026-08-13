"""Launch coverage planning and optional RViz visualization."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create the coverage-planning launch description."""
    package_share = get_package_share_directory('climbot_coverage')
    default_config = os.path.join(
        package_share, 'config', 'coverage_rectangle.yaml')
    rviz_config = os.path.join(package_share, 'rviz', 'coverage.rviz')

    planner = Node(
        package='climbot_coverage',
        executable='coverage_planner_node',
        name='coverage_planner',
        parameters=[LaunchConfiguration('config_file'), {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_mode': LaunchConfiguration('input_mode'),
            'region_type': LaunchConfiguration('region_type'),
            'sweep_direction': LaunchConfiguration('sweep_direction'),
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
