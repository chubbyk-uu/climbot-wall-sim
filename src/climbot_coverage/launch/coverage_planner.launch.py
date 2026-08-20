"""Launch coverage planning and optional RViz visualization."""

import os

from ament_index_python.packages import get_package_share_directory
from climbot_description.wall_frame import reference_grid_spacing
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
            # Typed, because the node declares a double and a launch
            # configuration is a string: without this the node refuses
            # the parameter and never starts.
            'wall_grid_spacing': ParameterValue(
                LaunchConfiguration('wall_grid_spacing'), value_type=float),
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
        # The same word the wall launch takes, so one argument switches the
        # grid off in both views. RViz can also untick the display live, which
        # is the switch an operator reaches for; this one is for a run that
        # should never draw it, such as one photographing the wall.
        DeclareLaunchArgument(
            'wall_grid_spacing',
            default_value=repr(reference_grid_spacing()),
            description='Reference grid pitch in metres for the RViz overlay; '
                        '0 draws none.',
        ),
        planner,
        rviz,
    ])
