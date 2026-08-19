"""Launch the straight-line tracker with shared robot hard limits."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    """Inject robot geometry and hard limits into the controller."""
    control_share = get_package_share_directory('climbot_control')
    description_share = get_package_share_directory('climbot_description')
    default_config = os.path.join(control_share, 'config', 'control.yaml')
    with open(os.path.join(description_share, 'config', 'robot.yaml')) as handle:
        robot = yaml.safe_load(handle)['robot']
    wheel = robot['drive_wheel']
    drive = robot['drive']

    tracker = Node(
        package='climbot_control',
        executable='line_tracker_node',
        name='line_tracker',
        parameters=[LaunchConfiguration('config_file'), {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'standalone_mode': LaunchConfiguration('standalone_mode'),
            'tracking_mode': LaunchConfiguration('tracking_mode'),
            'wheel_separation': float(wheel['separation_m']),
            'wheel_speed_limit': float(drive['max_linear_velocity_mps']),
            'wheel_acceleration_limit': float(
                drive['max_linear_acceleration_mps2']),
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('standalone_mode', default_value='true'),
        # Deliberately an argument and not a config entry: an A/B run switches
        # it per launch, and a value in control.yaml would be overridden here
        # without saying so.
        DeclareLaunchArgument('tracking_mode', default_value='time'),
        DeclareLaunchArgument('config_file', default_value=default_config),
        tracker,
    ])
