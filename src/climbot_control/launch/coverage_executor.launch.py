"""Launch the multi-segment coverage executor."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
            }],
            output='screen',
        ),
    ])
