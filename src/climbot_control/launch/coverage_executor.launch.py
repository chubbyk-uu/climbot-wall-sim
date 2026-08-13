"""Launch the multi-segment coverage executor."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Reuse the tracker configuration with standalone mode disabled."""
    tracker_launch = PathJoinSubstitution([
        FindPackageShare('climbot_control'), 'launch', 'line_tracker.launch.py'])
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tracker_launch),
            launch_arguments={'standalone_mode': 'false'}.items(),
        ),
    ])
