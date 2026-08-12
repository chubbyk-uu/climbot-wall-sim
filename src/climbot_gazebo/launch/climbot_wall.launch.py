"""Launch the wall robot with ROS 2 command and odometry bridges."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Build the wall simulation launch description."""
    package_share = get_package_share_directory('climbot_gazebo')
    ros_gz_share = get_package_share_directory('ros_gz_sim')
    world = os.path.join(package_share, 'worlds', 'climbot_wall.sdf')
    model_path = os.path.join(package_share, 'models')
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')

    gazebo_resources = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=model_path + os.pathsep + existing_resource_path,
    )
    d3d12_driver = SetEnvironmentVariable(
        name='GALLIUM_DRIVER',
        value='d3d12',
    )
    d3d12_adapter = SetEnvironmentVariable(
        name='MESA_D3D12_DEFAULT_ADAPTER_NAME',
        value='NVIDIA',
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v 3 ', world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/climbot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
        parameters=[{
            'qos_overrides./cmd_vel.subscriber.reliability': 'reliable',
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation time.',
        ),
        gazebo_resources,
        d3d12_driver,
        d3d12_adapter,
        gazebo,
        bridge,
    ])
