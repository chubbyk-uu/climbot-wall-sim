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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the wall simulation launch description."""
    package_share = get_package_share_directory('climbot_gazebo')
    ros_gz_share = get_package_share_directory('ros_gz_sim')
    world = os.path.join(package_share, 'worlds', 'climbot_wall.sdf')
    ekf_config = os.path.join(package_share, 'config', 'ekf_wall.yaml')
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
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/climbot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/model/climbot/ground_truth@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        parameters=[{
            'qos_overrides./cmd_vel.subscriber.reliability': 'reliable',
        }],
        output='screen',
    )

    total_station = Node(
        package='climbot_gazebo',
        executable='total_station_sim.py',
        name='total_station_sim',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_rate_hz': LaunchConfiguration('total_station_rate_hz'),
            'position_stddev_m': LaunchConfiguration('total_station_stddev_m'),
            'fixed_delay_s': LaunchConfiguration('total_station_delay_s'),
        }],
        output='screen',
    )

    wheel_odom_adapter = Node(
        package='climbot_gazebo',
        executable='wall_wheel_odom_adapter.py',
        name='wall_wheel_odom_adapter',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'forward_velocity_stddev_mps': LaunchConfiguration(
                'wheel_forward_velocity_stddev_mps'),
            'yaw_rate_stddev_rps': LaunchConfiguration(
                'wheel_yaw_rate_stddev_rps'),
        }],
        output='screen',
    )

    imu_adapter = Node(
        package='climbot_gazebo',
        executable='wall_imu_adapter.py',
        name='wall_imu_adapter',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'orientation_stddev_rad': LaunchConfiguration(
                'imu_orientation_stddev_rad'),
        }],
        output='screen',
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config, {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[('odometry/filtered', '/odometry/filtered')],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation time.',
        ),
        DeclareLaunchArgument(
            'total_station_rate_hz',
            default_value='12.0',
            description='Simulated total-station observation frequency.',
        ),
        DeclareLaunchArgument(
            'total_station_stddev_m',
            default_value='0.005',
            description='One-sigma total-station position noise in metres.',
        ),
        DeclareLaunchArgument(
            'total_station_delay_s',
            default_value='0.05',
            description='Fixed total-station delivery delay in seconds.',
        ),
        DeclareLaunchArgument(
            'wheel_forward_velocity_stddev_mps',
            default_value='0.03',
            description='Wall-wheel forward velocity one-sigma uncertainty.',
        ),
        DeclareLaunchArgument(
            'wheel_yaw_rate_stddev_rps',
            default_value='0.05',
            description='Wall-wheel yaw-rate one-sigma uncertainty.',
        ),
        DeclareLaunchArgument(
            'imu_orientation_stddev_rad',
            default_value='0.00872664626',
            description='IMU attitude one-sigma uncertainty in radians.',
        ),
        gazebo_resources,
        d3d12_driver,
        d3d12_adapter,
        gazebo,
        bridge,
        total_station,
        wheel_odom_adapter,
        imu_adapter,
        ekf,
    ])
