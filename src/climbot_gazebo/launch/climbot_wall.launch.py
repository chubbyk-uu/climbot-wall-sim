"""Launch the wall robot with ROS 2 command and odometry bridges."""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import xacro
import yaml

_CONTACT_PREFIX = '/world/climbot_wall/model/climbot/link'
CONTACT_TOPIC_LEFT = (
    _CONTACT_PREFIX + '/left_wheel_link/sensor/left_wheel_contact/contact')
CONTACT_TOPIC_RIGHT = (
    _CONTACT_PREFIX + '/right_wheel_link/sensor/right_wheel_contact/contact')
CONTACT_TOPIC_CASTER = (
    _CONTACT_PREFIX + '/caster_ball_link/sensor/caster_contact/contact')


def running_on_wsl():
    """Report whether this is WSL, which needs Mesa's D3D12 driver for the GPU."""
    try:
        with open('/proc/version') as handle:
            return 'microsoft' in handle.read().lower()
    except OSError:
        return False


def render_world(package_share):
    """Render the Gazebo world from wall.yaml and return the generated path."""
    # Both the world and the ROS nodes read the same description, so the wall
    # is never restated as a literal on either side.
    with open(os.path.join(package_share, 'config', 'wall.yaml')) as handle:
        description = yaml.safe_load(handle)
    wall = description['wall']
    surface = wall['surface']
    spawn = wall['spawn']
    centre = surface['centre_xyz']
    roll, pitch, yaw = wall['origin_rpy']
    mappings = {
        'centre_x': repr(float(centre[0])),
        'centre_y': repr(float(centre[1])),
        'centre_z': repr(float(centre[2])),
        'thickness': repr(float(surface['thickness_m'])),
        'width': repr(float(surface['width_m'])),
        'height': repr(float(surface['height_m'])),
        'mu': repr(float(surface['mu'])),
        'grid_spacing': repr(float(surface['grid_spacing_m'])),
        'suction_force': repr(float(description['suction']['force_n'])),
        'spawn_gap': repr(float(spawn['surface_gap_m'])),
        'spawn_lateral': repr(float(spawn['lateral_m'])),
        'spawn_height': repr(float(spawn['height_m'])),
        'spawn_roll': repr(float(roll)),
        'spawn_pitch': repr(float(pitch)),
        'spawn_yaw': repr(float(yaw)),
    }
    source = os.path.join(package_share, 'worlds', 'climbot_wall.sdf.xacro')
    document = xacro.process_file(source, mappings=mappings)
    handle = tempfile.NamedTemporaryFile(
        mode='w', prefix='climbot_wall_', suffix='.sdf', delete=False)
    handle.write(document.toprettyxml(indent='  '))
    handle.close()
    return handle.name


def launch_setup(context, *args, **kwargs):
    """Build the actions that depend on resolved launch configurations."""
    package_share = get_package_share_directory('climbot_gazebo')
    ros_gz_share = get_package_share_directory('ros_gz_sim')
    world = render_world(package_share)
    wall_config = os.path.join(package_share, 'config', 'wall.yaml')
    ekf_config = os.path.join(package_share, 'config', 'ekf_wall.yaml')
    model_path = os.path.join(package_share, 'models')
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')

    actions = [
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=model_path + os.pathsep + existing_resource_path,
        ),
    ]

    backend = LaunchConfiguration('gpu_backend').perform(context)
    if backend == 'auto':
        backend = 'wsl_d3d12' if running_on_wsl() else 'native'
    if backend == 'wsl_d3d12':
        # Route OGRE2 through Mesa's D3D12 driver so it reaches the host GPU.
        actions.append(SetEnvironmentVariable(
            name='GALLIUM_DRIVER', value='d3d12'))
        actions.append(SetEnvironmentVariable(
            name='MESA_D3D12_DEFAULT_ADAPTER_NAME', value='NVIDIA'))
    elif backend != 'native':
        raise ValueError(
            'gpu_backend must be auto, wsl_d3d12, or native, not ' + backend)

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [
                PythonExpression(
                    ["'-s ' if '", LaunchConfiguration('headless'),
                     "' == 'true' else ''"]),
                '-r -v 3 ', world],
            'on_exit_shutdown': 'true',
        }.items(),
    ))

    actions.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/climbot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/model/climbot/ground_truth@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            # Contact sensors ignore their <topic> tag, so the fully qualified
            # Gazebo names are bridged and remapped to short ROS topics below.
            CONTACT_TOPIC_LEFT + '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            CONTACT_TOPIC_RIGHT + '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            CONTACT_TOPIC_CASTER + '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
        ],
        remappings=[
            (CONTACT_TOPIC_LEFT, '/contact/left_wheel'),
            (CONTACT_TOPIC_RIGHT, '/contact/right_wheel'),
            (CONTACT_TOPIC_CASTER, '/contact/caster'),
        ],
        parameters=[{
            'qos_overrides./cmd_vel.subscriber.reliability': 'reliable',
        }],
        output='screen',
    ))

    actions.append(Node(
        package='climbot_gazebo',
        executable='total_station_sim.py',
        name='total_station_sim',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'wall_config': wall_config,
            'publish_rate_hz': LaunchConfiguration('total_station_rate_hz'),
            'position_stddev_m': LaunchConfiguration('total_station_stddev_m'),
            'fixed_delay_s': LaunchConfiguration('total_station_delay_s'),
        }],
        output='screen',
    ))

    actions.append(Node(
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
    ))

    actions.append(Node(
        package='climbot_gazebo',
        executable='wall_imu_adapter.py',
        name='wall_imu_adapter',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'orientation_stddev_rad': LaunchConfiguration(
                'imu_orientation_stddev_rad'),
        }],
        output='screen',
    ))

    actions.append(Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config, {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[('odometry/filtered', '/odometry/filtered')],
        output='screen',
    ))

    return actions


def generate_launch_description():
    """Build the wall simulation launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation time.',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo without its GUI, for automated tests.',
        ),
        DeclareLaunchArgument(
            'gpu_backend',
            default_value='auto',
            description='Rendering backend: auto, wsl_d3d12, or native.',
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
        OpaqueFunction(function=launch_setup),
    ])
