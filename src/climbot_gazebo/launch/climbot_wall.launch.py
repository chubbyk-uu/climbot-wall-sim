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

"""Launch the wall robot with ROS 2 command and odometry bridges."""

import math
import os
import shutil
import tempfile
from xml.dom import minidom

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from climbot_description.wall_frame import reference_grid_spacing
from climbot_gazebo.total_station_model import resolve_component_enabled
from climbot_gazebo.wall_texture import load_manifest, texture_visuals
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
    TimerAction,
    UnsetEnvironmentVariable,
)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
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
JOINT_STATE_TOPIC = '/model/climbot/joint_state'
CAMERA_IDEAL_IMAGE_TOPIC = '/simulation/inspection_camera/ideal_image'
CAMERA_IDEAL_INFO_TOPIC = '/simulation/inspection_camera/ideal_camera_info'
CAMERA_TRIGGER_TOPIC = '/simulation/inspection_camera/trigger'


def resolved_localization_measurement_model(context):
    """Resolve a named profile plus explicit component overrides once."""
    profile = LaunchConfiguration('localization_profile').perform(context)
    prism_mode = LaunchConfiguration(
        'prism_extrinsic_error_mode').perform(context)
    timestamp_mode = LaunchConfiguration(
        'measurement_timestamp_error_mode').perform(context)
    residual = yaml.safe_load(LaunchConfiguration(
        'prism_extrinsic_error_robot_m').perform(context))
    if not isinstance(residual, (list, tuple)) or len(residual) != 3:
        raise ValueError('prism_extrinsic_error_robot_m must be a three-value YAML list.')
    try:
        residual = [float(value) for value in residual]
    except (TypeError, ValueError) as error:
        raise ValueError(
            'prism_extrinsic_error_robot_m must contain finite numbers.') from error
    if not all(math.isfinite(value) for value in residual):
        raise ValueError('prism_extrinsic_error_robot_m must contain finite numbers.')
    return {
        'localization_profile': profile,
        'prism_extrinsic_error_enabled': resolve_component_enabled(
            profile, prism_mode),
        'prism_extrinsic_error_robot_m': residual,
        'measurement_timestamp_error_enabled': resolve_component_enabled(
            profile, timestamp_mode),
        'measurement_timestamp_bias_s': float(LaunchConfiguration(
            'measurement_timestamp_bias_s').perform(context)),
        'measurement_timestamp_jitter_stddev_s': float(LaunchConfiguration(
            'measurement_timestamp_jitter_stddev_s').perform(context)),
        'measurement_timestamp_jitter_seed': int(LaunchConfiguration(
            'measurement_timestamp_jitter_seed').perform(context)),
    }


def running_on_wsl():
    """Report whether this is WSL, which needs Mesa's D3D12 driver for the GPU."""
    try:
        with open('/proc/version') as handle:
            return 'microsoft' in handle.read().lower()
    except OSError:
        return False


def apply_wall_texture(document, manifest_path, thickness, wall_origin, link_centre,
                       wall_size):
    """Add the baked texture blocks to the rendered wall, if one is configured."""
    if not manifest_path:
        return
    path = os.path.abspath(os.path.expanduser(manifest_path))
    if not os.path.exists(path):
        # Refusing here rather than falling back to the flat wall. A run that
        # silently photographs a blank surface produces images that look like a
        # camera fault, and the cost of finding that out is a whole run.
        raise FileNotFoundError(
            'the wall texture manifest %s does not exist; run '
            'tools/fetch_wall_texture.sh and tools/bake_wall_texture.py, or '
            'clear texture_manifest' % path)
    manifest, directory = load_manifest(path, wall_size=wall_size)
    links = [node for node in document.getElementsByTagName('link')
             if node.getAttribute('name') == 'wall_link']
    if not links:
        raise RuntimeError('the rendered world has no wall_link to texture')
    for element in texture_visuals(
            manifest, directory, thickness, wall_origin, link_centre):
        fragment = minidom.parseString(element.strip()).documentElement
        links[0].appendChild(document.importNode(fragment, True))


def render_world(gazebo_share, description_share, grid_spacing,
                 texture_manifest=None, inspection_target=False,
                 inspection_flat_field_target=False):
    """Render the Gazebo world from shared and simulation-only settings."""
    with open(os.path.join(description_share, 'config', 'wall.yaml')) as handle:
        wall = yaml.safe_load(handle)['wall']
    with open(os.path.join(gazebo_share, 'config', 'simulation.yaml')) as handle:
        simulation = yaml.safe_load(handle)['simulation']
    with open(os.path.join(
            description_share, 'config', 'inspection_camera.yaml')) as handle:
        camera = yaml.safe_load(handle)['inspection_camera']
    surface = wall['surface']
    simulated_wall = simulation['wall']
    spawn = simulation['spawn']
    centre = simulated_wall['centre_xyz']
    roll, pitch, yaw = wall['origin_rpy']
    mappings = {
        'centre_x': repr(float(centre[0])),
        'centre_y': repr(float(centre[1])),
        'centre_z': repr(float(centre[2])),
        'thickness': repr(float(simulated_wall['thickness_m'])),
        'width': repr(float(surface['width_m'])),
        'height': repr(float(surface['height_m'])),
        'mu': repr(float(simulated_wall['mu'])),
        'grid_spacing': repr(float(grid_spacing)),
        'suction_force': repr(float(simulation['suction']['force_n'])),
        'spawn_gap': repr(float(spawn['surface_gap_m'])),
        'spawn_lateral': repr(float(spawn['lateral_m'])),
        'spawn_height': repr(float(spawn['height_m'])),
        'spawn_roll': repr(float(roll)),
        'spawn_pitch': repr(float(pitch)),
        'spawn_yaw': repr(float(yaw)),
        'inspection_target': str(bool(inspection_target)).lower(),
        'inspection_flat_field_target': str(bool(
            inspection_flat_field_target)).lower(),
        'target_lateral': repr(
            float(spawn['lateral_m']) +
            float(camera['optical_mount']['center_xyz_m'][0])),
        'target_height': repr(float(spawn['height_m'])),
        'target_length': repr(float(
            camera['footprint']['effective_length_m'])),
        'target_width': repr(float(
            camera['footprint']['effective_width_m'])),
        'ambient_r': repr(float(simulation['lighting']['ambient_rgb'][0])),
        'ambient_g': repr(float(simulation['lighting']['ambient_rgb'][1])),
        'ambient_b': repr(float(simulation['lighting']['ambient_rgb'][2])),
        'moonlight_intensity': repr(float(
            simulation['lighting']['moonlight']['intensity'])),
        'moonlight_direction_x': repr(float(
            simulation['lighting']['moonlight']['direction_xyz'][0])),
        'moonlight_direction_y': repr(float(
            simulation['lighting']['moonlight']['direction_xyz'][1])),
        'moonlight_direction_z': repr(float(
            simulation['lighting']['moonlight']['direction_xyz'][2])),
        'moonlight_diffuse_r': repr(float(
            simulation['lighting']['moonlight']['diffuse_rgb'][0])),
        'moonlight_diffuse_g': repr(float(
            simulation['lighting']['moonlight']['diffuse_rgb'][1])),
        'moonlight_diffuse_b': repr(float(
            simulation['lighting']['moonlight']['diffuse_rgb'][2])),
        'moonlight_cast_shadows': str(bool(
            simulation['lighting']['moonlight']['cast_shadows'])).lower(),
    }
    source = os.path.join(gazebo_share, 'worlds', 'climbot_wall.sdf.xacro')
    document = xacro.process_file(source, mappings=mappings)
    # A launch argument beats the configured default, so the wall can be looked
    # at with the texture on without editing a file every other run reads.
    configured = simulated_wall.get('texture_manifest', '')
    apply_wall_texture(
        document, configured if texture_manifest is None else texture_manifest,
        float(simulated_wall['thickness_m']),
        [float(value) for value in wall['origin_xyz']],
        [float(value) for value in centre],
        (float(surface['width_m']), float(surface['height_m'])))
    handle = tempfile.NamedTemporaryFile(
        mode='w', prefix='climbot_wall_', suffix='.sdf', delete=False)
    handle.write(document.toprettyxml(indent='  '))
    handle.close()
    return handle.name


def robot_mappings(gazebo_share, description_share):
    """Flatten shared robot and Gazebo-only settings for the xacro files."""
    with open(os.path.join(description_share, 'config', 'robot.yaml')) as handle:
        robot = yaml.safe_load(handle)['robot']
    with open(os.path.join(
            description_share, 'config', 'inspection_camera.yaml')) as handle:
        camera = yaml.safe_load(handle)['inspection_camera']
    with open(os.path.join(gazebo_share, 'config', 'simulation.yaml')) as handle:
        simulation = yaml.safe_load(handle)['simulation']
    base = robot['base']
    wheel = robot['drive_wheel']
    caster = robot['caster']
    payload = robot['inspection_payload']
    camera_body = payload['camera_body']
    bracket = payload['bracket']
    optical_mount = camera['optical_mount']
    drive = robot['drive']
    simulated_wheel = simulation['drive_wheel']
    simulated_drive = simulation['drive']
    imu = simulation['imu']
    simulated_camera = simulation['inspection_camera']
    inspection_led = simulation['lighting']['inspection_led']
    image = camera['image']
    calibration = camera['calibration']
    intrinsics = calibration['intrinsics']
    render_scale = float(simulated_camera['render_overscan_focal_scale'])
    render_fx = float(intrinsics['fx_px']) * render_scale
    return {
        'base_length': repr(float(base['size_xyz'][0])),
        'base_width': repr(float(base['size_xyz'][1])),
        'base_thickness': repr(float(base['size_xyz'][2])),
        'base_mass': repr(float(base['mass_kg'])),
        'base_x': repr(float(base['centre_xyz'][0])),
        'base_y': repr(float(base['centre_xyz'][1])),
        'base_z': repr(float(base['centre_xyz'][2])),
        'wheel_radius': repr(float(wheel['radius_m'])),
        'wheel_width': repr(float(wheel['width_m'])),
        'wheel_mass': repr(float(wheel['mass_kg'])),
        'wheel_axle_x': repr(float(wheel['axle_x_m'])),
        'wheel_separation': repr(float(wheel['separation_m'])),
        'wheel_mu': repr(float(simulated_wheel['mu'])),
        'slip_lateral': repr(float(simulated_wheel['slip_lateral'])),
        'slip_longitudinal': repr(float(simulated_wheel['slip_longitudinal'])),
        'nominal_normal_force': repr(
            float(simulated_wheel['nominal_normal_force_n'])),
        'caster_radius': repr(float(caster['radius_m'])),
        'caster_mass': repr(float(caster['mass_kg'])),
        'caster_x': repr(float(caster['centre_x_m'])),
        'caster_mu': repr(float(simulation['caster']['mu'])),
        'camera_body_size_x': repr(float(camera_body['size_xyz_m'][0])),
        'camera_body_size_y': repr(float(camera_body['size_xyz_m'][1])),
        'camera_body_size_z': repr(float(camera_body['size_xyz_m'][2])),
        'camera_body_mass': repr(float(camera_body['mass_kg'])),
        'camera_body_x': repr(float(camera_body['centre_xyz_m'][0])),
        'camera_body_y': repr(float(camera_body['centre_xyz_m'][1])),
        'camera_body_z': repr(float(camera_body['centre_xyz_m'][2])),
        'camera_optical_x': repr(float(optical_mount['center_xyz_m'][0])),
        'camera_optical_y': repr(float(optical_mount['center_xyz_m'][1])),
        'camera_optical_z': repr(float(optical_mount['center_xyz_m'][2])),
        'camera_optical_roll': repr(float(optical_mount['rpy_rad'][0])),
        'camera_optical_pitch': repr(float(optical_mount['rpy_rad'][1])),
        'camera_optical_yaw': repr(float(optical_mount['rpy_rad'][2])),
        'camera_bracket_mass': repr(float(bracket['mass_kg'])),
        'camera_bracket_x': repr(float(bracket['centre_xyz_m'][0])),
        'camera_bracket_y': repr(float(bracket['centre_xyz_m'][1])),
        'camera_bracket_z': repr(float(bracket['centre_xyz_m'][2])),
        'camera_bracket_inertia_x': repr(float(
            bracket['inertia_box_size_xyz_m'][0])),
        'camera_bracket_inertia_y': repr(float(
            bracket['inertia_box_size_xyz_m'][1])),
        'camera_bracket_inertia_z': repr(float(
            bracket['inertia_box_size_xyz_m'][2])),
        'camera_bracket_upright_size_x': repr(float(
            bracket['upright_size_xyz_m'][0])),
        'camera_bracket_upright_size_y': repr(float(
            bracket['upright_size_xyz_m'][1])),
        'camera_bracket_upright_size_z': repr(float(
            bracket['upright_size_xyz_m'][2])),
        'camera_bracket_upright_x': repr(float(
            bracket['upright_centre_xyz_m'][0])),
        'camera_bracket_upright_y': repr(float(
            bracket['upright_centre_xyz_m'][1])),
        'camera_bracket_upright_z': repr(float(
            bracket['upright_centre_xyz_m'][2])),
        'camera_bracket_arm_size_x': repr(float(
            bracket['top_arm_size_xyz_m'][0])),
        'camera_bracket_arm_size_y': repr(float(
            bracket['top_arm_size_xyz_m'][1])),
        'camera_bracket_arm_size_z': repr(float(
            bracket['top_arm_size_xyz_m'][2])),
        'camera_bracket_arm_x': repr(float(
            bracket['top_arm_centre_xyz_m'][0])),
        'camera_bracket_arm_y': repr(float(
            bracket['top_arm_centre_xyz_m'][1])),
        'camera_bracket_arm_z': repr(float(
            bracket['top_arm_centre_xyz_m'][2])),
        'camera_image_width': str(int(image['width_px'])),
        'camera_image_height': str(int(image['height_px'])),
        'camera_render_fx': repr(render_fx),
        'camera_render_fy': repr(
            float(intrinsics['fy_px']) * render_scale),
        'camera_render_hfov': repr(2.0 * math.atan(
            float(image['width_px']) / (2.0 * render_fx))),
        'camera_cx': repr(float(intrinsics['cx_px'])),
        'camera_cy': repr(float(intrinsics['cy_px'])),
        'camera_skew': repr(float(intrinsics['skew'])),
        'camera_triggered': str(bool(
            simulated_camera['triggered'])).lower(),
        'camera_update_rate': repr(float(
            simulated_camera['update_rate_hz'])),
        'camera_image_format': str(simulated_camera['image_format']),
        'camera_near_clip': repr(float(simulated_camera['near_clip_m'])),
        'camera_far_clip': repr(float(simulated_camera['far_clip_m'])),
        'camera_noise_mean': repr(float(simulated_camera['noise_mean'])),
        'camera_noise_stddev': repr(float(
            simulated_camera['noise_stddev'])),
        'camera_visualize': str(bool(
            simulated_camera['visualize'])).lower(),
        'inspection_led_x': repr(float(inspection_led['centre_xyz_m'][0])),
        'inspection_led_y': repr(float(inspection_led['centre_xyz_m'][1])),
        'inspection_led_z': repr(float(inspection_led['centre_xyz_m'][2])),
        'inspection_led_direction_x': repr(float(
            inspection_led['direction_xyz'][0])),
        'inspection_led_direction_y': repr(float(
            inspection_led['direction_xyz'][1])),
        'inspection_led_direction_z': repr(float(
            inspection_led['direction_xyz'][2])),
        'inspection_led_intensity': repr(float(inspection_led['intensity'])),
        'inspection_led_range': repr(float(inspection_led['range_m'])),
        'inspection_led_constant': repr(float(
            inspection_led['attenuation_constant'])),
        'inspection_led_linear': repr(float(
            inspection_led['attenuation_linear'])),
        'inspection_led_quadratic': repr(float(
            inspection_led['attenuation_quadratic'])),
        'inspection_led_inner_angle': repr(float(
            inspection_led['inner_angle_rad'])),
        'inspection_led_outer_angle': repr(float(
            inspection_led['outer_angle_rad'])),
        'inspection_led_falloff': repr(float(inspection_led['falloff'])),
        'inspection_led_cast_shadows': str(bool(
            inspection_led['cast_shadows'])).lower(),
        'max_linear_velocity': repr(float(drive['max_linear_velocity_mps'])),
        'max_angular_velocity': repr(float(drive['max_angular_velocity_rps'])),
        'max_linear_acceleration': repr(
            float(drive['max_linear_acceleration_mps2'])),
        'max_angular_acceleration': repr(
            float(drive['max_angular_acceleration_rps2'])),
        'joint_effort': repr(float(drive['joint_effort_nm'])),
        'joint_velocity': repr(float(drive['joint_velocity_rps'])),
        'joint_damping': repr(float(simulated_drive['joint_damping'])),
        'odom_frequency': repr(
            float(simulated_drive['odom_publish_frequency_hz'])),
        'imu_rate': repr(float(imu['update_rate_hz'])),
        'imu_gyro_stddev': repr(float(imu['angular_velocity_stddev'])),
        'imu_accel_stddev': repr(float(imu['linear_acceleration_stddev'])),
        'contact_rate': repr(float(simulation['contact']['update_rate_hz'])),
    }


def render_robot(gazebo_share, description_share):
    """Render the Gazebo model and the URDF from one robot description."""
    mappings = robot_mappings(gazebo_share, description_share)
    source_dir = os.path.join(gazebo_share, 'models', 'climbot')
    # Gazebo resolves model://climbot by directory name, so the rendered model
    # is written into a matching directory alongside its unchanged manifest.
    model_root = tempfile.mkdtemp(prefix='climbot_model_')
    model_dir = os.path.join(model_root, 'climbot')
    os.makedirs(model_dir)
    shutil.copy(os.path.join(source_dir, 'model.config'), model_dir)
    model = xacro.process_file(
        os.path.join(source_dir, 'model.sdf.xacro'), mappings=mappings)
    with open(os.path.join(model_dir, 'model.sdf'), 'w') as handle:
        handle.write(model.toprettyxml(indent='  '))
    urdf = xacro.process_file(os.path.join(
        description_share, 'urdf', 'climbot.urdf.xacro'), mappings=mappings)
    return model_root, urdf.toxml()


def cleanup_rendered_assets(context, *, world_path, model_root):
    """Remove launch-local rendered SDF and model files during shutdown."""
    del context
    try:
        os.unlink(world_path)
    except FileNotFoundError:
        pass
    except OSError:
        pass
    shutil.rmtree(model_root, ignore_errors=True)
    return []


def launch_setup(context, *args, **kwargs):
    """Build the actions that depend on resolved launch configurations."""
    package_share = get_package_share_directory('climbot_gazebo')
    control_share = get_package_share_directory('climbot_control')
    description_share = get_package_share_directory('climbot_description')
    world = render_world(
        package_share, description_share,
        LaunchConfiguration('wall_grid_spacing').perform(context),
        LaunchConfiguration('wall_texture').perform(context) or None,
        LaunchConfiguration('inspection_target').perform(context).lower() in (
            'true', '1', 'yes'),
        LaunchConfiguration('inspection_flat_field_target').perform(
            context).lower() in ('true', '1', 'yes'))
    model_path, robot_description = render_robot(
        package_share, description_share)
    wall_config = os.path.join(description_share, 'config', 'wall.yaml')
    control_config = os.path.join(control_share, 'config', 'control.yaml')
    ekf_config = os.path.join(package_share, 'config', 'ekf_wall.yaml')
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    existing_system_plugin_path = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    library_path = os.environ.get('LD_LIBRARY_PATH', '')
    gazebo_supervisor = os.path.join(
        get_package_prefix('climbot_gazebo'), 'lib', 'climbot_gazebo',
        'gz_sim_supervisor.py')

    with open(wall_config) as handle:
        wall = yaml.safe_load(handle)['wall']
    with open(os.path.join(package_share, 'config', 'simulation.yaml')) as handle:
        simulation = yaml.safe_load(handle)['simulation']
    localization_model = resolved_localization_measurement_model(context)

    actions = [
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=model_path + os.pathsep + existing_resource_path,
        ),
        SetEnvironmentVariable(
            # Match ros_gz_sim's standard launcher environment. The
            # supervisor must invoke the actual Gazebo process directly so it
            # can terminate its whole process group without leaving Ruby's
            # gz child behind.
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value=os.pathsep.join((existing_system_plugin_path, library_path)),
        ),
        RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(
                function=cleanup_rendered_assets,
                kwargs={'world_path': world, 'model_root': model_path}),
        ])),
    ]

    clock_publish_hz = float(LaunchConfiguration('clock_publish_hz').perform(context))
    if not math.isfinite(clock_publish_hz) or clock_publish_hz < 0.0:
        raise ValueError('clock_publish_hz must be zero or positive and finite')
    throttle_clock = clock_publish_hz > 0.0

    backend = LaunchConfiguration('gpu_backend').perform(context)
    if backend == 'auto':
        backend = 'wsl_d3d12' if running_on_wsl() else 'native'
    if backend == 'wsl_d3d12':
        # Route OGRE2 through Mesa's D3D12 driver so it reaches the host GPU.
        actions.append(SetEnvironmentVariable(
            name='GALLIUM_DRIVER', value='d3d12'))
        actions.append(SetEnvironmentVariable(
            name='MESA_D3D12_DEFAULT_ADAPTER_NAME', value='NVIDIA'))
    elif backend == 'software':
        # This launch-level environment reaches the server and, in the
        # combined bringup launch, the Gazebo GUI and RViz as well.
        actions.append(SetEnvironmentVariable(
            name='GALLIUM_DRIVER', value='llvmpipe'))
        actions.append(UnsetEnvironmentVariable(
            name='MESA_D3D12_DEFAULT_ADAPTER_NAME'))
    elif backend != 'native':
        raise ValueError(
            'gpu_backend must be auto, software, wsl_d3d12, or native, not ' + backend)

    gui_backend = LaunchConfiguration('gui_gpu_backend').perform(context)
    if gui_backend == 'auto':
        gui_backend = 'shared'
    if gui_backend not in ('shared', 'software'):
        raise ValueError(
            'gui_gpu_backend must be auto, shared, or software, not ' + gui_backend)
    gui_command = [gazebo_supervisor, 'gui']
    if gui_backend == 'software':
        gui_command.append('--software-rendering')

    # The simulation server owns physics, sensors and /clock, so it remains a
    # required process. Keep both Gazebo clients under a small supervisor:
    # GZ 8 can segfault when a terminal sends SIGINT into Ogre, while sending
    # TERM to the clients' isolated process groups exits cleanly and prevents
    # Ruby's `gz` launcher from leaving its actual child behind.
    actions.append(ExecuteProcess(
        cmd=[gazebo_supervisor, 'server', '--world', world],
        name='gazebo_server', output='screen', on_exit=Shutdown()))
    actions.append(TimerAction(
        period=2.0,
        actions=[ExecuteProcess(
            cmd=gui_command, name='gazebo_gui', output='screen',
            condition=UnlessCondition(LaunchConfiguration('headless')),
        )],
    ))

    bridge_remappings = [
        (CONTACT_TOPIC_LEFT, '/contact/left_wheel'),
        (CONTACT_TOPIC_RIGHT, '/contact/right_wheel'),
        (CONTACT_TOPIC_CASTER, '/contact/caster'),
        (JOINT_STATE_TOPIC, '/joint_states'),
    ]
    if throttle_clock:
        bridge_remappings.append(('/clock', '/clock_raw'))
    actions.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='simulation_data_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/climbot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/climbot/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            CAMERA_IDEAL_IMAGE_TOPIC +
            '@sensor_msgs/msg/Image[gz.msgs.Image',
            CAMERA_IDEAL_INFO_TOPIC +
            '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            JOINT_STATE_TOPIC + '@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Contact sensors ignore their <topic> tag, so the fully qualified
            # Gazebo names are bridged and remapped to short ROS topics below.
            CONTACT_TOPIC_LEFT + '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            CONTACT_TOPIC_RIGHT + '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            CONTACT_TOPIC_CASTER + '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
        ],
        remappings=bridge_remappings,
        parameters=[{
            'qos_overrides./cmd_vel.subscriber.reliability': 'reliable',
        }],
        output='screen',
    ))
    if throttle_clock:
        # Keep the 1 ms Gazebo physics step and its raw clock untouched. One
        # small C++ subscriber absorbs that stream; every other ROS node sees
        # only the selected rate, so the experiment changes no plant physics.
        actions.append(Node(
            package='climbot_gazebo',
            executable='clock_throttle_node',
            name='clock_throttle',
            parameters=[{
                'use_sim_time': False,
                'publish_rate_hz': clock_publish_hz,
            }],
            output='screen',
            # Every node ages on this clock, including each stall detector. If
            # the throttle dies alone the whole system stops with nothing left
            # running that could notice, so its exit takes the launch down.
            on_exit=Shutdown(),
        ))
    # A full-HD image and the reverse-direction exposure trigger must not
    # share one bridge executor. Under GUI/RViz load, serialization or DDS
    # flow control for the image can otherwise starve the tiny trigger long
    # enough for the moving robot to leave its exposure target.
    actions.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='inspection_trigger_bridge',
        arguments=[
            CAMERA_TRIGGER_TOPIC + '@std_msgs/msg/Bool]gz.msgs.Boolean',
        ],
        parameters=[{
            'qos_overrides./simulation/inspection_camera/trigger.subscriber.reliability':
                'reliable',
        }],
        output='screen',
    ))

    actions.append(Node(
        package='climbot_gazebo',
        executable='camera_distortion_adapter.py',
        name='camera_distortion_adapter',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'camera_config': os.path.join(
                description_share, 'config', 'inspection_camera.yaml'),
            'render_focal_scale': float(
                simulation['inspection_camera'][
                    'render_overscan_focal_scale']),
            'exposure_scale': float(
                simulation['inspection_camera']['exposure_scale']),
            'output_noise_stddev': float(
                simulation['inspection_camera']['noise_stddev']),
            'output_noise_seed': int(
                simulation['inspection_camera']['noise_seed']),
        }],
        output='screen',
    ))

    # This is the only publisher allowed on the actuator-facing /cmd_vel.
    # Teleoperation, experiments, and autonomous control all feed the guarded
    # /control/cmd_vel input instead.
    actions.append(Node(
        package='climbot_control',
        executable='cmd_vel_watchdog_node',
        name='cmd_vel_watchdog',
        parameters=[
            control_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    ))

    # world -> wall -> odom -> base_link -> {imu, wheels, caster}. The wall
    # work frame and odom are the same frame by construction: the total
    # station already reports wall coordinates, so the link is the identity.
    actions.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_wall',
        arguments=[
            '--x', str(wall['origin_xyz'][0]),
            '--y', str(wall['origin_xyz'][1]),
            '--z', str(wall['origin_xyz'][2]),
            '--roll', str(wall['origin_rpy'][0]),
            '--pitch', str(wall['origin_rpy'][1]),
            '--yaw', str(wall['origin_rpy'][2]),
            '--frame-id', 'world', '--child-frame-id', 'wall',
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    ))

    actions.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='wall_to_odom',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'wall', '--child-frame-id', 'odom',
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    ))

    actions.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description,
        }],
        output='screen',
    ))

    actions.append(Node(
        package='climbot_gazebo',
        executable='total_station_sim_node',
        name='total_station_sim',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'wall_config': wall_config,
            'publish_rate_hz': LaunchConfiguration('total_station_rate_hz'),
            'position_stddev_m': LaunchConfiguration('total_station_stddev_m'),
            'fixed_delay_s': LaunchConfiguration('total_station_delay_s'),
            'drop_probability': LaunchConfiguration('total_station_drop_probability'),
            'random_seed': LaunchConfiguration('total_station_seed'),
            **localization_model,
        }],
        output='screen',
    ))

    actions.append(Node(
        package='climbot_gazebo',
        executable='wall_wheel_odom_adapter_node',
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
        executable='wall_imu_adapter_node',
        name='wall_imu_adapter',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'orientation_stddev_rad': LaunchConfiguration(
                'imu_orientation_stddev_rad'),
            'random_seed': LaunchConfiguration('imu_seed'),
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

    # This monitor is intentionally C++ and only observes the six small
    # localization topics.  It keeps fixed-size in-memory timing statistics,
    # emits threshold events and a ten-second wall-time summary, and never
    # subscribes to images or writes a per-frame diagnostic file.
    actions.append(Node(
        package='climbot_control',
        executable='localization_timing_monitor_node',
        name='localization_timing_monitor',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'summary_period_s': 10.0,
        }],
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
            'clock_publish_hz',
            default_value='0',
            description='ROS /clock rate; 0 directly bridges every Gazebo physics step.',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo without its GUI, for automated tests.',
        ),
        DeclareLaunchArgument(
            'gpu_backend',
            default_value='auto',
            description='Rendering backend: auto, software, wsl_d3d12, or native. '
                        'WSL auto uses the D3D12 GPU path.',
        ),
        DeclareLaunchArgument(
            'gui_gpu_backend',
            default_value='auto',
            description='Gazebo GUI backend: auto, shared, or software. Auto shares the '
                        'selected rendering backend.',
        ),
        DeclareLaunchArgument(
            'total_station_rate_hz',
            default_value='12.0',
            description='Simulated total-station observation frequency.',
        ),
        DeclareLaunchArgument(
            'total_station_stddev_m',
            default_value='0.001',
            description='One-sigma total-station position noise in metres.',
        ),
        DeclareLaunchArgument(
            'total_station_delay_s',
            default_value='0.01',
            description='Fixed total-station delivery delay in seconds.',
        ),
        DeclareLaunchArgument(
            'localization_profile',
            default_value='precision',
            description='Total-station model: precision or realistic.',
        ),
        DeclareLaunchArgument(
            'prism_extrinsic_error_mode',
            default_value='auto',
            description='Fixed prism residual: auto, enabled, or disabled.',
        ),
        DeclareLaunchArgument(
            'prism_extrinsic_error_robot_m',
            default_value='[0.020, -0.010, 0.0]',
            description='Actual prism-minus-EKF robot-frame residual [m].',
        ),
        DeclareLaunchArgument(
            'measurement_timestamp_error_mode',
            default_value='auto',
            description='Clock stamp bias/jitter: auto, enabled, or disabled.',
        ),
        DeclareLaunchArgument(
            'measurement_timestamp_bias_s',
            default_value='0.020',
            description='Total-station header-stamp clock bias in seconds.',
        ),
        DeclareLaunchArgument(
            'measurement_timestamp_jitter_stddev_s',
            default_value='0.002',
            description='Header-stamp zero-mean jitter one-sigma in seconds.',
        ),
        DeclareLaunchArgument(
            'measurement_timestamp_jitter_seed',
            default_value='20260827',
            description='Seed for total-station header-stamp jitter.',
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
            default_value='0.00174532925',
            description='IMU attitude one-sigma uncertainty in radians.',
        ),
        # The nodes have always had these; without the pass-through they could
        # only be reached by bypassing this launch with --ros-args -p, which
        # the regression script cannot do. A repeatability run needs to set
        # both seeds and hold everything else fixed. Defaults match the node
        # defaults, so declaring them changes no existing behaviour.
        DeclareLaunchArgument(
            'total_station_seed',
            default_value='42',
            description='Seed for the total-station noise and dropouts.',
        ),
        DeclareLaunchArgument(
            'total_station_drop_probability',
            default_value='0.0',
            description='Fraction of total-station observations to drop.',
        ),
        DeclareLaunchArgument(
            'imu_seed',
            default_value='17',
            description='Seed for the IMU attitude noise.',
        ),
        DeclareLaunchArgument(
            'wall_grid_spacing',
            default_value=repr(reference_grid_spacing()),
            description='Reference grid pitch on the wall face in metres; 0 '
                        'removes the grid. Photography runs remove it: it '
                        'repeats, and it is not on the wall plane.',
        ),
        DeclareLaunchArgument(
            'wall_texture',
            default_value='',
            description='Bake manifest from tools/bake_wall_texture.py to put '
                        'on the wall face; empty uses simulation.yaml.',
        ),
        DeclareLaunchArgument(
            'inspection_target',
            default_value='false',
            description='Show the asymmetric G1 calibration target at the '
                        'initial camera projection centre.',
        ),
        DeclareLaunchArgument(
            'inspection_flat_field_target',
            default_value='false',
            description='Cover the initial view with a uniform grey target '
                        'for LED flat-field calibration.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
