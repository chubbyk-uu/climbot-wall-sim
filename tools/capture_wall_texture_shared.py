#!/usr/bin/env python3
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

"""Static wall probes using the robot's actual camera and LED definitions.

Only one probe/light exists. Move it, wait for simulation settling, then
trigger a fresh exposure; do not illuminate a view with other probe lights.
This is a headless optical test, never dual-GUI performance evidence.
"""

import argparse
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from audit_wall_randomization import digest
from capture_wall_texture import image_array
from capture_wall_texture import stop_process
from climbot_gazebo import simulation_stack
from climbot_gazebo.camera_distortion import apply_relative_exposure
from climbot_gazebo.camera_distortion import load_calibration
from climbot_gazebo.camera_distortion import make_distortion_maps
from climbot_gazebo.camera_distortion import matrices
from climbot_gazebo.mesa_runtime import rendering_environment
from climbot_image_processing.processing import _processed_image
from climbot_image_processing.processing import _undistort_maps
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image as ImageMessage
import xacro
import yaml


def configuration():
    description = Path(get_package_share_directory('climbot_description'))
    gazebo = Path(get_package_share_directory('climbot_gazebo'))
    camera_path = description / 'config/inspection_camera.yaml'
    simulation_path = gazebo / 'config/simulation.yaml'
    camera = load_calibration(camera_path)
    simulation = yaml.safe_load(simulation_path.read_text())['simulation']
    return description, gazebo, camera, simulation, {
        'camera_yaml_sha256': digest(camera_path),
        'simulation_yaml_sha256': digest(simulation_path),
    }


def build_world(manifest, directory, description, gazebo):
    """Reuse current production sensor/light xacro; remove robot mechanics."""
    rendered = Path(
        simulation_stack.render_world(
            str(gazebo),
            str(description),
            0.,
            str(manifest)))
    document = ET.parse(rendered)
    rendered.unlink()
    world = document.getroot().find('world')
    for include in list(world.findall('include')):
        if include.findtext('uri') == 'model://climbot':
            world.remove(include)
    for plugin in list(world.findall('plugin')):
        if plugin.get('filename') == 'gz-sim-apply-link-wrench-system':
            world.remove(plugin)
    mappings = simulation_stack.robot_mappings(str(gazebo), str(description))
    component_path = gazebo / 'models/climbot/components.sdf.xacro'
    components = ET.fromstring(xacro.process_file(
        str(component_path), mappings=mappings).toxml()).find('model')
    probe = ET.SubElement(world, 'model', name='texture_shared_probe')
    ET.SubElement(probe, 'static').text = 'true'
    ET.SubElement(probe, 'pose').text = '0 0 -20 0 0 0'
    for name in ('inspection_camera_link', 'inspection_camera_bracket_link'):
        source = components.find(f'link[@name="{name}"]')
        link = ET.SubElement(probe, 'link', name=name)
        link.append(deepcopy(source.find('pose')))
        for element in list(source):
            if element.tag in ('sensor', 'light'):
                link.append(deepcopy(element))
    path = directory / 'probe_world.sdf'
    document.write(path, encoding='utf-8', xml_declaration=True)
    return path, world.get('name')


def probe_pose(view, camera, wall, simulation):
    """Return world base pose and independent optical-to-wall truth rotation."""
    heading = float(view.get('heading_rad', 0.0))
    roll, pitch = float(view.get('roll_rad', 0.0)), float(view.get('pitch_rad', 0.0))
    distance = float(view.get('distance_m', camera['optical_mount']['center_xyz_m'][2]))
    if not all(math.isfinite(float(v)) for v in (
            heading, roll, pitch, distance, view['x_m'], view['y_m'])):
        raise ValueError('probe pose must be finite')
    if not (.265 <= distance <= .285 and max(abs(roll), abs(pitch)) <= math.radians(2)):
        raise ValueError('probe lies outside the frozen optical envelope')
    base_to_wall = Rotation.from_euler('xyz', [roll, pitch, heading]).as_matrix()
    normal_tilt = math.acos(float(np.clip(base_to_wall[2, 2], -1, 1)))
    if normal_tilt > math.radians(2) + 1e-12:
        raise ValueError('combined probe tilt exceeds the frozen two-degree envelope')
    centre = np.array([view['x_m'], view['y_m'], distance], np.float64)
    offset = np.array(camera['optical_mount']['center_xyz_m'])
    base = centre - base_to_wall @ offset
    wall_to_world = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])
    face = simulation['wall']['centre_xyz'][0] + simulation['wall']['thickness_m'] / 2
    origin = np.array([face, wall['origin_xyz'][1], wall['origin_xyz'][2]])
    position = origin + wall_to_world @ base
    quaternion = Rotation.from_matrix(wall_to_world @ base_to_wall).as_quat()
    optical_to_wall = base_to_wall @ Rotation.from_euler(
        'xyz', camera['optical_mount']['rpy_rad']).as_matrix()
    return position, quaternion, centre, optical_to_wall


class Receiver(Node):
    def __init__(self):
        super().__init__('wall_texture_shared_capture')
        self.image = None
        self.stamp = -1
        self.sim_ns = 0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.subscription = self.create_subscription(
            ImageMessage, '/simulation/inspection_camera/ideal_image', self.receive, qos)
        self.clock_subscription = self.create_subscription(Clock, '/clock', self.clock, 10)

    def receive(self, message):
        self.image = image_array(message)
        self.stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    def clock(self, message):
        self.sim_ns = message.clock.sec * 1_000_000_000 + message.clock.nanosec


def request(arguments, environment, timeout=10):
    result = subprocess.run(
        arguments,
        env=environment,
        capture_output=True,
        text=True,
        timeout=timeout)
    if result.returncode or ('service' in arguments and 'data: true' not in result.stdout):
        raise RuntimeError(f'Gazebo request failed: {arguments}: {result.stderr} {result.stdout}')
    return result


def renderer_maps(root_pid, expected):
    pending, records = [root_pid], []
    while pending:
        pid = pending.pop()
        root = Path('/proc') / str(pid)
        try:
            pending.extend(map(int, (root / 'task' / str(pid) / 'children').read_text().split()))
            names = ('libgallium', 'libGLX_mesa', 'libEGL_mesa', 'libgbm')
            libraries = sorted({
                line.split()[-1] for line in (root / 'maps').read_text().splitlines()
                if any(name in line for name in names)})
        except (OSError, ValueError):
            continue
        if any('libgallium' in item for item in libraries):
            if expected['backend'] == 'wsl_d3d12' and any(
                    'libgallium' in item and digest(item) != expected['gallium_sha256']
                    for item in libraries):
                raise RuntimeError('Gazebo mapped an unexpected Gallium library')
            records.append({'pid': pid, 'libraries': libraries})
    if expected['backend'] == 'wsl_d3d12' and not records:
        raise RuntimeError('no actual Gazebo renderer library mapping found')
    return records


def capture(manifest, views, output, flat_field, timeout_s=120):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    producer_path = output / 'producer_capture_wall_texture_shared.py'
    producer_path.write_bytes(Path(__file__).read_bytes())
    producer_hash = digest(producer_path)
    description, gazebo, camera, simulation, hashes = configuration()
    wall = yaml.safe_load((description / 'config/wall.yaml').read_text())['wall']
    world_path, world_name = build_world(Path(manifest).resolve(), output, description, gazebo)
    environment, graphics = rendering_environment()
    environment['GZ_PARTITION'] = f'climbot_texture_{os.getpid()}_{time.time_ns()}'
    profile = Path(get_package_share_directory('climbot_common')) / \
        'config/fastdds_inspection_image.xml'
    if profile.is_file() and not environment.get('FASTRTPS_DEFAULT_PROFILES_FILE'):
        environment['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(profile)
    matrix, distortion = matrices(camera)
    params = simulation['inspection_camera']
    map_x, map_y = make_distortion_maps(camera, params['render_overscan_focal_scale'])
    undistort_x, undistort_y = _undistort_maps(
        matrix, distortion, matrix, camera['image']['width_px'], camera['image']['height_px'])
    pin_path = Path(get_package_share_directory('climbot_image_processing')) / \
        'config/inspection_flat_field.yaml'
    pin = yaml.safe_load(pin_path.read_text())
    if digest(flat_field) != pin['sha256']:
        raise ValueError('probe flat-field hash differs from current pinned calibration')
    with np.load(flat_field, allow_pickle=False) as archive:
        gain = np.asarray(archive['gain'], np.float32)
    if gain.shape != map_x.shape or not np.isfinite(gain).all() or not (gain > 0).all():
        raise ValueError('invalid flat-field matrix')
    cv2.setNumThreads(1)
    cv2.setRNGSeed(params['noise_seed'])
    noise = np.empty(map_x.shape, np.float32)
    processes, logs, frames = [], [], []
    rclpy.init()
    receiver = Receiver()
    try:
        commands = [
            ('gazebo', ['gz', 'sim', '-s', '-r', '-v', '2', str(world_path),
                        '--force-version', '8']),
            ('bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                        '/simulation/inspection_camera/ideal_image'
                        '@sensor_msgs/msg/Image[gz.msgs.Image',
                        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']),
        ]
        for name, command in commands:
            log = open(output / f'{name}.log', 'w')
            logs.append(log)
            processes.append(subprocess.Popen(command, env=environment, stdout=log,
                                              stderr=subprocess.STDOUT, start_new_session=True))
        deadline = time.monotonic() + timeout_s
        while not receiver.sim_ns or not receiver.subscription.get_publisher_count():
            if any(p.poll() is not None for p in processes) or time.monotonic() > deadline:
                raise RuntimeError(
                    'shared camera sources did not become ready; see component logs')
            rclpy.spin_once(receiver, timeout_sec=.1)
        for index, view in enumerate(views):
            position, quaternion, centre, rotation = probe_pose(view, camera, wall, simulation)
            x, y, z = position
            qx, qy, qz, qw = quaternion
            body = (f'name: "texture_shared_probe", position {{x: {x} y: {y} z: {z}}}, '
                    f'orientation {{x: {qx} y: {qy} z: {qz} w: {qw}}}')
            request(['gz', 'service', '-s', f'/world/{world_name}/set_pose',
                     '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '5000', '--req', body], environment)
            settle = receiver.sim_ns + 100_000_000
            deadline = time.monotonic() + timeout_s
            while receiver.sim_ns < settle:
                if time.monotonic() > deadline:
                    raise TimeoutError('probe simulation did not settle')
                rclpy.spin_once(receiver, timeout_sec=.05)
            receiver.image = None
            earliest = receiver.sim_ns
            retry_at = 0.
            while receiver.image is None or receiver.stamp < earliest:
                if any(p.poll() is not None for p in processes) or time.monotonic() > deadline:
                    raise TimeoutError(
                        f'probe exposure {index} did not arrive; see component logs')
                if time.monotonic() >= retry_at:
                    request(['gz', 'topic', '-t', '/simulation/inspection_camera/trigger',
                             '-m', 'gz.msgs.Boolean', '-p', 'data: true'], environment)
                    retry_at = time.monotonic() + .5
                rclpy.spin_once(receiver, timeout_sec=.1)
            distorted = cv2.remap(receiver.image, map_x, map_y, cv2.INTER_LINEAR)
            raw = apply_relative_exposure(
                cv2.cvtColor(
                    distorted,
                    cv2.COLOR_RGB2GRAY),
                params['exposure_scale'])
            cv2.randn(noise, 0., params['noise_stddev'] * 255)
            raw = cv2.add(raw, noise, dtype=cv2.CV_8U)
            processed = _processed_image(raw, None, gain, 'none', undistort_x, undistort_y)
            raw_path, processed_path = output / \
                f'raw_{index:04d}.png', output / f'processed_{index:04d}.png'
            if not cv2.imwrite(
                    str(raw_path),
                    raw) or not cv2.imwrite(
                    str(processed_path),
                    processed):
                raise OSError('cannot write probe image')
            frames.append({'index': index,
                           'view': view,
                           'stamp_ns': receiver.stamp,
                           'optical_center_wall_m': centre.tolist(),
                           'optical_to_wall_rotation': rotation.tolist(),
                           'raw_file': raw_path.name,
                           'raw_sha256': digest(raw_path),
                           'processed_file': processed_path.name,
                           'processed_sha256': digest(processed_path)})
        graphics['actual_renderer_maps'] = renderer_maps(processes[0].pid, graphics)
        # Retire any bounded trigger retries before unloading the renderer.
        # This is a fixture quiet barrier, not a claim to fix native teardown.
        quiet_stamp = receiver.stamp
        quiet_until = receiver.sim_ns + 1_000_000_000
        quiet_deadline = time.monotonic() + 5
        while receiver.sim_ns < quiet_until and time.monotonic() < quiet_deadline:
            rclpy.spin_once(receiver, timeout_sec=.05)
            if receiver.stamp != quiet_stamp:
                quiet_stamp = receiver.stamp
                quiet_until = receiver.sim_ns + 1_000_000_000
    finally:
        receiver.destroy_node()
        rclpy.shutdown()
        for process in reversed(processes):
            stop_process(process)
        for log in logs:
            log.close()
    document = {'schema_version': 1, 'camera': camera, 'simulation_camera': params,
                'manifest_sha256': digest(manifest), 'flat_field_sha256': digest(flat_field),
                'graphics': graphics, 'input_hashes': hashes, 'frames': frames,
                'cleanup_exit_codes': [p.returncode for p in processes],
                'cleanup_ok': all(p.returncode in (0, -15) for p in processes),
                'producer_sha256': producer_hash,
                'probe_world_sha256': digest(world_path),
                'scope': ('single static probe; production camera/LED and processing; '
                          'no robot occlusion or GUI performance')}
    (output / 'capture.json').write_text(json.dumps(document, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'frames': len(frames), 'output': str(output),
                      'cleanup_exit_codes': document['cleanup_exit_codes']}))
    return document


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--views-json', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--flat-field-file', type=Path, required=True)
    args = parser.parse_args()
    result = capture(
        args.manifest,
        json.loads(
            args.views_json.read_text()),
        args.output_dir,
        args.flat_field_file)
    return 0 if result['cleanup_ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
