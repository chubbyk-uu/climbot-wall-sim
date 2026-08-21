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

"""Capture reproducible Gazebo camera frames from a textured wall manifest."""

import argparse
import importlib.util
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import time
from xml.dom import minidom

from ament_index_python.packages import get_package_share_directory
import numpy as np
from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image as ImageMessage
import yaml


def load_wall_launch():
    """Load render_world from the package launch file without copying it."""
    share = get_package_share_directory('climbot_gazebo')
    path = os.path.join(share, 'launch', 'climbot_wall.launch.py')
    specification = importlib.util.spec_from_file_location(
        'climbot_wall_launch_for_capture', path)
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def camera_fragment(index, work_x, work_y, distance, yaw_error, width, height,
                    field_width_m, wall_origin, wall_face_x):
    """Return one static camera looking along the wall's outward normal."""
    horizontal_fov = 2.0 * math.atan(field_width_m / (2.0 * distance))
    world_y = wall_origin[1] + work_x
    world_z = wall_origin[2] + work_y
    world_x = wall_face_x + distance
    topic = '/texture_probe/p%03d' % index
    return minidom.parseString(
        """<model name="texture_probe_%03d">
  <static>true</static>
  <pose>%.9f %.9f %.9f 0 0 %.9f</pose>
  <link name="camera_link">
    <sensor name="camera" type="camera">
      <always_on>true</always_on>
      <update_rate>0.5</update_rate>
      <topic>%s</topic>
      <camera>
        <horizontal_fov>%.12f</horizontal_fov>
        <image>
          <width>%d</width>
          <height>%d</height>
          <format>R8G8B8</format>
        </image>
        <clip><near>0.05</near><far>5.0</far></clip>
      </camera>
    </sensor>
  </link>
</model>""" % (
            index, world_x, world_y, world_z, math.pi + yaw_error, topic,
            horizontal_fov, width, height)).documentElement


def build_capture_world(manifest, cameras, output_path, width, height,
                        field_width_m):
    """Render the wall world without its robot and add probe cameras."""
    wall_launch = load_wall_launch()
    gazebo_share = get_package_share_directory('climbot_gazebo')
    description_share = get_package_share_directory('climbot_description')
    wall_path = os.path.join(description_share, 'config', 'wall.yaml')
    with open(wall_path, encoding='utf-8') as handle:
        wall = yaml.safe_load(handle)['wall']
    simulation_path = os.path.join(
        gazebo_share, 'config', 'simulation.yaml')
    with open(simulation_path, encoding='utf-8') as handle:
        simulation = yaml.safe_load(handle)['simulation']
    wall_origin = [float(value) for value in wall['origin_xyz']]
    wall_size = (float(wall['surface']['width_m']),
                 float(wall['surface']['height_m']))
    centre_x = float(simulation['wall']['centre_xyz'][0])
    wall_face_x = centre_x + float(simulation['wall']['thickness_m']) / 2.0
    rendered = wall_launch.render_world(
        gazebo_share, description_share, 0.0, os.path.abspath(manifest))
    document = minidom.parse(rendered)
    os.unlink(rendered)
    world = document.getElementsByTagName('world')[0]

    for include in list(document.getElementsByTagName('include')):
        uris = include.getElementsByTagName('uri')
        if uris and uris[0].firstChild.data.strip() == 'model://climbot':
            include.parentNode.removeChild(include)
    for plugin in list(document.getElementsByTagName('plugin')):
        filename = plugin.getAttribute('filename')
        if filename == 'gz-sim-apply-link-wrench-system':
            plugin.parentNode.removeChild(plugin)

    for index, camera in enumerate(cameras):
        for key in ('x_m', 'y_m', 'distance_m'):
            if key not in camera or not math.isfinite(float(camera[key])):
                raise ValueError('camera %d needs finite %s' % (index, key))
        work_x = float(camera['x_m'])
        work_y = float(camera['y_m'])
        distance = float(camera['distance_m'])
        yaw_error = float(camera.get('yaw_error_rad', 0.0))
        if distance <= 0.05:
            raise ValueError('camera %d distance_m must exceed 0.05' % index)
        if not math.isfinite(yaw_error):
            raise ValueError('camera %d yaw_error_rad must be finite' % index)
        if not (0.0 <= work_x <= wall_size[0] and
                0.0 <= work_y <= wall_size[1]):
            raise ValueError(
                'camera %d lies outside the %.3f x %.3f m wall' %
                (index, wall_size[0], wall_size[1]))
        element = camera_fragment(
            index, work_x, work_y, distance, yaw_error, width, height,
            field_width_m, wall_origin, wall_face_x)
        world.appendChild(document.importNode(element, True))
    with open(output_path, 'w', encoding='utf-8') as handle:
        handle.write(document.toprettyxml(indent='  '))


def image_array(message):
    """Convert the ROS image encodings emitted by ros_gz_bridge to RGB."""
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(
        message.height, message.step)
    if message.encoding in ('rgb8', 'bgr8'):
        image = rows[:, :message.width * 3].reshape(
            message.height, message.width, 3)
        if message.encoding == 'bgr8':
            image = image[:, :, ::-1]
        return image.copy()
    if message.encoding in ('rgba8', 'bgra8'):
        image = rows[:, :message.width * 4].reshape(
            message.height, message.width, 4)[:, :, :3]
        if message.encoding == 'bgra8':
            image = image[:, :, ::-1]
        return image.copy()
    raise ValueError('unsupported camera encoding %s' % message.encoding)


class Collector(Node):
    """Save the first valid frame received from every requested camera."""

    def __init__(self, count, output_dir):
        """Subscribe to all probe-camera image topics."""
        super().__init__('wall_texture_capture')
        self.output_dir = output_dir
        self.received = {}
        self.camera_subscriptions = []
        for index in range(count):
            topic = '/texture_probe/p%03d' % index
            self.camera_subscriptions.append(self.create_subscription(
                ImageMessage, topic,
                lambda message, i=index: self.receive(i, message),
                qos_profile_sensor_data))

    def receive(self, index, message):
        """Write one camera exactly once."""
        if index in self.received:
            return
        image = image_array(message)
        path = os.path.join(self.output_dir, 'p%03d.png' % index)
        Image.fromarray(image).save(path)
        self.received[index] = {
            'file': os.path.basename(path), 'width': int(message.width),
            'height': int(message.height), 'encoding': message.encoding,
            'stamp_ns': (int(message.header.stamp.sec) * 1000000000 +
                         int(message.header.stamp.nanosec)),
        }


def stop_process(process):
    """Terminate one independently started process group."""
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5.0)


def capture(manifest, cameras, output_dir, timeout_s, width, height,
            field_width_m, log_prefix):
    """Run Gazebo server and a bridge until every camera has one frame."""
    os.makedirs(output_dir, exist_ok=True)
    with tempfile.NamedTemporaryFile(
            prefix='climbot_texture_capture_', suffix='.sdf',
            delete=False) as temporary:
        world_path = temporary.name
    build_capture_world(
        manifest, cameras, world_path, width, height, field_width_m)

    environment = os.environ.copy()
    environment['GALLIUM_DRIVER'] = 'd3d12'
    environment['MESA_D3D12_DEFAULT_ADAPTER_NAME'] = 'NVIDIA'
    gazebo_log = open(log_prefix + '_gazebo.log', 'w', encoding='utf-8')
    bridge_log = open(log_prefix + '_bridge.log', 'w', encoding='utf-8')
    gazebo = bridge = None
    rclpy.init(args=None)
    collector = Collector(len(cameras), output_dir)
    try:
        gazebo = subprocess.Popen(
            ['gz', 'sim', '-s', '-r', '-v', '3', world_path,
             '--force-version', '8'], stdout=gazebo_log,
            stderr=subprocess.STDOUT, env=environment,
            start_new_session=True)
        arguments = [
            '/texture_probe/p%03d@sensor_msgs/msg/Image[gz.msgs.Image' % index
            for index in range(len(cameras))]
        bridge = subprocess.Popen(
            ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge'] + arguments,
            stdout=bridge_log, stderr=subprocess.STDOUT, env=environment,
            start_new_session=True)
        deadline = time.monotonic() + timeout_s
        while len(collector.received) < len(cameras):
            if gazebo.poll() is not None:
                raise RuntimeError('Gazebo exited with %d; see %s' %
                                   (gazebo.returncode, gazebo_log.name))
            if bridge.poll() is not None:
                raise RuntimeError('bridge exited with %d; see %s' %
                                   (bridge.returncode, bridge_log.name))
            if time.monotonic() >= deadline:
                missing = sorted(set(range(len(cameras))) -
                                 set(collector.received))
                raise TimeoutError('camera timeout; missing %s' % missing)
            rclpy.spin_once(collector, timeout_sec=0.2)
    finally:
        collector.destroy_node()
        rclpy.shutdown()
        if bridge is not None:
            stop_process(bridge)
        if gazebo is not None:
            stop_process(gazebo)
        gazebo_log.close()
        bridge_log.close()
        try:
            os.unlink(world_path)
        except FileNotFoundError:
            pass
    return collector.received


def main():
    """Capture cameras described by a JSON list."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', required=True)
    parser.add_argument('--cameras-json', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--log-prefix', required=True)
    parser.add_argument('--timeout-s', type=float, default=60.0)
    parser.add_argument('--width', type=int, default=1920)
    parser.add_argument('--height', type=int, default=1080)
    parser.add_argument('--field-width-m', type=float, default=0.50)
    arguments = parser.parse_args()
    with open(arguments.cameras_json, encoding='utf-8') as handle:
        cameras = json.load(handle)
    received = capture(
        arguments.manifest, cameras, arguments.output_dir,
        arguments.timeout_s, arguments.width, arguments.height,
        arguments.field_width_m, arguments.log_prefix)
    print(json.dumps(
        {'captured': len(received), 'frames': received}, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
