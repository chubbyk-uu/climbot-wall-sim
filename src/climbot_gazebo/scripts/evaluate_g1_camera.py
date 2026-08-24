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

"""Produce traceable G1 evidence from one real triggered Gazebo frame."""

import argparse
import json
import math
import os
import time

from ament_index_python.packages import get_package_share_directory
from climbot_gazebo.camera_distortion import (
    load_calibration,
    make_distortion_maps,
    matrices,
)
from climbot_gazebo.provenance import git_state
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
import yaml


def stamp_key(message):
    """Return an exact integer key shared by Image and CameraInfo."""
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def quaternion_from_rpy(roll, pitch, yaw):
    """Use the fixed-axis roll-pitch-yaw convention used by URDF."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


class G1CameraEvaluator(Node):
    """Observe public and private simulation topics without feeding control."""

    def __init__(self):
        super().__init__('g1_camera_evaluator')
        qos = QoSProfile(depth=4, reliability=ReliabilityPolicy.RELIABLE)
        self.public_images = {}
        self.public_infos = {}
        self.ideal_images = {}
        self.create_subscription(
            Image, '/inspection/camera/image_raw', self._public_image, qos)
        self.create_subscription(
            CameraInfo, '/inspection/camera/camera_info', self._public_info, qos)
        self.create_subscription(
            Image, '/simulation/inspection_camera/ideal_image', self._ideal_image, qos)
        self.client = self.create_client(Trigger, '/inspection/capture_once')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _public_image(self, message):
        self.public_images[stamp_key(message)] = message

    def _public_info(self, message):
        self.public_infos[stamp_key(message)] = message

    def _ideal_image(self, message):
        self.ideal_images[stamp_key(message)] = message

    def spin_for(self, duration_s):
        deadline = time.monotonic() + duration_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.05, deadline - time.monotonic()))

    def capture(self, timeout_s):
        if not self.client.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError('/inspection/capture_once is unavailable')
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            future = self.client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
            if not future.done() or future.result() is None:
                raise RuntimeError('capture_once did not answer')
            result = future.result()
            if result.success:
                return result.message
            if 'warming up' not in result.message:
                raise RuntimeError('capture_once failed: ' + result.message)
            self.spin_for(0.10)
        raise RuntimeError('camera did not finish warm-up')


def finite_list(values):
    return all(math.isfinite(float(value)) for value in values)


def payload_mass_properties(robot):
    """Recompute the same proxy total mass and COM used by description tests."""
    wheel = robot['drive_wheel']
    components = [
        (robot['base']['mass_kg'], robot['base']['centre_xyz']),
        (wheel['mass_kg'], [
            wheel['axle_x_m'], wheel['separation_m'] / 2.0, wheel['radius_m']]),
        (wheel['mass_kg'], [
            wheel['axle_x_m'], -wheel['separation_m'] / 2.0, wheel['radius_m']]),
        (robot['caster']['mass_kg'], [
            robot['caster']['centre_x_m'], 0.0, robot['caster']['radius_m']]),
        (robot['inspection_payload']['camera_body']['mass_kg'],
         robot['inspection_payload']['camera_body']['centre_xyz_m']),
        (robot['inspection_payload']['bracket']['mass_kg'],
         robot['inspection_payload']['bracket']['centre_xyz_m']),
    ]
    total_mass = float(sum(mass for mass, _ in components))
    centre = [
        float(sum(mass * xyz[axis] for mass, xyz in components) / total_mass)
        for axis in range(3)
    ]
    return {'total_mass_kg': total_mass, 'centre_of_mass_xyz_m': centre}


def message_array(message):
    """Interpret the configured RGB8 source without a second conversion path."""
    if message.encoding != 'rgb8' or message.step != message.width * 3:
        raise ValueError('camera image must be tightly packed rgb8')
    array = np.frombuffer(bytes(message.data), dtype=np.uint8)
    expected = int(message.height * message.step)
    if array.size != expected:
        raise ValueError('camera image data length does not match height * step')
    return array.reshape(message.height, message.width, 3)


def compare_frame(evaluator, camera, render_scale):
    """Validate the single public pair and recompute the Brown pixel mapping."""
    common = set(evaluator.public_images) & set(evaluator.public_infos)
    if len(common) != 1:
        raise RuntimeError('expected exactly one public image/info timestamp pair')
    key = next(iter(common))
    image = evaluator.public_images[key]
    info = evaluator.public_infos[key]
    ideal = evaluator.ideal_images.get(key)
    if ideal is None:
        raise RuntimeError('matching private ideal image was not observed')

    width = int(camera['image']['width_px'])
    height = int(camera['image']['height_px'])
    matrix, distortion = matrices(camera)
    expected_projection = [
        matrix[0, 0], 0.0, matrix[0, 2], 0.0,
        0.0, matrix[1, 1], matrix[1, 2], 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]
    map_x, map_y = make_distortion_maps(camera, render_scale)
    ideal_array = message_array(ideal)
    raw_array = message_array(image)
    expected_raw = cv2.remap(
        ideal_array, map_x, map_y, interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REPLICATE)
    delta = raw_array.astype(np.float64) - expected_raw.astype(np.float64)
    ideal_delta = raw_array.astype(np.float64) - ideal_array.astype(np.float64)
    return {
        'stamp_ns': key,
        'width_px': int(image.width),
        'height_px': int(image.height),
        'encoding': image.encoding,
        'frame_id': image.header.frame_id,
        'distortion_model': info.distortion_model,
        'k': list(info.k),
        'd': list(info.d),
        'p': list(info.p),
        'brown_mapping_rms_px': float(np.sqrt(np.mean(delta * delta))),
        'ideal_to_distorted_mean_abs_px': float(np.mean(np.abs(ideal_delta))),
        'checks': {
            'resolution': image.width == width and image.height == height,
            'matching_stamp_and_frame': (
                image.header == info.header and
                image.header.frame_id == 'inspection_camera_optical_frame'),
            'camera_info_finite': finite_list(list(info.k) + list(info.d) + list(info.p)),
            'camera_info_k': bool(np.allclose(
                info.k, matrix.reshape(-1), atol=1e-9)),
            'camera_info_d': bool(np.allclose(info.d, distortion, atol=1e-9)),
            'camera_info_p': bool(np.allclose(
                info.p, expected_projection, atol=1e-9)),
            'nonzero_distortion': (
                info.distortion_model == 'plumb_bob' and bool(np.any(distortion))),
            'brown_mapping_rms_le_1px': float(np.sqrt(np.mean(delta * delta))) <= 1.0,
            'distortion_visibly_changes_pixels': float(np.mean(np.abs(ideal_delta))) >= 1.0,
        },
    }


def compare_tf(evaluator, camera, timeout_s):
    """Compare the running static TF with the shared mechanical external calibration."""
    deadline = time.monotonic() + timeout_s
    transform = None
    while time.monotonic() < deadline and transform is None:
        try:
            transform = evaluator.tf_buffer.lookup_transform(
                'base_link', 'inspection_camera_optical_frame', rclpy.time.Time())
        except Exception:
            evaluator.spin_for(0.05)
    if transform is None:
        raise RuntimeError('base_link -> inspection_camera_optical_frame TF unavailable')
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    actual_xyz = np.array([translation.x, translation.y, translation.z])
    actual_quaternion = np.array([rotation.x, rotation.y, rotation.z, rotation.w])
    mount = camera['optical_mount']
    expected_xyz = np.asarray(mount['center_xyz_m'], dtype=np.float64)
    expected_quaternion = quaternion_from_rpy(*mount['rpy_rad'])
    translation_error = float(np.linalg.norm(actual_xyz - expected_xyz))
    dot = min(1.0, abs(float(np.dot(actual_quaternion, expected_quaternion))))
    rotation_error = math.degrees(2.0 * math.acos(dot))
    return {
        'translation_xyz_m': actual_xyz.tolist(),
        'translation_error_m': translation_error,
        'rotation_xyzw': actual_quaternion.tolist(),
        'rotation_error_deg': rotation_error,
        'checks': {
            'translation_error_le_1mm': translation_error <= 0.001,
            'rotation_error_le_0_1deg': rotation_error <= 0.1,
        },
    }


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--idle-seconds', type=float, default=10.0)
    parser.add_argument('--timeout', type=float, default=15.0)
    return parser.parse_args()


def main():
    args = parse_arguments()
    if not math.isfinite(args.idle_seconds) or args.idle_seconds < 0.0:
        raise ValueError('--idle-seconds must be finite and non-negative')
    if not math.isfinite(args.timeout) or args.timeout <= 0.0:
        raise ValueError('--timeout must be positive and finite')
    description = get_package_share_directory('climbot_description')
    gazebo = get_package_share_directory('climbot_gazebo')
    camera_path = os.path.join(description, 'config', 'inspection_camera.yaml')
    robot_path = os.path.join(description, 'config', 'robot.yaml')
    simulation_path = os.path.join(gazebo, 'config', 'simulation.yaml')
    camera = load_calibration(camera_path)
    with open(robot_path) as handle:
        robot = yaml.safe_load(handle)['robot']
    with open(simulation_path) as handle:
        simulation = yaml.safe_load(handle)['simulation']['inspection_camera']

    rclpy.init()
    evaluator = G1CameraEvaluator()
    try:
        evaluator.spin_for(args.idle_seconds)
        idle_frames = len(evaluator.public_images)
        service_message = evaluator.capture(args.timeout)
        evaluator.spin_for(1.0)
        frame = compare_frame(
            evaluator, camera, simulation['render_overscan_focal_scale'])
        transform = compare_tf(evaluator, camera, args.timeout)
        intrinsics = camera['calibration']['intrinsics']
        mount_distance = float(camera['optical_mount']['center_xyz_m'][2])
        full_view = [
            camera['image']['width_px'] * mount_distance / intrinsics['fx_px'],
            camera['image']['height_px'] * mount_distance / intrinsics['fy_px'],
        ]
        checks = {
            'idle_zero_frames': idle_frames == 0,
            'one_public_pair': (
                len(evaluator.public_images) == 1 and
                len(evaluator.public_infos) == 1),
            'raw_view_meets_nominal': full_view[0] >= 0.550 and full_view[1] >= 0.309375,
            'effective_view_meets_nominal': (
                camera['footprint']['effective_width_m'] >= 0.500 and
                camera['footprint']['effective_length_m'] >= 0.28125),
            **frame['checks'],
            **transform['checks'],
        }
        summary = {
            'schema_version': 1,
            'passed': all(checks.values()),
            'checks': checks,
            'service_message': service_message,
            'idle_observation_s': args.idle_seconds,
            'full_view_m': full_view,
            'effective_view_m': [
                camera['footprint']['effective_width_m'],
                camera['footprint']['effective_length_m'],
            ],
            'frame': frame,
            'transform': transform,
            'configuration': {
                'camera': camera,
                'inspection_payload': robot['inspection_payload'],
                'robot_mass_properties': payload_mass_properties(robot),
                'simulation_camera': simulation,
            },
            'provenance': {'git': git_state()},
        }
        os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
        with open(args.output, 'w') as handle:
            json.dump(summary, handle, indent=2, sort_keys=True, allow_nan=False)
            handle.write('\n')
        print('G1_CAMERA_PASS={} SUMMARY={}'.format(summary['passed'], args.output))
        if not summary['passed']:
            return 1
        return 0
    finally:
        evaluator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
