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
from climbot_interfaces.srv import CaptureOnce
from climbot_gazebo.camera_distortion import (
    apply_relative_exposure,
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
        self.public_image_subscription = self.create_subscription(
            Image, '/inspection/camera/image_raw', self._public_image, qos)
        self.public_info_subscription = self.create_subscription(
            CameraInfo, '/inspection/camera/camera_info', self._public_info, qos)
        self.ideal_image_subscription = self.create_subscription(
            Image, '/simulation/inspection_camera/ideal_image', self._ideal_image, qos)
        self.client = self.create_client(CaptureOnce, '/inspection/capture_once')
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

    def wait_for_sources(self, timeout_s):
        """Finish DDS discovery before a volatile one-shot frame can be sent."""
        subscriptions = (
            self.public_image_subscription,
            self.public_info_subscription,
            self.ideal_image_subscription,
        )
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if all(item.get_publisher_count() > 0 for item in subscriptions):
                return
            self.spin_for(0.05)
        raise RuntimeError('camera image publishers did not finish DDS discovery')

    def capture(self, timeout_s):
        if not self.client.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError('/inspection/capture_once is unavailable')
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            future = self.client.call_async(CaptureOnce.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
            if not future.done() or future.result() is None:
                raise RuntimeError('capture_once did not answer')
            result = future.result()
            if result.success:
                return result.message
            if result.reason != CaptureOnce.Response.WARMING:
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
    """Interpret tightly packed RGB8 render frames or mono8 camera output."""
    channels = {'rgb8': 3, 'mono8': 1}.get(message.encoding)
    if channels is None or message.step != message.width * channels:
        raise ValueError('camera image must be tightly packed rgb8 or mono8')
    array = np.frombuffer(bytes(message.data), dtype=np.uint8)
    expected = int(message.height * message.step)
    if array.size != expected:
        raise ValueError('camera image data length does not match height * step')
    shape = ((message.height, message.width, 3) if channels == 3 else
             (message.height, message.width))
    return array.reshape(shape)


def marker_metrics(raw_array):
    """Locate the asymmetric target colors and prove the two image axes."""
    red = (
        (raw_array[:, :, 0] > 140) &
        (raw_array[:, :, 0] > 1.6 * raw_array[:, :, 1]) &
        (raw_array[:, :, 0] > 1.6 * raw_array[:, :, 2]))
    green = (
        (raw_array[:, :, 1] > 140) &
        (raw_array[:, :, 1] > 1.6 * raw_array[:, :, 0]) &
        (raw_array[:, :, 1] > 1.6 * raw_array[:, :, 2]))
    blue = (
        (raw_array[:, :, 2] > 140) &
        (raw_array[:, :, 2] > 1.6 * raw_array[:, :, 0]) &
        (raw_array[:, :, 2] > 1.6 * raw_array[:, :, 1]))

    def describe(mask):
        rows, columns = np.nonzero(mask)
        return {
            'pixel_count': int(rows.size),
            'centroid_xy_px': [
                float(np.mean(columns)) if columns.size else None,
                float(np.mean(rows)) if rows.size else None,
            ],
        }

    markers = {'red_forward': describe(red), 'green_up': describe(green),
               'blue_centre': describe(blue)}
    height, width = raw_array.shape[:2]
    enough = all(marker['pixel_count'] >= 100 for marker in markers.values())
    if enough:
        red_xy = markers['red_forward']['centroid_xy_px']
        green_xy = markers['green_up']['centroid_xy_px']
        blue_xy = markers['blue_centre']['centroid_xy_px']
        checks = {
            'target_markers_visible': True,
            'robot_forward_appears_image_top': red_xy[1] < blue_xy[1] - 0.10 * height,
            'wall_up_appears_image_left': green_xy[0] < blue_xy[0] - 0.10 * width,
            'target_centre_near_principal_point': (
                abs(blue_xy[0] - width / 2.0) < 0.08 * width and
                abs(blue_xy[1] - height / 2.0) < 0.08 * height),
        }
    else:
        checks = {
            'target_markers_visible': False,
            'robot_forward_appears_image_top': False,
            'wall_up_appears_image_left': False,
            'target_centre_near_principal_point': False,
        }
    return {'markers': markers, 'checks': checks}


def contiguous_groups(indices):
    """Turn sorted threshold indices into inclusive pixel bands."""
    if not len(indices):
        return []
    groups = [[int(indices[0]), int(indices[0])]]
    for value in indices[1:]:
        value = int(value)
        if value == groups[-1][1] + 1:
            groups[-1][1] = value
        else:
            groups.append([value, value])
    return groups


def straight_line_residuals(mask, horizontal):
    """Fit the centre of each long colored stripe, ignoring its thickness."""
    height, width = mask.shape
    if horizontal:
        groups = contiguous_groups(np.flatnonzero(
            np.sum(mask, axis=1) > 0.20 * width))
    else:
        groups = contiguous_groups(np.flatnonzero(
            np.sum(mask, axis=0) > 0.20 * height))
    residuals = []
    for start, end in groups:
        coordinates = []
        centres = []
        if horizontal:
            for column in range(width):
                rows = np.flatnonzero(mask[start:end + 1, column]) + start
                if rows.size:
                    coordinates.append(column)
                    centres.append(float(np.mean(rows)))
        else:
            for row in range(height):
                columns = np.flatnonzero(mask[row, start:end + 1]) + start
                if columns.size:
                    coordinates.append(row)
                    centres.append(float(np.mean(columns)))
        if len(coordinates) < 100:
            continue
        fit = np.polyfit(coordinates, centres, 1)
        errors = np.asarray(centres) - np.polyval(fit, coordinates)
        residuals.append(float(np.sqrt(np.mean(errors * errors))))
    return residuals


def calibration_grid_metrics(raw_array, matrix, distortion):
    """Undistort the rendered target and measure its physical straight lines."""
    rectified = cv2.undistort(raw_array, matrix, distortion)

    def yellow_mask(image):
        minimum_rg = np.minimum(image[:, :, 0], image[:, :, 1])
        return (
            (image[:, :, 0] > 40) & (image[:, :, 1] > 40) &
            (minimum_rg > 1.5 * image[:, :, 2]))

    raw_mask = yellow_mask(raw_array)
    rectified_mask = yellow_mask(rectified)
    raw_horizontal = straight_line_residuals(raw_mask, horizontal=True)
    raw_vertical = straight_line_residuals(raw_mask, horizontal=False)
    rectified_horizontal = straight_line_residuals(rectified_mask, horizontal=True)
    rectified_vertical = straight_line_residuals(rectified_mask, horizontal=False)
    rectified_all = rectified_horizontal + rectified_vertical
    maximum = max(rectified_all) if rectified_all else None
    checks = {
        'calibration_grid_has_5_horizontal_lines': len(rectified_horizontal) == 5,
        'calibration_grid_has_9_vertical_lines': len(rectified_vertical) == 9,
        'rectified_line_residual_rms_le_1px': (
            maximum is not None and maximum <= 1.0),
    }
    return {
        'raw_horizontal_rms_px': raw_horizontal,
        'raw_vertical_rms_px': raw_vertical,
        'rectified_horizontal_rms_px': rectified_horizontal,
        'rectified_vertical_rms_px': rectified_vertical,
        'rectified_max_rms_px': maximum,
        'checks': checks,
    }


def target_obstruction_metrics(raw_array, matrix, distortion, camera):
    """Require every inset effective-ROI pixel to belong to the emissive target."""
    rectified = cv2.undistort(raw_array, matrix, distortion)
    intrinsics = camera['calibration']['intrinsics']
    footprint = camera['footprint']
    distance = camera['optical_mount']['center_xyz_m'][2]
    half_width = (
        0.5 * footprint['effective_width_m'] * intrinsics['fx_px'] / distance)
    half_height = (
        0.5 * footprint['effective_length_m'] * intrinsics['fy_px'] / distance)
    margin = 20
    left = int(math.ceil(intrinsics['cx_px'] - half_width)) + margin
    right = int(math.floor(intrinsics['cx_px'] + half_width)) - margin
    top = int(math.ceil(intrinsics['cy_px'] - half_height)) + margin
    bottom = int(math.floor(intrinsics['cy_px'] + half_height)) - margin
    roi = rectified[top:bottom + 1, left:right + 1]
    maximum = np.max(roi, axis=2)
    minimum = np.min(roi, axis=2)
    white = minimum > 170
    red = (
        (roi[:, :, 0] > 120) &
        (roi[:, :, 1] < 0.25 * roi[:, :, 0]) &
        (roi[:, :, 2] < 0.25 * roi[:, :, 0]))
    green = (
        (roi[:, :, 1] > 120) &
        (roi[:, :, 0] < 0.25 * roi[:, :, 1]) &
        (roi[:, :, 2] < 0.25 * roi[:, :, 1]))
    blue = (
        (roi[:, :, 2] > 120) &
        (roi[:, :, 0] < 0.25 * roi[:, :, 2]) &
        (roi[:, :, 1] < 0.25 * roi[:, :, 2]))
    yellow = (
        (roi[:, :, 0] > 80) & (roi[:, :, 1] > 80) &
        (np.minimum(roi[:, :, 0], roi[:, :, 1]) > 1.4 * roi[:, :, 2]) &
        (np.abs(
            roi[:, :, 0].astype(np.int16) - roi[:, :, 1].astype(np.int16)) <
         0.20 * maximum))
    known_target = white | red | green | blue | yellow
    raw_unknown_count = int(np.count_nonzero(~known_target))
    # Ogre's bilinear edge pixels are blends of two valid target colors and
    # intentionally fail both strict color tests. Absorb exactly that one
    # raster pixel; an occluding model still leaves its interior unknown.
    known_with_edges = cv2.dilate(
        known_target.astype(np.uint8), np.ones((3, 3), dtype=np.uint8)) > 0
    unknown = ~known_with_edges
    unknown_count = int(np.count_nonzero(unknown))
    pixel_count = int(unknown.size)
    unknown_ratio = unknown_count / pixel_count
    minimum_brightest = int(np.min(maximum))
    return {
        'rectified_inset_roi_xyxy_px': [left, top, right, bottom],
        'pixel_count': pixel_count,
        'raw_unknown_pixel_count': raw_unknown_count,
        'unknown_after_one_pixel_edge_allowance': unknown_count,
        'unknown_ratio': unknown_ratio,
        'minimum_brightest_channel': minimum_brightest,
        'checks': {
            'effective_roi_has_no_modelled_obstruction': (
                unknown_count == 0 and minimum_brightest >= 150),
        },
    }


def compare_frame(evaluator, camera, render_scale, exposure_scale,
                  check_target=False):
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
    expected_rgb = cv2.remap(
        ideal_array, map_x, map_y, interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REPLICATE)
    expected_raw = apply_relative_exposure(
        cv2.cvtColor(expected_rgb, cv2.COLOR_RGB2GRAY), exposure_scale)
    ideal_gray = apply_relative_exposure(
        cv2.cvtColor(ideal_array, cv2.COLOR_RGB2GRAY), exposure_scale)
    delta = raw_array.astype(np.float64) - expected_raw.astype(np.float64)
    ideal_delta = raw_array.astype(np.float64) - ideal_gray.astype(np.float64)
    result = {
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
            'mono8_output': image.encoding == 'mono8',
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
    if check_target:
        # Axis colors are a simulation-only calibration aid. Inspect the
        # geometrically identical RGB render here; the public industrial
        # camera output correctly remains mono8.
        target = marker_metrics(expected_rgb)
        target['calibration_grid'] = calibration_grid_metrics(
            expected_rgb, matrix, distortion)
        target['obstruction'] = target_obstruction_metrics(
            expected_rgb, matrix, distortion, camera)
        target['checks'].update(target['calibration_grid']['checks'])
        target['checks'].update(target['obstruction']['checks'])
        result['target'] = target
        result['checks'].update(target['checks'])
    return result


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
    parser.add_argument('--check-target', action='store_true')
    parser.add_argument('--image-output', default='')
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
        evaluator.wait_for_sources(args.timeout)
        evaluator.spin_for(args.idle_seconds)
        idle_frames = len(evaluator.public_images)
        service_message = evaluator.capture(args.timeout)
        evaluator.spin_for(1.0)
        frame = compare_frame(
            evaluator, camera, simulation['render_overscan_focal_scale'],
            simulation['exposure_scale'],
            check_target=args.check_target)
        if args.image_output:
            key = next(iter(evaluator.public_images))
            captured = message_array(evaluator.public_images[key])
            directory = os.path.dirname(os.path.abspath(args.image_output))
            os.makedirs(directory, exist_ok=True)
            if not cv2.imwrite(args.image_output, captured):
                raise RuntimeError('failed to write --image-output')
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
