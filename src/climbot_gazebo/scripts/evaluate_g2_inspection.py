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

"""Evaluate G2 image spacing, binding and exposure-time EKF camera poses."""

from collections import defaultdict, deque
from datetime import datetime, timezone
import json
import math
import os
import time

from ament_index_python.packages import get_package_share_directory
from climbot_description.geometry import quaternion_tuple, yaw_from_quaternion
from climbot_description.wall_frame import WallFrame
from climbot_gazebo.coverage_metrics import footprint_coverage
from climbot_gazebo.provenance import git_state
from climbot_interfaces.msg import CoverageStatus, CoverageTask, InspectionCapture
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
import yaml


def stamp_ns(header):
    """Convert one ROS header stamp into its exact integer key."""
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def wrap(value):
    """Return an angle in [-pi, pi]."""
    return math.atan2(math.sin(value), math.cos(value))


class G2InspectionEvaluator(Node):
    """Observe a complete managed task without influencing capture decisions."""

    def __init__(self):
        super().__init__('g2_inspection_evaluator')
        self.declare_parameter('summary_path', '')
        self.declare_parameter('timeout_s', 600.0)
        self.declare_parameter('nominal_overlap_ratio', 0.25)
        self.declare_parameter('minimum_actual_overlap_ratio', 0.20)
        self.declare_parameter('maximum_camera_position_error_m', 0.005)
        self.declare_parameter('maximum_heading_error_deg', 1.0)
        self.declare_parameter('minimum_photo_coverage_ratio', 0.98)
        wall_path = os.path.join(
            get_package_share_directory('climbot_description'),
            'config', 'wall.yaml')
        self.wall = WallFrame.from_yaml(wall_path)
        camera_path = os.path.join(
            get_package_share_directory('climbot_description'),
            'config', 'inspection_camera.yaml')
        with open(camera_path) as handle:
            mount = yaml.safe_load(handle)['inspection_camera']['optical_mount']
        self.camera_offset = tuple(float(value) for value in mount['center_xyz_m'])
        self.task = None
        self.status = None
        self.images = defaultdict(int)
        self.metadata = []
        self.truth = deque()
        self.pending_truth = []
        self.position_errors = []
        self.position_error_components = []
        self.heading_errors = []
        self.finished_at = None
        reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            CoverageTask, '/coverage/task', self._on_task, latched)
        self.create_subscription(
            CoverageStatus, '/coverage/manager_status', self._on_status, latched)
        self.create_subscription(
            Image, '/inspection/camera/image_raw', self._on_image, reliable)
        self.create_subscription(
            InspectionCapture, '/inspection/capture_metadata',
            self._on_metadata, reliable)
        self.create_subscription(
            Odometry, '/model/climbot/ground_truth', self._on_truth, 100)

    def _on_task(self, message):
        if message.waypoints:
            self.task = message

    def _on_status(self, message):
        self.status = message
        if message.state == CoverageStatus.FINISHED and self.finished_at is None:
            self.finished_at = time.monotonic()

    def _on_image(self, message):
        self.images[stamp_ns(message.header)] += 1

    def _on_metadata(self, message):
        self.metadata.append(message)
        self.pending_truth.append(message)
        self._resolve_truth()

    def _on_truth(self, message):
        timestamp = stamp_ns(message.header)
        position = message.pose.pose.position
        wall_position = self.wall.position_from_world(
            (position.x, position.y, position.z))
        wall_orientation = self.wall.orientation_from_world(
            quaternion_tuple(message.pose.pose.orientation))
        self.truth.append((timestamp, wall_position,
                           yaw_from_quaternion(wall_orientation)))
        while len(self.truth) > 2 and timestamp - self.truth[0][0] > 5_000_000_000:
            self.truth.popleft()
        self._resolve_truth()

    def _truth_at(self, target):
        for first, second in zip(self.truth, list(self.truth)[1:]):
            if first[0] <= target <= second[0] and second[0] > first[0]:
                ratio = (target - first[0]) / (second[0] - first[0])
                position = tuple(
                    first[1][axis] + ratio * (second[1][axis] - first[1][axis])
                    for axis in range(3))
                heading = first[2] + ratio * wrap(second[2] - first[2])
                return position, heading
        return None

    def _resolve_truth(self):
        unresolved = []
        for metadata in self.pending_truth:
            truth = self._truth_at(stamp_ns(metadata.header))
            if truth is None:
                unresolved.append(metadata)
                continue
            base, heading = truth
            truth_camera = (
                base[0] + self.camera_offset[0] * math.cos(heading) -
                self.camera_offset[1] * math.sin(heading),
                base[1] + self.camera_offset[0] * math.sin(heading) +
                self.camera_offset[1] * math.cos(heading),
                base[2] + self.camera_offset[2],
            )
            estimated = metadata.camera_pose.pose.position
            components = (
                estimated.x - truth_camera[0],
                estimated.y - truth_camera[1],
                estimated.z - truth_camera[2],
            )
            self.position_error_components.append(components)
            self.position_errors.append(math.sqrt(sum(
                value * value for value in components)))
            self.heading_errors.append(abs(wrap(
                metadata.wall_heading_rad - heading)))
        self.pending_truth = unresolved

    def run(self):
        """Wait for one task result, then write strict, executable evidence."""
        deadline = time.monotonic() + float(self.get_parameter('timeout_s').value)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.finished_at is not None and time.monotonic() - self.finished_at >= 2.0:
                break
        if self.finished_at is None:
            raise RuntimeError('coverage task did not finish before evaluator timeout')
        return self._summarize()

    def _summarize(self):
        nominal_overlap = float(
            self.get_parameter('nominal_overlap_ratio').value)
        minimum_overlap = float(
            self.get_parameter('minimum_actual_overlap_ratio').value)
        groups = defaultdict(list)
        for item in self.metadata:
            groups[(item.task_id, item.revision, item.segment_index)].append(item)

        group_results = []
        line_centres = defaultdict(list)
        numbering_ok = True
        counts_ok = True
        maximum_target_gap = 0.0
        maximum_actual_gap = 0.0
        scan_only = True
        for key, values in sorted(groups.items()):
            values.sort(key=lambda item: item.trigger_index)
            indices = [item.trigger_index for item in values]
            numbering_ok &= indices == list(range(len(values)))
            segment = key[2]
            scan_only &= bool(
                self.task and 0 <= segment < len(self.task.segment_types) and
                self.task.segment_types[segment] == CoverageTask.SEGMENT_SCAN)
            reference_length = math.hypot(
                values[0].reference_end.x - values[0].reference_start.x,
                values[0].reference_end.y - values[0].reference_start.y)
            nominal_spacing = self.task.detection_length * (1.0 - nominal_overlap)
            capture_span = max(0.0, reference_length - self.task.detection_length)
            expected_count = (1 if capture_span <= 1e-9 else
                              math.ceil(capture_span / nominal_spacing) + 1)
            counts_ok &= len(values) == expected_count
            target_gaps = [
                second.target_along_track - first.target_along_track
                for first, second in zip(values, values[1:])]
            actual_gaps = [
                second.actual_along_track - first.actual_along_track
                for first, second in zip(values, values[1:])]
            if target_gaps:
                maximum_target_gap = max(maximum_target_gap, max(target_gaps))
                maximum_actual_gap = max(maximum_actual_gap, max(actual_gaps))
            reference_dx = (values[0].reference_end.x -
                            values[0].reference_start.x)
            reference_dy = (values[0].reference_end.y -
                            values[0].reference_start.y)
            direction = ('horizontal' if abs(reference_dx) >= abs(reference_dy)
                         else 'vertical')
            cross_coordinate = (
                sum(item.camera_pose.pose.position.y for item in values) / len(values)
                if direction == 'horizontal' else
                sum(item.camera_pose.pose.position.x for item in values) / len(values))
            line_centres[direction].append(cross_coordinate)
            group_results.append({
                'task_id': key[0], 'revision': key[1], 'segment_index': segment,
                'captures': len(values), 'expected_captures': expected_count,
                'maximum_target_gap_m': max(target_gaps) if target_gaps else 0.0,
                'maximum_actual_gap_m': max(actual_gaps) if actual_gaps else 0.0,
            })

        expected_scan_segments = [] if self.task is None else [
            index for index, kind in enumerate(self.task.segment_types)
            if kind == CoverageTask.SEGMENT_SCAN]
        captured_segments = sorted(key[2] for key in groups)
        all_scan_segments = captured_segments == expected_scan_segments
        image_keys = {key for key, count in self.images.items() if count == 1}
        metadata_keys = [stamp_ns(item.header) for item in self.metadata]
        exact_pairing = (
            len(metadata_keys) == len(set(metadata_keys)) and
            set(metadata_keys) == image_keys and
            all(count == 1 for count in self.images.values()))
        actual_limit = self.task.detection_length * (1.0 - minimum_overlap)
        lateral_limit = self.task.detection_width * (1.0 - minimum_overlap)
        maximum_lateral_spacing = 0.0
        for coordinates in line_centres.values():
            coordinates.sort()
            if len(coordinates) > 1:
                maximum_lateral_spacing = max(
                    maximum_lateral_spacing,
                    max(second - first for first, second in zip(
                        coordinates, coordinates[1:])))
        polygon = [(point.x, point.y) for point in self.task.coverage_region.points]
        photo_paths = [[(
            item.camera_pose.pose.position.x,
            item.camera_pose.pose.position.y,
            item.wall_heading_rad,
        )] for item in self.metadata]
        photo_coverage = footprint_coverage(
            polygon, photo_paths, self.task.detection_width,
            self.task.detection_length, resolution=0.01)
        minimum_photo_coverage = float(
            self.get_parameter('minimum_photo_coverage_ratio').value)
        position_limit = float(
            self.get_parameter('maximum_camera_position_error_m').value)
        heading_limit = math.radians(float(
            self.get_parameter('maximum_heading_error_deg').value))
        passed = all((
            self.task is not None,
            self.status.result_code == 0,
            bool(self.metadata), scan_only, all_scan_segments, numbering_ok,
            counts_ok, exact_pairing, not self.pending_truth,
            maximum_actual_gap <= actual_limit + 1e-9,
            maximum_lateral_spacing <= lateral_limit + 1e-9,
            photo_coverage['ratio'] >= minimum_photo_coverage,
            bool(self.position_errors) and max(self.position_errors) <= position_limit,
            bool(self.heading_errors) and max(self.heading_errors) <= heading_limit,
        ))
        summary = {
            'passed': passed,
            'task_id': self.task.task_id if self.task else None,
            'revision': self.task.revision if self.task else None,
            'captures': len(self.metadata),
            'scan_segments': len(expected_scan_segments),
            'captured_segments': captured_segments,
            'scan_only': scan_only,
            'all_scan_segments_captured': all_scan_segments,
            'numbering_complete': numbering_ok,
            'capture_counts_complete': counts_ok,
            'exact_image_metadata_pairing': exact_pairing,
            'nominal_overlap_ratio': nominal_overlap,
            'minimum_actual_overlap_ratio': minimum_overlap,
            'maximum_target_gap_m': maximum_target_gap,
            'maximum_actual_gap_m': maximum_actual_gap,
            'maximum_actual_gap_limit_m': actual_limit,
            'maximum_lateral_spacing_m': maximum_lateral_spacing,
            'maximum_lateral_spacing_limit_m': lateral_limit,
            'photo_coverage': photo_coverage,
            'minimum_photo_coverage_ratio': minimum_photo_coverage,
            'maximum_camera_position_error_m': (
                max(self.position_errors) if self.position_errors else None),
            'maximum_camera_position_error_components_m': (
                [max(abs(value[axis]) for value in self.position_error_components)
                 for axis in range(3)]
                if self.position_error_components else None),
            'maximum_camera_position_error_limit_m': position_limit,
            'maximum_heading_error_deg': (
                math.degrees(max(self.heading_errors)) if self.heading_errors else None),
            'maximum_heading_error_limit_deg': math.degrees(heading_limit),
            'groups': group_results,
            'provenance': {
                'recorded_utc': datetime.now(timezone.utc).isoformat(),
                'git': git_state(),
            },
        }
        path = str(self.get_parameter('summary_path').value)
        if path:
            with open(path, 'w') as handle:
                json.dump(summary, handle, indent=2, allow_nan=False)
                handle.write('\n')
        self.get_logger().info(
            'G2_RESULT passed=%s captures=%d segments=%d actual_gap_max=%.3f mm '
            'camera_error_max=%s mm heading_error_max=%s deg' % (
                passed, len(self.metadata), len(expected_scan_segments),
                maximum_actual_gap * 1000.0,
                ('%.3f' % (max(self.position_errors) * 1000.0)
                 if self.position_errors else 'n/a'),
                ('%.3f' % math.degrees(max(self.heading_errors))
                 if self.heading_errors else 'n/a')))
        return passed


def main(args=None):
    rclpy.init(args=args)
    node = G2InspectionEvaluator()
    try:
        passed = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not passed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
