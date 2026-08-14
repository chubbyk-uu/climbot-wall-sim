#!/usr/bin/env python3
"""Execute compact coverage cases and evaluate each dynamic line with truth."""

import copy
import csv
import json
import math
import os
import time

from ament_index_python.packages import get_package_share_directory
from climbot_description.geometry import quaternion_tuple
from climbot_description.geometry import yaw_from_quaternion
from climbot_description.wall_frame import WallFrame
from climbot_gazebo.coverage_metrics import footprint_coverage
from climbot_interfaces.action import ExecuteCoverage
from climbot_interfaces.msg import CoverageTask
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


def make_pose(x, y, yaw):
    """Build a planar pose."""
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def make_point(x, y):
    """Build a polygon point."""
    point = Point32()
    point.x = x
    point.y = y
    return point


class CoverageExecutionEvaluator(Node):
    """Send one compact task and score TRACK_LINE samples against truth."""

    def __init__(self):
        super().__init__('coverage_execution_evaluator')
        self.declare_parameter('case', 'short_top_trapezoid')
        self.declare_parameter('startup_timeout_s', 10.0)
        self.declare_parameter('execution_timeout_s', 120.0)
        self.declare_parameter('maximum_cross_rms_m', 0.020)
        self.declare_parameter('maximum_cross_error_m', 0.050)
        self.declare_parameter('visible_excursion_m', 0.020)
        self.declare_parameter('maximum_visible_reversals', 0)
        self.declare_parameter('maximum_heading_range_deg', 5.0)
        self.declare_parameter('minimum_actual_coverage_ratio', 0.98)
        self.declare_parameter('coverage_grid_resolution_m', 0.01)
        self.declare_parameter('trajectory_csv', '')
        self.declare_parameter('summary_json', '')
        wall_path = os.path.join(
            get_package_share_directory('climbot_description'),
            'config', 'wall.yaml')
        self.wall = WallFrame.from_yaml(wall_path)
        self.filtered = None
        self.planned_task = None
        self.reference = None
        self.executed_task = None
        self.segment = -1
        self.state = ExecuteCoverage.Feedback.WAITING
        self.samples = {}
        self.references = []
        self.trajectory = []
        self.recording = False
        self.create_subscription(
            Odometry, '/odometry/filtered', self._filtered_callback, 10)
        self.create_subscription(
            Odometry, '/model/climbot/ground_truth', self._truth_callback, 10)
        reference_qos = QoSProfile(depth=1)
        reference_qos.reliability = ReliabilityPolicy.RELIABLE
        reference_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Path, '/control/reference_path', self._reference_callback,
            reference_qos)
        self.create_subscription(
            CoverageTask, '/coverage/task', self._task_callback,
            reference_qos)
        self.client = ActionClient(
            self, ExecuteCoverage, '/coverage/execute')

    def _filtered_callback(self, message):
        self.filtered = message

    def _task_callback(self, message):
        self.planned_task = message

    def _feedback_callback(self, message):
        self.segment = message.feedback.current_segment
        self.state = message.feedback.state

    def _reference_callback(self, message):
        if len(message.poses) < 2:
            return
        first = message.poses[0].pose.position
        last = message.poses[-1].pose.position
        self.reference = (first.x, first.y, last.x, last.y)
        self.references.append(self.reference)

    def _truth_callback(self, message):
        if not self.recording or self.executed_task is None:
            return
        position = message.pose.pose.position
        x, y, _ = self.wall.position_from_world(
            (position.x, position.y, position.z))
        orientation = self.wall.orientation_from_world(
            quaternion_tuple(message.pose.pose.orientation))
        yaw = yaw_from_quaternion(orientation)
        filtered_x = math.nan
        filtered_y = math.nan
        filtered_yaw = math.nan
        if self.filtered is not None:
            filtered_pose = self.filtered.pose.pose
            filtered_x = filtered_pose.position.x
            filtered_y = filtered_pose.position.y
            filtered_yaw = yaw_from_quaternion(
                quaternion_tuple(filtered_pose.orientation))

        reference = self.reference or (math.nan,) * 4
        cross_track = math.nan
        scored = False
        valid_segment = 0 <= self.segment < len(
            self.executed_task.segment_types)
        tracking = self.state in (
            ExecuteCoverage.Feedback.TRACK_LINE,
            ExecuteCoverage.Feedback.FINAL_APPROACH)
        if self.reference is not None and valid_segment and tracking:
            first_x, first_y, last_x, last_y = self.reference
            delta_x = last_x - first_x
            delta_y = last_y - first_y
            length = math.hypot(delta_x, delta_y)
            if length > 1e-9:
                nominal_first = self.executed_task.waypoints[
                    self.segment].position
                nominal_last = self.executed_task.waypoints[
                    self.segment + 1].position
                nominal_heading = math.atan2(
                    nominal_last.y - nominal_first.y,
                    nominal_last.x - nominal_first.x)
                reference_heading = math.atan2(delta_y, delta_x)
                heading_difference = math.atan2(
                    math.sin(reference_heading - nominal_heading),
                    math.cos(reference_heading - nominal_heading))
                # Scan-entry arcs are not accepted scan-line samples.
                parallel_scan = not (
                    self.executed_task.segment_types[self.segment] ==
                    CoverageTask.SEGMENT_SCAN and
                    abs(heading_difference) > math.radians(2.0))
                if parallel_scan:
                    cross_track = (
                        -(x - first_x) * delta_y +
                        (y - first_y) * delta_x) / length
                    self.samples.setdefault(self.segment, []).append(
                        (cross_track, yaw, x, y))
                    scored = True

        stamp = message.header.stamp
        self.trajectory.append({
            'time_s': stamp.sec + stamp.nanosec * 1e-9,
            'segment': self.segment,
            'segment_type': (
                self.executed_task.segment_types[self.segment]
                if valid_segment else 0),
            'state': self.state,
            'truth_x_m': x,
            'truth_y_m': y,
            'truth_yaw_rad': yaw,
            'filtered_x_m': filtered_x,
            'filtered_y_m': filtered_y,
            'filtered_yaw_rad': filtered_yaw,
            'reference_start_x_m': reference[0],
            'reference_start_y_m': reference[1],
            'reference_end_x_m': reference[2],
            'reference_end_y_m': reference[3],
            'cross_track_error_m': cross_track,
            'scored_line_sample': int(scored),
        })

    @staticmethod
    def _motion_region():
        return [
            make_point(-4.6, 0.4), make_point(4.6, 0.4),
            make_point(4.6, 7.6), make_point(-4.6, 7.6)]

    def _vertical_rectangle(self, x, y):
        task = CoverageTask()
        task.task_id = 'evaluation-vertical-rectangle'
        task.sweep_direction = CoverageTask.SWEEP_VERTICAL
        task.waypoints = [
            make_pose(x, y, math.pi / 2.0),
            make_pose(x, y + 0.6, 0.0),
            make_pose(x + 0.4, y + 0.6, -math.pi / 2.0),
            make_pose(x + 0.4, y, -math.pi / 2.0),
        ]
        task.segment_types = [
            CoverageTask.SEGMENT_SCAN,
            CoverageTask.SEGMENT_TRANSITION,
            CoverageTask.SEGMENT_SCAN,
        ]
        task.coverage_region.points = [
            make_point(x - 0.1, y),
            make_point(x + 0.5, y),
            make_point(x + 0.5, y + 0.6),
            make_point(x - 0.1, y + 0.6),
        ]
        task.detection_width = 0.5
        task.detection_length = 0.1
        return task

    def _short_top_trapezoid(self, x, y):
        # Isosceles trapezoid: bottom 1.2 m, top 0.4 m, height 0.8 m.
        points = [
            (x, y), (x + 1.2, y),
            (x + 1.0, y + 0.4), (x + 0.2, y + 0.4),
            (x + 0.4, y + 0.8), (x + 0.8, y + 0.8),
        ]
        task = CoverageTask()
        task.task_id = 'evaluation-short-top-trapezoid'
        task.sweep_direction = CoverageTask.SWEEP_HORIZONTAL
        for first, second in zip(points, points[1:]):
            yaw = math.atan2(second[1] - first[1], second[0] - first[0])
            task.waypoints.append(make_pose(*first, yaw))
        task.waypoints.append(make_pose(*points[-1], 0.0))
        task.segment_types = [
            CoverageTask.SEGMENT_SCAN,
            CoverageTask.SEGMENT_TRANSITION,
            CoverageTask.SEGMENT_SCAN,
            CoverageTask.SEGMENT_TRANSITION,
            CoverageTask.SEGMENT_SCAN,
        ]
        task.coverage_region.points = [
            make_point(x, y), make_point(x + 1.2, y),
            make_point(x + 0.8, y + 0.8),
            make_point(x + 0.4, y + 0.8),
        ]
        task.detection_width = 0.4
        task.detection_length = 0.1
        return task

    def _make_task(self):
        case_name = str(self.get_parameter('case').value)
        if case_name == 'planned_task':
            if self.planned_task is None:
                raise RuntimeError('No latched /coverage/task is available.')
            return copy.deepcopy(self.planned_task)
        position = self.filtered.pose.pose.position
        if case_name == 'vertical_rectangle':
            task = self._vertical_rectangle(position.x, position.y)
        elif case_name == 'short_top_trapezoid':
            task = self._short_top_trapezoid(position.x, position.y)
        else:
            raise ValueError(
                'case must be planned_task, vertical_rectangle, or short_top_trapezoid')
        task.header.frame_id = 'odom'
        task.header.stamp = self.get_clock().now().to_msg()
        task.revision = 1
        task.motion_region.points = self._motion_region()
        return task

    @staticmethod
    def _unwrap(values):
        unwrapped = [values[0]]
        for value in values[1:]:
            difference = math.atan2(
                math.sin(value - unwrapped[-1]),
                math.cos(value - unwrapped[-1]))
            unwrapped.append(unwrapped[-1] + difference)
        return unwrapped

    def _metrics(self, segment, values):
        cross = [value[0] for value in values]
        yaw = self._unwrap([value[1] for value in values])
        excursion = float(self.get_parameter('visible_excursion_m').value)
        signs = []
        for error in cross:
            sign = 1 if error > excursion else -1 if error < -excursion else 0
            if sign and (not signs or sign != signs[-1]):
                signs.append(sign)
        return {
            'segment': segment,
            'samples': len(values),
            'rms': math.sqrt(sum(value * value for value in cross) / len(cross)),
            'maximum': max(abs(value) for value in cross),
            'reversals': max(0, len(signs) - 1),
            'heading_range_deg': math.degrees(max(yaw) - min(yaw)),
        }

    @staticmethod
    def _write_csv(path, rows):
        """Write the full task trajectory when an output path is configured."""
        if not path:
            return
        expanded = os.path.abspath(os.path.expanduser(path))
        os.makedirs(os.path.dirname(expanded), exist_ok=True)
        fields = list(rows[0].keys()) if rows else [
            'time_s', 'segment', 'segment_type', 'state']
        with open(expanded, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(
                handle, fieldnames=fields, lineterminator='\n')
            writer.writeheader()
            writer.writerows(rows)

    @staticmethod
    def _write_json(path, values):
        """Write a machine-readable acceptance summary when configured."""
        if not path:
            return
        expanded = os.path.abspath(os.path.expanduser(path))
        os.makedirs(os.path.dirname(expanded), exist_ok=True)
        with open(expanded, 'w', encoding='utf-8') as handle:
            json.dump(values, handle, indent=2, sort_keys=True)
            handle.write('\n')

    def run(self):
        """Execute the selected case and return true only if every line passes."""
        startup_timeout = float(self.get_parameter('startup_timeout_s').value)
        deadline = time.monotonic() + startup_timeout
        case_name = str(self.get_parameter('case').value)
        while (self.filtered is None or
               (case_name == 'planned_task' and self.planned_task is None)) and \
                time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if (self.filtered is None or
                (case_name == 'planned_task' and self.planned_task is None) or
                not self.client.wait_for_server(
                    timeout_sec=startup_timeout)):
            raise RuntimeError('Localization or /coverage/execute is unavailable.')

        goal = ExecuteCoverage.Goal()
        goal.task = self._make_task()
        self.executed_task = goal.task
        self.samples = {
            index: [] for index in range(len(goal.task.segment_types))}
        self.trajectory = []
        future = self.client.send_goal_async(
            goal, feedback_callback=self._feedback_callback)
        rclpy.spin_until_future_complete(self, future, timeout_sec=startup_timeout)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('Coverage evaluation goal was rejected.')
        self.recording = True
        result_future = handle.get_result_async()
        deadline = time.monotonic() + float(
            self.get_parameter('execution_timeout_s').value)
        while not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not result_future.done():
            handle.cancel_goal_async()
            self.recording = False
            raise RuntimeError('Coverage evaluation timed out.')
        self.recording = False
        wrapped = result_future.result()
        result = wrapped.result
        self.get_logger().info(
            'Action result=%d completed=%d elapsed=%.3f s: %s' % (
                result.result_code, result.completed_segments,
                result.elapsed_time_s, result.message))
        passed = result.result_code == ExecuteCoverage.Result.SUCCESS
        limits = (
            float(self.get_parameter('maximum_cross_rms_m').value),
            float(self.get_parameter('maximum_cross_error_m').value),
            int(self.get_parameter('maximum_visible_reversals').value),
            float(self.get_parameter('maximum_heading_range_deg').value),
        )
        segment_metrics = []
        for segment, values in self.samples.items():
            if not values:
                self.get_logger().error('Segment %d has no truth samples.' % segment)
                passed = False
                continue
            metrics = self._metrics(segment, values)
            segment_metrics.append(metrics)
            self.get_logger().info(
                'segment=%d samples=%d cross_rms=%.2f mm cross_max=%.2f mm '
                'visible_reversals=%d heading_range=%.2f deg' % (
                    segment, metrics['samples'], metrics['rms'] * 1000.0,
                    metrics['maximum'] * 1000.0, metrics['reversals'],
                    metrics['heading_range_deg']))
            passed = passed and (
                metrics['rms'] <= limits[0] and
                metrics['maximum'] <= limits[1] and
                metrics['reversals'] <= limits[2] and
                metrics['heading_range_deg'] <= limits[3])

        polygon = [
            (point.x, point.y) for point in goal.task.coverage_region.points]
        scan_paths = []
        for segment, segment_type in enumerate(goal.task.segment_types):
            if segment_type != CoverageTask.SEGMENT_SCAN:
                continue
            values = self.samples.get(segment, [])
            if values:
                scan_paths.append([
                    (value[2], value[3], value[1]) for value in values])
        coverage = footprint_coverage(
            polygon, scan_paths, goal.task.detection_width,
            goal.task.detection_length,
            float(self.get_parameter('coverage_grid_resolution_m').value))
        minimum_coverage = float(
            self.get_parameter('minimum_actual_coverage_ratio').value)
        self.get_logger().info(
            'actual_coverage=%.3f%% missed=%.3f%% covered=%.4f/%.4f m^2 '
            'grid=%.1f mm' % (
                coverage['ratio'] * 100.0,
                coverage['missed_ratio'] * 100.0,
                coverage['covered_area_m2'], coverage['region_area_m2'],
                coverage['resolution_m'] * 1000.0))
        passed = passed and coverage['ratio'] >= minimum_coverage
        self._write_csv(
            str(self.get_parameter('trajectory_csv').value), self.trajectory)
        self._write_json(
            str(self.get_parameter('summary_json').value), {
                'task_id': goal.task.task_id,
                'revision': goal.task.revision,
                'result_code': result.result_code,
                'result_message': result.message,
                'completed_segments': result.completed_segments,
                'elapsed_time_s': result.elapsed_time_s,
                'trajectory_samples': len(self.trajectory),
                'coverage': coverage,
                'segment_metrics': segment_metrics,
                'passed': passed,
            })
        return passed


def main(args=None):
    """Run one truth-based coverage execution case."""
    rclpy.init(args=args)
    node = CoverageExecutionEvaluator()
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
