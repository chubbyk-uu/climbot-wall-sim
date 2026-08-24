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

"""Execute compact coverage cases and evaluate each dynamic line with truth."""

import copy
from datetime import datetime
from datetime import timezone
import json
import math
import os
import time

from ament_index_python.packages import get_package_share_directory
from climbot_description.geometry import quaternion_tuple
from climbot_description.geometry import yaw_from_quaternion
from climbot_description.wall_frame import WallFrame
from climbot_gazebo.coverage_metrics import footprint_coverage
from climbot_gazebo.execution_metrics import count_visible_reversals
from climbot_gazebo.execution_metrics import execution_quality
from climbot_gazebo.execution_metrics import scan_line_spacing
from climbot_gazebo.provenance import CONTROL_SOURCES
from climbot_gazebo.provenance import git_state
from climbot_gazebo.provenance import NOISE_SOURCES
from climbot_gazebo.provenance import parameter_groups
from climbot_gazebo.trajectory_io import write_trajectory
from climbot_interfaces.action import ExecuteCoverage
from climbot_interfaces.msg import CoverageTask
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
import yaml


def _number(value, form):
    """Format a metric that may not apply, without inventing a number for it."""
    return 'n/a' if value is None else form % value


def _millimetres(value):
    """Format a metre-valued metric in millimetres, or say it does not apply."""
    return 'n/a' if value is None else '%.2f' % (value * 1000.0)


def _finite_or_none(value):
    """Replace a non-finite float with None, so the record stays valid JSON."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


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

    #: Acceptance thresholds and outputs, declared here so a run can record the
    #: exact values it was judged against (PROJECT_GUIDE 12 and 14.6).
    PARAMETERS = (
        ('case', 'short_top_trapezoid'),
        ('startup_timeout_s', 10.0),
        ('execution_timeout_s', 120.0),
        ('execution_timeout_wall_factor', 4.0),
        ('maximum_cross_rms_m', 0.020),
        ('maximum_cross_error_m', 0.050),
        ('visible_excursion_m', 0.020),
        ('maximum_visible_reversals', 0),
        ('maximum_heading_range_deg', 5.0),
        ('maximum_endpoint_error_m', 0.030),
        ('maximum_turn_end_heading_error_deg', 2.0),
        ('maximum_horizontal_height_drift_m', 0.030),
        ('maximum_scan_line_spacing_error_m', 0.020),
        ('minimum_actual_coverage_ratio', 0.95),
        ('coverage_grid_resolution_m', 0.01),
        ('trajectory_csv', ''),
        ('summary_json', ''),
        # PROJECT_GUIDE 15.7 asks for horizontal, vertical, and diagonal single
        # segments. One parameterised case rather than three fixed ones,
        # because what separates them is a bearing and nothing else, and a
        # bearing nobody can vary would leave 15.7 answered only at the three
        # angles somebody thought of.
        ('straight_line_bearing_deg', 0.0),
        ('straight_line_length_m', 2.0),
        # How far ahead of the robot the line begins. Measured, not guessed: a
        # line that starts exactly where the robot is leaves the start approach
        # nowhere to happen, so the entry arc is taken out of the line itself -
        # 383 mm of it after a 180 degree alignment turn, which is 8.6% of a 4 m
        # line that never gets covered (results/README.md, tag line1). A real
        # task never asks for that; the planner puts the approach before the
        # first scan line. This restores that structure.
        ('straight_line_start_offset_m', 0.6),
        # Which way the start point lies from the robot, as opposed to which
        # way the line then runs. The two are separate because a start approach
        # is only exercised when they differ: an offset along the line bearing
        # puts the robot already pointing at the start, which is the easy case
        # and the only one a single angle can express. NaN means "along the
        # line", so a case that does not set it keeps the simple geometry.
        ('straight_line_approach_bearing_deg', float('nan')),
    )

    def __init__(self):
        super().__init__('coverage_execution_evaluator')
        for name, default in self.PARAMETERS:
            self.declare_parameter(name, default)
        self.summary = {}
        wall_path = os.path.join(
            get_package_share_directory('climbot_description'),
            'config', 'wall.yaml')
        self.wall = WallFrame.from_yaml(wall_path)
        self.motion_region_corners = self._reachable_rectangle(wall_path)
        self.filtered = None
        self.planned_task = None
        self.reference = None
        self.executed_task = None
        self.segment = -1
        self.state = ExecuteCoverage.Feedback.WAITING
        self.planned_total_s = 0.0
        self.schedule_lag_max_s = 0.0
        self.schedule_lag_min_s = 0.0
        self.heading_error = math.nan
        self.command_linear = 0.0
        self.command_angular = 0.0
        self.samples = {}
        self.references = []
        self.trajectory = []
        self.recording = False
        self.create_subscription(
            Odometry, '/odometry/filtered', self._filtered_callback, 10)
        self.create_subscription(
            Odometry, '/model/climbot/ground_truth', self._truth_callback, 10)
        self.create_subscription(
            Twist, '/control/cmd_vel', self._command_callback, 10)
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
        self.heading_error = message.feedback.heading_error
        # planned_total_s is fixed for the run, so the last value seen is the
        # value. The lag is tracked at its extremes: the peak is what the
        # schedule has to absorb, and the signed maximum separately from the
        # magnitude tells a robot that ran late from one that ran early.
        self.planned_total_s = message.feedback.planned_total_s
        self.schedule_lag_max_s = max(
            self.schedule_lag_max_s, message.feedback.schedule_lag_s)
        self.schedule_lag_min_s = min(
            self.schedule_lag_min_s, message.feedback.schedule_lag_s)

    def _command_callback(self, message):
        self.command_linear = message.linear.x
        self.command_angular = message.angular.z

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
            'heading_error_rad': self.heading_error,
            'command_linear_mps': self.command_linear,
            'command_angular_rps': self.command_angular,
            'scored_line_sample': int(scored),
        })

    @staticmethod
    def _reachable_rectangle(wall_path):
        """Derive the rectangle the planner would hand out."""
        # This used to be four literals for a 10 x 8 m wall inset by 0.4 m,
        # which was neither the wall nor the planner's margin:
        # coverage_planner_node insets by 0.5 * hypot(length, width) +
        # edge_clearance. A line case that fits here but not there would only
        # fail once it reached the tracker.
        with open(wall_path) as handle:
            surface = yaml.safe_load(handle)['wall']['surface']
        robot_path = os.path.join(
            get_package_share_directory('climbot_description'),
            'config', 'robot.yaml')
        with open(robot_path) as handle:
            footprint = yaml.safe_load(handle)['robot']['footprint']
        margin = 0.5 * math.hypot(
            float(footprint['length_m']), float(footprint['width_m'])
        ) + float(footprint['edge_clearance_m'])
        return (margin, margin,
                float(surface['width_m']) - margin,
                float(surface['height_m']) - margin)

    def _motion_region(self):
        left, bottom, right, top = self.motion_region_corners
        return [
            make_point(left, bottom), make_point(right, bottom),
            make_point(right, top), make_point(left, top)]

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

    def _straight_line(self, x, y):
        """One scan segment from here, at a bearing in the wall frame."""
        bearing = math.radians(
            float(self.get_parameter('straight_line_bearing_deg').value))
        length = float(self.get_parameter('straight_line_length_m').value)
        if not length > 0.0 or not math.isfinite(length):
            raise ValueError('straight_line_length_m must be positive.')
        along = (math.cos(bearing), math.sin(bearing))
        normal = (-along[1], along[0])

        offset = float(
            self.get_parameter('straight_line_start_offset_m').value)
        if offset < 0.0 or not math.isfinite(offset):
            raise ValueError(
                'straight_line_start_offset_m must be zero or positive.')
        approach_deg = float(
            self.get_parameter('straight_line_approach_bearing_deg').value)
        approach = bearing if math.isnan(approach_deg) else math.radians(
            approach_deg)
        start = (
            x + math.cos(approach) * offset, y + math.sin(approach) * offset)

        task = CoverageTask()
        task.task_id = 'evaluation-straight-line'
        # Only decides how the task is labelled: which metrics apply is decided
        # per segment from the reference heading the run actually had, so a
        # diagonal is not forced into one of the two sweeps.
        task.sweep_direction = (
            CoverageTask.SWEEP_VERTICAL
            if abs(along[1]) > abs(along[0]) else CoverageTask.SWEEP_HORIZONTAL)
        task.waypoints = [
            make_pose(start[0], start[1], bearing),
            make_pose(
                start[0] + along[0] * length,
                start[1] + along[1] * length, bearing)]
        task.segment_types = [CoverageTask.SEGMENT_SCAN]
        task.detection_width = 0.5
        task.detection_length = 0.1

        # The strip the footprint sweeps, pulled in by half a footprint at each
        # end. The footprint centre only ever reaches the endpoints, so the
        # last half footprint of the nominal strip is outside what any correct
        # run could cover; leaving it in would charge the tracker for geometry
        # rather than for tracking.
        inset = task.detection_length / 2.0
        half = task.detection_width / 2.0
        task.coverage_region.points = [
            make_point(
                start[0] + along[0] * distance + normal[0] * half * side,
                start[1] + along[1] * distance + normal[1] * half * side)
            for distance, side in (
                (inset, 1.0), (length - inset, 1.0),
                (length - inset, -1.0), (inset, -1.0))]
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
        elif case_name == 'straight_line':
            task = self._straight_line(position.x, position.y)
        else:
            raise ValueError(
                'case must be planned_task, vertical_rectangle, '
                'short_top_trapezoid, or straight_line')
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
        return {
            'segment': segment,
            'samples': len(values),
            'rms': math.sqrt(sum(value * value for value in cross) / len(cross)),
            'maximum': max(abs(value) for value in cross),
            'reversals': count_visible_reversals(cross, excursion),
            'heading_range_deg': math.degrees(max(yaw) - min(yaw)),
        }

    def _provenance(self):
        """Record what a later reader needs to reproduce or discard this run."""
        return {
            'recorded_utc': datetime.now(timezone.utc).isoformat(),
            'git': git_state(),
            'noise_sources': parameter_groups(self, NOISE_SOURCES),
            'control_parameters': parameter_groups(self, CONTROL_SOURCES),
            'evaluator_parameters': {
                # straight_line_approach_bearing_deg carries NaN as its "along
                # the line" sentinel, which is fine as a parameter and not fine
                # in a record meant to be read by anything but Python.
                name: _finite_or_none(self.get_parameter(name).value)
                for name, _ in self.PARAMETERS},
        }

    @staticmethod
    def _task_record(task):
        """Keep the nominal geometry the executed task was judged against."""
        return {
            'task_id': task.task_id,
            'revision': task.revision,
            'sweep_direction': task.sweep_direction,
            'detection_width_m': task.detection_width,
            'detection_length_m': task.detection_length,
            'segment_types': list(task.segment_types),
            'waypoints': [
                [pose.position.x, pose.position.y] for pose in task.waypoints],
            'coverage_region': [
                [point.x, point.y] for point in task.coverage_region.points],
            'motion_region': [
                [point.x, point.y] for point in task.motion_region.points],
        }

    @staticmethod
    def _write_csv(path, rows):
        """Write the full task trajectory when an output path is configured."""
        if not path:
            return
        fields = list(rows[0].keys()) if rows else [
            'time_s', 'segment', 'segment_type', 'state']
        write_trajectory(path, fields, rows)

    @staticmethod
    def _write_json(path, values):
        """Write a machine-readable acceptance summary when configured."""
        if not path:
            return
        expanded = os.path.abspath(os.path.expanduser(path))
        os.makedirs(os.path.dirname(expanded), exist_ok=True)
        with open(expanded, 'w', encoding='utf-8') as handle:
            # allow_nan=False is the point of the exercise: Python's default
            # writes the bare tokens NaN and Infinity, which are not JSON, and
            # a summary that only Python can read is not machine-readable
            # acceptance evidence. Failing here is better than writing one -
            # anything non-finite that reaches this is a metric that has not
            # decided what it means yet.
            json.dump(values, handle, indent=2, sort_keys=True, allow_nan=False)
            handle.write('\n')

    def run(self):
        """Execute the selected case, persisting whatever was captured."""
        # A timed-out or aborted run is exactly when the trajectory is most
        # needed for diagnosis, so the outputs are written from a finally block
        # and the summary states whether the run reached its own conclusion.
        self.summary = {'completed': False, 'passed': False}
        try:
            passed = self.summary['passed'] = self._execute()
            self.summary['completed'] = True
            return passed
        except Exception as error:
            self.summary['failure_reason'] = '%s: %s' % (
                type(error).__name__, error)
            raise
        finally:
            self.recording = False
            self.summary['trajectory_samples'] = len(self.trajectory)
            self.summary['provenance'] = self._provenance()
            # Said out loud, at the end, where whoever is watching the run
            # sees it. Buried three levels into a JSON file it went unread
            # through eighteen archived tags.
            git = self.summary['provenance']['git']
            if git.get('traceable'):
                self.get_logger().info(
                    'traceable=true commit=%s' % (git['commit'] or '?')[:12])
            else:
                self.get_logger().warning(
                    'traceable=FALSE commit=%s: the working tree under src '
                    'differs from that commit, so this result cannot be tied '
                    'to a source state and must not be filed as a baseline.'
                    % (git.get('commit') or '?')[:12])
            # Same reasoning as the line above: a repeatability comparison is
            # only about seeds if the seeds are known, and a missing answer
            # here means the noise source was not running to be asked.
            for name, values in self.summary['provenance']['noise_sources'].items():
                self.get_logger().info(
                    '%s seed=%s' % (
                        name,
                        'UNKNOWN' if values is None else values['random_seed']))
            self._write_csv(
                str(self.get_parameter('trajectory_csv').value), self.trajectory)
            self._write_json(
                str(self.get_parameter('summary_json').value), self.summary)

    def _execute(self):
        """Run the selected case and return true only if every line passes."""
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
        self.summary['task'] = self._task_record(goal.task)
        self.recording = True
        result_future = handle.get_result_async()
        # Budgeted in the robot's own seconds, not the operator's. Every task
        # this judges is timed against a simulated clock, and the regression
        # script sets 900 s meaning 900 s of task time; measuring it on a wall
        # clock instead makes the real budget depend on whatever real-time
        # factor the machine happened to sustain, which moved the acceptance
        # boundary between a quiet run and eight parallel lanes.
        timeout = float(self.get_parameter('execution_timeout_s').value)
        deadline = self.get_clock().now() + Duration(seconds=timeout)
        # A stopped simulation clock never reaches that deadline, and this
        # process would then wait for a robot that cannot move. The wall clock
        # only backstops that case, so its factor is deliberately loose enough
        # never to be the limit that fires on a merely slow machine.
        wall_factor = float(self.get_parameter('execution_timeout_wall_factor').value)
        wall_deadline = time.monotonic() + timeout * wall_factor
        while not result_future.done() and time.monotonic() < wall_deadline:
            if self.get_clock().now() >= deadline:
                break
            rclpy.spin_once(self, timeout_sec=0.02)
        if not result_future.done():
            handle.cancel_goal_async()
            self.recording = False
            raise RuntimeError(
                'Coverage evaluation timed out after %.0f s of task time '
                '(wall backstop %.0f s).' % (timeout, timeout * wall_factor))
        self.recording = False
        wrapped = result_future.result()
        result = wrapped.result
        self.summary.update({
            'task_id': goal.task.task_id,
            'revision': goal.task.revision,
            'result_code': result.result_code,
            'result_message': result.message,
            'completed_segments': result.completed_segments,
            'elapsed_time_s': result.elapsed_time_s,
        })
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

        self.summary['segment_metrics'] = segment_metrics
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
            float(self.get_parameter('coverage_grid_resolution_m').value),
            goal.task.detection_forward_offset)
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
        self.summary['coverage'] = coverage
        planned_lengths = []
        for first, second in zip(goal.task.waypoints, goal.task.waypoints[1:]):
            planned_lengths.append(math.hypot(
                second.position.x - first.position.x,
                second.position.y - first.position.y))
        quality = execution_quality(
            self.trajectory, goal.task.segment_types, planned_lengths,
            CoverageTask.SEGMENT_SCAN)
        self.summary['execution_quality'] = quality
        spacing = scan_line_spacing(
            self.trajectory, goal.task.segment_types,
            [(pose.position.x, pose.position.y)
             for pose in goal.task.waypoints],
            CoverageTask.SEGMENT_SCAN)
        self.summary['scan_line_spacing'] = spacing
        # What the schedule predicted against what the run took. The ratio is
        # the number the duration model is calibrated on; the lag bounds say
        # how much of the difference the controller was already correcting for
        # while it ran.
        self.summary['schedule'] = {
            'planned_total_s': self.planned_total_s,
            'schedule_lag_max_s': self.schedule_lag_max_s,
            'schedule_lag_min_s': self.schedule_lag_min_s,
        }
        self.get_logger().info(
            'scan_line_offset_max=%s mm spacing_error_max=%s mm '
            'offsets=[%s] mm' % (
                _millimetres(spacing['maximum_scan_line_offset_m']),
                _millimetres(spacing['maximum_scan_line_spacing_error_m']),
                ' '.join('%+.1f' % (value * 1000.0)
                         for value in spacing['scan_line_offsets_m'])))
        maximum_spacing_error = float(
            self.get_parameter('maximum_scan_line_spacing_error_m').value)
        # "Does not apply" and "was not measured" are different facts about a
        # run, and both used to be spelled NaN. Here it is the first: spacing
        # between adjacent lines is not a property of a task with fewer than two
        # parallel scan lines, and the metric says so itself now. The endpoint
        # check below is the second, and it is a failure - a run that produced
        # no evidence. Relying on NaN comparison semantics to tell those apart
        # read as an accident even while it worked.
        spacing_error = spacing['maximum_scan_line_spacing_error_m']
        spacing_applies = spacing['applicable']
        passed = passed and (
            not spacing_applies or spacing_error <= maximum_spacing_error)
        if not quality['segments']:
            self.get_logger().error(
                'No segment was measured, so no endpoint, heading or drift '
                'evidence exists for this run.')
            passed = False
        horizontal_drift = quality['maximum_horizontal_height_drift_m']
        endpoint_error = quality['maximum_endpoint_error_m']
        turn_end_error = quality['maximum_turn_end_heading_error_deg']
        self.get_logger().info(
            'endpoint_max=%s mm turn_end_max=%s deg '
            'horizontal_drift_max=%s mm path_ratio=%.4f '
            'heading_compensation_max=%s deg tracking_angular_max=%s rad/s' % (
                _millimetres(endpoint_error),
                _number(turn_end_error, '%.2f'),
                _millimetres(horizontal_drift),
                quality['actual_to_planned_length_ratio'],
                _number(quality['maximum_heading_compensation_deg'], '%.2f'),
                _number(quality['maximum_tracking_angular_speed_rps'], '%.3f')))
        # None is not one answer here. No endpoint at all means no segment was
        # measured, which the check above has already failed the run for; no
        # turn means an ordinary single-line task, which is not a fault.
        passed = passed and (
            endpoint_error is not None and endpoint_error <=
            float(self.get_parameter('maximum_endpoint_error_m').value) and
            (turn_end_error is None or turn_end_error <=
             float(self.get_parameter(
                 'maximum_turn_end_heading_error_deg').value)) and
            (horizontal_drift is None or horizontal_drift <=
             float(self.get_parameter(
                 'maximum_horizontal_height_drift_m').value)))
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
