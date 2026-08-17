#!/usr/bin/env python3
"""Measure a horizontal line with cross-track compensation off, then on."""

from datetime import datetime
from datetime import timezone
import json
import math
import os
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from climbot_description.geometry import quaternion_tuple
from climbot_description.geometry import wrap_angle
from climbot_description.geometry import yaw_from_quaternion
from climbot_description.wall_frame import WallFrame
from climbot_gazebo.execution_metrics import coefficient_of_variation
from climbot_gazebo.execution_metrics import count_visible_reversals
from climbot_gazebo.safe_stop import install_stop_on_termination
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
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


def default_wall_config():
    """Return the installed wall description, so no path is hardcoded."""
    return os.path.join(
        get_package_share_directory('climbot_description'),
        'config', 'wall.yaml')


def stamp_seconds(message):
    """Return a ROS message timestamp in seconds."""
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


class SlipCompensationEvaluator(Node):
    """Drive equal horizontal lines with and without cross-track control."""

    #: Declared here so a run records the values it was judged against
    #: (PROJECT_GUIDE 12 and 14.6).
    PARAMETERS = (
        ('mode', 'open_loop'),
        ('repetitions', 3),
        ('line_length_m', 1.20),
        ('entry_lead_m', 0.40),
        ('minimum_measured_fraction', 0.95),
        ('linear_speed_mps', 0.15),
        ('heading_hold_gain', 1.5),
        ('settle_duration_s', 3.0),
        ('turn_timeout_s', 30.0),
        ('startup_timeout_s', 30.0),
        ('execution_timeout_s', 300.0),
        ('minimum_height_error_reduction', 0.70),
        ('maximum_open_loop_cv', 0.05),
        ('visible_excursion_m', 0.020),
        ('maximum_visible_reversals', 0),
        ('wall_config', default_wall_config()),
        ('reference_summary_json', ''),
        ('trajectory_csv', ''),
        ('summary_json', ''),
    )

    def __init__(self):
        super().__init__('slip_compensation_evaluator')
        for name, default in self.PARAMETERS:
            self.declare_parameter(name, default)
        self.wall = WallFrame.from_yaml(
            str(self.get_parameter('wall_config').value))
        self.summary = {}
        self.truth = None
        self.filtered = None
        self.reference = None
        self.state = ExecuteCoverage.Feedback.WAITING
        self.segment = -1
        self.trajectory = []
        self.label = None
        self.command = self.create_publisher(Twist, '/control/cmd_vel', 10)
        self.create_subscription(
            Odometry, '/model/climbot/ground_truth', self._truth_callback, 10)
        self.create_subscription(
            Odometry, '/odometry/filtered', self._filtered_callback, 10)
        reference_qos = QoSProfile(depth=1)
        reference_qos.reliability = ReliabilityPolicy.RELIABLE
        reference_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Path, '/control/reference_path', self._reference_callback,
            reference_qos)
        self.client = ActionClient(self, ExecuteCoverage, '/coverage/execute')

    # -- plumbing ---------------------------------------------------------

    def _truth_callback(self, message):
        self.truth = message
        if self.label is None:
            return
        forward, up, _ = self.wall.position_from_world(
            (message.pose.pose.position.x,
             message.pose.pose.position.y,
             message.pose.pose.position.z))
        yaw = yaw_from_quaternion(self.wall.orientation_from_world(
            quaternion_tuple(message.pose.pose.orientation)))
        self.trajectory.append({
            'phase': self.label,
            'time_s': stamp_seconds(message),
            'forward_m': forward,
            'up_m': up,
            'truth_yaw_rad': yaw,
            'filtered_yaw_rad': self._filtered_yaw(),
            'state': self.state,
            'segment': self.segment,
        })

    def _filtered_callback(self, message):
        self.filtered = message

    def _reference_callback(self, message):
        if len(message.poses) < 2:
            return
        first = message.poses[0].pose.position
        last = message.poses[-1].pose.position
        self.reference = (first.x, first.y, last.x, last.y)

    def _feedback_callback(self, message):
        self.state = message.feedback.state
        self.segment = message.feedback.current_segment

    def _filtered_yaw(self):
        if self.filtered is None:
            return math.nan
        return yaw_from_quaternion(
            quaternion_tuple(self.filtered.pose.pose.orientation))

    def stop(self):
        """Command zero velocity, used on normal and abnormal exit."""
        self._publish()

    def _publish(self, linear=0.0, angular=0.0):
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self.command.publish(message)

    def _wait_for_data(self):
        # Wall clock is the only option before the first simulated stamp.
        deadline = time.monotonic() + float(
            self.get_parameter('startup_timeout_s').value)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.truth is not None and self.filtered is not None:
                return
        raise RuntimeError('Timed out waiting for truth and fused odometry.')

    # -- open-loop leg ----------------------------------------------------

    def _turn_to(self, target_yaw):
        """Turn in place on the fused heading, exactly as the calibrator does."""
        tolerance = math.radians(1.0)
        deadline = stamp_seconds(self.truth) + float(
            self.get_parameter('turn_timeout_s').value)
        while rclpy.ok() and stamp_seconds(self.truth) < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            error = wrap_angle(target_yaw - self._filtered_yaw())
            if abs(error) <= tolerance:
                self._publish()
                return
            self._publish(angular=max(-0.6, min(0.6, 1.5 * error)))
        self._publish()
        raise RuntimeError('Timed out turning to the requested heading.')

    def _settle(self):
        deadline = stamp_seconds(self.truth) + float(
            self.get_parameter('settle_duration_s').value)
        while rclpy.ok() and stamp_seconds(self.truth) < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            self._publish()

    def _open_loop_leg(self, repetition):
        """Drive a fixed distance holding heading only, with no cross-track term."""
        # The distance comes from the commanded speed and elapsed simulated
        # time, never from truth: truth must not close any loop (8.1).
        speed = float(self.get_parameter('linear_speed_mps').value)
        duration = float(self.get_parameter('line_length_m').value) / speed
        gain = float(self.get_parameter('heading_hold_gain').value)
        self._turn_to(0.0)
        self._settle()
        self.label = 'open_loop_%d' % repetition
        first = len(self.trajectory)
        deadline = stamp_seconds(self.truth) + duration
        while rclpy.ok() and stamp_seconds(self.truth) < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            error = wrap_angle(0.0 - self._filtered_yaw())
            self._publish(
                linear=speed,
                angular=max(-0.35, min(0.35, gain * error)))
        self._publish()
        samples = self.trajectory[first:]
        self.label = None
        self._settle()
        if len(samples) < 2:
            raise RuntimeError('The open-loop leg recorded no truth samples.')
        return self._leg_metrics(repetition, samples, 0.0)

    # -- compensated leg --------------------------------------------------

    def _compensated_task(self, scans):
        """Build a lead-in transition, an entry scan, and the measured scans."""
        # The legs run opposite to the open-loop ones so the pair fits on the
        # same stretch of wall; gravity slip is downward either way.
        #
        # Turning 180 deg onto the line costs about 85 mm of height, and 10.7
        # carries that offset forward rather than climbing back to nominal, so
        # the first scan starts above parallel_scan_offset_m (45 mm) and the
        # tracker arcs onto the line instead of translating it. The arc eats
        # roughly 0.33 m, which is why the first scan is an entry scan that is
        # executed but not measured. The measured scans follow it with no turn
        # in between, which is the steady state the open-loop legs also run in.
        length = float(self.get_parameter('line_length_m').value)
        lead = float(self.get_parameter('entry_lead_m').value)
        position = self.filtered.pose.pose.position
        start = (position.x, position.y)
        task = CoverageTask()
        task.header.frame_id = 'odom'
        task.header.stamp = self.get_clock().now().to_msg()
        task.task_id = 'slip-compensation'
        task.revision = 1
        task.sweep_direction = CoverageTask.SWEEP_HORIZONTAL
        task.waypoints = [make_pose(start[0], start[1], math.pi)]
        task.segment_types = [CoverageTask.SEGMENT_TRANSITION]
        offset = lead
        task.waypoints.append(
            make_pose(start[0] - offset, start[1], math.pi))
        for _ in range(scans):
            offset += length
            task.waypoints.append(
                make_pose(start[0] - offset, start[1], math.pi))
            task.segment_types.append(CoverageTask.SEGMENT_SCAN)
        end_x = start[0] - offset
        margin = 0.6
        task.coverage_region.points = [
            make_point(end_x, start[1] - margin),
            make_point(start[0], start[1] - margin),
            make_point(start[0], start[1] + margin),
            make_point(end_x, start[1] + margin),
        ]
        task.motion_region.points = [
            make_point(end_x - 1.5, start[1] - 1.5),
            make_point(start[0] + 1.5, start[1] - 1.5),
            make_point(start[0] + 1.5, start[1] + 1.5),
            make_point(end_x - 1.5, start[1] + 1.5),
        ]
        task.detection_width = 0.5
        task.detection_length = 0.01
        return task

    def _run_compensated_task(self, scans):
        """Execute one task of consecutive horizontal scans through the tracker."""
        startup = float(self.get_parameter('startup_timeout_s').value)
        if not self.client.wait_for_server(timeout_sec=startup):
            raise RuntimeError('/coverage/execute is unavailable.')
        goal = ExecuteCoverage.Goal()
        goal.task = self._compensated_task(scans)
        self.state = ExecuteCoverage.Feedback.WAITING
        self.segment = -1
        future = self.client.send_goal_async(
            goal, feedback_callback=self._feedback_callback)
        rclpy.spin_until_future_complete(self, future, timeout_sec=startup)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('The compensated goal was rejected.')
        self.label = 'compensated'
        first = len(self.trajectory)
        result_future = handle.get_result_async()
        deadline = time.monotonic() + float(
            self.get_parameter('execution_timeout_s').value)
        while not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        self.label = None
        if not result_future.done():
            handle.cancel_goal_async()
            raise RuntimeError('The compensated run timed out.')
        result = result_future.result().result
        if result.result_code != ExecuteCoverage.Result.SUCCESS:
            raise RuntimeError(
                'The compensated run failed with code %d: %s' % (
                    result.result_code, result.message))
        return self.trajectory[first:]

    def _measured_scan(self, rows, segment, repetition):
        """Score one tracked scan segment and reject a partly measured one."""
        samples = [
            row for row in rows
            if row['segment'] == segment and row['state'] in (
                ExecuteCoverage.Feedback.TRACK_LINE,
                ExecuteCoverage.Feedback.FINAL_APPROACH)]
        if len(samples) < 2:
            raise RuntimeError(
                'Scan segment %d recorded no tracked samples.' % segment)
        metrics = self._leg_metrics(repetition, samples, math.pi)
        # A leg that measured only part of the line is not comparable to the
        # open-loop leg, and silently averaging it in would understate drift.
        length = float(self.get_parameter('line_length_m').value)
        shortest = length * float(
            self.get_parameter('minimum_measured_fraction').value)
        if metrics['advance_m'] < shortest:
            raise RuntimeError(
                'Scan segment %d measured only %.3f m of the %.3f m line.' % (
                    segment, metrics['advance_m'], length))
        return metrics

    # -- shared metrics ---------------------------------------------------

    def _leg_metrics(self, repetition, samples, line_heading):
        """Score one horizontal leg the way execution_metrics scores a scan."""
        advance = abs(samples[-1]['forward_m'] - samples[0]['forward_m'])
        # Same definition as execution_metrics.horizontal_height_drift: the net
        # change in wall height across a horizontal line.
        height_error = samples[-1]['up_m'] - samples[0]['up_m']
        # The nominal line is horizontal and passes through the leg start, so
        # cross-track error is the height offset itself.
        cross_errors = [row['up_m'] - samples[0]['up_m'] for row in samples]
        offsets = [
            wrap_angle(row['truth_yaw_rad'] - line_heading) for row in samples]
        # Positive means the body points up-slope, which is the compensation
        # 14.4 expects to see.
        signed_offsets = [
            offset if line_heading == 0.0 else -offset for offset in offsets]
        path_angle = math.atan2(height_error, advance) if advance > 1e-6 else 0.0
        return {
            'repetition': repetition,
            'samples': len(samples),
            'advance_m': advance,
            'net_height_error_m': height_error,
            'descent_ratio': (
                -height_error / advance if advance > 1e-6 else math.nan),
            'maximum_absolute_height_error_m': max(
                abs(value) for value in cross_errors),
            'visible_reversals': count_visible_reversals(
                cross_errors, float(
                    self.get_parameter('visible_excursion_m').value)),
            'mean_heading_offset_deg': math.degrees(
                sum(signed_offsets) / len(signed_offsets)),
            'maximum_heading_offset_deg': math.degrees(
                max(abs(value) for value in offsets)),
            'path_angle_deg': math.degrees(path_angle),
        }

    # -- runs -------------------------------------------------------------

    def _run_open_loop(self):
        repetitions = int(self.get_parameter('repetitions').value)
        legs = [self._open_loop_leg(index)
                for index in range(1, repetitions + 1)]
        ratios = [leg['descent_ratio'] for leg in legs]
        heights = [abs(leg['net_height_error_m']) for leg in legs]
        cv = coefficient_of_variation(ratios)
        limit = float(self.get_parameter('maximum_open_loop_cv').value)
        self.summary.update({
            'mode': 'open_loop',
            'legs': legs,
            'mean_descent_ratio': sum(ratios) / len(ratios),
            'descent_ratio_cv': cv,
            'mean_absolute_height_error_m': sum(heights) / len(heights),
            'drift_is_downward': all(
                leg['net_height_error_m'] < 0.0 for leg in legs),
        })
        for leg in legs:
            self.get_logger().info(
                'open_loop[%d]: advance=%.4f m net_height=%+.4f m '
                'ratio=%.2f%% heading=%+.2f deg' % (
                    leg['repetition'], leg['advance_m'],
                    leg['net_height_error_m'], 100.0 * leg['descent_ratio'],
                    leg['mean_heading_offset_deg']))
        self.get_logger().info(
            'open_loop summary: mean_ratio=%.2f%% cv=%.2f%% '
            'mean_height_error=%.2f mm' % (
                100.0 * self.summary['mean_descent_ratio'], 100.0 * cv,
                1000.0 * self.summary['mean_absolute_height_error_m']))
        passed = (
            self.summary['drift_is_downward'] and cv <= limit)
        self.summary['passed'] = passed
        if not self.summary['drift_is_downward']:
            self.get_logger().error(
                'A leg did not drift downward; there is nothing to compensate.')
        if cv > limit:
            self.get_logger().error(
                'Descent-ratio CV %.2f%% exceeds %.2f%%.' % (
                    100.0 * cv, 100.0 * limit))
        return passed

    def _run_compensated(self):
        repetitions = int(self.get_parameter('repetitions').value)
        # Segment 0 is the lead-in transition and segment 1 is the entry scan
        # that absorbs the turn; the measured scans are the ones after it.
        rows = self._run_compensated_task(repetitions + 1)
        legs = [
            self._measured_scan(rows, index + 1, index)
            for index in range(1, repetitions + 1)]
        heights = [abs(leg['net_height_error_m']) for leg in legs]
        reversals = max(leg['visible_reversals'] for leg in legs)
        allowed = int(self.get_parameter('maximum_visible_reversals').value)
        # 14.4 asks the position track to stay closer to the planned line than
        # the body heading does; the up-slope heading is what buys that.
        position_leads = all(
            abs(leg['path_angle_deg']) < abs(leg['mean_heading_offset_deg'])
            for leg in legs)
        self.summary.update({
            'mode': 'compensated',
            'legs': legs,
            'mean_absolute_height_error_m': sum(heights) / len(heights),
            'maximum_absolute_height_error_m': max(
                leg['maximum_absolute_height_error_m'] for leg in legs),
            'maximum_visible_reversals': reversals,
            'mean_heading_offset_deg': sum(
                leg['mean_heading_offset_deg'] for leg in legs) / len(legs),
            'position_closer_than_heading': position_leads,
        })
        for leg in legs:
            self.get_logger().info(
                'compensated[%d]: advance=%.4f m net_height=%+.4f m '
                'heading=%+.2f deg path=%+.3f deg reversals=%d' % (
                    leg['repetition'], leg['advance_m'],
                    leg['net_height_error_m'], leg['mean_heading_offset_deg'],
                    leg['path_angle_deg'], leg['visible_reversals']))
        passed = reversals <= allowed and position_leads
        if reversals > allowed:
            self.get_logger().error(
                'Visible cross-track reversals %d exceed %d.' % (
                    reversals, allowed))
        if not position_leads:
            self.get_logger().error(
                'The body heading tracked the planned line more closely than '
                'the position did, so no compensation is visible.')
        passed = self._compare_with_reference() and passed
        self.summary['passed'] = passed
        return passed

    def _compare_with_reference(self):
        """Apply the 14.4 reduction gate against an open-loop summary."""
        path = str(self.get_parameter('reference_summary_json').value)
        if not path:
            self.get_logger().warn(
                'No reference_summary_json given; the 14.4 reduction '
                'requirement was not evaluated.')
            return True
        with open(path) as handle:
            reference = json.load(handle)
        if reference.get('mode') != 'open_loop':
            raise ValueError('reference_summary_json is not an open-loop run.')
        baseline = float(reference['mean_absolute_height_error_m'])
        if baseline <= 1e-9:
            raise ValueError('The open-loop run recorded no height error.')
        compensated = float(self.summary['mean_absolute_height_error_m'])
        reduction = 1.0 - compensated / baseline
        required = float(
            self.get_parameter('minimum_height_error_reduction').value)
        # Slip shortens the open-loop leg by a few percent, so also report the
        # comparison normalised by distance travelled.
        reference_ratio = abs(float(reference['mean_descent_ratio']))
        compensated_ratio = abs(sum(
            leg['descent_ratio'] for leg in self.summary['legs']) /
            len(self.summary['legs']))
        self.summary['comparison'] = {
            'reference_summary_json': os.path.abspath(path),
            'reference_commit': reference.get(
                'provenance', {}).get('git', {}).get('commit'),
            'open_loop_mean_absolute_height_error_m': baseline,
            'compensated_mean_absolute_height_error_m': compensated,
            'height_error_reduction': reduction,
            'required_height_error_reduction': required,
            'open_loop_mean_descent_ratio': reference_ratio,
            'compensated_mean_descent_ratio': compensated_ratio,
            'descent_ratio_reduction': (
                1.0 - compensated_ratio / reference_ratio
                if reference_ratio > 1e-9 else math.nan),
        }
        self.get_logger().info(
            'compensation: %.2f mm -> %.2f mm, reduction %.2f%% '
            '(requires %.2f%%)' % (
                1000.0 * baseline, 1000.0 * compensated,
                100.0 * reduction, 100.0 * required))
        if reduction < required:
            self.get_logger().error(
                'Height-error reduction %.2f%% is below %.2f%%.' % (
                    100.0 * reduction, 100.0 * required))
            return False
        return True

    # -- provenance and output --------------------------------------------

    @staticmethod
    def _git_state():
        """Describe the source revision, or nulls when git is unavailable."""
        def capture(arguments):
            return subprocess.run(
                ['git'] + arguments, check=True, capture_output=True,
                text=True, timeout=5.0,
                cwd=os.path.dirname(os.path.abspath(__file__))).stdout.strip()

        try:
            root = capture(['rev-parse', '--show-toplevel'])
            return {
                'commit': capture(['rev-parse', 'HEAD']),
                'branch': capture(['rev-parse', '--abbrev-ref', 'HEAD']),
                'source_modified': bool(capture(
                    ['-C', root, 'status', '--porcelain', '--', 'src'])),
            }
        except (OSError, subprocess.SubprocessError):
            return {'commit': None, 'branch': None, 'source_modified': None}

    def _provenance(self):
        return {
            'recorded_utc': datetime.now(timezone.utc).isoformat(),
            'git': self._git_state(),
            'evaluator_parameters': {
                name: self.get_parameter(name).value
                for name, _ in self.PARAMETERS},
        }

    def _write_outputs(self):
        csv_path = str(self.get_parameter('trajectory_csv').value)
        if csv_path and self.trajectory:
            written = write_trajectory(
                csv_path, list(self.trajectory[0]), self.trajectory)
            self.get_logger().info('Wrote %s' % written)
        json_path = str(self.get_parameter('summary_json').value)
        if json_path:
            directory = os.path.dirname(json_path)
            if directory:
                os.makedirs(directory, exist_ok=True)
            with open(json_path, 'w') as handle:
                json.dump(self.summary, handle, indent=2, sort_keys=True)
                handle.write('\n')
            self.get_logger().info('Wrote %s' % os.path.abspath(json_path))

    def run(self):
        """Run the selected mode, writing results even when it fails."""
        mode = str(self.get_parameter('mode').value)
        if mode not in ('open_loop', 'compensated'):
            raise ValueError('mode must be open_loop or compensated.')
        self.summary['mode'] = mode
        self.summary['completed'] = False
        self.summary['passed'] = False
        try:
            self._wait_for_data()
            passed = (
                self._run_open_loop() if mode == 'open_loop'
                else self._run_compensated())
            self.summary['completed'] = True
            return passed
        except Exception as error:  # noqa: BLE001 - recorded, then re-raised
            self.summary['failure_reason'] = '%s: %s' % (
                type(error).__name__, error)
            raise
        finally:
            self.summary['trajectory_samples'] = len(self.trajectory)
            self.summary['provenance'] = self._provenance()
            self._write_outputs()


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


def main():
    rclpy.init()
    evaluator = SlipCompensationEvaluator()
    # DiffDrive latches the last command, so a killed run must stop first.
    install_stop_on_termination(evaluator.stop)
    passed = False
    try:
        passed = evaluator.run()
    finally:
        evaluator.stop()
        evaluator.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()
