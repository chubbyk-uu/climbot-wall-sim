#!/usr/bin/env python3
"""Measure how far an in-place turn slides the robot down the wall."""

# The angular profile is duplicated here in Python on purpose: the guide keeps
# the real controller in C++ and allows Python for experiment tooling, and this
# script has to drive the turn itself to measure what the turn costs.

import csv
import math
import os
import time

from ament_index_python.packages import get_package_share_directory
from climbot_description.geometry import (
    quaternion_tuple,
    wrap_angle,
    yaw_from_quaternion,
)
from climbot_description.wall_frame import WallFrame
from climbot_gazebo.safe_stop import install_stop_on_termination
from climbot_gazebo.turn_slip_model import summarise
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


def default_wall_config():
    """Return the installed wall description, so no path is hardcoded."""
    return os.path.join(
        get_package_share_directory('climbot_description'), 'config', 'wall.yaml')


def stamp_nanoseconds(message):
    """Return a ROS message timestamp in nanoseconds."""
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def plan_turn(delta_rad, max_rate, acceleration):
    """Return the triangular or trapezoidal angular profile for a turn."""
    magnitude = abs(delta_rad)
    sign = 1.0 if delta_rad >= 0.0 else -1.0
    if magnitude >= max_rate * max_rate / acceleration:
        peak = max_rate
        coast = (magnitude - max_rate * max_rate / acceleration) / max_rate
        shape = 'trapezoid'
    else:
        # Too small to reach the rate limit, so the profile never coasts.
        peak = math.sqrt(acceleration * magnitude)
        coast = 0.0
        shape = 'triangle'
    return {
        'sign': sign,
        'peak': peak,
        'ramp': peak / acceleration,
        'coast': coast,
        'acceleration': acceleration,
        'duration': 2.0 * peak / acceleration + coast,
        'shape': shape,
    }


def sample_turn(profile, elapsed):
    """Return the reference angle and feedforward rate at a profile time."""
    ramp = profile['ramp']
    coast = profile['coast']
    peak = profile['peak']
    acceleration = profile['acceleration']
    ramp_angle = 0.5 * peak * ramp
    if elapsed <= 0.0:
        angle, rate = 0.0, 0.0
    elif elapsed < ramp:
        angle = 0.5 * acceleration * elapsed * elapsed
        rate = acceleration * elapsed
    elif elapsed < ramp + coast:
        angle = ramp_angle + peak * (elapsed - ramp)
        rate = peak
    elif elapsed < profile['duration']:
        tail = elapsed - ramp - coast
        angle = ramp_angle + peak * coast + peak * tail - 0.5 * acceleration * tail * tail
        rate = peak - acceleration * tail
    else:
        angle = 2.0 * ramp_angle + peak * coast
        rate = 0.0
    return profile['sign'] * angle, profile['sign'] * rate


class TurnSlipMeasurement(Node):
    """Run the guide's in-place turn test across angles and rate limits."""

    def __init__(self):
        super().__init__('turn_slip_measurement')
        self.declare_parameter('angles_deg', [30.0, 45.0, 90.0, 135.0, 180.0])
        self.declare_parameter('max_rates_rps', [0.3, 0.6])
        self.declare_parameter('angular_acceleration_rps2', 1.5)
        self.declare_parameter('repetitions', 2)
        self.declare_parameter('heading_gain', 1.5)
        self.declare_parameter('heading_tolerance_deg', 1.0)
        self.declare_parameter('converge_timeout_s', 12.0)
        self.declare_parameter('settle_duration_s', 1.0)
        self.declare_parameter('recentre_height_m', 2.0)
        self.declare_parameter('recentre_speed_mps', 0.15)
        self.declare_parameter('wall_config', default_wall_config())
        self.declare_parameter('output_csv', 'results/turn_slip.csv')
        self.declare_parameter('maximum_reference_offset_m', 0.05)

        self._wall_frame = WallFrame.from_yaml(
            str(self.get_parameter('wall_config').value))
        self._records = []
        self._offset_ok = True
        self._truth = None
        self._filtered = None
        self._command = self.create_publisher(Twist, '/control/cmd_vel', 10)
        self.create_subscription(
            Odometry, '/model/climbot/ground_truth', self._truth_callback, 10)
        self.create_subscription(
            Odometry, '/odometry/filtered', self._filtered_callback, 10)

    def _truth_callback(self, message):
        self._truth = message

    def _filtered_callback(self, message):
        self._filtered = message

    def stop(self):
        """Command zero velocity, used on normal and abnormal exit."""
        self._publish()

    def _publish(self, linear=0.0, angular=0.0):
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self._command.publish(command)

    def _filtered_yaw(self):
        return yaw_from_quaternion(
            quaternion_tuple(self._filtered.pose.pose.orientation))

    def _truth_yaw(self):
        return yaw_from_quaternion(self._wall_frame.orientation_from_world(
            quaternion_tuple(self._truth.pose.pose.orientation)))

    def _wall_position(self):
        position = self._truth.pose.pose.position
        return self._wall_frame.position_from_world(
            (position.x, position.y, position.z))

    def _wait_for_data(self):
        # Wall clock is the only option before the first simulated stamp.
        deadline = time.monotonic() + 30.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._truth is not None and self._filtered is not None:
                return
        raise RuntimeError('Timed out waiting for truth and fused odometry.')

    def _hold(self, duration_s, linear=0.0, angular=0.0):
        target = stamp_nanoseconds(self._truth) + int(duration_s * 1e9)
        while rclpy.ok() and stamp_nanoseconds(self._truth) < target:
            rclpy.spin_once(self, timeout_sec=0.02)
            self._publish(linear=linear, angular=angular)
        self._publish()

    def execute_turn(self, delta_deg, max_rate):
        """Drive one profiled turn and return what it cost in wall metres."""
        acceleration = float(
            self.get_parameter('angular_acceleration_rps2').value)
        gain = float(self.get_parameter('heading_gain').value)
        tolerance = math.radians(
            float(self.get_parameter('heading_tolerance_deg').value))
        profile = plan_turn(math.radians(delta_deg), max_rate, acceleration)

        start_yaw = self._filtered_yaw()
        target_yaw = start_yaw + math.radians(delta_deg)
        start_truth_yaw = self._truth_yaw()
        start_position = self._wall_position()
        start_ns = stamp_nanoseconds(self._truth)

        # Feedforward profile with heading feedback, as the guide requires;
        # the line-tracking state may only begin once the error has settled.
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            elapsed = (stamp_nanoseconds(self._truth) - start_ns) * 1e-9
            if elapsed >= profile['duration']:
                break
            reference, feedforward = sample_turn(profile, elapsed)
            error = wrap_angle(start_yaw + reference - self._filtered_yaw())
            self._publish(angular=feedforward + gain * error)
        profile_ns = stamp_nanoseconds(self._truth)

        converge_deadline = profile_ns + int(
            float(self.get_parameter('converge_timeout_s').value) * 1e9)
        converged = False
        while rclpy.ok() and stamp_nanoseconds(self._truth) < converge_deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            error = wrap_angle(target_yaw - self._filtered_yaw())
            if abs(error) <= tolerance:
                converged = True
                break
            self._publish(angular=max(-max_rate, min(max_rate, gain * error)))
        self._publish()
        self._hold(float(self.get_parameter('settle_duration_s').value))

        end_position = self._wall_position()
        shift = (end_position[0] - start_position[0],
                 end_position[1] - start_position[1])
        heading_error = math.degrees(
            wrap_angle(target_yaw - self._filtered_yaw()))
        truth_error = math.degrees(wrap_angle(target_yaw - self._truth_yaw()))
        # The start and end headings are recorded because the reported
        # point sits behind the rotation centre: without them the
        # kinematic swing cannot be separated from the real sliding.
        return {
            'angle_deg': delta_deg,
            'start_heading_deg': math.degrees(start_truth_yaw),
            'end_heading_deg': math.degrees(self._truth_yaw()),
            'max_rate_rps': max_rate,
            'shape': profile['shape'],
            'planned_duration_s': profile['duration'],
            'total_duration_s': (
                stamp_nanoseconds(self._truth) - start_ns) * 1e-9,
            'converged': converged,
            'horizontal_mm': shift[0] * 1000.0,
            'vertical_mm': shift[1] * 1000.0,
            'slide_mm': math.hypot(*shift) * 1000.0,
            'ekf_heading_error_deg': heading_error,
            'truth_heading_error_deg': truth_error,
            'wall_height_m': end_position[1],
        }

    def recentre(self):
        """Drive back up to the start height, using the fused estimate only."""
        target = float(self.get_parameter('recentre_height_m').value)
        speed = float(self.get_parameter('recentre_speed_mps').value)
        gain = float(self.get_parameter('heading_gain').value)
        error = target - self._filtered.pose.pose.position.y
        if abs(error) < 0.05:
            return
        heading = math.pi / 2.0 if error > 0.0 else -math.pi / 2.0
        deadline = stamp_nanoseconds(self._truth) + int(30e9)
        while rclpy.ok() and stamp_nanoseconds(self._truth) < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            yaw_error = wrap_angle(heading - self._filtered_yaw())
            if abs(yaw_error) <= math.radians(1.0):
                break
            self._publish(angular=max(-0.6, min(0.6, gain * yaw_error)))
        while rclpy.ok() and stamp_nanoseconds(self._truth) < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            remaining = target - self._filtered.pose.pose.position.y
            if abs(remaining) < 0.03:
                break
            yaw_error = wrap_angle(heading - self._filtered_yaw())
            self._publish(
                linear=speed,
                angular=max(-0.35, min(0.35, gain * yaw_error)))
        self._publish()
        self._hold(0.5)

    def _write_csv(self):
        path = str(self.get_parameter('output_csv').value)
        if not path or not self._records:
            return
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(path, 'w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(self._records[0]))
            writer.writeheader()
            writer.writerows(self._records)
        self.get_logger().info('Wrote %s' % os.path.abspath(path))

    def run(self):
        self._wait_for_data()
        angles = list(self.get_parameter('angles_deg').value)
        rates = list(self.get_parameter('max_rates_rps').value)
        repetitions = int(self.get_parameter('repetitions').value)

        for rate in rates:
            for angle in angles:
                for repetition in range(repetitions):
                    self.recentre()
                    # Alternate the direction so the heading does not wander
                    # away across a long sweep.
                    signed = angle if repetition % 2 == 0 else -angle
                    record = self.execute_turn(signed, rate)
                    record['repetition'] = repetition + 1
                    self._records.append(record)
                    self.get_logger().info(
                        '%-9s %+6.0f deg @ %.2f rad/s: slide=%6.1f mm '
                        '(h %+7.1f, v %+7.1f)  %.2f s  heading_err=%+.2f deg%s'
                        % (record['shape'], signed, rate, record['slide_mm'],
                           record['horizontal_mm'], record['vertical_mm'],
                           record['total_duration_s'],
                           record['truth_heading_error_deg'],
                           '' if record['converged'] else '  NOT CONVERGED'))
        self._summarise()
        self._write_csv()

    def _summarise(self):
        for rate in sorted({record['max_rate_rps'] for record in self._records}):
            for angle in sorted({abs(record['angle_deg'])
                                 for record in self._records}):
                matching = [record for record in self._records
                            if record['max_rate_rps'] == rate
                            and abs(record['angle_deg']) == angle]
                if not matching:
                    continue
                # Downhill slide is what matters, so it is reported signed.
                mean_vertical = sum(
                    record['vertical_mm'] for record in matching) / len(matching)
                mean_duration = sum(
                    record['total_duration_s'] for record in matching) / len(matching)
                self.get_logger().info(
                    'summary %5.0f deg @ %.2f rad/s: mean_vertical=%+7.1f mm '
                    'over %.2f s (%d runs)' % (
                        angle, rate, mean_vertical, mean_duration, len(matching)))
        self._report_coefficient()

    def _report_coefficient(self):
        """Derive the control parameter this run implies, and self-check it."""
        fit = summarise(self._records)
        offset = fit['reference_offset_magnitude_m']
        self.get_logger().info(
            'fit: turn_slip_per_degree_m=%.5f from %d turns, residual RMS '
            '%.1f mm' % (fit['turn_slip_per_degree_m'], fit['turns'],
                         fit['residual_rms_m'] * 1000.0))
        limit = float(self.get_parameter('maximum_reference_offset_m').value)
        # A reported pose away from the rotation centre swings during the turn,
        # and that swing lands in vertical_mm as if it were sliding. The
        # 2026-08-13 data set was taken 79 mm behind the axle, where the raw
        # per-direction numbers even changed sign, so this is checked rather
        # than assumed. The coefficient itself survives because the sweep
        # covers both directions and the swing cancels in the aggregate slope.
        self.get_logger().info(
            'fit: reported pose sits %.1f mm from the rotation centre '
            '(limit %.1f mm)' % (offset * 1000.0, limit * 1000.0))
        if offset > limit:
            self.get_logger().error(
                'The reported pose is not the rotation centre. The fit removes '
                'that swing, but per-angle and per-direction numbers in the CSV '
                'are not real sliding. Fix the pose reference before reading '
                'them.')
            self._offset_ok = False


def main():
    rclpy.init()
    measurement = TurnSlipMeasurement()
    # DiffDrive latches the last command, so a killed run must stop first.
    install_stop_on_termination(measurement.stop)
    try:
        measurement.run()
    finally:
        measurement.stop()
        measurement.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
