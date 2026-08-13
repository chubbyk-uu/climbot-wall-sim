#!/usr/bin/env python3
"""Measure static, lateral, and longitudinal wall-slip characteristics."""

import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


def wall_position(message):
    """Convert Gazebo world position into (forward, up, normal) wall axes."""
    position = message.pose.pose.position
    return (position.y, position.z, position.x)


def stamp_nanoseconds(message):
    """Return a ROS message timestamp in nanoseconds."""
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    """Return a ROS yaw angle from a unit quaternion."""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2))


class WallSlipCalibrator(Node):
    """Run repeatable truth-based tests without feeding truth to control."""

    def __init__(self):
        super().__init__('wall_slip_calibrator')
        self.declare_parameter('repetitions', 3)
        self.declare_parameter('static_duration_s', 30.0)
        self.declare_parameter('drive_duration_s', 8.0)
        self.declare_parameter('linear_speed_mps', 0.15)
        self.declare_parameter('heading_hold_gain', 1.5)
        self._truth = None
        self._filtered = None
        self._command = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            Odometry, '/model/climbot/ground_truth', self._truth_callback, 10)
        self.create_subscription(
            Odometry, '/odometry/filtered', self._filtered_callback, 10)

    def _truth_callback(self, message):
        self._truth = message

    def _filtered_callback(self, message):
        self._filtered = message

    def _publish(self, linear=0.0, angular=0.0):
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self._command.publish(command)

    def _wait_for_data(self):
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._truth is not None and self._filtered is not None:
                return
        raise RuntimeError('Timed out waiting for truth and fused odometry.')

    def _turn_to(self, target_yaw):
        tolerance = math.radians(1.0)
        deadline = time.monotonic() + 12.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            current = yaw_from_quaternion(self._filtered.pose.pose.orientation)
            error = wrap_angle(target_yaw - current)
            if abs(error) <= tolerance:
                self._publish()
                return
            self._publish(angular=max(-0.6, min(0.6, 1.5 * error)))
        self._publish()
        raise RuntimeError('Timed out turning to requested heading.')

    def _run_for_sim_time(self, duration_s, linear, target_yaw=None):
        start = self._truth
        target_ns = stamp_nanoseconds(start) + int(duration_s * 1e9)
        while rclpy.ok() and stamp_nanoseconds(self._truth) < target_ns:
            rclpy.spin_once(self, timeout_sec=0.02)
            angular = 0.0
            if target_yaw is not None:
                current = yaw_from_quaternion(self._filtered.pose.pose.orientation)
                error = wrap_angle(target_yaw - current)
                angular = max(
                    -0.35,
                    min(0.35, float(self.get_parameter('heading_hold_gain').value)
                        * error))
            self._publish(linear=linear, angular=angular)
        end = self._truth
        self._publish()
        return start, end

    @staticmethod
    def _delta(start, end):
        start_position = wall_position(start)
        end_position = wall_position(end)
        elapsed = (stamp_nanoseconds(end) - stamp_nanoseconds(start)) * 1e-9
        return tuple(end_position[index] - start_position[index] for index in range(3)), elapsed

    def _report_static(self):
        start, end = self._run_for_sim_time(
            float(self.get_parameter('static_duration_s').value), 0.0)
        delta, elapsed = self._delta(start, end)
        self.get_logger().info(
            'static: duration=%.3fs forward=%.4fm up=%.4fm normal=%.4fm' % (
                elapsed, *delta))

    def _report_horizontal(self, repetition):
        self._turn_to(0.0)
        start, end = self._run_for_sim_time(
            float(self.get_parameter('drive_duration_s').value),
            float(self.get_parameter('linear_speed_mps').value))
        delta, elapsed = self._delta(start, end)
        descent_ratio = -delta[1] / delta[0] if delta[0] > 1e-6 else math.nan
        self.get_logger().info(
            'horizontal[%d]: duration=%.3fs forward=%.4fm descent=%.4fm '
            'descent_ratio=%.2f%%' % (
                repetition, elapsed, delta[0], -delta[1], 100.0 * descent_ratio))
        return descent_ratio

    def _report_vertical(self, repetition, target_yaw, label):
        self._turn_to(target_yaw)
        start, end = self._run_for_sim_time(
            float(self.get_parameter('drive_duration_s').value),
            float(self.get_parameter('linear_speed_mps').value), target_yaw)
        delta, elapsed = self._delta(start, end)
        speed = delta[1] / elapsed
        self.get_logger().info(
            '%s[%d]: duration=%.3fs vertical=%.4fm speed=%.5fm/s' % (
                label, repetition, elapsed, delta[1], speed))
        return speed

    def run(self):
        self._wait_for_data()
        self._report_static()
        repetitions = int(self.get_parameter('repetitions').value)
        lateral_ratios = []
        up_speeds = []
        down_speeds = []
        for repetition in range(1, repetitions + 1):
            lateral_ratios.append(self._report_horizontal(repetition))
            up_speeds.append(self._report_vertical(
                repetition, math.pi / 2.0, 'up'))
            down_speeds.append(-self._report_vertical(
                repetition, -math.pi / 2.0, 'down'))
        mean_lateral = sum(lateral_ratios) / len(lateral_ratios)
        mean_up = sum(up_speeds) / len(up_speeds)
        mean_down = sum(down_speeds) / len(down_speeds)
        self.get_logger().info(
            'summary: mean_horizontal_descent_ratio=%.2f%% mean_up_speed=%.5fm/s '
            'mean_down_speed=%.5fm/s down_faster=%.2f%%' % (
                100.0 * mean_lateral, mean_up, mean_down,
                100.0 * (mean_down / mean_up - 1.0)))


def main():
    rclpy.init()
    calibrator = WallSlipCalibrator()
    try:
        calibrator.run()
    finally:
        calibrator._publish()
        calibrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
