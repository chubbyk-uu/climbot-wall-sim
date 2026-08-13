#!/usr/bin/env python3
"""Drive a closed-loop four-direction test and report EKF position errors."""

import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    """Return the ROS yaw angle from a unit quaternion."""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2))


def quaternion_multiply(first, second):
    """Multiply quaternions stored as (x, y, z, w) tuples."""
    first_x, first_y, first_z, first_w = first
    second_x, second_y, second_z, second_w = second
    return (
        first_w * second_x + first_x * second_w + first_y * second_z - first_z * second_y,
        first_w * second_y - first_x * second_z + first_y * second_w + first_z * second_x,
        first_w * second_z + first_x * second_y - first_y * second_x + first_z * second_w,
        first_w * second_w - first_x * second_x - first_y * second_y - first_z * second_z,
    )


def inverse_unit_quaternion(quaternion):
    """Return the inverse of a normalized geometry_msgs Quaternion."""
    return (-quaternion.x, -quaternion.y, -quaternion.z, quaternion.w)


def quaternion_tuple(quaternion):
    """Return a geometry_msgs Quaternion as an (x, y, z, w) tuple."""
    return (quaternion.x, quaternion.y, quaternion.z, quaternion.w)


def yaw_from_tuple(quaternion):
    """Return yaw from an (x, y, z, w) tuple."""
    x, y, z, w = quaternion
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y ** 2 + z ** 2))


class LocalizationEvaluator(Node):
    """Use fused heading closure and compare wheel, EKF, and truth states."""

    def __init__(self):
        super().__init__('localization_evaluator')
        self.declare_parameter('segment_duration_s', 8.0)
        self.declare_parameter('linear_speed_mps', 0.15)
        self.declare_parameter('turn_tolerance_deg', 1.0)
        self.declare_parameter('heading_hold_gain', 1.5)
        self._truth = None
        self._wheel_odom = None
        self._filtered = None
        self._initial_truth_orientation = None
        self._command = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            Odometry, '/model/climbot/ground_truth', self._truth_callback, 10)
        self.create_subscription(
            Odometry, '/model/climbot/odometry', self._wheel_callback, 10)
        self.create_subscription(
            Odometry, '/odometry/filtered', self._filtered_callback, 10)

    def _truth_callback(self, message):
        self._truth = message

    def _wheel_callback(self, message):
        self._wheel_odom = message

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
            if self._truth and self._wheel_odom and self._filtered:
                return
        raise RuntimeError('Timed out waiting for truth, wheel odometry, and EKF.')

    def _turn_to(self, target_yaw):
        tolerance = math.radians(
            float(self.get_parameter('turn_tolerance_deg').value))
        deadline = time.monotonic() + 12.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            # The path-following feedback is the fused IMU + wheel estimate,
            # never the wheel-only yaw.
            current = yaw_from_quaternion(self._filtered.pose.pose.orientation)
            error = wrap_angle(target_yaw - current)
            if abs(error) <= tolerance:
                self._publish()
                time.sleep(0.5)
                return
            self._publish(angular=max(-0.6, min(0.6, 1.5 * error)))
        self._publish()
        raise RuntimeError('Timed out turning to requested heading.')

    def _drive_segment(self, label, target_yaw):
        self._turn_to(target_yaw)
        duration = float(self.get_parameter('segment_duration_s').value)
        speed = float(self.get_parameter('linear_speed_mps').value)
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            # Maintain the desired wall heading for the entire straight-line
            # segment. This corrects force-dependent wall slip during motion.
            current = yaw_from_quaternion(self._filtered.pose.pose.orientation)
            error = wrap_angle(target_yaw - current)
            angular = max(
                -0.35,
                min(0.35, float(self.get_parameter('heading_hold_gain').value)
                    * error))
            self._publish(linear=speed, angular=angular)
        self._publish()
        time.sleep(1.0)  # Let the delayed 12 Hz total-station update reach EKF.
        self._report(label)

    def _report(self, label):
        # Gazebo world: X is wall-normal, Y is wall-forward, Z is wall-up.
        truth = self._truth.pose.pose.position
        estimate = self._filtered.pose.pose.position
        truth_wall = (truth.y, truth.z, truth.x)
        estimate_wall = (estimate.x, estimate.y, estimate.z)
        error = math.dist(truth_wall, estimate_wall)
        truth_orientation = self._truth.pose.pose.orientation
        truth_relative = quaternion_multiply(
            self._initial_truth_orientation, quaternion_tuple(truth_orientation))
        truth_yaw = math.degrees(yaw_from_tuple(truth_relative))
        ekf_yaw = math.degrees(
            yaw_from_quaternion(self._filtered.pose.pose.orientation))
        wheel_yaw = math.degrees(
            yaw_from_quaternion(self._wheel_odom.pose.pose.orientation))
        self.get_logger().info(
            '%s: truth_wall=(%.3f, %.3f, %.3f) '
            'ekf=(%.3f, %.3f, %.3f) error=%.4f m; '
            'yaw_deg truth=%.2f ekf=%.2f wheel=%.2f' % (
                label, *truth_wall, *estimate_wall, error,
                truth_yaw, ekf_yaw, wheel_yaw))

    def run(self):
        self._wait_for_data()
        self._initial_truth_orientation = inverse_unit_quaternion(
            self._truth.pose.pose.orientation)
        self._report('start')
        for label, yaw in (
                ('right_plus_x', 0.0),
                ('up_plus_y', math.pi / 2.0),
                ('left_minus_x', math.pi),
                ('down_minus_y', -math.pi / 2.0)):
            self._drive_segment(label, yaw)
        self._publish()


def main():
    rclpy.init()
    evaluator = LocalizationEvaluator()
    try:
        evaluator.run()
    finally:
        evaluator._publish()
        evaluator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
