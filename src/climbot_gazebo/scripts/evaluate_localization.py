#!/usr/bin/env python3
"""Drive a closed-loop four-direction test and report EKF position errors."""

import math
import os
import time

from ament_index_python.packages import get_package_share_directory
from climbot_gazebo.geometry import (
    quaternion_tuple,
    wrap_angle,
    yaw_from_quaternion,
)
from climbot_gazebo.safe_stop import install_stop_on_termination
from climbot_gazebo.wall_frame import WallFrame
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


def default_wall_config():
    """Return the installed wall description, so no path is hardcoded."""
    return os.path.join(
        get_package_share_directory('climbot_gazebo'), 'config', 'wall.yaml')


def stamp_nanoseconds(message):
    """Return a ROS message timestamp in nanoseconds."""
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


class LocalizationEvaluator(Node):
    """Use fused heading closure and compare wheel, EKF, and truth states."""

    def __init__(self):
        super().__init__('localization_evaluator')
        self.declare_parameter('segment_duration_s', 8.0)
        self.declare_parameter('linear_speed_mps', 0.15)
        self.declare_parameter('turn_tolerance_deg', 1.0)
        self.declare_parameter('turn_timeout_s', 25.0)
        self.declare_parameter('settle_duration_s', 1.0)
        self.declare_parameter('heading_hold_gain', 1.5)
        self.declare_parameter('wall_config', default_wall_config())
        self._wall_frame = WallFrame.from_yaml(
            str(self.get_parameter('wall_config').value))
        self._truth = None
        self._wheel_odom = None
        self._filtered = None
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

    def _wait_for_data(self):
        # Wall clock is the only option before the first simulated stamp.
        deadline = time.monotonic() + 30.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._truth and self._wheel_odom and self._filtered:
                return
        raise RuntimeError('Timed out waiting for truth, wheel odometry, and EKF.')

    def _drive_for_sim_time(self, duration_s, linear=0.0, target_yaw=None):
        """Hold a command for a simulated duration, optionally holding heading."""
        target_ns = stamp_nanoseconds(self._truth) + int(duration_s * 1e9)
        gain = float(self.get_parameter('heading_hold_gain').value)
        while rclpy.ok() and stamp_nanoseconds(self._truth) < target_ns:
            rclpy.spin_once(self, timeout_sec=0.02)
            angular = 0.0
            if target_yaw is not None:
                error = wrap_angle(target_yaw - self._filtered_yaw())
                angular = max(-0.35, min(0.35, gain * error))
            self._publish(linear=linear, angular=angular)
        self._publish()

    def _turn_to(self, target_yaw):
        tolerance = math.radians(
            float(self.get_parameter('turn_tolerance_deg').value))
        deadline_ns = stamp_nanoseconds(self._truth) + int(
            float(self.get_parameter('turn_timeout_s').value) * 1e9)
        while rclpy.ok() and stamp_nanoseconds(self._truth) < deadline_ns:
            rclpy.spin_once(self, timeout_sec=0.02)
            # The path-following feedback is the fused IMU + wheel estimate,
            # never the wheel-only yaw.
            error = wrap_angle(target_yaw - self._filtered_yaw())
            if abs(error) <= tolerance:
                self._publish()
                self._drive_for_sim_time(0.5)
                return
            self._publish(angular=max(-0.6, min(0.6, 1.5 * error)))
        self._publish()
        raise RuntimeError('Timed out turning to requested heading.')

    def _drive_segment(self, label, target_yaw):
        self._turn_to(target_yaw)
        # Maintain the desired wall heading for the entire straight-line
        # segment. This corrects force-dependent wall slip during motion.
        self._drive_for_sim_time(
            float(self.get_parameter('segment_duration_s').value),
            linear=float(self.get_parameter('linear_speed_mps').value),
            target_yaw=target_yaw)
        # Let the delayed 12 Hz total-station update reach the EKF.
        self._drive_for_sim_time(
            float(self.get_parameter('settle_duration_s').value))
        self._report(label)

    def _report(self, label):
        truth = self._truth.pose.pose.position
        truth_wall = self._wall_frame.position_from_world(
            (truth.x, truth.y, truth.z))
        estimate = self._filtered.pose.pose.position
        estimate_wall = (estimate.x, estimate.y, estimate.z)
        error = math.dist(truth_wall, estimate_wall)
        truth_yaw = math.degrees(yaw_from_quaternion(
            self._wall_frame.orientation_from_world(
                quaternion_tuple(self._truth.pose.pose.orientation))))
        ekf_yaw = math.degrees(self._filtered_yaw())
        wheel_yaw = math.degrees(yaw_from_quaternion(
            quaternion_tuple(self._wheel_odom.pose.pose.orientation)))
        self.get_logger().info(
            '%s: truth_wall=(%.3f, %.3f, %.3f) '
            'ekf=(%.3f, %.3f, %.3f) error=%.4f m; '
            'yaw_deg truth=%.2f ekf=%.2f wheel=%.2f' % (
                label, *truth_wall, *estimate_wall, error,
                truth_yaw, ekf_yaw, wheel_yaw))

    def run(self):
        self._wait_for_data()
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
    # DiffDrive latches the last command, so a killed run must stop first.
    install_stop_on_termination(evaluator.stop)
    try:
        evaluator.run()
    finally:
        evaluator.stop()
        evaluator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
