#!/usr/bin/env python3
"""Measure static, lateral, and longitudinal wall-slip characteristics."""

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


class WallSlipCalibrator(Node):
    """Run repeatable truth-based tests without feeding truth to control."""

    def __init__(self):
        super().__init__('wall_slip_calibrator')
        self.declare_parameter('repetitions', 3)
        self.declare_parameter('static_duration_s', 30.0)
        self.declare_parameter('drive_duration_s', 8.0)
        self.declare_parameter('linear_speed_mps', 0.15)
        self.declare_parameter('heading_hold_gain', 1.5)
        self.declare_parameter('turn_timeout_s', 25.0)
        self.declare_parameter('wall_config', default_wall_config())
        self.declare_parameter('trajectory_csv', '')
        self._wall_frame = WallFrame.from_yaml(
            str(self.get_parameter('wall_config').value))
        self._trajectory = []
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

    def stop(self):
        """Command zero velocity, used on normal and abnormal exit."""
        self._publish()

    def _publish(self, linear=0.0, angular=0.0):
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self._command.publish(command)

    def _wall_position(self, message):
        """Return a truth message position in (forward, up, normal) wall axes."""
        position = message.pose.pose.position
        return self._wall_frame.position_from_world(
            (position.x, position.y, position.z))

    def _filtered_yaw(self):
        return yaw_from_quaternion(
            quaternion_tuple(self._filtered.pose.pose.orientation))

    def _wait_for_data(self):
        # Wall clock is the only option before the first simulated stamp.
        deadline = time.monotonic() + 30.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._truth is not None and self._filtered is not None:
                return
        raise RuntimeError('Timed out waiting for truth and fused odometry.')

    def _turn_to(self, target_yaw):
        tolerance = math.radians(1.0)
        deadline_ns = stamp_nanoseconds(self._truth) + int(
            float(self.get_parameter('turn_timeout_s').value) * 1e9)
        while rclpy.ok() and stamp_nanoseconds(self._truth) < deadline_ns:
            rclpy.spin_once(self, timeout_sec=0.02)
            error = wrap_angle(target_yaw - self._filtered_yaw())
            if abs(error) <= tolerance:
                self._publish()
                return
            self._publish(angular=max(-0.6, min(0.6, 1.5 * error)))
        self._publish()
        raise RuntimeError('Timed out turning to requested heading.')

    def _run_for_sim_time(self, duration_s, linear, target_yaw=None, label=None):
        start = self._truth
        target_ns = stamp_nanoseconds(start) + int(duration_s * 1e9)
        start_ns = stamp_nanoseconds(start)
        start_position = self._wall_position(start)
        while rclpy.ok() and stamp_nanoseconds(self._truth) < target_ns:
            rclpy.spin_once(self, timeout_sec=0.02)
            angular = 0.0
            if target_yaw is not None:
                error = wrap_angle(target_yaw - self._filtered_yaw())
                angular = max(
                    -0.35,
                    min(0.35, float(self.get_parameter('heading_hold_gain').value)
                        * error))
            self._publish(linear=linear, angular=angular)
            if label is not None:
                # Truth path relative to where the phase began, so every
                # repetition can be overlaid on one plot.
                position = self._wall_position(self._truth)
                self._trajectory.append((
                    label,
                    (stamp_nanoseconds(self._truth) - start_ns) * 1e-9,
                    position[0] - start_position[0],
                    position[1] - start_position[1],
                    math.degrees(self._truth_yaw())))
        end = self._truth
        self._publish()
        return start, end

    def _truth_yaw(self):
        """Return the Gazebo truth heading in wall coordinates."""
        return yaw_from_quaternion(self._wall_frame.orientation_from_world(
            quaternion_tuple(self._truth.pose.pose.orientation)))

    def _write_trajectory(self):
        path = str(self.get_parameter('trajectory_csv').value)
        if not path or not self._trajectory:
            return
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(path, 'w', newline='') as handle:
            writer = csv.writer(handle)
            writer.writerow(['phase', 'time_s', 'forward_m', 'up_m', 'yaw_deg'])
            for row in self._trajectory:
                writer.writerow(
                    [row[0]] + ['%.6f' % value for value in row[1:]])
        self.get_logger().info('Wrote %s' % os.path.abspath(path))

    def _delta(self, start, end):
        start_position = self._wall_position(start)
        end_position = self._wall_position(end)
        elapsed = (stamp_nanoseconds(end) - stamp_nanoseconds(start)) * 1e-9
        return tuple(end_position[index] - start_position[index] for index in range(3)), elapsed

    def _report_static(self):
        start, end = self._run_for_sim_time(
            float(self.get_parameter('static_duration_s').value), 0.0,
            label='static')
        delta, elapsed = self._delta(start, end)
        self.get_logger().info(
            'static: duration=%.3fs forward=%.4fm up=%.4fm normal=%.4fm' % (
                elapsed, *delta))

    def _report_horizontal(self, repetition):
        self._turn_to(0.0)
        start, end = self._run_for_sim_time(
            float(self.get_parameter('drive_duration_s').value),
            float(self.get_parameter('linear_speed_mps').value),
            label='horizontal_%d' % repetition)
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
            float(self.get_parameter('linear_speed_mps').value), target_yaw,
            label='%s_%d' % (label, repetition))
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
        self._write_trajectory()


def main():
    rclpy.init()
    calibrator = WallSlipCalibrator()
    # DiffDrive latches the last command, so a killed run must stop first.
    install_stop_on_termination(calibrator.stop)
    try:
        calibrator.run()
    finally:
        calibrator.stop()
        calibrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
