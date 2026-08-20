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

"""
Drive a closed-loop four-direction test and compare EKF against wheel odometry.

PROJECT_GUIDE 14.5 asks that the fused position error be clearly smaller than
the long-run error of wheel odometry alone. That is the whole claim the EKF
exists to support, so both errors are measured against Gazebo truth here and
written to a summary a reader can cite, rather than only logged.
"""

from datetime import datetime
from datetime import timezone
import json
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
from climbot_gazebo.provenance import CONTROL_SOURCES
from climbot_gazebo.provenance import git_state
from climbot_gazebo.provenance import NOISE_SOURCES
from climbot_gazebo.provenance import parameter_groups
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
        self.declare_parameter('summary_json', '')
        self._wall_frame = WallFrame.from_yaml(
            str(self.get_parameter('wall_config').value))
        self._truth = None
        self._wheel_odom = None
        self._filtered = None
        self._records = []
        self._origin_truth = None
        self._origin_wheel = None
        self._command = self.create_publisher(Twist, '/control/cmd_vel', 10)
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
        # Wheel odometry is anchored at the spawn pose, not at the wall origin:
        # its first sample is (0, 0, 0) while truth is at the spawn point. So
        # the comparable quantity is displacement, not position - how far dead
        # reckoning thinks it has travelled against how far it really has. The
        # EKF needs no such treatment because it estimates the wall pose
        # itself, which the start record is here to show: it begins on truth.
        wheel = self._wheel_odom.pose.pose.position
        wheel_wall = (wheel.x, wheel.y, wheel.z)
        if self._origin_truth is None:
            self._origin_truth = truth_wall
            self._origin_wheel = wheel_wall
        truth_moved = tuple(
            value - origin for value, origin in zip(truth_wall, self._origin_truth))
        wheel_moved = tuple(
            value - origin for value, origin in zip(wheel_wall, self._origin_wheel))
        wheel_error = math.dist(truth_moved, wheel_moved)
        truth_yaw = math.degrees(yaw_from_quaternion(
            self._wall_frame.orientation_from_world(
                quaternion_tuple(self._truth.pose.pose.orientation))))
        ekf_yaw = math.degrees(self._filtered_yaw())
        wheel_yaw = math.degrees(yaw_from_quaternion(
            quaternion_tuple(self._wheel_odom.pose.pose.orientation)))
        self._records.append({
            'label': label,
            'truth_wall_m': list(truth_wall),
            'ekf_wall_m': list(estimate_wall),
            'wheel_odom_m': list(wheel_wall),
            'truth_displacement_m': list(truth_moved),
            'wheel_displacement_m': list(wheel_moved),
            'ekf_position_error_m': error,
            'wheel_dead_reckoning_error_m': wheel_error,
            'truth_yaw_deg': truth_yaw,
            'ekf_yaw_deg': ekf_yaw,
            'wheel_yaw_deg': wheel_yaw,
        })
        self.get_logger().info(
            '%s: truth_wall=(%.3f, %.3f, %.3f) '
            'ekf=(%.3f, %.3f, %.3f) error=%.4f m; '
            'wheel_dead_reckoning=(%.3f, %.3f, %.3f) error=%.4f m; '
            'yaw_deg truth=%.2f ekf=%.2f wheel=%.2f' % (
                label, *truth_wall, *estimate_wall, error,
                *wheel_moved, wheel_error,
                truth_yaw, ekf_yaw, wheel_yaw))

    def _summarize(self):
        """State the 14.5 comparison as a number, not as a list to eyeball."""
        # The start record is the frame check, not evidence about drift: both
        # estimates begin on top of truth. Long-run error is what 14.5 asks
        # about, so the comparison is over the four driven legs.
        driven = [record for record in self._records if record['label'] != 'start']
        ekf = [record['ekf_position_error_m'] for record in driven]
        wheel = [record['wheel_dead_reckoning_error_m'] for record in driven]
        summary = {
            'records': self._records,
            'legs': len(driven),
            'maximum_ekf_position_error_m': max(ekf) if ekf else None,
            'maximum_wheel_dead_reckoning_error_m': max(wheel) if wheel else None,
            'final_ekf_position_error_m': ekf[-1] if ekf else None,
            'final_wheel_dead_reckoning_error_m': wheel[-1] if wheel else None,
            'provenance': {
                'recorded_utc': datetime.now(timezone.utc).isoformat(),
                'git': git_state(),
                'noise_sources': parameter_groups(self, NOISE_SOURCES),
                'control_parameters': parameter_groups(self, CONTROL_SOURCES),
            },
        }
        if ekf and wheel and max(ekf) > 0.0:
            summary['wheel_to_ekf_maximum_ratio'] = max(wheel) / max(ekf)
        if ekf and wheel:
            summary['passed'] = max(wheel) > max(ekf)
            self.get_logger().info(
                'max position error: ekf=%.4f m wheel=%.4f m (%.1fx)' % (
                    max(ekf), max(wheel),
                    max(wheel) / max(ekf) if max(ekf) > 0.0 else float('inf')))
        git = summary['provenance']['git']
        if git.get('traceable'):
            self.get_logger().info(
                'traceable=true commit=%s' % (git['commit'] or '?')[:12])
        else:
            self.get_logger().warning(
                'traceable=FALSE commit=%s: the working tree under src differs '
                'from that commit, so this result cannot be tied to a source '
                'state and must not be filed as a baseline.'
                % (git.get('commit') or '?')[:12])
        return summary

    def _write_summary(self):
        path = str(self.get_parameter('summary_json').value)
        summary = self._summarize()
        if not path:
            return
        expanded = os.path.abspath(os.path.expanduser(path))
        os.makedirs(os.path.dirname(expanded), exist_ok=True)
        with open(expanded, 'w', encoding='utf-8') as handle:
            json.dump(summary, handle, indent=2, sort_keys=True)
            handle.write('\n')

    def run(self):
        self._wait_for_data()
        self._report('start')
        try:
            for label, yaw in (
                    ('right_plus_x', 0.0),
                    ('up_plus_y', math.pi / 2.0),
                    ('left_minus_x', math.pi),
                    ('down_minus_y', -math.pi / 2.0)):
                self._drive_segment(label, yaw)
            self._publish()
        finally:
            # Same reasoning as the coverage evaluator: an interrupted run is
            # when the partial record is most worth having.
            self._write_summary()


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
