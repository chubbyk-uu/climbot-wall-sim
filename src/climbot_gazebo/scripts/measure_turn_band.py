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

"""Sweep in-place turns over start heading, turn angle and turn direction."""

# measure_turn_slip.py answers "what does a turn cost per degree", and it turns
# from wherever the robot happens to be facing. That hid a slip band: at 220 N
# of suction, a turn started 12 to 40 degrees off vertical in the direction
# that keeps lowering the robot's nose slid about 68 mm on the spot whatever
# angle it went on to turn. This script holds the start heading fixed so the
# band shows up.
#
# The band was the drive wheels running out of Coulomb friction, and raising
# the suction to 400 N removed it: the whole heading circle now reads 0.392 to
# 0.440 mm/deg. Re-run this sweep after any change to the suction force, the
# friction coefficients or the robot's mass, and check the circle stays flat.
# See results/band_variants/ and docs/STATUS.md.

import csv
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
from climbot_gazebo.provenance import git_state
from climbot_gazebo.safe_stop import install_stop_on_termination
from climbot_gazebo.turn_slip_model import summarise_turn_map
from geometry_msgs.msg import Twist
from measure_turn_slip import plan_turn, sample_turn, stamp_nanoseconds
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


def default_wall_config():
    """Return the installed wall description, so no path is hardcoded."""
    return os.path.join(
        get_package_share_directory('climbot_description'), 'config', 'wall.yaml')


class TurnBandMeasurement(Node):
    """Turn from a commanded heading, so the start heading is the variable."""

    def __init__(self):
        super().__init__('turn_band_measurement')
        self.declare_parameter(
            'headings_deg', [float(value) for value in range(-180, 180, 15)])
        self.declare_parameter('angles_deg', [30.0, -30.0])
        # The tracker's own alignment profile, so the sweep measures what the
        # coverage run will actually drive.
        self.declare_parameter('angular_acceleration_rps2', 1.0)
        self.declare_parameter('max_rate_rps', 0.6)
        self.declare_parameter('heading_gain', 1.5)
        self.declare_parameter('heading_tolerance_deg', 0.3)
        self.declare_parameter('turn_tolerance_deg', 1.0)
        self.declare_parameter('settle_duration_s', 1.5)
        self.declare_parameter('recentre_height_m', 2.0)
        self.declare_parameter('recentre_speed_mps', 0.15)
        self.declare_parameter('wall_config', default_wall_config())
        self.declare_parameter('output_csv', 'results/turn_map.csv')
        self.declare_parameter('summary_json', '')
        self.declare_parameter('maximum_mm_per_deg', 0.50)
        self.declare_parameter('maximum_range_mm_per_deg', 0.10)
        self.declare_parameter('maximum_turn_error_deg', 2.0)

        self._wall_frame = WallFrame.from_yaml(
            str(self.get_parameter('wall_config').value))
        self._records = []
        self._truth = None
        self._command = self.create_publisher(Twist, '/control/cmd_vel', 10)
        self.create_subscription(
            Odometry, '/model/climbot/ground_truth', self._truth_callback, 10)

    def _truth_callback(self, message):
        self._truth = message

    def stop(self):
        """Command zero velocity, used on normal and abnormal exit."""
        self._publish()

    def _publish(self, linear=0.0, angular=0.0):
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self._command.publish(command)

    def _yaw(self):
        return yaw_from_quaternion(self._wall_frame.orientation_from_world(
            quaternion_tuple(self._truth.pose.pose.orientation)))

    def _position(self):
        position = self._truth.pose.pose.position
        return self._wall_frame.position_from_world(
            (position.x, position.y, position.z))

    def _wait_for_data(self):
        # Wall clock is the only option before the first simulated stamp.
        deadline = time.monotonic() + 30.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._truth is not None:
                return
        raise RuntimeError('Timed out waiting for ground truth.')

    def _hold(self, duration_s):
        target = stamp_nanoseconds(self._truth) + int(duration_s * 1e9)
        while rclpy.ok() and stamp_nanoseconds(self._truth) < target:
            rclpy.spin_once(self, timeout_sec=0.02)
            self._publish()

    def face(self, heading_deg):
        """Turn to an absolute heading under feedback, then let it settle."""
        target = math.radians(heading_deg)
        tolerance = math.radians(
            float(self.get_parameter('heading_tolerance_deg').value))
        rate = float(self.get_parameter('max_rate_rps').value)
        gain = float(self.get_parameter('heading_gain').value)
        deadline = stamp_nanoseconds(self._truth) + int(25e9)
        while rclpy.ok() and stamp_nanoseconds(self._truth) < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            error = wrap_angle(target - self._yaw())
            if abs(error) <= tolerance:
                break
            self._publish(angular=max(-rate, min(rate, gain * error)))
        self._publish()
        self._hold(float(self.get_parameter('settle_duration_s').value))

    def recentre(self):
        """Drive back to the start height, so every turn begins alike."""
        target = float(self.get_parameter('recentre_height_m').value)
        speed = float(self.get_parameter('recentre_speed_mps').value)
        gain = float(self.get_parameter('heading_gain').value)
        deadline = stamp_nanoseconds(self._truth) + int(40e9)
        while rclpy.ok() and stamp_nanoseconds(self._truth) < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            error = target - self._position()[1]
            if abs(error) < 0.03:
                break
            heading = math.pi / 2.0 if error > 0.0 else -math.pi / 2.0
            yaw_error = wrap_angle(heading - self._yaw())
            if abs(yaw_error) > math.radians(3.0):
                self._publish(angular=max(-0.6, min(0.6, gain * yaw_error)))
            else:
                self._publish(
                    linear=speed,
                    angular=max(-0.35, min(0.35, gain * yaw_error)))
        self._publish()
        self._hold(1.0)

    def execute_turn(self, delta_deg):
        """Drive one profiled turn and return what it cost in wall metres."""
        rate = float(self.get_parameter('max_rate_rps').value)
        acceleration = float(
            self.get_parameter('angular_acceleration_rps2').value)
        gain = float(self.get_parameter('heading_gain').value)
        tolerance = math.radians(
            float(self.get_parameter('turn_tolerance_deg').value))
        profile = plan_turn(math.radians(delta_deg), rate, acceleration)

        start_yaw = self._yaw()
        start_position = self._position()
        start_ns = stamp_nanoseconds(self._truth)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            elapsed = (stamp_nanoseconds(self._truth) - start_ns) * 1e-9
            if elapsed >= profile['duration']:
                break
            reference, feedforward = sample_turn(profile, elapsed)
            error = wrap_angle(start_yaw + reference - self._yaw())
            self._publish(angular=feedforward + 2.0 * error)
        target_yaw = start_yaw + math.radians(delta_deg)
        deadline = stamp_nanoseconds(self._truth) + int(12e9)
        while rclpy.ok() and stamp_nanoseconds(self._truth) < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            error = wrap_angle(target_yaw - self._yaw())
            if abs(error) <= tolerance:
                break
            self._publish(angular=max(-rate, min(rate, gain * error)))
        self._publish()
        self._hold(1.0)

        end_position = self._position()
        shift = (end_position[0] - start_position[0],
                 end_position[1] - start_position[1])
        slide = math.hypot(*shift)
        return {
            'start_heading_deg': round(math.degrees(start_yaw), 3),
            'commanded_deg': delta_deg,
            'achieved_deg': round(
                math.degrees(wrap_angle(self._yaw() - start_yaw)), 3),
            'horizontal_mm': round(shift[0] * 1000.0, 3),
            'vertical_mm': round(shift[1] * 1000.0, 3),
            'slide_mm': round(slide * 1000.0, 3),
            'mm_per_deg': round(slide * 1000.0 / abs(delta_deg), 4),
            'wall_height_m': round(end_position[1], 4),
        }

    def _write_csv(self):
        path = str(self.get_parameter('output_csv').value)
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(path, 'w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(self._records[0]))
            writer.writeheader()
            writer.writerows(self._records)
        self.get_logger().info(
            'Wrote %d turns to %s' % (len(self._records), path))

    def _write_summary(self, expected_count):
        summary = summarise_turn_map(
            self._records, expected_count,
            float(self.get_parameter('maximum_mm_per_deg').value),
            float(self.get_parameter('maximum_range_mm_per_deg').value),
            float(self.get_parameter('maximum_turn_error_deg').value))
        summary.update({
            'schema_version': 1,
            'parameters': {
                name: self.get_parameter(name).value
                for name in (
                    'headings_deg', 'angles_deg',
                    'angular_acceleration_rps2', 'max_rate_rps',
                    'heading_tolerance_deg', 'turn_tolerance_deg',
                    'settle_duration_s')
            },
            'records': self._records,
            'provenance': {'git': git_state()},
        })
        path = str(self.get_parameter('summary_json').value)
        if path:
            expanded = os.path.abspath(os.path.expanduser(path))
            os.makedirs(os.path.dirname(expanded), exist_ok=True)
            with open(expanded, 'w') as handle:
                json.dump(summary, handle, indent=2, sort_keys=True, allow_nan=False)
                handle.write('\n')
            self.get_logger().info('Wrote %s' % expanded)
        self.get_logger().info(
            'G1_TURN_MAP_PASS=%s range=%.4f..%.4f mm/deg spread=%.4f' % (
                summary['passed'], summary['minimum_mm_per_deg'],
                summary['maximum_mm_per_deg'], summary['range_mm_per_deg']))
        return summary

    def run(self):
        """Sweep the grid, recentring and re-facing before every turn."""
        self._wait_for_data()
        headings = list(self.get_parameter('headings_deg').value)
        angles = list(self.get_parameter('angles_deg').value)
        for heading in headings:
            for angle in angles:
                self.recentre()
                self.face(heading)
                record = self.execute_turn(angle)
                self._records.append(record)
                self.get_logger().info(
                    'start=%7.2f deg turn=%6.1f deg slide=%6.2f mm '
                    '%.4f mm/deg' % (
                        record['start_heading_deg'], angle,
                        record['slide_mm'], record['mm_per_deg']))
        if self._records:
            self._write_csv()
            summary = self._write_summary(len(headings) * len(angles))
            if not summary['passed']:
                raise RuntimeError('G1 full-heading turn-slip acceptance failed')


def main():
    """Run the sweep, stopping the robot on any exit path."""
    rclpy.init()
    node = TurnBandMeasurement()
    install_stop_on_termination(node.stop)
    try:
        node.run()
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
