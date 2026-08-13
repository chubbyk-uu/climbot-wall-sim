#!/usr/bin/env python3
"""Measure the three wall-contact normal loads across the guide's manoeuvres."""

import csv
import math
import os
import time

from climbot_gazebo.geometry import (
    quaternion_tuple,
    wrap_angle,
    yaw_from_quaternion,
)
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts

CONTACT_TOPICS = {
    'left_wheel': '/contact/left_wheel',
    'right_wheel': '/contact/right_wheel',
    'caster': '/contact/caster',
}


def stamp_nanoseconds(message):
    """Return a ROS message timestamp in nanoseconds."""
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def contact_normal_load(message):
    """Sum the force pressing body 1 along each reported contact normal."""
    # Projecting onto the reported normal keeps this independent of how the
    # wall happens to be oriented in the Gazebo world frame.
    total = 0.0
    for contact in message.contacts:
        for index, wrench in enumerate(contact.wrenches):
            if index >= len(contact.normals):
                continue
            force = wrench.body_1_wrench.force
            normal = contact.normals[index]
            total += force.x * normal.x + force.y * normal.y + force.z * normal.z
    return total


class NormalLoadMeasurement(Node):
    """Run the load-distribution test and report per-manoeuvre contact loads."""

    def __init__(self):
        super().__init__('normal_load_measurement')
        self.declare_parameter('static_duration_s', 10.0)
        self.declare_parameter('drive_duration_s', 8.0)
        self.declare_parameter('brake_duration_s', 2.0)
        self.declare_parameter('turn_duration_s', 6.0)
        self.declare_parameter('linear_speed_mps', 0.15)
        self.declare_parameter('angular_speed_rps', 0.6)
        self.declare_parameter('heading_hold_gain', 1.5)
        self.declare_parameter('contact_timeout_s', 0.15)
        self.declare_parameter('turn_timeout_s', 25.0)
        self.declare_parameter('output_csv', 'results/normal_loads.csv')

        self._truth = None
        self._filtered = None
        # Latest load and its arrival time, so a silent sensor reads as
        # zero load instead of holding the last value from before lift-off.
        self._loads = {name: 0.0 for name in CONTACT_TOPICS}
        self._load_stamps = {name: None for name in CONTACT_TOPICS}
        self._samples = {}

        self._command = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            Odometry, '/model/climbot/ground_truth', self._truth_callback, 10)
        self.create_subscription(
            Odometry, '/odometry/filtered', self._filtered_callback, 10)
        for name, topic in CONTACT_TOPICS.items():
            self.create_subscription(
                Contacts, topic,
                lambda message, key=name: self._contact_callback(key, message), 20)

    def _truth_callback(self, message):
        self._truth = message

    def _filtered_callback(self, message):
        self._filtered = message

    def _contact_callback(self, name, message):
        self._loads[name] = contact_normal_load(message)
        self._load_stamps[name] = stamp_nanoseconds(message)

    def _publish(self, linear=0.0, angular=0.0):
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self._command.publish(command)

    def _filtered_yaw(self):
        return yaw_from_quaternion(
            quaternion_tuple(self._filtered.pose.pose.orientation))

    def _current_loads(self):
        """Return the three loads, treating a stale sensor as no contact."""
        timeout_ns = int(
            float(self.get_parameter('contact_timeout_s').value) * 1e9)
        now = stamp_nanoseconds(self._truth)
        loads = {}
        for name in CONTACT_TOPICS:
            stamp = self._load_stamps[name]
            fresh = stamp is not None and now - stamp <= timeout_ns
            loads[name] = self._loads[name] if fresh else 0.0
        return loads

    def _wait_for_data(self):
        # Wall clock is the only option before the first simulated stamp.
        deadline = time.monotonic() + 30.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._truth is not None and self._filtered is not None:
                if all(stamp is not None for stamp in self._load_stamps.values()):
                    return
        raise RuntimeError('Timed out waiting for truth, EKF, and contact sensors.')

    def _turn_to(self, target_yaw):
        # Simulated time, so the result does not depend on the real-time factor.
        tolerance = math.radians(1.0)
        deadline_ns = stamp_nanoseconds(self._truth) + int(
            float(self.get_parameter('turn_timeout_s').value) * 1e9)
        while rclpy.ok() and stamp_nanoseconds(self._truth) < deadline_ns:
            rclpy.spin_once(self, timeout_sec=0.02)
            current = self._filtered_yaw()
            error = wrap_angle(target_yaw - current)
            if abs(error) <= tolerance:
                self._publish()
                return
            self._publish(angular=max(-0.6, min(0.6, 1.5 * error)))
        self._publish()
        raise RuntimeError('Timed out turning to requested heading.')

    def _record(self, label, duration_s, linear, angular=0.0, target_yaw=None):
        """Drive for a simulated duration while sampling the contact loads."""
        target_ns = stamp_nanoseconds(self._truth) + int(duration_s * 1e9)
        samples = {name: [] for name in CONTACT_TOPICS}
        while rclpy.ok() and stamp_nanoseconds(self._truth) < target_ns:
            rclpy.spin_once(self, timeout_sec=0.02)
            correction = angular
            if target_yaw is not None:
                current = self._filtered_yaw()
                error = wrap_angle(target_yaw - current)
                correction = max(
                    -0.35,
                    min(0.35, float(self.get_parameter('heading_hold_gain').value)
                        * error))
            self._publish(linear=linear, angular=correction)
            for name, load in self._current_loads().items():
                samples[name].append(load)
        self._publish()
        self._samples[label] = samples
        self._report(label, samples)

    def _report(self, label, samples):
        parts = []
        for name in ('left_wheel', 'right_wheel', 'caster'):
            values = samples[name]
            if not values:
                parts.append('%s=no-data' % name)
                continue
            parts.append('%s mean=%.1f min=%.1f max=%.1f' % (
                name, sum(values) / len(values), min(values), max(values)))
        self.get_logger().info('%-16s %s' % (label + ':', '  '.join(parts)))

    def _write_csv(self):
        path = str(self.get_parameter('output_csv').value)
        if not path:
            return
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(path, 'w', newline='') as handle:
            writer = csv.writer(handle)
            writer.writerow([
                'manoeuvre', 'contact', 'mean_n', 'min_n', 'max_n', 'samples'])
            for label, samples in self._samples.items():
                for name in ('left_wheel', 'right_wheel', 'caster'):
                    values = samples[name]
                    if not values:
                        continue
                    writer.writerow([
                        label, name,
                        '%.3f' % (sum(values) / len(values)),
                        '%.3f' % min(values), '%.3f' % max(values), len(values)])
        self.get_logger().info('Wrote %s' % os.path.abspath(path))

    def run(self):
        self._wait_for_data()
        static_s = float(self.get_parameter('static_duration_s').value)
        drive_s = float(self.get_parameter('drive_duration_s').value)
        brake_s = float(self.get_parameter('brake_duration_s').value)
        turn_s = float(self.get_parameter('turn_duration_s').value)
        speed = float(self.get_parameter('linear_speed_mps').value)
        turn_rate = float(self.get_parameter('angular_speed_rps').value)

        self._record('static', static_s, 0.0)

        # Right then left, up then down, so the robot stays near its start
        # and never approaches the wall edges.
        self._turn_to(0.0)
        self._record('drive_right', drive_s, speed, target_yaw=0.0)
        self._turn_to(math.pi)
        self._record('drive_left', drive_s, speed, target_yaw=math.pi)
        self._turn_to(math.pi / 2.0)
        self._record('drive_up', drive_s, speed, target_yaw=math.pi / 2.0)
        self._turn_to(-math.pi / 2.0)
        self._record('drive_down', drive_s, speed, target_yaw=-math.pi / 2.0)

        # Braking while descending is the worst case for rear-caster unloading.
        self._record('brake_from_down', brake_s, 0.0)

        self._turn_to(0.0)
        self._record('turn_in_place', turn_s, 0.0, angular=turn_rate)

        self._write_csv()


def main():
    rclpy.init()
    measurement = NormalLoadMeasurement()
    try:
        measurement.run()
    finally:
        measurement._publish()
        measurement.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
