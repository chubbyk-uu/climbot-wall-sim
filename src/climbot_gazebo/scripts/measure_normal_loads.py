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

"""Measure the three wall-contact normal loads across the guide's manoeuvres."""

import csv
import json
import math
import os
import time

from climbot_description.geometry import (
    quaternion_tuple,
    wrap_angle,
    yaw_from_quaternion,
)
from climbot_gazebo.provenance import git_state
from climbot_gazebo.safe_stop import install_stop_on_termination
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


def load_statistics(values):
    """Summarise event-based contact samples, including lift-off steps."""
    if not values:
        return None
    loaded = sum(value > 0.0 for value in values)
    return {
        'mean': sum(values) / len(values),
        'min': min(values),
        'max': max(values),
        'samples': len(values),
        'zero_samples': len(values) - loaded,
        'contact_ratio': loaded / len(values),
    }


def normalise_headings(values):
    """Validate heading degrees and return unique values wrapped to [0, 360)."""
    headings = []
    for value in values:
        heading = float(value)
        if not math.isfinite(heading):
            raise ValueError('static_headings_deg must contain finite values')
        wrapped = heading % 360.0
        if any(abs(wrapped - existing) < 1e-9 for existing in headings):
            raise ValueError('static_headings_deg must not contain duplicates')
        headings.append(wrapped)
    if not headings:
        raise ValueError('static_headings_deg must not be empty')
    return headings


class NormalLoadMeasurement(Node):
    """Run the load-distribution test and report per-manoeuvre contact loads."""

    def __init__(self):
        super().__init__('normal_load_measurement')
        self.declare_parameter('static_duration_s', 10.0)
        self.declare_parameter('static_settle_duration_s', 1.0)
        self.declare_parameter(
            'static_headings_deg', [float(value) for value in range(0, 360, 15)])
        self.declare_parameter('drive_duration_s', 8.0)
        self.declare_parameter('brake_duration_s', 2.0)
        self.declare_parameter('turn_angle_deg', 360.0)
        self.declare_parameter('linear_speed_mps', 0.15)
        self.declare_parameter('angular_speed_rps', 0.6)
        self.declare_parameter('heading_hold_gain', 1.5)
        self.declare_parameter('contact_timeout_s', 0.15)
        self.declare_parameter('turn_timeout_s', 25.0)
        self.declare_parameter('output_csv', 'results/normal_loads.csv')
        self.declare_parameter('summary_json', '')
        self.declare_parameter('minimum_caster_load_n', 60.0)

        self._truth = None
        self._filtered = None
        # Latest load and its arrival time, so a silent sensor reads as
        # zero load instead of holding the last value from before lift-off.
        self._loads = {name: 0.0 for name in CONTACT_TOPICS}
        self._load_stamps = {name: None for name in CONTACT_TOPICS}
        self._samples = {}
        self._sample_metadata = {}
        self._active_samples = None
        self._recorded_stamps = {name: None for name in CONTACT_TOPICS}

        self._command = self.create_publisher(Twist, '/control/cmd_vel', 10)
        self.create_subscription(
            Odometry, '/model/climbot/ground_truth', self._truth_callback, 10)
        self.create_subscription(
            Odometry, '/odometry/filtered', self._filtered_callback, 10)
        for name, topic in CONTACT_TOPICS.items():
            self.create_subscription(
                Contacts, topic,
                lambda message, key=name: self._contact_callback(key, message),
                1000)

    def _truth_callback(self, message):
        self._truth = message

    def _filtered_callback(self, message):
        self._filtered = message

    def _contact_callback(self, name, message):
        load = contact_normal_load(message)
        stamp = stamp_nanoseconds(message)
        self._loads[name] = load
        self._load_stamps[name] = stamp
        if self._active_samples is None:
            return
        if stamp == self._recorded_stamps[name]:
            return
        self._active_samples[name].append(load)
        self._recorded_stamps[name] = stamp

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
            if self._truth is None or self._filtered is None:
                continue
            if any(stamp is None for stamp in self._load_stamps.values()):
                continue
            # Also wait for the contact stream to catch up with the truth
            # clock. Sampling before that reports the startup lag as a
            # spurious lift-off in the first manoeuvre's minimum.
            if all(load > 0.0 for load in self._current_loads().values()):
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

    def _settle(self, duration_s):
        """Hold zero command for a simulated-time duration without recording."""
        target_ns = stamp_nanoseconds(self._truth) + int(duration_s * 1e9)
        while rclpy.ok() and stamp_nanoseconds(self._truth) < target_ns:
            rclpy.spin_once(self, timeout_sec=0.02)
            self._publish()

    def _record(self, label, duration_s, linear, angular=0.0, target_yaw=None,
                category='dynamic', heading_deg=None):
        """Drive while recording each contact-sensor message exactly once."""
        target_ns = stamp_nanoseconds(self._truth) + int(duration_s * 1e9)
        samples = {name: [] for name in CONTACT_TOPICS}
        self._recorded_stamps = {name: None for name in CONTACT_TOPICS}
        self._active_samples = samples
        try:
            while rclpy.ok() and stamp_nanoseconds(self._truth) < target_ns:
                rclpy.spin_once(self, timeout_sec=0.02)
                correction = angular
                if target_yaw is not None:
                    current = self._filtered_yaw()
                    error = wrap_angle(target_yaw - current)
                    correction = max(
                        -0.35,
                        min(0.35, float(self.get_parameter(
                            'heading_hold_gain').value) * error))
                self._publish(linear=linear, angular=correction)
        finally:
            self._active_samples = None
            self._publish()
        self._samples[label] = samples
        self._sample_metadata[label] = (category, heading_deg)
        self._report(label, samples)

    def _record_turn(self, label, direction):
        """Record one complete in-place revolution in the requested direction."""
        requested = math.radians(float(self.get_parameter('turn_angle_deg').value))
        if not math.isfinite(requested) or requested <= 0.0:
            raise ValueError('turn_angle_deg must be finite and positive')
        rate = float(self.get_parameter('angular_speed_rps').value)
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError('angular_speed_rps must be finite and positive')
        deadline_ns = stamp_nanoseconds(self._truth) + int(
            float(self.get_parameter('turn_timeout_s').value) * 1e9)
        samples = {name: [] for name in CONTACT_TOPICS}
        self._recorded_stamps = {name: None for name in CONTACT_TOPICS}
        self._active_samples = samples
        previous = self._filtered_yaw()
        progress = 0.0
        try:
            while (rclpy.ok() and progress < requested and
                   stamp_nanoseconds(self._truth) < deadline_ns):
                rclpy.spin_once(self, timeout_sec=0.02)
                current = self._filtered_yaw()
                progress += max(
                    0.0, direction * wrap_angle(current - previous))
                previous = current
                self._publish(angular=direction * rate)
        finally:
            self._active_samples = None
            self._publish()
        if progress < requested:
            raise RuntimeError('Timed out recording a complete in-place turn.')
        self._samples[label] = samples
        self._sample_metadata[label] = ('turn', None)
        self._report(label, samples)

    def _report(self, label, samples):
        parts = []
        for name in ('left_wheel', 'right_wheel', 'caster'):
            statistics = load_statistics(samples[name])
            if statistics is None:
                parts.append('%s=no-data' % name)
                continue
            parts.append('%s mean=%.1f min=%.1f max=%.1f contact=%.2f%%' % (
                name, statistics['mean'], statistics['min'], statistics['max'],
                100.0 * statistics['contact_ratio']))
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
                'manoeuvre', 'category', 'heading_deg', 'contact', 'mean_n',
                'min_n', 'max_n', 'samples', 'zero_samples',
                'contact_ratio_percent'])
            for label, samples in self._samples.items():
                category, heading_deg = self._sample_metadata[label]
                for name in ('left_wheel', 'right_wheel', 'caster'):
                    statistics = load_statistics(samples[name])
                    if statistics is None:
                        continue
                    writer.writerow([
                        label, category,
                        '' if heading_deg is None else '%.3f' % heading_deg,
                        name,
                        '%.3f' % statistics['mean'],
                        '%.3f' % statistics['min'],
                        '%.3f' % statistics['max'], statistics['samples'],
                        statistics['zero_samples'],
                        '%.3f' % (100.0 * statistics['contact_ratio'])])
        self.get_logger().info('Wrote %s' % os.path.abspath(path))

    def _summarize(self):
        """State the G1 load verdict and the exact manoeuvre that limits it."""
        records = []
        for label, samples in self._samples.items():
            category, heading_deg = self._sample_metadata[label]
            for contact in ('left_wheel', 'right_wheel', 'caster'):
                statistics = load_statistics(samples[contact])
                if statistics is None:
                    continue
                records.append({
                    'manoeuvre': label,
                    'category': category,
                    'heading_deg': heading_deg,
                    'contact': contact,
                    **statistics,
                })
        minima = {}
        for contact in ('left_wheel', 'right_wheel', 'caster'):
            candidates = [
                record for record in records if record['contact'] == contact]
            worst = min(candidates, key=lambda record: record['min'])
            minima[contact] = {
                'minimum_n': worst['min'],
                'manoeuvre': worst['manoeuvre'],
                'category': worst['category'],
                'heading_deg': worst['heading_deg'],
            }
        caster_limit = float(
            self.get_parameter('minimum_caster_load_n').value)
        if not math.isfinite(caster_limit) or caster_limit <= 0.0:
            raise ValueError('minimum_caster_load_n must be positive and finite')
        no_lift_off = all(record['zero_samples'] == 0 for record in records)
        passed = no_lift_off and minima['caster']['minimum_n'] >= caster_limit
        parameters = {
            name: self.get_parameter(name).value
            for name in (
                'static_duration_s', 'static_settle_duration_s',
                'static_headings_deg', 'drive_duration_s', 'brake_duration_s',
                'turn_angle_deg', 'linear_speed_mps', 'angular_speed_rps',
                'minimum_caster_load_n')
        }
        return {
            'schema_version': 1,
            'passed': passed,
            'record_count': len(records),
            'manoeuvre_count': len(self._samples),
            'no_lift_off': no_lift_off,
            'minimum_caster_load_n': caster_limit,
            'caster_margin_n': minima['caster']['minimum_n'] - caster_limit,
            'minimum_by_contact': minima,
            'parameters': parameters,
            'records': records,
            'provenance': {'git': git_state()},
        }

    def _write_summary(self):
        path = str(self.get_parameter('summary_json').value)
        summary = self._summarize()
        if path:
            expanded = os.path.abspath(os.path.expanduser(path))
            os.makedirs(os.path.dirname(expanded), exist_ok=True)
            with open(expanded, 'w') as handle:
                json.dump(summary, handle, indent=2, sort_keys=True, allow_nan=False)
                handle.write('\n')
            self.get_logger().info('Wrote %s' % expanded)
        self.get_logger().info(
            'G1_LOAD_PASS=%s caster_min=%.3f N margin=%.3f N records=%d' % (
                summary['passed'],
                summary['minimum_by_contact']['caster']['minimum_n'],
                summary['caster_margin_n'], summary['record_count']))
        return summary

    def run(self):
        self._wait_for_data()
        static_s = float(self.get_parameter('static_duration_s').value)
        drive_s = float(self.get_parameter('drive_duration_s').value)
        brake_s = float(self.get_parameter('brake_duration_s').value)
        speed = float(self.get_parameter('linear_speed_mps').value)
        settle_s = float(
            self.get_parameter('static_settle_duration_s').value)
        if not math.isfinite(static_s) or static_s <= 0.0:
            raise ValueError('static_duration_s must be finite and positive')
        if not math.isfinite(settle_s) or settle_s < 0.0:
            raise ValueError(
                'static_settle_duration_s must be finite and non-negative')

        headings = normalise_headings(
            self.get_parameter('static_headings_deg').value)
        for heading_deg in headings:
            self._turn_to(math.radians(heading_deg))
            self._settle(settle_s)
            self._record(
                'static_heading_%03d' % round(heading_deg), static_s, 0.0,
                category='static', heading_deg=heading_deg)

        # Right then left, up then down, so the robot stays near its start
        # and never approaches the wall edges.
        directions = [
            ('right', 0.0), ('left', math.pi),
            ('up', math.pi / 2.0), ('down', -math.pi / 2.0),
        ]
        for name, yaw in directions:
            self._turn_to(yaw)
            self._record(
                'drive_' + name, drive_s, speed, target_yaw=yaw,
                heading_deg=math.degrees(yaw) % 360.0)
            self._record(
                'brake_from_' + name, brake_s, 0.0, category='brake',
                heading_deg=math.degrees(yaw) % 360.0)

        self._turn_to(0.0)
        self._record_turn('turn_in_place_ccw', 1.0)
        self._record_turn('turn_in_place_cw', -1.0)

        self._write_csv()
        summary = self._write_summary()
        if not summary['passed']:
            raise RuntimeError('G1 normal-load acceptance failed')


def main():
    rclpy.init()
    measurement = NormalLoadMeasurement()
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
