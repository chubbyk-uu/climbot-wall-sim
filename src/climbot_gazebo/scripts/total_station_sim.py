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

"""Derive a delayed, noisy total-station position from Gazebo truth."""

from collections import deque
import os
import random

from ament_index_python.packages import get_package_share_directory
from climbot_description.geometry import quaternion_tuple, yaw_from_quaternion
from climbot_description.wall_frame import WallFrame
from climbot_gazebo.parameter_checks import require_finite
from climbot_gazebo.total_station_model import (
    LOCALIZATION_PROFILES,
    rotate_robot_residual_to_wall,
    timestamp_with_clock_error_ns,
)
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def default_wall_config():
    """Return the installed wall description, so no path is hardcoded."""
    return os.path.join(
        get_package_share_directory('climbot_description'), 'config', 'wall.yaml')


class TotalStationSimulator(Node):
    """Publish position-only total-station observations at a configured rate."""

    def __init__(self):
        super().__init__('total_station_sim')
        self.declare_parameter('publish_rate_hz', 12.0)
        self.declare_parameter('position_stddev_m', 0.001)
        self.declare_parameter('fixed_delay_s', 0.01)
        self.declare_parameter('drop_probability', 0.0)
        self.declare_parameter('random_seed', 42)
        self.declare_parameter('localization_profile', 'precision')
        self.declare_parameter('prism_extrinsic_error_enabled', False)
        self.declare_parameter('prism_extrinsic_error_robot_m',
                               [0.020, -0.010, 0.0])
        self.declare_parameter('measurement_timestamp_error_enabled', False)
        self.declare_parameter('measurement_timestamp_bias_s', 0.020)
        self.declare_parameter('measurement_timestamp_jitter_stddev_s', 0.002)
        self.declare_parameter('measurement_timestamp_jitter_seed', 20260827)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('wall_config', default_wall_config())

        self._rate = float(self.get_parameter('publish_rate_hz').value)
        self._stddev = float(self.get_parameter('position_stddev_m').value)
        delay_s = float(self.get_parameter('fixed_delay_s').value)
        self._drop_probability = float(
            self.get_parameter('drop_probability').value)
        self._profile = str(self.get_parameter('localization_profile').value)
        self._prism_error_enabled = bool(self.get_parameter(
            'prism_extrinsic_error_enabled').value)
        self._prism_error_robot = tuple(float(value) for value in self.get_parameter(
            'prism_extrinsic_error_robot_m').value)
        self._timestamp_error_enabled = bool(self.get_parameter(
            'measurement_timestamp_error_enabled').value)
        self._timestamp_bias_s = float(self.get_parameter(
            'measurement_timestamp_bias_s').value)
        self._timestamp_jitter_stddev_s = float(self.get_parameter(
            'measurement_timestamp_jitter_stddev_s').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._validate(delay_s)

        self._delay_ns = int(delay_s * 1e9)
        self._random = random.Random(
            int(self.get_parameter('random_seed').value))
        # Keep timestamp jitter independent from position noise and drops, so
        # enabling it cannot silently change the precision profile's samples.
        self._timestamp_random = random.Random(int(self.get_parameter(
            'measurement_timestamp_jitter_seed').value))
        # The Gazebo-world to wall-frame conversion comes from the shared wall
        # description, so moving the wall never means editing this node.
        self._wall_frame = WallFrame.from_yaml(
            str(self.get_parameter('wall_config').value))
        self._latest_truth = None
        self._pending = deque()

        self.create_subscription(
            Odometry, '/model/climbot/ground_truth', self._truth_callback, 20)
        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/total_station/pose', 20)
        self.create_timer(1.0 / self._rate, self._sample_truth)
        self.create_timer(0.005, self._publish_due_measurements)

    def _validate(self, delay_s):
        """Reject parameter values that would silently corrupt measurements."""
        require_finite('publish_rate_hz', self._rate)
        require_finite('position_stddev_m', self._stddev)
        require_finite('fixed_delay_s', delay_s)
        require_finite('drop_probability', self._drop_probability)
        require_finite('measurement_timestamp_bias_s', self._timestamp_bias_s)
        require_finite('measurement_timestamp_jitter_stddev_s',
                       self._timestamp_jitter_stddev_s)
        if self._rate <= 0.0:
            raise ValueError('publish_rate_hz must be positive.')
        if self._stddev < 0.0:
            raise ValueError('position_stddev_m cannot be negative.')
        if delay_s < 0.0:
            raise ValueError('fixed_delay_s cannot be negative.')
        if not 0.0 <= self._drop_probability <= 1.0:
            raise ValueError('drop_probability must be within [0, 1].')
        if self._profile not in LOCALIZATION_PROFILES:
            raise ValueError('localization_profile must be precision or realistic.')
        if len(self._prism_error_robot) != 3:
            raise ValueError('prism_extrinsic_error_robot_m needs three values.')
        for index, value in enumerate(self._prism_error_robot):
            require_finite('prism_extrinsic_error_robot_m[%d]' % index, value)
        if self._timestamp_jitter_stddev_s < 0.0:
            raise ValueError('measurement_timestamp_jitter_stddev_s cannot be negative.')

    def _truth_callback(self, message):
        self._latest_truth = message

    def _sample_truth(self):
        if self._latest_truth is None:
            return
        if self._random.random() < self._drop_probability:
            return

        source = self._latest_truth
        observation = PoseWithCovarianceStamped()
        # The stamp is copied rather than aliased: sharing the truth message's
        # header would let this node mutate the message it was handed.
        source_ns = (
            source.header.stamp.sec * 1_000_000_000
            + source.header.stamp.nanosec)
        stamped_ns = source_ns
        if self._timestamp_error_enabled:
            stamped_ns = timestamp_with_clock_error_ns(
                source_ns, self._timestamp_bias_s,
                self._timestamp_jitter_stddev_s, self._timestamp_random)
        observation.header.stamp.sec = stamped_ns // 1_000_000_000
        observation.header.stamp.nanosec = stamped_ns % 1_000_000_000
        observation.header.frame_id = self._frame_id

        truth = source.pose.pose.position
        wall = self._wall_frame.position_from_world((truth.x, truth.y, truth.z))
        residual_wall = (0.0, 0.0, 0.0)
        if self._prism_error_enabled:
            yaw = yaw_from_quaternion(self._wall_frame.orientation_from_world(
                quaternion_tuple(source.pose.pose.orientation)))
            residual_wall = rotate_robot_residual_to_wall(
                self._prism_error_robot, yaw)
        observation.pose.pose.position.x = (
            wall[0] + residual_wall[0] + self._random.gauss(0.0, self._stddev))
        observation.pose.pose.position.y = (
            wall[1] + residual_wall[1] + self._random.gauss(0.0, self._stddev))
        observation.pose.pose.position.z = (
            wall[2] + residual_wall[2] + self._random.gauss(0.0, self._stddev))
        # Identity rather than an all-zero quaternion, which is not a rotation.
        observation.pose.pose.orientation.w = 1.0

        variance = self._stddev * self._stddev
        observation.pose.covariance = [0.0] * 36
        observation.pose.covariance[0] = variance
        observation.pose.covariance[7] = variance
        observation.pose.covariance[14] = variance
        # Orientation is intentionally unobserved by the total station.
        observation.pose.covariance[21] = 1e6
        observation.pose.covariance[28] = 1e6
        observation.pose.covariance[35] = 1e6

        # Delivery remains a separately known transport delay from the truth
        # sample time.  A clock-error stamp must not alter when a packet arrives.
        self._pending.append((source_ns + self._delay_ns, observation))

    def _publish_due_measurements(self):
        now = self.get_clock().now().nanoseconds
        while self._pending and self._pending[0][0] <= now:
            _, observation = self._pending.popleft()
            self._publisher.publish(observation)


def main():
    rclpy.init()
    node = TotalStationSimulator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        except RCLError:
            if rclpy.ok():
                raise
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
