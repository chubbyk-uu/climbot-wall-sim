#!/usr/bin/env python3
"""Derive a delayed, noisy total-station position from Gazebo truth."""

from collections import deque
import os
import random

from ament_index_python.packages import get_package_share_directory
from climbot_gazebo.wall_frame import WallFrame
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


def default_wall_config():
    """Return the installed wall description, so no path is hardcoded."""
    return os.path.join(
        get_package_share_directory('climbot_gazebo'), 'config', 'wall.yaml')


class TotalStationSimulator(Node):
    """Publish position-only total-station observations at a configured rate."""

    def __init__(self):
        super().__init__('total_station_sim')
        self.declare_parameter('publish_rate_hz', 12.0)
        self.declare_parameter('position_stddev_m', 0.005)
        self.declare_parameter('fixed_delay_s', 0.05)
        self.declare_parameter('drop_probability', 0.0)
        self.declare_parameter('random_seed', 42)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('wall_config', default_wall_config())

        self._rate = float(self.get_parameter('publish_rate_hz').value)
        self._stddev = float(self.get_parameter('position_stddev_m').value)
        delay_s = float(self.get_parameter('fixed_delay_s').value)
        self._drop_probability = float(
            self.get_parameter('drop_probability').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._validate(delay_s)

        self._delay_ns = int(delay_s * 1e9)
        self._random = random.Random(
            int(self.get_parameter('random_seed').value))
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
        if self._rate <= 0.0:
            raise ValueError('publish_rate_hz must be positive.')
        if self._stddev < 0.0:
            raise ValueError('position_stddev_m cannot be negative.')
        if delay_s < 0.0:
            raise ValueError('fixed_delay_s cannot be negative.')
        if not 0.0 <= self._drop_probability <= 1.0:
            raise ValueError('drop_probability must be within [0, 1].')

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
        observation.header.stamp = source.header.stamp
        observation.header.frame_id = self._frame_id

        truth = source.pose.pose.position
        wall = self._wall_frame.position_from_world((truth.x, truth.y, truth.z))
        observation.pose.pose.position.x = wall[0] + self._random.gauss(0.0, self._stddev)
        observation.pose.pose.position.y = wall[1] + self._random.gauss(0.0, self._stddev)
        observation.pose.pose.position.z = wall[2] + self._random.gauss(0.0, self._stddev)
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

        measurement_time = (
            observation.header.stamp.sec * 1_000_000_000
            + observation.header.stamp.nanosec)
        self._pending.append((measurement_time + self._delay_ns, observation))

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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
