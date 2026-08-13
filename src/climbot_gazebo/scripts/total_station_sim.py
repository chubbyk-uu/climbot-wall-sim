#!/usr/bin/env python3
"""Derive a delayed, noisy total-station position from Gazebo truth."""

from collections import deque
import random

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


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

        self._rate = float(self.get_parameter('publish_rate_hz').value)
        self._stddev = float(self.get_parameter('position_stddev_m').value)
        self._delay_ns = int(float(
            self.get_parameter('fixed_delay_s').value) * 1e9)
        self._drop_probability = float(
            self.get_parameter('drop_probability').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._random = random.Random(
            int(self.get_parameter('random_seed').value))
        self._latest_truth = None
        self._pending = deque()

        self.create_subscription(
            Odometry, '/model/climbot/ground_truth', self._truth_callback, 20)
        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/total_station/pose', 20)
        self.create_timer(1.0 / self._rate, self._sample_truth)
        self.create_timer(0.005, self._publish_due_measurements)

    def _truth_callback(self, message):
        self._latest_truth = message

    def _sample_truth(self):
        if self._latest_truth is None:
            return
        if self._random.random() < self._drop_probability:
            return

        observation = PoseWithCovarianceStamped()
        observation.header = self._latest_truth.header
        observation.header.frame_id = self._frame_id
        # The wall work frame is right-handed: +X is robot-forward along the
        # wall (Gazebo +Y), +Y is wall-up (Gazebo +Z), and +Z is wall-normal
        # away from the wall (Gazebo +X). This agrees with the IMU's initial
        # identity orientation, unlike the Gazebo world frame.
        truth_position = self._latest_truth.pose.pose.position
        observation.pose.pose.position.x = (
            truth_position.y + self._random.gauss(0.0, self._stddev))
        observation.pose.pose.position.y = (
            truth_position.z + self._random.gauss(0.0, self._stddev))
        observation.pose.pose.position.z = (
            truth_position.x + self._random.gauss(0.0, self._stddev))

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
