#!/usr/bin/env python3
"""Publish an IMU observation with explicit wall-robot attitude uncertainty."""

import math
import random

from climbot_description.geometry import quaternion_from_rpy, quaternion_multiply
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class WallImuAdapter(Node):
    """Add attitude noise and covariance absent from Gazebo's IMU message."""

    def __init__(self):
        super().__init__('wall_imu_adapter')
        self.declare_parameter('orientation_stddev_rad', math.radians(0.1))
        self.declare_parameter('random_seed', 17)
        self._orientation_stddev = float(
            self.get_parameter('orientation_stddev_rad').value)
        if self._orientation_stddev < 0.0:
            raise ValueError('orientation_stddev_rad cannot be negative.')
        self._random = random.Random(
            int(self.get_parameter('random_seed').value))
        self._publisher = self.create_publisher(Imu, '/imu_wall', 50)
        self.create_subscription(Imu, '/imu', self._callback, 50)

    def _callback(self, message):
        # Fields are copied rather than aliased, so this node never mutates
        # the raw /imu message that other subscribers may also receive.
        observation = Imu()
        observation.header.stamp = message.header.stamp
        observation.header.frame_id = message.header.frame_id
        observation.angular_velocity = message.angular_velocity
        observation.linear_acceleration = message.linear_acceleration
        observation.angular_velocity_covariance = message.angular_velocity_covariance
        observation.linear_acceleration_covariance = message.linear_acceleration_covariance

        truth_orientation = (
            message.orientation.x,
            message.orientation.y,
            message.orientation.z,
            message.orientation.w,
        )
        attitude_noise = quaternion_from_rpy(
            self._random.gauss(0.0, self._orientation_stddev),
            self._random.gauss(0.0, self._orientation_stddev),
            self._random.gauss(0.0, self._orientation_stddev),
        )
        noisy_orientation = quaternion_multiply(truth_orientation, attitude_noise)
        observation.orientation.x = noisy_orientation[0]
        observation.orientation.y = noisy_orientation[1]
        observation.orientation.z = noisy_orientation[2]
        observation.orientation.w = noisy_orientation[3]

        variance = self._orientation_stddev ** 2
        observation.orientation_covariance = [0.0] * 9
        observation.orientation_covariance[0] = variance
        observation.orientation_covariance[4] = variance
        observation.orientation_covariance[8] = variance
        self._publisher.publish(observation)


def main():
    rclpy.init()
    node = WallImuAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
