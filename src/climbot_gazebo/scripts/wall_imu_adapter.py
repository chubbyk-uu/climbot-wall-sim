#!/usr/bin/env python3
"""Publish an IMU observation with explicit wall-robot attitude uncertainty."""

import math
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def multiply(first, second):
    """Multiply quaternions represented as (x, y, z, w) tuples."""
    first_x, first_y, first_z, first_w = first
    second_x, second_y, second_z, second_w = second
    return (
        first_w * second_x + first_x * second_w + first_y * second_z - first_z * second_y,
        first_w * second_y - first_x * second_z + first_y * second_w + first_z * second_x,
        first_w * second_z + first_x * second_y - first_y * second_x + first_z * second_w,
        first_w * second_w - first_x * second_x - first_y * second_y - first_z * second_z,
    )


def rpy_quaternion(roll, pitch, yaw):
    """Return an (x, y, z, w) quaternion from roll, pitch, yaw angles."""
    roll_half = roll * 0.5
    pitch_half = pitch * 0.5
    yaw_half = yaw * 0.5
    cosine_roll = math.cos(roll_half)
    sine_roll = math.sin(roll_half)
    cosine_pitch = math.cos(pitch_half)
    sine_pitch = math.sin(pitch_half)
    cosine_yaw = math.cos(yaw_half)
    sine_yaw = math.sin(yaw_half)
    return (
        sine_roll * cosine_pitch * cosine_yaw - cosine_roll * sine_pitch * sine_yaw,
        cosine_roll * sine_pitch * cosine_yaw + sine_roll * cosine_pitch * sine_yaw,
        cosine_roll * cosine_pitch * sine_yaw - sine_roll * sine_pitch * cosine_yaw,
        cosine_roll * cosine_pitch * cosine_yaw + sine_roll * sine_pitch * sine_yaw,
    )


class WallImuAdapter(Node):
    """Add attitude noise and covariance absent from Gazebo's IMU message."""

    def __init__(self):
        super().__init__('wall_imu_adapter')
        self.declare_parameter('orientation_stddev_rad', math.radians(0.5))
        self.declare_parameter('random_seed', 17)
        self._orientation_stddev = float(
            self.get_parameter('orientation_stddev_rad').value)
        self._random = random.Random(
            int(self.get_parameter('random_seed').value))
        self._publisher = self.create_publisher(Imu, '/imu_wall', 50)
        self.create_subscription(Imu, '/imu', self._callback, 50)

    def _callback(self, message):
        observation = Imu()
        observation.header = message.header
        observation.orientation = message.orientation
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
        attitude_noise = rpy_quaternion(
            self._random.gauss(0.0, self._orientation_stddev),
            self._random.gauss(0.0, self._orientation_stddev),
            self._random.gauss(0.0, self._orientation_stddev),
        )
        noisy_orientation = multiply(truth_orientation, attitude_noise)
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
