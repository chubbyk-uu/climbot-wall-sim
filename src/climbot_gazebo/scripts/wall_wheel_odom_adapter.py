#!/usr/bin/env python3
"""Inflate wheel-odometry uncertainty for force-dependent wall slip."""

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class WallWheelOdomAdapter(Node):
    """Publish wheel odometry with wall-robot-appropriate covariance."""

    def __init__(self):
        super().__init__('wall_wheel_odom_adapter')
        self.declare_parameter('forward_velocity_stddev_mps', 0.03)
        self.declare_parameter('yaw_rate_stddev_rps', 0.05)
        self.declare_parameter('unobserved_variance', 1e6)

        self._forward_variance = float(
            self.get_parameter('forward_velocity_stddev_mps').value) ** 2
        self._yaw_rate_variance = float(
            self.get_parameter('yaw_rate_stddev_rps').value) ** 2
        self._unobserved_variance = float(
            self.get_parameter('unobserved_variance').value)

        self._publisher = self.create_publisher(Odometry, '/wheel_odom', 20)
        self.create_subscription(
            Odometry, '/model/climbot/odometry', self._callback, 20)

    def _callback(self, message):
        # DiffDrive's pose is intentionally retained for diagnostic comparison,
        # but EKF only selects twist.x and twist.angular.z from this message.
        adapted = Odometry()
        adapted.header = message.header
        adapted.child_frame_id = message.child_frame_id
        adapted.pose = message.pose
        adapted.twist = message.twist

        adapted.pose.covariance = [0.0] * 36
        for index in (0, 7, 14, 21, 28, 35):
            adapted.pose.covariance[index] = self._unobserved_variance

        adapted.twist.covariance = [0.0] * 36
        for index in (7, 14, 21, 28):
            adapted.twist.covariance[index] = self._unobserved_variance
        adapted.twist.covariance[0] = self._forward_variance
        adapted.twist.covariance[35] = self._yaw_rate_variance
        self._publisher.publish(adapted)


def main():
    rclpy.init()
    node = WallWheelOdomAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
