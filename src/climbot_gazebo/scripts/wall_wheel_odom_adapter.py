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

"""Inflate wheel-odometry uncertainty for force-dependent wall slip."""

from climbot_gazebo.parameter_checks import require_finite
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

        forward_stddev = float(
            self.get_parameter('forward_velocity_stddev_mps').value)
        yaw_rate_stddev = float(self.get_parameter('yaw_rate_stddev_rps').value)
        self._unobserved_variance = float(
            self.get_parameter('unobserved_variance').value)
        require_finite('forward_velocity_stddev_mps', forward_stddev)
        require_finite('yaw_rate_stddev_rps', yaw_rate_stddev)
        require_finite('unobserved_variance', self._unobserved_variance)
        if forward_stddev < 0.0 or yaw_rate_stddev < 0.0:
            raise ValueError('Wheel odometry standard deviations cannot be negative.')
        if self._unobserved_variance <= 0.0:
            raise ValueError('unobserved_variance must be positive.')
        self._forward_variance = forward_stddev ** 2
        self._yaw_rate_variance = yaw_rate_stddev ** 2

        self._publisher = self.create_publisher(Odometry, '/wheel_odom', 20)
        self.create_subscription(
            Odometry, '/model/climbot/odometry', self._callback, 20)

    def _callback(self, message):
        # DiffDrive's pose is intentionally retained for diagnostic comparison,
        # but EKF only selects twist.x and twist.angular.z from this message.
        # Fields are copied rather than aliased so the incoming message keeps
        # the covariance it arrived with.
        adapted = Odometry()
        adapted.header.stamp = message.header.stamp
        adapted.header.frame_id = message.header.frame_id
        adapted.child_frame_id = message.child_frame_id
        adapted.pose.pose = message.pose.pose
        adapted.twist.twist = message.twist.twist

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
