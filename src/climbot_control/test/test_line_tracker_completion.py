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

"""Node-level verification of spatial line-segment completion."""

from threading import Event
from threading import Lock
from threading import Thread
import time
import unittest

from geometry_msgs.msg import Twist
import launch
import launch_ros.actions
import launch_testing.actions
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool


@pytest.mark.launch_test
def generate_test_description():
    """Start a tracker on a short horizontal segment."""
    tracker = launch_ros.actions.Node(
        package='climbot_control',
        executable='line_tracker_node',
        parameters=[{
            'start_x': 0.0,
            'start_y': 0.0,
            'end_x': 0.20,
            'end_y': 0.0,
            'alignment_settle_duration_s': 0.05,
            'final_approach_distance_m': 0.08,
            'final_approach_speed_mps': 0.06,
            'goal_settle_duration_s': 0.10,
            'odometry_timeout_s': 0.15,
            'wheel_separation': 0.43,
            'wheel_speed_limit': 0.45,
            'wheel_acceleration_limit': 0.40,
        }],
    )
    return launch.LaunchDescription([
        tracker,
        launch_testing.actions.ReadyToTest(),
    ])


class TestLineTrackerCompletion(unittest.TestCase):
    """Close a simple planar plant around the single-segment controller."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('line_tracker_completion_test')
        self.command = Twist()
        self.complete = False
        self.lock = Lock()
        self.publisher = self.node.create_publisher(
            Odometry, '/odometry/filtered', 10)
        self.node.create_subscription(
            Twist, '/control/cmd_vel', self._command_callback, 10)
        completion_qos = QoSProfile(depth=1)
        completion_qos.reliability = ReliabilityPolicy.RELIABLE
        completion_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.node.create_subscription(
            Bool, '/control/segment_complete', self._completion_callback,
            completion_qos)
        self.stop_spin = Event()
        self.spin_thread = Thread(target=self._spin)
        self.spin_thread.start()

    def tearDown(self):
        self.stop_spin.set()
        self.spin_thread.join()
        self.node.destroy_node()
        rclpy.shutdown()

    def _spin(self):
        while rclpy.ok() and not self.stop_spin.is_set():
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _command_callback(self, message):
        with self.lock:
            self.command = message

    def _completion_callback(self, message):
        with self.lock:
            self.complete = message.data

    def _state(self):
        with self.lock:
            return self.command.linear.x, self.command.angular.z, self.complete

    def test_requires_position_heading_and_stopped_velocity(self):
        """Completion latches only after the robot reaches and stops at the goal."""
        x = 0.0
        step = 0.02
        deadline = time.monotonic() + 8.0

        while time.monotonic() < deadline:
            linear, angular, complete = self._state()
            odometry = Odometry()
            odometry.pose.pose.position.x = x
            odometry.pose.pose.orientation.w = 1.0
            odometry.twist.twist.linear.x = linear
            odometry.twist.twist.angular.z = angular
            self.publisher.publish(odometry)
            x += linear * step
            if complete:
                break
            time.sleep(step)

        self.assertTrue(complete)
        self.assertLessEqual(abs(x - 0.20), 0.03)
        self.assertLessEqual(abs(angular), 0.02)

        for _ in range(10):
            odometry = Odometry()
            odometry.pose.pose.position.x = x
            odometry.pose.pose.orientation.w = 1.0
            self.publisher.publish(odometry)
            time.sleep(step)
        linear, angular, complete = self._state()
        self.assertTrue(complete)
        self.assertEqual((linear, angular), (0.0, 0.0))
