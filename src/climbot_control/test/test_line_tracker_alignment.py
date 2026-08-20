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

"""Node-level verification of profiled in-place alignment."""

import math
from threading import Event
from threading import Lock
from threading import Thread
import time
import unittest

from geometry_msgs.msg import Twist
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
from nav_msgs.msg import Odometry
import pytest
import rclpy


@pytest.mark.launch_test
def generate_test_description():
    """Start a tracker whose first segment requires a 90-degree turn."""
    tracker = launch_ros.actions.Node(
        package='climbot_control',
        executable='line_tracker_node',
        parameters=[{
            'start_x': 0.0,
            'start_y': 0.0,
            'end_x': 0.0,
            'end_y': 1.0,
            'odometry_timeout_s': 0.15,
            'alignment_settle_duration_s': 0.10,
            'wheel_separation': 0.43,
            'wheel_speed_limit': 0.45,
            'wheel_acceleration_limit': 0.40,
        }],
    )
    return launch.LaunchDescription([
        tracker,
        launch_testing.actions.ReadyToTest(),
    ])


class TestLineTrackerAlignment(unittest.TestCase):
    """Close a simple yaw-only plant around the alignment controller."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('line_tracker_alignment_test')
        self.command = Twist()
        self.lock = Lock()
        self.output_event = Event()
        self.publisher = self.node.create_publisher(
            Odometry, '/odometry/filtered', 10)
        self.node.create_subscription(
            Twist, '/control/cmd_vel', self._output_callback, 10)
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

    def _output_callback(self, message):
        with self.lock:
            self.command = message
        self.output_event.set()

    def _current_command(self):
        with self.lock:
            return self.command.linear.x, self.command.angular.z

    def test_turns_in_place_and_settles_before_driving(self):
        """Linear motion starts only after the profiled turn has settled."""
        yaw = 0.0
        target = math.pi / 2.0
        step = 0.02
        aligned_since = None
        deadline = time.monotonic() + 8.0
        motion_observed = False

        while time.monotonic() < deadline:
            odometry = Odometry()
            odometry.pose.pose.orientation.z = math.sin(yaw / 2.0)
            odometry.pose.pose.orientation.w = math.cos(yaw / 2.0)
            self.publisher.publish(odometry)
            self.output_event.wait(step)
            self.output_event.clear()
            linear, angular = self._current_command()

            self.assertLessEqual(abs(angular), 0.60 + 1e-9)
            error = abs(math.atan2(math.sin(target - yaw), math.cos(target - yaw)))
            if error <= math.radians(2.0):
                aligned_since = aligned_since or time.monotonic()
            else:
                aligned_since = None
            if linear > 1e-3:
                self.assertIsNotNone(aligned_since)
                self.assertGreaterEqual(time.monotonic() - aligned_since, 0.08)
                motion_observed = True
                break

            self.assertEqual(linear, 0.0)
            yaw += angular * step
            time.sleep(step)

        self.assertTrue(motion_observed)
        self.assertLessEqual(error, math.radians(2.0))


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    """A crashed tracker must fail the launch test."""

    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
