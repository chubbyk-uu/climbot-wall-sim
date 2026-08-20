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

"""Node-level verification that time mode holds a schedule and catches up."""

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

SEGMENT_LENGTH = 1.0
CRUISE_SPEED = 0.20
CATCH_UP_SPEED = 0.35

# The trapezoidal profile the controller plans: two 1 s ramps at 0.20 m/s^2
# around a coast at the cruise speed.
PLANNED_DURATION = SEGMENT_LENGTH / CRUISE_SPEED + CRUISE_SPEED / 0.20


@pytest.mark.launch_test
def generate_test_description():
    """Start a tracker driving one straight segment from a time schedule."""
    tracker = launch_ros.actions.Node(
        package='climbot_control',
        executable='line_tracker_node',
        parameters=[{
            'start_x': 0.0,
            'start_y': 0.0,
            'end_x': SEGMENT_LENGTH,
            'end_y': 0.0,
            'tracking_mode': 'time',
            'cruise_speed': CRUISE_SPEED,
            'time_along_gain': 1.0,
            'time_profile_acceleration': 0.20,
            'time_profile_deceleration': 0.20,
            'catch_up_max_linear_speed': CATCH_UP_SPEED,
            'catch_up_max_linear_acceleration': 0.35,
            'alignment_settle_duration_s': 0.05,
            'goal_settle_duration_s': 0.10,
            'odometry_timeout_s': 0.15,
            'wheel_separation': 0.43,
            'wheel_speed_limit': 0.45,
            'wheel_acceleration_limit': 0.70,
        }],
    )
    return launch.LaunchDescription([
        tracker,
        launch_testing.actions.ReadyToTest(),
    ])


class TestLineTrackerTimeMode(unittest.TestCase):
    """Close a slipping planar plant around the time-parameterised controller."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('line_tracker_time_mode_test')
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

    # The node drives one segment and latches its completion, so this class
    # deliberately holds a single test: a second one would start against an
    # already finished controller.
    def test_falls_behind_under_slip_and_then_catches_up(self):
        """A robot that loses ground is commanded above its cruise speed."""
        x = 0.0
        step = 0.02
        peak_command = 0.0
        started = None
        deadline = time.monotonic() + 40.0

        while time.monotonic() < deadline:
            linear, angular, complete = self._state()
            odometry = Odometry()
            odometry.pose.pose.position.x = x
            odometry.pose.pose.orientation.w = 1.0
            odometry.twist.twist.linear.x = linear
            odometry.twist.twist.angular.z = angular
            self.publisher.publish(odometry)
            if linear > 0.0 and started is None:
                started = time.monotonic()
            # Half the commanded distance is lost for the first three seconds,
            # which is what puts the robot behind its own schedule.
            slipping = started is not None and time.monotonic() - started < 3.0
            x += linear * step * (0.5 if slipping else 1.0)
            peak_command = max(peak_command, linear)
            if complete:
                break
            time.sleep(step)

        self.assertTrue(complete, 'the segment never completed')
        self.assertLessEqual(abs(x - SEGMENT_LENGTH), 0.03)
        # Recovering the lost ground is what keeps the segment near its planned
        # duration; the settle and stop conditions add a fixed cost after the
        # curve ends, so this bounds the schedule rather than asserting it.
        self.assertLess(time.monotonic() - started, PLANNED_DURATION + 5.0)
        # The whole point of the mode: the correction reaches above the rated
        # cruise speed rather than merely observing that the robot is late.
        self.assertGreater(peak_command, CRUISE_SPEED * 1.05)
        # And it stays inside the ceiling it was given.
        self.assertLessEqual(peak_command, CATCH_UP_SPEED + 1e-6)
