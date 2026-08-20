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

"""Node-level verification that stale commands are replaced with a stop."""

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
import pytest
import rclpy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


@pytest.mark.launch_test
def generate_test_description():
    """Start the watchdog with a short, deterministic timeout."""
    watchdog = launch_ros.actions.Node(
        package='climbot_control',
        executable='cmd_vel_watchdog_node',
        parameters=[{
            'command_timeout_s': 0.15,
            'publish_rate_hz': 50.0,
        }],
    )
    return launch.LaunchDescription([
        watchdog,
        launch_testing.actions.ReadyToTest(),
    ])


class TestCmdVelWatchdogNode(unittest.TestCase):
    """Test the running ROS node instead of only its time arithmetic."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('cmd_vel_watchdog_test')
        self.received = []
        self.lock = Lock()
        self.output_event = Event()
        self.publisher = self.node.create_publisher(Twist, '/control/cmd_vel', 10)
        self.node.create_subscription(Twist, '/cmd_vel', self._output_callback, 10)
        self.holds = []
        self.node.create_subscription(
            Bool, '/control/hold_active', lambda message: self.holds.append(message.data),
            rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))
        self.hold_client = self.node.create_client(SetBool, '/control/hold')
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
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _output_callback(self, message):
        with self.lock:
            self.received.append((message.linear.x, message.angular.z))
        self.output_event.set()

    def _last_output(self):
        with self.lock:
            return self.received[-1]

    def test_forwards_fresh_command_then_stops_after_timeout(self):
        self.assertTrue(self.output_event.wait(5.0), 'Watchdog did not publish initial stop.')
        self.assertEqual(self._last_output(), (0.0, 0.0))

        command = Twist()
        command.linear.x = 0.12
        command.angular.z = -0.20
        self.publisher.publish(command)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if self._last_output() == (0.12, -0.20):
                break
            time.sleep(0.01)
        self.assertEqual(self._last_output(), (0.12, -0.20))

        time.sleep(0.25)
        self.assertEqual(self._last_output(), (0.0, 0.0))

    def _set_hold(self, held):
        self.assertTrue(self.hold_client.wait_for_service(timeout_sec=10.0))
        future = self.hold_client.call_async(SetBool.Request(data=held))
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(future.done(), 'The hold service did not answer.')
        self.assertTrue(future.result().success)

    def _wait_for_output(self, expected, what, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._last_output() == expected:
                return
            time.sleep(0.01)
        self.fail('Timed out waiting for {}; last output was {}'.format(
            what, self._last_output()))

    def test_a_hold_zeroes_the_output_while_commands_keep_arriving(self):
        """The stop that works when the thing driving the robot will not listen."""
        # Cancelling a goal asks the controller to stop, which is the right way
        # round while the controller is answering. This is the case where it is
        # not: something keeps refreshing /control/cmd_vel and no request
        # reaches it. Freshness alone will never stop the robot, because the
        # commands are not stale - they are unwanted.
        self.assertTrue(self.output_event.wait(5.0), 'Watchdog did not publish initial stop.')
        command = Twist()
        command.linear.x = 0.12

        stop_driving = Event()

        def keep_driving():
            while not stop_driving.is_set():
                self.publisher.publish(command)
                time.sleep(0.02)

        driver = Thread(target=keep_driving)
        driver.start()
        try:
            self._wait_for_output((0.12, 0.0), 'the command to reach /cmd_vel')
            self._set_hold(True)
            self._wait_for_output((0.0, 0.0), 'the hold to zero the output')
            # Still driving, and still zero: the hold is not a timeout that a
            # busy publisher can keep resetting.
            time.sleep(0.5)
            self.assertEqual(self._last_output(), (0.0, 0.0))
            self.assertIn(True, self.holds, 'The hold was never announced.')

            self._set_hold(False)
            self._wait_for_output((0.12, 0.0), 'the release to restore the command')
        finally:
            stop_driving.set()
            driver.join()
            # One watchdog serves every test in this file, and the next one
            # measures how long a command survives being un-refreshed. Leave it
            # released and already fallen back to a stop, so nothing this test
            # published - including whatever is still in flight when the driver
            # thread stops - can be mistaken for that test's command.
            self._set_hold(False)
            self._wait_for_output(
                (0.0, 0.0), 'the watchdog to fall back to a stop', timeout=5.0)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    """A crashed safety node must fail the launch test."""

    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
