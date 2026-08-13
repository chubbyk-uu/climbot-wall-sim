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
import pytest
import rclpy


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
