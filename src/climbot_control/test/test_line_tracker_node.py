"""Node-level verification of filtered-odometry timeout handling."""

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


@pytest.mark.launch_test
def generate_test_description():
    """Start a line tracker with a short odometry timeout."""
    tracker = launch_ros.actions.Node(
        package='climbot_control',
        executable='line_tracker_node',
        parameters=[{
            'start_x': 0.0,
            'start_y': 0.0,
            'end_x': 1.0,
            'end_y': 0.0,
            'odometry_timeout_s': 0.15,
        }],
    )
    return launch.LaunchDescription([
        tracker,
        launch_testing.actions.ReadyToTest(),
    ])


class TestLineTrackerNode(unittest.TestCase):
    """Exercise fresh and stale odometry through ROS topics."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('line_tracker_timeout_test')
        self.received = []
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
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _output_callback(self, message):
        with self.lock:
            self.received.append((message.linear.x, message.angular.z))
        self.output_event.set()

    def _last_output(self):
        with self.lock:
            return self.received[-1]

    def test_stale_odometry_forces_stop(self):
        """A once-valid pose must not authorize motion forever."""
        self.assertTrue(self.output_event.wait(5.0))
        self.assertEqual(self._last_output(), (0.0, 0.0))

        odometry = Odometry()
        odometry.pose.pose.orientation.w = 1.0
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            self.publisher.publish(odometry)
            time.sleep(0.02)
            if self._last_output()[0] > 0.0:
                break
        self.assertGreater(self._last_output()[0], 0.0)

        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if self._last_output() == (0.0, 0.0):
                break
            time.sleep(0.01)
        self.assertEqual(self._last_output(), (0.0, 0.0))
