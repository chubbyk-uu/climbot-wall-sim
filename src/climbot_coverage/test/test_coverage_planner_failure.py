"""Verify a planning failure clears the transient coverage Path."""

from threading import Event
from threading import Thread
import unittest

from climbot_interfaces.msg import CoverageTask
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
from nav_msgs.msg import Path
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Start a planner with an invalid rectangle."""
    planner = launch_ros.actions.Node(
        package='climbot_coverage',
        executable='coverage_planner_node',
        parameters=[{
            'input_mode': 'parameters',
            'region_type': 'rectangle',
            'lower_left': [1.0, 1.0],
            'upper_right': [0.0, 2.0],
            'robot_length': 0.76,
            'robot_width': 0.475,
            'edge_clearance': 0.1,
            'wall_width': 10.0,
            'wall_height': 8.0,
        }],
    )
    return launch.LaunchDescription([
        planner,
        launch_testing.actions.ReadyToTest(),
    ])


class TestCoveragePlannerFailure(unittest.TestCase):
    """Confirm a late subscriber receives an empty path, never stale work."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('coverage_planner_failure_test')
        self.path = None
        self.task = None
        self.path_event = Event()
        self.task_event = Event()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(Path, '/coverage/path', self._callback, qos)
        self.node.create_subscription(CoverageTask, '/coverage/task', self._task_callback, qos)
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
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def _callback(self, message):
        self.path = message
        self.path_event.set()

    def _task_callback(self, message):
        self.task = message
        self.task_event.set()

    def test_failure_publishes_empty_path(self):
        self.assertTrue(self.path_event.wait(10.0), 'No clearing Path received.')
        self.assertTrue(self.task_event.wait(10.0), 'No clearing task received.')
        self.assertEqual(len(self.path.poses), 0)
        self.assertEqual(len(self.task.waypoints), 0)
        self.assertGreater(self.task.revision, 0)
