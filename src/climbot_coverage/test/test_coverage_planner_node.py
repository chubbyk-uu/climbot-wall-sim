"""Node-level test for path headings and wall dimensions."""

import math
from threading import Event
from threading import Thread
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
from nav_msgs.msg import Path
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import MarkerArray


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Start one deterministic parameter-mode planner."""
    planner = launch_ros.actions.Node(
        package='climbot_coverage',
        executable='coverage_planner_node',
        parameters=[{
            'input_mode': 'parameters',
            'region_type': 'rectangle',
            'lower_left': [-3.0, 0.5],
            'upper_right': [3.0, 6.5],
            'detection_width': 0.5,
            'overlap_ratio': 0.2,
            'robot_length': 0.76,
            'robot_width': 0.475,
            'edge_clearance': 0.1,
            'wall_width': 12.0,
            'wall_height': 9.0,
        }],
    )
    return launch.LaunchDescription([
        planner,
        launch_testing.actions.ReadyToTest(),
    ])


class TestCoveragePlannerNode(unittest.TestCase):
    """Observe the transient-local outputs as an external consumer would."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('coverage_planner_node_test')
        self.path = None
        self.markers = None
        self.path_event = Event()
        self.marker_event = Event()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(Path, '/coverage/path', self._path_callback, qos)
        self.node.create_subscription(
            MarkerArray, '/coverage/markers', self._marker_callback, qos)
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

    def _path_callback(self, message):
        self.path = message
        self.path_event.set()

    def _marker_callback(self, message):
        self.markers = message
        self.marker_event.set()

    def test_path_has_segment_headings_and_configured_wall(self):
        self.assertTrue(self.path_event.wait(10.0), 'No coverage Path received.')
        self.assertTrue(self.marker_event.wait(10.0), 'No MarkerArray received.')
        self.assertGreater(len(self.path.poses), 2)
        for index in range(len(self.path.poses) - 1):
            current = self.path.poses[index].pose
            following = self.path.poses[index + 1].pose
            expected = math.atan2(
                following.position.y - current.position.y,
                following.position.x - current.position.x)
            actual = 2.0 * math.atan2(current.orientation.z, current.orientation.w)
            self.assertAlmostEqual(
                math.atan2(math.sin(actual - expected), math.cos(actual - expected)),
                0.0, places=9)
        wall = next(marker for marker in self.markers.markers if marker.ns == 'wall')
        self.assertAlmostEqual(wall.scale.x, 12.0)
        self.assertAlmostEqual(wall.scale.y, 9.0)
        self.assertAlmostEqual(wall.pose.position.y, 4.5)
