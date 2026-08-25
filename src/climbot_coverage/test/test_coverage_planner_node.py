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

"""Node-level test for path headings and wall dimensions."""

import math
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
            'lower_left': [3.0, 1.0],
            'upper_right': [9.0, 6.5],
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
        self.task = None
        self.markers = None
        self.grid = None
        self.path_event = Event()
        self.task_event = Event()
        self.marker_event = Event()
        self.grid_event = Event()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(Path, '/coverage/path', self._path_callback, qos)
        self.node.create_subscription(CoverageTask, '/coverage/task', self._task_callback, qos)
        self.node.create_subscription(
            MarkerArray, '/coverage/markers', self._marker_callback, qos)
        self.node.create_subscription(
            MarkerArray, '/coverage/wall_grid', self._grid_callback, qos)
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

    def _task_callback(self, message):
        self.task = message
        self.task_event.set()

    def _marker_callback(self, message):
        self.markers = message
        self.marker_event.set()

    def _grid_callback(self, message):
        self.grid = message
        self.grid_event.set()

    def test_wall_grid_draws_the_lines_gazebo_paints_on_the_wall(self):
        """The overlay has to be that grid, not a similar-looking one."""
        # A grid that starts at the wrong place or uses a different pitch is
        # still a plausible grid, and an operator reading a coordinate off it
        # gets a wrong answer with nothing to warn them. The rule, written once
        # in climbot_wall.sdf.xacro: whole multiples of the pitch from the work
        # frame's origin, interior lines only, each one spanning the wall. This
        # node runs on a 12 x 9 m wall at the default 1 m pitch.
        self.assertTrue(self.grid_event.wait(10.0), 'No wall grid received.')
        lines = [marker for marker in self.grid.markers if marker.ns == 'wall_grid']
        self.assertEqual(len(lines), 1)
        points = lines[0].points
        self.assertEqual(len(points) % 2, 0)
        segments = list(zip(points[::2], points[1::2]))
        vertical = sorted(
            round(start.x, 9) for start, end in segments
            if abs(start.x - end.x) < 1e-9)
        horizontal = sorted(
            round(start.y, 9) for start, end in segments
            if abs(start.y - end.y) < 1e-9)
        self.assertEqual(vertical, [float(index) for index in range(1, 12)])
        self.assertEqual(horizontal, [float(index) for index in range(1, 9)])
        for start, end in segments:
            if abs(start.x - end.x) < 1e-9:
                self.assertAlmostEqual(min(start.y, end.y), 0.0, places=9)
                self.assertAlmostEqual(max(start.y, end.y), 9.0, places=9)
            else:
                self.assertAlmostEqual(min(start.x, end.x), 0.0, places=9)
                self.assertAlmostEqual(max(start.x, end.x), 12.0, places=9)
        self.assertEqual(lines[0].header.frame_id, 'odom')

    def test_path_has_segment_headings_and_configured_wall(self):
        self.assertTrue(self.path_event.wait(10.0), 'No coverage Path received.')
        self.assertTrue(self.task_event.wait(10.0), 'No coverage task received.')
        self.assertTrue(self.marker_event.wait(10.0), 'No MarkerArray received.')
        self.assertGreater(len(self.path.poses), 2)
        self.assertEqual(len(self.task.waypoints), len(self.path.poses))
        self.assertEqual(len(self.task.segment_types), len(self.task.waypoints) - 1)
        self.assertGreater(self.task.revision, 0)
        self.assertEqual(self.task.header.frame_id, 'odom')
        self.assertEqual(self.task.sweep_direction, CoverageTask.SWEEP_HORIZONTAL)
        self.assertEqual(len(self.task.coverage_region.points), 4)
        self.assertEqual(len(self.task.motion_region.points), 4)
        self.assertLess(
            self.task.motion_region.points[0].x,
            self.task.coverage_region.points[0].x)
        self.assertGreater(
            self.task.motion_region.points[2].x,
            self.task.coverage_region.points[2].x)
        self.assertGreater(
            self.task.motion_region.points[2].y,
            self.task.coverage_region.points[2].y)
        self.assertGreater(self.task.detection_width, 0.0)
        self.assertGreater(self.task.detection_length, 0.0)
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
            expected_type = (
                CoverageTask.SEGMENT_SCAN if index % 2 == 0
                else CoverageTask.SEGMENT_TRANSITION)
            self.assertEqual(self.task.segment_types[index], expected_type)
            self.assertAlmostEqual(
                self.task.waypoints[index].position.x,
                current.position.x,
                places=9)
            self.assertAlmostEqual(
                self.task.waypoints[index].position.y,
                current.position.y,
                places=9)
        for waypoint in self.task.waypoints:
            self.assertGreaterEqual(
                waypoint.position.x, self.task.motion_region.points[0].x)
            self.assertLessEqual(
                waypoint.position.x, self.task.motion_region.points[2].x)
            self.assertGreaterEqual(
                waypoint.position.y, self.task.motion_region.points[0].y)
            self.assertLessEqual(
                waypoint.position.y, self.task.motion_region.points[2].y)
        wall = next(marker for marker in self.markers.markers if marker.ns == 'wall')
        self.assertAlmostEqual(wall.scale.x, 12.0)
        self.assertAlmostEqual(wall.scale.y, 9.0)
        # Centred on a surface that runs from the origin, not across it.
        self.assertAlmostEqual(wall.pose.position.x, 6.0)
        self.assertAlmostEqual(wall.pose.position.y, 4.5)
