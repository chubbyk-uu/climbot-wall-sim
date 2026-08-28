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

"""Verify boundary-adjacent routes reserve the execution maneuver envelope."""

from threading import Event
import unittest

from climbot_interfaces.msg import CoverageTask
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile


# These are the P2-06 wall dimensions and the same full safe-frame selection
# that exposed the bug.  The planner must retain the requested coverage region
# but keep executable waypoints inside a 100 mm up/down turn envelope.
P206 = {
    'input_mode': 'parameters',
    'region_type': 'rectangle',
    'lower_left': [0.55, 0.55],
    'upper_right': [9.45, 7.45],
    'detection_width': 0.50,
    'detection_length': 0.28125,
    'detection_forward_offset': 0.340,
    'overlap_ratio': 0.20,
    'robot_length': 0.76,
    'robot_width': 0.475,
    'edge_clearance': 0.10,
    'wall_width': 10.0,
    'wall_height': 8.0,
    'maneuver_boundary_margin_m': 0.10,
    'maneuver_drift_direction': [0.0, -1.0],
}


def _planner(name, direction):
    return launch_ros.actions.Node(
        package='climbot_coverage',
        executable='coverage_planner_node',
        name=name,
        remappings=[
            ('/coverage/task', '/%s/task' % name),
            ('/coverage/path', '/%s/path' % name),
            ('/coverage/status', '/%s/status' % name),
            ('/coverage/markers', '/%s/markers' % name),
        ],
        parameters=[dict(P206, sweep_direction=direction)],
    )


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription([
        _planner('vertical_maneuver_envelope', 'vertical'),
        _planner('horizontal_maneuver_envelope', 'horizontal'),
        launch_testing.actions.ReadyToTest(),
    ])


class TestCoverageManeuverEnvelope(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('coverage_maneuver_envelope_test')
        self.tasks = {}
        self.events = {}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for name in ('vertical_maneuver_envelope', 'horizontal_maneuver_envelope'):
            self.events[name] = Event()
            self.node.create_subscription(
                CoverageTask, '/%s/task' % name, self._callback(name), qos)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def _callback(self, name):
        def receive(task):
            self.tasks[name] = task
            self.events[name].set()
        return receive

    def _task(self, name):
        remaining = 20.0
        while remaining > 0.0 and not self.events[name].is_set():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            remaining -= 0.1
        self.assertTrue(self.events[name].is_set(), '%s published no task' % name)
        task = self.tasks[name]
        self.assertGreater(len(task.waypoints), 0, 'planner rejected a feasible route')
        return task

    @staticmethod
    def _contains(rectangle, x, y):
        return (rectangle[0].x - 1e-6 <= x <= rectangle[2].x + 1e-6 and
                rectangle[0].y - 1e-6 <= y <= rectangle[2].y + 1e-6)

    def test_vertical_and_horizontal_routes_keep_the_turn_envelope_in_bounds(self):
        for name in ('vertical_maneuver_envelope', 'horizontal_maneuver_envelope'):
            task = self._task(name)
            self.assertEqual(len(task.coverage_region.points), 4)
            self.assertAlmostEqual(task.coverage_region.points[0].x, 0.55, places=6)
            self.assertAlmostEqual(task.coverage_region.points[0].y, 0.55, places=6)
            for waypoint in task.waypoints:
                self.assertTrue(self._contains(
                    task.motion_region.points, waypoint.position.x,
                    waypoint.position.y - 0.10), name)
                self.assertTrue(self._contains(
                    task.motion_region.points, waypoint.position.x,
                    waypoint.position.y + 0.10), name)

    def test_horizontal_route_does_not_lose_side_reach_for_vertical_gravity(self):
        task = self._task('horizontal_maneuver_envelope')
        leftmost = min(point.position.x for point in task.waypoints)
        rightmost = max(point.position.x for point in task.waypoints)
        # A vertical maneuver envelope protects top/bottom movement only.  It
        # must not act like an isotropic 100 mm inset and shrink horizontal
        # track endpoints away from the selected side boundaries.
        self.assertAlmostEqual(leftmost, 0.55, places=6)
        self.assertAlmostEqual(rightmost, 9.45, places=6)
