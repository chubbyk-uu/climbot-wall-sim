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

"""Node-level test for the RViz click-to-plan input mode."""

from threading import Event
from threading import Thread
import time
import unittest

from climbot_interfaces.msg import CoverageTask
from geometry_msgs.msg import PointStamped
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray

# Inside the reachable rectangle of the 12 x 9 m wall declared below,
# which the lower-left work-frame origin makes 0.548..11.452 by
# 0.548..8.452.
LOWER_LEFT = (3.0, 1.0)
UPPER_RIGHT = (9.0, 7.0)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Start a planner that only accepts regions clicked in RViz."""
    planner = launch_ros.actions.Node(
        package='climbot_coverage',
        executable='coverage_planner_node',
        parameters=[{
            'input_mode': 'rviz',
            'region_type': 'rectangle',
            'detection_width': 0.5,
            'detection_length': 0.28125,
            'detection_forward_offset': 0.340,
            'detection_edge_overlap': 0.020,
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


class TestCoveragePlannerRvizInput(unittest.TestCase):
    """Drive the planner exactly as the RViz Publish Point tool would."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('coverage_planner_rviz_test')
        self.task = None
        self.markers = None
        self.status = []
        transient = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(
            CoverageTask, '/coverage/task', self._task_callback, transient)
        self.node.create_subscription(
            MarkerArray, '/coverage/markers', self._marker_callback, transient)
        self.node.create_subscription(
            String, '/coverage/status', self._status_callback, transient)
        # The tool publishes on a plain volatile topic, so the test publisher
        # must match it or early clicks are silently dropped.
        self.click_publisher = self.node.create_publisher(
            PointStamped, '/clicked_point', 10)
        self.clear_client = self.node.create_client(
            Trigger, '/coverage/clear_points')
        self.replan_client = self.node.create_client(Trigger, '/coverage/replan')
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

    def _task_callback(self, message):
        self.task = message

    def _marker_callback(self, message):
        self.markers = message

    def _status_callback(self, message):
        self.status.append(message.data)

    def _wait_for(self, predicate, timeout=10.0, description='condition'):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.05)
        self.fail('Timed out waiting for ' + description)

    def _click(self, x, y, frame_id='odom'):
        message = PointStamped()
        message.header.frame_id = frame_id
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.point.x = float(x)
        message.point.y = float(y)
        self.click_publisher.publish(message)

    def _clicked_point_markers(self):
        if self.markers is None:
            return []
        return [
            marker for marker in self.markers.markers
            if marker.ns == 'clicked_points']

    def _markers_in(self, marker_namespace):
        if self.markers is None:
            return []
        return [
            marker for marker in self.markers.markers
            if marker.ns == marker_namespace]

    def _call(self, client):
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        future = client.call_async(Trigger.Request())
        self._wait_for(future.done, description='a service response')
        return future.result()

    def test_replanning_without_a_selection_is_refused(self):
        """The configured corners survive into rviz mode and must not be run."""
        self._wait_for(
            lambda: self.task is not None, description='the initial empty task')
        # lower_left and friends still hold their parameter defaults here, and
        # clearing a selection does not reset them, so replanning from them
        # would hand the operator a startable task over an unselected region.
        refused = self._call(self.replan_client)
        self.assertFalse(refused.success)
        # The reason is worded once, in the planner, and reused by the replan
        # guard, the configure service and the panel's greying, so it names the
        # shape and the shortfall rather than the action that asked.
        self.assertIn('2 more points', refused.message)
        self.assertIn('rectangle', refused.message)
        self.assertEqual(len(self.task.waypoints), 0)

        self._wait_for(
            lambda: self.click_publisher.get_subscription_count() > 0,
            description='the planner to subscribe to /clicked_point')
        self._click(*LOWER_LEFT)
        self._wait_for(
            lambda: len(self._clicked_point_markers()) == 1,
            description='the first corner marker')
        # One point of two is still not a selection.
        self.assertFalse(self._call(self.replan_client).success)
        self.assertEqual(len(self.task.waypoints), 0)

        self._click(*UPPER_RIGHT)
        self._wait_for(
            lambda: len(self.task.waypoints) > 2,
            description='a planned path from the clicked corners')
        self.assertTrue(self._call(self.replan_client).success)

        self.assertTrue(self._call(self.clear_client).success)
        self._wait_for(
            lambda: len(self.task.waypoints) == 0,
            description='the task to be emptied')
        # Clearing leaves the last corners in place, so this is the case that
        # would silently resurrect the previous region.
        self.assertFalse(self._call(self.replan_client).success)
        self.assertEqual(len(self.task.waypoints), 0)

    def test_startup_shows_only_the_dashed_wall_safe_limit(self):
        self._wait_for(
            lambda: self.markers is not None,
            description='the startup boundary markers')
        green = self._markers_in('effective')
        self.assertEqual(len(green), 1)
        self.assertEqual(green[0].type, green[0].LINE_LIST)
        self.assertEqual(self._markers_in('camera_coverage'), [])

        self._wait_for(
            lambda: self.click_publisher.get_subscription_count() > 0,
            description='the planner to subscribe to /clicked_point')
        self._click(0.50, 1.0)
        self._wait_for(
            lambda: any('green wall-safe' in text for text in self.status),
            description='the wall-safety rejection')
        self.assertEqual(self._clicked_point_markers(), [])

    def test_two_clicks_produce_the_task_the_corners_describe(self):
        self._wait_for(
            lambda: self.click_publisher.get_subscription_count() > 0,
            description='the planner to subscribe to /clicked_point')
        self._wait_for(
            lambda: self.task is not None, description='the initial empty task')
        self.assertEqual(len(self.task.waypoints), 0)

        # A point published in another frame carries different coordinates and
        # must never be silently reinterpreted as a wall-plane corner.
        self._click(*LOWER_LEFT, frame_id='map')
        self._wait_for(
            lambda: any('Rejected clicked point' in text for text in self.status),
            description='the wrong-frame rejection')
        self.assertEqual(self._clicked_point_markers(), [])

        self._click(*LOWER_LEFT)
        self._wait_for(
            lambda: len(self._clicked_point_markers()) == 1,
            description='the first corner marker')
        self.assertEqual(len(self.task.waypoints), 0)

        self._click(*UPPER_RIGHT)
        self._wait_for(
            lambda: len(self.task.waypoints) > 2,
            description='a planned path from the clicked corners')
        self.assertEqual(len(self._clicked_point_markers()), 2)
        yellow = self._markers_in('camera_coverage')
        self.assertEqual(len(yellow), 1)
        self.assertEqual(yellow[0].type, yellow[0].TRIANGLE_LIST)
        self.assertGreater(len(yellow[0].points), 0)
        self.assertEqual(self.task.header.frame_id, 'odom')
        self.assertEqual(len(self.task.coverage_region.points), 4)
        self.assertEqual(
            len(self.task.segment_types), len(self.task.waypoints) - 1)
        corners = self.task.coverage_region.points
        self.assertAlmostEqual(corners[0].x, LOWER_LEFT[0], places=9)
        self.assertAlmostEqual(corners[0].y, LOWER_LEFT[1], places=9)
        self.assertAlmostEqual(corners[2].x, UPPER_RIGHT[0], places=9)
        self.assertAlmostEqual(corners[2].y, UPPER_RIGHT[1], places=9)

        self.assertTrue(
            self.clear_client.wait_for_service(timeout_sec=10.0),
            'The clear-points service never appeared.')
        future = self.clear_client.call_async(Trigger.Request())
        self._wait_for(future.done, description='the clear-points response')
        self.assertTrue(future.result().success)
        self._wait_for(
            lambda: len(self.task.waypoints) == 0,
            description='the task to be emptied')
        self.assertEqual(self._clicked_point_markers(), [])
