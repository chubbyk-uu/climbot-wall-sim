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

"""Verify /coverage/configure survives being driven out of order."""

from threading import Event
import unittest

from climbot_interfaces.msg import CoverageConfig
from climbot_interfaces.msg import CoverageTask
from climbot_interfaces.srv import ConfigureCoverage
from geometry_msgs.msg import PointStamped
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import Trigger


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Start one point-selection planner, the mode the panel drives."""
    planner = launch_ros.actions.Node(
        package='climbot_coverage',
        executable='coverage_planner_node',
        parameters=[{
            'input_mode': 'rviz',
            'region_type': 'rectangle',
            'sweep_direction': 'horizontal',
            'detection_width': 0.5,
            'overlap_ratio': 0.2,
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


class TestConfigure(unittest.TestCase):
    """Drive the service the way an operator clicking at random would."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('configure_test')
        self.config = None
        self.config_event = Event()
        self.task = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(
            CoverageConfig, '/coverage/config', self._config_callback, latched)
        self.node.create_subscription(
            CoverageTask, '/coverage/task', self._task_callback, latched)
        self.clicker = self.node.create_publisher(
            PointStamped, '/clicked_point', 10)
        self.configure = self.node.create_client(
            ConfigureCoverage, '/coverage/configure')
        self.clear = self.node.create_client(Trigger, '/coverage/clear_points')
        self.replan = self.node.create_client(Trigger, '/coverage/replan')
        self.assertTrue(self.configure.wait_for_service(timeout_sec=15.0))
        self.assertTrue(self.clear.wait_for_service(timeout_sec=15.0))
        self.assertTrue(self.replan.wait_for_service(timeout_sec=15.0))
        # Each test builds a fresh node, so the click publisher has to find the
        # planner again. Publishing before the match silently drops the point.
        deadline = 15.0
        while deadline > 0.0 and self.clicker.get_subscription_count() == 0:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            deadline -= 0.05
        self.assertGreater(self.clicker.get_subscription_count(), 0)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def _config_callback(self, message):
        self.config = message
        self.config_event.set()

    def _task_callback(self, message):
        self.task = message

    def _call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=15.0)
        self.assertTrue(future.done(), 'service call did not return')
        return future.result()

    def _set(self, region='', sweep=''):
        request = ConfigureCoverage.Request()
        request.region_type = region
        request.sweep_direction = sweep
        return self._call(self.configure, request)

    def _click(self, x, y, expect):
        """Click once and wait until the planner reports it counted."""
        message = PointStamped()
        message.header.frame_id = 'odom'
        message.point.x = float(x)
        message.point.y = float(y)
        self.clicker.publish(message)
        deadline = 8.0
        while deadline > 0.0:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            deadline -= 0.05
            if self.config is not None and self.config.selected_points == expect:
                return
        self.fail(
            'click was not counted: expected %d points, planner reports %s'
            % (expect, None if self.config is None else self.config.selected_points))

    def _latest_config(self):
        deadline = 5.0
        while deadline > 0.0 and self.config is None:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            deadline -= 0.05
        self.assertIsNotNone(self.config)
        return self.config

    def _wait_for_preview(self):
        """Wait for the task topic, not only the earlier config callback."""
        deadline = 5.0
        while deadline > 0.0:
            if self.task is not None and len(self.task.waypoints) > 2:
                return
            rclpy.spin_once(self.node, timeout_sec=0.05)
            deadline -= 0.05
        self.fail('planned preview was not received')

    def _rectangle(self):
        self._set(region='rectangle')
        self._call(self.clear, Trigger.Request())
        self._click(4.4, 1.4, expect=1)
        self._click(7.7, 4.2, expect=2)

    def test_a_late_panel_still_learns_the_configuration(self):
        """The topic is latched, so a panel started afterwards is not blind."""
        config = self._latest_config()
        self.assertEqual(config.region_type, 'rectangle')
        self.assertEqual(config.sweep_direction, 'horizontal')
        self.assertEqual(config.required_points, 2)
        self.assertEqual(config.input_mode, 'rviz')

    def test_rejects_nonsense_without_changing_anything(self):
        before = self._set(region='rectangle', sweep='vertical').config
        response = self._set(region='octagon')
        self.assertFalse(response.success)
        self.assertIn('rectangle or trapezoid', response.message)
        # The rejected call must not have moved the sweep either.
        self.assertEqual(response.config.region_type, before.region_type)
        self.assertEqual(response.config.sweep_direction, before.sweep_direction)
        response = self._set(sweep='sideways')
        self.assertFalse(response.success)
        self.assertIn('horizontal or vertical', response.message)
        self.assertEqual(response.config.sweep_direction, before.sweep_direction)

    def test_an_empty_field_leaves_that_setting_alone(self):
        """So changing one control cannot clobber the other."""
        self._set(region='trapezoid', sweep='vertical')
        response = self._set(sweep='horizontal')
        self.assertTrue(response.success)
        self.assertEqual(response.config.region_type, 'trapezoid')
        self.assertEqual(response.config.sweep_direction, 'horizontal')

    def test_switching_to_a_bigger_shape_keeps_the_points_and_says_what_is_missing(self):
        """A mis-click on a drop-down must not throw away the selection."""
        self._rectangle()
        self.assertTrue(self._latest_config().can_plan)
        response = self._set(region='trapezoid')
        self.assertTrue(response.success)
        self.assertEqual(response.config.selected_points, 2)
        self.assertEqual(response.config.required_points, 3)
        self.assertFalse(response.config.can_plan)
        self.assertIn('1 more point', response.message)
        # Replan has to refuse for the same reason, not a different one.
        replan = self._call(self.replan, Trigger.Request())
        self.assertFalse(replan.success)
        self.assertIn('1 more point', replan.message)

    def test_switching_back_restores_the_ability_to_plan(self):
        """Because nothing was discarded on the way out."""
        self._rectangle()
        self._set(region='trapezoid')
        response = self._set(region='rectangle')
        self.assertTrue(response.success)
        self.assertTrue(response.config.can_plan)
        self.assertEqual(response.config.selected_points, 2)

    def test_completing_a_trapezoid_after_switching_needs_only_the_third_click(self):
        """A and B mean the same corner in both shapes, so they carry over."""
        self._rectangle()
        self._set(region='trapezoid')
        self._click(8.4, 1.4, expect=3)
        config = self._latest_config()
        self.assertEqual(config.selected_points, 3)
        self.assertTrue(config.can_plan)
        self._wait_for_preview()

    # Three trapezoid points are enough for a rectangle, so this used to
    # reinterpret them as two and draw a different trajectory the moment the
    # shape changed. The points survive - a mis-click on a drop-down should not
    # cost a selection - but the task does not, until the operator asks for it.
    def test_switching_to_a_smaller_shape_withdraws_the_preview(self):
        """The drop-down must not plan something nobody asked for."""
        self._set(region='trapezoid')
        self._call(self.clear, Trigger.Request())
        self._click(4.4, 1.4, expect=1)
        self._click(7.7, 4.2, expect=2)
        self._click(8.4, 1.4, expect=3)
        self._wait_for_preview()
        response = self._set(region='rectangle')
        self.assertTrue(response.success)
        self.assertTrue(response.config.can_plan)
        self.assertEqual(response.config.selected_points, 3)
        self.assertIn('Preview withdrawn', response.message)
        self.assertEqual(len(self.task.waypoints), 0)

    def test_replanning_after_a_shape_change_rebuilds_from_the_same_points(self):
        """Withdrawing the preview must not strand the points that made it."""
        self._set(region='trapezoid')
        self._call(self.clear, Trigger.Request())
        self._click(4.4, 1.4, expect=1)
        self._click(7.7, 4.2, expect=2)
        self._click(8.4, 1.4, expect=3)
        self._set(region='rectangle')
        self.assertEqual(len(self.task.waypoints), 0)
        response = self._call(self.replan, Trigger.Request())
        self.assertTrue(response.success)
        self.assertGreater(len(self.task.waypoints), 2)

    def test_configuring_with_no_points_never_produces_a_startable_task(self):
        """The failure mode that once planned a region nobody selected."""
        self._call(self.clear, Trigger.Request())
        response = self._set(region='trapezoid', sweep='vertical')
        self.assertTrue(response.success)
        self.assertFalse(response.config.can_plan)
        self.assertEqual(response.config.selected_points, 0)
        self.assertEqual(len(self.task.waypoints), 0)

    def test_repeating_the_same_configuration_is_harmless(self):
        self._rectangle()
        first = self._set(region='rectangle', sweep='horizontal')
        second = self._set(region='rectangle', sweep='horizontal')
        self.assertTrue(first.success)
        self.assertTrue(second.success)
        self.assertIn('unchanged', second.message)
        self.assertTrue(second.config.can_plan)

    def test_sweep_direction_survives_a_shape_change(self):
        self._rectangle()
        self._set(sweep='vertical')
        response = self._set(region='trapezoid')
        self.assertEqual(response.config.sweep_direction, 'vertical')
        response = self._set(region='rectangle')
        self.assertEqual(response.config.sweep_direction, 'vertical')

    def test_clearing_points_drops_the_count_and_blocks_planning(self):
        self._rectangle()
        self.assertTrue(self._latest_config().can_plan)
        self.config_event.clear()
        self._call(self.clear, Trigger.Request())
        deadline = 5.0
        while deadline > 0.0 and not self.config_event.is_set():
            rclpy.spin_once(self.node, timeout_sec=0.05)
            deadline -= 0.05
        config = self._latest_config()
        self.assertEqual(config.selected_points, 0)
        self.assertFalse(config.can_plan)
