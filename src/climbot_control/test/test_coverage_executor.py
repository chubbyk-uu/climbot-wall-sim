"""Node-level verification of multi-segment coverage execution."""

import math
from threading import Event
from threading import Lock
from threading import Thread
import time
import unittest

from action_msgs.msg import GoalStatus
from climbot_interfaces.action import ExecuteCoverage
from climbot_interfaces.msg import CoverageTask
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import launch
import launch_ros.actions
import launch_testing.actions
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.action import ActionClient


@pytest.mark.launch_test
def generate_test_description():
    """Start the tracker in task-execution mode with faster test settling."""
    executor = launch_ros.actions.Node(
        package='climbot_control',
        executable='line_tracker_node',
        parameters=[{
            'standalone_mode': False,
            # Keep this integration test tolerant of loaded CI scheduling;
            # stale-input behavior has a dedicated short-timeout launch test.
            'odometry_timeout_s': 2.0,
            'segment_timeout_s': 15.0,
            'alignment_settle_duration_s': 0.05,
            'goal_settle_duration_s': 0.05,
            'final_approach_distance_m': 0.08,
            'final_approach_speed_mps': 0.06,
            'max_turn_angular_speed': 0.80,
            'max_turn_angular_acceleration': 2.0,
            'wheel_separation': 0.43,
            'wheel_speed_limit': 0.30,
            'wheel_acceleration_limit': 0.40,
        }],
    )
    return launch.LaunchDescription([
        executor,
        launch_testing.actions.ReadyToTest(),
    ])


def _pose(x, y, yaw):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def _task():
    task = CoverageTask()
    task.header.frame_id = 'odom'
    task.task_id = 'node-test'
    task.revision = 1
    task.sweep_direction = CoverageTask.SWEEP_HORIZONTAL
    task.waypoints = [
        _pose(0.0, 0.0, 0.0),
        _pose(0.20, 0.0, math.pi / 2.0),
        _pose(0.20, 0.20, math.pi),
        _pose(0.0, 0.20, math.pi),
    ]
    task.segment_types = [
        CoverageTask.SEGMENT_SCAN, CoverageTask.SEGMENT_TRANSITION,
        CoverageTask.SEGMENT_SCAN]
    for x, y in [(-0.5, -0.5), (0.5, -0.5),
                 (0.5, 0.5), (-0.5, 0.5)]:
        point = Point32()
        point.x = x
        point.y = y
        task.coverage_region.points.append(point)
        task.motion_region.points.append(point)
    task.detection_width = 0.10
    task.detection_length = 0.10
    return task


class TestCoverageExecutor(unittest.TestCase):
    """Close a planar kinematic plant around the Action server."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('coverage_executor_test')
        self.command = Twist()
        self.feedback_segments = set()
        self.feedback_state = ExecuteCoverage.Feedback.WAITING
        self.lock = Lock()
        self.publisher = self.node.create_publisher(
            Odometry, '/odometry/filtered', 10)
        self.node.create_subscription(
            Twist, '/control/cmd_vel', self._command_callback, 10)
        self.client = ActionClient(
            self.node, ExecuteCoverage, '/coverage/execute')
        self.stop_spin = Event()
        self.spin_thread = Thread(target=self._spin)
        self.spin_thread.start()

    def tearDown(self):
        self.stop_spin.set()
        self.spin_thread.join()
        self.client.destroy()
        self.node.destroy_node()
        rclpy.shutdown()

    def _spin(self):
        while rclpy.ok() and not self.stop_spin.is_set():
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def _command_callback(self, message):
        with self.lock:
            self.command = message

    def _feedback_callback(self, message):
        with self.lock:
            self.feedback_segments.add(message.feedback.current_segment)
            self.feedback_state = message.feedback.state

    def _current_command(self):
        with self.lock:
            return (self.command.linear.x, self.command.angular.z,
                    self.feedback_state)

    def _publish_odometry(self, x, y, yaw, linear=0.0, angular=0.0):
        message = Odometry()
        message.pose.pose = _pose(x, y, yaw)
        message.twist.twist.linear.x = linear * math.cos(yaw)
        message.twist.twist.linear.y = linear * math.sin(yaw)
        message.twist.twist.angular.z = angular
        self.publisher.publish(message)

    def test_executes_three_segments_without_cutting_the_turn(self):
        """The Action advances segments and turns only after linear stop."""
        self.assertTrue(self.client.wait_for_server(timeout_sec=3.0))
        x = y = yaw = 0.0
        for _ in range(5):
            self._publish_odometry(x, y, yaw)
            time.sleep(0.02)

        goal = ExecuteCoverage.Goal()
        goal.task = _task()
        send_future = self.client.send_goal_async(
            goal, feedback_callback=self._feedback_callback)
        deadline = time.monotonic() + 3.0
        while not send_future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(send_future.done())
        goal_handle = send_future.result()
        self.assertTrue(goal_handle.accepted)
        result_future = goal_handle.get_result_async()

        step = 0.01
        deadline = time.monotonic() + 40.0
        max_linear_while_turning = 0.0
        while not result_future.done() and time.monotonic() < deadline:
            linear, angular, state = self._current_command()
            if (state in (ExecuteCoverage.Feedback.ALIGN,
                          ExecuteCoverage.Feedback.TURN_SETTLE)
                    and abs(angular) > 0.02):
                max_linear_while_turning = max(
                    max_linear_while_turning, abs(linear))
            yaw_delta = angular * step
            y -= 0.0005 * abs(math.degrees(yaw_delta))
            yaw += yaw_delta
            x += linear * math.cos(yaw) * step
            y += linear * math.sin(yaw) * step
            self._publish_odometry(x, y, yaw, linear, angular)
            time.sleep(step)

        self.assertTrue(result_future.done())
        wrapped = result_future.result()
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(
            wrapped.result.result_code, ExecuteCoverage.Result.SUCCESS)
        self.assertEqual(wrapped.result.completed_segments, 3)
        self.assertEqual(self.feedback_segments, {0, 1, 2})
        self.assertLessEqual(max_linear_while_turning, 1e-4)
        self.assertLessEqual(math.hypot(x, y - 0.20), 0.04)

    def test_cancel_stops_an_active_task(self):
        """Cancellation returns a canceled result after commanding zero."""
        self.assertTrue(self.client.wait_for_server(timeout_sec=3.0))
        for _ in range(5):
            self._publish_odometry(0.0, 0.0, 0.0)
            time.sleep(0.02)
        goal = ExecuteCoverage.Goal()
        goal.task = _task()
        send_future = self.client.send_goal_async(goal)
        deadline = time.monotonic() + 2.0
        while not send_future.done() and time.monotonic() < deadline:
            self._publish_odometry(0.0, 0.0, 0.0)
            time.sleep(0.01)
        goal_handle = send_future.result()
        self.assertTrue(goal_handle.accepted)
        cancel_future = goal_handle.cancel_goal_async()
        while not cancel_future.done() and time.monotonic() < deadline:
            self._publish_odometry(0.0, 0.0, 0.0)
            time.sleep(0.01)
        self.assertTrue(cancel_future.done())
        self.assertGreater(len(cancel_future.result().goals_canceling), 0)
        result_future = goal_handle.get_result_async()
        while not result_future.done() and time.monotonic() < deadline:
            self._publish_odometry(0.0, 0.0, 0.0)
            time.sleep(0.01)
        wrapped = result_future.result()
        self.assertEqual(wrapped.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(
            wrapped.result.result_code, ExecuteCoverage.Result.CANCELED)
        time.sleep(0.05)
        linear, angular, _ = self._current_command()
        self.assertEqual((linear, angular), (0.0, 0.0))

    def test_approaches_first_waypoint_before_counting_scan_segments(self):
        """A distant start enters the first scan only after turn-slip recovery."""
        self.assertTrue(self.client.wait_for_server(timeout_sec=3.0))
        x, y, yaw = 0.0, -0.20, 0.0
        feedback_states = []
        for _ in range(5):
            self._publish_odometry(x, y, yaw)
            time.sleep(0.02)
        goal = ExecuteCoverage.Goal()
        goal.task = _task()
        goal.task.waypoints = goal.task.waypoints[:2]
        goal.task.waypoints[1] = _pose(0.60, 0.0, 0.0)
        goal.task.segment_types = [CoverageTask.SEGMENT_SCAN]
        for polygon in (goal.task.coverage_region, goal.task.motion_region):
            for point in polygon.points:
                if point.x > 0.0:
                    point.x = 0.75

        def record_feedback(message):
            feedback_states.append(message.feedback.state)
            self._feedback_callback(message)
        send_future = self.client.send_goal_async(
            goal, feedback_callback=record_feedback)
        deadline = time.monotonic() + 45.0
        while not send_future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(send_future.done())
        goal_handle = send_future.result()
        self.assertTrue(goal_handle.accepted)
        result_future = goal_handle.get_result_async()

        step = 0.01
        first_scan_y = None
        while not result_future.done() and time.monotonic() < deadline:
            linear, angular, state = self._current_command()
            if state == ExecuteCoverage.Feedback.TRACK_LINE and first_scan_y is None:
                first_scan_y = y
            yaw_delta = angular * step
            if abs(linear) <= 1e-4:
                y -= 0.0005 * abs(math.degrees(yaw_delta))
            yaw += angular * step
            x += linear * math.cos(yaw) * step
            y += linear * math.sin(yaw) * step
            self._publish_odometry(x, y, yaw, linear, angular)
            time.sleep(step)

        self.assertTrue(result_future.done())
        result = result_future.result()
        self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(result.result.completed_segments, 1)
        self.assertIn(ExecuteCoverage.Feedback.APPROACH_START, feedback_states)
        self.assertIsNotNone(first_scan_y)
        self.assertLess(abs(first_scan_y), 0.05)
        self.assertLess(math.hypot(x - 0.60, y - first_scan_y), 0.04)

    def test_survives_a_task_rejected_during_acceptance(self):
        """A goal that ends inside handleAccepted must not disturb the node."""
        # The task validates structurally, so it is accepted and only then
        # found to start outside the motion region. The long task_id keeps the
        # string off the small-string buffer, so any read after the task is
        # released touches returned heap memory, not stale inline bytes.
        self.assertTrue(self.client.wait_for_server(timeout_sec=3.0))
        for _ in range(5):
            self._publish_odometry(10.0, 10.0, 0.0)
            time.sleep(0.02)

        goal = ExecuteCoverage.Goal()
        goal.task = _task()
        goal.task.task_id = 'outside-motion-region-' + 'x' * 48
        send_future = self.client.send_goal_async(goal)
        deadline = time.monotonic() + 3.0
        while not send_future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(send_future.done())
        handle = send_future.result()
        self.assertTrue(handle.accepted)

        result_future = handle.get_result_async()
        deadline = time.monotonic() + 3.0
        while not result_future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(result_future.done())
        self.assertEqual(
            result_future.result().result.result_code,
            ExecuteCoverage.Result.OUT_OF_BOUNDS)

        # The server is still healthy and no longer holds the released task.
        for _ in range(5):
            self._publish_odometry(0.0, 0.0, 0.0)
            time.sleep(0.02)
        second = ExecuteCoverage.Goal()
        second.task = _task()
        second_future = self.client.send_goal_async(second)
        deadline = time.monotonic() + 3.0
        while not second_future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(second_future.done())
        self.assertTrue(second_future.result().accepted)
        second_future.result().cancel_goal_async()

    def test_rejects_structurally_invalid_task(self):
        """An empty task never becomes an executable goal."""
        self.assertTrue(self.client.wait_for_server(timeout_sec=3.0))
        goal = ExecuteCoverage.Goal()
        send_future = self.client.send_goal_async(goal)
        deadline = time.monotonic() + 2.0
        while not send_future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(send_future.done())
        self.assertFalse(send_future.result().accepted)
