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
            'odometry_timeout_s': 0.20,
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
        _pose(0.20, 0.20, math.pi / 2.0),
    ]
    task.segment_types = [
        CoverageTask.SEGMENT_SCAN, CoverageTask.SEGMENT_TRANSITION]
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

    def _current_command(self):
        with self.lock:
            return self.command.linear.x, self.command.angular.z

    def _publish_odometry(self, x, y, yaw, linear=0.0, angular=0.0):
        message = Odometry()
        message.pose.pose = _pose(x, y, yaw)
        message.twist.twist.linear.x = linear * math.cos(yaw)
        message.twist.twist.linear.y = linear * math.sin(yaw)
        message.twist.twist.angular.z = angular
        self.publisher.publish(message)

    def test_executes_two_segments_without_cutting_the_turn(self):
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
        deadline = time.monotonic() + 20.0
        max_linear_while_turning = 0.0
        while not result_future.done() and time.monotonic() < deadline:
            linear, angular = self._current_command()
            if abs(angular) > 0.02:
                max_linear_while_turning = max(
                    max_linear_while_turning, abs(linear))
            yaw += angular * step
            x += linear * math.cos(yaw) * step
            y += linear * math.sin(yaw) * step
            self._publish_odometry(x, y, yaw, linear, angular)
            time.sleep(step)

        self.assertTrue(result_future.done())
        wrapped = result_future.result()
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(
            wrapped.result.result_code, ExecuteCoverage.Result.SUCCESS)
        self.assertEqual(wrapped.result.completed_segments, 2)
        self.assertEqual(self.feedback_segments, {0, 1})
        self.assertLessEqual(max_linear_while_turning, 1e-4)
        self.assertLessEqual(math.hypot(x - 0.20, y - 0.20), 0.04)

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
        self.assertEqual(self._current_command(), (0.0, 0.0))

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
