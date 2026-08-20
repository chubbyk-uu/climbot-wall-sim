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
from rosgraph_msgs.msg import Clock


# cmd_vel_watchdog's command_timeout_s default, mirrored here so the plant
# coasts for exactly as long as the real one would and no longer.
COMMAND_TIMEOUT_S = 0.40

# The tracker runs on simulated time here and this test owns the clock, so the
# whole closed loop can be stepped faster than real time without changing what
# the controller experiences: it still sees a 50 Hz control period and the same
# timeouts, because those are measured on the clock this file publishes.
SIM_STEP_S = 0.01
TIME_SCALE = 10.0

# Every wait below is bounded in simulated seconds, which is what the numbers
# always meant. This is the wall-clock backstop for the case where the loop
# itself stops making progress.
REAL_TIME_MARGIN_S = 15.0


@pytest.mark.launch_test
def generate_test_description():
    """Start the tracker in task-execution mode with faster test settling."""
    executor = launch_ros.actions.Node(
        package='climbot_control',
        executable='line_tracker_node',
        parameters=[{
            'standalone_mode': False,
            # This file publishes /clock; see SIM_STEP_S.
            'use_sim_time': True,
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
            'wheel_speed_limit': 0.45,
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
        self.command_time = None
        self.sim_time = 0.0
        self.feedback_segments = set()
        self.feedback_state = ExecuteCoverage.Feedback.WAITING
        self.lock = Lock()
        self.publisher = self.node.create_publisher(
            Odometry, '/odometry/filtered', 10)
        self.clock_publisher = self.node.create_publisher(Clock, '/clock', 10)
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
            self.command_time = self.sim_time

    def _feedback_callback(self, message):
        with self.lock:
            self.feedback_segments.add(message.feedback.current_segment)
            self.feedback_state = message.feedback.state

    def _current_command(self):
        """Return the command the wheels would actually be given."""
        # The real robot never sees /control/cmd_vel directly: cmd_vel_watchdog
        # sits in between and zeroes the output once commands stop arriving for
        # command_timeout_s. A plant that instead integrates the last command
        # forever credits the controller with motion nothing asked for, and
        # turns any pause in the control loop into a silent overshoot rather
        # than the stop the real machine would perform.
        with self.lock:
            # Simulated time, not wall time: the watchdog this models times
            # out on the clock the controller runs on, so scaling the run must
            # not quietly scale its timeout with it.
            stale = (
                self.command_time is None or
                self.sim_time - self.command_time > COMMAND_TIMEOUT_S)
            if stale:
                return (0.0, 0.0, self.feedback_state)
            return (self.command.linear.x, self.command.angular.z,
                    self.feedback_state)

    def _advance(self, step=SIM_STEP_S):
        """Move the simulated clock one plant step and pace the real loop."""
        with self.lock:
            self.sim_time += step
            now = self.sim_time
        message = Clock()
        message.clock.sec = int(now)
        message.clock.nanosec = int(round((now - int(now)) * 1e9))
        self.clock_publisher.publish(message)
        time.sleep(step / TIME_SCALE)

    def _pending(self, future, timeout_s):
        """Yield while a future is unresolved, bounded in simulated time."""
        sim_deadline = self.sim_time + timeout_s
        real_deadline = time.monotonic() + timeout_s / TIME_SCALE + \
            REAL_TIME_MARGIN_S
        while (not future.done() and self.sim_time < sim_deadline and
               time.monotonic() < real_deadline):
            yield

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
            self._advance(0.02)

        goal = ExecuteCoverage.Goal()
        goal.task = _task()
        send_future = self.client.send_goal_async(
            goal, feedback_callback=self._feedback_callback)
        for _ in self._pending(send_future, 12.0):
            self._advance()
        self.assertTrue(send_future.done())
        goal_handle = send_future.result()
        self.assertTrue(goal_handle.accepted)
        result_future = goal_handle.get_result_async()

        step = SIM_STEP_S
        max_linear_while_turning = 0.0
        for _ in self._pending(result_future, 40.0):
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
            self._advance(step)

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
            self._advance(0.02)
        goal = ExecuteCoverage.Goal()
        goal.task = _task()
        send_future = self.client.send_goal_async(goal)
        for _ in self._pending(send_future, 12.0):
            self._publish_odometry(0.0, 0.0, 0.0)
            self._advance()
        goal_handle = send_future.result()
        self.assertTrue(goal_handle.accepted)
        cancel_future = goal_handle.cancel_goal_async()
        for _ in self._pending(cancel_future, 12.0):
            self._publish_odometry(0.0, 0.0, 0.0)
            self._advance()
        self.assertTrue(cancel_future.done())
        self.assertGreater(len(cancel_future.result().goals_canceling), 0)
        result_future = goal_handle.get_result_async()
        for _ in self._pending(result_future, 12.0):
            self._publish_odometry(0.0, 0.0, 0.0)
            self._advance()
        wrapped = result_future.result()
        self.assertEqual(wrapped.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(
            wrapped.result.result_code, ExecuteCoverage.Result.CANCELED)
        self._advance(0.05)
        linear, angular, _ = self._current_command()
        self.assertEqual((linear, angular), (0.0, 0.0))

    def test_approaches_first_waypoint_before_counting_scan_segments(self):
        """A distant start enters the first scan only after turn-slip recovery."""
        self.assertTrue(self.client.wait_for_server(timeout_sec=3.0))
        x, y, yaw = 0.0, -0.20, 0.0
        feedback_states = []
        for _ in range(5):
            self._publish_odometry(x, y, yaw)
            self._advance(0.02)
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
        for _ in self._pending(send_future, 45.0):
            self._advance()
        self.assertTrue(send_future.done())
        goal_handle = send_future.result()
        self.assertTrue(goal_handle.accepted)
        result_future = goal_handle.get_result_async()

        step = SIM_STEP_S
        first_scan_y = None
        for _ in self._pending(result_future, 45.0):
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
            self._advance(step)

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
            self._advance(0.02)

        goal = ExecuteCoverage.Goal()
        goal.task = _task()
        goal.task.task_id = 'outside-motion-region-' + 'x' * 48
        send_future = self.client.send_goal_async(goal)
        for _ in self._pending(send_future, 12.0):
            self._advance()
        self.assertTrue(send_future.done())
        handle = send_future.result()
        self.assertTrue(handle.accepted)

        result_future = handle.get_result_async()
        for _ in self._pending(result_future, 12.0):
            self._advance()
        self.assertTrue(result_future.done())
        self.assertEqual(
            result_future.result().result.result_code,
            ExecuteCoverage.Result.OUT_OF_BOUNDS)

        # The server is still healthy and no longer holds the released task.
        for _ in range(5):
            self._publish_odometry(0.0, 0.0, 0.0)
            self._advance(0.02)
        second = ExecuteCoverage.Goal()
        second.task = _task()
        second_future = self.client.send_goal_async(second)
        for _ in self._pending(second_future, 12.0):
            self._advance()
        self.assertTrue(second_future.done())
        self.assertTrue(second_future.result().accepted)
        second_future.result().cancel_goal_async()

    def test_refuses_a_first_scan_entry_whose_drop_cannot_be_reserved(self):
        """Approaching backwards onto a line pinned to the region's top edge."""
        # The entry leg's end is normally lifted by the drop of the turn
        # waiting at it. Here the scan line sits on the motion region's upper
        # boundary, so there is nowhere to lift it to: the turn of about 169
        # deg drops about 85 mm, all of it normal to a horizontal scan line,
        # and none of it can be reserved. That exceeds what the scan entry can
        # recover, and the run has to be refused before it drives.
        self.assertTrue(self.client.wait_for_server(timeout_sec=3.0))
        for _ in range(5):
            self._publish_odometry(0.50, -0.10, 0.0)
            self._advance(0.02)

        goal = ExecuteCoverage.Goal()
        goal.task = _task()
        goal.task.waypoints = [_pose(0.0, 0.0, 0.0), _pose(0.60, 0.0, 0.0)]
        goal.task.segment_types = [CoverageTask.SEGMENT_SCAN]
        for polygon in (goal.task.coverage_region, goal.task.motion_region):
            del polygon.points[:]
            for x, y in [(0.0, -0.5), (0.75, -0.5), (0.75, 0.0), (0.0, 0.0)]:
                point = Point32()
                point.x = x
                point.y = y
                polygon.points.append(point)

        send_future = self.client.send_goal_async(goal)
        for _ in self._pending(send_future, 12.0):
            self._advance()
        self.assertTrue(send_future.done())
        handle = send_future.result()
        self.assertTrue(handle.accepted)

        result_future = handle.get_result_async()
        for _ in self._pending(result_future, 15.0):
            self._publish_odometry(0.50, -0.10, 0.0)
            self._advance(0.02)
        self.assertTrue(result_future.done())
        result = result_future.result().result
        self.assertEqual(
            result.result_code, ExecuteCoverage.Result.TRACKING_FAILED)
        self.assertIn('scan entry can recover', result.message)
        self.assertEqual(result.completed_segments, 0)

    def test_the_same_backwards_entry_succeeds_when_the_drop_can_be_reserved(self):
        """The case the retired runway point could not serve."""
        # Identical approach, identical 169 deg turn, but with room above the
        # scan line. The entry leg's end is lifted by the drop instead of the
        # robot driving past its own first waypoint to enter along the line,
        # which is what the runway made it do - and the runway could not be
        # placed here at all, so this used to be refused outright.
        self.assertTrue(self.client.wait_for_server(timeout_sec=3.0))
        x, y, yaw = 0.50, 0.10, 0.0
        for _ in range(5):
            self._publish_odometry(x, y, yaw)
            self._advance(0.02)

        goal = ExecuteCoverage.Goal()
        goal.task = _task()
        goal.task.waypoints = [_pose(0.0, 0.0, 0.0), _pose(0.60, 0.0, 0.0)]
        goal.task.segment_types = [CoverageTask.SEGMENT_SCAN]
        for polygon in (goal.task.coverage_region, goal.task.motion_region):
            del polygon.points[:]
            for corner in [(-0.4, -0.5), (0.9, -0.5), (0.9, 0.5), (-0.4, 0.5)]:
                point = Point32()
                point.x = corner[0]
                point.y = corner[1]
                polygon.points.append(point)

        send_future = self.client.send_goal_async(goal)
        for _ in self._pending(send_future, 12.0):
            self._publish_odometry(x, y, yaw)
            self._advance()
        self.assertTrue(send_future.done())
        handle = send_future.result()
        self.assertTrue(handle.accepted)
        result_future = handle.get_result_async()

        step = SIM_STEP_S
        for _ in self._pending(result_future, 60.0):
            linear, angular, _ = self._current_command()
            yaw_delta = angular * step
            y -= 0.0005 * abs(math.degrees(yaw_delta))
            yaw += yaw_delta
            x += linear * math.cos(yaw) * step
            y += linear * math.sin(yaw) * step
            self._publish_odometry(x, y, yaw, linear, angular)
            self._advance(step)
        self.assertTrue(result_future.done())
        result = result_future.result()
        self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(result.result.completed_segments, 1)
        # It reached the far end of the scan without the detour the runway
        # forced, and on the line rather than a turn-drop below it.
        self.assertLess(math.hypot(x - 0.60, y), 0.05)

    def test_rejects_structurally_invalid_task(self):
        """An empty task never becomes an executable goal."""
        self.assertTrue(self.client.wait_for_server(timeout_sec=3.0))
        goal = ExecuteCoverage.Goal()
        send_future = self.client.send_goal_async(goal)
        for _ in self._pending(send_future, 12.0):
            self._advance()
        self.assertTrue(send_future.done())
        self.assertFalse(send_future.result().accepted)
