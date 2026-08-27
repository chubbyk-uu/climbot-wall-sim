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

"""
Pausing a coverage task before its first pose has ever arrived.

A tracker latches have_pose_ for the life of its process, so this state can
only be reached once and never again in the same node. It gets a launch test of
its own for that reason, and it is worth one: it is the only pause that has to
hold the start-up localization grace open as well as the segment deadline, and
the only one where nothing has to be brought to a stop first.
"""

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
import launch_testing.asserts
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.action import ActionClient
from rosgraph_msgs.msg import Clock
from std_srvs.srv import SetBool


# cmd_vel_watchdog's command_timeout_s default, mirrored so the plant coasts
# for exactly as long as the real one would.
COMMAND_TIMEOUT_S = 0.40

SIM_STEP_S = 0.01
TIME_SCALE = 10.0
REAL_TIME_MARGIN_S = 20.0

# Deliberately shorter than the pauses this file takes. A pause that failed to
# freeze the segment deadline would end the task instead of holding it, and
# that is the difference these tests are looking for.
SEGMENT_TIMEOUT_S = 8.0
PAUSE_HOLD_S = 12.0

# The plant slips downhill while turning in place, as the real one does.
TURN_SLIP_M_PER_DEG = 0.0005


@pytest.mark.launch_test
def generate_test_description():
    """Start the tracker in task-execution mode with a short segment deadline."""
    executor = launch_ros.actions.Node(
        package='climbot_control',
        executable='line_tracker_node',
        parameters=[{
            'standalone_mode': False,
            'use_sim_time': True,
            'odometry_timeout_s': 2.0,
            'segment_timeout_s': SEGMENT_TIMEOUT_S,
            'pause_stop_timeout_s': 5.0,
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
    """Scan, transition, scan - so one run visits every execution state."""
    task = CoverageTask()
    task.header.frame_id = 'odom'
    task.task_id = 'pause-test'
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
    for x, y in [(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)]:
        point = Point32()
        point.x = x
        point.y = y
        task.coverage_region.points.append(point)
        task.motion_region.points.append(point)
    task.detection_width = 0.10
    task.detection_length = 0.10
    return task


class TestPauseBeforeFirstPose(unittest.TestCase):
    """Close a planar plant around the Action server and interrupt it."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('coverage_executor_pause_test')
        self.command = Twist()
        self.command_time = None
        self.sim_time = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.state = ExecuteCoverage.Feedback.WAITING
        self.segment = -1
        self.segment_type = CoverageTask.SEGMENT_SCAN
        self.lock = Lock()
        self.publisher = self.node.create_publisher(
            Odometry, '/odometry/filtered', 10)
        self.clock_publisher = self.node.create_publisher(Clock, '/clock', 10)
        self.node.create_subscription(
            Twist, '/control/cmd_vel', self._command_callback, 10)
        self.client = ActionClient(
            self.node, ExecuteCoverage, '/coverage/execute')
        self.pause_client = self.node.create_client(
            SetBool, '/coverage/executor_pause')
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
            self.state = message.feedback.state
            self.segment = message.feedback.current_segment
            self.segment_type = message.feedback.segment_type

    def _current_command(self):
        """Return the command the wheels would actually be given."""
        with self.lock:
            stale = (
                self.command_time is None or
                self.sim_time - self.command_time > COMMAND_TIMEOUT_S)
            if stale:
                return (0.0, 0.0)
            return (self.command.linear.x, self.command.angular.z)

    def _observed(self):
        with self.lock:
            return (self.state, self.segment, self.segment_type)

    def _advance(self, step=SIM_STEP_S):
        with self.lock:
            self.sim_time += step
            now = self.sim_time
        message = Clock()
        message.clock.sec = int(now)
        message.clock.nanosec = int(round((now - int(now)) * 1e9))
        self.clock_publisher.publish(message)
        time.sleep(step / TIME_SCALE)

    def _publish_odometry(self, linear=0.0, angular=0.0):
        message = Odometry()
        message.pose.pose = _pose(self.x, self.y, self.yaw)
        message.twist.twist.linear.x = linear * math.cos(self.yaw)
        message.twist.twist.linear.y = linear * math.sin(self.yaw)
        message.twist.twist.angular.z = angular
        self.publisher.publish(message)

    def _tick(self):
        """Integrate one plant step from the command the wheels would see."""
        linear, angular = self._current_command()
        yaw_delta = angular * SIM_STEP_S
        self.y -= TURN_SLIP_M_PER_DEG * abs(math.degrees(yaw_delta))
        self.yaw += yaw_delta
        self.x += linear * math.cos(self.yaw) * SIM_STEP_S
        self.y += linear * math.sin(self.yaw) * SIM_STEP_S
        self._publish_odometry(linear, angular)
        self._advance()
        return linear, angular

    def _drive_until(self, predicate, timeout_s, description):
        """Run the plant until a predicate holds, bounded in simulated time."""
        sim_deadline = self.sim_time + timeout_s
        real_deadline = time.monotonic() + timeout_s / TIME_SCALE + \
            REAL_TIME_MARGIN_S
        while self.sim_time < sim_deadline and time.monotonic() < real_deadline:
            if predicate():
                return
            self._tick()
        self.fail('timed out waiting for ' + description)

    def _drive_for(self, duration_s):
        end = self.sim_time + duration_s
        while self.sim_time < end:
            self._tick()

    def _call_pause(self, value, drive=True, timeout_s=10.0):
        """Ask the executor to pause or resume, optionally driving meanwhile."""
        self.assertTrue(self.pause_client.wait_for_service(timeout_sec=15.0))
        request = SetBool.Request()
        request.data = value
        future = self.pause_client.call_async(request)
        real_deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < real_deadline:
            if drive:
                self._tick()
            else:
                time.sleep(0.005)
        self.assertTrue(future.done(), 'the executor never answered the pause service')
        return future.result()

    def _start_task(self, with_odometry=True):
        self.assertTrue(self.client.wait_for_server(timeout_sec=15.0))
        for _ in range(5):
            if with_odometry:
                self._publish_odometry()
            self._advance(0.02)
        goal = ExecuteCoverage.Goal()
        goal.task = _task()
        send_future = self.client.send_goal_async(
            goal, feedback_callback=self._feedback_callback)
        real_deadline = time.monotonic() + 20.0
        while not send_future.done() and time.monotonic() < real_deadline:
            if with_odometry:
                self._publish_odometry()
            self._advance()
        self.assertTrue(send_future.done())
        handle = send_future.result()
        self.assertTrue(handle.accepted)
        return handle

    def test_pause_before_the_first_pose_holds_the_task(self):
        """A task paused before localization arrives keeps its start-up grace."""
        handle = self._start_task(with_odometry=False)
        result_future = handle.get_result_async()
        response = self._call_pause(True, drive=False)
        self.assertTrue(response.success, response.message)

        # No odometry at all: the clock advances and nothing else does. There
        # is no motion to bring to a stop here, so PAUSED is reached on the
        # first control cycle that sees the request.
        deadline = self.sim_time + 2.0
        while (self.sim_time < deadline and
               self._observed()[0] != ExecuteCoverage.Feedback.PAUSED):
            self._advance()
        self.assertEqual(self._observed()[0], ExecuteCoverage.Feedback.PAUSED)

        # Well past both the localization grace and the segment deadline,
        # still with no pose published. Neither may fire.
        end = self.sim_time + PAUSE_HOLD_S
        while self.sim_time < end:
            self._advance()
        self.assertFalse(
            result_future.done(),
            'a deadline expired while the task was paused without a pose')

        response = self._call_pause(False, drive=False)
        self.assertTrue(response.success, response.message)
        self._drive_until(lambda: result_future.done(), 240.0, 'the task to finish')
        wrapped = result_future.result()
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(
            wrapped.result.result_code, ExecuteCoverage.Result.SUCCESS)
        self.assertEqual(wrapped.result.completed_segments, 3)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    """A crashed executor must fail the launch test."""

    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
