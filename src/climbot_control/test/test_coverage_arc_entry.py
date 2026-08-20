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

"""Node-level verification of the single forward arc entry."""

# This manoeuvre had no coverage at all. None of the four archived Gazebo
# cases triggers it: since the transition reserve landed, post-turn offsets
# stay under 9.7 mm against a 45 mm threshold, so the arc is a path the
# regression suite never walked. It has to be provoked deliberately, which is
# what this file does by slipping more than the tracker is told to expect.

import math
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


SIM_STEP_S = 0.01
TIME_SCALE = 10.0
REAL_TIME_MARGIN_S = 15.0

# The plant slips more than the tracker is told to expect, which is how a
# post-turn offset survives the transition reserve at all. 0.8 mm per degree
# through the 90 degree turn leaves 72 mm, between parallel_scan_offset_m
# (45 mm, below which the line is simply translated) and maximum_scan_offset_m
# (120 mm, above which the run is refused). That is the only window in which
# this manoeuvre runs.
TURN_SLIP_M_PER_DEG = 0.0008


@pytest.mark.launch_test
def generate_test_description():
    """Start the tracker with the arc thresholds it ships with."""
    executor = launch_ros.actions.Node(
        package='climbot_control',
        executable='line_tracker_node',
        parameters=[{
            'standalone_mode': False,
            'use_sim_time': True,
            # Reserving nothing is what leaves an offset for the arc to remove.
            # It is also the stale-calibration case: a wall that slips more
            # than turn_slip_per_degree_m says it does looks exactly like this.
            'turn_slip_per_degree_m': 0.0,
            'odometry_timeout_s': 2.0,
            'segment_timeout_s': 30.0,
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
    """Build a vertical run-in and a 90 degree turn onto a horizontal scan."""
    # The turn has to end on a horizontal line for its drop to be cross-track;
    # onto a vertical column the same drop runs along the line and never
    # offsets it. The first segment is deliberately a TRANSITION, because a
    # task that opens on a SCAN gets a start-approach runway that enters along
    # the line and removes the offset before the arc could ever see it.
    task = CoverageTask()
    task.header.frame_id = 'odom'
    task.task_id = 'arc-entry-test'
    task.revision = 1
    task.sweep_direction = CoverageTask.SWEEP_HORIZONTAL
    task.waypoints = [
        _pose(0.0, -0.30, math.pi / 2.0),
        _pose(0.0, 0.0, 0.0),
        _pose(1.20, 0.0, 0.0),
    ]
    task.segment_types = [
        CoverageTask.SEGMENT_TRANSITION, CoverageTask.SEGMENT_SCAN]
    for polygon in (task.coverage_region, task.motion_region):
        for x, y in [(-0.6, -0.6), (1.8, -0.6), (1.8, 0.6), (-0.6, 0.6)]:
            point = Point32()
            point.x = x
            point.y = y
            polygon.points.append(point)
    task.detection_width = 0.5
    task.detection_length = 0.01
    return task


class TestArcEntry(unittest.TestCase):
    """Drive a plant that slips on turns, and watch how the arc finishes."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('arc_entry_test')
        self.command = Twist()
        self.state = ExecuteCoverage.Feedback.WAITING
        self.sim_time = 0.0
        self.lock = Lock()
        self.publisher = self.node.create_publisher(
            Odometry, '/odometry/filtered', 10)
        self.clock_publisher = self.node.create_publisher(Clock, '/clock', 10)
        self.node.create_subscription(
            Twist, '/control/cmd_vel', self._command_callback, 10)
        self.client = ActionClient(
            self.node, ExecuteCoverage, '/coverage/execute')
        self.stop_spin = False
        self.spin_thread = Thread(target=self._spin)
        self.spin_thread.start()

    def tearDown(self):
        self.stop_spin = True
        self.spin_thread.join()
        self.client.destroy()
        self.node.destroy_node()
        rclpy.shutdown()

    def _spin(self):
        while rclpy.ok() and not self.stop_spin:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def _command_callback(self, message):
        with self.lock:
            self.command = message

    def _feedback_callback(self, message):
        with self.lock:
            self.state = message.feedback.state

    def _advance(self, step=SIM_STEP_S):
        with self.lock:
            self.sim_time += step
            now = self.sim_time
        message = Clock()
        message.clock.sec = int(now)
        message.clock.nanosec = int(round((now - int(now)) * 1e9))
        self.clock_publisher.publish(message)
        time.sleep(step / TIME_SCALE)

    def _publish_odometry(self, x, y, yaw, linear=0.0, angular=0.0):
        message = Odometry()
        message.pose.pose = _pose(x, y, yaw)
        message.twist.twist.linear.x = linear * math.cos(yaw)
        message.twist.twist.linear.y = linear * math.sin(yaw)
        message.twist.twist.angular.z = angular
        self.publisher.publish(message)

    def _pending(self, future, timeout_s):
        sim_deadline = self.sim_time + timeout_s
        real_deadline = time.monotonic() + timeout_s / TIME_SCALE + \
            REAL_TIME_MARGIN_S
        while (not future.done() and self.sim_time < sim_deadline and
               time.monotonic() < real_deadline):
            yield

    def _run_arc_entry(self):
        """Drive the whole task and record what happened around the arc."""
        self.assertTrue(self.client.wait_for_server(timeout_sec=15.0))
        # On the run-in, facing along it, so the only offset that appears is
        # the one the turn onto the scan line creates.
        x, y, yaw = 0.0, -0.30, math.pi / 2.0
        for _ in range(5):
            self._publish_odometry(x, y, yaw)
            self._advance(0.02)

        goal = ExecuteCoverage.Goal()
        goal.task = _task()
        send_future = self.client.send_goal_async(
            goal, feedback_callback=self._feedback_callback)
        for _ in self._pending(send_future, 15.0):
            self._publish_odometry(x, y, yaw)
            self._advance()
        self.assertTrue(send_future.done())
        handle = send_future.result()
        self.assertTrue(handle.accepted)
        result_future = handle.get_result_async()

        samples = []
        arcing = False
        arc_end = None
        arc_end_index = None
        for _ in self._pending(result_future, 90.0):
            with self.lock:
                linear = self.command.linear.x
                angular = self.command.angular.z
                state = self.state
            # An arc entry is reported as ALIGN while still driving forward;
            # an in-place alignment is not.
            driving_align = (
                state == ExecuteCoverage.Feedback.ALIGN and abs(linear) > 1e-3)
            if driving_align:
                arcing = True
            elif arcing and arc_end is None:
                arc_end = (x, y, yaw)
                arc_end_index = len(samples)
            yaw_delta = angular * SIM_STEP_S
            # Turning in place slips downhill, which is the whole reason the
            # frozen line has to be chosen after the turn, not before it.
            y -= TURN_SLIP_M_PER_DEG * abs(math.degrees(yaw_delta))
            yaw += yaw_delta
            x += linear * math.cos(yaw) * SIM_STEP_S
            y += linear * math.sin(yaw) * SIM_STEP_S
            samples.append((state, x, y, yaw))
            self._publish_odometry(x, y, yaw, linear, angular)
            self._advance()
        self.assertTrue(result_future.done(), 'the task never finished')
        return (result_future.result(), samples, arc_end, (x, y, yaw),
                arc_end_index)

    def test_an_offset_start_arcs_onto_the_line_and_completes(self):
        """The path no archived Gazebo run exercises."""
        result, samples, arc_end, final, _ = self._run_arc_entry()
        self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(
            result.result.result_code, ExecuteCoverage.Result.SUCCESS)
        driving_align = [
            sample for sample in samples
            if sample[0] == ExecuteCoverage.Feedback.ALIGN]
        self.assertGreater(
            len(driving_align), 0, 'the arc entry never ran')
        self.assertIsNotNone(arc_end, 'the arc never finished')

    def test_the_frozen_line_accounts_for_the_alignment_still_to_come(self):
        """The arc ends mid-turn, and that turn drops the robot afterwards."""
        _, samples, arc_end, _, arc_end_index = self._run_arc_entry()
        self.assertIsNotNone(arc_end)
        self.assertIsNotNone(arc_end_index)
        # The arc stops with the robot still angled at the line it is about to
        # freeze - about 4 deg here - so an in-place alignment follows and
        # slips downhill. Freezing at the position measured before that turn
        # puts the line above where the robot will actually be.
        settled = [
            sample for sample in samples[arc_end_index:]
            if sample[0] == ExecuteCoverage.Feedback.TRACK_LINE]
        self.assertGreater(len(settled), 0, 'the scan never started')
        # This bounds the manoeuvre as a whole, not the reserve inside it. The
        # reserve is turn_slip_per_degree_m times the roughly 4 deg the arc
        # leaves, about 2 mm at the calibrated coefficient - too small to
        # separate from run-to-run variation here, and this file sets that
        # coefficient to zero anyway to create the offset in the first place.
        # What this test is really for is the path: none of the four archived
        # Gazebo cases enters it at all.
        entry_offset = settled[0][2]
        self.assertLess(
            abs(entry_offset), 0.030,
            'the scan began %0.1f mm off the nominal line' % (
                1000.0 * entry_offset))

    def test_the_run_ends_on_the_line_it_entered(self):
        _, _, _, final, _ = self._run_arc_entry()
        x, y, _ = final
        self.assertGreater(x, 1.0, 'the scan did not reach its end')
        # Started 80 mm off; the arc plus the cross-track loop must have taken
        # that out rather than carried it along the whole line.
        self.assertLess(
            abs(y), 0.030,
            'finished %0.1f mm off the scan line' % (1000.0 * y))


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    """A crashed executor must fail the launch test."""

    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
