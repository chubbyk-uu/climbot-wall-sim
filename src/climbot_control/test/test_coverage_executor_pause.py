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
Executor-side verification of pausing and resuming a coverage task.

Pausing a scan in mid-line is the easy case and the one least likely to be
wrong. What this file exercises instead is every other moment a task can be
interrupted at - before the first pose has ever arrived, on the way to the
start, in an in-place turn, while the turn settles, on a transition, and inside
the final approach - because each of those reads a different deadline, and a
pause that forgets one of them is only visible in that state.

The one state this file cannot reach is a task paused before its first pose
ever arrived: have_pose_ latches for the life of the process, so that case
needs a tracker of its own and lives in test_coverage_executor_pause_start.py.
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
from climbot_interfaces.msg import ExecutionReference
from climbot_interfaces.msg import InspectionCaptureGate
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
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
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

# The tracker's own stopped_linear_speed_mps default, which is what it uses to
# decide that motion has ended.
STOPPED_LINEAR_SPEED_MPS = 0.01

# Deliberately not the first waypoint, and facing away from it. A run that
# starts already on the line reaches its first scan within one control cycle,
# and the approach and the alignment before it are then over before a test can
# interrupt either of them.
START_X = -0.20
START_Y = -0.12
START_YAW = math.pi


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
            # This file supplies the heartbeat itself; keep the deadlines
            # tolerant of loaded CI rather than testing DDS timing here.
            'capture_gate_timeout_s': 2.0,
            'capture_gate_start_timeout_s': 2.0,
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


def _long_scan_task():
    """
    Build one scan line long enough that the robot reaches and holds cruise.

    The three-segment task above is 0.20 m a side, which the acceleration and
    braking ramps consume entirely - there is no plateau on it to compare a
    resumed speed against.
    """
    task = CoverageTask()
    task.header.frame_id = 'odom'
    task.task_id = 'pause-test'
    task.revision = 1
    task.sweep_direction = CoverageTask.SWEEP_HORIZONTAL
    task.waypoints = [_pose(0.0, 0.0, 0.0), _pose(0.80, 0.0, 0.0)]
    task.segment_types = [CoverageTask.SEGMENT_SCAN]
    for x, y in [(-0.6, -0.6), (1.4, -0.6), (1.4, 0.6), (-0.6, 0.6)]:
        point = Point32()
        point.x = x
        point.y = y
        task.coverage_region.points.append(point)
        task.motion_region.points.append(point)
    task.detection_width = 0.10
    task.detection_length = 0.10
    return task


class TestCoverageExecutorPause(unittest.TestCase):
    """Close a planar plant around the Action server and interrupt it."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('coverage_executor_pause_test')
        self.command = Twist()
        self.command_time = None
        self.sim_time = 0.0
        self.x = START_X
        self.y = START_Y
        self.yaw = START_YAW
        self.state = ExecuteCoverage.Feedback.WAITING
        self.plant_speed = 0.0
        self.segment = -1
        self.segment_type = CoverageTask.SEGMENT_SCAN
        self.lock = Lock()
        self.publisher = self.node.create_publisher(
            Odometry, '/odometry/filtered', 10)
        self.clock_publisher = self.node.create_publisher(Clock, '/clock', 10)
        self.node.create_subscription(
            Twist, '/control/cmd_vel', self._command_callback, 10)
        self.references = []
        self.node.create_subscription(
            ExecutionReference, '/control/execution_reference',
            self._reference_callback, 10)
        self.gate_publisher = self.node.create_publisher(
            InspectionCaptureGate, '/inspection/capture_gate',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
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

    def _reference_callback(self, message):
        with self.lock:
            # Paired with the speed the plant was actually running at when the
            # reference was published, because the claim under test is about
            # exactly that pairing.
            self.references.append((message.inspection_enabled, self.plant_speed))

    def _publish_capture_gate(self, segment):
        """Stand in for the capture node's liveness heartbeat."""
        gate = InspectionCaptureGate()
        gate.task_id = 'pause-test'
        gate.revision = 1
        gate.segment_index = segment
        gate.active = False
        gate.reason = 'pause test heartbeat'
        self.gate_publisher.publish(gate)

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
        with self.lock:
            self.plant_speed = abs(linear)
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

    def _start_task_with_inspection(self):
        """Start the same task as an archived inspection run."""
        self.assertTrue(self.client.wait_for_server(timeout_sec=15.0))
        for _ in range(5):
            self._publish_odometry()
            self._publish_capture_gate(0)
            self._advance(0.02)
        goal = ExecuteCoverage.Goal()
        goal.task = _task()
        goal.inspection_enabled = True
        send_future = self.client.send_goal_async(
            goal, feedback_callback=self._feedback_callback)
        real_deadline = time.monotonic() + 20.0
        while not send_future.done() and time.monotonic() < real_deadline:
            self._publish_odometry()
            self._publish_capture_gate(0)
            self._advance()
        self.assertTrue(send_future.done())
        handle = send_future.result()
        self.assertTrue(handle.accepted)
        return handle

    def _start_task(self, with_odometry=True, task=None):
        self.assertTrue(self.client.wait_for_server(timeout_sec=15.0))
        for _ in range(5):
            if with_odometry:
                self._publish_odometry()
            self._advance(0.02)
        goal = ExecuteCoverage.Goal()
        goal.task = _task() if task is None else task
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

    def _pause_and_resume(self, hold_s=0.4):
        """Stop the task where it stands, hold it there, and let it continue."""
        response = self._call_pause(True)
        self.assertTrue(response.success, response.message)
        self._drive_until(
            lambda: self._observed()[0] == ExecuteCoverage.Feedback.PAUSED,
            10.0, 'the executor to report PAUSED')

        # Held, not merely quiet: every cycle of the hold has to command an
        # exact zero, because the wheel watchdog would mask a controller that
        # simply stopped publishing and the task would look identical.
        self._drive_for(hold_s)
        for _ in range(20):
            linear, angular = self._current_command()
            self.assertEqual((linear, angular), (0.0, 0.0))
            self.assertEqual(
                self._observed()[0], ExecuteCoverage.Feedback.PAUSED)
            self._tick()

        response = self._call_pause(False)
        self.assertTrue(response.success, response.message)
        self._drive_until(
            lambda: self._observed()[0] not in (
                ExecuteCoverage.Feedback.PAUSED,
                ExecuteCoverage.Feedback.PAUSING),
            10.0, 'the executor to leave the pause states')

    def test_pause_and_resume_in_every_execution_state(self):
        """One run is interrupted once in each state and still completes."""
        handle = self._start_task()
        result_future = handle.get_result_async()

        # (label, predicate on the last feedback) in the order the run meets
        # them. current_segment is -1 only before the first segment starts.
        targets = [
            ('approach_start',
             lambda state, segment, kind:
                 state == ExecuteCoverage.Feedback.APPROACH_START),
            ('scan_align',
             lambda state, segment, kind:
                 state == ExecuteCoverage.Feedback.ALIGN and segment == 0),
            ('turn_settle',
             lambda state, segment, kind:
                 state == ExecuteCoverage.Feedback.TURN_SETTLE),
            ('scan_track',
             lambda state, segment, kind:
                 state == ExecuteCoverage.Feedback.TRACK_LINE and
                 kind == CoverageTask.SEGMENT_SCAN),
            ('final_approach',
             lambda state, segment, kind:
                 state == ExecuteCoverage.Feedback.FINAL_APPROACH),
            ('transition_track',
             lambda state, segment, kind:
                 state == ExecuteCoverage.Feedback.TRACK_LINE and
                 kind == CoverageTask.SEGMENT_TRANSITION),
        ]
        exercised = []
        pending = list(targets)

        sim_deadline = self.sim_time + 240.0
        real_deadline = time.monotonic() + 240.0 / TIME_SCALE + REAL_TIME_MARGIN_S
        while (not result_future.done() and self.sim_time < sim_deadline and
               time.monotonic() < real_deadline):
            state, segment, kind = self._observed()
            matched = next(
                (entry for entry in pending if entry[1](state, segment, kind)),
                None)
            if matched is not None:
                pending.remove(matched)
                # The segment the task resumes on has to be the segment it
                # paused on; a pause that restarted the run would still finish
                # and still report success.
                before = self._observed()[1]
                self._pause_and_resume()
                self.assertEqual(
                    self._observed()[1], before,
                    'the task changed segment across the ' + matched[0] + ' pause')
                exercised.append(matched[0])
                continue
            self._tick()

        self.assertTrue(result_future.done(), 'the task never finished')
        self.assertEqual(
            [entry[0] for entry in pending], [],
            'these execution states were never paused: ' +
            repr([entry[0] for entry in pending]))
        self.assertEqual(len(exercised), len(targets))
        wrapped = result_future.result()
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(
            wrapped.result.result_code, ExecuteCoverage.Result.SUCCESS)
        self.assertEqual(wrapped.result.completed_segments, 3)

    def test_a_paused_scan_keeps_capturing_until_the_robot_stops(self):
        """
        Close the capture gate on the standstill, not on the request.

        A pause masks the reported execution state immediately, but the robot
        still has its whole braking distance to travel. Withdrawing the SCAN
        reference at the request skipped every trigger target inside those
        centimetres; the exposure then fired on resume from the standstill,
        far enough past its target that the recorder rejected it and failed
        the run. So what the reference has to follow is the robot, not the
        announcement.
        """
        handle = self._start_task_with_inspection()
        result_future = handle.get_result_async()

        def tick():
            self._publish_capture_gate(max(0, self._observed()[1]))
            self._tick()

        # Reach a scan line at speed, which is the only place this can go wrong.
        sim_deadline = self.sim_time + 60.0
        real_deadline = time.monotonic() + 60.0 / TIME_SCALE + REAL_TIME_MARGIN_S
        while self.sim_time < sim_deadline and time.monotonic() < real_deadline:
            state, segment, kind = self._observed()
            if (state == ExecuteCoverage.Feedback.TRACK_LINE and
                    kind == CoverageTask.SEGMENT_SCAN and
                    self._current_command()[0] > 0.05):
                break
            tick()
        else:
            self.fail('the scan line never reached speed')

        with self.lock:
            self.references.clear()
        self.assertTrue(self._call_pause(True).success)
        self._drive_until(
            lambda: self._observed()[0] == ExecuteCoverage.Feedback.PAUSED,
            10.0, 'the executor to report PAUSED')
        # A few more cycles at a standstill, so the closed half is not empty.
        for _ in range(20):
            tick()

        with self.lock:
            samples = list(self.references)
        self.assertTrue(samples, 'the executor published no execution reference')
        moving_but_closed = [
            speed for enabled, speed in samples
            if not enabled and speed > STOPPED_LINEAR_SPEED_MPS]
        self.assertFalse(
            moving_but_closed,
            'the SCAN reference was withdrawn while the robot was still '
            'travelling at up to %.3f m/s, which skips every trigger target '
            'inside the braking distance' % max(moving_but_closed or [0.0]))
        self.assertFalse(
            samples[-1][0],
            'the SCAN reference was still enabled after the robot stopped')

        self.assertTrue(self._call_pause(False).success)
        self._drive_until(
            lambda: self._observed()[0] not in (
                ExecuteCoverage.Feedback.PAUSED,
                ExecuteCoverage.Feedback.PAUSING),
            10.0, 'the executor to leave the pause states')
        handle.cancel_goal_async()
        self._drive_until(lambda: result_future.done(), 30.0, 'the canceled result')

    def test_resuming_a_scan_does_not_sprint_to_catch_up(self):
        """
        Resume from the standstill, not from the middle of the speed curve.

        The travel curve is time-parameterised from a standing start. Resuming
        it where it was left asks the robot to already be at cruise while it is
        at zero, and in time mode the schedule correction reads that gap as lag
        and works it off at catch_up_max_linear_speed. The deceleration adds to
        it: the curve advances at cruise for the whole braking time while the
        robot covers half the distance. Both are repaired by planning again
        from where the robot actually stands.
        """
        handle = self._start_task(task=_long_scan_task())
        result_future = handle.get_result_async()
        self._drive_until(
            lambda: self._observed()[0] == ExecuteCoverage.Feedback.TRACK_LINE and
            self._current_command()[0] > 0.05,
            60.0, 'the scan line to reach speed')

        # What the schedule asks for when nothing has interrupted it.
        cruising = 0.0
        for _ in range(150):
            cruising = max(cruising, self._current_command()[0])
            self._tick()
        self.assertGreater(cruising, 0.10, 'the scan line never reached cruise')

        self._pause_and_resume(hold_s=0.3)

        resumed = 0.0
        for _ in range(250):
            resumed = max(resumed, self._current_command()[0])
            self._tick()
        self.assertLessEqual(
            resumed, cruising * 1.10 + 0.005,
            'resuming commanded %.3f m/s against %.3f m/s before the pause, so the '
            'speed curve was resumed in its middle rather than replanned from the '
            'standstill' % (resumed, cruising))

        handle.cancel_goal_async()
        self._drive_until(lambda: result_future.done(), 30.0, 'the canceled result')

    def test_a_pause_inside_the_turn_profile_replans_the_turn(self):
        """
        Reach the in-place turn itself, which the state feedback cannot name.

        feedbackState() reports WAITING_FOR_ALIGNMENT, ALIGN_BRAKE,
        ALIGN_PROFILE and ARC_ENTRY all as ALIGN, so the every-state test above
        pauses in whichever comes first - the brake, not the turn. Only
        ALIGN_PROFILE is running a time-parameterised curve, and it is the one
        the resume path replans, so it needs reaching deliberately. It is the
        only ALIGN sub-state that commands angular speed with no linear speed;
        an arc entry drives forward while it turns.
        """
        handle = self._start_task()
        result_future = handle.get_result_async()
        self._drive_until(
            lambda: self._observed()[0] == ExecuteCoverage.Feedback.ALIGN and
            abs(self._current_command()[1]) > 0.2 and
            abs(self._current_command()[0]) < 1e-3,
            60.0, 'an in-place turn to reach speed')
        segment = self._observed()[1]

        self._pause_and_resume(hold_s=0.3)
        self.assertEqual(
            self._observed()[1], segment,
            'the task changed segment across a pause inside the turn')

        self._drive_until(lambda: result_future.done(), 240.0, 'the task to finish')
        wrapped = result_future.result()
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(wrapped.result.completed_segments, 3)

    def test_a_pause_outlasts_the_segment_deadline(self):
        """The segment timeout is frozen, not merely generous."""
        handle = self._start_task()
        result_future = handle.get_result_async()
        self._drive_until(
            lambda: self._observed()[0] == ExecuteCoverage.Feedback.TRACK_LINE,
            60.0, 'the first scan line')

        response = self._call_pause(True)
        self.assertTrue(response.success, response.message)
        self._drive_until(
            lambda: self._observed()[0] == ExecuteCoverage.Feedback.PAUSED,
            10.0, 'the executor to report PAUSED')

        # Longer than the segment deadline the executor was started with.
        self.assertGreater(PAUSE_HOLD_S, SEGMENT_TIMEOUT_S)
        self._drive_for(PAUSE_HOLD_S)
        self.assertFalse(
            result_future.done(),
            'the segment deadline expired while the task was paused')
        self.assertEqual(self._observed()[0], ExecuteCoverage.Feedback.PAUSED)

        response = self._call_pause(False)
        self.assertTrue(response.success, response.message)
        self._drive_until(lambda: result_future.done(), 240.0, 'the task to finish')
        wrapped = result_future.result()
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(wrapped.result.completed_segments, 3)

    def test_stop_during_a_pause_cancels_the_task(self):
        """Cancel keeps its meaning while the task is held at zero."""
        handle = self._start_task()
        result_future = handle.get_result_async()
        self._drive_until(
            lambda: self._observed()[0] == ExecuteCoverage.Feedback.TRACK_LINE,
            60.0, 'the first scan line')
        self.assertTrue(self._call_pause(True).success)
        self._drive_until(
            lambda: self._observed()[0] == ExecuteCoverage.Feedback.PAUSED,
            10.0, 'the executor to report PAUSED')

        cancel_future = handle.cancel_goal_async()
        self._drive_until(lambda: cancel_future.done(), 20.0, 'the cancel response')
        self.assertGreater(len(cancel_future.result().goals_canceling), 0)
        self._drive_until(lambda: result_future.done(), 20.0, 'the canceled result')
        wrapped = result_future.result()
        self.assertEqual(wrapped.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(
            wrapped.result.result_code, ExecuteCoverage.Result.CANCELED)
        self._drive_for(0.1)
        self.assertEqual(self._current_command(), (0.0, 0.0))

    def test_repeated_pause_and_resume_keeps_one_task(self):
        """Three cycles on one line neither restart the task nor lose it."""
        handle = self._start_task()
        result_future = handle.get_result_async()
        self._drive_until(
            lambda: self._observed()[0] == ExecuteCoverage.Feedback.TRACK_LINE,
            60.0, 'the first scan line')
        segment = self._observed()[1]
        for _ in range(3):
            self._pause_and_resume(hold_s=0.2)
            self.assertEqual(self._observed()[1], segment)
        self._drive_until(lambda: result_future.done(), 240.0, 'the task to finish')
        wrapped = result_future.result()
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(wrapped.result.completed_segments, 3)

    def test_resume_is_refused_while_the_robot_is_still_decelerating(self):
        """PAUSED means stopped, so nothing may resume out of PAUSING."""
        handle = self._start_task()
        result_future = handle.get_result_async()
        self._drive_until(
            lambda: self._observed()[0] == ExecuteCoverage.Feedback.TRACK_LINE and
            self._current_command()[0] > 0.05,
            60.0, 'the scan line to reach speed')
        self.assertTrue(self._call_pause(True, drive=False).success)
        # Simulated time is frozen here, so the control loop cannot have
        # advanced the deceleration between the two calls.
        response = self._call_pause(False, drive=False)
        self.assertFalse(response.success)
        self.assertIn('decelerating', response.message)
        self._drive_until(
            lambda: self._observed()[0] == ExecuteCoverage.Feedback.PAUSED,
            10.0, 'the executor to report PAUSED')
        self.assertTrue(self._call_pause(False).success)
        self._drive_until(lambda: result_future.done(), 240.0, 'the task to finish')
        self.assertEqual(
            result_future.result().status, GoalStatus.STATUS_SUCCEEDED)

    def test_pause_is_refused_without_an_active_task(self):
        """Nothing to pause is an explained refusal, not a silent success."""
        response = self._call_pause(True, drive=False)
        self.assertFalse(response.success)
        self.assertIn('No active coverage task', response.message)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    """A crashed executor must fail the launch test."""

    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
