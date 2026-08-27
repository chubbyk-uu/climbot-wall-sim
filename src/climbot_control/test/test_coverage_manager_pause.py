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
What the manager publishes and permits around a pause.

The manager does not decide when a task is paused; the executor does, and the
manager echoes it. Nothing orders a service response against a feedback stream,
though, so feedback produced before the executor saw the request still arrives
after it. Echoing that made the panel flap between EXECUTING and PAUSING for as
long as the request took to land, which is exactly the moment an operator is
looking at it. The sequence assertions below are about that, not about whether
the robot stopped - test_coverage_executor_pause.py covers the stopping.
"""

import math
from threading import Event, Thread
import time
import unittest

from climbot_interfaces.msg import CoverageStatus, CoverageTask
from geometry_msgs.msg import Point32, Pose
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
from nav_msgs.msg import Odometry
import pytest
import rclpy
from std_srvs.srv import Trigger


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Run the real executor and manager, driven by synthetic odometry."""
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='climbot_control', executable='line_tracker_node',
            parameters=[{
                'standalone_mode': False,
                'odometry_timeout_s': 2.0,
                'segment_timeout_s': 60.0,
                'alignment_settle_duration_s': 0.05,
                'wheel_separation': 0.43,
                'wheel_speed_limit': 0.45,
                'wheel_acceleration_limit': 0.40,
            }]),
        launch_ros.actions.Node(
            package='climbot_control', executable='coverage_manager_node'),
        launch_testing.actions.ReadyToTest(),
    ])


# One revision per test, so waiting on a status can never be satisfied by the
# latched one the previous test left on the topic.
_revisions = iter(range(61, 200))

PAUSE_STATES = (CoverageStatus.PAUSING, CoverageStatus.PAUSED)


def _pose(x, y, yaw):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def _task(revision):
    task = CoverageTask()
    task.header.frame_id = 'odom'
    task.task_id = 'pause-manager-test'
    task.revision = revision
    task.sweep_direction = CoverageTask.SWEEP_HORIZONTAL
    task.waypoints = [_pose(0.0, 0.0, 0.0), _pose(0.4, 0.0, 0.0)]
    task.segment_types = [CoverageTask.SEGMENT_SCAN]
    for x, y in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]:
        point = Point32(x=x, y=y)
        task.coverage_region.points.append(point)
        task.motion_region.points.append(point)
    task.detection_width = 0.1
    task.detection_length = 0.1
    return task


class TestCoverageManagerPause(unittest.TestCase):
    """Pause and resume, from the side the operator's panel reads."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('coverage_manager_pause_test')
        self.statuses = []
        self.task_publisher = self.node.create_publisher(
            CoverageTask, '/coverage/task',
            rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))
        self.odom_publisher = self.node.create_publisher(
            Odometry, '/odometry/filtered', 10)
        # Deep enough to keep every transition. PAUSING can be passed through
        # in a single tick, and at depth 1 the reliable queue simply replaces
        # it with the PAUSED that follows - which is the shape these tests are
        # here to inspect.
        self.node.create_subscription(
            CoverageStatus, '/coverage/manager_status', self.statuses.append,
            rclpy.qos.QoSProfile(
                depth=100,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))
        self.start_client = self.node.create_client(Trigger, '/coverage/start')
        self.cancel_client = self.node.create_client(Trigger, '/coverage/cancel')
        self.pause_client = self.node.create_client(Trigger, '/coverage/pause')
        self.resume_client = self.node.create_client(Trigger, '/coverage/resume')
        self.stop_spin = Event()
        self.spin_thread = Thread(target=self._spin)
        self.spin_thread.start()
        for client in (self.start_client, self.cancel_client,
                       self.pause_client, self.resume_client):
            self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        self.revision = next(_revisions)

    def tearDown(self):
        self._call(self.cancel_client)
        self.stop_spin.set()
        self.spin_thread.join()
        self.node.destroy_node()
        rclpy.shutdown()

    def _spin(self):
        while rclpy.ok() and not self.stop_spin.is_set():
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def _call(self, client):
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            self._publish_odom()
            time.sleep(0.01)
        self.assertTrue(future.done(), 'Service call never returned.')
        return future.result()

    def _publish_odom(self):
        message = Odometry()
        message.header.frame_id = 'odom'
        message.pose.pose = _pose(0.0, 0.0, 0.0)
        self.odom_publisher.publish(message)

    def _latest(self):
        return self.statuses[-1] if self.statuses else None

    def _wait_until(self, predicate, what, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            latest = self._latest()
            if latest is not None and predicate(latest):
                return latest
            self._publish_odom()
            time.sleep(0.01)
        self.fail('Timed out waiting for {}; last status was {}'.format(
            what, self._latest()))

    def _states_since(self, mark):
        """Collapse the status history since a mark into a state sequence."""
        states = []
        for status in self.statuses[mark:]:
            if not states or states[-1] != status.state:
                states.append(status.state)
        return states

    def _reach_executing(self):
        self.task_publisher.publish(_task(self.revision))
        self._wait_until(
            lambda s: s.state == CoverageStatus.READY and
            s.revision == self.revision, 'the preview to be accepted')
        self.assertTrue(self._call(self.start_client).success)
        return self._wait_until(
            lambda s: s.state == CoverageStatus.EXECUTING, 'execution to start')

    def _pause(self):
        mark = len(self.statuses)
        response = self._call(self.pause_client)
        self.assertTrue(response.success, response.message)
        paused = self._wait_until(
            lambda s: s.state == CoverageStatus.PAUSED, 'the task to pause')
        return mark, paused

    def _resume(self):
        mark = len(self.statuses)
        response = self._call(self.resume_client)
        self.assertTrue(response.success, response.message)
        executing = self._wait_until(
            lambda s: s.state == CoverageStatus.EXECUTING, 'the task to resume')
        return mark, executing

    def test_pause_and_resume_move_the_permissions_with_the_state(self):
        """The panel's buttons follow the state, and only one is ever legal."""
        executing = self._reach_executing()
        self.assertTrue(executing.can_pause)
        self.assertFalse(executing.can_resume)

        _, paused = self._pause()
        self.assertFalse(paused.can_pause)
        self.assertTrue(paused.can_resume)
        # Stop keeps its meaning throughout: a held task is still a task, and
        # taking the stop away is exactly what an operator cannot afford.
        self.assertTrue(paused.can_cancel)
        self.assertFalse(paused.can_start)
        self.assertEqual(paused.revision, self.revision)

        _, executing = self._resume()
        self.assertTrue(executing.can_pause)
        self.assertFalse(executing.can_resume)
        self.assertTrue(executing.can_cancel)
        self.assertEqual(executing.revision, self.revision)

    def test_the_reported_state_never_flaps_back_out_of_a_pause(self):
        """Feedback older than the request must not undo it."""
        self._reach_executing()
        mark, _ = self._pause()
        states = self._states_since(mark)
        # Anything before the first pause state is the run still being
        # reported as it was; from there to PAUSED nothing may report the task
        # as executing again.
        first_pause = next(
            index for index, state in enumerate(states) if state in PAUSE_STATES)
        self.assertNotIn(
            CoverageStatus.EXECUTING, states[first_pause:],
            'the manager reported the task as executing again while it was '
            'being paused: ' + repr(states))
        self.assertEqual(states[-1], CoverageStatus.PAUSED)

        mark, _ = self._resume()
        states = self._states_since(mark)
        first_running = next(
            index for index, state in enumerate(states)
            if state == CoverageStatus.EXECUTING)
        self.assertFalse(
            [state for state in states[first_running:] if state in PAUSE_STATES],
            'the manager reported the task as paused again after it had '
            'resumed: ' + repr(states))

    def test_the_task_announces_pausing_before_it_announces_paused(self):
        """PAUSING is a real state, not a label the manager skips."""
        self._reach_executing()
        mark, _ = self._pause()
        self.assertIn(
            CoverageStatus.PAUSING, self._states_since(mark),
            'the manager never announced PAUSING: ' +
            repr(self._states_since(mark)))

    def test_pause_is_refused_when_there_is_nothing_to_pause(self):
        """A refusal explains itself instead of quietly doing nothing."""
        self.task_publisher.publish(_task(self.revision))
        ready = self._wait_until(
            lambda s: s.state == CoverageStatus.READY and
            s.revision == self.revision, 'the preview to be accepted')
        self.assertFalse(ready.can_pause)
        self.assertFalse(ready.can_resume)
        response = self._call(self.pause_client)
        self.assertFalse(response.success)
        self.assertIn('No active coverage task', response.message)

    def test_resume_is_refused_while_the_task_is_running(self):
        """Resume is not a second start button."""
        self._reach_executing()
        response = self._call(self.resume_client)
        self.assertFalse(response.success)
        self.assertIn('not paused', response.message)

    def test_a_second_pause_while_paused_is_refused(self):
        """Pausing twice is an operator mistake, and it is answered as one."""
        self._reach_executing()
        self._pause()
        response = self._call(self.pause_client)
        self.assertFalse(response.success)
        self.assertIn('already paused', response.message)
        self.assertEqual(self._latest().state, CoverageStatus.PAUSED)

    def test_repeated_pause_and_resume_keeps_one_task(self):
        """Three cycles neither restart the task nor lose its identity."""
        self._reach_executing()
        for _ in range(3):
            _, paused = self._pause()
            self.assertEqual(paused.revision, self.revision)
            self.assertEqual(paused.task_id, 'pause-manager-test')
            _, executing = self._resume()
            self.assertEqual(executing.revision, self.revision)
            self.assertEqual(executing.task_id, 'pause-manager-test')
        self.assertEqual(self._latest().state, CoverageStatus.EXECUTING)

    def test_stop_during_a_pause_cancels_the_task(self):
        """Stop keeps meaning cancel, from inside the pause as well."""
        self._reach_executing()
        self._pause()
        self.assertTrue(self._call(self.cancel_client).success)
        finished = self._wait_until(
            lambda s: s.state == CoverageStatus.FINISHED, 'the cancel to land')
        self.assertFalse(finished.can_pause)
        self.assertFalse(finished.can_resume)
        self.assertFalse(finished.can_cancel)
        # The preview is still cached, so the run can be started again.
        self.assertTrue(finished.can_start)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    """A crashed executor or manager must fail the launch test."""

    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
