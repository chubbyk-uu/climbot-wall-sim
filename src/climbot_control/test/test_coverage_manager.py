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

"""Node-level checks for explicit coverage task start and cancellation."""

import math
from threading import Event, Thread
import time
import unittest

from climbot_interfaces.action import ExecuteCoverage
from climbot_interfaces.msg import CoverageStatus, CoverageTask
from geometry_msgs.msg import Point32, Pose
import launch
import launch_ros.actions
import launch_testing.actions
from nav_msgs.msg import Odometry
import pytest
import rclpy
from std_srvs.srv import Trigger

# Discovery and service calls are given room well past what they need, because
# these are launch tests: the whole suite runs in parallel and the machine is
# at its busiest exactly here. A tight bound does not catch a slow manager, it
# only reports the load.
DISCOVERY_TIMEOUT = 15.0
CALL_TIMEOUT = 10.0


@pytest.mark.launch_test
def generate_test_description():
    """Run the real Action server and manager together."""
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='climbot_control', executable='line_tracker_node',
            parameters=[{
                'standalone_mode': False,
                'odometry_timeout_s': 2.0,
                'segment_timeout_s': 15.0,
                'alignment_settle_duration_s': 0.05,
                'wheel_separation': 0.43,
                'wheel_speed_limit': 0.45,
                'wheel_acceleration_limit': 0.40,
            }]),
        launch_ros.actions.Node(
            package='climbot_control', executable='coverage_manager_node'),
        launch_testing.actions.ReadyToTest(),
    ])


def _pose(x, y, yaw):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def _task(revision):
    # Each test uses a revision of its own. The status topic is transient local
    # and these tests share one manager, so a test that matched on a revision
    # another test had already driven through could be satisfied by that older
    # status the instant it subscribed, and would pass without the manager
    # having done anything.
    task = CoverageTask()
    task.header.frame_id = 'odom'
    task.task_id = 'manager-test'
    task.revision = revision
    task.sweep_direction = CoverageTask.SWEEP_HORIZONTAL
    task.waypoints = [_pose(0.0, 0.0, 0.0), _pose(0.4, 0.0, 0.0)]
    task.segment_types = [CoverageTask.SEGMENT_SCAN]
    for x, y in [(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)]:
        point = Point32(x=x, y=y)
        task.coverage_region.points.append(point)
        task.motion_region.points.append(point)
    task.detection_width = 0.1
    task.detection_length = 0.1
    return task


class TestCoverageManager(unittest.TestCase):
    """Preview publication alone cannot move the robot; service start can."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('coverage_manager_test')
        self.statuses = []
        self.task_publisher = self.node.create_publisher(
            CoverageTask, '/coverage/task',
            rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))
        self.odom_publisher = self.node.create_publisher(
            Odometry, '/odometry/filtered', 10)
        self.node.create_subscription(CoverageStatus, '/coverage/manager_status',
                                      self.statuses.append, 10)
        self.start_client = self.node.create_client(Trigger, '/coverage/start')
        self.cancel_client = self.node.create_client(Trigger, '/coverage/cancel')
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
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def _call(self, client):
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + CALL_TIMEOUT
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(future.done())
        return future.result()

    def _publish_odom(self):
        message = Odometry()
        message.pose.pose = _pose(0.0, 0.0, 0.0)
        self.odom_publisher.publish(message)

    def _wait_for_state(self, state, text=None, timeout=10.0, pump_odom=False, since=0):
        # The status topic is transient local, so a fresh subscription receives
        # whatever the previous test left behind. Matching on the message text
        # as well keeps each test independent of the order they run in; where
        # the text cannot distinguish them, `since` is the index the caller took
        # before it asked for anything, so only statuses published in answer to
        # this test count.
        def matches(status):
            return status.state == state and (text is None or text in status.message)

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            match = next((status for status in self.statuses[since:] if matches(status)), None)
            if match is not None:
                return match
            if pump_odom:
                self._publish_odom()
            time.sleep(0.01)
        self.fail('No status with state {} and text {!r} arrived.'.format(state, text))

    def _clear_preview(self):
        """Put the manager back in the state an empty preview leaves it in."""
        empty = CoverageTask()
        empty.header.frame_id = 'odom'
        empty.task_id = 'manager-test'
        empty.revision = 10
        empty.sweep_direction = CoverageTask.SWEEP_HORIZONTAL
        empty.detection_width = 0.1
        empty.detection_length = 0.1
        mark = len(self.statuses)
        self.task_publisher.publish(empty)
        return self._wait_for_state(
            CoverageStatus.IDLE, 'no coverage region selected', since=mark)

    def _cancel_and_settle(self):
        """Cancel, and wait for the manager to stop being busy."""
        # A manager with a goal in hand answers a new preview by saying it kept
        # running, not by going Ready, so a test that returned while the cancel
        # was still in flight would leave the next one waiting for a status the
        # manager had already decided not to publish.
        mark = len(self.statuses)
        self._call(self.cancel_client)
        deadline = time.monotonic() + CALL_TIMEOUT
        while time.monotonic() < deadline:
            if any(status.state == CoverageStatus.FINISHED
                   for status in self.statuses[mark:]):
                return
            self._publish_odom()
            time.sleep(0.01)
        self.fail('The manager never reported the cancelled task as finished.')

    def test_requires_explicit_start_and_can_cancel(self):
        """A valid preview remains idle until start, then manager owns cancellation."""
        self.assertTrue(self.start_client.wait_for_service(timeout_sec=DISCOVERY_TIMEOUT))
        self.assertTrue(self.cancel_client.wait_for_service(timeout_sec=DISCOVERY_TIMEOUT))
        # Clear the preview rather than assuming the manager has none. Whatever
        # ran before this left a cached task behind, and "start refuses without
        # one" is only a claim about the manager if this test is what put it in
        # that state.
        self._clear_preview()
        no_task = self._call(self.start_client)
        self.assertFalse(no_task.success)

        self.task_publisher.publish(_task(13))
        ready = self._wait_for_state(
            CoverageStatus.READY, 'Ready: manager-test revision 13')
        self.assertEqual(ready.task_id, 'manager-test')
        self.assertEqual(ready.revision, 13)
        # Known from the cached task, before anything runs, so a display can
        # show "0 of 1" rather than waiting for the first feedback message.
        self.assertEqual(ready.total_segments, 1)
        self.assertEqual(ready.current_segment, -1)
        self.assertIn('Ready: manager-test revision 13', ready.message)

        for _ in range(4):
            self._publish_odom()
            time.sleep(0.02)
        started = self._call(self.start_client)
        self.assertTrue(started.success)
        executing = self._wait_for_state(
            CoverageStatus.EXECUTING, 'Executing manager-test revision 13',
            pump_odom=True)
        self.assertEqual(executing.task_id, 'manager-test')
        self.assertEqual(executing.revision, 13)
        self.assertIn('Executing manager-test revision 13', executing.message)

        mark = len(self.statuses)
        canceled = self._call(self.cancel_client)
        self.assertTrue(canceled.success)
        finished = self._wait_for_state(
            CoverageStatus.FINISHED, 'canceled', pump_odom=True, since=mark)
        self.assertEqual(finished.result_code, ExecuteCoverage.Result.CANCELED)

    def test_publishes_executor_progress(self):
        """Without the feedback callback there is no progress to display at all."""
        self.assertTrue(self.start_client.wait_for_service(timeout_sec=DISCOVERY_TIMEOUT))
        self.assertTrue(self.cancel_client.wait_for_service(timeout_sec=DISCOVERY_TIMEOUT))
        self.task_publisher.publish(_task(11))
        self._wait_for_state(CoverageStatus.READY, 'Ready: manager-test revision 11')
        for _ in range(4):
            self._publish_odom()
            time.sleep(0.02)
        self.assertTrue(self._call(self.start_client).success)
        self._wait_for_state(
            CoverageStatus.EXECUTING, 'Executing manager-test revision 11',
            pump_odom=True)

        mark = len(self.statuses)
        moving = []
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            moving = [
                status for status in self.statuses[mark:]
                if status.state == CoverageStatus.EXECUTING and
                status.executor_state != ExecuteCoverage.Feedback.WAITING]
            if moving:
                break
            self._publish_odom()
            time.sleep(0.01)
        self.assertTrue(moving, 'The executor never reported a motion state.')
        self.assertGreaterEqual(moving[-1].progress, 0.0)
        self.assertLessEqual(moving[-1].progress, 1.0)
        self._cancel_and_settle()

    def test_reports_a_cleared_preview_as_idle_rather_than_malformed(self):
        """An empty task is how the planner clears a preview, not a fault."""
        self.assertTrue(self.start_client.wait_for_service(timeout_sec=DISCOVERY_TIMEOUT))
        mark = len(self.statuses)
        idle = self._clear_preview()
        self.assertIn('no coverage region selected', idle.message)
        self.assertEqual(idle.total_segments, 0)
        # An empty preview is the one thing that must not read as a fault, so
        # only what the manager said about this one is evidence either way.
        self.assertFalse(any(
            status.state == CoverageStatus.INVALID
            for status in self.statuses[mark:]))
        self.assertFalse(self._call(self.start_client).success)
