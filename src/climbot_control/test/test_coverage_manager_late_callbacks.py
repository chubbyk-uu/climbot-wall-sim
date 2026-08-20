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

"""An unanswered start remains supervised until its real outcome arrives."""

# Every callback on a goal is registered once and lives as long as the client.
# Nothing in rclcpp retires the ones belonging to a request that has been given
# up on, so the manager has to do it: a start whose response never arrived is
# reported as not started, and its acceptance can still turn up minutes later.
#
# A timeout cannot retract the send-goal request, and before its response there
# is no handle to cancel. Returning to READY therefore creates an orphan if the
# request is accepted later: the one-goal line tracker rejects the replacement,
# while the old goal keeps running without a handle or a Stop entry in the
# manager. A different Action server may accept both, so the manager must not
# rely on this project's executor policy either.
#
# The stand-in executor refuses cancellation. The manager must stay STOPPING
# until that goal produces a real terminal result, not send one cancellation
# and forget it.

from threading import Event, Thread
import time
import unittest

from climbot_interfaces.action import ExecuteCoverage
from climbot_interfaces.msg import CoverageStatus, CoverageTask
from geometry_msgs.msg import Point32, Pose
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger


ABANDONED_REVISION = 7
STRAY_SEGMENT = 99
STRAY_PROGRESS = 0.99


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Run the manager alone, with a start deadline short enough to test."""
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='climbot_control', executable='coverage_manager_node',
            parameters=[{
                'start_response_timeout_s': 1.0,
                # High, so nothing here is the executor-loss path: this test is
                # about a request that was given up on while the executor was
                # present the whole time.
                'executor_timeout_s': 60.0,
            }]),
        launch_testing.actions.ReadyToTest(),
    ])


def _pose(x, y):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.orientation.w = 1.0
    return pose


def _task(revision):
    task = CoverageTask()
    task.header.frame_id = 'odom'
    task.task_id = 'late-callback-test'
    task.revision = revision
    task.sweep_direction = CoverageTask.SWEEP_HORIZONTAL
    task.waypoints = [_pose(0.0, 0.0), _pose(0.4, 0.0)]
    task.segment_types = [CoverageTask.SEGMENT_SCAN]
    for x, y in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]:
        point = Point32(x=x, y=y)
        task.coverage_region.points.append(point)
        task.motion_region.points.append(point)
    task.detection_width = 0.1
    task.detection_length = 0.1
    return task


class TestCoverageManagerLateCallbacks(unittest.TestCase):
    """The manager must ignore anything an abandoned request says afterwards."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('late_callback_test')
        self.statuses = []
        self.task_publisher = self.node.create_publisher(
            CoverageTask, '/coverage/task',
            rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))
        self.node.create_subscription(CoverageStatus, '/coverage/manager_status',
                                      self.statuses.append, 10)
        self.start_client = self.node.create_client(Trigger, '/coverage/start')
        self.force_abandon_client = self.node.create_client(
            Trigger, '/coverage/force_abandon')
        self.rearm_client = self.node.create_client(Trigger, '/coverage/rearm')

        self.executor_node = rclpy.create_node('stub_executor')
        self.first_goal_seen = Event()
        self.release_first = Event()
        self.abandoned_finished = Event()
        self.end_of_test = Event()
        self.goals = 0
        # Reentrant, on a multi-threaded executor: the first goal's decision
        # blocks until the test releases it, and the second goal has to be
        # accepted while it is still blocked.
        self.server = ActionServer(
            self.executor_node, ExecuteCoverage, '/coverage/execute',
            goal_callback=self._decide,
            cancel_callback=self._refuse_cancel,
            execute_callback=self._execute,
            callback_group=ReentrantCallbackGroup())

        self.ros_executor = MultiThreadedExecutor(num_threads=4)
        self.ros_executor.add_node(self.node)
        self.ros_executor.add_node(self.executor_node)
        self.spin_thread = Thread(target=self.ros_executor.spin)
        self.spin_thread.start()
        self.assertTrue(self.start_client.wait_for_service(timeout_sec=10.0))
        self.assertTrue(self.force_abandon_client.wait_for_service(timeout_sec=10.0))
        self.assertTrue(self.rearm_client.wait_for_service(timeout_sec=10.0))

    def tearDown(self):
        self.end_of_test.set()
        self.release_first.set()
        self.ros_executor.shutdown(timeout_sec=5.0)
        self.spin_thread.join(timeout=10.0)
        self.server.destroy()
        self.executor_node.destroy_node()
        self.node.destroy_node()
        rclpy.shutdown()

    def _decide(self, goal_request):
        self.goals += 1
        if self.goals == 1:
            self.first_goal_seen.set()
            # Answering only when the test says so is what makes the manager
            # give up on this request while it is still in flight.
            self.release_first.wait(60.0)
        return GoalResponse.ACCEPT

    def _refuse_cancel(self, goal_handle):
        return CancelResponse.REJECT

    def _execute(self, goal_handle):
        feedback = ExecuteCoverage.Feedback()
        feedback.current_segment = STRAY_SEGMENT
        feedback.progress = STRAY_PROGRESS
        feedback.state = ExecuteCoverage.Feedback.TRACK_LINE
        goal_handle.publish_feedback(feedback)
        time.sleep(0.5)
        goal_handle.succeed()
        result = ExecuteCoverage.Result()
        result.result_code = ExecuteCoverage.Result.SUCCESS
        result.message = 'abandoned goal finished on its own'
        self.abandoned_finished.set()
        return result

    def _call(self, client):
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 10.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(future.done(), 'The start service did not answer.')
        return future.result()

    def _call_start(self):
        return self._call(self.start_client)

    def _wait_until(self, predicate, what, timeout=30.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            latest = self.statuses[-1] if self.statuses else None
            if latest is not None and predicate(latest):
                return latest
            time.sleep(0.05)
        self.fail('Timed out waiting for {}; last status was {}'.format(
            what, self.statuses[-1] if self.statuses else None))

    def test_a_timed_out_start_stays_supervised_until_its_late_result(self):
        """A rejected cancel must not leave the accepted goal orphaned."""
        self.task_publisher.publish(_task(ABANDONED_REVISION))
        self._wait_until(
            lambda s: s.state == CoverageStatus.READY and s.revision == ABANDONED_REVISION,
            'the first preview')

        self.assertTrue(self._call_start().success)
        self.assertTrue(self.first_goal_seen.wait(10.0), 'The stub never saw the first goal.')
        timed_out = self._wait_until(
            lambda s: s.state == CoverageStatus.STOPPING and 'timed out' in s.message,
            'the unanswered start to enter the uncertain stopping state')
        self.assertFalse(timed_out.can_start)
        self.assertTrue(timed_out.can_cancel)

        # Stop remains meaningful even before there is a handle: it keeps the
        # independent hold requested while the response is unknown.
        cancel_client = self.node.create_client(Trigger, '/coverage/cancel')
        self.assertTrue(cancel_client.wait_for_service(timeout_sec=5.0))
        cancel_future = cancel_client.call_async(Trigger.Request())
        deadline = time.monotonic() + 5.0
        while not cancel_future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(cancel_future.done())
        self.assertTrue(cancel_future.result().success)

        # A newer preview may be cached, but it cannot become a second request
        # while the first one's acceptance is still unknown.
        self.task_publisher.publish(_task(8))
        time.sleep(0.2)
        refused = self._call_start()
        self.assertFalse(refused.success)
        self.assertIn('already', refused.message)

        # The old request is accepted late and refuses cancellation. It really
        # runs; the manager must retain STOPPING until its real result arrives.
        self.release_first.set()
        self._wait_until(
            lambda s: s.state == CoverageStatus.STOPPING and 'accepted' in s.message,
            'the late accepted goal to remain in stopping')
        self.assertTrue(
            self.abandoned_finished.wait(30.0),
            'The late goal never ran, so cancellation rejection was not tested.')
        finished = self._wait_until(
            lambda s: s.state == CoverageStatus.FINISHED,
            'the late goal to supply its terminal result')
        self.assertEqual(finished.result_code, ExecuteCoverage.Result.SUCCESS)
        self.assertTrue(finished.can_start)
        self.assertFalse(finished.can_cancel)

    def test_force_abandon_requires_rearm_and_a_late_acceptance_relocks(self):
        """The manual escape must remain fail-safe when its premise was wrong."""
        self.task_publisher.publish(_task(ABANDONED_REVISION + 10))
        self._wait_until(
            lambda s: s.state == CoverageStatus.READY,
            'the recovery preview')
        self.assertTrue(self._call_start().success)
        self.assertTrue(self.first_goal_seen.wait(10.0))
        stopping = self._wait_until(
            lambda s: s.state == CoverageStatus.STOPPING and
            s.can_force_abandon,
            'force abandon to become available for an unknown response')
        self.assertFalse(stopping.can_rearm)

        forced = self._call(self.force_abandon_client)
        self.assertTrue(forced.success)
        locked = self._wait_until(
            lambda s: s.state == CoverageStatus.RECOVERY_LOCKED,
            'the manager to enter recovery lock')
        self.assertFalse(locked.can_start)
        self.assertFalse(locked.can_cancel)
        self.assertFalse(locked.can_force_abandon)
        self.assertTrue(locked.can_rearm)
        self.assertFalse(self._call_start().success)

        self.assertTrue(self._call(self.rearm_client).success)
        ready = self._wait_until(
            lambda s: s.state == CoverageStatus.READY and s.can_start,
            'operator rearm to restore READY')
        self.assertFalse(ready.can_rearm)

        # The explicit recovery was based on external verification. If that
        # premise proves wrong and the retired request is accepted after all,
        # it must not run behind READY or a newer task: hold and lock again.
        self.release_first.set()
        relocked = self._wait_until(
            lambda s: s.state == CoverageStatus.RECOVERY_LOCKED and
            'accepted late' in s.message,
            'the late acceptance to invalidate the operator rearm')
        self.assertFalse(relocked.can_start)
        self.assertTrue(relocked.can_rearm)
        self.assertTrue(self.abandoned_finished.wait(30.0))
        time.sleep(0.2)
        self.assertEqual(self.statuses[-1].state, CoverageStatus.RECOVERY_LOCKED)

        # Test cleanup models a second physical verification after the now
        # known old Goal has reached its terminal result.
        self.assertTrue(self._call(self.rearm_client).success)
        self._wait_until(
            lambda s: s.state == CoverageStatus.READY,
            'the second rearm to release the test fixture')


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    """A crashed manager must fail the launch test."""

    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
