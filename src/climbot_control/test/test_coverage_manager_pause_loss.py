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
A pause the executor cannot answer must not leave the task in limbo.

Pause is the one control that asks the executor a question and waits for the
answer. Two ways of not getting one have to be told apart, because they call
for opposite responses. An executor that offers no pause service at all has
answered: the task is untouched and still running, so the request is refused
and nothing else changes. An executor that accepted the request and then went
silent has not - the robot may be decelerating, may be at speed, and the
manager has no way to know which. That one is a lost executor, and it takes the
same stop path as any other.
"""

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
from rclpy.action import ActionServer
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger


# Long enough to be reached deliberately and short enough to be waited out.
PAUSE_RESPONSE_TIMEOUT_S = 1.0


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Run the manager alone; this test supplies its own executor."""
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='climbot_control', executable='coverage_manager_node',
            parameters=[{
                'executor_timeout_s': 2.0,
                'command_quiet_s': 0.5,
                'hold_response_timeout_s': 0.3,
                'pause_response_timeout_s': PAUSE_RESPONSE_TIMEOUT_S,
            }]),
        launch_testing.actions.ReadyToTest(),
    ])


_revisions = iter(range(201, 300))


def _pose(x, y):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.orientation.w = 1.0
    return pose


def _task(revision):
    task = CoverageTask()
    task.header.frame_id = 'odom'
    task.task_id = 'pause-loss-test'
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


class TestCoverageManagerPauseLoss(unittest.TestCase):
    """Drive the manager against an executor that will not answer."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('pause_loss_test')
        self.statuses = []
        self.task_publisher = self.node.create_publisher(
            CoverageTask, '/coverage/task',
            rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))
        self.node.create_subscription(
            CoverageStatus, '/coverage/manager_status', self.statuses.append,
            rclpy.qos.QoSProfile(
                depth=100,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))
        self.start_client = self.node.create_client(Trigger, '/coverage/start')
        self.cancel_client = self.node.create_client(Trigger, '/coverage/cancel')
        self.pause_client = self.node.create_client(Trigger, '/coverage/pause')

        # Stands in for the speed watchdog, so the manager has a stop path that
        # does not go through the executor it has just lost.
        self.hold_publisher = self.node.create_publisher(
            Bool, '/control/hold_active',
            rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))
        self.hold_publisher.publish(Bool(data=False))
        self.hold_requests = []
        self.node.create_service(SetBool, '/control/hold', self._serve_hold)

        # An executor that accepts the goal and never finishes it.
        self.executor_node = rclpy.create_node('stub_pause_executor')
        self.server = ActionServer(
            self.executor_node, ExecuteCoverage, '/coverage/execute',
            execute_callback=self._execute,
            handle_accepted_callback=self._accept)
        # The pause service lives on a node of its own so a blocked reply
        # wedges only itself, exactly as a half-alive executor would.
        self.pause_node = None
        self.pause_release = Event()

        self.stop_spin = Event()
        # One thread serves both nodes, as the other executor-loss tests do.
        # Only the deliberately blocked pause service gets a thread of its own.
        self.threads = [Thread(target=self._spin_pair)]
        for thread in self.threads:
            thread.start()
        for client in (self.start_client, self.cancel_client, self.pause_client):
            self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        self.revision = next(_revisions)

    def tearDown(self):
        self.pause_release.set()
        self._call(self.cancel_client)
        # This stub accepts a goal and never finishes it, so a cancel has
        # nowhere to land. Taking the Action server away is what tells the
        # manager the executor is gone, and its own stop path does the rest;
        # the next test needs it back in a startable state.
        self._destroy_executor()
        self._settle()
        self.hold_publisher.publish(Bool(data=False))
        time.sleep(0.2)
        self.stop_spin.set()
        for thread in self.threads:
            thread.join()
        if self.pause_node is not None:
            self.pause_node.destroy_node()
        self.node.destroy_node()
        rclpy.shutdown()

    def _spin_pair(self):
        while rclpy.ok() and not self.stop_spin.is_set():
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if self.executor_node is not None:
                rclpy.spin_once(self.executor_node, timeout_sec=0.01)

    def _destroy_executor(self):
        if self.executor_node is None:
            return
        node, self.executor_node = self.executor_node, None
        time.sleep(0.1)
        self.server.destroy()
        node.destroy_node()

    def _spin(self, node):
        # An executor of its own, not rclpy.spin_once(): that helper drives the
        # process-wide default executor, so a second thread calling it raises
        # 'Executor is already spinning' and this thread dies silently. The
        # stub below would then be unanswering because nothing served it rather
        # than because it deliberately holds its reply, and this test would
        # pass without ever exercising what it claims to.
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        try:
            while rclpy.ok() and not self.stop_spin.is_set():
                executor.spin_once(timeout_sec=0.01)
        finally:
            executor.remove_node(node)
            executor.shutdown()

    def _serve_hold(self, request, response):
        self.hold_requests.append(request.data)
        self.hold_publisher.publish(Bool(data=request.data))
        response.success = True
        return response

    def _accept(self, goal_handle):
        # Deliberately never executed, so no result is ever published.
        self.goal_handle = goal_handle

    @staticmethod
    def _execute(goal_handle):
        goal_handle.abort()
        return ExecuteCoverage.Result()

    def _offer_a_pause_service_that_never_answers(self):
        self.pause_node = rclpy.create_node('stub_pause_service')

        def never_answer(request, response):
            # Held until tearDown releases it, so the manager's own deadline is
            # the only thing that can end the wait.
            self.pause_release.wait(timeout=30.0)
            response.success = True
            return response

        self.pause_node.create_service(
            SetBool, '/coverage/executor_pause', never_answer)
        thread = Thread(target=self._spin, args=(self.pause_node,))
        self.threads.append(thread)
        thread.start()
        # The manager checks service_is_ready() before sending, so the client
        # has to have discovered this service before the request is made.
        deadline = time.monotonic() + 10.0
        probe = self.node.create_client(SetBool, '/coverage/executor_pause')
        while not probe.service_is_ready() and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertTrue(probe.service_is_ready(), 'the stub pause service never appeared')
        self.node.destroy_client(probe)
        time.sleep(0.5)

    def _call(self, client, timeout_s=10.0):
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(future.done(), 'Service call never returned.')
        return future.result()

    def _latest(self):
        return self.statuses[-1] if self.statuses else None

    def _wait_until(self, predicate, what, timeout=15.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            latest = self._latest()
            if latest is not None and predicate(latest):
                return latest
            time.sleep(0.01)
        self.fail('Timed out waiting for {}; last status was {}'.format(
            what, self._latest()))

    def _settle(self):
        self._wait_until(
            lambda s: s.can_start or s.state in (
                CoverageStatus.FINISHED, CoverageStatus.READY,
                CoverageStatus.IDLE, CoverageStatus.INVALID),
            'the manager to settle', timeout=30.0)

    def _reach_executing(self):
        self.task_publisher.publish(_task(self.revision))
        self._wait_until(
            lambda s: s.state == CoverageStatus.READY and
            s.revision == self.revision, 'the preview to be accepted')
        self.assertTrue(self._call(self.start_client).success)
        return self._wait_until(
            lambda s: s.state == CoverageStatus.EXECUTING, 'execution to start')

    def test_a_pause_is_refused_when_the_executor_offers_no_pause_service(self):
        """An executor that cannot pause leaves the task exactly as it was."""
        self._reach_executing()
        response = self._call(self.pause_client)
        self.assertFalse(response.success)
        self.assertIn('unavailable', response.message)
        # Refused, not half-applied: the run is still the run.
        time.sleep(0.5)
        latest = self._latest()
        self.assertEqual(latest.state, CoverageStatus.EXECUTING)
        self.assertTrue(latest.can_cancel)
        self.assertNotIn(
            True, self.hold_requests,
            'a refused pause engaged the speed hold, which is a stop the '
            'operator did not ask for')

    def test_a_pause_the_executor_never_answers_stops_the_task(self):
        """Silence after an accepted request is a lost executor, not a pause."""
        self._offer_a_pause_service_that_never_answers()
        self._reach_executing()
        response = self._call(self.pause_client)
        self.assertTrue(response.success, response.message)
        self._wait_until(
            lambda s: s.state == CoverageStatus.PAUSING,
            'the manager to announce PAUSING')
        stopping = self._wait_until(
            lambda s: s.state == CoverageStatus.STOPPING,
            'the pause deadline to expire')
        self.assertIn('timed out', stopping.message)
        # The robot is held before the operator is told anything.
        self.assertIn(True, self.hold_requests)
        self._wait_until(
            lambda s: s.state == CoverageStatus.FINISHED,
            'the stop to complete')


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    """A crashed manager must fail the launch test."""

    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
