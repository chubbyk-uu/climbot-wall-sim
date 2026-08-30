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

"""Manager-level G4 checks for archive preflight before motion is dispatched."""

import math
from threading import Event, Thread
import time
import unittest

from climbot_interfaces.action import ExecuteCoverage
from climbot_interfaces.msg import CoverageStatus, CoverageTask, InspectionArchiveStatus
from climbot_interfaces.srv import (
    FinalizeInspectionArchive,
    PrepareInspectionArchive,
    StartCoverage,
)
from geometry_msgs.msg import Point32, Pose
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
from nav_msgs.msg import Odometry
import pytest
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='climbot_control', executable='line_tracker_node',
            parameters=[{
                'standalone_mode': False,
                'odometry_timeout_s': 2.0,
                'segment_timeout_s': 30.0,
                'alignment_settle_duration_s': 0.05,
                'wheel_separation': 0.43,
                'wheel_speed_limit': 0.45,
                'wheel_acceleration_limit': 0.40,
            }]),
        launch_ros.actions.Node(
            package='climbot_control', executable='coverage_manager_node',
            parameters=[{
                'archive_finalize_timeout_s': 0.30,
                # Make every accepted archive topic update observable. With
                # the production feedback throttle, a bad late update can
                # mutate internal state without publishing before this test
                # ends, which is the race -j8 happened to expose.
                'feedback_publish_period_s': 0.0,
            }]),
        launch_testing.actions.ReadyToTest(),
    ])


# Which task the mock recorder refuses to prepare for. Keyed on the revision
# rather than on a flag, because the manager outlives every test in this file
# while each test builds a new node: under load its archive client can still be
# routed to the previous test's service, and a flag then answers for a task it
# was never set for. That is how the cancellable-preflight test below came to
# fail carrying the failure test's own 'disk preflight failed'.
FAILING_PREFLIGHT_REVISION = 22


def pose(x, y, yaw):
    result = Pose()
    result.position.x = x
    result.position.y = y
    result.orientation.z = math.sin(yaw / 2.0)
    result.orientation.w = math.cos(yaw / 2.0)
    return result


def task(revision):
    result = CoverageTask()
    result.header.frame_id = 'odom'
    result.task_id = 'archive-manager-test'
    result.revision = revision
    result.sweep_direction = CoverageTask.SWEEP_HORIZONTAL
    result.waypoints = [pose(0.0, 0.0, 0.0), pose(0.4, 0.0, 0.0)]
    result.segment_types = [CoverageTask.SEGMENT_SCAN]
    result.detection_width = 0.1
    result.detection_length = 0.25
    for x, y in [(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)]:
        point = Point32(x=x, y=y)
        result.coverage_region.points.append(point)
        result.motion_region.points.append(point)
    return result


class TestCoverageManagerArchive(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('coverage_manager_archive_test')
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.status_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tasks = self.node.create_publisher(CoverageTask, '/coverage/task', self.status_qos)
        self.odometry = self.node.create_publisher(Odometry, '/odometry/filtered', reliable)
        self.archive_status = self.node.create_publisher(
            InspectionArchiveStatus, '/inspection/archive/status', self.status_qos)
        self.node.create_subscription(
            CoverageStatus, '/coverage/manager_status', self._on_status, self.status_qos)
        self.recorder_callbacks = ReentrantCallbackGroup()
        self.node.create_service(
            PrepareInspectionArchive, '/inspection/archive/prepare', self._prepare,
            callback_group=self.recorder_callbacks)
        self.node.create_service(
            FinalizeInspectionArchive, '/inspection/archive/finalize', self._finalize,
            callback_group=self.recorder_callbacks)
        self.start = self.node.create_client(StartCoverage, '/coverage/start_configured')
        self.cancel = self.node.create_client(Trigger, '/coverage/cancel')
        self.pause = self.node.create_client(Trigger, '/coverage/pause')
        self.resume = self.node.create_client(Trigger, '/coverage/resume')
        self.manager_parameters = self.node.create_client(
            GetParameters, '/coverage_manager/get_parameters')
        self.statuses = []
        self.prepare_release = None
        self.finalize_release = None
        self.prepare_entered = Event()
        self.finalize_entered = Event()
        self.finalize_completed = Event()
        self.prepare_requests = []
        self.finalize_requests = []
        self.stop = Event()
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self.thread = Thread(target=self._spin)
        self.thread.start()
        # Every test builds a new node, so every test has to rediscover these.
        # A call_async() on a client that has not matched yet is simply never
        # delivered, and the test then fails waiting for a status the manager
        # was never asked to produce - which is what made the cancellable
        # preflight test fail once other launch tests ran before it.
        for client in (self.start, self.cancel, self.pause, self.resume):
            self.assertTrue(client.wait_for_service(timeout_sec=15.0))

    def tearDown(self):
        self.stop.set()
        self.executor.shutdown()
        self.thread.join()
        self.node.destroy_node()
        rclpy.shutdown()

    def _spin(self):
        self.executor.spin()

    def _on_status(self, message):
        self.statuses.append(message)

    def _prepare(self, request, response):
        self.prepare_requests.append(request)
        self.prepare_entered.set()
        if self.prepare_release is not None:
            self.prepare_release.wait(timeout=5.0)
        prepare_ok = request.task.revision != FAILING_PREFLIGHT_REVISION
        response.success = prepare_ok
        response.message = 'prepared' if prepare_ok else 'disk preflight failed'
        if prepare_ok:
            response.run_id = f'run-{request.task.revision}'
            response.task_directory = f'/tmp/archive-{request.task.revision}'
            response.expected_images = 2
            response.estimated_bytes = 1234
            self._publish_archive(InspectionArchiveStatus.READY, request.task, response.run_id)
        return response

    def _finalize(self, request, response):
        self.finalize_requests.append(request)
        self.finalize_entered.set()
        if self.finalize_release is not None:
            self.finalize_release.wait(timeout=5.0)
        response.success = True
        response.message = 'archive finalized'
        active = self.prepare_requests[-1].task
        state = InspectionArchiveStatus.CANCELED if request.outcome == \
            FinalizeInspectionArchive.Request.CANCELED else InspectionArchiveStatus.FAILED
        self._publish_archive(state, active, request.run_id)
        self.finalize_completed.set()
        return response

    def _publish_archive(self, state, coverage_task, run_id, expected_images=2):
        message = InspectionArchiveStatus()
        message.state = state
        message.inspection_enabled = True
        message.task_id = coverage_task.task_id
        message.revision = coverage_task.revision
        message.run_id = run_id
        message.task_directory = f'/tmp/archive-{coverage_task.revision}'
        message.expected_images = expected_images
        message.message = 'mock archive state'
        self.archive_status.publish(message)

    def _call(self, client, request, timeout=12.0):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            self._odom()
            time.sleep(0.01)
        self.assertTrue(future.done(), 'service did not answer')
        return future.result()

    def _odom(self):
        message = Odometry()
        message.header.frame_id = 'odom'
        message.pose.pose = pose(0.0, 0.0, 0.0)
        self.odometry.publish(message)

    def _wait(self, predicate, description, since=0, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            match = next((item for item in self.statuses[since:] if predicate(item)), None)
            if match is not None:
                return match
            self._odom()
            time.sleep(0.01)
        last_status = self.statuses[-1] if self.statuses else None
        self.fail(f'No status for {description}; last={last_status}')

    def _start_request(self):
        request = StartCoverage.Request()
        request.inspection_enabled = True
        request.output_root = '/tmp/g4-root'
        return request

    def test_a_pause_holds_the_archive_open_on_the_same_run(self):
        """A pause is not an ending: nothing is finalized and nothing reopens."""
        self.assertTrue(self.start.wait_for_service(timeout_sec=10.0))
        self.assertTrue(self.pause.wait_for_service(timeout_sec=10.0))
        self.assertTrue(self.resume.wait_for_service(timeout_sec=10.0))
        self.tasks.publish(task(27))
        self._wait(lambda item: item.state == CoverageStatus.READY and item.revision == 27,
                   'ready preview')
        mark = len(self.statuses)
        self.assertTrue(self._call(self.start, self._start_request()).success)
        self._wait(
            lambda item: item.state == CoverageStatus.EXECUTING and
            item.archive_run_id == 'run-27',
            'goal execution after archive preparation', since=mark)
        self._publish_archive(
            InspectionArchiveStatus.RECORDING, self.prepare_requests[-1].task, 'run-27')
        self._wait(
            lambda item: item.archive_state == InspectionArchiveStatus.RECORDING,
            'the archive to be recording', since=mark)
        prepared = len(self.prepare_requests)

        self.assertTrue(self._call(self.pause, Trigger.Request()).success)
        paused = self._wait(
            lambda item: item.state == CoverageStatus.PAUSED, 'the task to pause',
            since=mark)
        self.assertEqual(paused.archive_run_id, 'run-27')
        self.assertEqual(paused.archive_state, InspectionArchiveStatus.RECORDING)
        self.assertTrue(paused.inspection_enabled)

        # Held long enough that a finalization on the way would have landed.
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            self._odom()
            time.sleep(0.01)
        self.assertEqual(
            self.finalize_requests, [],
            'a pause finalized the inspection archive')
        self.assertEqual(
            len(self.prepare_requests), prepared,
            'a pause opened a second archive run')

        self.assertTrue(self._call(self.resume, Trigger.Request()).success)
        resumed = self._wait(
            lambda item: item.state == CoverageStatus.EXECUTING, 'the task to resume',
            since=mark)
        self.assertEqual(resumed.archive_run_id, 'run-27')
        self.assertEqual(len(self.prepare_requests), prepared)
        self.assertEqual(self.finalize_requests, [])

        # The run that opened before the pause is the run that closes after it.
        self.assertTrue(self._call(self.cancel, Trigger.Request()).success)
        finished = self._wait(
            lambda item: item.state == CoverageStatus.FINISHED and
            item.archive_state == InspectionArchiveStatus.CANCELED,
            'the canceled archive task', since=mark)
        self.assertEqual(finished.archive_run_id, 'run-27')
        self.assertEqual(len(self.finalize_requests), 1)
        self.assertEqual(self.finalize_requests[0].run_id, 'run-27')
        self.assertEqual(self.finalize_requests[0].outcome,
                         FinalizeInspectionArchive.Request.CANCELED)

    def test_archive_preflight_completes_before_goal_execution(self):
        self.assertTrue(self.start.wait_for_service(timeout_sec=10.0))
        self.assertTrue(self.cancel.wait_for_service(timeout_sec=10.0))
        self.tasks.publish(task(21))
        self._wait(lambda item: item.state == CoverageStatus.READY and item.revision == 21,
                   'ready preview')
        mark = len(self.statuses)
        started = self._call(self.start, self._start_request())
        self.assertTrue(started.success, started.message)
        preparing = self._wait(
            lambda item: item.state == CoverageStatus.STARTING and
            item.archive_state == InspectionArchiveStatus.PREPARING,
            'archive preparing', since=mark)
        self.assertTrue(preparing.inspection_enabled)
        self.assertEqual(self.prepare_requests[-1].output_root, '/tmp/g4-root')
        executing = self._wait(
            lambda item: item.state == CoverageStatus.EXECUTING and
            item.revision == 21 and item.archive_run_id == 'run-21',
            'goal execution after archive preparation', since=mark)
        self.assertTrue(executing.inspection_enabled)
        self.assertEqual(executing.archive_directory, '/tmp/archive-21')
        self.assertEqual(executing.archive_preflight_expected_images, 2)
        self._publish_archive(
            InspectionArchiveStatus.RECORDING, self.prepare_requests[-1].task,
            'run-21', expected_images=1)
        frozen = self._wait(
            lambda item: item.archive_state == InspectionArchiveStatus.RECORDING and
            item.archive_preflight_expected_images == 2 and
            item.archive_expected_images == 1,
            'frozen capture plan distinct from the nominal preflight total', since=mark)
        self.assertEqual(frozen.archive_saved_images, 0)

        canceled = self._call(self.cancel, Trigger.Request())
        self.assertTrue(canceled.success)
        finished = self._wait(
            lambda item: item.state == CoverageStatus.FINISHED and
            item.result_code == ExecuteCoverage.Result.CANCELED and
            item.archive_state == InspectionArchiveStatus.CANCELED,
            'canceled archive task', since=mark)
        self.assertEqual(finished.archive_state, InspectionArchiveStatus.CANCELED)
        self.assertEqual(self.finalize_requests[-1].outcome,
                         FinalizeInspectionArchive.Request.CANCELED)

    def test_preflight_failure_never_dispatches_motion_goal(self):
        self.tasks.publish(task(FAILING_PREFLIGHT_REVISION))
        self._wait(
            lambda item: item.state == CoverageStatus.READY and
            item.revision == FAILING_PREFLIGHT_REVISION, 'ready preview')
        mark = len(self.statuses)
        self.assertTrue(self._call(self.start, self._start_request()).success)
        failed = self._wait(
            lambda item: item.state == CoverageStatus.READY and
            item.revision == FAILING_PREFLIGHT_REVISION and
            item.archive_state == InspectionArchiveStatus.FAILED,
            'preflight failure', since=mark)
        self.assertIn('disk preflight failed', failed.message)
        self.assertFalse(any(item.state == CoverageStatus.EXECUTING
                             for item in self.statuses[mark:]))

    def test_preflight_is_cancellable_before_motion_is_dispatched(self):
        self.prepare_release = Event()
        self.tasks.publish(task(24))
        self._wait(lambda item: item.state == CoverageStatus.READY and item.revision == 24,
                   'ready preview')
        mark = len(self.statuses)
        request = self._start_request()
        future = self.start.call_async(request)
        preparing = self._wait(
            lambda item: item.state == CoverageStatus.STARTING and
            item.archive_state == InspectionArchiveStatus.PREPARING,
            'cancellable archive preparation', since=mark)
        self.assertTrue(preparing.can_cancel)
        self.assertTrue(self.prepare_entered.wait(timeout=2.0))

        canceled = self._call(self.cancel, Trigger.Request())
        self.assertTrue(canceled.success, canceled.message)
        self.prepare_release.set()
        self._wait(
            lambda item: item.state == CoverageStatus.READY and item.revision == 24 and
            item.archive_state == InspectionArchiveStatus.CANCELED,
            'preflight cancellation', since=mark)
        deadline = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(future.done(), 'start request did not complete after cancellation')
        self.assertFalse(any(item.state == CoverageStatus.EXECUTING
                             for item in self.statuses[mark:]))

    def test_runtime_archive_failure_requests_stop_and_marks_task_failed(self):
        self.tasks.publish(task(23))
        self._wait(lambda item: item.state == CoverageStatus.READY and item.revision == 23,
                   'ready preview')
        mark = len(self.statuses)
        self.assertTrue(self._call(self.start, self._start_request()).success)
        self._wait(lambda item: item.state == CoverageStatus.EXECUTING and item.revision == 23,
                   'archive-backed execution', since=mark)
        self._publish_archive(
            InspectionArchiveStatus.FAILED, self.prepare_requests[-1].task, 'run-23')
        failed = self._wait(
            lambda item: item.state == CoverageStatus.FINISHED and
            item.result_code == ExecuteCoverage.Result.ARCHIVE_FAILED and
            item.archive_state == InspectionArchiveStatus.FAILED,
            'archive failure to stop and finalize the task', since=mark)
        self.assertIn('archive', failed.message.lower())
        self.assertEqual(self.finalize_requests[-1].outcome,
                         FinalizeInspectionArchive.Request.FAILED)

    def test_finalize_timeout_releases_manager_and_ignores_late_response(self):
        """A recorder lost mid-RPC must not leave every operator action disabled."""
        self.assertTrue(self.manager_parameters.wait_for_service(timeout_sec=5.0))
        parameters = GetParameters.Request(names=['archive_finalize_timeout_s'])
        configured = self._call(self.manager_parameters, parameters)
        self.assertAlmostEqual(configured.values[0].double_value, 0.30)
        self.finalize_release = Event()
        self.tasks.publish(task(25))
        self._wait(lambda item: item.state == CoverageStatus.READY and item.revision == 25,
                   'ready preview')
        mark = len(self.statuses)
        self.assertTrue(self._call(self.start, self._start_request()).success)
        self._wait(lambda item: item.state == CoverageStatus.EXECUTING and item.revision == 25,
                   'archive-backed execution', since=mark)
        canceled = self._call(self.cancel, Trigger.Request())
        self.assertTrue(canceled.success, canceled.message)
        self.assertTrue(self.finalize_entered.wait(timeout=2.0),
                        'finalize request did not enter recorder')
        timed_out = self._wait(
            lambda item: item.state == CoverageStatus.FINISHED and
            item.result_code == ExecuteCoverage.Result.ARCHIVE_FAILED and
            item.archive_state == InspectionArchiveStatus.FAILED and item.can_start,
            'archive finalization timeout releases manager', since=mark, timeout=4.0)
        self.assertIn('timed out', timed_out.message)
        count = len(self.statuses)
        self.finalize_release.set()
        self.assertTrue(self.finalize_completed.wait(timeout=2.0),
                        'late finalize service did not complete')
        # The recorder publishes its terminal topic before returning the late
        # service response. Repeat that latched state after the manager's
        # feedback throttle has elapsed: without a topic-side retirement guard
        # the first delivery can mutate the manager silently and this one then
        # exposes the mutation. A delayed pre-timeout status is not a rewrite,
        # so compare publication stamps as well as local arrival order.
        time.sleep(0.25)
        self._publish_archive(
            InspectionArchiveStatus.CANCELED, self.prepare_requests[-1].task, 'run-25')
        time.sleep(0.50)
        timed_out_stamp = (timed_out.header.stamp.sec, timed_out.header.stamp.nanosec)
        rewrites = [
            item for item in self.statuses[count:]
            if (item.header.stamp.sec, item.header.stamp.nanosec) >= timed_out_stamp and
            item.archive_state != InspectionArchiveStatus.FAILED]
        self.assertEqual(
            rewrites, [],
            'late finalize status rewrote the timed-out archive result: %r' %
            [(item.archive_state, item.message) for item in rewrites])


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
