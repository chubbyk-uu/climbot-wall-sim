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
            parameters=[{'archive_finalize_timeout_s': 0.30}]),
        launch_testing.actions.ReadyToTest(),
    ])


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
        self.manager_parameters = self.node.create_client(
            GetParameters, '/coverage_manager/get_parameters')
        self.statuses = []
        self.prepare_ok = True
        self.prepare_release = None
        self.finalize_release = None
        self.prepare_entered = Event()
        self.finalize_entered = Event()
        self.prepare_requests = []
        self.finalize_requests = []
        self.stop = Event()
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self.thread = Thread(target=self._spin)
        self.thread.start()

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
        response.success = self.prepare_ok
        response.message = 'prepared' if self.prepare_ok else 'disk preflight failed'
        if self.prepare_ok:
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

    def _call(self, client, request, timeout=8.0):
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
        self.prepare_ok = False
        self.tasks.publish(task(22))
        self._wait(lambda item: item.state == CoverageStatus.READY and item.revision == 22,
                   'ready preview')
        mark = len(self.statuses)
        self.assertTrue(self._call(self.start, self._start_request()).success)
        failed = self._wait(
            lambda item: item.state == CoverageStatus.READY and item.revision == 22 and
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
        time.sleep(0.30)
        self.assertFalse(any(
            item.archive_state != InspectionArchiveStatus.FAILED
            for item in self.statuses[count:]),
            'late finalize response rewrote the timed-out archive result')


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
