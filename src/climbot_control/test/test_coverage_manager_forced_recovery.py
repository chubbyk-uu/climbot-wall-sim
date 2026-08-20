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

"""Recover when an executor dies before answering a sent Goal request."""

import os
import subprocess
import sys
from threading import Thread
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
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Run only the manager; the test owns and kills its executor process."""
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='climbot_control', executable='coverage_manager_node',
            parameters=[{
                'start_response_timeout_s': 1.0,
                'executor_timeout_s': 1.0,
                'command_quiet_s': 0.5,
            }]),
        launch_testing.actions.ReadyToTest(),
    ])


def _pose(x, y):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.orientation.w = 1.0
    return pose


def _task():
    task = CoverageTask()
    task.header.frame_id = 'odom'
    task.task_id = 'forced-recovery-test'
    task.revision = 1
    task.sweep_direction = CoverageTask.SWEEP_HORIZONTAL
    task.waypoints = [_pose(0.0, 0.0), _pose(0.4, 0.0)]
    task.segment_types = [CoverageTask.SEGMENT_SCAN]
    for x, y in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]:
        task.coverage_region.points.append(Point32(x=x, y=y))
        task.motion_region.points.append(Point32(x=x, y=y))
    task.detection_width = 0.1
    task.detection_length = 0.1
    return task


class TestCoverageManagerForcedRecovery(unittest.TestCase):
    """Exercise the exact crash window that otherwise needs a manager restart."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('forced_recovery_test')
        self.statuses = []
        self.node.create_subscription(
            CoverageStatus, '/coverage/manager_status', self.statuses.append,
            rclpy.qos.QoSProfile(
                depth=20,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))
        self.task_publisher = self.node.create_publisher(
            CoverageTask, '/coverage/task',
            rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))
        self.hold_requests = []
        self.hold_publisher = self.node.create_publisher(
            Bool, '/control/hold_active',
            rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))

        def set_hold(request, response):
            self.hold_requests.append(request.data)
            self.hold_publisher.publish(Bool(data=request.data))
            response.success = True
            return response

        self.hold_service = self.node.create_service(
            SetBool, '/control/hold', set_hold)
        self.start_client = self.node.create_client(Trigger, '/coverage/start')
        self.force_client = self.node.create_client(
            Trigger, '/coverage/force_abandon')
        self.rearm_client = self.node.create_client(Trigger, '/coverage/rearm')
        self.action_client = ActionClient(
            self.node, ExecuteCoverage, '/coverage/execute')
        self.spin_thread = Thread(target=rclpy.spin, args=(self.node,))
        self.spin_thread.start()

        fixture = os.path.join(
            os.path.dirname(__file__), 'fixtures', 'unanswering_executor.py')
        self.executor_process = subprocess.Popen(
            [sys.executable, fixture], stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, start_new_session=True)
        self.assertTrue(self.action_client.wait_for_server(timeout_sec=10.0))
        self.assertTrue(self.start_client.wait_for_service(timeout_sec=10.0))

    def tearDown(self):
        if self.executor_process.poll() is None:
            self.executor_process.kill()
        self.executor_process.wait(timeout=5.0)
        rclpy.shutdown()
        self.spin_thread.join(timeout=5.0)
        self.node.destroy_node()

    def _call(self, client):
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 10.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(future.done(), 'A manager service did not answer.')
        return future.result()

    def _wait(self, predicate, description, timeout=15.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            latest = self.statuses[-1] if self.statuses else None
            if latest is not None and predicate(latest):
                return latest
            time.sleep(0.05)
        self.fail('Timed out waiting for {}; last status was {}'.format(
            description, self.statuses[-1] if self.statuses else None))

    def test_operator_can_recover_without_calling_an_unknown_goal_finished(self):
        """SIGKILL after send but before response must have an explicit escape."""
        self.hold_publisher.publish(Bool(data=False))
        self.task_publisher.publish(_task())
        self._wait(lambda s: s.state == CoverageStatus.READY, 'READY')
        self.assertTrue(self._call(self.start_client).success)
        stopping = self._wait(
            lambda s: s.state == CoverageStatus.STOPPING and
            s.can_force_abandon,
            'the unanswered request to time out')
        self.assertFalse(stopping.can_start)

        self.executor_process.kill()
        self.executor_process.wait(timeout=5.0)
        time.sleep(2.0)
        self.assertEqual(self.statuses[-1].state, CoverageStatus.STOPPING)
        self.assertFalse(self.statuses[-1].can_start)

        self.assertTrue(self._call(self.force_client).success)
        locked = self._wait(
            lambda s: s.state == CoverageStatus.RECOVERY_LOCKED,
            'RECOVERY_LOCKED')
        self.assertFalse(locked.can_start)
        self.assertTrue(locked.can_rearm)
        self.assertIn(True, self.hold_requests)

        self.assertTrue(self._call(self.rearm_client).success)
        ready = self._wait(
            lambda s: s.state == CoverageStatus.READY and s.can_start,
            'READY after operator rearm')
        self.assertFalse(ready.can_rearm)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    """The manager itself must still exit cleanly."""

    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
