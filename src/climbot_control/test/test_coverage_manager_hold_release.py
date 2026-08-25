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

"""An unresolved speed-hold release must leave an operator recovery path."""

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
import pytest
import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='climbot_control', executable='coverage_manager_node',
            parameters=[{
                'start_response_timeout_s': 0.5,
                'hold_response_timeout_s': 0.1,
                'hold_discovery_grace_s': 10.0,
            }]),
        launch_testing.actions.ReadyToTest(),
    ])


def _task():
    task = CoverageTask()
    task.header.frame_id = 'odom'
    task.task_id = 'unresolved-hold-test'
    task.revision = 1
    task.sweep_direction = CoverageTask.SWEEP_HORIZONTAL
    task.waypoints = [Pose(), Pose()]
    task.waypoints[1].position.x = 0.4
    for pose in task.waypoints:
        pose.orientation.w = 1.0
    task.segment_types = [CoverageTask.SEGMENT_SCAN]
    for x, y in ((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)):
        task.coverage_region.points.append(Point32(x=x, y=y))
        task.motion_region.points.append(Point32(x=x, y=y))
    task.detection_width = 0.1
    task.detection_length = 0.1
    return task


class TestCoverageManagerHoldRelease(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('unresolved_hold_test')
        self.statuses = []
        qos = rclpy.qos.QoSProfile(
            depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)
        self.task_pub = self.node.create_publisher(CoverageTask, '/coverage/task', qos)
        self.node.create_subscription(CoverageStatus, '/coverage/manager_status',
                                      self.statuses.append, qos)
        self.hold_pub = self.node.create_publisher(Bool, '/control/hold_active', qos)
        self.hold_pub.publish(Bool(data=True))
        # A recent hold status makes the manager wait for a release even before
        # DDS can discover the service. Deliberately advertise no service: the
        # high discovery grace models a controller whose hold endpoint vanished
        # at exactly that point, without blocking this test's own executor.
        self.executor_node = rclpy.create_node('unresolved_hold_executor')
        self.goal_count = 0

        def accept(_goal):
            self.goal_count += 1
            return GoalResponse.ACCEPT

        self.server = ActionServer(
            self.executor_node, ExecuteCoverage, '/coverage/execute',
            goal_callback=accept, execute_callback=lambda handle: ExecuteCoverage.Result(),
            callback_group=ReentrantCallbackGroup())
        self.start = self.node.create_client(Trigger, '/coverage/start')
        self.cancel = self.node.create_client(Trigger, '/coverage/cancel')
        self.rearm = self.node.create_client(Trigger, '/coverage/rearm')
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.executor.add_node(self.executor_node)
        self.thread = Thread(target=self.executor.spin)
        self.thread.start()
        for client in (self.start, self.cancel, self.rearm):
            self.assertTrue(client.wait_for_service(timeout_sec=10.0))

    def tearDown(self):
        self.executor.shutdown(timeout_sec=5.0)
        self.thread.join(timeout=10.0)
        self.server.destroy()
        self.executor_node.destroy_node()
        self.node.destroy_node()
        rclpy.shutdown()

    def _call(self, client):
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(future.done())
        return future.result()

    def _wait(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.statuses and predicate(self.statuses[-1]):
                return self.statuses[-1]
            time.sleep(0.01)
        self.fail('Timed out; last state: {}'.format(self.statuses[-1] if self.statuses else None))

    def test_cancel_and_deadline_recover_from_an_unresolved_release(self):
        self.task_pub.publish(_task())
        self._wait(lambda status: status.state == CoverageStatus.READY)
        self.assertTrue(self._call(self.start).success)
        queued = self._wait(lambda status: status.state == CoverageStatus.STARTING)
        self.assertTrue(queued.can_cancel)
        self.assertEqual(self.goal_count, 0)

        self.assertTrue(self._call(self.cancel).success)
        locked = self._wait(lambda status: status.state == CoverageStatus.RECOVERY_LOCKED)
        self.assertFalse(locked.can_start)
        self.assertTrue(locked.can_rearm)
        self.assertEqual(self.goal_count, 0)

        self.assertTrue(self._call(self.rearm).success)
        self._wait(lambda status: status.state == CoverageStatus.READY)
        self.assertTrue(self._call(self.start).success)
        self._wait(lambda status: status.state == CoverageStatus.RECOVERY_LOCKED)
        self.assertEqual(self.goal_count, 0)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
