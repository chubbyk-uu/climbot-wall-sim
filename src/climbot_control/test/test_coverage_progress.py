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

"""Check that the start approach is not counted as task progress."""

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

FIRST_WAYPOINT_X = 1.0


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
            package='climbot_control', executable='coverage_manager_node',
            parameters=[{'feedback_publish_period_s': 0.0}]),
        launch_testing.actions.ReadyToTest(),
    ])


def _pose(x, y, yaw):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def _distant_task():
    # The first waypoint is far enough away that the executor inserts a start
    # approach, which is the situation this test is about.
    task = CoverageTask()
    task.header.frame_id = 'odom'
    task.task_id = 'progress-test'
    task.revision = 3
    task.sweep_direction = CoverageTask.SWEEP_HORIZONTAL
    task.waypoints = [
        _pose(FIRST_WAYPOINT_X, 0.0, 0.0), _pose(FIRST_WAYPOINT_X + 0.4, 0.0, 0.0)]
    task.segment_types = [CoverageTask.SEGMENT_SCAN]
    for x, y in [(-2.0, -2.0), (2.0, -2.0), (2.0, 2.0), (-2.0, 2.0)]:
        point = Point32(x=x, y=y)
        task.coverage_region.points.append(point)
        task.motion_region.points.append(point)
    task.detection_width = 0.1
    task.detection_length = 0.1
    return task


class TestCoverageProgress(unittest.TestCase):
    """Progress must describe the task, not the drive that precedes it."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('coverage_progress_test')
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
        deadline = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return future.result() if future.done() else None

    def _publish_odom(self, x):
        message = Odometry()
        message.header.frame_id = 'odom'
        message.pose.pose = _pose(x, 0.0, 0.0)
        self.odom_publisher.publish(message)

    def _wait_for_state(self, state, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if any(status.state == state for status in self.statuses):
                return
            self._publish_odom(0.0)
            time.sleep(0.01)
        self.fail('No status with state {} arrived.'.format(state))

    def test_start_approach_does_not_report_task_progress(self):
        """Approach travel used to drive progress up and then back to zero."""
        self.assertTrue(self.start_client.wait_for_service(timeout_sec=5.0))
        self.task_publisher.publish(_distant_task())
        self._wait_for_state(CoverageStatus.READY)
        self._publish_odom(0.0)
        time.sleep(0.2)
        self.assertTrue(self._call(self.start_client).success)
        self._wait_for_state(CoverageStatus.EXECUTING)

        # Walk the robot along the approach line towards the first waypoint.
        for step in range(60):
            self._publish_odom(min(0.55, step * 0.01))
            time.sleep(0.02)

        approach = [
            status for status in self.statuses
            if status.state == CoverageStatus.EXECUTING and status.current_segment < 0]
        self.assertTrue(approach, 'The executor never reported a start approach.')
        worst = max(status.progress for status in approach)
        self.assertEqual(
            worst, 0.0,
            'Start approach reported {:.3f} task progress; the approach is not a '
            'task segment, so its travel makes progress climb and then fall back '
            'to zero when the first segment begins.'.format(worst))


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    """A crashed executor or manager must fail the launch test."""

    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
