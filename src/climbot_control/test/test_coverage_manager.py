"""Node-level checks for explicit coverage task start and cancellation."""

import math
from threading import Event, Thread
import time
import unittest

from climbot_interfaces.msg import CoverageTask
from geometry_msgs.msg import Point32, Pose
import launch
import launch_ros.actions
import launch_testing.actions
from nav_msgs.msg import Odometry
import pytest
import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger


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
                'wheel_speed_limit': 0.30,
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


def _task():
    task = CoverageTask()
    task.header.frame_id = 'odom'
    task.task_id = 'manager-test'
    task.revision = 9
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
        self.node.create_subscription(String, '/coverage/manager_status',
                                      lambda msg: self.statuses.append(msg.data), 10)
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
        deadline = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(future.done())
        return future.result()

    def _publish_odom(self):
        message = Odometry()
        message.pose.pose = _pose(0.0, 0.0, 0.0)
        self.odom_publisher.publish(message)

    def test_requires_explicit_start_and_can_cancel(self):
        """A valid preview remains idle until start, then manager owns cancellation."""
        self.assertTrue(self.start_client.wait_for_service(timeout_sec=3.0))
        self.assertTrue(self.cancel_client.wait_for_service(timeout_sec=3.0))
        no_task = self._call(self.start_client)
        self.assertFalse(no_task.success)

        self.task_publisher.publish(_task())
        deadline = time.monotonic() + 3.0
        while (not any('Ready: manager-test revision 9' in status for status in self.statuses)
               and time.monotonic() < deadline):
            time.sleep(0.01)
        self.assertTrue(any(
            'Ready: manager-test revision 9' in status for status in self.statuses))

        for _ in range(4):
            self._publish_odom()
            time.sleep(0.02)
        started = self._call(self.start_client)
        self.assertTrue(started.success)
        deadline = time.monotonic() + 3.0
        while (not any('Executing manager-test revision 9' in status for status in self.statuses)
               and time.monotonic() < deadline):
            self._publish_odom()
            time.sleep(0.01)
        self.assertTrue(any(
            'Executing manager-test revision 9' in status for status in self.statuses))

        canceled = self._call(self.cancel_client)
        self.assertTrue(canceled.success)

    def test_reports_a_cleared_preview_as_idle_rather_than_malformed(self):
        """An empty task is how the planner clears a preview, not a fault."""
        self.assertTrue(self.start_client.wait_for_service(timeout_sec=3.0))
        empty = CoverageTask()
        empty.header.frame_id = 'odom'
        empty.task_id = 'manager-test'
        empty.revision = 10
        empty.sweep_direction = CoverageTask.SWEEP_HORIZONTAL
        empty.detection_width = 0.1
        empty.detection_length = 0.1
        self.task_publisher.publish(empty)
        deadline = time.monotonic() + 3.0
        while (not any('Idle: no coverage region selected.' in status
                       for status in self.statuses)
               and time.monotonic() < deadline):
            time.sleep(0.01)
        self.assertTrue(any(
            'Idle: no coverage region selected.' in status for status in self.statuses))
        self.assertFalse(any('No executable preview' in status for status in self.statuses))
        self.assertFalse(self._call(self.start_client).success)
