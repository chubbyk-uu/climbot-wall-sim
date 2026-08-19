"""Check what start and cancel do from every manager state."""

import math
from threading import Event, Thread
import time
import unittest

from climbot_interfaces.msg import CoverageStatus, CoverageTask
from geometry_msgs.msg import Point32, Pose
import launch
import launch_ros.actions
import launch_testing.actions
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


def _pose(x, y, yaw):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def _task(revision, task_id='states-test'):
    task = CoverageTask()
    task.header.frame_id = 'odom'
    task.task_id = task_id
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


def _cleared_task(revision):
    """Build the empty task a cleared selection produces."""
    task = CoverageTask()
    task.header.frame_id = 'odom'
    task.task_id = 'states-test'
    task.revision = revision
    task.sweep_direction = CoverageTask.SWEEP_HORIZONTAL
    task.detection_width = 0.1
    task.detection_length = 0.1
    return task


class TestCoverageManagerStates(unittest.TestCase):
    """Every button, in every state the operator can reach it from."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('coverage_manager_states_test')
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
        self.assertTrue(self.start_client.wait_for_service(timeout_sec=10.0))
        self.assertTrue(self.cancel_client.wait_for_service(timeout_sec=10.0))

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

    def _wait_until(self, predicate, what, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            latest = self._latest()
            if latest is not None and predicate(latest):
                return latest
            self._publish_odom()
            time.sleep(0.01)
        self.fail('Timed out waiting for {}; last status was {}'.format(
            what, self._latest()))

    def _reach_executing(self, revision):
        self.task_publisher.publish(_task(revision))
        self._wait_until(
            lambda s: s.state == CoverageStatus.READY and s.revision == revision,
            'the preview to be accepted')
        self.assertTrue(self._call(self.start_client).success)
        return self._wait_until(
            lambda s: s.state == CoverageStatus.EXECUTING, 'execution to start')

    def test_a_canceled_task_can_be_started_again(self):
        """Cancel leaves the preview cached, so a restart must stay offered."""
        self._reach_executing(11)
        self.assertTrue(self._call(self.cancel_client).success)
        finished = self._wait_until(
            lambda s: s.state == CoverageStatus.FINISHED, 'the cancel to land')
        self.assertTrue(
            finished.can_start,
            'A canceled task is still cached, so the panel must keep offering '
            'a restart instead of greying the button out.')
        self.assertFalse(finished.can_cancel)
        self.assertTrue(self._call(self.start_client).success)

    def test_a_new_preview_does_not_replace_the_running_task(self):
        """Replan during a run must not report the new task as the live one."""
        self._reach_executing(21)
        self.task_publisher.publish(_task(22))
        time.sleep(0.8)
        latest = self._latest()
        self.assertEqual(
            latest.state, CoverageStatus.EXECUTING,
            'A preview arriving mid-run reported the manager as no longer '
            'executing, which takes the cancel button away while the robot '
            'is still moving.')
        self.assertEqual(
            latest.revision, 21,
            'The status must keep identifying the task that is running, not '
            'the preview that would run next.')
        self.assertTrue(latest.can_cancel)
        self.assertFalse(latest.can_start)

    def test_clearing_points_during_a_run_keeps_the_stop_button(self):
        """Clearing only drops the preview; the run still has to be stoppable."""
        self._reach_executing(31)
        self.task_publisher.publish(_cleared_task(32))
        time.sleep(0.8)
        latest = self._latest()
        self.assertEqual(
            latest.state, CoverageStatus.EXECUTING,
            'Clearing the selection reported the manager as idle while a task '
            'was still executing.')
        self.assertTrue(
            latest.can_cancel, 'The stop path disappeared mid-run.')
        self.assertTrue(self._call(self.cancel_client).success)

    def test_clearing_points_after_a_run_blocks_a_restart(self):
        """With no selection there is nothing to start, and that is correct."""
        self._reach_executing(41)
        self.assertTrue(self._call(self.cancel_client).success)
        self._wait_until(
            lambda s: s.state == CoverageStatus.FINISHED, 'the cancel to land')
        self.task_publisher.publish(_cleared_task(42))
        idle = self._wait_until(
            lambda s: s.state == CoverageStatus.IDLE, 'the cleared preview')
        self.assertFalse(idle.can_start)
        self.assertFalse(idle.can_cancel)
        self.assertFalse(self._call(self.start_client).success)

    def test_start_and_cancel_are_refused_while_the_goal_is_being_accepted(self):
        """STARTING is a real state: neither button is legal inside it."""
        self.task_publisher.publish(_task(51))
        self._wait_until(
            lambda s: s.state == CoverageStatus.READY, 'the preview')
        starting = [s for s in self.statuses if s.state == CoverageStatus.STARTING]
        self.assertTrue(self._call(self.start_client).success)
        self._wait_until(
            lambda s: s.state == CoverageStatus.EXECUTING, 'execution to start')
        starting = [s for s in self.statuses if s.state == CoverageStatus.STARTING]
        self.assertTrue(starting, 'The manager never announced STARTING.')
        for status in starting:
            self.assertFalse(status.can_start)
            self.assertFalse(status.can_cancel)
