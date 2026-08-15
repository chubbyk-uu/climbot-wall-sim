"""Check that losing the executor does not lock the manager out permanently."""

from threading import Event, Thread
import time
import unittest

from climbot_interfaces.action import ExecuteCoverage
from climbot_interfaces.msg import CoverageStatus, CoverageTask
from geometry_msgs.msg import Point32, Pose
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.action import ActionServer
from std_srvs.srv import Trigger


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Run the manager alone; this test supplies its own executor."""
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='climbot_control', executable='coverage_manager_node',
            parameters=[{'executor_timeout_s': 1.0}]),
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
    task.task_id = 'loss-test'
    task.revision = 7
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


class TestCoverageManagerExecutorLoss(unittest.TestCase):
    """An accepted goal whose result never arrives must still be released."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('executor_loss_test')
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

        # A stand-in executor that accepts the goal and never finishes it, which
        # is what a crashed controller looks like to the manager.
        self.executor_node = rclpy.create_node('stub_executor')
        self.accepted = Event()
        self.server = ActionServer(
            self.executor_node, ExecuteCoverage, '/coverage/execute',
            execute_callback=lambda goal_handle: ExecuteCoverage.Result(),
            handle_accepted_callback=self._accept)

        self.stop_spin = Event()
        self.spin_thread = Thread(target=self._spin)
        self.spin_thread.start()
        self.assertTrue(self.start_client.wait_for_service(timeout_sec=10.0))

    def _accept(self, goal_handle):
        # Deliberately never executed, so no result is ever published.
        self.goal_handle = goal_handle
        self.accepted.set()

    def tearDown(self):
        self.stop_spin.set()
        self.spin_thread.join()
        self._destroy_executor()
        self.node.destroy_node()
        rclpy.shutdown()

    def _destroy_executor(self):
        if self.executor_node is not None:
            self.server.destroy()
            self.executor_node.destroy_node()
            self.executor_node = None

    def _spin(self):
        while rclpy.ok() and not self.stop_spin.is_set():
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if self.executor_node is not None:
                rclpy.spin_once(self.executor_node, timeout_sec=0.01)

    def _call(self, client):
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(future.done())
        return future.result()

    def _wait_until(self, predicate, what, timeout=30.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            latest = self.statuses[-1] if self.statuses else None
            if latest is not None and predicate(latest):
                return latest
            time.sleep(0.05)
        self.fail('Timed out waiting for {}; last status was {}'.format(
            what, self.statuses[-1] if self.statuses else None))

    def test_a_vanished_executor_releases_the_task(self):
        """Otherwise the manager reports EXECUTING forever and refuses to start."""
        self.task_publisher.publish(_task())
        self._wait_until(lambda s: s.state == CoverageStatus.READY, 'the preview')
        self.assertTrue(self._call(self.start_client).success)
        self._wait_until(lambda s: s.state == CoverageStatus.EXECUTING, 'execution')
        self.assertTrue(self.accepted.wait(5.0), 'The stub executor never got the goal.')

        self._destroy_executor()
        released = self._wait_until(
            lambda s: s.state == CoverageStatus.FINISHED,
            'the manager to release the goal after the executor vanished')
        self.assertEqual(released.result_code, ExecuteCoverage.Result.CONTROL_TIMEOUT)
        self.assertIn('Executor disappeared', released.message)
        self.assertTrue(
            released.can_start,
            'The operator must be able to start again without restarting the '
            'manager.')
        self.assertFalse(released.can_cancel)
