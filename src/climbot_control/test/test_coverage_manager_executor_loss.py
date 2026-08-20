"""Losing the executor must stop the robot before the task is called finished."""

# Two different facts get confused here, and only one of them is about the
# robot. "The Action server is undiscoverable" is the one the manager can see
# directly. "The robot has stopped" is the one it reports. A dead executor
# gives both at once, which is the case the first version of this test covered
# and the reason the shortcut looked safe.
#
# The other case is a live executor the manager cannot reach: a discovery
# stall, a wedged Action channel, a cancel that never lands. /control/cmd_vel
# keeps being refreshed, so the speed watchdog has nothing to time out and the
# robot drives on - while the manager publishes FINISHED, drops the goal handle
# and, because the stop entry is drawn from that handle, removes the operator's
# only way to intervene at the exact moment they need it.
#
# So the manager now stops the robot first and reports afterwards, and the
# evidence it accepts has to be about the robot: the speed hold is engaged, or
# nothing has commanded motion for a while, or the executor answered after all.

from threading import Event, Thread
import time
import unittest

from climbot_interfaces.action import ExecuteCoverage
from climbot_interfaces.msg import CoverageStatus, CoverageTask
from geometry_msgs.msg import Point32, Pose, Twist
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.action import ActionServer
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Run the manager alone; this test supplies its own executor."""
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='climbot_control', executable='coverage_manager_node',
            parameters=[{'executor_timeout_s': 1.0, 'command_quiet_s': 1.0}]),
        launch_testing.actions.ReadyToTest(),
    ])


# A revision of its own per test, so waiting on a status cannot be satisfied by
# the latched one the previous test left on the topic.
_revisions = iter(range(7, 100))


def _pose(x, y):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.orientation.w = 1.0
    return pose


def _task(revision):
    task = CoverageTask()
    task.header.frame_id = 'odom'
    task.task_id = 'loss-test'
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
        # Transient local, matching the manager: the recovery in setUp needs the
        # state the previous test left behind, and the manager only publishes on
        # events, so a volatile subscription would see nothing until something
        # happened.
        self.node.create_subscription(
            CoverageStatus, '/coverage/manager_status', self.statuses.append,
            rclpy.qos.QoSProfile(
                # Deep enough to keep every transition. STOPPING is passed
                # through in a single tick when nothing is driving, and at
                # depth 1 the reliable queue simply replaces it with the
                # FINISHED that follows.
                depth=50,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))
        self.start_client = self.node.create_client(Trigger, '/coverage/start')
        self.cancel_client = self.node.create_client(Trigger, '/coverage/cancel')
        self.command_publisher = self.node.create_publisher(Twist, '/control/cmd_vel', 10)
        # Stands in for the speed watchdog. It exists only when a test wants
        # the manager to have a stop path that does not go through the
        # executor, so the case where it has none is testable too.
        self.hold_publisher = None
        self.hold_service = None
        self.hold_requests = []

        # A stand-in executor that accepts the goal and never finishes it, which
        # is what a crashed controller looks like to the manager.
        self.executor_node = rclpy.create_node('stub_executor')
        self.accepted = Event()
        self.server = ActionServer(
            self.executor_node, ExecuteCoverage, '/coverage/execute',
            execute_callback=lambda goal_handle: ExecuteCoverage.Result(),
            handle_accepted_callback=self._accept)

        self.stop_spin = Event()
        self.stop_driving = Event()
        self.driver_thread = None
        self.mark = 0
        self.spin_thread = Thread(target=self._spin)
        self.spin_thread.start()
        self.assertTrue(self.start_client.wait_for_service(timeout_sec=10.0))
        self.assertTrue(self.cancel_client.wait_for_service(timeout_sec=10.0))
        # One manager node serves every test in this file, and each of these
        # tests deliberately leaves it having lost an executor. Whatever that
        # left behind is cleared here, so a test that fails fails on its own
        # behalf rather than on the previous one's leftovers.
        self._recover_the_manager()
        self.revision = next(_revisions)

    def _accept(self, goal_handle):
        # Deliberately never executed, so no result is ever published.
        self.goal_handle = goal_handle
        self.accepted.set()

    def _offer_the_speed_hold(self):
        self.hold_publisher = self.node.create_publisher(
            Bool, '/control/hold_active',
            rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))
        self.hold_publisher.publish(Bool(data=False))

        def serve(request, response):
            self.hold_requests.append(request.data)
            self.hold_publisher.publish(Bool(data=request.data))
            response.success = True
            return response

        self.hold_service = self.node.create_service(SetBool, '/control/hold', serve)

    def _keep_driving(self):
        """Publish motion on /control/cmd_vel, as a live executor would."""
        command = Twist()
        command.linear.x = 0.1
        while not self.stop_driving.is_set():
            self.command_publisher.publish(command)
            time.sleep(0.05)

    def tearDown(self):
        self.stop_driving.set()
        if self.driver_thread is not None:
            self.driver_thread.join(timeout=5.0)
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

    def _recover_the_manager(self, timeout=30.0):
        deadline = time.monotonic() + timeout
        asked = False
        while time.monotonic() < deadline:
            latest = self.statuses[-1] if self.statuses else None
            if latest is not None:
                if latest.can_start or latest.state in (
                        CoverageStatus.IDLE, CoverageStatus.INVALID,
                        CoverageStatus.READY, CoverageStatus.FINISHED):
                    return
                if not asked:
                    self._call(self.cancel_client)
                    asked = True
            time.sleep(0.1)
        self.fail('The manager never came back to a startable state; last '
                  'status was {}'.format(self.statuses[-1] if self.statuses else None))

    def _start_once_the_executor_is_visible(self, timeout=30.0):
        """Retry the start until the manager can see the stand-in executor."""
        # The manager refuses to start while the Action server is undiscovered,
        # which is a correct refusal and not what any of these tests are about.
        # Under a full parallel test run that discovery is not instant.
        deadline = time.monotonic() + timeout
        response = None
        while time.monotonic() < deadline:
            response = self._call(self.start_client)
            if response.success:
                return response
            time.sleep(0.2)
        self.fail('The manager never accepted a start; last refusal was {}'.format(
            response.message if response else None))

    def _wait_until(self, predicate, what, timeout=30.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            latest = self.statuses[-1] if self.statuses else None
            if latest is not None and predicate(latest):
                return latest
            time.sleep(0.05)
        self.fail('Timed out waiting for {}; last status was {}'.format(
            what, self.statuses[-1] if self.statuses else None))

    def _run_until_the_executor_is_lost(self):
        """Get to an accepted goal, then take the Action server away."""
        # Everything before this belongs to a previous test, including the
        # latched status this node received the moment it subscribed.
        self.mark = len(self.statuses)
        self.task_publisher.publish(_task(self.revision))
        self._wait_until(
            lambda s: s.state == CoverageStatus.READY and s.revision == self.revision,
            'the preview')
        self._start_once_the_executor_is_visible()
        self._wait_until(
            lambda s: s.state == CoverageStatus.EXECUTING and s.revision == self.revision,
            'execution')
        self.assertTrue(self.accepted.wait(5.0), 'The stub executor never got the goal.')
        self._destroy_executor()

    def test_a_dead_executor_releases_the_task(self):
        """Nothing is driving, so there is nothing to stop; still, say why."""
        self._run_until_the_executor_is_lost()
        released = self._wait_until(
            lambda s: s.state == CoverageStatus.FINISHED,
            'the manager to release the goal once nothing was commanding motion')
        # Checked over the whole history rather than waited for: with nothing
        # commanding motion the evidence is already in, so STOPPING is passed
        # through in one tick. It still has to be passed through, and it still
        # has to keep the stop entry while it is.
        stopping = [status for status in self.statuses[self.mark:]
                    if status.state == CoverageStatus.STOPPING]
        self.assertTrue(
            stopping,
            'The manager went straight from executing to finished without ever '
            'stopping the robot.')
        self.assertTrue(
            stopping[0].can_cancel,
            'The stop entry has to survive losing the executor; it is withdrawn '
            'only once the robot is known not to be driving.')
        # Not CONTROL_TIMEOUT: no controller timed out here. That code named a
        # cause that had not happened and sent an operator to look at the
        # tracker instead of at the connection.
        self.assertEqual(released.result_code, ExecuteCoverage.Result.EXECUTOR_LOST)
        self.assertIn('after losing the executor', released.message)
        self.assertIn('commanded motion', released.message)
        self.assertTrue(
            released.can_start,
            'The operator must be able to start again without restarting the '
            'manager.')
        self.assertFalse(released.can_cancel)

    def test_an_unreachable_but_still_driving_executor_is_not_called_finished(self):
        """The case the dead-executor test cannot reach, and the dangerous one."""
        self.driver_thread = Thread(target=self._keep_driving)
        self.driver_thread.start()
        self._run_until_the_executor_is_lost()

        stopping = self._wait_until(
            lambda s: s.state == CoverageStatus.STOPPING,
            'the manager to start stopping')
        self.assertTrue(stopping.can_cancel)
        self.assertFalse(stopping.can_start)

        # Well past the point the old code called this finished: the Action
        # server has been gone for several timeouts, and the robot is still
        # being commanded to move the whole time.
        time.sleep(4.0)
        latest = self.statuses[-1]
        self.assertEqual(
            latest.state, CoverageStatus.STOPPING,
            'The manager reported a run as finished while motion commands were '
            'still arriving and it had no way to stop them.')
        self.assertTrue(
            latest.can_cancel,
            'An operator watching a robot that has not stopped must still have '
            'a stop entry.')
        self.assertNotIn(
            CoverageStatus.FINISHED,
            [status.state for status in self.statuses[self.mark:]])

        # A stop path that does not go through the executor turns up. That is
        # evidence about the robot rather than about the connection, so the
        # manager may now release the task.
        self._offer_the_speed_hold()
        released = self._wait_until(
            lambda s: s.state == CoverageStatus.FINISHED,
            'the manager to release the task once the robot was held')
        self.assertEqual(released.result_code, ExecuteCoverage.Result.EXECUTOR_LOST)
        self.assertIn('the hold is engaged', released.message)
        self.assertIn(
            True, self.hold_requests,
            'The manager never engaged the one stop path that does not depend '
            'on the executor answering.')
        self.assertTrue(released.can_start)
        self.assertFalse(released.can_cancel)

    def test_the_operator_stop_still_does_something_while_stopping(self):
        """Pressing stop on a robot that has not stopped must not be a no-op."""
        self.driver_thread = Thread(target=self._keep_driving)
        self.driver_thread.start()
        self._offer_the_speed_hold()
        self._run_until_the_executor_is_lost()
        self._wait_until(
            lambda s: s.state == CoverageStatus.STOPPING or
            s.state == CoverageStatus.FINISHED, 'the manager to react to the loss')

        response = self._call(self.cancel_client)
        self.assertTrue(response.success)
        self.assertIn(
            True, self.hold_requests,
            'Stop during a loss did not reach the speed hold, which is the only '
            'path left when the executor is unreachable.')
