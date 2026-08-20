"""An abandoned start request must never speak for the task that replaced it."""

# Every callback on a goal is registered once and lives as long as the client.
# Nothing in rclcpp retires the ones belonging to a request that has been given
# up on, so the manager has to do it: a start whose response never arrived is
# reported as not started, and its acceptance can still turn up minutes later.
#
# Left unretired it lands on whatever is running by then. The late acceptance
# republished EXECUTING for a goal nobody was watching, the late feedback wrote
# an old task's progress into the new task's status, and the late result reset
# the goal handle and reported FINISHED over a run that was still going - which
# also takes the operator's stop entry away, since that is drawn from the
# handle.
#
# The stand-in executor here refuses to be cancelled, so the abandoned goal runs
# to completion and produces every one of those three callbacks.

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
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger


ABANDONED_REVISION = 7
LIVE_REVISION = 8

# Values no real run of this task would produce, so anything showing them in the
# manager status came from the abandoned goal and from nowhere else.
STRAY_SEGMENT = 99
STRAY_PROGRESS = 0.99


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Run the manager alone, with a start deadline short enough to test."""
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='climbot_control', executable='coverage_manager_node',
            parameters=[{
                'start_response_timeout_s': 1.0,
                # High, so nothing here is the executor-loss path: this test is
                # about a request that was given up on while the executor was
                # present the whole time.
                'executor_timeout_s': 60.0,
            }]),
        launch_testing.actions.ReadyToTest(),
    ])


def _pose(x, y):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.orientation.w = 1.0
    return pose


def _task(revision):
    task = CoverageTask()
    task.header.frame_id = 'odom'
    task.task_id = 'late-callback-test'
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


class TestCoverageManagerLateCallbacks(unittest.TestCase):
    """The manager must ignore anything an abandoned request says afterwards."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('late_callback_test')
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

        self.executor_node = rclpy.create_node('stub_executor')
        self.first_goal_seen = Event()
        self.release_first = Event()
        self.abandoned_finished = Event()
        self.end_of_test = Event()
        self.goals = 0
        # Reentrant, on a multi-threaded executor: the first goal's decision
        # blocks until the test releases it, and the second goal has to be
        # accepted while it is still blocked.
        self.server = ActionServer(
            self.executor_node, ExecuteCoverage, '/coverage/execute',
            goal_callback=self._decide,
            cancel_callback=self._refuse_cancel,
            execute_callback=self._execute,
            callback_group=ReentrantCallbackGroup())

        self.ros_executor = MultiThreadedExecutor(num_threads=4)
        self.ros_executor.add_node(self.node)
        self.ros_executor.add_node(self.executor_node)
        self.spin_thread = Thread(target=self.ros_executor.spin)
        self.spin_thread.start()
        self.assertTrue(self.start_client.wait_for_service(timeout_sec=10.0))

    def tearDown(self):
        self.end_of_test.set()
        self.release_first.set()
        self.ros_executor.shutdown(timeout_sec=5.0)
        self.spin_thread.join(timeout=10.0)
        self.server.destroy()
        self.executor_node.destroy_node()
        self.node.destroy_node()
        rclpy.shutdown()

    def _decide(self, goal_request):
        self.goals += 1
        if self.goals == 1:
            self.first_goal_seen.set()
            # Answering only when the test says so is what makes the manager
            # give up on this request while it is still in flight.
            self.release_first.wait(60.0)
        return GoalResponse.ACCEPT

    def _refuse_cancel(self, goal_handle):
        return CancelResponse.REJECT

    def _execute(self, goal_handle):
        if goal_handle.request.task.revision == LIVE_REVISION:
            # The live run stays running and says nothing, so any feedback the
            # manager reports can only have come from the abandoned goal.
            self.end_of_test.wait(60.0)
            goal_handle.abort()
            return ExecuteCoverage.Result()
        feedback = ExecuteCoverage.Feedback()
        feedback.current_segment = STRAY_SEGMENT
        feedback.progress = STRAY_PROGRESS
        feedback.state = ExecuteCoverage.Feedback.TRACK_LINE
        goal_handle.publish_feedback(feedback)
        time.sleep(0.5)
        goal_handle.succeed()
        result = ExecuteCoverage.Result()
        result.result_code = ExecuteCoverage.Result.SUCCESS
        result.message = 'abandoned goal finished on its own'
        self.abandoned_finished.set()
        return result

    def _call_start(self):
        future = self.start_client.call_async(Trigger.Request())
        deadline = time.monotonic() + 10.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(future.done(), 'The start service did not answer.')
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

    def test_a_late_answer_cannot_overwrite_the_task_that_replaced_it(self):
        """Response, feedback and result of an abandoned goal, in one run."""
        self.task_publisher.publish(_task(ABANDONED_REVISION))
        self._wait_until(
            lambda s: s.state == CoverageStatus.READY and s.revision == ABANDONED_REVISION,
            'the first preview')

        self.assertTrue(self._call_start().success)
        self.assertTrue(self.first_goal_seen.wait(10.0), 'The stub never saw the first goal.')
        timed_out = self._wait_until(
            lambda s: s.state == CoverageStatus.READY and 'timed out' in s.message,
            'the start request to be given up on')
        self.assertTrue(
            timed_out.can_start,
            'A start that was never answered has to leave the operator able to try again.')

        self.task_publisher.publish(_task(LIVE_REVISION))
        self._wait_until(
            lambda s: s.state == CoverageStatus.READY and s.revision == LIVE_REVISION,
            'the second preview')
        self.assertTrue(self._call_start().success)
        running = self._wait_until(
            lambda s: s.state == CoverageStatus.EXECUTING and s.revision == LIVE_REVISION,
            'the replacement task to start')
        self.assertTrue(running.can_cancel)
        mark = len(self.statuses)

        # Everything the abandoned goal has to say now arrives at once.
        self.release_first.set()
        self.assertTrue(
            self.abandoned_finished.wait(30.0),
            'The abandoned goal never ran, so none of its callbacks were tested.')
        # Its result is delivered asynchronously; give the manager room to act
        # on it wrongly before checking that it did not.
        time.sleep(1.5)

        later = self.statuses[mark:]
        self.assertNotIn(
            CoverageStatus.FINISHED, [status.state for status in later],
            'The abandoned goal finishing reported the live task as finished.')
        latest = self.statuses[-1]
        self.assertEqual(latest.state, CoverageStatus.EXECUTING)
        self.assertEqual(latest.revision, LIVE_REVISION)
        self.assertTrue(
            latest.can_cancel,
            'The stop entry was withdrawn by a goal that is not the one running.')
        self.assertNotEqual(
            latest.current_segment, STRAY_SEGMENT,
            'The abandoned goal wrote its progress into the live task.')
        self.assertNotAlmostEqual(latest.progress, STRAY_PROGRESS, places=3)
