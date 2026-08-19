"""Check that the schedule is planned against a real pose, not the origin."""

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

# Offset sideways from the first waypoint so that planning from the origin and
# planning from the real pose disagree in both terms the schedule is made of:
# the drive is 2.0 m against 2.5 m, and the turn onto it is 0 deg against 37.
POSE_X = 0.0
POSE_Y = 1.5
FIRST_WAYPOINT_X = 2.0


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Run the real executor and manager, driven by synthetic odometry."""
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='climbot_control', executable='line_tracker_node',
            parameters=[{
                'standalone_mode': False,
                # Wide enough to leave a window in which the executor is
                # holding a goal and has no pose yet, which is the situation
                # under test.
                'odometry_timeout_s': 3.0,
                'segment_timeout_s': 60.0,
                'alignment_settle_duration_s': 0.05,
                'wheel_separation': 0.43,
                'wheel_speed_limit': 0.30,
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
    task = CoverageTask()
    task.header.frame_id = 'odom'
    task.task_id = 'schedule-pose-test'
    task.revision = 1
    task.sweep_direction = CoverageTask.SWEEP_HORIZONTAL
    task.waypoints = [
        _pose(FIRST_WAYPOINT_X, 0.0, 0.0), _pose(FIRST_WAYPOINT_X + 0.4, 0.0, 0.0)]
    task.segment_types = [CoverageTask.SEGMENT_SCAN]
    for x, y in [(-3.0, -3.0), (3.0, -3.0), (3.0, 3.0), (-3.0, 3.0)]:
        point = Point32(x=x, y=y)
        task.coverage_region.points.append(point)
        task.motion_region.points.append(point)
    task.detection_width = 0.1
    task.detection_length = 0.1
    return task


class TestCoverageSchedulePose(unittest.TestCase):
    """The schedule must be built from the pose the robot actually has."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('coverage_schedule_pose_test')
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

    def _publish_odom(self):
        message = Odometry()
        message.header.frame_id = 'odom'
        message.pose.pose = _pose(POSE_X, POSE_Y, 0.0)
        self.odom_publisher.publish(message)

    def _wait_for_state(self, state, timeout=5.0, odom=True):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if any(status.state == state for status in self.statuses):
                return
            if odom:
                self._publish_odom()
            time.sleep(0.01)
        self.fail('No status with state {} arrived.'.format(state))

    def _planned_total(self, since=0, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executing = [
                status.planned_total_s for status in self.statuses[since:]
                if status.state == CoverageStatus.EXECUTING and
                status.planned_total_s > 0.0]
            if executing:
                return executing[0]
            self._publish_odom()
            time.sleep(0.01)
        self.fail('No executing status carried a planned schedule.')

    def test_a_goal_accepted_before_the_first_pose_plans_from_that_pose(self):
        """The estimate used to be built from a default-constructed pose."""
        self.assertTrue(self.start_client.wait_for_service(timeout_sec=5.0))
        self.task_publisher.publish(_distant_task())
        self._wait_for_state(CoverageStatus.READY, odom=False)

        # Round one: start with no pose at all, so the executor holds the goal
        # in its wait-for-pose branch.
        self.assertTrue(self._call(self.start_client).success)
        self._wait_for_state(CoverageStatus.EXECUTING, odom=False)
        # Sampled over a window rather than at the instant EXECUTING first
        # appears: the manager's first EXECUTING status is the goal-accepted
        # transition and carries no schedule yet, so a single sample passes
        # whatever the executor would go on to publish.
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            time.sleep(0.01)
        waiting = [
            status.planned_total_s for status in self.statuses
            if status.state == CoverageStatus.EXECUTING]
        self.assertGreater(
            len(waiting), 5,
            'Too few statuses to tell a schedule of zero from one that had '
            'not been published yet.')
        self.assertEqual(
            max(waiting), 0.0,
            'A schedule of {:.1f} s was published before any pose had arrived. '
            'Planning needs the robot position, so with none the schedule has '
            'to read as unknown rather than as an estimate measured from the '
            'origin.'.format(max(waiting)))

        # The pose lands; this is where the schedule has to be built.
        deferred = self._planned_total()
        self.assertTrue(self._call(self.cancel_client).success)
        self._wait_for_state(CoverageStatus.FINISHED)

        # Round two: the same task and the same pose, but the pose is already
        # there when the goal is accepted. Both rounds plan the same drive from
        # the same place, so they have to agree.
        # Statuses are separated by index rather than cleared: with no task
        # running the manager has no feedback to republish from, so an emptied
        # list would simply stay empty.
        mark = len(self.statuses)
        for _ in range(30):
            self._publish_odom()
            time.sleep(0.01)
        self.assertTrue(self._call(self.start_client).success)
        immediate = self._planned_total(since=mark)

        self.assertAlmostEqual(
            deferred, immediate, delta=0.05,
            msg='Waiting for the pose gave a {:.1f} s schedule and having it '
                'gave {:.1f} s. The deferred one was planned once, at goal '
                'acceptance, from a pose that did not exist yet, and nothing '
                'recomputed it when the real one arrived.'.format(
                    deferred, immediate))
