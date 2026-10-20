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

"""Node-level verification of a start approach that stops off its target."""

# M2 run r8 (flat wall, compliant cylinder tyres) slid 8 cm across its
# diagonal start approach. The tracker reached the end of the approach line,
# stopped there outside start_approach_exit_tolerance_m, and then held still
# until the 180 s segment timeout: past the line end trackLine commands no
# forward speed, and nothing restarted the leg. The plant below reproduces
# that by dropping the robot once it comes to rest at the end of the leg.

import math
from threading import Lock
from threading import Thread
import time
import unittest

from action_msgs.msg import GoalStatus
from climbot_interfaces.action import ExecuteCoverage
from climbot_interfaces.msg import CoverageTask
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.action import ActionClient
from rosgraph_msgs.msg import Clock


SIM_STEP_S = 0.01
TIME_SCALE = 10.0
REAL_TIME_MARGIN_S = 15.0
SEGMENT_TIMEOUT_S = 40.0
# Beyond the 60 mm exit tolerance, like the 80 mm r8 was left with.
DROP_M = 0.09


@pytest.mark.launch_test
def generate_test_description():
    """Start the tracker with a segment timeout the recovery must beat."""
    executor = launch_ros.actions.Node(
        package='climbot_control',
        executable='line_tracker_node',
        parameters=[{
            'standalone_mode': False,
            'use_sim_time': True,
            # No turn reserve: the plant does not slip on turns, so the only
            # error at the end of the approach is the injected drop.
            'turn_slip_per_degree_m': 0.0,
            'odometry_timeout_s': 2.0,
            'odometry_future_tolerance_s': 1.0,
            'segment_timeout_s': SEGMENT_TIMEOUT_S,
            'alignment_settle_duration_s': 0.05,
            'goal_settle_duration_s': 0.05,
            'final_approach_distance_m': 0.08,
            'final_approach_speed_mps': 0.06,
            'max_turn_angular_speed': 0.80,
            'max_turn_angular_acceleration': 2.0,
            'wheel_separation': 0.43,
            'wheel_speed_limit': 0.45,
            'wheel_acceleration_limit': 0.40,
        }],
    )
    return launch.LaunchDescription([
        executor,
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
    """Open on a horizontal scan, reached by a descending diagonal approach."""
    task = CoverageTask()
    task.header.frame_id = 'odom'
    task.task_id = 'start-approach-recovery-test'
    task.revision = 1
    task.sweep_direction = CoverageTask.SWEEP_HORIZONTAL
    task.waypoints = [
        _pose(0.60, 0.0, 0.0),
        _pose(1.40, 0.0, 0.0),
    ]
    task.segment_types = [CoverageTask.SEGMENT_SCAN]
    for polygon in (task.coverage_region, task.motion_region):
        for x, y in [(-0.6, -0.6), (1.8, -0.6), (1.8, 0.6), (-0.6, 0.6)]:
            point = Point32()
            point.x = x
            point.y = y
            polygon.points.append(point)
    task.detection_width = 0.5
    task.detection_length = 0.01
    return task


class TestStartApproachRecovery(unittest.TestCase):
    """Drop the robot at the end of its start approach and watch the result."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('start_approach_recovery_test')
        self.command = Twist()
        self.sim_time = 0.0
        self.lock = Lock()
        self.publisher = self.node.create_publisher(
            Odometry, '/odometry/filtered', 10)
        self.clock_publisher = self.node.create_publisher(Clock, '/clock', 10)
        self.node.create_subscription(
            Twist, '/control/cmd_vel', self._command_callback, 10)
        self.client = ActionClient(
            self.node, ExecuteCoverage, '/coverage/execute')
        self.stop_spin = False
        self.spin_thread = Thread(target=self._spin)
        self.spin_thread.start()

    def tearDown(self):
        self.stop_spin = True
        self.spin_thread.join()
        self.client.destroy()
        self.node.destroy_node()
        rclpy.shutdown()

    def _spin(self):
        while rclpy.ok() and not self.stop_spin:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def _command_callback(self, message):
        with self.lock:
            self.command = message

    def _advance(self, step=SIM_STEP_S):
        with self.lock:
            self.sim_time += step
            now = self.sim_time
        message = Clock()
        message.clock.sec = int(now)
        message.clock.nanosec = int(round((now - int(now)) * 1e9))
        self.clock_publisher.publish(message)
        time.sleep(step / TIME_SCALE)

    def _publish_odometry(self, x, y, yaw, linear=0.0, angular=0.0):
        message = Odometry()
        message.header.stamp = rclpy.time.Time(nanoseconds=int(self.sim_time * 1e9)).to_msg()
        message.pose.pose = _pose(x, y, yaw)
        message.twist.twist.linear.x = linear * math.cos(yaw)
        message.twist.twist.linear.y = linear * math.sin(yaw)
        message.twist.twist.angular.z = angular
        self.publisher.publish(message)

    def _pending(self, future, timeout_s):
        sim_deadline = self.sim_time + timeout_s
        real_deadline = time.monotonic() + timeout_s / TIME_SCALE + \
            REAL_TIME_MARGIN_S
        while (not future.done() and self.sim_time < sim_deadline and
               time.monotonic() < real_deadline):
            yield

    def _run(self, drops, sideways=False):
        """
        Displace the robot DROP_M at the end of each approach leg, `drops` times.

        Downhill is what r8 met. A re-approach from below then runs uphill, so
        a further downhill drop only lengthens it; `sideways` pushes to the
        robot's right instead, across whichever leg it is on.
        """
        self.assertTrue(self.client.wait_for_server(timeout_sec=15.0))
        x, y, yaw = 0.0, 0.30, 0.0
        for _ in range(5):
            self._publish_odometry(x, y, yaw)
            self._advance(0.02)

        goal = ExecuteCoverage.Goal()
        goal.task = _task()
        send_future = self.client.send_goal_async(goal)
        for _ in self._pending(send_future, 15.0):
            self._publish_odometry(x, y, yaw)
            self._advance()
        self.assertTrue(send_future.done())
        handle = send_future.result()
        self.assertTrue(handle.accepted)
        result_future = handle.get_result_async()

        driven = 0.0
        drop_times = []
        for _ in self._pending(result_future, 4.0 * SEGMENT_TIMEOUT_S):
            with self.lock:
                linear = self.command.linear.x
                angular = self.command.angular.z
            yaw += angular * SIM_STEP_S
            x += linear * math.cos(yaw) * SIM_STEP_S
            y += linear * math.sin(yaw) * SIM_STEP_S
            driven += abs(linear) * SIM_STEP_S
            # Braking below 0.02 m/s after a real drive is the end of a leg,
            # still above the tracker's 0.01 m/s stopped speed so its arrival
            # settle cannot have begun. In-place turns drive no distance, and
            # x < 0.9 keeps the drops on the start approach, not the scan end.
            if (len(drop_times) < drops and abs(linear) < 0.02 and driven > 0.05 and
                    x < 0.9):
                if sideways:
                    x += DROP_M * math.sin(yaw)
                    y -= DROP_M * math.cos(yaw)
                else:
                    y -= DROP_M
                driven = 0.0
                drop_times.append(self.sim_time)
            self._publish_odometry(x, y, yaw, linear, angular)
            self._advance()
        self.assertTrue(result_future.done(), 'the task never finished')
        return result_future.result(), drop_times, (x, y, yaw)

    def test_a_drop_at_the_approach_end_is_re_approached(self):
        result, drop_times, final = self._run(drops=1)
        self.assertEqual(len(drop_times), 1, 'the approach never came to rest')
        self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(
            result.result.result_code, ExecuteCoverage.Result.SUCCESS,
            result.result.message)
        x, y, _ = final
        self.assertGreater(x, 1.30, 'the scan did not reach its end')
        self.assertLess(
            abs(y), 0.030, 'finished %0.1f mm off the scan line' % (1000.0 * y))

    def test_a_robot_that_keeps_dropping_fails_before_the_segment_timeout(self):
        result, drop_times, _ = self._run(drops=10, sideways=True)
        self.assertEqual(
            result.result.result_code, ExecuteCoverage.Result.TRACKING_FAILED,
            result.result.message)
        self.assertIn('Start approach', result.result.message)
        # The default two re-approaches, then a clear failure: three drops,
        # and well inside one segment timeout of the last of them.
        self.assertEqual(len(drop_times), 3)
        self.assertLess(self.sim_time - drop_times[-1], 0.5 * SEGMENT_TIMEOUT_S)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    """A crashed executor must fail the launch test."""

    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
