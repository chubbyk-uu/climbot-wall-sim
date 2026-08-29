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

"""
The total station must sample on simulation time, not on wall time.

Its rate is a property of the simulated survey instrument, so it has to follow
the simulator: a paused simulator must produce no new observations, and a
real-time factor below one must slow the station down with everything else.
The C++ port briefly used create_wall_timer, which is silent at a real-time
factor of one and wrong everywhere else, so these drive the clock directly
rather than relying on a run that happens to keep up.
"""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rosgraph_msgs.msg import Clock

SAMPLE_RATE_HZ = 12.0


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    station = launch_ros.actions.Node(
        package='climbot_gazebo',
        executable='total_station_sim_node',
        name='total_station_sim',
        parameters=[{
            'use_sim_time': True,
            'publish_rate_hz': SAMPLE_RATE_HZ,
            'position_stddev_m': 0.0,
            'fixed_delay_s': 0.0,
            'drop_probability': 0.0,
            'wall_config': os.path.join(
                get_package_share_directory('climbot_description'),
                'config', 'wall.yaml'),
        }],
    )
    return launch.LaunchDescription([
        station,
        launch_testing.actions.ReadyToTest(),
    ])


class TestTotalStationClock(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('total_station_clock_test')
        self.clock = self.node.create_publisher(Clock, '/clock', 10)
        self.truth = self.node.create_publisher(
            Odometry, '/model/climbot/ground_truth', 20)
        self.poses = []
        self.node.create_subscription(
            PoseWithCovarianceStamped, '/total_station/pose',
            lambda message: self.poses.append(message), 20)
        self._wait_for_discovery()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def _wait_for_discovery(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if (self.clock.get_subscription_count() > 0 and
                    self.truth.get_subscription_count() > 0):
                return
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.fail('total_station_sim_node never subscribed')

    def _advance(self, simulated_s, real_time_factor, step_s=0.01):
        """Drive the clock at a chosen ratio of simulated to wall time."""
        published = 0.0
        while published < simulated_s:
            published += step_s
            message = Clock()
            message.clock.sec = int(published)
            message.clock.nanosec = int((published - int(published)) * 1e9)
            self.clock.publish(message)
            truth = Odometry()
            truth.header.stamp = message.clock
            truth.pose.pose.position.x = 1.0
            truth.pose.pose.position.y = 2.0
            truth.pose.pose.orientation.w = 1.0
            self.truth.publish(truth)
            self._spin_for(step_s / real_time_factor)

    def _spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.002)

    def test_sampling_follows_simulated_time_not_wall_time(self):
        # Two simulated seconds at a quarter of real time. A wall-clock timer
        # would fire 12 times per real second and produce roughly 96 samples in
        # the eight seconds this takes; a simulation-time timer produces about
        # 12 per simulated second, so about 24.
        self._advance(simulated_s=2.0, real_time_factor=0.25)
        self._spin_for(0.5)
        expected = SAMPLE_RATE_HZ * 2.0
        self.assertGreater(
            len(self.poses), expected * 0.5,
            'sampled %d times in 2 simulated seconds' % len(self.poses))
        self.assertLess(
            len(self.poses), expected * 2.0,
            'sampled %d times in 2 simulated seconds, which is wall-time '
            'behaviour rather than simulation-time behaviour' % len(self.poses))

    def test_a_paused_simulator_produces_no_new_observations(self):
        self._advance(simulated_s=1.0, real_time_factor=1.0)
        self._spin_for(0.5)
        settled = len(self.poses)
        self.assertGreater(settled, 0, 'no observations before the pause')
        # Stop advancing the clock but keep the process alive and spinning.
        self._spin_for(2.0)
        self.assertEqual(
            len(self.poses), settled,
            'the station kept sampling while the simulator was paused')

    def test_observations_carry_distinct_source_timestamps(self):
        self._advance(simulated_s=1.5, real_time_factor=0.5)
        self._spin_for(0.5)
        stamps = [
            message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            for message in self.poses]
        self.assertGreater(len(stamps), 1, 'not enough observations to compare')
        # A wall-clock timer sampling a frozen truth repeats one source stamp.
        self.assertGreater(
            len(set(stamps)), len(stamps) * 0.5,
            'observations repeat the same source timestamp: %d distinct of %d'
            % (len(set(stamps)), len(stamps)))
