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

"""Exercise the configured geometry profile through the real launch file."""

import os
from threading import Event, Thread
import unittest

from ament_index_python.packages import get_package_share_directory
from climbot_interfaces.msg import CoverageTask
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch a frozen historical planner configuration without camera injection."""
    coverage_share = get_package_share_directory('climbot_coverage')
    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coverage_share, 'launch', 'coverage_planner.launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'rviz': 'false',
            'config_file': os.path.join(
                coverage_share, 'config', 'coverage_horizontal_large.yaml'),
            'input_mode': 'parameters',
            'region_type': 'rectangle',
            'sweep_direction': 'horizontal',
            'inspection_geometry_profile': 'configured',
        }.items())
    return launch.LaunchDescription([
        planner,
        launch_testing.actions.ReadyToTest(),
    ])


class TestConfiguredGeometryProfile(unittest.TestCase):
    """A configured historical run must publish the frozen task geometry."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('configured_geometry_profile_test')
        self.task = None
        self.received = Event()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(CoverageTask, '/coverage/task', self._on_task, qos)
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
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def _on_task(self, message):
        self.task = message
        self.received.set()

    def test_configured_profile_does_not_change_frozen_task_geometry(self):
        self.assertTrue(self.received.wait(10.0), 'No planner task received.')
        self.assertEqual(self.task.task_id, 'horizontal-rectangle-large')
        self.assertEqual(len(self.task.waypoints), 12)
        self.assertAlmostEqual(self.task.detection_width, 0.50, places=9)
        self.assertAlmostEqual(self.task.detection_length, 0.01, places=9)
        self.assertAlmostEqual(self.task.detection_forward_offset, 0.0, places=9)
