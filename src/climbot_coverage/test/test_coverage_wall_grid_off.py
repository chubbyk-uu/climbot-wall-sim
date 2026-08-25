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

"""Check that switching the reference grid off actually removes it."""

# The grid is switched off for runs that photograph the wall, where it is not
# scenery but a periodic high-contrast feature standing 3 mm off the surface a
# stitch treats as a plane. A switch that quietly leaves the grid drawn would
# be found in the images, one whole run later.

from threading import Event
from threading import Thread
import unittest

from climbot_interfaces.msg import CoverageTask
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Start a planner told to draw no grid."""
    planner = launch_ros.actions.Node(
        package='climbot_coverage',
        executable='coverage_planner_node',
        parameters=[{
            'input_mode': 'parameters',
            'region_type': 'rectangle',
            'lower_left': [3.0, 1.0],
            'upper_right': [9.0, 6.5],
            'robot_length': 0.76,
            'robot_width': 0.475,
            'edge_clearance': 0.1,
            'wall_width': 12.0,
            'wall_height': 9.0,
            'wall_grid_spacing': 0.0,
        }],
    )
    return launch.LaunchDescription([
        planner,
        launch_testing.actions.ReadyToTest(),
    ])


class TestWallGridOff(unittest.TestCase):
    """Observe the latched grid topic as RViz would."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('wall_grid_off_test')
        self.grid = None
        self.task = None
        self.grid_event = Event()
        self.task_event = Event()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(
            MarkerArray, '/coverage/wall_grid', self._grid_callback, qos)
        self.node.create_subscription(
            CoverageTask, '/coverage/task', self._task_callback, qos)
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

    def _grid_callback(self, message):
        self.grid = message
        self.grid_event.set()

    def _task_callback(self, message):
        self.task = message
        self.task_event.set()

    def test_no_grid_is_drawn_and_the_planner_still_plans(self):
        """Nothing to draw, and a message that clears whatever was there."""
        self.assertTrue(self.grid_event.wait(10.0), 'No wall grid message.')
        self.assertTrue(
            all(marker.action == Marker.DELETEALL
                for marker in self.grid.markers),
            'grid spacing 0 still published something to draw')
        # Silence would be indistinguishable from a node that failed to start,
        # and a subscriber that connects later would keep an old grid forever.
        self.assertTrue(self.grid.markers, 'nothing published to clear the grid')
        self.assertTrue(self.task_event.wait(10.0), 'No coverage task received.')
        self.assertGreater(len(self.task.waypoints), 2)
