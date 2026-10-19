# Copyright 2026 jerry
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Recover a continuing RViz subscriber after time rollback without replanning."""

from copy import deepcopy
import time
import unittest

from climbot_interfaces.msg import CoverageTask
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
from nav_msgs.msg import Path
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from rosidl_runtime_py.convert import message_to_ordereddict
from visualization_msgs.msg import MarkerArray


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Start a simulation-clock planner with a fixed task."""
    planner = launch_ros.actions.Node(
        package='climbot_coverage', executable='coverage_planner_node',
        parameters=[{
            'use_sim_time': True, 'input_mode': 'parameters',
            'lower_left': [3., 2.], 'upper_right': [6., 4.],
            'robot_length': .78, 'robot_width': .482, 'edge_clearance': .1,
            'wall_width': 10., 'wall_height': 8.,
        }])
    return launch.LaunchDescription([planner, launch_testing.actions.ReadyToTest()])


def geometry(message):
    """Ignore only visualization timestamps, preserve geometry and marker ids."""
    value = deepcopy(message)
    items = value.markers if isinstance(value, MarkerArray) else [value, *value.poses]
    for item in items:
        item.header.stamp.sec = 0
        item.header.stamp.nanosec = 0
    return message_to_ordereddict(value)


class TestVisualizationRecovery(unittest.TestCase):
    """The cached task is immutable while visual outputs recover automatically."""

    def test_clock_rollback_restores_visuals_without_publishing_another_task(self):
        rclpy.init()
        node = rclpy.create_node('visual_refresh_test')
        topics = {'grid': ('/coverage/wall_grid', MarkerArray),
                  'markers': ('/coverage/markers', MarkerArray),
                  'path': ('/coverage/path', Path)}
        values = {}
        tasks = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscriptions = [node.create_subscription(
            kind, topic, lambda message, key=name: values.update({key: message}), qos)
            for name, (topic, kind) in topics.items()]
        subscriptions.append(node.create_subscription(
            CoverageTask, '/coverage/task', tasks.append, qos))
        clock = node.create_publisher(Clock, '/clock', 10)

        def pump(seconds, predicate, timeout):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                tick = Clock()
                tick.clock.sec = seconds
                clock.publish(tick)
                rclpy.spin_once(node, timeout_sec=.02)
                if predicate():
                    return True
            return False

        try:
            self.assertTrue(pump(100, lambda: len(values) == 3 and bool(tasks), 8.))
            before = {name: geometry(message) for name, message in values.items()}
            task_before = message_to_ordereddict(tasks[-1])
            values.clear()
            tasks.clear()
            self.assertTrue(pump(1, lambda: len(values) == 3 and all(
                (m.markers[-1].header.stamp.sec if isinstance(m, MarkerArray)
                 else m.header.stamp.sec) == 1 for m in values.values()), 4.),
                'RViz-cleared grid, wall/regions and path were never restored')
            self.assertEqual(before, {name: geometry(m) for name, m in values.items()})
            self.assertFalse(tasks, 'visual recovery must not republish/revise execution tasks')
            self.assertTrue(task_before)
        finally:
            node.destroy_node()
            rclpy.shutdown()
