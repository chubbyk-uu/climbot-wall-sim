"""Check that a combined launch cannot divert the tracker's parameter file."""

import os
import tempfile
from threading import Event, Thread
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
import launch.actions
import launch.launch_description_sources
import launch_testing.actions
import launch_testing.markers
import pytest
from rcl_interfaces.srv import GetParameters
import rclpy
import yaml

FOREIGN_CONFIG = os.path.join(
    tempfile.gettempdir(), 'climbot_foreign_config.yaml')


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Include the executor from a scope that already defines config_file."""
    control_share = get_package_share_directory('climbot_control')
    # A parameter file for some other node parses cleanly and simply leaves the
    # tracker on its built-in defaults, which is what made this silent.
    with open(FOREIGN_CONFIG, 'w') as handle:
        yaml.safe_dump({'some_other_node': {'ros__parameters': {'unused': 1}}}, handle)
    executor = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            control_share + '/launch/coverage_executor.launch.py'))
    return launch.LaunchDescription([
        # A combined mission launch naturally declares config_file for its own
        # planner, and included launch files inherit that name.
        launch.actions.DeclareLaunchArgument(
            'config_file', default_value=FOREIGN_CONFIG),
        executor,
        launch_testing.actions.ReadyToTest(),
    ])


class TestCoverageExecutorConfig(unittest.TestCase):
    """Read the running tracker's parameters instead of trusting the launch."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('coverage_executor_config_test')
        self.client = self.node.create_client(
            GetParameters, '/line_tracker/get_parameters')
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
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_tracker_uses_its_own_control_configuration(self):
        """Slip compensation defaults to zero, so a diverted file is silent."""
        self.assertTrue(
            self.client.wait_for_service(timeout_sec=20.0),
            'The line tracker never advertised its parameter services.')
        control_share = get_package_share_directory('climbot_control')
        with open(control_share + '/config/control.yaml') as handle:
            expected = yaml.safe_load(handle)['line_tracker']['ros__parameters']
        names = [
            'gravity_slip_ratio',
            'turn_slip_per_degree_m',
            'final_approach_distance_m',
        ]
        request = GetParameters.Request(names=names)
        future = self.client.call_async(request)
        deadline = time.monotonic() + 10.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertTrue(future.done(), 'No parameter response from the tracker.')
        values = future.result().values
        self.assertEqual(len(values), len(names))
        for name, value in zip(names, values):
            self.assertAlmostEqual(value.double_value, float(expected[name]), places=9)
