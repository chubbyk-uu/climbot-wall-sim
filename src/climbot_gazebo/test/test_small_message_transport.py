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

"""Keep DDS endpoint discovery out of the control and localization data path."""

import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
import pytest

GAZEBO = Path(get_package_share_directory('climbot_gazebo'))
CONTROL = Path(get_package_share_directory('climbot_control'))
VARIABLE = 'FASTDDS_BUILTIN_TRANSPORTS'
PROFILE = 'FASTRTPS_DEFAULT_PROFILES_FILE'
CRITICAL = {
    'simulation_data_bridge', 'inspection_trigger_bridge', 'clock_throttle',
    'cmd_vel_watchdog', 'robot_state_publisher', 'total_station_sim',
    'wall_wheel_odom_adapter', 'wall_imu_adapter', 'ekf_filter_node',
    'localization_timing_monitor',
}


def _text(value, context):
    if isinstance(value, str):
        return value
    return ''.join(item.perform(context) for item in value)


# Transport settings do not depend on the wall. The default undulating wall
# builds its whole-wall surface assets on every expansion (3.5 s -> 100 s),
# which pushed this test past its CTest timeout under -j8.
FLAT_WALL = {'wall_surface': 'flat', 'wheel_compliance': 'false'}


def _environments(path, overrides=None):
    spec = importlib.util.spec_from_file_location('transport_' + path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    description = module.generate_launch_description()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            context.launch_configurations[entity.name] = _text(
                entity.default_value, context) if entity.default_value else ''
    context.launch_configurations.update(overrides or {})
    if hasattr(module, 'launch_setup'):
        actions = module.launch_setup(context)
    else:
        actions = []
        for entity in description.entities:
            if isinstance(entity, OpaqueFunction):
                actions.extend(entity.execute(context) or [])
            else:
                actions.append(entity)
    result = {}
    for action in actions:
        if not isinstance(action, Node):
            continue
        pairs = action._ExecuteLocal__process_description._Executable__additional_env or []
        if hasattr(pairs, 'items'):
            pairs = pairs.items()
        result[action._Node__node_name] = {
            _text(key, context): _text(value, context) for key, value in pairs}
    return result


@pytest.fixture(autouse=True)
def clean_transport_environment(monkeypatch):
    for key in [VARIABLE, PROFILE, 'RMW_IMPLEMENTATION', 'ROS_AUTOMATIC_DISCOVERY_RANGE']:
        monkeypatch.delenv(key, raising=False)
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')


def _assert_local_udp(environment, name):
    assert environment.get(PROFILE, '').endswith('/config/fastdds_control_local.xml'), name
    assert environment.get('ROS_LOCALHOST_ONLY') == '0', name
    assert environment.get('ROS_AUTOMATIC_DISCOVERY_RANGE') == 'SYSTEM_DEFAULT', name


def test_clock_localization_and_actuator_nodes_do_not_open_shared_memory_ports():
    envs = _environments(GAZEBO / 'launch/climbot_wall.launch.py', FLAT_WALL)
    assert CRITICAL <= set(envs)
    for name in CRITICAL:
        _assert_local_udp(envs[name], name)
    for name in ['inspection_camera_bridge', 'camera_distortion_adapter']:
        assert VARIABLE not in envs[name]
        assert envs[name]['FASTRTPS_DEFAULT_PROFILES_FILE'].endswith(
            'climbot_common/config/fastdds_inspection_image.xml')


def test_controller_and_manager_receive_the_same_latency_default():
    for filename, name in [('line_tracker.launch.py', 'line_tracker'),
                           ('coverage_executor.launch.py', 'coverage_manager')]:
        envs = _environments(CONTROL / 'launch' / filename)
        _assert_local_udp(envs[name], name)


def test_candidate_pressure_and_fan_bridge_receive_the_latency_default():
    envs = _environments(GAZEBO / 'launch/climbot_vacuum_stack.launch.py')
    _assert_local_udp(envs['vacuum_observation_bridge'], 'vacuum_observation_bridge')


@pytest.mark.parametrize('key,value', [
    (VARIABLE, 'DEFAULT'), (VARIABLE, 'SHM'),
    ('FASTRTPS_DEFAULT_PROFILES_FILE', '/operator/site.xml'),
])
def test_operator_transport_choices_are_inherited(monkeypatch, key, value):
    monkeypatch.setenv(key, value)
    envs = _environments(GAZEBO / 'launch/climbot_wall.launch.py', FLAT_WALL)
    envs.update(_environments(CONTROL / 'launch/line_tracker.launch.py'))
    for name in CRITICAL | {'line_tracker'}:
        assert PROFILE not in envs[name], name
        assert 'ROS_LOCALHOST_ONLY' not in envs[name], name


@pytest.mark.skipif(sys.platform != 'linux', reason='inspects Linux process transport resources')
def test_actual_fastdds_participant_has_no_shm_and_binds_only_loopback(tmp_path):
    """Environment variables alone are insufficient when RMW appends SHM."""
    from climbot_common.dds import local_dds_environment

    profile = (Path(get_package_share_directory('climbot_common')) /
               'config/fastdds_control_local.xml')
    environment = dict(os.environ, ROS_DOMAIN_ID='219', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    environment.update(local_dds_environment(profile, environment))
    ready = tmp_path / 'ready'
    log = tmp_path / 'participant.log'
    child = """import sys
from pathlib import Path
import rclpy
from rclpy.utilities import get_rmw_implementation_identifier
rclpy.init()
node = rclpy.create_node('climbot_local_dds_resource_probe')
try:
    assert get_rmw_implementation_identifier() == 'rmw_fastrtps_cpp'
    Path(sys.argv[1]).write_text('ready')
    sys.stdin.readline()
finally:
    node.destroy_node()
    rclpy.shutdown()
"""
    with log.open('w') as stream:
        process = subprocess.Popen(
            [sys.executable, '-c', child, str(ready)], env=environment,
            stdin=subprocess.PIPE, stdout=stream, stderr=subprocess.STDOUT)
    try:
        deadline = time.monotonic() + 30
        while not ready.exists():
            assert process.poll() is None, log.read_text()
            assert time.monotonic() < deadline, 'participant initialization timed out'
            time.sleep(.02)
        proc = Path('/proc') / str(process.pid)
        assert '/dev/shm/fastrtps_' not in (proc / 'maps').read_text()
        sockets = set()
        for fd in (proc / 'fd').iterdir():
            try:
                target = os.readlink(fd)
            except OSError:
                continue
            if target.startswith('socket:['):
                sockets.add(target[8:-1])
        udp = [line.split() for line in (proc / 'net/udp').read_text().splitlines()[1:]
               if line.split()[9] in sockets]
        assert udp, 'participant created no UDP sockets'
        assert all(row[1].split(':')[0] == '0100007F' for row in udp), udp
    finally:
        try:
            process.communicate(input=b'stop\n', timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
    assert process.returncode == 0, log.read_text()
    assert 'XMLPARSER Error' not in log.read_text()
