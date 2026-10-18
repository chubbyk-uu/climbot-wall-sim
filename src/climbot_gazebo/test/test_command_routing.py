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

"""Guard the single actuator-facing velocity-command route."""

import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = PACKAGE_ROOT / 'climbot_gazebo' / 'simulation_stack.py'


def _wall_launch_module():
    spec = importlib.util.spec_from_file_location('climbot_wall_launch', LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_experiment_scripts_use_guarded_command_input():
    """Prevent experiment tools from bypassing the velocity watchdog."""
    direct_publisher = "create_publisher(Twist, '/cmd_vel'"
    offenders = [
        script.name
        for script in (PACKAGE_ROOT / 'scripts').glob('*.py')
        if direct_publisher in script.read_text()
    ]
    assert offenders == []


def test_wall_launch_starts_watchdog():
    """Keep the watchdog in the standard wall-simulation launch."""
    source = LAUNCH_PATH.read_text()
    assert "package='climbot_control'" in source
    assert "executable='cmd_vel_watchdog_node'" in source
    assert "'/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'" in source
    assert "'/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'" not in source


def test_camera_transport_has_dedicated_bridge_executors():
    """Full-HD camera traffic cannot delay control or its reverse trigger."""
    source = LAUNCH_PATH.read_text()
    assert "name='simulation_data_bridge'" in source
    assert "name='inspection_camera_bridge'" in source
    assert "name='inspection_trigger_bridge'" in source
    trigger_route = "CAMERA_TRIGGER_TOPIC + '@std_msgs/msg/Bool]gz.msgs.Boolean'"
    assert source.count(trigger_route) == 1
    image_route = (
        'CAMERA_IDEAL_IMAGE_TOPIC +\n'
        "            '@sensor_msgs/msg/Image[gz.msgs.Image'")
    info_route = (
        'CAMERA_IDEAL_INFO_TOPIC +\n'
        "            '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'")
    assert source.count(image_route) == 1
    assert source.count(info_route) == 1


def test_gui_exit_cannot_terminate_the_required_simulation_server():
    """An intermittent WSLg GUI context failure must leave physics running."""
    source = LAUNCH_PATH.read_text()
    assert 'gz_sim_supervisor.py' in source
    assert "cmd=[gazebo_supervisor, 'server', '--world', world]" in source
    assert "name='gazebo_server', output='screen', on_exit=Shutdown()" in source
    assert "cmd=gui_command, name='gazebo_gui', output='screen'" in source
    assert 'period=2.0' in source


def test_gazebo_supervisor_forwards_shutdown_to_its_whole_process_group():
    """Keep GZ's Ruby launcher from orphaning its real server or GUI child."""
    source = (PACKAGE_ROOT / 'scripts' / 'gz_sim_supervisor.py').read_text()
    assert 'start_new_session=True' in source
    assert "environment['GALLIUM_DRIVER'] = 'llvmpipe'" in source
    assert 'os.killpg(self._child.pid, signal.SIGTERM)' in source
    assert 'return 0 if self._stopping else self._child.returncode' in source


def test_gazebo_supervisor_does_not_extend_its_kill_deadline_on_a_second_signal():
    """Launch escalation must not kill the supervisor before it reaps GZ."""
    supervisor_path = PACKAGE_ROOT / 'scripts' / 'gz_sim_supervisor.py'
    child = (
        'import signal,time; signal.signal(signal.SIGTERM, signal.SIG_IGN); '
        'time.sleep(60)')
    runner = (
        'import importlib.util,sys; '
        'spec=importlib.util.spec_from_file_location("supervisor", {!r}); '.format(
            str(supervisor_path)) +
        'module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module); '
        'raise SystemExit(module.GazeboSupervisor([sys.executable, "-c", {!r}]).run())'.format(
            child))
    process = subprocess.Popen([sys.executable, '-c', runner])
    try:
        time.sleep(0.2)
        os.kill(process.pid, signal.SIGINT)
        time.sleep(0.2)
        os.kill(process.pid, signal.SIGTERM)
        # If SIGTERM reset the deadline to eight more seconds, this wait would
        # time out. The fixed supervisor kills its whole child group in ~4 s.
        assert process.wait(timeout=5.5) == 0
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=5.0)


def test_simulation_adapters_exit_cleanly_after_launch_sigint():
    """Do not mask a required-process exit with duplicate rclpy shutdown errors."""
    # total_station_sim is a C++ node now; rclpy shutdown handling only has to
    # be asserted for the Python adapters that remain.
    for name in ('camera_distortion_adapter.py',):
        source = (PACKAGE_ROOT / 'scripts' / name).read_text()
        assert 'ExternalShutdownException' in source
        assert 'except (KeyboardInterrupt, ExternalShutdownException):' in source
        assert 'except RCLError:' in source
        assert 'node.destroy_node()' in source
        assert 'if rclpy.ok():' in source


def test_wall_launch_uses_the_current_total_station_delay_default():
    """Keep the documented 10 ms delivery delay from silently drifting."""
    source = LAUNCH_PATH.read_text()
    argument = source.rsplit("'total_station_delay_s'", maxsplit=1)[1]
    assert "default_value='0.01'" in argument.split('DeclareLaunchArgument', maxsplit=1)[0]


def test_total_station_node_uses_the_same_delay_when_run_standalone():
    """Launching the node directly must not restore the retired 50 ms default."""
    source = (PACKAGE_ROOT / 'src' / 'total_station_sim_node.cpp').read_text()
    assert 'declare_parameter("fixed_delay_s", 0.01)' in source


def test_rendered_launch_assets_are_removed_on_shutdown(tmp_path):
    """Do not leak the generated SDF and temporary model directory into /tmp."""
    world_path = tmp_path / 'climbot_wall.sdf'
    world_path.write_text('<sdf/>')
    model_root = tmp_path / 'climbot_model'
    (model_root / 'climbot').mkdir(parents=True)
    (model_root / 'climbot' / 'model.sdf').write_text('<sdf/>')

    _wall_launch_module().cleanup_rendered_assets(
        None, world_path=str(world_path), model_root=str(model_root))

    assert not world_path.exists()
    assert not model_root.exists()


def _supervisor_module():
    spec = importlib.util.spec_from_file_location(
        'supervisor_readiness_test', PACKAGE_ROOT / 'scripts/gz_sim_supervisor.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _transport(monkeypatch, on_subscribe):
    removed = []

    class Node:
        def subscribe(self, kind, topic, callback):
            on_subscribe(callback)
            return True

        def unsubscribe(self, topic):
            removed.append(topic)

    monkeypatch.setitem(sys.modules, 'gz.transport13', SimpleNamespace(Node=Node))
    monkeypatch.setitem(sys.modules, 'gz.msgs10.world_stats_pb2',
                        SimpleNamespace(WorldStatistics=object))
    monkeypatch.setitem(sys.modules, 'gz.msgs10.pose_pb2', SimpleNamespace(Pose=object))
    return removed


def test_gui_wait_requires_initialized_server_and_unsubscribes(monkeypatch):
    """Early discovery/iteration messages cannot release GUI initialization."""
    module = _supervisor_module()
    supervisor = module.GazeboSupervisor(['unused'], wait_for_render_ready=True)
    removed = _transport(monkeypatch, lambda callback:
                         callback(SimpleNamespace(iterations=99)))
    clock = iter([0., 181.])
    monkeypatch.setattr(module.time, 'monotonic', lambda: next(clock))
    with pytest.raises(RuntimeError, match='180s'):
        supervisor._wait_for_server()
    assert removed == ['/world/climbot_wall/stats']
    removed.clear()
    _transport(monkeypatch, lambda callback:
               callback(SimpleNamespace(iterations=100)))
    monkeypatch.setattr(module.time, 'monotonic', lambda: 0.)
    assert supervisor._wait_for_server()


def test_shutdown_while_waiting_never_starts_a_gui_child(monkeypatch):
    """Closing launch during a stalled server must leave no late GUI process."""
    module = _supervisor_module()
    supervisor = module.GazeboSupervisor(['unused'], wait_for_render_ready=True)
    supervisor._stopping = True
    removed = _transport(monkeypatch, lambda callback: None)
    monkeypatch.setattr(module.signal, 'signal', lambda *args: None)
    monkeypatch.setattr(module.subprocess, 'Popen',
                        lambda *a, **k: pytest.fail('GUI spawned after shutdown'))
    assert supervisor.run() == 0
    assert removed == ['/world/climbot_wall/stats']


@pytest.mark.parametrize('failure,rendered,expected_attempts,expected_exit', [
    ('both_once', False, 2, 0),
    ('both', False, 3, 250),
    ('both', True, 1, 250),
    ('device_only', False, 1, 250),
    ('unrelated', False, 1, 250),
])
def test_gui_recovery_is_bounded_and_only_before_first_frame(
        monkeypatch, tmp_path, capsys, failure, rendered, expected_attempts, expected_exit):
    """Real failed children may recover without hiding a rendered-client crash."""
    module = _supervisor_module()
    removed = _transport(monkeypatch, lambda callback: callback(object()) if rendered else None)
    monkeypatch.setattr(module.signal, 'signal', lambda *args: None)
    attempts = tmp_path / 'attempts'
    child = f"""
import os
from pathlib import Path
p = Path({str(attempts)!r})
n = int(p.read_text()) + 1 if p.exists() else 1
p.write_text(str(n))
assert os.environ['GALLIUM_DRIVER'] == 'd3d12'
if {failure!r} == 'both_once' and n > 1:
    raise SystemExit(0)
if {failure!r} != 'unrelated':
    print('D3D12: Removing Device.', flush=True)
if {failure!r} in ('both_once', 'both'):
    print('Failed to create OpenGL context for format QSurfaceFormat', flush=True)
raise SystemExit(250)
"""
    supervisor = module.GazeboSupervisor(
        [sys.executable, '-c', child], dict(os.environ, GALLIUM_DRIVER='d3d12'),
        gui_startup_retries=2)
    assert supervisor.run() == expected_exit
    assert int(attempts.read_text()) == expected_attempts
    assert removed == ['/gui/camera/pose']
    output = capsys.readouterr().out
    assert output.count('retrying unchanged GPU backend') == expected_attempts - 1
    if failure == 'both_once':
        assert 'Failed to create OpenGL context' in output


@pytest.mark.parametrize('stopping,elapsed', [(False, 31.), (True, 1.)])
def test_gui_does_not_retry_late_failures_or_shutdown(monkeypatch, stopping, elapsed):
    """The recovery window cannot cover steady operation or requested shutdown."""
    module = _supervisor_module()
    removed = _transport(monkeypatch, lambda callback: None)
    monkeypatch.setattr(module.signal, 'signal', lambda *args: None)
    clock = iter([0., elapsed])
    monkeypatch.setattr(module.time, 'monotonic', lambda: next(clock))
    supervisor = module.GazeboSupervisor(['unused'], gui_startup_retries=2)
    calls = []

    def child():
        calls.append(True)
        supervisor._child = SimpleNamespace(returncode=250)
        supervisor._stopping = stopping
        return {'D3D12: Removing Device.', 'Failed to create OpenGL context'}

    monkeypatch.setattr(supervisor, '_run_child', child)
    assert supervisor.run() == (0 if stopping else 250)
    assert len(calls) == 1
    assert removed == ['/gui/camera/pose']


def test_rviz_wait_requires_a_rendered_gui_pose_and_releases_subscription(monkeypatch):
    """Require a rendered GUI rather than only a server or empty window."""
    path = PACKAGE_ROOT.parents[1] / 'tools/with_mesa_runtime.py'
    spec = importlib.util.spec_from_file_location('mesa_client_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    removed = _transport(monkeypatch, lambda callback: None)
    with pytest.raises(RuntimeError, match='did not render'):
        module.wait_for_gazebo_gui(timeout=0.)
    assert removed == ['/gui/camera/pose']
    removed = _transport(monkeypatch, lambda callback: callback(object()))
    module.wait_for_gazebo_gui(timeout=0.)
    assert removed == ['/gui/camera/pose']
