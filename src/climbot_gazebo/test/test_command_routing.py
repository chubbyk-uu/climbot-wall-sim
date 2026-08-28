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


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = PACKAGE_ROOT / 'launch' / 'climbot_wall.launch.py'


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


def test_camera_trigger_has_a_dedicated_bridge_executor():
    """A full-HD image callback must not starve the reverse trigger route."""
    source = LAUNCH_PATH.read_text()
    assert "name='simulation_data_bridge'" in source
    assert "name='inspection_trigger_bridge'" in source
    trigger_route = "CAMERA_TRIGGER_TOPIC + '@std_msgs/msg/Bool]gz.msgs.Boolean'"
    assert source.count(trigger_route) == 1


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
    for name in ('total_station_sim.py', 'camera_distortion_adapter.py'):
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
    """Launching the adapter directly must not restore the retired 50 ms default."""
    source = (PACKAGE_ROOT / 'scripts' / 'total_station_sim.py').read_text()
    assert "self.declare_parameter('fixed_delay_s', 0.01)" in source


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
