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
from pathlib import Path


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
