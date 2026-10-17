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

"""Native physics regression for pressure startup, pause and world reset."""

import csv
import json
import os
from pathlib import Path
import subprocess
import uuid

from ament_index_python.packages import get_package_prefix
from climbot_gazebo.mesa_runtime import rendering_environment
from climbot_gazebo.vacuum_assets import build_assets
import pytest
import yaml


@pytest.mark.parametrize('mode', ['fixed_pressure', 'fan_ode'])
def test_native_observer_and_vacuum_survive_reset(tmp_path, mode):
    """Use an actual Gazebo process; a regenerated formula cannot pass this."""
    build_assets(tmp_path, mode=mode, realtime_factor=0)
    request = tmp_path / 'request.yaml'
    request.write_text(yaml.safe_dump({
        'world': 'world.sdf', 'step_s': .001, 'duration_s': 4,
        'pause_at_s': .3, 'pause_iterations': 20, 'reset_at_s': 1.5,
    }))
    prefix = Path(get_package_prefix('climbot_gazebo'))
    env, identity = rendering_environment()
    env['GZ_PARTITION'] = 'climbot_s3_test_' + uuid.uuid4().hex
    env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = os.pathsep.join(filter(None, [
        str(prefix / 'lib'), env.get('GZ_SIM_SYSTEM_PLUGIN_PATH')]))
    output = tmp_path / 'native'
    log = tmp_path / 'native.log'
    with log.open('w') as stream:
        result = subprocess.run([
            str(prefix / 'lib/climbot_gazebo/vacuum_sim_probe'), str(request), str(output)],
            env=env, stdout=stream, stderr=subprocess.STDOUT, timeout=60)
    assert result.returncode == 0, f'Native lifecycle failed; inspect {log}'
    events = json.loads((output / 'run.json').read_text())
    assert events['native_frames'] == 4000
    assert events['pause_state_equal'] and events['pause_pose_equal']
    assert events['reset_restored'] and events['reset_snapshots'] == 1
    assert not events['last_runtime_failed'] and not events['last_startup_failed']
    libraries = json.loads((output / 'runtime_libraries.json').read_text())
    if identity['mesa'] == 'private':
        assert libraries['private_mapping_complete']
        assert not libraries['system_gallium_mapped']
    with (output / 'frames.csv').open() as stream:
        free = [row for row in csv.DictReader(stream)
                if row['paused'] == '0' and row['vac_fixture_active'] == '0']
    assert len(free) > 1000
    assert float(free[-1]['vac_generation']) == 1
    for row in free:
        for side, floor in [('left', 5), ('right', 5), ('ball', 10)]:
            assert row[side + '_valid'] == '1'
            assert float(row[side + '_fx']) >= floor
        for kind in 'fm':
            for axis in 'xyz':
                assert float(row[f'vac_fixture_{kind}{axis}']) == 0
