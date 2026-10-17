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


def test_native_sensor_faults_are_isolated_from_commanded_fan_physics(tmp_path):
    """Compare two real physics runs, with identical fan failures and recovery."""
    records = []
    fan_events = [{'time_s': .15, 'kind': 1, 'index': 0, 'value': 1},
                  {'time_s': .3, 'kind': 1, 'index': 0, 'value': 0}]
    sensor_events = [{'time_s': .2, 'kind': 2, 'index': 0, 'value': 1},
                     {'time_s': .4, 'kind': 2, 'index': 0, 'value': 2},
                     {'time_s': .6, 'kind': 2, 'index': 0, 'value': 0}]
    for variant in ['normal', 'faults']:
        directory = tmp_path / variant
        events = fan_events + (sensor_events if variant == 'faults' else [])
        events = sorted(events, key=lambda e: e['time_s'])
        build_assets(directory, realtime_factor=0, io_overrides={'events': events})
        request = directory / 'request.yaml'
        request.write_text(yaml.safe_dump({
            'world': 'world.sdf', 'step_s': .001, 'duration_s': 2,
            'pause_at_s': .25, 'pause_iterations': 10, 'reset_at_s': 1.0,
        }))
        prefix = Path(get_package_prefix('climbot_gazebo'))
        env, _ = rendering_environment()
        env['GZ_PARTITION'] = 'climbot_s4_test_' + uuid.uuid4().hex
        env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = os.pathsep.join(filter(None, [
            str(prefix / 'lib'), env.get('GZ_SIM_SYSTEM_PLUGIN_PATH')]))
        output = directory / 'native'
        log = directory / 'native.log'
        with log.open('w') as stream:
            result = subprocess.run([
                str(prefix / 'lib/climbot_gazebo/vacuum_sim_probe'), str(request), str(output)],
                env=env, stdout=stream, stderr=subprocess.STDOUT, timeout=60)
        assert result.returncode == 0, f'Native S4 failed; inspect {log}'
        lifecycle = json.loads((output / 'run.json').read_text())
        assert lifecycle['pause_state_equal'] and lifecycle['reset_restored']
        with (output / 'frames.csv').open() as stream:
            rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(stream)]
        assert all(row['vac_runtime_failed'] == 0 for row in rows)
        assert all(row['vac_sensor_fault'] == 0 and row['vac_command_revision'] == 0
                   for row in rows if row['paused'] == 1 and row['vac_time_s'] == 0)
        for generation in [0, 1]:
            active = [r for r in rows if r['vac_generation'] == generation and r['paused'] == 0]
            assert active[0]['vac_sensor_sequence'] == 0
            assert active[0]['vac_sensor_stamp_s'] == -1
            before = next(r for r in active if abs(r['vac_time_s'] - .149) < 1e-8)
            after = next(r for r in active if abs(r['vac_time_s'] - .299) < 1e-8)
            assert after['vac_fan_speed_0'] < before['vac_fan_speed_0']
            assert after['vac_fan_speed_1'] > before['vac_fan_speed_1']
            if variant == 'faults':
                assert all(r['vac_sensor_stamp_s'] < .2 for r in active
                           if .2 <= r['vac_time_s'] < .4)
                stuck = [r for r in active if .43 <= r['vac_time_s'] < .6]
                assert len({r['vac_sensor_value_pa'] for r in stuck}) == 1
                assert stuck[-1]['vac_sensor_stamp_s'] > stuck[0]['vac_sensor_stamp_s']
        records.append(rows)
    excluded = {'vac_cpu_ns', 'vac_command_revision'}
    keys = [k for k in records[0][0] if k not in excluded and not k.startswith('vac_sensor_')]
    assert len(records[0]) == len(records[1])
    assert all(a[k] == b[k] for a, b in zip(*records) for k in keys)
