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
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node, NodeOptions
import pytest
import yaml


def test_all_fans_off_failure_remains_observable_and_reset_recovers(tmp_path):
    """After real descent, prescribe domain exit; fault truth must stay fresh."""
    events = [{'time_s': 1.2, 'kind': 0, 'index': i, 'value': 0} for i in range(3)]
    events += [{'time_s': 2.5, 'kind': 0, 'index': i, 'value': 1} for i in range(3)]
    build_assets(tmp_path, heading_deg=0, realtime_factor=0, io_overrides={'events': events})
    request = tmp_path / 'request.yaml'
    request.write_text(yaml.safe_dump({
        'world': 'world.sdf', 'step_s': .001, 'duration_s': 7,
        'reset_at_s': 3.2,
        # Impact depends on the collision proxy. Do not rely on a particular
        # tumble to trigger an exception: explicitly exit the supported normal
        # orientation after the real gravity-driven all-stop descent.
        'diagnostic_pose': {'time_s': 2.2, 'xyz_m': [2, 0, 2], 'rpy_rad': [0, 0, 0]},
    }))
    prefix = Path(get_package_prefix('climbot_gazebo'))
    env, _ = rendering_environment()
    env['GZ_PARTITION'] = 'climbot_failure_truth_' + uuid.uuid4().hex
    env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = os.pathsep.join(filter(None, [
        str(prefix / 'lib'), env.get('GZ_SIM_SYSTEM_PLUGIN_PATH')]))
    options = NodeOptions()
    options.partition = env['GZ_PARTITION']
    node = Node(options)
    messages = []
    assert node.subscribe(StringMsg, '/suction/ground_truth/state',
                          lambda msg: messages.append(json.loads(msg.data)))
    log = tmp_path / 'native.log'
    try:
        with log.open('w') as stream:
            result = subprocess.run([
                str(prefix / 'lib/climbot_gazebo/vacuum_sim_probe'), str(request),
                str(tmp_path / 'native')], env=env, stdout=stream,
                stderr=subprocess.STDOUT, timeout=60)
    finally:
        node.unsubscribe('/suction/ground_truth/state')
    (tmp_path / 'transport_truth.jsonl').write_text(
        ''.join(json.dumps(message) + '\n' for message in messages))
    assert result.returncode == 0, f'Failure/reset run failed; inspect {log}'
    lifecycle = json.loads((tmp_path / 'native/run.json').read_text())
    assert lifecycle['reset_restored']
    with (tmp_path / 'native/frames.csv').open() as stream:
        rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(stream)]
    for generation in [0, 1]:
        active = [r for r in rows if r['vac_generation'] == generation and r['paused'] == 0]
        failed = [r for r in active if r['vac_runtime_failed'] == 1]
        assert len(failed) > 200, 'Expected actual lost-seal/impact domain exit'
        # State time keeps advancing after failure; forces cannot retain a stale
        # pre-error value or silently restore the startup fixture.
        assert all(r['vac_time_s'] == pytest.approx(r['sim_time_s']) for r in failed)
        assert all(r['vac_fixture_active'] == 0 for r in failed)
        assert all(r['vac_last_valid_time_s'] < r['vac_time_s'] for r in failed)
        assert len({r['vac_sensor_stamp_s'] for r in failed}) == 1
        for field in ['pressure_f', 'pressure_m', 'skirt_f', 'skirt_m',
                      'fixture_f', 'fixture_m', 'rolling_m']:
            assert all(r['vac_' + field + axis] == 0 for r in failed for axis in 'xyz')
        assert failed[-1]['vac_fan_speed_0'] > .9  # command still applied; no reattachment
        assert all(r['vac_runtime_failed'] == 0 for r in active if r['sim_time_s'] < 1.2)
        public = [m for m in messages if m['generation'] == generation and m['runtime_failed']]
        assert len(public) >= 2, 'Fault must be visible on the actual transport topic'
        assert public[-1]['time_s'] > public[0]['time_s']


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


def test_explicit_pressure_bench_opening_ramp_pause_and_reset(tmp_path):
    """A labelled constrained bench must not masquerade as free holding."""
    opening = [{'time_s': t, 'area_m2': a} for t, a in [
        (0, 0), (.2, 0), (.4, .0006), (.6, .0006), (.8, 0)]]
    build_assets(tmp_path, realtime_factor=0,
                 runtime_overrides={'fixture_bench_hold': True},
                 io_overrides={'opening_schedule': opening})
    request = tmp_path / 'request.yaml'
    request.write_text(yaml.safe_dump({
        'world': 'world.sdf', 'step_s': .001, 'duration_s': 2,
        'pause_at_s': .3, 'pause_iterations': 10, 'reset_at_s': .9,
    }))
    prefix = Path(get_package_prefix('climbot_gazebo'))
    env, _ = rendering_environment()
    env['GZ_PARTITION'] = 'climbot_s5_test_' + uuid.uuid4().hex
    env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = os.pathsep.join(filter(None, [
        str(prefix / 'lib'), env.get('GZ_SIM_SYSTEM_PLUGIN_PATH')]))
    log = tmp_path / 'native.log'
    with log.open('w') as stream:
        result = subprocess.run([
            str(prefix / 'lib/climbot_gazebo/vacuum_sim_probe'), str(request),
            str(tmp_path / 'native')], env=env, stdout=stream,
            stderr=subprocess.STDOUT, timeout=60)
    assert result.returncode == 0, f'Native pressure bench failed; inspect {log}'
    state = json.loads((tmp_path / 'native/run.json').read_text())
    assert state['pause_state_equal'] and state['reset_restored']
    with (tmp_path / 'native/frames.csv').open() as stream:
        rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(stream)]
    for row in rows:
        assert row['vac_fixture_bench_hold'] == 1
        assert row['vac_fixture_active'] == 1
        assert row['vac_release_time_s'] == -1
        assert row['vac_runtime_failed'] == 0
        t = row['vac_time_s']
        expected = 0 if t <= .2 or t >= .8 else (
            .003 * (t - .2) if t <= .4 else (
                .0006 if t <= .6 else .003 * (.8 - t)))
        assert row['vac_controlled_opening_area_m2'] == pytest.approx(expected, abs=1e-14)
    for generation in [0, 1]:
        trace = [r for r in rows if r['vac_generation'] == generation and r['paused'] == 0]
        before = next(r for r in trace if abs(r['vac_time_s'] - .199) < 1e-8)
        open_port = next(r for r in trace if abs(r['vac_time_s'] - .599) < 1e-8)
        assert open_port['vac_delta_pa'] < before['vac_delta_pa']


@pytest.mark.parametrize('step', [.001, .0005])
def test_bottomed_skirt_keeps_pressure_and_has_native_stop_reaction(tmp_path, step):
    """Isolated seal bench: native stops carry load; wheels supply no reaction."""
    from xml.etree import ElementTree as ET
    build_assets(tmp_path, realtime_factor=0, step_s=step,
                 runtime_overrides={'fixture_bench_hold': True})
    path = tmp_path / 'world.sdf'
    tree = ET.parse(path)
    for name in ['left_wheel_link', 'right_wheel_link', 'caster_ball_link']:
        collision = tree.find(f'.//model[@name="climbot"]/link[@name="{name}"]/collision')
        surface = collision.find('surface')
        contact = ET.SubElement(surface, 'contact')
        ET.SubElement(contact, 'collide_bitmask').text = '0'
    tree.write(path)
    request = tmp_path / 'request.yaml'
    request.write_text(yaml.safe_dump({'world': 'world.sdf', 'step_s': step, 'duration_s': 2}))
    prefix = Path(get_package_prefix('climbot_gazebo'))
    env, _ = rendering_environment()
    env['GZ_PARTITION'] = 'climbot_stop_' + uuid.uuid4().hex
    env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = str(prefix / 'lib')
    log = tmp_path / 'native.log'
    with log.open('w') as stream:
        result = subprocess.run([str(prefix / 'lib/climbot_gazebo/vacuum_sim_probe'),
                                 str(request), str(tmp_path / 'native')], env=env,
                                stdout=stream, stderr=subprocess.STDOUT, timeout=60)
    assert result.returncode == 0, f'Native stop bench failed; inspect {log}'
    with (tmp_path / 'native/frames.csv').open() as stream:
        rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(stream)]
    assert all(r['vac_runtime_failed'] == 0 for r in rows)
    steady = [r for r in rows if r['sim_time_s'] > 1.5]
    assert len(steady) > 100
    assert min(r['vac_delta_pa'] for r in steady) > 6000
    assert min(r['vac_skirt_stop_previous_normal_n'] for r in steady) > 100
    assert min(r['vac_minimum_gap_m'] for r in steady) > .0115
    assert max(r['vac_minimum_gap_m'] for r in steady) < .0122
    assert all(r['vac_last_valid_time_s'] == pytest.approx(r['sim_time_s']) for r in steady)
