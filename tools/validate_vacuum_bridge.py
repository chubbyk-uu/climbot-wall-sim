#!/usr/bin/env python3
"""Exercise the real S4 launch and ROS/Gazebo bridge, including live fault inputs."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

import rclpy
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Bool, Int32, String
import yaml


def run(output, mode):
    output.mkdir(parents=True, exist_ok=False)
    (output / 'io.yaml').write_text(yaml.safe_dump({'vacuum_io': {
        'sensor': {'noise_pa': 0, 'bias_pa': 0}}}))
    env = os.environ.copy()
    env['GZ_PARTITION'] = 'climbot_s4_bridge_' + uuid.uuid4().hex
    log = Path('/tmp') / ('climbot_s4_bridge_' + uuid.uuid4().hex + '.log')
    stream = log.open('w')
    rclpy.init()
    node = rclpy.create_node('vacuum_bridge_acceptance')
    states = []
    pressures = []
    ambient = []
    commands = []
    checks = {}
    trace = (output / 'ros.jsonl').open('w')

    def record(kind, item):
        trace.write(json.dumps({'kind': kind, **item}, allow_nan=False) + '\n')

    def state(message):
        item = json.loads(message.data)
        states.append(item)
        record('truth', item)

    def measurement(message, target, kind):
        item = {'stamp_s': message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                'pressure_pa': message.fluid_pressure, 'variance_pa2': message.variance,
                'frame_id': message.header.frame_id}
        target.append(item)
        record(kind, item)
    node.create_subscription(String, '/suction/ground_truth/state', state, 100)
    node.create_subscription(FluidPressure, '/suction/pressure',
                             lambda m: measurement(m, pressures, 'pressure'), 100)
    node.create_subscription(FluidPressure, '/suction/ambient_pressure',
                             lambda m: measurement(m, ambient, 'ambient'), 100)
    publishers = {(i, kind): node.create_publisher(Bool, f'/suction/fans/fan_{i}/{kind}', 10)
                  for i in range(3) for kind in ['enabled', 'failed']}
    sensor = node.create_publisher(Int32, '/suction/sensor/fault', 10)
    process = subprocess.Popen([
        'ros2', 'launch', 'climbot_gazebo', 'climbot_vacuum.launch.py',
        'headless:=true', 'pressure_mode:=' + mode,
        'io_config:=' + str((output / 'io.yaml').resolve())], env=env,
        stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)

    def wait(predicate, timeout=15):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError(f'Launch exited {process.returncode}; inspect {log}')
            rclpy.spin_once(node, timeout_sec=.02)
            if predicate():
                return
        raise TimeoutError(f'Bridge condition timed out; inspect {log}')

    def command(publisher, message, predicate, name):
        revision = states[-1]['command_revision']
        sent = states[-1]['time_s']
        publisher.publish(message)
        wait(lambda: states[-1]['command_revision'] > revision and predicate(states[-1]))
        commands.append({'name': name, 'sent_after_sim_s': sent,
                         'observed_sim_s': states[-1]['time_s'],
                         'revision': states[-1]['command_revision']})
        checks[name] = True
    completed = False
    try:
        wait(lambda: len(states) > 10 and len(pressures) > 10 and len(ambient) > 10 and
             sensor.get_subscription_count() > 0 and
             all(p.get_subscription_count() > 0 for p in publishers.values()), 30)
        checks['absolute_pressure_units'] = all(
            90000 < p['pressure_pa'] < 101325 for p in pressures)
        checks['ambient_available'] = all(p['pressure_pa'] == 101325 for p in ambient)
        checks['frame_and_variance'] = all(
            p['frame_id'] == 'chassis_link' and p['variance_pa2'] == 0 for p in pressures)
        for i in range(3):
            command(publishers[i, 'enabled'], Bool(data=False),
                    lambda s, i=i: s[f'fan_enabled_{i}'] == 0, f'fan_{i}_disable_ack')
            t = states[-1]['time_s']
            speed = states[-1][f'fan_speed_{i}']
            wait(lambda: states[-1]['time_s'] >= t + .1)
            checks[f'fan_{i}_disable_physical_speed'] = states[-1][f'fan_speed_{i}'] < speed
            command(publishers[i, 'enabled'], Bool(data=True),
                    lambda s, i=i: s[f'fan_enabled_{i}'] == 1, f'fan_{i}_enable_ack')
            command(publishers[i, 'failed'], Bool(data=True),
                    lambda s, i=i: s[f'fan_failed_{i}'] == 1, f'fan_{i}_fail_ack')
            command(publishers[i, 'failed'], Bool(data=False),
                    lambda s, i=i: s[f'fan_failed_{i}'] == 0, f'fan_{i}_recover_ack')
        command(sensor, Int32(data=1), lambda s: s['sensor_fault'] == 1, 'dropout_ack')
        t = states[-1]['time_s']
        wait(lambda: states[-1]['time_s'] >= t + .08)
        n = len(pressures)
        wait(lambda: states[-1]['time_s'] >= t + .2)
        checks['dropout_reaches_ros'] = len(pressures) == n
        command(sensor, Int32(data=0), lambda s: s['sensor_fault'] == 0, 'sensor_normal_ack')
        wait(lambda: len(pressures) > n + 5)
        command(sensor, Int32(data=2), lambda s: s['sensor_fault'] == 2, 'stuck_ack')
        t = states[-1]['time_s']
        wait(lambda: states[-1]['time_s'] >= t + .1)
        stuck = [p for p in pressures if p['stamp_s'] > t + .03]
        checks['stuck_reaches_ros_with_new_stamps'] = len(stuck) >= 3 and len(
            {p['pressure_pa'] for p in stuck}) == 1 and len(
                {p['stamp_s'] for p in stuck}) == len(stuck)
        rejected = states[-1]['rejected_commands']
        sensor.publish(Int32(data=9))
        wait(lambda: states[-1]['rejected_commands'] > rejected)
        checks['invalid_fault_rejected'] = states[-1]['sensor_fault'] == 2
        command(sensor, Int32(data=0), lambda s: s['sensor_fault'] == 0, 'final_recovery_ack')
        wait(lambda: states[-1]['time_s'] >= t + .2)
        span = max(s['cavity_pa'] for s in states) - min(s['cavity_pa'] for s in states)
        checks['truth_pressure_source_contract'] = span < 1e-8 if mode == 'fixed_pressure' else (
            span > 100)
        # Match transmitted readings against independently transmitted truth at acquisition.
        lookup = {round(s['time_s'], 8): s for s in states}
        matched = [p for p in pressures if round(p['stamp_s'], 8) in lookup and
                   lookup[round(p['stamp_s'], 8)]['sensor_fault'] == 0]
        checks['bridge_preserves_acquisition_stamp_and_value'] = len(matched) > 30 and all(
            abs(p['pressure_pa'] - lookup[round(p['stamp_s'], 8)]['cavity_pa']) < 1e-9
            for p in matched)
        checks['truth_reports_20ms_delay'] = all(
            abs(s['sensor_delivery_s'] - s['sensor_stamp_s'] - .02) < 1e-8
            for s in states if s['sensor_stamp_s'] >= 0)
        endpoints = node.get_publishers_info_by_topic('/suction/pressure')
        checks['pressure_reliable_qos'] = bool(endpoints) and all(
            p.qos_profile.reliability == rclpy.qos.ReliabilityPolicy.RELIABLE for p in endpoints)
        completed = True
    finally:
        checks['scenario_completed'] = completed
        node.destroy_node()
        rclpy.shutdown()
        trace.close()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=10)
        stream.close()
        # Every launched descendant shares this process group or is supervised by it.
        try:
            os.killpg(process.pid, 0)
            cleaned = False
        except ProcessLookupError:
            cleaned = True
        checks['process_group_cleaned'] = cleaned
        (output / 'summary.json').write_text(json.dumps({
            'mode': mode, 'checks': checks, 'all_passed': all(checks.values()),
            'commands': commands,
            'truth_samples': len(states), 'pressure_samples': len(pressures),
            'ambient_samples': len(ambient), 'launch_exit_code': process.returncode,
            'log_name': log.name}, indent=2, allow_nan=False) + '\n')
    return checks


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mode', default='fan_ode', choices=['fan_ode', 'fixed_pressure'])
    options = parser.parse_args()
    result = run(options.output, options.mode)
    print(json.dumps({'checks': len(result), 'failed': [k for k, v in result.items() if not v]}))
    raise SystemExit(0 if all(result.values()) else 1)
