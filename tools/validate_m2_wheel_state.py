#!/usr/bin/env python3
"""One short ROS motion/reset check for real tyre joint states and dynamic TF."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
import yaml


def stamp_ns(stamp):
    """Use exact source timestamps when matching joint states and TF."""
    return stamp.sec * 10**9 + stamp.nanosec


def run(output):
    """Exercise the real launch, bridge and state publisher without a GUI."""
    output.mkdir(parents=True, exist_ok=False)
    share = Path(get_package_share_directory('climbot_description'))
    wheel = yaml.safe_load((share / 'config/vacuum_chassis.yaml').read_text())[
        'vacuum_chassis']['wheel']
    env = dict(os.environ, GZ_PARTITION='climbot_t1_tf_' + uuid.uuid4().hex)
    log = Path('/tmp') / ('climbot_m2_t1_state_' + uuid.uuid4().hex + '.log')
    rclpy.init()
    node = rclpy.create_node('m2_tyre_state_check')
    joints, transforms, truth = [], [], []
    joint_sub = node.create_subscription(JointState, '/joint_states', joints.append, 100)
    tf_sub = node.create_subscription(TFMessage, '/tf', transforms.append, 100)
    truth_sub = node.create_subscription(String, '/suction/ground_truth/state',
                                         lambda m: truth.append(json.loads(m.data)), 100)
    command = node.create_publisher(Twist, '/control/cmd_vel', 10)
    checks, epochs = {}, []
    with log.open('w') as stream:
        process = subprocess.Popen([
            'ros2', 'launch', 'climbot_gazebo', 'climbot_vacuum.launch.py',
            'headless:=true', 'wheel_compliance:=true', 'heading_deg:=90'], env=env,
            stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)

        def wait(predicate, timeout=25):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError(f'launch exited {process.returncode}; inspect {log}')
                rclpy.spin_once(node, timeout_sec=.02)
                if predicate():
                    return
            raise TimeoutError(f'state check deadline; inspect {log}')

        def match_tf(label):
            positions = {}
            for message in joints:
                for name, position in zip(message.name, message.position):
                    positions[stamp_ns(message.header.stamp), name] = position
            differences, matched, compressions = [], {}, []
            for message in transforms:
                for tf in message.transforms:
                    for side in ['left', 'right']:
                        if tf.child_frame_id != side + '_wheel_axle_link':
                            continue
                        q = positions.get((stamp_ns(tf.header.stamp), side + '_tyre_joint'))
                        if q is None:
                            continue
                        p = tf.transform.translation
                        lateral = wheel['separation_m'] / 2 * (1 if side == 'left' else -1)
                        differences.append(max(abs(p.x), abs(p.y - lateral),
                                               abs(p.z - (wheel['radius_m'] + q))))
                        compressions.append(q)
                        matched[side] = matched.get(side, 0) + 1
                        checks[label + '/' + side + '_parent'] = tf.header.frame_id == 'base_link'
            checks[label + '/both_joints_match_tf'] = all(matched.get(s, 0) >= 10
                                                          for s in ['left', 'right'])
            checks[label + '/tf_matches_compression'] = (
                bool(differences) and max(differences) < 1e-7)
            # Preloaded springs sit near zero at the working load, either side.
            checks[label + '/loaded_compression_nonzero'] = bool(compressions) and max(
                map(abs, compressions)) > .0001
            epochs.append({'name': label, 'matched_tf': matched,
                           'maximum_tf_error_m': max(differences) if differences else None,
                           'compression_range_m': [min(compressions), max(compressions)]
                           if compressions else None})

        try:
            required = {'left_tyre_joint', 'right_tyre_joint',
                        'left_wheel_joint', 'right_wheel_joint'}
            wait(lambda: truth and truth[-1]['fixture_active'] == 0 and joints and
                 required.issubset(joints[-1].name) and command.get_subscription_count() > 0)
            start = truth[-1]['time_s']
            while truth[-1]['time_s'] < start + 5.5:
                elapsed = truth[-1]['time_s'] - start
                message = Twist()
                message.linear.x = .2 if elapsed < 2 else 0.
                message.angular.z = .35 if 2.5 <= elapsed < 4.5 else 0.
                command.publish(message)
                wait(lambda: truth[-1]['time_s'] > start + elapsed, timeout=5)
            command.publish(Twist())
            match_tf('motion')
            checks['actual_translation'] = max(s['linear_speed_mps'] for s in truth) > .05
            checks['actual_turn'] = max(abs(s['force_input_wx_rps']) for s in truth) > .1
            checks['no_runtime_failure'] = all(s['runtime_failed'] == 0 for s in truth)
            checks['no_stop_contact'] = all(s['skirt_stop_previous_normal_n'] == 0 for s in truth)
            generation = truth[-1]['generation']
            reset_log = Path('/tmp') / ('climbot_m2_t1_reset_' + uuid.uuid4().hex + '.log')
            with reset_log.open('w') as reset_stream:
                result = subprocess.run([
                    'gz', 'service', '-s', '/world/climbot_wall/control', '--reqtype',
                    'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean', '--timeout', '3000',
                    '--req', 'reset: {all: true}'], env=env, stdout=reset_stream,
                    stderr=subprocess.STDOUT, timeout=6)
            checks['reset_request_exit_zero'] = result.returncode == 0
            wait(lambda: truth[-1]['generation'] > generation)
            joints.clear()
            transforms.clear()
            wait(lambda: truth[-1]['fixture_active'] == 0)
            ready = truth[-1]['time_s']
            wait(lambda: truth[-1]['time_s'] >= ready + 1)
            match_tf('after_reset')
            checks['reset_recovered'] = truth[-1]['runtime_failed'] == 0
        finally:
            node.destroy_subscription(joint_sub)
            node.destroy_subscription(tf_sub)
            node.destroy_subscription(truth_sub)
            node.destroy_node()
            rclpy.shutdown()
            os.killpg(process.pid, signal.SIGINT)
            forced = False
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                forced = True
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
            checks['graceful_shutdown'] = not forced
            try:
                os.killpg(process.pid, 0)
                checks['process_group_cleaned'] = False
            except ProcessLookupError:
                checks['process_group_cleaned'] = True
            result = {'scope': 'Headless ROS motion/reset and dynamic TF; no GUI or T2 acceptance',
                      'checks': checks, 'epochs': epochs,
                      'truth_samples': len(truth), 'launch_exit_code': process.returncode,
                      'log_path': str(log), 'passed': all(checks.values())}
            (output / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = run(args.output)
    print(json.dumps(result))
    raise SystemExit(0 if result['passed'] else 1)
