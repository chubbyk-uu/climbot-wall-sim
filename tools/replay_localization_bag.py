#!/usr/bin/env python3
# Copyright 2026 jerry
# Licensed under the Apache License, Version 2.0

"""Replay recorded localization inputs through the installed, real EKF node.

Requires an IMU recording; never synthesizes missing measurements. Bag timestamps
are preserved. The output topic is isolated from the recorded filter output.
Use an otherwise idle ROS domain and source install/setup.bash before running.
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from climbot_common.hashing import sha256_file
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosgraph_msgs.msg import Clock


INPUT_TOPICS = ('/wheel_odom', '/total_station/pose', '/imu_wall')


def odometry_row(message):
    pose = message.pose.pose
    twist = message.twist.twist
    return [message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
            *message.pose.covariance,
            twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z,
            *message.twist.covariance]


def read_reference(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag)),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if any(topic not in types for topic in INPUT_TOPICS):
        raise ValueError('all wheel, total-station and actual IMU inputs must be recorded')
    counts = dict.fromkeys(INPUT_TOPICS, 0)
    reference = {'/odometry/filtered': [], '/model/climbot/ground_truth': []}
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic in counts:
            counts[topic] += 1
        if topic in reference:
            reference[topic].append(odometry_row(deserialize_message(raw, get_message(types[topic]))))
    if min(counts.values()) == 0 or any(not values for values in reference.values()):
        raise ValueError('recording is missing input samples, archived estimate, or truth')
    return counts, reference


def stop(process):
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--domain', required=True, type=int)
    parser.add_argument('--params', type=Path, default=Path('src/climbot_gazebo/config/ekf_wall.yaml'))
    parser.add_argument('--override-params', type=Path)
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--timeout-s', type=float, default=300)
    args = parser.parse_args()
    if (not 0 <= args.domain <= 232 or not math.isfinite(args.rate) or args.rate <= 0 or
            not math.isfinite(args.timeout_s) or args.timeout_s <= 0):
        raise ValueError('invalid domain or playback rate')
    counts, reference = read_reference(args.bag)
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1')
    rclpy.init()
    node = rclpy.create_node('localization_review_collector')
    clock_publisher = node.create_publisher(Clock, '/clock', 10)
    bootstrap_clock = Clock()
    start_s = max(1.0, min(reference['/odometry/filtered'][0][0],
                           reference['/model/climbot/ground_truth'][0][0]) - 1.0)
    bootstrap_clock.clock.sec = int(start_s)
    bootstrap_clock.clock.nanosec = int((start_s % 1) * 1e9)
    estimates = []
    node.create_subscription(Odometry, '/review/filtered',
                             lambda message: estimates.append(odometry_row(message)), 2000)
    executable = Path(get_package_prefix('robot_localization')) / 'lib/robot_localization/ekf_node'
    command = [str(executable), '--ros-args', '-r', '__node:=ekf_filter_node',
               '--params-file', str(args.params), '-p', 'use_sim_time:=true',
               '-p', 'publish_tf:=false', '-r', 'odometry/filtered:=/review/filtered']
    if args.override_params:
        command.extend(['--params-file', str(args.override_params)])
    log_prefix = Path('/tmp') / f'climbot_replay_{args.output.name}_{os.getpid()}'
    ekf = player = None
    try:
        with log_prefix.with_suffix('.ekf.log').open('w') as ekf_log, \
                log_prefix.with_suffix('.player.log').open('w') as player_log:
            ekf = subprocess.Popen(command, stdout=ekf_log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 20
            while (not node.count_publishers('/review/filtered') or
                   any(not node.count_subscribers(topic) for topic in INPUT_TOPICS)):
                # robot_localization creates its sensor endpoints only after
                # its first nonzero clock. Discovering sensors before starting
                # a clock would otherwise deadlock this readiness handshake.
                clock_publisher.publish(bootstrap_clock)
                if time.monotonic() > deadline or ekf.poll() is not None:
                    raise RuntimeError('EKF input/output discovery failed')
                rclpy.spin_once(node, timeout_sec=0.05)
            player = subprocess.Popen([
                'ros2', 'bag', 'play', str(args.bag), '--topics', *INPUT_TOPICS,
                '--clock', '200', '--rate', str(args.rate), '--delay', '1'],
                stdout=player_log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + args.timeout_s
            while player.poll() is None:
                if time.monotonic() > deadline or ekf.poll() is not None:
                    raise RuntimeError('playback timed out or EKF exited')
                rclpy.spin_once(node, timeout_sec=0.02)
            if player.returncode:
                raise RuntimeError(f'bag playback exited {player.returncode}')
            deadline = time.monotonic() + 1
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        stop(player)
        stop(ekf)
        node.destroy_node()
        rclpy.shutdown()
    if len(estimates) < 2:
        raise RuntimeError('no usable replay output')
    np.savez_compressed(args.output / 'estimates.npz', estimates=np.asarray(estimates),
                        archived=np.asarray(reference['/odometry/filtered']),
                        truth=np.asarray(reference['/model/climbot/ground_truth']))
    summary = {'bag': str(args.bag.resolve()), 'recorded_input_counts': counts,
               'output_count': len(estimates), 'imu': 'recorded', 'rate': args.rate,
               'params': str(args.params), 'override_params': str(args.override_params),
               'logs': str(log_prefix), 'ekf_exit_code': ekf.returncode,
               'player_exit_code': player.returncode}
    sources = [args.params, args.bag / 'metadata.yaml']
    sources.extend(sorted(args.bag.glob('*.mcap')) + sorted(args.bag.glob('*.db3')))
    if args.override_params:
        sources.append(args.override_params)
    summary['input_sha256'] = {str(path.resolve()): sha256_file(path) for path in sources}
    summary['output_sha256'] = sha256_file(args.output / 'estimates.npz')
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
