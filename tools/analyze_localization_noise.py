#!/usr/bin/env python3
# Copyright 2026 jerry
# Licensed under the Apache License, Version 2.0

"""Audit recorded sensor covariance and multihorizon motion-model residuals.

Simulator truth is used exclusively for offline calibration. No derived truth
samples are published, replayed as measurements, or supplied to mosaic tools.
Process residuals are diagnostics, not a claim that Q is uniquely identifiable.
"""

import argparse
import json
from pathlib import Path

from climbot_common.hashing import sha256_file
from climbot_description.wall_frame import WallFrame
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation, Slerp


def ordered(rows):
    values = np.asarray(rows, dtype=float)
    values = values[np.argsort(values[:, 0], kind='stable')]
    return values[np.r_[np.diff(values[:, 0]) > 0, True]]


def residual_summary(error, variance):
    """Keep bias, centered variance and mean-square error distinct."""
    error, variance = np.asarray(error), np.asarray(variance)
    if len(error) < 3 or not np.all(np.isfinite(error)) or np.any(variance <= 0):
        raise ValueError('residuals need finite samples and positive reported variances')
    centered = error - error.mean(axis=0)
    lag = np.sum(centered[:-1] * centered[1:], axis=0)
    lag /= np.maximum(np.sqrt(np.sum(centered[:-1] ** 2, axis=0) *
                              np.sum(centered[1:] ** 2, axis=0)), 1e-30)
    return {'count': len(error), 'bias': error.mean(0).tolist(),
            'std': error.std(0).tolist(), 'rmse': np.sqrt(np.mean(error ** 2, 0)).tolist(),
            'covariance': np.atleast_2d(np.cov(error, rowvar=False)).tolist(),
            'reported_variance_median': np.median(variance, axis=0).tolist(),
            'mean_square_over_reported_variance': np.mean(error ** 2 / variance, 0).tolist(),
            'lag_one_correlation': lag.tolist(),
            'absolute_p95': np.percentile(abs(error), 95, axis=0).tolist()}


def interpolate(rows, times):
    if times.min() < rows[0, 0] or times.max() > rows[-1, 0]:
        raise ValueError('no truth extrapolation permitted')
    return np.column_stack([np.interp(times, rows[:, 0], rows[:, i])
                            for i in range(1, rows.shape[1])])


def read_bag(bag, wall):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag)),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    data = {name: [] for name in ('truth', 'station', 'wheel', 'imu')}
    for_read = {'/model/climbot/ground_truth', '/total_station/pose', '/wheel_odom', '/imu_wall'}
    while reader.has_next():
        topic, raw, record_ns = reader.read_next()
        if topic not in for_read:
            continue
        message = deserialize_message(raw, get_message(types[topic]))
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if topic == '/model/climbot/ground_truth':
            p, q = message.pose.pose.position, message.pose.pose.orientation
            data['truth'].append([stamp, *wall.position_from_world([p.x, p.y, p.z]),
                                  *wall.orientation_from_world([q.x, q.y, q.z, q.w])])
        elif topic == '/total_station/pose':
            p = message.pose.pose.position
            data['station'].append([stamp, p.x, p.y, p.z,
                                    *np.asarray(message.pose.covariance)[[0, 7, 14]],
                                    record_ns * 1e-9 - stamp])
        elif topic == '/wheel_odom':
            t = message.twist.twist
            data['wheel'].append([stamp, t.linear.x, t.angular.z,
                                  message.twist.covariance[0], message.twist.covariance[35]])
        else:
            q, w = message.orientation, message.angular_velocity
            data['imu'].append([stamp, q.x, q.y, q.z, q.w, w.x, w.y, w.z,
                                *np.asarray(message.orientation_covariance)[[0, 4, 8]],
                                *np.asarray(message.angular_velocity_covariance)[[0, 4, 8]]])
    return {name: ordered(rows) for name, rows in data.items()}


def truth_kinematics(truth):
    times = truth[:, 0]
    rotation = Rotation.from_quat(truth[:, 4:8])
    velocity = rotation.inv().apply(np.gradient(truth[:, 1:4], times, axis=0))
    interval_rate = (rotation[:-1].inv() * rotation[1:]).as_rotvec() / np.diff(times)[:, None]
    mid = (times[:-1] + times[1:]) / 2
    omega = np.column_stack([np.interp(times, mid, interval_rate[:, i]) for i in range(3)])
    return velocity, omega


def motion_residuals(truth, horizon):
    """Compare omnidirectional EKF predictions against sampled true states."""
    step = 0.02
    times = np.arange(truth[0, 0] + 0.2, truth[-1, 0] - horizon - 0.2, step)
    rotation = Slerp(truth[:, 0], Rotation.from_quat(truth[:, 4:8]))(times)
    future_rotation = Slerp(truth[:, 0], Rotation.from_quat(truth[:, 4:8]))(times + horizon)
    velocity, omega = truth_kinematics(truth)
    acceleration = np.gradient(velocity, truth[:, 0], axis=0)
    def sample(values, target):
        return interpolate(np.column_stack((truth[:, 0], values)), target)
    v, a, w = sample(velocity, times), sample(acceleration, times), sample(omega, times)
    position = sample(truth[:, 1:4], times)
    predicted_position = position + rotation.apply(v * horizon + 0.5 * a * horizon ** 2)
    roll, pitch, yaw = rotation.as_euler('xyz').T
    sr, cr, tp, cp = np.sin(roll), np.cos(roll), np.tan(pitch), np.cos(pitch)
    euler_rate = np.column_stack((w[:, 0] + sr * tp * w[:, 1] + cr * tp * w[:, 2],
                                  cr * w[:, 1] - sr * w[:, 2],
                                  sr / cp * w[:, 1] + cr / cp * w[:, 2]))
    predicted_angles = np.column_stack((roll, pitch, yaw)) + euler_rate * horizon
    angle_error = future_rotation.as_euler('xyz') - predicted_angles
    angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
    error = np.column_stack((sample(truth[:, 1:4], times + horizon) - predicted_position,
                             angle_error, sample(velocity, times + horizon) - v - a * horizon,
                             sample(omega, times + horizon) - w,
                             sample(acceleration, times + horizon) - a))
    phases = {'all': np.ones(len(times), bool),
              'moving': np.linalg.norm(v[:, :2], axis=1) > 0.1,
              'turning': (np.linalg.norm(v[:, :2], axis=1) <= 0.1) & (abs(w[:, 2]) > 0.05),
              'stopped': (np.linalg.norm(v[:, :2], axis=1) < 0.01) & (abs(w[:, 2]) <= 0.05)}
    return {name: {'count': int(mask.sum()),
                    'bias': error[mask].mean(0).tolist(),
                    'effective_second_moment_per_second': (
                        np.mean(error[mask] ** 2, axis=0) / horizon).tolist()}
            for name, mask in phases.items() if mask.sum() >= 10}


def analyze(data):
    truth = data['truth']
    # Drop derivative boundaries, never shift timestamps to minimize residuals.
    start, end = truth[0, 0] + 0.2, truth[-1, 0] - 0.2
    rotations = Slerp(truth[:, 0], Rotation.from_quat(truth[:, 4:8]))
    velocity, omega = truth_kinematics(truth)
    result = {}
    for sensor in ('station', 'imu', 'wheel'):
        rows = data[sensor]
        rows = rows[(rows[:, 0] >= start) & (rows[:, 0] <= end)]
        times = rows[:, 0]
        if sensor == 'station':
            error = rows[:, 1:4] - interpolate(truth[:, :4], times)
            result[sensor] = residual_summary(error, rows[:, 4:7])
            result[sensor]['delivery_delay_p50_p95_max_s'] = np.percentile(
                rows[:, 7], [50, 95, 100]).tolist()
        elif sensor == 'imu':
            error = (rotations(times).inv() * Rotation.from_quat(rows[:, 1:5])).as_rotvec()
            result['imu_attitude'] = residual_summary(error, rows[:, 8:11])
            truth_omega = interpolate(np.column_stack((truth[:, 0], omega)), times)
            result['imu_gyro'] = residual_summary(rows[:, 5:8] - truth_omega, rows[:, 11:14])
            result['imu_gyro']['reference_limitation'] = (
                'Reference is an interval-derived angular rate from 100 Hz poses, not an '
                'instantaneous physical angular-rate sensor. High-frequency contact motion '
                'can alias into this difference. Do not use it to inflate gyro R.')
        else:
            kinematics = interpolate(np.column_stack((truth[:, 0], velocity, omega)), times)
            error = rows[:, 1:3] - kinematics[:, [0, 5]]
            result[sensor] = residual_summary(error, rows[:, 3:5])
            moving = np.linalg.norm(kinematics[:, :2], axis=1) > 0.1
            for name, mask in [('moving', moving), ('slow', ~moving)]:
                if mask.sum() >= 10:
                    result[sensor][name] = residual_summary(error[mask], rows[mask, 3:5])
    result['motion_model'] = {str(dt): motion_residuals(truth, dt) for dt in (0.02, 0.08, 0.2)}
    result['limitations'] = ('Motion-model second moments depend on horizon and numerical truth '
                            'derivatives; they are initialization evidence, not a unique Q fit. '
                            'Correlated slip and bias must not be interpreted as white sensor noise.')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--wall', type=Path, default=Path('src/climbot_description/config/wall.yaml'))
    args = parser.parse_args()
    args.output_dir.mkdir(exist_ok=False)
    data = read_bag(args.bag, WallFrame.from_yaml(str(args.wall)))
    np.savez_compressed(args.output_dir / 'sensor_samples.npz', **data)
    report = analyze(data)
    report['sources_sha256'] = {str(p.resolve()): sha256_file(p) for p in
                                [args.wall, args.bag / 'metadata.yaml',
                                 *sorted(args.bag.glob('*.mcap')), *sorted(args.bag.glob('*.db3'))]}
    (args.output_dir / 'noise_audit.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'status': 'analyzed', 'output': str(args.output_dir)}))


if __name__ == '__main__':
    main()
