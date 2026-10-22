#!/usr/bin/env python3
# Copyright 2026 jerry
# Licensed under the Apache License, Version 2.0

"""Score real EKF replays on a shared grid and report covariance consistency.

Truth remains evaluation-only. XY NEES uses the full 2x2 covariance, and phase
classification uses measured translation and rotation rather than commands.
"""

import argparse
import json
from pathlib import Path

from climbot_description.wall_frame import WallFrame
from climbot_gazebo.camera_projection import optical_axis_footprint
from climbot_gazebo.wall_surface import WallSurface
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import yaml


def ordered_unique(rows):
    rows = np.asarray(rows)
    order = np.argsort(rows[:, 0], kind='stable')
    rows = rows[order]
    # Preserve the last estimate if a filter republishes at the same stamp.
    return rows[np.r_[np.diff(rows[:, 0]) != 0, True]]


def interpolate(rows, grid):
    if grid[0] < rows[0, 0] or grid[-1] > rows[-1, 0]:
        raise ValueError('evaluation cannot extrapolate')
    return np.column_stack([np.interp(grid, rows[:, 0], rows[:, index])
                            for index in range(1, rows.shape[1])])


def error_summary(error, covariance):
    norm = np.linalg.norm(error, axis=1)
    covariance = (covariance + covariance.transpose(0, 2, 1)) / 2
    if np.min(np.linalg.eigvalsh(covariance)) <= 0:
        raise ValueError('nonpositive output covariance')
    nees = np.einsum('ni,ni->n', error, np.linalg.solve(covariance, error[..., None])[..., 0])
    worst = int(np.argmax(norm))
    return {'count': len(norm), 'bias_xy_mm': (error.mean(0) * 1000).tolist(),
            'std_xy_mm': (error.std(0) * 1000).tolist(),
            'p95_mm': float(np.percentile(norm, 95) * 1000),
            'max_mm': float(norm[worst] * 1000), 'worst_index': worst,
            'median_sigma_xy_mm': (np.median(np.sqrt(np.diagonal(
                covariance, axis1=1, axis2=2)), axis=0) * 1000).tolist(),
            'nees_xy_mean': float(nees.mean()), 'nees_xy_p95': float(np.percentile(nees, 95)),
            'fraction_inside_95pct_xy_ellipse': float(np.mean(nees <= 5.991464547)),
            'over_5mm_count': int(np.count_nonzero(norm > 0.005))}


def camera_error(rows, truth, times, mount, surface, surface_offset=0.05):
    values = interpolate(rows, times)
    body_rotations = Slerp(rows[:, 0], Rotation.from_quat(rows[:, 4:8]))(times)
    heading = body_rotations.as_euler('xyz')[:, 2]
    cosine, sine = np.cos(heading), np.sin(heading)
    mx, my, mz = mount['center_xyz_m']
    estimate = values[:, :2] + np.column_stack((cosine * mx - sine * my,
                                               sine * mx + cosine * my))
    truth_xyz = interpolate(truth[:, :4], times)
    truth_rotation = Slerp(truth[:, 0], Rotation.from_quat(truth[:, 4:8]))(times)
    cameras = truth_xyz + truth_rotation.apply(mount['center_xyz_m'])
    cameras[:, 2] -= surface_offset
    mount_axis = Rotation.from_euler('xyz', mount['rpy_rad']).as_matrix()[:, 2]
    axes = truth_rotation.apply(mount_axis)
    footprints = np.array([optical_axis_footprint(camera, axis, surface.height)[0][:2]
                           for camera, axis in zip(cameras, axes)])
    covariance = values[:, 7:43].reshape(-1, 6, 6)[:, (0, 1, 5), :][:, :, (0, 1, 5)]
    jacobian = np.zeros((len(times), 2, 3))
    jacobian[:, 0, 0] = jacobian[:, 1, 1] = 1
    jacobian[:, 0, 2] = -sine * mx - cosine * my
    jacobian[:, 1, 2] = cosine * mx - sine * my
    camera_covariance = jacobian @ covariance @ jacobian.transpose(0, 2, 1)
    report = error_summary(estimate - footprints, camera_covariance)
    report['worst_time_s'] = float(times[report['worst_index']])
    return report


def score(paths, wall_path, settle_s=2, capture_run=None):
    archives = [np.load(path / 'estimates.npz') for path in paths]
    wall = WallFrame.from_yaml(str(wall_path))
    raw_truth = ordered_unique(archives[0]['truth'])
    truth = raw_truth.copy()
    truth[:, 1:4] = [wall.position_from_world(row.tolist()) for row in raw_truth[:, 1:4]]
    truth[:, 4:8] = [wall.orientation_from_world(row.tolist()) for row in raw_truth[:, 4:8]]
    series = {'archived': ordered_unique(archives[0]['archived'])}
    for path, archive in zip(paths, archives):
        if not np.array_equal(archive['truth'], archives[0]['truth']):
            raise ValueError('replays do not share exact recorded truth')
        series[path.name] = ordered_unique(archive['estimates'])
    start = max([truth[0, 0]] + [rows[0, 0] for rows in series.values()]) + settle_s
    end = min([truth[-1, 0]] + [rows[-1, 0] for rows in series.values()])
    if end - start < 1:
        raise ValueError('insufficient common evaluation interval')
    start = float(np.ceil(start * 50) / 50)
    grid = np.arange(start, end, 0.02)
    xyz = interpolate(truth[:, :4], grid)
    rotations = Slerp(truth[:, 0], Rotation.from_quat(truth[:, 4:8]))(grid)
    truth_velocity = np.gradient(truth[:, 1:4], truth[:, 0], axis=0)
    world_velocity = np.column_stack([np.interp(grid, truth[:, 0], truth_velocity[:, i])
                                     for i in range(3)])
    body_velocity = rotations.inv().apply(world_velocity)
    speed = np.linalg.norm(world_velocity[:, :2], axis=1)
    yaw_rate = np.abs(np.gradient(np.unwrap(rotations.as_euler('xyz')[:, 2]), grid))
    phase = {'moving': speed > 0.1,
             'turning': (speed <= 0.1) & (yaw_rate > 0.05),
             'stopped': (speed < 0.01) & (yaw_rate <= 0.05),
             'transition': (speed >= 0.01) & (speed <= 0.1) & (yaw_rate <= 0.05)}
    mount = yaml.safe_load(Path('src/climbot_description/config/inspection_camera.yaml').read_text())[
        'inspection_camera']['optical_mount']
    surface = WallSurface(yaml.safe_load(Path('src/climbot_gazebo/config/m2_wall_surface.yaml').read_text()))
    exposures = None
    if capture_run:
        exposures = np.array([json.loads(path.read_text())['header']['stamp_ns'] * 1e-9
                              for path in sorted(capture_run.glob('metadata/*.json'))])
        if not len(exposures) or exposures[0] < start or exposures[-1] > end:
            raise ValueError('exposures are missing or outside common evaluation interval')
    report = {'common_time_range_s': [float(grid[0]), float(grid[-1])],
              'grid_step_s': 0.02, 'settle_s': settle_s,
              'note': 'NEES and ellipse fractions are descriptive; samples are time-correlated',
              'variants': {}}
    for name, rows in series.items():
        if np.max(np.diff(rows[:, 0])) > 0.1:
            raise ValueError(f'{name} has an evaluation gap above 100 ms')
        estimate = interpolate(rows, grid)
        error = estimate[:, :2] - xyz[:, :2]
        covariance = estimate[:, 7:43].reshape(-1, 6, 6)[:, :2, :2]
        result = error_summary(error, covariance)
        result['worst_time_s'] = float(grid[result['worst_index']])
        native = rows[(rows[:, 0] >= start) & (rows[:, 0] <= end)]
        native_truth = interpolate(truth[:, :4], native[:, 0])
        native_covariance = native[:, 8:44].reshape(-1, 6, 6)[:, :2, :2]
        result['native_time'] = error_summary(native[:, 1:3] - native_truth[:, :2],
                                              native_covariance)
        result['native_time']['worst_time_s'] = float(
            native[result['native_time']['worst_index'], 0])
        result['phases'] = {key: error_summary(error[select], covariance[select])
                            for key, select in phase.items() if select.any()}
        result['camera_footprint'] = camera_error(rows, truth, grid, mount, surface)
        if exposures is not None:
            result['camera_exposures'] = camera_error(rows, truth, exposures, mount, surface)
        if rows.shape[1] >= 50:
            result['body_velocity_bias_mps'] = (estimate[:, 43:46] - body_velocity).mean(0).tolist()
            result['body_velocity_rmse_mps'] = np.sqrt(np.mean(
                (estimate[:, 43:46] - body_velocity) ** 2, axis=0)).tolist()
            worst = result['worst_index']
            result['velocity_at_worst_mps'] = estimate[worst, 43:46].tolist()
            result['true_velocity_at_worst_mps'] = body_velocity[worst].tolist()
        report['variants'][name] = result
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--replay-dir', type=Path, action='append', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--wall', type=Path, default=Path('src/climbot_description/config/wall.yaml'))
    parser.add_argument('--capture-run', type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError('score output must be new')
    report = score(args.replay_dir, args.wall, capture_run=args.capture_run)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps({name: {key: value[key] for key in
                             ('p95_mm', 'max_mm', 'median_sigma_xy_mm', 'nees_xy_mean')}
                      for name, value in report['variants'].items()}))


if __name__ == '__main__':
    main()
