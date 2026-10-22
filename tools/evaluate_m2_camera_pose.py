#!/usr/bin/env python3
# Copyright 2026 jerry
# Licensed under the Apache License, Version 2.0
"""Evaluate immutable M2 captures against exposure-time optical-axis truth.

Uses LoadRecorder poses with position interpolation and quaternion SLERP. Truth
is evaluation-only; it is never written into capture labels or graph inputs.
Run after sourcing install/setup.bash.
"""

import argparse
import hashlib
import json
from pathlib import Path

from climbot_description.wall_frame import WallFrame
from climbot_gazebo.camera_projection import optical_axis_footprint
from climbot_gazebo.wall_surface import WallSurface
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import yaml


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def interpolated_pose(times, positions, quaternions_xyzw, target):
    """Interpolate within a recorded bracket; never silently extrapolate."""
    right = int(np.searchsorted(times, target, side='right'))
    if target < times[0] or target > times[-1]:
        raise ValueError('exposure is outside recorded truth')
    right = min(max(1, right), len(times) - 1)
    left = right - 1
    interval = times[right] - times[left]
    if interval <= 0 or interval > 0.1:
        raise ValueError('invalid or excessively wide truth bracket')
    fraction = (target - times[left]) / interval
    position = positions[left] + fraction * (positions[right] - positions[left])
    rotation = Slerp(times[left:right + 1], Rotation.from_quat(
        quaternions_xyzw[left:right + 1]))([target]).as_matrix()[0]
    return position, rotation, float(interval)


def statistics(error):
    values = np.linalg.norm(error, axis=1)
    return {'count': len(values), 'bias_xy_m': np.mean(error, axis=0).tolist(),
            'p95_m': float(np.percentile(values, 95)), 'max_m': float(values.max()),
            'p95_nearest_rank_m': float(np.sort(values)[int(np.ceil(0.95 * len(values))) - 1]),
            'over_5mm_count': int(np.count_nonzero(values > 0.005))}


def evaluate(run, output, wall_path, surface_path, camera_path, surface_offset):
    output.mkdir(parents=True, exist_ok=False)
    columns_path = run / 'loads.bin.json'
    columns = json.loads(columns_path.read_text())['columns']
    loads = np.memmap(run / 'loads.bin', dtype='<f8', mode='r').reshape(-1, len(columns))
    col = {name: index for index, name in enumerate(columns)}
    times = loads[:, col['sim_time_s']]
    if not np.all(np.diff(times) > 0):
        raise ValueError('truth stamps must be strictly increasing; split reset generations')
    positions = loads[:, [col['pose_' + axis] for axis in 'xyz']]
    quaternions = loads[:, [col['pose_q' + axis] for axis in 'xyzw']]
    wall = WallFrame.from_yaml(str(wall_path))
    surface = WallSurface(yaml.safe_load(surface_path.read_text()))
    mount = yaml.safe_load(camera_path.read_text())['inspection_camera']['optical_mount']
    offset = np.asarray(mount['center_xyz_m'])
    mount_rotation = Rotation.from_euler('xyz', mount['rpy_rad']).as_matrix()
    records, rows = [], []
    for path in sorted(run.glob('capture/*/*/metadata/*.json')):
        label = json.loads(path.read_text())
        stamp = label['header']['stamp_ns'] * 1e-9
        position, rotation, interval = interpolated_pose(times, positions, quaternions, stamp)
        base = np.asarray(wall.position_from_world(position.tolist()))
        base[2] -= surface_offset
        world_q = Rotation.from_matrix(rotation).as_quat()
        wall_q = wall.orientation_from_world(tuple(world_q))
        body_rotation = Rotation.from_quat(wall_q).as_matrix()
        camera = base + body_rotation @ offset
        axis = body_rotation @ mount_rotation[:, 2]
        foot, distance = optical_axis_footprint(camera, axis, surface.height)
        plane_foot, _ = optical_axis_footprint(camera, axis)
        estimated = label['camera_pose']['pose']['position']
        estimated = np.array([estimated['x'], estimated['y']])
        heading = float(label['wall_heading_rad'])
        yaw = np.arctan2(body_rotation[1, 0], body_rotation[0, 0])
        planar_truth = base[:2] + np.array([[np.cos(yaw), -np.sin(yaw)],
                                          [np.sin(yaw), np.cos(yaw)]]) @ offset[:2]
        covariance = np.array(label['camera_pose']['covariance']).reshape(6, 6)
        records.append({'metadata': str(path.relative_to(run)), 'sha256': sha256(path)})
        rows.append([int(path.stem), stamp, heading, *estimated, *camera,
                     *foot, distance, *plane_foot, *planar_truth,
                     interval, covariance[0, 0], covariance[1, 1]])
    if not rows:
        raise ValueError('no capture metadata')
    names = ['frame_index', 'time_s', 'heading_rad', 'estimate_x', 'estimate_y',
             'camera_x', 'camera_y', 'camera_z', 'foot_x', 'foot_y', 'foot_z',
             'distance_m', 'plane_foot_x', 'plane_foot_y', 'plane_foot_z',
             'planar_truth_x', 'planar_truth_y', 'truth_interval_s', 'cov_xx', 'cov_yy']
    data = np.asarray(rows)
    np.savez_compressed(output / 'frames.npz', names=names, data=data)
    estimate = data[:, 3:5]
    summary = {'frame_count': len(data), 'hard_limit_m': 0.005,
               'truth_definition': 'optical axis intersected with native M2 surface at exposure',
               'surface_offset_m': surface_offset,
               'optical_footprint': statistics(estimate - data[:, 8:10]),
               'camera_center': statistics(estimate - data[:, 5:7]),
               'legacy_planar': statistics(estimate - data[:, 15:17]),
               'surface_vs_plane_footprint': statistics(data[:, 8:10] - data[:, 12:14]),
               'max_truth_bracket_s': float(data[:, 17].max()),
               'mean_reported_position_sigma_m': np.sqrt(data[:, 18:20]).mean(axis=0).tolist(),
               'inputs': {str(path): sha256(path) for path in
                          (wall_path, surface_path, camera_path, columns_path, run / 'loads.bin')},
               'metadata': records}
    for name, select in [('up', np.sin(data[:, 2]) > 0.5),
                         ('down', np.sin(data[:, 2]) < -0.5),
                         ('horizontal', np.abs(np.sin(data[:, 2])) <= 0.5)]:
        if select.any():
            summary[name] = statistics((estimate - data[:, 8:10])[select])
    (output / 'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
    print(json.dumps({key: value for key, value in summary.items()
                      if key not in ('inputs', 'metadata')}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--wall', type=Path, default=Path('src/climbot_description/config/wall.yaml'))
    parser.add_argument('--surface', type=Path,
                        default=Path('src/climbot_gazebo/config/m2_wall_surface.yaml'))
    parser.add_argument('--camera', type=Path,
                        default=Path('src/climbot_description/config/inspection_camera.yaml'))
    parser.add_argument('--surface-offset-m', type=float, default=0.05)
    args = parser.parse_args()
    evaluate(args.run, args.output, args.wall, args.surface, args.camera, args.surface_offset_m)


if __name__ == '__main__':
    main()
