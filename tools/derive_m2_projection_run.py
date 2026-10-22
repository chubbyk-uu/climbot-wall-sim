#!/usr/bin/env python3
# Copyright 2026 jerry
# Licensed under the Apache License, Version 2.0

"""Derive a new processed run with recorded EKF SE(3) projection geometry.

Original capture poses, covariances and image bytes remain unchanged. This
explicit version-2 derivative binds the additional projection poses to sorted
filter samples and bag hashes; no simulator truth is read by this producer.
"""

import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile

from climbot_common.atomic import write_json
from climbot_common.hashing import sha256_file
from climbot_mosaic.mosaic_inputs import validate_processed_runs
from climbot_mosaic.stage_provenance import artifact, write_stage_provenance
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation, Slerp


def read_filter_samples(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag)),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    rows = []
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic != '/odometry/filtered':
            continue
        message = deserialize_message(raw, get_message(types[topic]))
        if message.header.frame_id != 'odom' or message.child_frame_id != 'base_link':
            raise ValueError('filter poses must be odom -> base_link in the shared wall frame')
        pose = message.pose.pose
        rows.append([message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                     pose.position.x, pose.position.y, pose.position.z,
                     pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    if len(rows) < 2:
        raise ValueError('bag contains too few filtered poses')
    rows = np.asarray(rows)
    backwards = int(np.count_nonzero(np.diff(rows[:, 0]) < 0))
    rows = rows[np.argsort(rows[:, 0], kind='stable')]
    selected = np.r_[np.diff(rows[:, 0]) != 0, True]
    duplicates = int(np.count_nonzero(~selected))
    return rows[selected], backwards, duplicates


def derive(input_run, bag, output, plane_offset):
    inputs = validate_processed_runs([input_run])
    parent = json.loads((input_run / 'processing_manifest.json').read_text())
    if parent['processing_format_version'] != 1:
        raise ValueError('derive from an original version-1 processed run')
    if not output.is_absolute() or output.exists() or not output.parent.is_dir():
        raise ValueError('output must be a new absolute directory with an existing parent')
    if not np.isfinite(plane_offset):
        raise ValueError('physical wall plane offset must be finite')
    rows, backwards, duplicates = read_filter_samples(bag)
    times = np.array([frame.label['header']['stamp_ns'] * 1e-9 for frame in inputs.frames])
    if times.min() < rows[0, 0] or times.max() > rows[-1, 0]:
        raise ValueError('capture timestamps fall outside recorded filter samples')
    right = np.clip(np.searchsorted(rows[:, 0], times, side='right'), 1, len(rows) - 1)
    if np.max(rows[right, 0] - rows[right - 1, 0]) > 0.1:
        raise ValueError('capture interpolation bracket exceeds 100 ms')
    rotation = Slerp(rows[:, 0], Rotation.from_quat(rows[:, 4:8]))(times)
    position = np.column_stack([np.interp(times, rows[:, 0], rows[:, axis])
                                for axis in (1, 2, 3)])
    position += rotation.apply(inputs.camera.mount_center_xyz_m)
    position[:, 2] -= plane_offset
    optical_rotation = rotation * Rotation.from_euler('xyz', inputs.camera.mount_rpy_rad)
    quaternion = optical_rotation.as_quat()
    temporary = Path(tempfile.mkdtemp(prefix=f'.{output.name}.tmp-', dir=output.parent))
    (temporary / 'images').mkdir()
    (temporary / 'metadata').mkdir()
    (temporary / 'projection').mkdir()
    shutil.copytree(input_run / 'calibration', temporary / 'calibration')
    samples_path = temporary / 'projection/filtered_poses.npz'
    np.savez_compressed(samples_path, rows=rows)
    report = deepcopy(parent)
    report['processing_format_version'] = 2
    report['parent_processing_execution'] = report.pop('execution', {})
    report['execution'] = {'operation': 'derive_recorded_ekf_se3', 'image_bytes': 'hardlinked_unchanged'}
    report['projection_geometry'] = {
        'model': 'recorded_ekf_se3', 'wall_plane_offset_m': plane_offset,
        'parent_processing_manifest_sha256': sha256_file(input_run / 'processing_manifest.json'),
        'pose_samples_file': 'projection/filtered_poses.npz',
        'pose_samples_sha256': sha256_file(samples_path),
        'backwards_header_count_before_sort': backwards, 'duplicate_header_count': duplicates,
        'source_topic': '/odometry/filtered',
        'source_bag_sha256': {path.name: sha256_file(path) for path in
                              [bag / 'metadata.yaml', *sorted(bag.glob('*.mcap')),
                               *sorted(bag.glob('*.db3'))]},
    }
    records = {record['source_label_file']: record for record in report['frames']}
    for index, frame in enumerate(inputs.frames):
        label = deepcopy(frame.label)
        label['processing_schema_version'] = 2
        pose = {'position': dict(zip('xyz', map(float, position[index]))),
                'orientation': dict(zip('xyzw', map(float, quaternion[index])))}
        label['projection_camera_pose'] = pose
        record = records[label['source_label_file']]
        record['projection_pose_sha256'] = hashlib.sha256(json.dumps(
            pose, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
        write_json(temporary / label['source_label_file'], label)
        os.link(frame.image_path, temporary / label['processed_image_file'])
    write_json(temporary / 'processing_manifest.json', report)
    validate_processed_runs([temporary])
    write_stage_provenance(
        temporary, 'projection_geometry', {'wall_plane_offset_m': plane_offset},
        {'parent_processing_manifest': artifact(input_run / 'processing_manifest.json'),
         'filtered_poses': artifact(samples_path)}, ('processing_manifest.json',))
    temporary.replace(output)
    print(json.dumps({'status': 'derived', 'frames': len(inputs.frames),
                      'backwards_header_count': backwards, 'output': str(output)}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-run', required=True, type=Path)
    parser.add_argument('--localization-bag', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--wall-plane-offset-m', type=float, default=0.05)
    args = parser.parse_args()
    derive(args.input_run, args.localization_bag, args.output_dir, args.wall_plane_offset_m)


if __name__ == '__main__':
    main()
