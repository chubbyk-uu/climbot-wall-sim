#!/usr/bin/env python3
# Copyright 2026 jerry
# Licensed under the Apache License, Version 2.0

"""Derive planar capture poses from real EKF replay, retaining image bytes.

The immutable archive is never edited. Derivative manifests identify the parent,
replay output, and configuration hashes. This tool never reads simulator truth.
The pose/covariance transform follows automatic_capture_node's planar contract.
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
import numpy as np
from scipy.spatial.transform import Rotation


def camera_pose_at(rows, stamp, center, mount_rpy):
    """Interpolate yaw across its wrap and propagate the full camera covariance."""
    if stamp < rows[0, 0] or stamp > rows[-1, 0]:
        raise ValueError('exposure outside replay range')
    right = int(np.clip(np.searchsorted(rows[:, 0], stamp, side='right'), 1, len(rows) - 1))
    a, b = rows[right - 1:right + 1]
    dt = b[0] - a[0]
    if not 0 < dt <= 0.1:
        raise ValueError('exposure interpolation bracket exceeds 100 ms')
    fraction = (stamp - a[0]) / dt
    value = a + fraction * (b - a)
    ya, yb = Rotation.from_quat([a[4:8], b[4:8]]).as_euler('xyz')[:, 2]
    yaw = ya + fraction * np.arctan2(np.sin(yb - ya), np.cos(yb - ya))
    cosine, sine = np.cos(yaw), np.sin(yaw)
    mx, my, mz = center
    position = [value[1] + cosine * mx - sine * my,
                value[2] + sine * mx + cosine * my, mz]
    quaternion = (Rotation.from_euler('z', yaw) * Rotation.from_euler('xyz', mount_rpy)).as_quat()
    jacobian = np.eye(6)
    jacobian[2, 2] = 0
    jacobian[0, 5] = -sine * mx - cosine * my
    jacobian[1, 5] = cosine * mx - sine * my
    covariance = jacobian @ value[8:44].reshape(6, 6) @ jacobian.T
    pose = {'position': dict(zip('xyz', map(float, position))),
            'orientation': dict(zip('xyzw', map(float, quaternion)))}
    return {'pose': pose, 'covariance': covariance.ravel().tolist()}, float(yaw)


def derive(input_run, replay_dir, output):
    from climbot_mosaic.mosaic_inputs import validate_processed_runs
    from climbot_mosaic.stage_provenance import artifact, write_stage_provenance
    inputs = validate_processed_runs([input_run])
    report = json.loads((input_run / 'processing_manifest.json').read_text())
    if report['processing_format_version'] != 1:
        raise ValueError('start from an original planar processed run')
    if not output.is_absolute() or output.exists() or not output.parent.is_dir():
        raise ValueError('output must be a new absolute directory with existing parent')
    with np.load(replay_dir / 'estimates.npz') as archive:
        rows = archive['estimates']  # Never access archived estimates or truth.
    rows = rows[np.argsort(rows[:, 0], kind='stable')]
    rows = rows[np.r_[np.diff(rows[:, 0]) > 0, True]]
    temporary = Path(tempfile.mkdtemp(prefix=f'.{output.name}.tmp-', dir=output.parent))
    for directory in ('images', 'metadata'):
        (temporary / directory).mkdir()
    shutil.copytree(input_run / 'calibration', temporary / 'calibration')
    report['parent_processing_execution'] = report.pop('execution', {})
    report['execution'] = {'operation': 'derive_replayed_ekf_planar',
                           'image_bytes': 'hardlinked_unchanged'}
    report['localization_derivation'] = {
        'model': 'replayed_ekf_planar',
        'pose_binding_version': 1,
        'parent_processing_manifest_sha256': sha256_file(input_run / 'processing_manifest.json'),
        'replay_estimates_sha256': sha256_file(replay_dir / 'estimates.npz'),
        'replay_summary_sha256': sha256_file(replay_dir / 'summary.json'),
        'replay_input_sha256': json.loads((replay_dir / 'summary.json').read_text())['input_sha256'],
    }
    records = {record['source_label_file']: record for record in report['frames']}
    for frame in inputs.frames:
        label = deepcopy(frame.label)
        label['archived_camera_pose'] = label['camera_pose']
        label['archived_wall_heading_rad'] = label['wall_heading_rad']
        label['camera_pose'], label['wall_heading_rad'] = camera_pose_at(
            rows, label['header']['stamp_ns'] * 1e-9,
            inputs.camera.mount_center_xyz_m, inputs.camera.mount_rpy_rad)
        records[label['source_label_file']]['localization_pose_sha256'] = hashlib.sha256(
            json.dumps({'camera_pose': label['camera_pose'],
                        'wall_heading_rad': label['wall_heading_rad']},
                       sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
        # Recompute the metadata coordinate tied to the replaced camera estimate.
        if 'reference_start' in label and 'reference_end' in label:
            a, b = label['reference_start'], label['reference_end']
            delta = np.array([b['x'] - a['x'], b['y'] - a['y']])
            p = label['camera_pose']['pose']['position']
            label['actual_along_track_m'] = float(np.dot(
                [p['x'] - a['x'], p['y'] - a['y']], delta) / np.linalg.norm(delta))
        write_json(temporary / label['source_label_file'], label)
        os.link(frame.image_path, temporary / label['processed_image_file'])
    write_json(temporary / 'processing_manifest.json', report)
    validate_processed_runs([temporary])
    write_stage_provenance(
        temporary, 'localization_derivation', {'model': 'replayed_ekf_planar'},
        {'parent_processing_manifest': artifact(input_run / 'processing_manifest.json'),
         'replay': artifact(replay_dir / 'estimates.npz'),
         'replay_summary': artifact(replay_dir / 'summary.json')}, ('processing_manifest.json',))
    temporary.replace(output)
    print(json.dumps({'status': 'derived', 'frames': len(inputs.frames), 'output': str(output)}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-run', type=Path, required=True)
    parser.add_argument('--replay-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    derive(args.input_run, args.replay_dir, args.output_dir)


if __name__ == '__main__':
    main()
