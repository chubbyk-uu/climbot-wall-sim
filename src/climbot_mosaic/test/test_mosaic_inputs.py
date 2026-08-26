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

"""Repeatable checks for processed-run mosaic input validation."""

import hashlib
import json

from climbot_mosaic.mosaic_inputs import (
    input_summary,
    MosaicInputError,
    validate_processed_runs,
)
import cv2
import numpy as np
import pytest
import yaml


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, document):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, indent=2, sort_keys=True) + '\n', encoding='utf-8')


def make_processed_run(tmp_path, name, run_id, frame_count=2, focal_length=14.0):
    root = tmp_path / name
    image_dir = root / 'images'
    metadata = root / 'metadata'
    calibration = root / 'calibration'
    image_dir.mkdir(parents=True)
    metadata.mkdir()
    calibration.mkdir()
    records = []
    for index in range(frame_count):
        image_path = image_dir / f'{index:06d}.png'
        image = np.full((12, 16), 40 + index, dtype=np.uint8)
        image[3:6, 6:9] = 180
        assert cv2.imwrite(str(image_path), image)
        image_digest = sha256(image_path)
        label_name = f'{index:06d}.json'
        write_json(metadata / label_name, {
            'archive_schema_version': 1,
            'camera_pose': {
                'covariance': [0.01] * 36,
                'pose': {
                    'orientation': {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0},
                    'position': {'x': float(index), 'y': 1.0, 'z': 0.275},
                },
            },
            'header': {'frame_id': 'inspection_camera_optical_frame', 'stamp_ns': index},
            'image_encoding': 'mono8',
            'image_file': f'images/raw/{index:06d}.png',
            'image_height': 12,
            'image_sha256': 'a' * 64,
            'image_width': 16,
            'processed_image_file': f'images/{index:06d}.png',
            'processed_image_sha256': image_digest,
            'processing': {
                'dark_subtraction': False,
                'denoise': 'none',
                'flat_field': True,
                'undistorted': True,
            },
            'processing_schema_version': 1,
            'revision': 1,
            'segment_index': 0,
            'source_label_file': f'metadata/{label_name}',
            'target_along_track_m': 0.3,
            'task_id': f'task-{name}',
            'trigger_index': index,
            'wall_heading_rad': 0.0,
        })
        records.append({
            'source_image_file': f'images/raw/{index:06d}.png',
            'source_image_sha256': 'a' * 64,
            'source_label_file': f'metadata/{label_name}',
            'processed_image_file': f'images/{index:06d}.png',
            'processed_image_sha256': image_digest,
        })
    camera = {
        'd': [0.0] * 5,
        'distortion_model': 'plumb_bob',
        'height': 12,
        'k': [focal_length, 0.0, 7.5, 0.0, focal_length, 5.5, 0.0, 0.0, 1.0],
        'rectified': True,
        'source_distortion_removed': True,
        'width': 16,
    }
    (calibration / 'rectified_camera_info.yaml').write_text(
        yaml.safe_dump(camera, sort_keys=True), encoding='utf-8')
    (calibration / 'camera_extrinsics.yaml').write_text(yaml.safe_dump({
        'optical_mount': {
            'center_xyz_m': [0.34, 0.0, 0.275],
            'rpy_rad': [3.141592653589793, 0.0, -1.5707963267948966],
        },
    }, sort_keys=True), encoding='utf-8')
    write_json(root / 'processing_manifest.json', {
        'execution': {'jobs': 1, 'memory_budget_gb': 1.0},
        'frames': records,
        'image_count': frame_count,
        'image_geometry': {'height_px': 12, 'width_px': 16},
        'operations': {'undistortion': {'source_distortion_model': 'plumb_bob'}},
        'processing_format_version': 1,
        'source_archive': {
            'archive_format_version': 1,
            'manifest_sha256': 'b' * 64,
            'outcome': 'completed',
            'run_id': run_id,
        },
    })
    return root


def test_multiple_compatible_runs_are_validated_in_stable_order(tmp_path):
    second = make_processed_run(tmp_path, 'second', 'b-run', frame_count=3)
    first = make_processed_run(tmp_path, 'first', 'a-run', frame_count=2)
    inputs = validate_processed_runs([second, first])
    assert [run.source_run_id for run in inputs.runs] == ['a-run', 'b-run']
    assert [frame.key.source_run_id for frame in inputs.frames] == [
        'a-run', 'a-run', 'b-run', 'b-run', 'b-run']
    summary = input_summary(inputs)
    assert summary['status'] == 'valid'
    assert summary['frame_count'] == 5
    assert summary['source_run_ids'] == ['a-run', 'b-run']


def test_processed_image_hash_tampering_is_rejected(tmp_path):
    root = make_processed_run(tmp_path, 'tampered', 'tampered-run')
    (root / 'images' / '000000.png').write_bytes(b'not a valid PNG')
    with pytest.raises(MosaicInputError, match='SHA-256'):
        validate_processed_runs([root])


def test_duplicate_source_run_ids_are_rejected(tmp_path):
    first = make_processed_run(tmp_path, 'one', 'same-run')
    second = make_processed_run(tmp_path, 'two', 'same-run')
    with pytest.raises(MosaicInputError, match='duplicate processed source run_id'):
        validate_processed_runs([first, second])


def test_nonfinite_exposure_pose_is_rejected(tmp_path):
    root = make_processed_run(tmp_path, 'nonfinite', 'nonfinite-run')
    label_path = root / 'metadata' / '000000.json'
    label = json.loads(label_path.read_text(encoding='utf-8'))
    label['camera_pose']['pose']['position']['x'] = float('nan')
    write_json(label_path, label)
    with pytest.raises(MosaicInputError, match='position must contain finite'):
        validate_processed_runs([root])


def test_joint_mosaic_rejects_mismatched_camera_model(tmp_path):
    first = make_processed_run(tmp_path, 'one', 'one-run')
    second = make_processed_run(tmp_path, 'two', 'two-run', focal_length=15.0)
    with pytest.raises(MosaicInputError, match='share one camera and mount snapshot'):
        validate_processed_runs([first, second])
