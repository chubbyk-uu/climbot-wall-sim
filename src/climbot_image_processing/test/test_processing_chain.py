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

"""Pure, repeatable checks for the first offline preprocessing chain."""

import hashlib
import json
from pathlib import Path

import climbot_image_processing.processing as processing
from climbot_image_processing.processing import (
    process_archive,
    ProcessingError,
    ProcessingOptions,
)
import cv2
import numpy as np
import pytest
import yaml


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + '\n', encoding='utf-8')


def make_archive(tmp_path, outcome='completed', frame_count=2):
    root = tmp_path / 'source_run'
    image_dir = root / 'images' / 'raw'
    image_dir.mkdir(parents=True)
    raw = np.tile(np.arange(16, dtype=np.uint8), (12, 1)) * 8 + 30
    raw[6, 8] = 250
    for index in range(frame_count):
        image = np.clip(raw.astype(np.int16) + index, 0, 255).astype(np.uint8)
        image_path = image_dir / f'{index:06d}.png'
        assert cv2.imwrite(str(image_path), image)
        write_json(root / 'metadata' / f'{index:06d}.json', {
            'archive_schema_version': 1,
            'image_file': f'images/raw/{index:06d}.png',
            'image_sha256': sha256(image_path),
            'image_width': 16,
            'image_height': 12,
            'image_encoding': 'mono8',
            'segment_index': 0,
            'trigger_index': index,
        })
    write_json(root / 'manifest.json', {
        'archive_format_version': 1,
        'run_id': 'test-run',
        'outcome': outcome,
        'expected_images': frame_count,
        'saved_images': frame_count,
        'failed_images': 0,
        'canonical_image': {
            'encoding': 'mono8', 'format': 'png', 'distorted': True,
            'illumination_compensated': False,
        },
    })
    calibration = root / 'calibration'
    calibration.mkdir()
    camera = {
        'width': 16, 'height': 12, 'distortion_model': 'plumb_bob',
        'k': [14.0, 0.0, 7.5, 0.0, 14.0, 5.5, 0.0, 0.0, 1.0],
        'd': [-0.12, 0.025, 0.0005, -0.0003, -0.004],
    }
    (calibration / 'camera_info.yaml').write_text(
        yaml.safe_dump(camera, sort_keys=True), encoding='utf-8')
    (calibration / 'camera_extrinsics.yaml').write_text(
        yaml.safe_dump({'optical_mount': {'center_xyz_m': [0.34, 0.0, 0.275]}}, sort_keys=True),
        encoding='utf-8')
    flat = tmp_path / 'flat.npz'
    np.savez_compressed(flat, gain=np.full((12, 16), 1.5, dtype=np.float32))
    write_json(calibration / 'flat_field_reference.json', {
        'available': True, 'file_name': flat.name, 'file_sha256': sha256(flat),
    })
    dark = tmp_path / 'dark.png'
    assert cv2.imwrite(str(dark), np.full((12, 16), 10, dtype=np.uint8))
    return root, flat, dark


def test_chain_verifies_source_and_writes_self_describing_output(tmp_path):
    root, flat, dark = make_archive(tmp_path)
    source_before = sha256(root / 'images' / 'raw' / '000000.png')
    output = tmp_path / 'processed_run'
    report = process_archive(root, output, ProcessingOptions(
        flat_field_file=flat, dark_frame_file=dark, denoise='median3'))
    assert report['image_count'] == 2
    assert report['operations']['flat_field']['sha256'] == sha256(flat)
    assert sha256(root / 'images' / 'raw' / '000000.png') == source_before
    processed = cv2.imread(str(output / 'images' / '000000.png'), cv2.IMREAD_UNCHANGED)
    raw = cv2.imread(str(root / 'images' / 'raw' / '000000.png'), cv2.IMREAD_UNCHANGED)
    assert processed.shape == raw.shape
    assert not np.array_equal(processed, raw)
    label = json.loads((output / 'metadata' / '000000.json').read_text(encoding='utf-8'))
    assert label['processed_image_file'] == 'images/000000.png'
    assert label['image_sha256'] == source_before
    calibration = yaml.safe_load(
        (output / 'calibration' / 'rectified_camera_info.yaml').read_text(encoding='utf-8'))
    assert calibration['source_distortion_removed'] is True
    assert calibration['d'] == [0.0] * 5
    assert calibration['p'][0] == calibration['k'][0]
    assert (output / 'calibration' / 'camera_extrinsics.yaml').is_file()
    manifest = json.loads((output / 'processing_manifest.json').read_text(encoding='utf-8'))
    assert manifest['source_archive']['run_id'] == 'test-run'
    assert str(root) not in json.dumps(manifest)


def test_corrupt_source_or_output_inside_source_is_rejected_before_output(tmp_path):
    root, flat, _ = make_archive(tmp_path)
    (root / 'images' / 'raw' / '000001.png').write_bytes(b'not a PNG')
    output = tmp_path / 'processed_run'
    with pytest.raises(ProcessingError, match='SHA-256'):
        process_archive(root, output, ProcessingOptions(flat_field_file=flat))
    assert not output.exists()
    with pytest.raises(ProcessingError, match='must not be inside'):
        process_archive(root, root / 'processed', ProcessingOptions(flat_field_file=flat))


def test_incomplete_archive_requires_explicit_forensic_option(tmp_path):
    root, _, _ = make_archive(tmp_path, outcome='failed')
    with pytest.raises(ProcessingError, match='not completed'):
        process_archive(root, tmp_path / 'blocked')
    report = process_archive(
        root, tmp_path / 'forensic', ProcessingOptions(allow_incomplete=True))
    assert report['image_count'] == 2


def test_parallel_output_matches_single_worker_byte_for_byte(tmp_path):
    root, flat, dark = make_archive(tmp_path, frame_count=6)
    serial = tmp_path / 'serial'
    parallel = tmp_path / 'parallel'
    serial_report = process_archive(
        root, serial, ProcessingOptions(
            flat_field_file=flat, dark_frame_file=dark, denoise='median3',
            jobs=1, memory_budget_gb=1.0))
    parallel_report = process_archive(
        root, parallel, ProcessingOptions(
            flat_field_file=flat, dark_frame_file=dark, denoise='median3',
            jobs=2, memory_budget_gb=1.0))
    assert serial_report['execution']['jobs'] == 1
    assert parallel_report['execution']['jobs'] == 2
    assert serial_report['frames'] == parallel_report['frames']
    for relative in sorted((serial / 'images').glob('*.png')):
        counterpart = parallel / relative.relative_to(serial)
        assert sha256(relative) == sha256(counterpart)
    for relative in sorted((serial / 'metadata').glob('*.json')):
        counterpart = parallel / relative.relative_to(serial)
        assert relative.read_bytes() == counterpart.read_bytes()


def test_parallel_worker_failure_removes_unpublished_temporary_output(tmp_path, monkeypatch):
    root, flat, _ = make_archive(tmp_path, frame_count=3)
    original_load = processing._load_source_frames

    def remove_verified_image(*args, **kwargs):
        frames = original_load(*args, **kwargs)
        frames[1].image_path.unlink()
        return frames

    monkeypatch.setattr(processing, '_load_source_frames', remove_verified_image)
    output = tmp_path / 'parallel_failure'
    with pytest.raises(ProcessingError, match='could not read verified raw image'):
        process_archive(
            root, output, ProcessingOptions(
                flat_field_file=flat, jobs=2, memory_budget_gb=1.0))
    assert not output.exists()
    assert not list(tmp_path.glob('.parallel_failure.processing-*'))


def test_memory_budget_rejects_unaffordable_worker_count(tmp_path):
    root, _, _ = make_archive(tmp_path)
    with pytest.raises(ProcessingError, match='exceeds memory budget capacity'):
        process_archive(root, tmp_path / 'blocked', ProcessingOptions(
            jobs=2, memory_budget_gb=0.75))
