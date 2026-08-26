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

"""Strict, side-effect-free validation of processed mosaic input archives."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import re
from typing import Any, Iterable

import cv2
import numpy as np
import yaml


class MosaicInputError(ValueError):
    """A processed archive cannot safely enter the offline mosaic pipeline."""


@dataclass(frozen=True, order=True)
class FrameKey:
    """A globally unique processed frame identifier across one or more runs."""

    source_run_id: str
    frame_index: int


@dataclass(frozen=True)
class CameraModel:
    """Validated rectified camera and fixed mount needed by wall projection."""

    width: int
    height: int
    matrix: tuple[float, ...]
    mount_center_xyz_m: tuple[float, float, float]
    mount_rpy_rad: tuple[float, float, float]
    signature: str


@dataclass(frozen=True)
class ProcessedFrame:
    """One integrity-checked processed image with its immutable capture label."""

    key: FrameKey
    image_path: Path
    image_sha256: str
    label_path: Path
    label: dict[str, Any]


@dataclass(frozen=True)
class ProcessedRun:
    """One verified processed-run directory and all of its frames."""

    source_run_id: str
    processing_manifest_sha256: str
    root: Path
    camera: CameraModel
    frames: tuple[ProcessedFrame, ...]


@dataclass(frozen=True)
class MosaicInputs:
    """A deterministic collection of compatible processed runs."""

    camera: CameraModel
    runs: tuple[ProcessedRun, ...]
    frames: tuple[ProcessedFrame, ...]


_SHA256 = re.compile(r'^[0-9a-f]{64}$')
_RUN_ID = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]{0,95}$')


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _read_json(path: Path, description: str) -> dict[str, Any]:
    try:
        document = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise MosaicInputError(f'{description} is not valid JSON: {error}') from error
    if not isinstance(document, dict):
        raise MosaicInputError(f'{description} must be a JSON object.')
    return document


def _read_yaml(path: Path, description: str) -> dict[str, Any]:
    try:
        document = yaml.safe_load(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeDecodeError, yaml.YAMLError) as error:
        raise MosaicInputError(f'{description} is not readable YAML: {error}') from error
    if not isinstance(document, dict):
        raise MosaicInputError(f'{description} must contain a mapping.')
    return document


def _finite_vector(value: Any, length: int, description: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != length:
        raise MosaicInputError(f'{description} must contain exactly {length} values.')
    try:
        result = tuple(float(item) for item in value)
    except (TypeError, ValueError) as error:
        raise MosaicInputError(f'{description} must contain numeric values.') from error
    if not all(math.isfinite(item) for item in result):
        raise MosaicInputError(f'{description} must contain finite values only.')
    return result


def _finite_fields(document: Any, names: tuple[str, ...],
                   description: str) -> tuple[float, ...]:
    if not isinstance(document, dict):
        raise MosaicInputError(f'{description} must be a mapping.')
    try:
        result = tuple(float(document[name]) for name in names)
    except (KeyError, TypeError, ValueError) as error:
        raise MosaicInputError(f'{description} lacks numeric {names!r}.') from error
    if not all(math.isfinite(item) for item in result):
        raise MosaicInputError(f'{description} must contain finite values only.')
    return result


def _nonnegative_integer(value: Any, description: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise MosaicInputError(f'{description} must be a non-negative integer.')
    return value


def _safe_relative(root: Path, value: Any, top_level: str,
                   suffix: str, description: str) -> tuple[str, Path]:
    if not isinstance(value, str):
        raise MosaicInputError(f'{description} must be a relative path string.')
    relative = PurePosixPath(value)
    if (relative.is_absolute() or '..' in relative.parts or not relative.parts or
            relative.parts[0] != top_level or relative.suffix.lower() != suffix):
        raise MosaicInputError(
            f'{description} must be a safe {top_level}/*{suffix} relative path.')
    candidate = root.joinpath(*relative.parts)
    try:
        resolved = candidate.resolve(strict=True)
        resolved.relative_to(root)
    except (OSError, ValueError) as error:
        raise MosaicInputError(f'{description} escapes or is absent from the run.') from error
    if not resolved.is_file():
        raise MosaicInputError(f'{description} is not a regular file.')
    return relative.as_posix(), resolved


def _sha256(value: Any, description: str) -> str:
    if not isinstance(value, str) or not _SHA256.fullmatch(value):
        raise MosaicInputError(f'{description} must be a lowercase SHA-256 string.')
    return value


def _camera_model(root: Path, manifest: dict[str, Any]) -> CameraModel:
    camera = _read_yaml(
        root / 'calibration' / 'rectified_camera_info.yaml',
        'rectified_camera_info.yaml')
    try:
        width = int(camera['width'])
        height = int(camera['height'])
    except (KeyError, TypeError, ValueError) as error:
        raise MosaicInputError('rectified camera dimensions are invalid.') from error
    if width <= 0 or height <= 0:
        raise MosaicInputError('rectified camera dimensions must be positive.')
    geometry = manifest.get('image_geometry')
    if not isinstance(geometry, dict) or geometry.get('width_px') != width or \
            geometry.get('height_px') != height:
        raise MosaicInputError(
            'processing manifest image geometry disagrees with camera calibration.')
    matrix = _finite_vector(camera.get('k'), 9, 'rectified_camera_info.k')
    if matrix[0] <= 0.0 or matrix[4] <= 0.0 or matrix[8] == 0.0:
        raise MosaicInputError('rectified camera matrix has invalid focal lengths or scale.')
    distortion = camera.get('d')
    if not isinstance(distortion, list) or not distortion:
        raise MosaicInputError('rectified camera distortion vector is missing.')
    if any(value != 0.0 for value in _finite_vector(
            distortion, len(distortion), 'rectified_camera_info.d')):
        raise MosaicInputError(
            'mosaic input camera calibration must have zero rectified distortion.')
    if camera.get('rectified') is not True or camera.get('source_distortion_removed') is not True:
        raise MosaicInputError('mosaic input camera calibration is not marked rectified.')
    extrinsics = _read_yaml(
        root / 'calibration' / 'camera_extrinsics.yaml', 'camera_extrinsics.yaml')
    mount = extrinsics.get('optical_mount')
    if not isinstance(mount, dict):
        raise MosaicInputError('camera_extrinsics.yaml lacks optical_mount.')
    center = _finite_vector(
        mount.get('center_xyz_m'), 3, 'optical_mount.center_xyz_m')
    rpy = _finite_vector(mount.get('rpy_rad'), 3, 'optical_mount.rpy_rad')
    signature_document = {
        'height': height,
        'k': matrix,
        'mount_center_xyz_m': center,
        'mount_rpy_rad': rpy,
        'width': width,
    }
    signature = hashlib.sha256(json.dumps(
        signature_document, allow_nan=False, separators=(',', ':'),
        sort_keys=True).encode('utf-8')).hexdigest()
    return CameraModel(width, height, matrix, center, rpy, signature)


def _validate_pose(label: dict[str, Any], description: str) -> None:
    camera_pose = label.get('camera_pose')
    if not isinstance(camera_pose, dict):
        raise MosaicInputError(f'{description} lacks camera_pose.')
    pose = camera_pose.get('pose')
    if not isinstance(pose, dict):
        raise MosaicInputError(f'{description} lacks camera_pose.pose.')
    _finite_fields(pose.get('position'), ('x', 'y', 'z'), f'{description} position')
    orientation = _finite_fields(
        pose.get('orientation'), ('x', 'y', 'z', 'w'), f'{description} orientation')
    norm = math.sqrt(sum(value * value for value in orientation))
    if abs(norm - 1.0) > 1e-3:
        raise MosaicInputError(f'{description} orientation is not a unit quaternion.')
    _finite_vector(camera_pose.get('covariance'), 36, f'{description} covariance')
    heading = label.get('wall_heading_rad')
    try:
        heading = float(heading)
    except (TypeError, ValueError) as error:
        raise MosaicInputError(f'{description} wall_heading_rad is not numeric.') from error
    if not math.isfinite(heading):
        raise MosaicInputError(f'{description} wall_heading_rad is not finite.')


def _load_run(path: Path) -> ProcessedRun:
    if not path.is_absolute():
        raise MosaicInputError('processed input run must be an absolute path.')
    try:
        root = path.expanduser().resolve(strict=True)
    except OSError as error:
        raise MosaicInputError(f'processed input run does not exist: {path}.') from error
    if not root.is_dir():
        raise MosaicInputError(f'processed input run is not a directory: {root}.')
    manifest_path = root / 'processing_manifest.json'
    manifest = _read_json(manifest_path, 'processing_manifest.json')
    if manifest.get('processing_format_version') != 1:
        raise MosaicInputError('unsupported or missing processing_format_version.')
    image_count = _nonnegative_integer(manifest.get('image_count'), 'processing image_count')
    if image_count == 0:
        raise MosaicInputError('processed run contains no frames.')
    source_archive = manifest.get('source_archive')
    if not isinstance(source_archive, dict) or source_archive.get('outcome') != 'completed':
        raise MosaicInputError('processed run source archive must be completed.')
    source_run_id = source_archive.get('run_id')
    if not isinstance(source_run_id, str) or not _RUN_ID.fullmatch(source_run_id):
        raise MosaicInputError('processed run has an invalid source run_id.')
    _sha256(source_archive.get('manifest_sha256'), 'source archive manifest SHA-256')
    frame_records = manifest.get('frames')
    if not isinstance(frame_records, list) or len(frame_records) != image_count:
        raise MosaicInputError('processing manifest frame count disagrees with image_count.')
    camera = _camera_model(root, manifest)
    records_by_label: dict[str, dict[str, Any]] = {}
    for record in frame_records:
        if not isinstance(record, dict):
            raise MosaicInputError('processing manifest contains a non-object frame record.')
        label_name, _ = _safe_relative(
            root, record.get('source_label_file'), 'metadata', '.json',
            'processing manifest source_label_file')
        if label_name in records_by_label:
            raise MosaicInputError('processing manifest repeats one source label.')
        _safe_relative(
            root, record.get('processed_image_file'), 'images', '.png',
            'processing manifest processed_image_file')
        _sha256(record.get('source_image_sha256'), 'processing manifest source image SHA-256')
        _sha256(
            record.get('processed_image_sha256'),
            'processing manifest processed image SHA-256')
        records_by_label[label_name] = record
    metadata = root / 'metadata'
    if not metadata.is_dir():
        raise MosaicInputError('processed run metadata directory is missing.')
    labels = sorted(metadata.glob('*.json'))
    expected_labels = set(records_by_label)
    actual_labels = {f'metadata/{item.name}' for item in labels}
    if actual_labels != expected_labels:
        raise MosaicInputError(
            'processing manifest and metadata directory do not contain the same labels.')
    frames: list[ProcessedFrame] = []
    used_indices: set[int] = set()
    for label_path in labels:
        label_name = f'metadata/{label_path.name}'
        record = records_by_label[label_name]
        label = _read_json(label_path, f'processed label {label_path.name}')
        if label.get('processing_schema_version') != 1:
            raise MosaicInputError(f'processed label {label_path.name} has an unsupported schema.')
        processing = label.get('processing')
        if not isinstance(processing, dict) or processing.get('undistorted') is not True:
            raise MosaicInputError(f'processed label {label_path.name} is not marked undistorted.')
        if label.get('source_label_file') != label_name:
            raise MosaicInputError(
                f'processed label {label_path.name} has a mismatched source label path.')
        image_relative, image_path = _safe_relative(
            root, label.get('processed_image_file'), 'images', '.png',
            f'processed label {label_path.name} image')
        if image_relative != record.get('processed_image_file'):
            raise MosaicInputError(
                f'processed label {label_path.name} disagrees with its manifest frame.')
        digest = _sha256(
            label.get('processed_image_sha256'),
            f'processed label {label_path.name} image SHA-256')
        if digest != record.get('processed_image_sha256') or _sha256_file(image_path) != digest:
            raise MosaicInputError(
                f'processed image SHA-256 differs from label: {image_relative}.')
        image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
        if (image is None or image.dtype != np.uint8 or image.ndim != 2 or
                image.shape != (camera.height, camera.width)):
            raise MosaicInputError(f'processed image is not rectified mono8: {image_relative}.')
        if (label.get('image_width') != camera.width or
                label.get('image_height') != camera.height or
                label.get('image_encoding') != 'mono8'):
            raise MosaicInputError(
                f'processed label {label_path.name} disagrees with camera dimensions.')
        if not isinstance(label.get('task_id'), str) or not label['task_id']:
            raise MosaicInputError(f'processed label {label_path.name} lacks task_id.')
        for field in ('revision', 'segment_index', 'trigger_index'):
            _nonnegative_integer(label.get(field), f'processed label {label_path.name} {field}')
        header = label.get('header')
        if not isinstance(header, dict):
            raise MosaicInputError(f'processed label {label_path.name} lacks header.')
        _nonnegative_integer(header.get('stamp_ns'), f'processed label {label_path.name} stamp_ns')
        _validate_pose(label, f'processed label {label_path.name}')
        if not label_path.stem.isdecimal():
            raise MosaicInputError(
                f'processed label filename is not a decimal frame index: {label_path.name}.')
        frame_index = int(label_path.stem)
        if frame_index in used_indices:
            raise MosaicInputError(f'processed run repeats frame index {frame_index}.')
        used_indices.add(frame_index)
        frames.append(ProcessedFrame(
            FrameKey(source_run_id, frame_index), image_path, digest, label_path, label))
    frames.sort(key=lambda frame: frame.key)
    return ProcessedRun(
        source_run_id, _sha256_file(manifest_path), root, camera, tuple(frames))


def validate_processed_runs(input_runs: Iterable[Path | str]) -> MosaicInputs:
    """Validate compatible processed archives without creating any output files."""
    paths = list(input_runs)
    if not paths:
        raise MosaicInputError('at least one processed input run is required.')
    runs = [_load_run(Path(path)) for path in paths]
    seen_runs: set[str] = set()
    for run in runs:
        if run.source_run_id in seen_runs:
            raise MosaicInputError(f'duplicate processed source run_id: {run.source_run_id}.')
        seen_runs.add(run.source_run_id)
    runs.sort(key=lambda run: run.source_run_id)
    camera = runs[0].camera
    if any(run.camera.signature != camera.signature for run in runs[1:]):
        raise MosaicInputError(
            'first planar mosaic requires all processed runs to share one camera '
            'and mount snapshot.')
    frames = tuple(frame for run in runs for frame in run.frames)
    if len({frame.key for frame in frames}) != len(frames):
        raise MosaicInputError(
            'processed inputs do not provide globally unique frame identifiers.')
    return MosaicInputs(camera, tuple(runs), tuple(sorted(frames, key=lambda frame: frame.key)))


def input_summary(inputs: MosaicInputs) -> dict[str, Any]:
    """Return strict-JSON, machine-readable preflight evidence without host paths."""
    task_ids = sorted({str(frame.label['task_id']) for frame in inputs.frames})
    return {
        'status': 'valid',
        'processed_run_count': len(inputs.runs),
        'frame_count': len(inputs.frames),
        'source_run_ids': [run.source_run_id for run in inputs.runs],
        'processing_manifest_sha256': [
            run.processing_manifest_sha256 for run in inputs.runs],
        'task_ids': task_ids,
        'camera': {
            'width_px': inputs.camera.width,
            'height_px': inputs.camera.height,
            'signature': inputs.camera.signature,
        },
    }
