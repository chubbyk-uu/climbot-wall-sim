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

"""
Verified offline processing for the immutable G4 inspection archive format.

The module deliberately has no ROS dependency.  It accepts one completed G4
archive, verifies every labelled source PNG before creating output, and writes
an independently self-describing processing directory.  Raw source data is
never modified, copied over, or used as an output parent.
"""

from __future__ import annotations

from concurrent.futures import FIRST_COMPLETED, ProcessPoolExecutor, wait
from dataclasses import dataclass
import hashlib
import json
import math
from multiprocessing import get_context
import os
from pathlib import Path, PurePosixPath
import shutil
import tempfile
import time
from typing import Any
import uuid

import cv2
import numpy as np
import yaml


class ProcessingError(ValueError):
    """A caller-visible archive validation or preprocessing failure."""


@dataclass(frozen=True)
class ProcessingOptions:
    """Explicit, recorded options for one deterministic preprocessing run."""

    flat_field_file: Path | None = None
    dark_frame_file: Path | None = None
    denoise: str = 'none'
    allow_incomplete: bool = False
    # ``None`` represents the CLI spelling ``auto``.  The resolved count is
    # recorded in the output manifest, so it is never ambiguous after a run.
    jobs: int | None = None
    memory_budget_gb: float = 4.0


@dataclass(frozen=True)
class SourceFrame:
    """One source image and its immutable capture label."""

    label_name: str
    image_name: str
    image_path: Path
    label: dict[str, Any]


# The estimate deliberately includes temporary float correction, remap output,
# PNG encoding buffers and a margin for OpenCV allocator behaviour.  It is a
# scheduling limit rather than a claim that every image always occupies this
# amount of resident memory.
_WORKER_MEMORY_ESTIMATE_BYTES = 256 * 1024 * 1024
_PARENT_MEMORY_RESERVE_BYTES = 512 * 1024 * 1024
_AUTO_JOB_CAP = 8
_WORKER_CONTEXT: dict[str, Any] | None = None


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _read_json(path: Path, description: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ProcessingError(f'{description} is not valid JSON: {error}') from error
    if not isinstance(value, dict):
        raise ProcessingError(f'{description} must be a JSON object.')
    return value


def _write_json(path: Path, document: dict[str, Any]) -> None:
    try:
        payload = json.dumps(
            document, ensure_ascii=False, allow_nan=False, indent=2,
            sort_keys=True).encode('utf-8') + b'\n'
    except (TypeError, ValueError) as error:
        raise ProcessingError(f'processing metadata is not strict JSON: {error}') from error
    _atomic_write(path, payload)


def _atomic_write(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f'.{path.name}.', suffix='.tmp', dir=path.parent)
    temporary_path = Path(temporary)
    try:
        with os.fdopen(descriptor, 'wb') as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, path)
    except Exception:
        temporary_path.unlink(missing_ok=True)
        raise


def _finite_vector(value: Any, length: int, name: str) -> list[float]:
    if not isinstance(value, list) or len(value) != length:
        raise ProcessingError(f'{name} must contain exactly {length} numeric values.')
    try:
        result = [float(item) for item in value]
    except (TypeError, ValueError) as error:
        raise ProcessingError(f'{name} must contain numeric values.') from error
    if not all(math.isfinite(item) for item in result):
        raise ProcessingError(f'{name} must contain only finite values.')
    return result


def _strict_source_image(root: Path, relative_name: str) -> tuple[str, Path]:
    pure = PurePosixPath(relative_name)
    if pure.is_absolute() or '..' in pure.parts or pure.parts[:2] != ('images', 'raw'):
        raise ProcessingError(
            f'label image_file must be a safe images/raw relative path, got {relative_name!r}.')
    if pure.suffix.lower() != '.png':
        raise ProcessingError(f'label image_file must name a PNG, got {relative_name!r}.')
    path = root.joinpath(*pure.parts)
    try:
        resolved = path.resolve(strict=True)
        resolved.relative_to(root.resolve(strict=True))
    except (OSError, ValueError) as error:
        raise ProcessingError(
            f'label image_file escapes archive root: {relative_name!r}.') from error
    if not resolved.is_file():
        raise ProcessingError(f'label image_file is not a regular file: {relative_name!r}.')
    return pure.as_posix(), resolved


def _load_camera_info(root: Path) -> tuple[np.ndarray, np.ndarray, int, int,
                                           dict[str, Any]]:
    path = root / 'calibration' / 'camera_info.yaml'
    try:
        document = yaml.safe_load(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeDecodeError, yaml.YAMLError) as error:
        raise ProcessingError(f'camera_info.yaml is not readable YAML: {error}') from error
    if not isinstance(document, dict):
        raise ProcessingError('camera_info.yaml must contain a mapping.')
    try:
        width = int(document['width'])
        height = int(document['height'])
    except (KeyError, TypeError, ValueError) as error:
        raise ProcessingError('camera_info.yaml has invalid width or height.') from error
    if width <= 0 or height <= 0:
        raise ProcessingError('camera_info.yaml width and height must be positive.')
    matrix = np.asarray(
        _finite_vector(document.get('k'), 9, 'camera_info.k'), dtype=np.float64)
    distortion_value = document.get('d')
    if not isinstance(distortion_value, list) or len(distortion_value) < 4:
        raise ProcessingError('camera_info.d must contain at least four distortion coefficients.')
    distortion = np.asarray(
        _finite_vector(distortion_value, len(distortion_value), 'camera_info.d'), dtype=np.float64)
    matrix = matrix.reshape((3, 3))
    if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0 or matrix[2, 2] == 0.0:
        raise ProcessingError('camera_info.k has an invalid focal length or homogeneous scale.')
    if document.get('distortion_model') != 'plumb_bob':
        raise ProcessingError(
            'only plumb_bob camera calibration is supported in the first processing chain.')
    return matrix, distortion, width, height, document


def _load_manifest(root: Path, allow_incomplete: bool) -> dict[str, Any]:
    manifest = _read_json(root / 'manifest.json', 'manifest.json')
    if manifest.get('archive_format_version') != 1:
        raise ProcessingError('unsupported or missing archive_format_version.')
    if not allow_incomplete and manifest.get('outcome') != 'completed':
        raise ProcessingError(
            'archive is not completed; use allow_incomplete only for explicit '
            'forensic processing.')
    for key in ('expected_images', 'saved_images', 'failed_images'):
        if not isinstance(manifest.get(key), int) or manifest[key] < 0:
            raise ProcessingError(f'manifest {key} must be a non-negative integer.')
    if not allow_incomplete and (
            manifest['expected_images'] != manifest['saved_images'] or
            manifest['failed_images'] != 0):
        raise ProcessingError(
            'completed archive does not satisfy its image-count integrity contract.')
    canonical = manifest.get('canonical_image')
    if not isinstance(canonical, dict) or canonical.get('encoding') != 'mono8' or \
            canonical.get('format') != 'png' or canonical.get('distorted') is not True or \
            canonical.get('illumination_compensated') is not False:
        raise ProcessingError(
            'archive canonical_image is not the required raw distorted mono8 PNG contract.')
    return manifest


def _load_source_frames(root: Path, manifest: dict[str, Any], width: int,
                        height: int, allow_incomplete: bool) -> list[SourceFrame]:
    metadata = root / 'metadata'
    if not metadata.is_dir():
        raise ProcessingError('archive metadata directory is missing.')
    labels = sorted(metadata.glob('*.json'))
    if not labels:
        raise ProcessingError('archive contains no metadata labels.')
    if not allow_incomplete and len(labels) != manifest['saved_images']:
        raise ProcessingError(
            f'archive has {len(labels)} labels but manifest records '
            f'{manifest["saved_images"]} images.')
    frames: list[SourceFrame] = []
    seen_images: set[str] = set()
    for label_path in labels:
        label = _read_json(label_path, f'label {label_path.name}')
        if label.get('archive_schema_version') != 1:
            raise ProcessingError(f'label {label_path.name} has an unsupported archive schema.')
        image_name = label.get('image_file')
        digest = label.get('image_sha256')
        if (not isinstance(image_name, str) or not isinstance(digest, str) or
                len(digest) != 64):
            raise ProcessingError(f'label {label_path.name} lacks image_file or SHA-256.')
        image_name, image_path = _strict_source_image(root, image_name)
        if image_name in seen_images:
            raise ProcessingError(f'multiple labels reference {image_name}.')
        seen_images.add(image_name)
        if _sha256_file(image_path) != digest:
            raise ProcessingError(f'raw image SHA-256 differs from label: {image_name}.')
        image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
        if (image is None or image.dtype != np.uint8 or image.ndim != 2 or
                image.shape != (height, width)):
            raise ProcessingError(
                f'raw image {image_name} is not {width}x{height} mono8 PNG data.')
        if label.get('image_width') != width or label.get('image_height') != height or \
                label.get('image_encoding') != 'mono8':
            raise ProcessingError(
                f'label {label_path.name} disagrees with the archived camera calibration.')
        frames.append(SourceFrame(label_path.name, image_name, image_path, label))
    return frames


def _load_gain(path: Path | None, expected_shape: tuple[int, int],
               root: Path) -> tuple[np.ndarray | None, dict[str, Any]]:
    if path is None:
        return None, {'enabled': False}
    if not path.is_file():
        raise ProcessingError(f'flat-field file does not exist: {path}.')
    try:
        with np.load(path, allow_pickle=False) as archive:
            gain = np.asarray(archive['gain'], dtype=np.float32)
    except (KeyError, OSError, ValueError) as error:
        raise ProcessingError(f'flat-field file lacks a readable gain matrix: {error}') from error
    if (gain.shape != expected_shape or not np.all(np.isfinite(gain)) or
            not np.all(gain > 0.0)):
        raise ProcessingError(
            'flat-field gain must be finite, positive and match camera dimensions.')
    digest = _sha256_file(path)
    reference_path = root / 'calibration' / 'flat_field_reference.json'
    if reference_path.is_file():
        reference = _read_json(reference_path, 'flat_field_reference.json')
        if reference.get('available') is True and reference.get('file_sha256') != digest:
            raise ProcessingError(
                'flat-field SHA-256 does not match the archive calibration reference.')
    return gain, {'enabled': True, 'sha256': digest, 'file_name': path.name}


def _load_dark_frame(path: Path | None, expected_shape: tuple[int, int]) -> tuple[
        np.ndarray | None, dict[str, Any]]:
    if path is None:
        return None, {'enabled': False}
    if not path.is_file():
        raise ProcessingError(f'dark-frame file does not exist: {path}.')
    dark = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if (dark is None or dark.dtype != np.uint8 or dark.ndim != 2 or
            dark.shape != expected_shape):
        raise ProcessingError('dark-frame must be a mono8 image matching the camera dimensions.')
    return dark.astype(np.float32), {
        'enabled': True, 'sha256': _sha256_file(path), 'file_name': path.name}


def _processed_image(raw: np.ndarray, dark: np.ndarray | None, gain: np.ndarray | None,
                     denoise: str, remap_x: np.ndarray,
                     remap_y: np.ndarray) -> np.ndarray:
    """Correct one frame using remaps created once for the whole archive."""
    corrected = raw.astype(np.float32)
    if dark is not None:
        corrected = np.maximum(corrected - dark, 0.0)
    if gain is not None:
        corrected *= gain
    corrected = np.clip(corrected, 0.0, 255.0).astype(np.uint8)
    if denoise == 'median3':
        corrected = cv2.medianBlur(corrected, 3)
    elif denoise != 'none':
        raise ProcessingError(f'unsupported denoise mode {denoise!r}.')
    return cv2.remap(
        corrected, remap_x, remap_y, interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT)


def _undistort_maps(matrix: np.ndarray, distortion: np.ndarray,
                    new_matrix: np.ndarray, width: int,
                    height: int) -> tuple[np.ndarray, np.ndarray]:
    """Build the immutable remaps shared by all archive worker processes."""
    remap_x, remap_y = cv2.initUndistortRectifyMap(
        matrix, distortion, None, new_matrix, (width, height), cv2.CV_32FC1)
    if (remap_x.shape != (height, width) or remap_y.shape != (height, width) or
            not np.all(np.isfinite(remap_x)) or not np.all(np.isfinite(remap_y))):
        raise ProcessingError('OpenCV produced invalid undistortion remaps.')
    return remap_x, remap_y


def _init_worker(context: dict[str, Any]) -> None:
    """Install one read-only processing context in a child process."""
    global _WORKER_CONTEXT
    # Parallel OpenCV workers must not each create their own CPU-sized thread
    # pool.  Process-level parallelism is controlled by ``jobs`` instead.
    cv2.setNumThreads(1)
    _WORKER_CONTEXT = context


def _process_one_frame(frame: SourceFrame, temporary: Path,
                       context: dict[str, Any]) -> dict[str, Any]:
    """Process and atomically persist one already-verified source frame."""
    raw = cv2.imread(str(frame.image_path), cv2.IMREAD_UNCHANGED)
    if raw is None:
        raise ProcessingError(f'OpenCV could not read verified raw image {frame.image_name}.')
    if (raw.dtype != np.uint8 or raw.ndim != 2 or
            raw.shape != context['shape']):
        raise ProcessingError(
            f'verified raw image changed shape or type before processing: {frame.image_name}.')
    processed = _processed_image(
        raw, context['dark'], context['gain'], context['denoise'],
        context['remap_x'], context['remap_y'])
    encoded_ok, encoded = cv2.imencode('.png', processed)
    if not encoded_ok:
        raise ProcessingError(f'OpenCV could not encode {frame.image_name}.')
    image_name = Path(frame.image_name).name
    processed_relative = f'images/{image_name}'
    image_path = temporary / 'images' / image_name
    _atomic_write(image_path, bytes(encoded))
    processed_sha = _sha256_file(image_path)
    label = dict(frame.label)
    label['processing_schema_version'] = 1
    label['source_label_file'] = f'metadata/{frame.label_name}'
    label['processed_image_file'] = processed_relative
    label['processed_image_sha256'] = processed_sha
    label['processing'] = {
        'dark_subtraction': context['dark'] is not None,
        'flat_field': context['gain'] is not None,
        'denoise': context['denoise'],
        'undistorted': True,
    }
    _write_json(temporary / 'metadata' / frame.label_name, label)
    return {
        'source_label_file': f'metadata/{frame.label_name}',
        'source_image_file': frame.image_name,
        'source_image_sha256': frame.label['image_sha256'],
        'processed_image_file': processed_relative,
        'processed_image_sha256': processed_sha,
    }


def _process_one_frame_in_worker(frame: SourceFrame, temporary: str) -> dict[str, Any]:
    """Worker entry point; context is installed once through the initializer."""
    if _WORKER_CONTEXT is None:
        raise RuntimeError('image-processing worker was not initialized.')
    return _process_one_frame(frame, Path(temporary), _WORKER_CONTEXT)


def _resolve_jobs(requested: int | None, memory_budget_gb: float) -> int:
    """Resolve a bounded process count before any temporary output exists."""
    if (not math.isfinite(memory_budget_gb) or memory_budget_gb <= 0.0):
        raise ProcessingError('memory_budget_gb must be a finite positive number.')
    if requested is not None and requested <= 0:
        raise ProcessingError('jobs must be a positive integer or auto.')
    budget_bytes = int(memory_budget_gb * 1024 ** 3)
    capacity = max(0, (budget_bytes - _PARENT_MEMORY_RESERVE_BYTES) //
                   _WORKER_MEMORY_ESTIMATE_BYTES)
    if capacity < 1:
        raise ProcessingError(
            'memory_budget_gb is too small for one worker after the parent reserve.')
    cpu_count = os.cpu_count() or 1
    if requested is not None:
        if requested > capacity:
            raise ProcessingError(
                f'jobs={requested} exceeds memory budget capacity {capacity}; '
                'increase memory_budget_gb or reduce jobs.')
        return requested
    return min(cpu_count, _AUTO_JOB_CAP, capacity)


def _parallel_outputs(frames: list[SourceFrame], temporary: Path,
                      context: dict[str, Any], jobs: int) -> list[dict[str, Any]]:
    """Run frame work with bounded submission and stable output ordering."""
    if jobs == 1:
        return [_process_one_frame(frame, temporary, context) for frame in frames]

    # On Linux/WSL, fork makes the large, immutable gain and remap arrays
    # copy-on-write shared.  Other POSIX platforms still work; their selected
    # context may make private copies and is covered by the memory budget.
    multiprocessing_context = get_context('fork') if os.name == 'posix' else None
    results: list[dict[str, Any] | None] = [None] * len(frames)
    in_flight: dict[Any, int] = {}
    next_index = 0
    # Limit queued tasks as well as workers.  Images remain on disk until the
    # worker reads them, so this bounds only small task objects and results.
    window = jobs * 2
    with ProcessPoolExecutor(
            max_workers=jobs, mp_context=multiprocessing_context,
            initializer=_init_worker, initargs=(context,)) as executor:
        while next_index < len(frames) or in_flight:
            while next_index < len(frames) and len(in_flight) < window:
                future = executor.submit(
                    _process_one_frame_in_worker, frames[next_index], str(temporary))
                in_flight[future] = next_index
                next_index += 1
            done, _ = wait(in_flight, return_when=FIRST_COMPLETED)
            for future in done:
                index = in_flight.pop(future)
                results[index] = future.result()
    # ``future.result`` either filled every stable slot or propagated a worker
    # exception.  This assertion guards against a future refactor publishing a
    # partial archive after a scheduling mistake.
    if any(item is None for item in results):
        raise RuntimeError('parallel processing ended with missing frame results.')
    return [item for item in results if item is not None]


def _output_root(input_root: Path, output_root: Path) -> tuple[Path, Path]:
    source = input_root.resolve(strict=True)
    candidate = output_root.expanduser()
    if not candidate.is_absolute():
        raise ProcessingError('output directory must be absolute.')
    output = candidate.resolve(strict=False)
    if output.exists():
        raise ProcessingError(f'output directory already exists: {output}.')
    try:
        output.relative_to(source)
    except ValueError:
        pass
    else:
        raise ProcessingError('output directory must not be inside the immutable source archive.')
    if not output.parent.is_dir():
        raise ProcessingError(f'output parent directory does not exist: {output.parent}.')
    temporary = output.with_name(f'.{output.name}.processing-{uuid.uuid4().hex}')
    return output, temporary


def _rectified_camera_info(source: dict[str, Any], new_matrix: np.ndarray) -> dict[
        str, Any]:
    result = dict(source)
    result['k'] = [float(value) for value in new_matrix.reshape(-1)]
    result['d'] = [0.0 for _ in source['d']]
    result['p'] = [
        float(new_matrix[0, 0]), 0.0, float(new_matrix[0, 2]), 0.0,
        0.0, float(new_matrix[1, 1]), float(new_matrix[1, 2]), 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]
    result['distortion_model'] = 'plumb_bob'
    result['rectified'] = True
    result['source_distortion_removed'] = True
    return result


def _load_camera_extrinsics(root: Path) -> dict[str, Any]:
    """Load the immutable mount snapshot required by later mosaic projection."""
    source = root / 'calibration' / 'camera_extrinsics.yaml'
    try:
        document = yaml.safe_load(source.read_text(encoding='utf-8'))
    except (OSError, UnicodeDecodeError, yaml.YAMLError) as error:
        raise ProcessingError(f'camera_extrinsics.yaml is not readable YAML: {error}') from error
    if not isinstance(document, dict) or not isinstance(document.get('optical_mount'), dict):
        raise ProcessingError('camera_extrinsics.yaml lacks an optical_mount mapping.')
    return document


def _write_camera_extrinsics(extrinsics: dict[str, Any], temporary: Path) -> None:
    """Write the validated mount snapshot into a processed archive."""
    _atomic_write(
        temporary / 'calibration' / 'camera_extrinsics.yaml',
        yaml.safe_dump(extrinsics, allow_unicode=True, sort_keys=True).encode('utf-8'))


def process_archive(input_run: Path | str, output_dir: Path | str,
                    options: ProcessingOptions | None = None) -> dict[str, Any]:
    """
    Verify and process one immutable archive into a newly created directory.

    The output directory is atomically published only after every source image
    has passed integrity validation and every corrected result has been written.
    Returned data is JSON-safe and is also stored as ``processing_manifest.json``.
    """
    options = options or ProcessingOptions()
    if options.denoise not in {'none', 'median3'}:
        raise ProcessingError("denoise must be either 'none' or 'median3'.")
    jobs = _resolve_jobs(options.jobs, options.memory_budget_gb)
    # Keep the sequential and process-pool paths semantically identical.  It
    # also prevents the one-worker path from quietly using a machine-sized
    # OpenCV thread pool and invalidating a jobs=1 versus jobs=N benchmark.
    cv2.setNumThreads(1)
    root = Path(input_run).expanduser()
    if not root.is_absolute():
        raise ProcessingError('input archive directory must be absolute.')
    try:
        root = root.resolve(strict=True)
    except OSError as error:
        raise ProcessingError(f'input archive directory does not exist: {root}.') from error
    if not root.is_dir():
        raise ProcessingError(f'input archive is not a directory: {root}.')
    output, temporary = _output_root(root, Path(output_dir))
    manifest = _load_manifest(root, options.allow_incomplete)
    matrix, distortion, width, height, camera_info = _load_camera_info(root)
    extrinsics = _load_camera_extrinsics(root)
    frames = _load_source_frames(root, manifest, width, height, options.allow_incomplete)
    gain, gain_metadata = _load_gain(
        Path(options.flat_field_file).expanduser() if options.flat_field_file else None,
        (height, width), root)
    dark, dark_metadata = _load_dark_frame(
        Path(options.dark_frame_file).expanduser() if options.dark_frame_file else None,
        (height, width))
    new_matrix, roi = cv2.getOptimalNewCameraMatrix(
        matrix, distortion, (width, height), 0.0, (width, height))
    remap_x, remap_y = _undistort_maps(
        matrix, distortion, new_matrix, width, height)
    source_manifest_sha = _sha256_file(root / 'manifest.json')
    context = {
        'shape': (height, width),
        'dark': dark,
        'gain': gain,
        'denoise': options.denoise,
        'remap_x': remap_x,
        'remap_y': remap_y,
    }
    started = time.monotonic()
    try:
        temporary.mkdir(parents=False, exist_ok=False)
        outputs = _parallel_outputs(frames, temporary, context, jobs)
        rectified_camera = _rectified_camera_info(camera_info, new_matrix)
        _atomic_write(
            temporary / 'calibration' / 'rectified_camera_info.yaml',
            yaml.safe_dump(
                rectified_camera, allow_unicode=True, sort_keys=True).encode('utf-8'))
        _write_camera_extrinsics(extrinsics, temporary)
        report = {
            'processing_format_version': 1,
            'source_archive': {
                'run_id': manifest.get('run_id'),
                'manifest_sha256': source_manifest_sha,
                'archive_format_version': manifest['archive_format_version'],
                'outcome': manifest.get('outcome'),
            },
            'image_count': len(outputs),
            'image_geometry': {'width_px': width, 'height_px': height},
            'operations': {
                'dark_subtraction': dark_metadata,
                'flat_field': gain_metadata,
                'denoise': options.denoise,
                'undistortion': {
                    'source_distortion_model': camera_info['distortion_model'],
                    'rectified_k': [float(value) for value in new_matrix.reshape(-1)],
                    'roi_xywh': [int(value) for value in roi],
                },
            },
            'execution': {
                'jobs': jobs,
                'memory_budget_gb': options.memory_budget_gb,
                'frame_processing_elapsed_s': time.monotonic() - started,
                'undistortion_remaps': 'precomputed_cv32fc1',
            },
            'frames': outputs,
        }
        _write_json(temporary / 'processing_manifest.json', report)
        os.replace(temporary, output)
        return report
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
