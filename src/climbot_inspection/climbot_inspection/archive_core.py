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

"""Small deterministic primitives shared by the G4 archive recorder and tests."""

from __future__ import annotations

import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import re
import tempfile
import uuid


class ArchiveError(ValueError):
    """A caller-visible archive preflight or integrity error."""


_SAFE_TASK_ID = re.compile(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,95}\Z')


def safe_task_id(task_id: str) -> str:
    """Return a path-safe task identifier or reject it without lossy rewriting."""
    if not _SAFE_TASK_ID.fullmatch(task_id):
        raise ArchiveError(
            "task_id must contain 1..96 letters, digits, '_', '-' or '.', "
            'start with an alphanumeric character, and contain no path separators.')
    if task_id in {'.', '..'}:
        raise ArchiveError("task_id must not be '.' or '..'.")
    return task_id


def resolved_output_root(value: str) -> Path:
    """Expand the recorder-host root, but never silently accept a relative path."""
    if not value or not value.strip():
        raise ArchiveError('output_root must not be empty.')
    root = Path(value).expanduser()
    if not root.is_absolute():
        raise ArchiveError("output_root must be absolute after expanding '~'.")
    return root.resolve(strict=False)


def new_run_id() -> str:
    """Generate an opaque, collision-resistant run identifier for one archive."""
    return uuid.uuid4().hex


def run_directory(root: Path, task_id: str, revision: int, utc_stamp: str, run_id: str) -> Path:
    """Build, but do not create, the immutable task/run directory."""
    safe_task_id(task_id)
    if revision < 0:
        raise ArchiveError('revision must be non-negative.')
    if not re.fullmatch(r'[0-9]{8}T[0-9]{6}Z', utc_stamp):
        raise ArchiveError('utc_stamp must use YYYYMMDDTHHMMSSZ.')
    if not re.fullmatch(r'[0-9a-f]{32}', run_id):
        raise ArchiveError('run_id must be a 32-character lowercase UUID hex string.')
    return root / task_id / f'r{revision:06d}_{utc_stamp}_{run_id}'


def capture_spacing(effective_length_m: float, overlap_ratio: float) -> float:
    """Return the contractual maximum along-track distance between exposures."""
    if not math.isfinite(effective_length_m) or effective_length_m <= 0.0:
        raise ArchiveError('effective_length_m must be positive and finite.')
    if not math.isfinite(overlap_ratio) or not 0.0 <= overlap_ratio < 1.0:
        raise ArchiveError('overlap_ratio must be finite and within [0, 1).')
    return effective_length_m * (1.0 - overlap_ratio)


def capture_count_for_length(length_m: float, effective_length_m: float,
                             overlap_ratio: float) -> int:
    """
    Return the one count contract used by capture planning and archiving.

    A formal SCAN must not require a photograph exactly at its terminal pose:
    line tracking is allowed to declare the end reached inside its arrival
    tolerance.  Starting at the route entry and dividing the remaining route
    into ``ceil(length / spacing)`` intervals guarantees every target is
    reachable before that terminal tolerance while keeping every adjacent
    target no farther apart than ``spacing``.
    """
    spacing = capture_spacing(effective_length_m, overlap_ratio)
    if not math.isfinite(length_m) or length_m <= 1e-6:
        raise ArchiveError('SCAN reference length must be finite and positive.')
    return max(1, math.ceil(length_m / spacing))


def expected_image_count(task, effective_length_m: float, overlap_ratio: float) -> int:
    """
    Return a nominal preflight storage estimate using the capture contract.

    This is only used to reserve capacity before SCAN references are frozen.
    The final expected count is accumulated from those frozen references by
    the archive recorder.
    """
    if len(task.waypoints) != len(task.segment_types) + 1:
        raise ArchiveError('task waypoint/segment counts are inconsistent.')
    total = 0
    for index, segment_type in enumerate(task.segment_types):
        if segment_type != task.SEGMENT_SCAN:
            continue
        start = task.waypoints[index].position
        end = task.waypoints[index + 1].position
        length = math.hypot(end.x - start.x, end.y - start.y)
        try:
            total += capture_count_for_length(length, effective_length_m, overlap_ratio)
        except ArchiveError as error:
            raise ArchiveError(f'SCAN segment {index} has invalid length.') from error
    return total


def estimated_archive_bytes(image_count: int, width: int, height: int) -> int:
    """Conservative raw-mono8 estimate: uncompressed pixels plus metadata margin."""
    if image_count < 0 or width <= 0 or height <= 0:
        raise ArchiveError('image_count, width and height must be positive/non-negative.')
    # PNG can be close to raw size on a textured wall. Reserve 10% container
    # headroom and 8 KiB for each JSON sidecar rather than assuming compression.
    return int(math.ceil(image_count * (width * height * 1.10 + 8192.0)))


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def atomic_write_bytes(path: Path, payload: bytes, *, durable: bool = True) -> None:
    """
    Write one complete file before making its destination name visible.

    ``durable=False`` keeps the atomic-rename visibility guarantee but leaves
    persistence to a later filesystem-wide commit.  It is for high-rate pairs
    such as image plus label; callers must subsequently call
    :func:`sync_filesystem` before declaring those files durable.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f'.{path.name}.', suffix='.tmp', dir=path.parent)
    temporary_path = Path(temporary)
    try:
        with os.fdopen(descriptor, 'wb') as stream:
            stream.write(payload)
            stream.flush()
            if durable:
                os.fsync(stream.fileno())
        os.replace(temporary_path, path)
        if durable:
            directory_fd = os.open(path.parent, os.O_DIRECTORY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
    except Exception:
        temporary_path.unlink(missing_ok=True)
        raise


def atomic_write_json(path: Path, document: dict, *, durable: bool = True) -> None:
    """Serialize strict JSON: NaN and Infinity must never leak into labels."""
    encoded = json.dumps(
        document, ensure_ascii=False, allow_nan=False, indent=2,
        sort_keys=True).encode('utf-8') + b'\n'
    atomic_write_bytes(path, encoded, durable=durable)


def sync_filesystem(path: Path) -> None:
    """
    Persist dirty data for ``path``'s Linux filesystem without global sync.

    Linux exposes ``syncfs(2)`` but CPython does not wrap it.  Calling it once
    for a batch flushes the image bytes and rename metadata together, avoiding
    two expensive ``fsync`` calls for every individual file.
    """
    directory = path if path.is_dir() else path.parent
    descriptor = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
    try:
        libc = ctypes.CDLL(None, use_errno=True)
        syncfs = libc.syncfs
        syncfs.argtypes = [ctypes.c_int]
        syncfs.restype = ctypes.c_int
        if syncfs(descriptor) != 0:
            error_number = ctypes.get_errno()
            raise OSError(error_number, os.strerror(error_number), str(directory))
    finally:
        os.close(descriptor)
