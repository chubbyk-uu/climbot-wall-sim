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


def expected_image_count(task, effective_length_m: float, overlap_ratio: float) -> int:
    """Match automatic_capture_node's per-SCAN trigger-count calculation."""
    if not math.isfinite(effective_length_m) or effective_length_m <= 0.0:
        raise ArchiveError('effective_length_m must be positive and finite.')
    if not math.isfinite(overlap_ratio) or not 0.0 <= overlap_ratio < 1.0:
        raise ArchiveError('overlap_ratio must be finite and within [0, 1).')
    spacing = effective_length_m * (1.0 - overlap_ratio)
    if len(task.waypoints) != len(task.segment_types) + 1:
        raise ArchiveError('task waypoint/segment counts are inconsistent.')
    total = 0
    for index, segment_type in enumerate(task.segment_types):
        if segment_type != task.SEGMENT_SCAN:
            continue
        start = task.waypoints[index].position
        end = task.waypoints[index + 1].position
        length = math.hypot(end.x - start.x, end.y - start.y)
        if not math.isfinite(length) or length <= 1e-6:
            raise ArchiveError(f'SCAN segment {index} has invalid length.')
        span = max(0.0, length - effective_length_m)
        total += 1 if span <= 1e-9 else math.ceil(span / spacing) + 1
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


def atomic_write_bytes(path: Path, payload: bytes) -> None:
    """Write one complete file before making its destination name visible."""
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
        directory_fd = os.open(path.parent, os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    except Exception:
        temporary_path.unlink(missing_ok=True)
        raise


def atomic_write_json(path: Path, document: dict) -> None:
    """Serialize strict JSON: NaN and Infinity must never leak into labels."""
    encoded = json.dumps(
        document, ensure_ascii=False, allow_nan=False, indent=2,
        sort_keys=True).encode('utf-8') + b'\n'
    atomic_write_bytes(path, encoded)
