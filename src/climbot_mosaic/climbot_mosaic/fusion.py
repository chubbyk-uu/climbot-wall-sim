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

"""Bounded-memory tiled fusion for directly comparable wall mosaics."""

from __future__ import annotations

from collections import OrderedDict
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import tempfile
import threading
import time
from typing import Any, Callable
from uuid import uuid4
import warnings

from climbot_mosaic.mosaic_inputs import FrameKey, input_summary, MosaicInputs
from climbot_mosaic.projection import FrameProjection, project_inputs
from climbot_mosaic.stage_provenance import artifact, processed_run_inputs
from climbot_mosaic.stage_provenance import write_stage_provenance
import cv2
import numpy as np
import tifffile


class FusionError(ValueError):
    """Inputs or resources cannot produce a traceable tiled mosaic."""


@dataclass(frozen=True)
class RenderFrame:
    """Everything a worker needs for one rectified wall image."""

    key: FrameKey
    image_path: str
    homography: tuple[float, ...]
    bbox_xy_m: tuple[float, float, float, float]
    center_xy_m: tuple[float, float]
    posterior_std: tuple[float, float, float]


@dataclass(frozen=True)
class RenderGrid:
    """One shared wall raster contract for both pose variants."""

    min_x_m: float
    min_y_m: float
    max_x_m: float
    max_y_m: float
    resolution_m: float
    width_px: int
    height_px: int
    tile_size_px: int


_WORKER_INITIAL: tuple[RenderFrame, ...] = ()
_WORKER_OPTIMIZED: tuple[RenderFrame, ...] = ()
_WORKER_GRID: RenderGrid | None = None
_WORKER_INTERIOR_DISTANCE: np.ndarray | None = None
_WORKER_IMAGES: OrderedDict[str, np.ndarray] = OrderedDict()
_WORKER_IMAGE_CACHE_BYTES = 32 * 1024 * 1024
_WORKER_IMAGE_CACHE_USED_BYTES = 0
_WORKER_IMAGE_CACHE_PEAK_BYTES = 0
_WORKER_CACHE_HITS = 0
# ``ProcessPoolExecutor.map`` otherwise hands successive tiles to whichever
# process happens to become idle.  That destroys the spatial locality that
# the worker-local image cache relies on: the P2-06 baseline decoded 1,340
# source PNGs more than 57,000 times.  A modest contiguous run lets one worker
# retain the frames that overlap neighbouring tiles while preserving the
# public row-major result order.
_RENDER_TASK_CHUNK_TILES = 16
#: Memory a fusion run costs before any worker starts: the render grid, the
#: frame tables and the tiled-TIFF writer's own buffers. Measured at 1.87 GB;
#: see resolve_jobs for the fit.
FUSION_BASE_MEMORY_GB = 1.9

#: Marginal resident cost of one render worker, measured at 98 MB on the same
#: fit with a four-image cache.  The bounded 32 MiB cache below adds roughly
#: 24 MiB of decoded images at the P2 camera geometry.
FUSION_WORKER_MEMORY_MB = 120.0

_WORKER_CACHE_MISSES = 0
_WORKER_DECODE_ELAPSED_S = 0.0
UNCERTAINTY_SCALE_M = 1e-5
UNCERTAINTY_NODATA = np.iinfo(np.uint16).max
# Level 1 remains lossless while avoiding most of zlib level 6's CPU cost.
# A 512-tile P2 sample encoded 6.6x faster for 11.5% more bytes.
TIFF_DEFLATE_LEVEL = 1


def _finite(value: Any, description: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise FusionError(f'{description} must be numeric.') from error
    if not math.isfinite(result):
        raise FusionError(f'{description} must be finite.')
    return result


def feather_map(width: int, height: int, fraction: float = 0.10) -> np.ndarray:
    """Return a common linear edge feather in rectified sensor pixels."""
    if width <= 0 or height <= 0 or not 0.0 < fraction <= 0.5:
        raise FusionError('feather dimensions and fraction are invalid.')
    x = np.minimum(np.arange(width) + 1, np.arange(width, 0, -1))
    y = np.minimum(np.arange(height) + 1, np.arange(height, 0, -1))
    distance = np.minimum(y[:, None], x[None, :]).astype(np.float32)
    return np.clip(distance / (fraction * min(width, height)), 0.0, 1.0)


def interior_distance_map(width: int, height: int) -> np.ndarray:
    """
    Return pixel distance to the closest source-image edge.

    The map is a deterministic ownership priority for hard-cut mosaics: in an
    overlap, the image whose source pixel is furthest from its own edge owns
    the wall pixel.  The caller keeps the existing owner on equal distances,
    which makes ties resolve by the stable input-frame order.
    """
    if width <= 0 or height <= 0:
        raise FusionError('interior-distance dimensions are invalid.')
    x = np.minimum(np.arange(width) + 1, np.arange(width, 0, -1))
    y = np.minimum(np.arange(height) + 1, np.arange(height, 0, -1))
    return np.minimum(y[:, None], x[None, :]).astype(np.float32)


def hard_cut_ownership(owner_priority: np.ndarray, candidate_priority: np.ndarray) -> np.ndarray:
    """Choose only strictly stronger owners, retaining stable ties."""
    if owner_priority.shape != candidate_priority.shape:
        raise FusionError('hard-cut ownership maps must share a shape.')
    return candidate_priority > owner_priority


def encode_uncertainty(values_m: np.ndarray) -> np.ndarray:
    """Encode metre standard deviations as 0.01 mm uint16 with explicit nodata."""
    output = np.full(values_m.shape, UNCERTAINTY_NODATA, np.uint16)
    finite = np.isfinite(values_m)
    output[finite] = np.clip(
        np.rint(values_m[finite] / UNCERTAINTY_SCALE_M),
        0, UNCERTAINTY_NODATA - 1).astype(np.uint16)
    return output


def _correction_matrix(delta: tuple[float, float, float],
                       center: tuple[float, float]) -> np.ndarray:
    dx, dy, yaw = delta
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotation = np.array(((cosine, -sine), (sine, cosine)), np.float64)
    centre = np.asarray(center, np.float64)
    translation = centre + (dx, dy) - rotation @ centre
    return np.array(((rotation[0, 0], rotation[0, 1], translation[0]),
                     (rotation[1, 0], rotation[1, 1], translation[1]),
                     (0.0, 0.0, 1.0)), np.float64)


def _transform_footprint(homography: np.ndarray, width: int, height: int) -> np.ndarray:
    corners = np.array(((0.0, 0.0), (width - 1.0, 0.0),
                        (width - 1.0, height - 1.0), (0.0, height - 1.0)), np.float64)
    return cv2.perspectiveTransform(corners[None].astype(np.float32),
                                    homography.astype(np.float64))[0].astype(np.float64)


def read_pose_graph(path: Path, inputs: MosaicInputs,
                    projections: tuple[FrameProjection, ...]) -> tuple[
                        tuple[RenderFrame, ...], tuple[RenderFrame, ...]]:
    """Validate P2.5 output and build initial/optimized render descriptions."""
    try:
        graph = json.loads((path / 'pose_graph.json').read_text(encoding='utf-8'))
        initial_doc = json.loads((path / 'initial_poses.json').read_text(encoding='utf-8'))
        optimized_doc = json.loads((path / 'optimized_poses.json').read_text(encoding='utf-8'))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise FusionError(f'pose graph directory is incomplete or invalid: {error}') from error
    if graph.get('pose_graph_format_version') != 1:
        raise FusionError('unsupported pose graph format.')
    if graph.get('input_summary') != input_summary(inputs):
        raise FusionError('pose graph input summary does not match processed inputs.')
    initial_values = initial_doc.get('poses')
    optimized_values = optimized_doc.get('poses')
    if not isinstance(initial_values, list) or not isinstance(optimized_values, list):
        raise FusionError('pose graph pose arrays are missing.')
    initial_by_key = {(item.get('source_run_id'), item.get('frame_index')): item
                      for item in initial_values if isinstance(item, dict)}
    optimized_by_key = {(item.get('source_run_id'), item.get('frame_index')): item
                        for item in optimized_values if isinstance(item, dict)}
    if len(initial_by_key) != len(inputs.frames) or len(optimized_by_key) != len(inputs.frames):
        raise FusionError('pose graph frame keys are incomplete or duplicated.')
    initial_frames, optimized_frames = [], []
    for frame, projection in zip(inputs.frames, projections):
        key = (frame.key.source_run_id, frame.key.frame_index)
        if key not in initial_by_key or key not in optimized_by_key:
            raise FusionError('pose graph lacks an input frame key.')
        recorded = initial_by_key[key]
        pose = frame.label['camera_pose']['pose']['position']
        if (abs(_finite(recorded.get('x_m'), 'initial x') - float(pose['x'])) > 1e-9 or
                abs(_finite(recorded.get('y_m'), 'initial y') - float(pose['y'])) > 1e-9 or
                abs(_finite(recorded.get('wall_heading_rad'), 'initial heading') -
                    float(frame.label['wall_heading_rad'])) > 1e-9):
            raise FusionError('pose graph initial pose does not reproduce the input label.')
        optimized = optimized_by_key[key]
        correction = optimized.get('correction')
        posterior = optimized.get('posterior_std')
        if not isinstance(correction, dict) or not isinstance(posterior, dict):
            raise FusionError('optimized pose lacks correction or posterior uncertainty.')
        delta = tuple(_finite(correction.get(name), f'correction {name}')
                      for name in ('dx_m', 'dy_m', 'dyaw_rad'))
        std = tuple(_finite(posterior.get(name), f'posterior {name}')
                    for name in ('x_m', 'y_m', 'yaw_rad'))
        if any(value < 0.0 for value in std):
            raise FusionError('posterior standard deviations must be non-negative.')
        initial_h = np.asarray(projection.homography_image_to_wall, np.float64).reshape(3, 3)
        center = (float(pose['x']), float(pose['y']))
        optimized_h = _correction_matrix(delta, center) @ initial_h
        variants = ((initial_h, (0.0, 0.0, 0.0)), (optimized_h, std))
        targets = (initial_frames, optimized_frames)
        for (homography, uncertainty), target in zip(variants, targets):
            footprint = _transform_footprint(
                homography, inputs.camera.width, inputs.camera.height)
            target.append(RenderFrame(
                frame.key, str(frame.image_path), tuple(float(x) for x in homography.reshape(-1)),
                (float(footprint[:, 0].min()), float(footprint[:, 1].min()),
                 float(footprint[:, 0].max()), float(footprint[:, 1].max())),
                center, uncertainty))
    return tuple(initial_frames), tuple(optimized_frames)


def common_grid(initial: tuple[RenderFrame, ...], optimized: tuple[RenderFrame, ...],
                resolution_m: float, tile_size_px: int = 512) -> RenderGrid:
    """Create one exact extent and resolution shared by both pose variants."""
    if not math.isfinite(resolution_m) or resolution_m <= 0.0:
        raise FusionError('resolution must be finite and positive.')
    if tile_size_px < 16 or tile_size_px % 16:
        raise FusionError('tile size must be at least 16 and divisible by 16.')
    frames = initial + optimized
    if not frames:
        raise FusionError('at least one render frame is required.')
    min_x = min(item.bbox_xy_m[0] for item in frames)
    min_y = min(item.bbox_xy_m[1] for item in frames)
    max_x = max(item.bbox_xy_m[2] for item in frames)
    max_y = max(item.bbox_xy_m[3] for item in frames)
    # Exact decimal grid spans often land a few ulps above an integer after
    # binary division. Do not add a spurious outer column for that roundoff.
    width = int(math.ceil((max_x - min_x) / resolution_m - 1e-9))
    height = int(math.ceil((max_y - min_y) / resolution_m - 1e-9))
    if width <= 0 or height <= 0:
        raise FusionError('shared render extent is empty.')
    return RenderGrid(min_x, min_y, min_x + width * resolution_m,
                      min_y + height * resolution_m, resolution_m,
                      width, height, tile_size_px)


def _init_worker(initial: tuple[RenderFrame, ...], optimized: tuple[RenderFrame, ...],
                 grid: RenderGrid, width: int, height: int) -> None:
    global _WORKER_INITIAL, _WORKER_OPTIMIZED, _WORKER_GRID
    global _WORKER_INTERIOR_DISTANCE, _WORKER_IMAGES, _WORKER_IMAGE_CACHE_USED_BYTES
    global _WORKER_IMAGE_CACHE_PEAK_BYTES, _WORKER_CACHE_HITS, _WORKER_CACHE_MISSES
    global _WORKER_DECODE_ELAPSED_S
    _WORKER_INITIAL, _WORKER_OPTIMIZED, _WORKER_GRID = initial, optimized, grid
    _WORKER_INTERIOR_DISTANCE = interior_distance_map(width, height)
    _WORKER_IMAGES = OrderedDict()
    _WORKER_IMAGE_CACHE_USED_BYTES = 0
    _WORKER_IMAGE_CACHE_PEAK_BYTES = 0
    _WORKER_CACHE_HITS = 0
    _WORKER_CACHE_MISSES = 0
    _WORKER_DECODE_ELAPSED_S = 0.0
    cv2.setNumThreads(1)


def _image(path: str) -> np.ndarray:
    global _WORKER_CACHE_HITS, _WORKER_CACHE_MISSES, _WORKER_DECODE_ELAPSED_S
    global _WORKER_IMAGE_CACHE_USED_BYTES, _WORKER_IMAGE_CACHE_PEAK_BYTES
    if path in _WORKER_IMAGES:
        _WORKER_CACHE_HITS += 1
        value = _WORKER_IMAGES.pop(path)
        _WORKER_IMAGES[path] = value
        return value
    _WORKER_CACHE_MISSES += 1
    started = time.perf_counter()
    value = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    _WORKER_DECODE_ELAPSED_S += time.perf_counter() - started
    if value is None or value.dtype != np.uint8 or value.ndim != 2:
        raise FusionError(f'render source is not mono8: {path}.')
    image_bytes = int(value.nbytes)
    if image_bytes > _WORKER_IMAGE_CACHE_BYTES:
        # The worker must still render an unusually large frame correctly; it
        # simply cannot retain it without violating the cache contract.
        return value
    _WORKER_IMAGES[path] = value
    _WORKER_IMAGE_CACHE_USED_BYTES += image_bytes
    while _WORKER_IMAGE_CACHE_USED_BYTES > _WORKER_IMAGE_CACHE_BYTES:
        _, evicted = _WORKER_IMAGES.popitem(last=False)
        _WORKER_IMAGE_CACHE_USED_BYTES -= int(evicted.nbytes)
    _WORKER_IMAGE_CACHE_PEAK_BYTES = max(
        _WORKER_IMAGE_CACHE_PEAK_BYTES, _WORKER_IMAGE_CACHE_USED_BYTES)
    return value


def _tile_transform(grid: RenderGrid, tile_row: int, tile_column: int) -> np.ndarray:
    scale = 1.0 / grid.resolution_m
    return np.array(((scale, 0.0, -grid.min_x_m * scale - 0.5 - tile_column),
                     (0.0, -scale, grid.max_y_m * scale - 0.5 - tile_row),
                     (0.0, 0.0, 1.0)), np.float64)


def _hard_cut(frames: tuple[RenderFrame, ...], tile_row: int, tile_column: int,
              candidates: tuple[int, ...], auxiliary: bool,
              quality: bool = False, coverage_output: bool = False) -> tuple[np.ndarray, ...]:
    if _WORKER_GRID is None or _WORKER_INTERIOR_DISTANCE is None:
        raise FusionError('render worker is not initialized.')
    size = _WORKER_GRID.tile_size_px
    output = np.zeros((size, size), np.uint8)
    owner_priority = np.zeros((size, size), np.float32)
    # Zero means no source image.  A positive value is the stable frame index
    # plus one, retained only long enough to extract the hard-cut boundaries.
    # Persisting every owner pixel would add a multi-gigabyte raster; the sparse
    # seam adjacencies are the actual input to the post-mosaic diagnostics.
    owner = np.zeros((size, size), np.uint16)
    coverage = np.zeros((size, size), np.uint16) \
        if auxiliary or quality or coverage_output else None
    uncertainty = np.full((size, size), np.nan, np.float32) if auxiliary else None
    value_sum = np.zeros((size, size), np.float32) if quality else None
    value_square_sum = np.zeros((size, size), np.float32) if quality else None
    transform = _tile_transform(_WORKER_GRID, tile_row, tile_column)
    columns = _WORKER_GRID.min_x_m + (
        tile_column + np.arange(size, dtype=np.float32) + 0.5) * _WORKER_GRID.resolution_m
    rows = _WORKER_GRID.max_y_m - (
        tile_row + np.arange(size, dtype=np.float32) + 0.5) * _WORKER_GRID.resolution_m
    for index in candidates:
        frame = frames[index]
        matrix = transform @ np.asarray(frame.homography, np.float64).reshape(3, 3)
        source_mask = np.ones(_WORKER_INTERIOR_DISTANCE.shape, np.uint8)
        mask = cv2.warpPerspective(
            source_mask, matrix, (size, size), flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT, borderValue=0).astype(bool)
        if not np.any(mask):
            continue
        warped = cv2.warpPerspective(
            _image(frame.image_path), matrix, (size, size), flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT, borderValue=0.0)
        priority = cv2.warpPerspective(
            _WORKER_INTERIOR_DISTANCE, matrix, (size, size), flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT, borderValue=0.0)
        take = mask & hard_cut_ownership(owner_priority, priority)
        output[take] = warped[take]
        owner_priority[take] = priority[take]
        owner[take] = index + 1
        if coverage is not None:
            coverage += mask.astype(np.uint16)
        if value_sum is not None and value_square_sum is not None:
            values = warped.astype(np.float32)
            value_sum[mask] += values[mask]
            value_square_sum[mask] += values[mask] ** 2
        if auxiliary and uncertainty is not None:
            sx, sy, syaw = frame.posterior_std
            radius_squared = ((columns[None, :] - frame.center_xy_m[0]) ** 2 +
                              (rows[:, None] - frame.center_xy_m[1]) ** 2)
            sigma = np.sqrt(sx * sx + sy * sy + syaw * syaw * radius_squared)
            uncertainty[take] = sigma[take]
    valid = owner_priority > 0.0
    products: list[np.ndarray] = [output, owner]
    if auxiliary and coverage is not None and uncertainty is not None:
        uncertainty[~valid] = np.nan
        products.extend((coverage, uncertainty))
    elif coverage_output and coverage is not None:
        products.append(coverage)
    if quality and coverage is not None and value_sum is not None and value_square_sum is not None:
        disagreement = np.full((size, size), np.nan, np.float32)
        overlap = coverage >= 2
        mean = np.zeros((size, size), np.float32)
        mean[overlap] = value_sum[overlap] / coverage[overlap]
        variance = np.zeros((size, size), np.float32)
        variance[overlap] = np.maximum(
            0.0, value_square_sum[overlap] / coverage[overlap] - mean[overlap] ** 2)
        disagreement[overlap] = np.sqrt(variance[overlap])
        products.append(disagreement)
    return tuple(products)


def _render_task(task: tuple[str, int, int, tuple[int, ...], tuple[int, ...]]):
    hits_before, misses_before = _WORKER_CACHE_HITS, _WORKER_CACHE_MISSES
    decode_before = _WORKER_DECODE_ELAPSED_S
    mode, row, column, initial_candidates, optimized_candidates = task
    if mode == 'initial':
        products = _hard_cut(
            _WORKER_INITIAL, row, column, initial_candidates, False, True, True)
    elif mode == 'optimized':
        products = _hard_cut(_WORKER_OPTIMIZED, row, column, optimized_candidates, True, True)
    else:
        raise FusionError(f'unknown render mode: {mode}.')
    return (row, column, *products,
            _WORKER_CACHE_HITS - hits_before, _WORKER_CACHE_MISSES - misses_before,
            _WORKER_DECODE_ELAPSED_S - decode_before, _WORKER_IMAGE_CACHE_PEAK_BYTES)


def _tasks(mode: str, grid: RenderGrid, initial: tuple[RenderFrame, ...],
           optimized: tuple[RenderFrame, ...]):
    """Yield row-major tiles with exact bbox candidates in stable frame order."""
    size = grid.tile_size_px
    tile_span = size * grid.resolution_m
    tile_rows = math.ceil(grid.height_px / size)
    tile_columns = math.ceil(grid.width_px / size)

    def candidate_index(frames: tuple[RenderFrame, ...]):
        result: dict[tuple[int, int], list[int]] = {}
        for index, frame in enumerate(frames):
            x0, y0, x1, y1 = frame.bbox_xy_m
            # Add one tile on each side before applying the exact predicate.
            # The margin absorbs bbox values that land within a few ulps of a
            # tile boundary; it cannot add false candidates because every
            # prospective tile is checked below.
            first_column = max(0, math.floor((x0 - grid.min_x_m) / tile_span) - 1)
            last_column = min(
                tile_columns - 1, math.floor((x1 - grid.min_x_m) / tile_span) + 1)
            first_row = max(0, math.floor((grid.max_y_m - y1) / tile_span) - 1)
            last_row = min(
                tile_rows - 1, math.floor((grid.max_y_m - y0) / tile_span) + 1)
            for tile_row in range(first_row, last_row + 1):
                row = tile_row * size
                top = grid.max_y_m - row * grid.resolution_m
                bottom = grid.max_y_m - min(
                    row + size, grid.height_px) * grid.resolution_m
                for tile_column in range(first_column, last_column + 1):
                    column = tile_column * size
                    left = grid.min_x_m + column * grid.resolution_m
                    right = grid.min_x_m + min(
                        column + size, grid.width_px) * grid.resolution_m
                    if x1 > left and x0 < right and y1 > bottom and y0 < top:
                        result.setdefault((row, column), []).append(index)
        return result

    initial_index = candidate_index(initial)
    optimized_index = candidate_index(optimized)
    for row in range(0, grid.height_px, size):
        for column in range(0, grid.width_px, size):
            yield (mode, row, column,
                   tuple(initial_index.get((row, column), ())),
                   tuple(optimized_index.get((row, column), ())))


def _tile_count(grid: RenderGrid) -> int:
    rows = math.ceil(grid.height_px / grid.tile_size_px)
    columns = math.ceil(grid.width_px / grid.tile_size_px)
    return rows * columns


def _tile_seam_adjacencies(owner: np.ndarray, row: int, column: int,
                           grid: RenderGrid, previous_right: np.ndarray | None,
                           previous_bottom: np.ndarray | None) -> tuple[
                               np.ndarray, np.ndarray, np.ndarray]:
    """
    Return source-owner transitions inside and immediately before one tile.

    ``axis == 0`` denotes an adjacency from ``(row, column)`` to its right
    neighbour; ``axis == 1`` denotes one to its lower neighbour.  The supplied
    edge vectors make the result independent of tiled writer boundaries.
    """
    height = min(grid.tile_size_px, grid.height_px - row)
    width = min(grid.tile_size_px, grid.width_px - column)
    cropped = owner[:height, :width]
    if cropped.dtype != np.uint16 or cropped.shape != (height, width):
        raise FusionError('hard-cut owner tile has an unexpected shape.')
    rows: list[np.ndarray] = []
    columns: list[np.ndarray] = []
    axes: list[np.ndarray] = []

    if width > 1:
        changed = ((cropped[:, :-1] != cropped[:, 1:]) &
                   (cropped[:, :-1] != 0) & (cropped[:, 1:] != 0))
        local_rows, local_columns = np.nonzero(changed)
        if local_rows.size:
            rows.append((row + local_rows).astype(np.uint32))
            columns.append((column + local_columns).astype(np.uint32))
            axes.append(np.zeros(local_rows.size, np.uint8))
    if height > 1:
        changed = ((cropped[:-1, :] != cropped[1:, :]) &
                   (cropped[:-1, :] != 0) & (cropped[1:, :] != 0))
        local_rows, local_columns = np.nonzero(changed)
        if local_rows.size:
            rows.append((row + local_rows).astype(np.uint32))
            columns.append((column + local_columns).astype(np.uint32))
            axes.append(np.ones(local_rows.size, np.uint8))
    if previous_right is not None:
        if previous_right.shape != (height,):
            raise FusionError('previous hard-cut tile edge has an unexpected shape.')
        changed = ((previous_right != cropped[:, 0]) & (previous_right != 0) &
                   (cropped[:, 0] != 0))
        local_rows = np.flatnonzero(changed)
        if local_rows.size:
            rows.append((row + local_rows).astype(np.uint32))
            columns.append(np.full(local_rows.size, column - 1, np.uint32))
            axes.append(np.zeros(local_rows.size, np.uint8))
    if previous_bottom is not None:
        if previous_bottom.shape != (width,):
            raise FusionError('previous hard-cut tile edge has an unexpected shape.')
        changed = ((previous_bottom != cropped[0, :]) & (previous_bottom != 0) &
                   (cropped[0, :] != 0))
        local_columns = np.flatnonzero(changed)
        if local_columns.size:
            rows.append(np.full(local_columns.size, row - 1, np.uint32))
            columns.append((column + local_columns).astype(np.uint32))
            axes.append(np.ones(local_columns.size, np.uint8))
    if not rows:
        return (np.empty(0, np.uint32), np.empty(0, np.uint32), np.empty(0, np.uint8))
    return (np.concatenate(rows), np.concatenate(columns), np.concatenate(axes))


def _write_seam_adjacencies(path: Path, parts: list[tuple[np.ndarray, np.ndarray, np.ndarray]],
                            grid: RenderGrid) -> int:
    """Publish the sparse source-owner boundaries required by seam diagnostics."""
    if parts:
        rows = np.concatenate([part[0] for part in parts])
        columns = np.concatenate([part[1] for part in parts])
        axes = np.concatenate([part[2] for part in parts])
    else:
        rows, columns, axes = (np.empty(0, np.uint32), np.empty(0, np.uint32),
                               np.empty(0, np.uint8))
    np.savez_compressed(
        path, seam_format_version=np.asarray(1, np.uint8), row_px=rows,
        column_px=columns, axis=axes,
        grid_width_px=np.asarray(grid.width_px, np.uint32),
        grid_height_px=np.asarray(grid.height_px, np.uint32))
    return int(rows.size)


def _write_raw_tile(file_descriptor: int, values: np.ndarray, grid: RenderGrid,
                    row: int, column: int) -> None:
    """Store one padded tile at its deterministic tile-order offset."""
    size = grid.tile_size_px
    columns = math.ceil(grid.width_px / size)
    tile_index = (row // size) * columns + column // size
    data = np.ascontiguousarray(values).tobytes()
    expected = size * size * values.dtype.itemsize
    if len(data) != expected:
        raise FusionError('raw auxiliary tile has an unexpected shape.')
    written = os.pwrite(file_descriptor, data, tile_index * expected)
    if written != expected:
        raise FusionError('failed to write a complete raw auxiliary tile.')


def _read_raw_tile(file_descriptor: int, dtype: np.dtype, grid: RenderGrid,
                   row: int, column: int) -> np.ndarray:
    """Read back one padded tile written by _write_raw_tile."""
    size = grid.tile_size_px
    columns = math.ceil(grid.width_px / size)
    tile_index = (row // size) * columns + column // size
    expected = size * size * dtype.itemsize
    data = os.pread(file_descriptor, expected, tile_index * expected)
    if len(data) != expected:
        raise FusionError('raw auxiliary tile cache is truncated.')
    return np.frombuffer(data, dtype=dtype).reshape(size, size)


def _tiles_from_raw(path: Path, dtype: np.dtype, grid: RenderGrid):
    size = grid.tile_size_px
    tile_bytes = size * size * dtype.itemsize
    with path.open('rb', buffering=0) as stream:
        for _ in range(_tile_count(grid)):
            data = stream.read(tile_bytes)
            if len(data) != tile_bytes:
                raise FusionError('raw auxiliary tile cache is truncated.')
            yield np.frombuffer(data, dtype=dtype).reshape(size, size)


def _description(grid: RenderGrid, kind: str) -> str:
    return json.dumps({'frame': 'wall', 'kind': kind,
                       'min_x_m': grid.min_x_m, 'min_y_m': grid.min_y_m,
                       'max_x_m': grid.max_x_m, 'max_y_m': grid.max_y_m,
                       'resolution_m_per_pixel': grid.resolution_m},
                      allow_nan=False, sort_keys=True)


def _write_image_pass(path: Path, mode: str, grid: RenderGrid,
                      initial: tuple[RenderFrame, ...], optimized: tuple[RenderFrame, ...],
                      pool: ProcessPoolExecutor,
                      seam_path: Path,
                      coverage_fd: int | None = None,
                      coverage_only_fd: int | None = None,
                      uncertainty_fd: int | None = None,
                      image_raw_fd: int | None = None,
                      difference_fd: int | None = None,
                      previous_raw_fd: int | None = None) -> dict[str, Any]:
    started = time.perf_counter()
    task_values = _tasks(mode, grid, initial, optimized)
    results = pool.map(_render_task, task_values, chunksize=_RENDER_TASK_CHUNK_TILES)
    quality_histogram = np.zeros(4096, np.int64)
    quality_sum = 0.0
    quality_count = 0
    cache_hits = 0
    cache_misses = 0
    decode_elapsed_s = 0.0
    cache_peak_bytes = 0
    seam_parts: list[tuple[np.ndarray, np.ndarray, np.ndarray]] = []
    previous_right: np.ndarray | None = None
    previous_bottoms: dict[int, np.ndarray] = {}
    result_wait_elapsed = 0.0
    seam_elapsed = 0.0
    raw_write_elapsed = 0.0
    quality_elapsed = 0.0
    tiff_elapsed = 0.0

    def image_tiles():
        nonlocal quality_sum, quality_count, cache_hits, cache_misses, decode_elapsed_s
        nonlocal cache_peak_bytes, previous_right
        nonlocal result_wait_elapsed, seam_elapsed, raw_write_elapsed, quality_elapsed
        nonlocal tiff_elapsed
        iterator = iter(results)
        while True:
            result_started = time.perf_counter()
            try:
                result = next(iterator)
            except StopIteration:
                break
            result_wait_elapsed += time.perf_counter() - result_started
            cache_hits += int(result[-4])
            cache_misses += int(result[-3])
            decode_elapsed_s += float(result[-2])
            cache_peak_bytes = max(cache_peak_bytes, int(result[-1]))
            row, column, image, owner = result[:4]
            if column == 0:
                previous_right = None
            seam_started = time.perf_counter()
            seams = _tile_seam_adjacencies(
                owner, row, column, grid, previous_right, previous_bottoms.get(column))
            if seams[0].size:
                seam_parts.append(seams)
            height = min(grid.tile_size_px, grid.height_px - row)
            width = min(grid.tile_size_px, grid.width_px - column)
            previous_right = owner[:height, width - 1].copy()
            previous_bottoms[column] = owner[height - 1, :width].copy()
            seam_elapsed += time.perf_counter() - seam_started
            # The difference raster used to be a third full render of both
            # variants. Both tiles already exist by the time the second
            # pass runs: the first pass keeps its own, and this one subtracts
            # against it, so the same bytes come out of one read instead of a
            # whole render pass.
            raw_started = time.perf_counter()
            if image_raw_fd is not None:
                _write_raw_tile(image_raw_fd, image, grid, row, column)
            if difference_fd is not None and previous_raw_fd is not None:
                previous = _read_raw_tile(
                    previous_raw_fd, np.dtype(np.uint8), grid, row, column)
                _write_raw_tile(
                    difference_fd, cv2.absdiff(previous, image), grid, row, column)
            if coverage_only_fd is not None:
                coverage = result[4]
                _write_raw_tile(coverage_only_fd, coverage, grid, row, column)
                quality = result[5]
            elif coverage_fd is not None and uncertainty_fd is not None:
                coverage, uncertainty = result[4:6]
                _write_raw_tile(coverage_fd, coverage, grid, row, column)
                _write_raw_tile(
                    uncertainty_fd, encode_uncertainty(uncertainty), grid, row, column)
                quality = result[6]
            elif mode == 'initial':
                quality = result[4]
            else:
                quality = None
            raw_write_elapsed += time.perf_counter() - raw_started
            quality_started = time.perf_counter()
            if isinstance(quality, dict):
                histogram = np.asarray(quality.get('histogram'))
                tile_sum = quality.get('sum')
                tile_count = quality.get('count')
                if (histogram.shape != (4096,) or histogram.dtype != np.uint64 or
                        not isinstance(tile_count, int) or tile_count < 0 or
                        not isinstance(tile_sum, float) or not math.isfinite(tile_sum)):
                    raise FusionError('CUDA quality reduction has an invalid shape or value.')
                quality_histogram[:] += histogram.astype(np.int64)
                quality_sum += tile_sum
                quality_count += tile_count
            elif quality is not None:
                finite = quality[np.isfinite(quality)]
                if finite.size:
                    quality_sum += float(finite.sum(dtype=np.float64))
                    quality_count += int(finite.size)
                    bins = np.clip(np.rint(finite * (4095.0 / 255.0)), 0, 4095)
                    quality_histogram[:] += np.bincount(
                        bins.astype(np.int32), minlength=4096)
            quality_elapsed += time.perf_counter() - quality_started
            tiff_started = time.perf_counter()
            yield image
            tiff_elapsed += time.perf_counter() - tiff_started

    with tifffile.TiffWriter(path, bigtiff=True) as writer:
        writer.write(data=image_tiles(), shape=(grid.height_px, grid.width_px),
                     dtype=np.uint8, tile=(grid.tile_size_px, grid.tile_size_px),
                     compression='deflate', compressionargs={'level': TIFF_DEFLATE_LEVEL},
                     predictor=True,
                     description=_description(grid, mode))
    seam_count = _write_seam_adjacencies(seam_path, seam_parts, grid)
    quality_result = None
    if quality_count:
        target = int(math.ceil(0.95 * quality_count))
        p95_bin = int(np.searchsorted(np.cumsum(quality_histogram), target))
        quality_result = {
            'overlap_pixel_count': quality_count,
            'gray_std_mean': quality_sum / quality_count,
            'gray_std_p95': p95_bin * 255.0 / 4095.0,
        }
    total_accesses = cache_hits + cache_misses
    return {
        'elapsed_s': time.perf_counter() - started,
        'image_cache': {
            'hits': cache_hits,
            'misses': cache_misses,
            'hit_ratio': cache_hits / total_accesses if total_accesses else None,
            'capacity_bytes_per_worker': _WORKER_IMAGE_CACHE_BYTES,
            'peak_bytes_in_one_worker': cache_peak_bytes,
            # This is a sum over workers, deliberately not wall time. It
            # quantifies decoding work even while workers run concurrently.
            'aggregate_png_decode_cpu_s': decode_elapsed_s,
            'task_chunk_tiles': _RENDER_TASK_CHUNK_TILES,
        },
        'quality': quality_result,
        'seam_adjacency_count': seam_count,
        'timing': {
            'result_wait_s': result_wait_elapsed,
            'seam_extraction_s': seam_elapsed,
            'raw_auxiliary_write_s': raw_write_elapsed,
            'quality_reduction_s': quality_elapsed,
            'tiff_writer_s': tiff_elapsed,
        },
    }


def _write_raw_tiles(path: Path, raw_path: Path, dtype: np.dtype, grid: RenderGrid,
                     kind: str, predictor: bool) -> None:
    with tifffile.TiffWriter(path, bigtiff=True) as writer:
        writer.write(data=_tiles_from_raw(raw_path, dtype, grid),
                     shape=(grid.height_px, grid.width_px), dtype=dtype,
                     tile=(grid.tile_size_px, grid.tile_size_px),
                     compression='deflate', compressionargs={'level': TIFF_DEFLATE_LEVEL},
                     predictor=predictor,
                     description=_description(grid, kind))


def _render_preview(initial: tuple[RenderFrame, ...], optimized: tuple[RenderFrame, ...],
                    grid: RenderGrid, camera_width: int, camera_height: int,
                    jobs: int, max_side: int) -> tuple[np.ndarray, np.ndarray]:
    coarse_resolution = grid.resolution_m * max(
        1.0, max(grid.width_px, grid.height_px) / float(max_side))
    coarse = common_grid(initial, optimized, coarse_resolution, 256)
    canvases = []
    for mode in ('initial', 'optimized'):
        canvas = np.zeros((coarse.height_px, coarse.width_px), np.uint8)
        with ProcessPoolExecutor(
                max_workers=jobs, initializer=_init_worker,
                initargs=(initial, optimized, coarse, camera_width, camera_height)) as pool:
            for result in pool.map(
                    _render_task, _tasks(mode, coarse, initial, optimized), chunksize=1):
                row, column, tile = result[:3]
                height = min(coarse.tile_size_px, coarse.height_px - row)
                width = min(coarse.tile_size_px, coarse.width_px - column)
                canvas[row:row + height, column:column + width] = tile[:height, :width]
        canvases.append(canvas)
    return canvases[0], canvases[1]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _process_tree_pss_bytes(root_pid: int) -> int:
    """Read aggregate Linux PSS without double-counting shared worker pages."""
    pending = [root_pid]
    visited: set[int] = set()
    total_kib = 0
    while pending:
        pid = pending.pop()
        if pid in visited:
            continue
        visited.add(pid)
        try:
            rollup = Path(f'/proc/{pid}/smaps_rollup').read_text(encoding='ascii')
            pss_line = next(line for line in rollup.splitlines() if line.startswith('Pss:'))
            total_kib += int(pss_line.split()[1])
            children = Path(f'/proc/{pid}/task/{pid}/children').read_text(
                encoding='ascii').split()
            pending.extend(int(value) for value in children)
        except (OSError, StopIteration, IndexError, ValueError):
            continue
    return total_kib * 1024


class _ProcessTreePssMonitor:
    """Sample aggregate process-tree PSS without adding a deployment dependency."""

    def __init__(self, interval_s: float = 1.0) -> None:
        self.interval_s = interval_s
        self.peak_bytes = 0
        self._finished = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _run(self) -> None:
        while not self._finished.is_set():
            self.peak_bytes = max(self.peak_bytes, _process_tree_pss_bytes(os.getpid()))
            self._finished.wait(self.interval_s)
        self.peak_bytes = max(self.peak_bytes, _process_tree_pss_bytes(os.getpid()))

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._finished.set()
        if self._thread.is_alive():
            self._thread.join()


def build_wall_mosaic(output_dir: Path, work_dir: Path, inputs: MosaicInputs,
                      pose_graph_dir: Path, resolution_m: float,
                      jobs: int, memory_budget_gb: float,
                      preview_max_side_px: int = 4096,
                      execution_backend: dict[str, Any] | None = None,
                      render_pool_factory: Callable[..., Any] | None = None) -> dict[str, Any]:
    """Atomically build comparable BigTIFF products with bounded worker memory."""
    if not output_dir.is_absolute() or output_dir.exists() or not output_dir.parent.is_dir():
        raise FusionError('output directory must be absolute, new and have an existing parent.')
    if not work_dir.is_absolute():
        raise FusionError('work directory must be absolute.')
    if not math.isfinite(memory_budget_gb) or memory_budget_gb <= 0.0:
        raise FusionError('memory budget must be finite and positive.')
    if jobs <= 0:
        raise FusionError('jobs must be positive.')
    if (isinstance(preview_max_side_px, bool) or
            not isinstance(preview_max_side_px, int) or preview_max_side_px < 512):
        raise FusionError('preview_max_side_px must be an integer of at least 512.')
    started = time.perf_counter()
    execution_started_utc = datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')
    projections = project_inputs(inputs)
    initial, optimized = read_pose_graph(pose_graph_dir, inputs, projections)
    if len(initial) > np.iinfo(np.uint16).max:
        raise FusionError('hard-cut seam ownership supports at most 65535 input frames.')
    grid = common_grid(initial, optimized, resolution_m)
    work_dir.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(
        prefix=f'.{output_dir.name}.tmp-{uuid4().hex}-', dir=output_dir.parent))
    cache = Path(tempfile.mkdtemp(prefix='fusion-', dir=work_dir))
    coverage_fd = None
    pose_coverage_fd = None
    uncertainty_fd = None
    pose_only_fd = None
    difference_fd = None
    resource_monitor = _ProcessTreePssMonitor()
    resource_monitor.start()
    try:
        coverage_path = cache / 'coverage.dat'
        pose_coverage_path = cache / 'pose_coverage.dat'
        uncertainty_path = cache / 'uncertainty.dat'
        pose_only_path = cache / 'pose_only.dat'
        difference_path = cache / 'difference.dat'
        tiles = _tile_count(grid) * grid.tile_size_px ** 2
        raw_bytes = tiles * np.dtype(np.uint16).itemsize
        image_bytes = tiles * np.dtype(np.uint8).itemsize
        coverage_fd = os.open(coverage_path, os.O_RDWR | os.O_CREAT | os.O_TRUNC, 0o600)
        pose_coverage_fd = os.open(
            pose_coverage_path, os.O_RDWR | os.O_CREAT | os.O_TRUNC, 0o600)
        uncertainty_fd = os.open(
            uncertainty_path, os.O_RDWR | os.O_CREAT | os.O_TRUNC, 0o600)
        pose_only_fd = os.open(pose_only_path, os.O_RDWR | os.O_CREAT | os.O_TRUNC, 0o600)
        difference_fd = os.open(difference_path, os.O_RDWR | os.O_CREAT | os.O_TRUNC, 0o600)
        os.ftruncate(coverage_fd, raw_bytes)
        os.ftruncate(pose_coverage_fd, raw_bytes)
        os.ftruncate(uncertainty_fd, raw_bytes)
        os.ftruncate(pose_only_fd, image_bytes)
        os.ftruncate(difference_fd, image_bytes)
        # Keep the pool alive over both full-resolution passes. Combined with
        # spatial task chunks this preserves decoded PNGs in the worker-local
        # cache instead of recreating a cold cache for every pass.
        pool_context = (ProcessPoolExecutor(
            max_workers=jobs, initializer=_init_worker,
            initargs=(initial, optimized, grid, inputs.camera.width, inputs.camera.height))
            if render_pool_factory is None else render_pool_factory(
                initial, optimized, grid, inputs.camera.width, inputs.camera.height))
        renderer_summary = None
        with pool_context as pool:
            initial_pass = _write_image_pass(
                temporary / 'mosaic_pose_only.tif', 'initial', grid,
                initial, optimized, pool, temporary / 'seams_pose_only.npz',
                coverage_only_fd=pose_coverage_fd,
                image_raw_fd=pose_only_fd)
            optimized_pass = _write_image_pass(
                temporary / 'mosaic_optimized.tif', 'optimized', grid,
                initial, optimized, pool, temporary / 'seams_optimized.npz',
                coverage_fd=coverage_fd, uncertainty_fd=uncertainty_fd,
                difference_fd=difference_fd, previous_raw_fd=pose_only_fd)
            if hasattr(pool, 'summary'):
                renderer_summary = pool.summary()
        os.close(coverage_fd)
        coverage_fd = None
        os.close(pose_coverage_fd)
        pose_coverage_fd = None
        os.close(uncertainty_fd)
        uncertainty_fd = None
        os.close(pose_only_fd)
        pose_only_fd = None
        os.close(difference_fd)
        difference_fd = None
        difference_started = time.perf_counter()
        _write_raw_tiles(temporary / 'mosaic_difference.tif', difference_path,
                         np.dtype(np.uint8), grid, 'difference', True)
        difference_pass = {
            'elapsed_s': time.perf_counter() - difference_started,
            'image_cache': None,
            'source': ('absolute difference of the two rendered variants, taken '
                       'tile by tile inside the optimized pass'),
        }
        _write_raw_tiles(temporary / 'coverage_count.tif', coverage_path,
                         np.dtype(np.uint16), grid, 'coverage_count', True)
        _write_raw_tiles(temporary / 'coverage_pose_only_count.tif', pose_coverage_path,
                         np.dtype(np.uint16), grid, 'coverage_pose_only_count', True)
        _write_raw_tiles(temporary / 'uncertainty.tif', uncertainty_path,
                         np.dtype(np.uint16), grid, 'position_std_uint16', True)
        pose_preview, optimized_preview = _render_preview(
            initial, optimized, grid, inputs.camera.width, inputs.camera.height, jobs,
            preview_max_side_px)
        difference = cv2.absdiff(pose_preview, optimized_preview)
        comparison = cv2.hconcat((pose_preview, optimized_preview, difference))
        panel_width = pose_preview.shape[1]
        for panel, label in enumerate(('POSE ONLY', 'OPTIMIZED', 'ABS DIFFERENCE')):
            cv2.putText(comparison, label, (panel * panel_width + 18, 38),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, 255, 2, cv2.LINE_AA)
        if not cv2.imwrite(
                str(temporary / 'mosaic_preview.jpg'), optimized_preview,
                (cv2.IMWRITE_JPEG_QUALITY, 92)):
            raise FusionError('failed to write optimized preview.')
        if not cv2.imwrite(
                str(temporary / 'mosaic_comparison.jpg'), comparison,
                (cv2.IMWRITE_JPEG_QUALITY, 92)):
            raise FusionError('failed to write comparison preview.')
        outputs = {}
        for path in sorted(temporary.iterdir()):
            if path.is_file():
                outputs[path.name] = {'bytes': path.stat().st_size, 'sha256': _sha256(path)}
        resource_monitor.stop()
        manifest = {
            'mosaic_format_version': 3,
            'input_summary': input_summary(inputs),
            'grid': {'frame': 'wall', 'min_x_m': grid.min_x_m, 'min_y_m': grid.min_y_m,
                     'max_x_m': grid.max_x_m, 'max_y_m': grid.max_y_m,
                     'resolution_m_per_pixel': grid.resolution_m,
                     'width_px': grid.width_px, 'height_px': grid.height_px,
                     'tile_size_px': grid.tile_size_px},
            'fusion': {'method': ('single-image hard cut by maximum interior distance; '
                                  'stable input-frame order resolves ties'),
                       'tiff_compression': {
                           'codec': 'deflate', 'level': TIFF_DEFLATE_LEVEL,
                           'lossless': True,
                       },
                       'same_grid_and_pixels_except_pose': True,
                       'jobs': jobs, 'memory_budget_gb': memory_budget_gb,
                       'preview_max_side_px': preview_max_side_px,
                       'preview_panel_width_px': int(pose_preview.shape[1]),
                       'preview_panel_height_px': int(pose_preview.shape[0]),
                       'comparison_width_px': int(comparison.shape[1]),
                       'comparison_height_px': int(comparison.shape[0]),
                       'elapsed_s': time.perf_counter() - started,
                       'peak_process_tree_pss_bytes': resource_monitor.peak_bytes,
                       'pss_sample_interval_s': resource_monitor.interval_s,
                       'passes': {
                           'pose_only': {key: value for key, value in initial_pass.items()
                                         if key != 'quality'},
                           'optimized': {key: value for key, value in optimized_pass.items()
                                         if key != 'quality'},
                           'difference': {key: value for key, value in difference_pass.items()
                                          if key != 'quality'},
                       },
                       **({'cuda_renderer': renderer_summary}
                          if renderer_summary is not None else {})},
            'seam_adjacencies': {
                'format_version': 1,
                'definition': ('adjacent covered pixels whose selected hard-cut source frame '
                               'differs; axis 0 is rightward and axis 1 is downward'),
                'pose_only_file': 'seams_pose_only.npz',
                'optimized_file': 'seams_optimized.npz',
                'pose_only_count': initial_pass['seam_adjacency_count'],
                'optimized_count': optimized_pass['seam_adjacency_count'],
            },
            'uncertainty_encoding': {
                'dtype': 'uint16', 'scale_m_per_count': UNCERTAINTY_SCALE_M,
                'nodata': int(UNCERTAINTY_NODATA),
            },
            'quality': {
                'metric': 'grayscale standard deviation across source images in >=2-image overlap',
                'pose_only': initial_pass['quality'],
                'optimized': optimized_pass['quality'],
            },
            'outputs': outputs,
        }
        if execution_backend is not None:
            if not isinstance(execution_backend, dict):
                raise FusionError('execution backend provenance must be an object.')
            # This record comes from the clean worker which loaded OpenCV. It
            # has build hashes, never a host-private installation path.
            execution = dict(execution_backend)
            attempts = execution.get('attempts', [])
            if (not isinstance(attempts, list) or
                    not all(isinstance(item, dict) for item in attempts)):
                raise FusionError('execution backend attempts must be objects.')
            effective_backend = execution.get('effective')
            if effective_backend not in ('cpu', 'cuda'):
                raise FusionError('execution backend has an invalid effective value.')
            execution['attempts'] = [*attempts, {
                'backend': effective_backend, 'outcome': 'completed',
                'started_utc': execution_started_utc,
                'elapsed_s': time.perf_counter() - started,
            }]
            manifest['execution'] = execution
        (temporary / 'mosaic_manifest.json').write_text(json.dumps(
            manifest, allow_nan=False, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
            encoding='utf-8')
        # The manifest already carries a digest for every raster it published,
        # so hashing it is enough to pin gigabytes of output without reading
        # them a second time.
        write_stage_provenance(
            temporary, 'wall_mosaic',
            {'resolution_m_per_pixel': resolution_m, 'jobs': jobs,
             'memory_budget_gb': memory_budget_gb,
             'preview_max_side_px': preview_max_side_px,
             'image_cache_bytes_per_worker': _WORKER_IMAGE_CACHE_BYTES,
             'render_task_chunk_tiles': _RENDER_TASK_CHUNK_TILES,
             'tiff_deflate_level': TIFF_DEFLATE_LEVEL,
             **({'backend': {
                 key: manifest['execution'][key]
                 for key in ('requested', 'effective', 'fallback')
                 if key in manifest['execution']
             }} if execution_backend is not None else {})},
            {**processed_run_inputs(manifest['input_summary']),
             'pose_graph': artifact(pose_graph_dir / 'pose_graph.json'),
             'optimized_poses': artifact(pose_graph_dir / 'optimized_poses.json')},
            ('mosaic_manifest.json',))
        temporary.replace(output_dir)
        return manifest
    # Cleanup is also required for Ctrl-C/SystemExit; both inherit directly
    # from BaseException and otherwise leave a multi-gigabyte staging tree.
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    finally:
        resource_monitor.stop()
        if coverage_fd is not None:
            os.close(coverage_fd)
        if pose_coverage_fd is not None:
            os.close(pose_coverage_fd)
        if uncertainty_fd is not None:
            os.close(uncertainty_fd)
        if pose_only_fd is not None:
            os.close(pose_only_fd)
        if difference_fd is not None:
            os.close(difference_fd)
        shutil.rmtree(cache, ignore_errors=True)


def resolve_jobs(value: int | None, memory_budget_gb: float) -> int:
    """
    Resolve bounded process parallelism from CPU count and the memory budget.

    The budget is a soft one: it decides how many workers to start, not how
    much memory the process may use.  Nothing here can hold a run to a number
    smaller than the run's own fixed cost, and refusing on that basis would be
    worse than useless -- FUSION_BASE_MEMORY_GB was fitted on the largest
    mosaic this repository builds, so a smaller job pays far less than it and
    would be turned away for a cost it never incurs.  What can be done
    honestly is to say so, which is what the warning below is for.

    The budget has to pay for the run before it pays for any worker.  Dividing
    all of it by the per-worker cost ignored that and produced 42 workers for
    a 4 GB budget, which a hard cap of 8 then quietly corrected; the cap was
    doing the memory model's job with a number that fit one machine.
    Measured on the P2-06 joint mosaic (1340 frames, 38226 x 29079 px) by
    fitting peak process-tree PSS over 4, 8, 14 and 20 workers: 1.87 GB fixed
    plus 98 MB each with the original four-image cache.  The current cache is
    explicitly capped at 32 MiB per worker, so the scheduling estimate is 120
    MB; a 4 GB planning budget therefore selects 17 workers, rather than
    silently treating the larger cache as free memory.
    """
    spare_mb = (memory_budget_gb - FUSION_WORKER_MEMORY_MB / 1024.0
                - FUSION_BASE_MEMORY_GB) * 1024.0
    if spare_mb < 0.0:
        warnings.warn(
            f'memory budget {memory_budget_gb:.3g} GB does not cover one worker beyond this '
            f"run's fixed cost; falling back to a single worker. This parameter is a soft "
            'budget for planning parallelism, not a hard limit on process memory, and what '
            'the run actually uses depends on the size of the mosaic.',
            RuntimeWarning, stacklevel=2)
    cap = max(1, 1 + int(spare_mb / FUSION_WORKER_MEMORY_MB))
    processors = os.cpu_count() or 1
    return max(1, min(value or processors, cap, processors))
