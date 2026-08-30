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
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import tempfile
import threading
import time
from typing import Any
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
_WORKER_CACHE_SIZE = 4
_WORKER_CACHE_HITS = 0
#: Memory a fusion run costs before any worker starts: the render grid, the
#: frame tables and the tiled-TIFF writer's own buffers. Measured at 1.87 GB;
#: see resolve_jobs for the fit.
FUSION_BASE_MEMORY_GB = 1.9

#: Marginal resident cost of one render worker, measured at 98 MB on the same
#: fit. Each holds its share of the decoded source frames.
FUSION_WORKER_MEMORY_MB = 96.0

_WORKER_CACHE_MISSES = 0
UNCERTAINTY_SCALE_M = 1e-5
UNCERTAINTY_NODATA = np.iinfo(np.uint16).max


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
    global _WORKER_INTERIOR_DISTANCE, _WORKER_IMAGES
    global _WORKER_CACHE_HITS, _WORKER_CACHE_MISSES
    _WORKER_INITIAL, _WORKER_OPTIMIZED, _WORKER_GRID = initial, optimized, grid
    _WORKER_INTERIOR_DISTANCE = interior_distance_map(width, height)
    _WORKER_IMAGES = OrderedDict()
    _WORKER_CACHE_HITS = 0
    _WORKER_CACHE_MISSES = 0
    cv2.setNumThreads(1)


def _image(path: str) -> np.ndarray:
    global _WORKER_CACHE_HITS, _WORKER_CACHE_MISSES
    if path in _WORKER_IMAGES:
        _WORKER_CACHE_HITS += 1
        value = _WORKER_IMAGES.pop(path)
        _WORKER_IMAGES[path] = value
        return value
    _WORKER_CACHE_MISSES += 1
    value = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if value is None or value.dtype != np.uint8 or value.ndim != 2:
        raise FusionError(f'render source is not mono8: {path}.')
    _WORKER_IMAGES[path] = value
    while len(_WORKER_IMAGES) > _WORKER_CACHE_SIZE:
        _WORKER_IMAGES.popitem(last=False)
    return value


def _tile_transform(grid: RenderGrid, tile_row: int, tile_column: int) -> np.ndarray:
    scale = 1.0 / grid.resolution_m
    return np.array(((scale, 0.0, -grid.min_x_m * scale - 0.5 - tile_column),
                     (0.0, -scale, grid.max_y_m * scale - 0.5 - tile_row),
                     (0.0, 0.0, 1.0)), np.float64)


def _hard_cut(frames: tuple[RenderFrame, ...], tile_row: int, tile_column: int,
              candidates: tuple[int, ...], auxiliary: bool,
              quality: bool = False) -> tuple[np.ndarray, ...]:
    if _WORKER_GRID is None or _WORKER_INTERIOR_DISTANCE is None:
        raise FusionError('render worker is not initialized.')
    size = _WORKER_GRID.tile_size_px
    output = np.zeros((size, size), np.uint8)
    owner_priority = np.zeros((size, size), np.float32)
    coverage = np.zeros((size, size), np.uint16) if auxiliary or quality else None
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
    products: list[np.ndarray] = [output]
    if auxiliary and coverage is not None and uncertainty is not None:
        uncertainty[~valid] = np.nan
        products.extend((coverage, uncertainty))
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
    mode, row, column, initial_candidates, optimized_candidates = task
    if mode == 'initial':
        products = _hard_cut(_WORKER_INITIAL, row, column, initial_candidates, False, True)
    elif mode == 'optimized':
        products = _hard_cut(_WORKER_OPTIMIZED, row, column, optimized_candidates, True, True)
    else:
        raise FusionError(f'unknown render mode: {mode}.')
    return (row, column, *products,
            _WORKER_CACHE_HITS - hits_before, _WORKER_CACHE_MISSES - misses_before)


def _tasks(mode: str, grid: RenderGrid, initial: tuple[RenderFrame, ...],
           optimized: tuple[RenderFrame, ...]):
    size = grid.tile_size_px
    for row in range(0, grid.height_px, size):
        top = grid.max_y_m - row * grid.resolution_m
        bottom = grid.max_y_m - min(row + size, grid.height_px) * grid.resolution_m
        for column in range(0, grid.width_px, size):
            left = grid.min_x_m + column * grid.resolution_m
            right = grid.min_x_m + min(column + size, grid.width_px) * grid.resolution_m

            def intersects(frame: RenderFrame) -> bool:
                x0, y0, x1, y1 = frame.bbox_xy_m
                return x1 > left and x0 < right and y1 > bottom and y0 < top
            yield (mode, row, column,
                   tuple(index for index, frame in enumerate(initial) if intersects(frame)),
                   tuple(index for index, frame in enumerate(optimized) if intersects(frame)))


def _tile_count(grid: RenderGrid) -> int:
    rows = math.ceil(grid.height_px / grid.tile_size_px)
    columns = math.ceil(grid.width_px / grid.tile_size_px)
    return rows * columns


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
                      camera_width: int, camera_height: int, jobs: int,
                      coverage_fd: int | None = None,
                      uncertainty_fd: int | None = None,
                      image_raw_fd: int | None = None,
                      difference_fd: int | None = None,
                      previous_raw_fd: int | None = None) -> dict[str, Any]:
    started = time.perf_counter()
    task_values = _tasks(mode, grid, initial, optimized)
    with ProcessPoolExecutor(
            max_workers=jobs, initializer=_init_worker,
            initargs=(initial, optimized, grid, camera_width, camera_height)) as pool:
        results = pool.map(_render_task, task_values, chunksize=1)
        quality_histogram = np.zeros(4096, np.int64)
        quality_sum = 0.0
        quality_count = 0
        cache_hits = 0
        cache_misses = 0

        def image_tiles():
            nonlocal quality_sum, quality_count, cache_hits, cache_misses
            for result in results:
                cache_hits += int(result[-2])
                cache_misses += int(result[-1])
                row, column, image = result[:3]
                # The difference raster used to be a third full render of both
                # variants. Both tiles already exist by the time the second
                # pass runs: the first pass keeps its own, and this one
                # subtracts against it, so the same bytes come out of one
                # read instead of a whole render pass.
                if image_raw_fd is not None:
                    _write_raw_tile(image_raw_fd, image, grid, row, column)
                if difference_fd is not None and previous_raw_fd is not None:
                    previous = _read_raw_tile(
                        previous_raw_fd, np.dtype(np.uint8), grid, row, column)
                    _write_raw_tile(
                        difference_fd, cv2.absdiff(previous, image), grid, row, column)
                if coverage_fd is not None and uncertainty_fd is not None:
                    coverage, uncertainty = result[3:5]
                    _write_raw_tile(coverage_fd, coverage, grid, row, column)
                    _write_raw_tile(
                        uncertainty_fd, encode_uncertainty(uncertainty), grid, row, column)
                    quality = result[5]
                elif mode == 'initial':
                    quality = result[3]
                else:
                    quality = None
                if quality is not None:
                    finite = quality[np.isfinite(quality)]
                    if finite.size:
                        quality_sum += float(finite.sum(dtype=np.float64))
                        quality_count += int(finite.size)
                        bins = np.clip(np.rint(finite * (4095.0 / 255.0)), 0, 4095)
                        quality_histogram[:] += np.bincount(
                            bins.astype(np.int32), minlength=4096)
                yield image

        with tifffile.TiffWriter(path, bigtiff=True) as writer:
            writer.write(data=image_tiles(), shape=(grid.height_px, grid.width_px),
                         dtype=np.uint8, tile=(grid.tile_size_px, grid.tile_size_px),
                         compression='deflate', predictor=True,
                         description=_description(grid, mode))
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
        },
        'quality': quality_result,
    }


def _write_raw_tiles(path: Path, raw_path: Path, dtype: np.dtype, grid: RenderGrid,
                     kind: str, predictor: bool) -> None:
    with tifffile.TiffWriter(path, bigtiff=True) as writer:
        writer.write(data=_tiles_from_raw(raw_path, dtype, grid),
                     shape=(grid.height_px, grid.width_px), dtype=dtype,
                     tile=(grid.tile_size_px, grid.tile_size_px),
                     compression='deflate', predictor=predictor,
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
                      preview_max_side_px: int = 4096) -> dict[str, Any]:
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
    projections = project_inputs(inputs)
    initial, optimized = read_pose_graph(pose_graph_dir, inputs, projections)
    grid = common_grid(initial, optimized, resolution_m)
    work_dir.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(
        prefix=f'.{output_dir.name}.tmp-{uuid4().hex}-', dir=output_dir.parent))
    cache = Path(tempfile.mkdtemp(prefix='fusion-', dir=work_dir))
    coverage_fd = None
    uncertainty_fd = None
    pose_only_fd = None
    difference_fd = None
    resource_monitor = _ProcessTreePssMonitor()
    resource_monitor.start()
    try:
        coverage_path = cache / 'coverage.dat'
        uncertainty_path = cache / 'uncertainty.dat'
        pose_only_path = cache / 'pose_only.dat'
        difference_path = cache / 'difference.dat'
        tiles = _tile_count(grid) * grid.tile_size_px ** 2
        raw_bytes = tiles * np.dtype(np.uint16).itemsize
        image_bytes = tiles * np.dtype(np.uint8).itemsize
        coverage_fd = os.open(coverage_path, os.O_RDWR | os.O_CREAT | os.O_TRUNC, 0o600)
        uncertainty_fd = os.open(
            uncertainty_path, os.O_RDWR | os.O_CREAT | os.O_TRUNC, 0o600)
        pose_only_fd = os.open(pose_only_path, os.O_RDWR | os.O_CREAT | os.O_TRUNC, 0o600)
        difference_fd = os.open(difference_path, os.O_RDWR | os.O_CREAT | os.O_TRUNC, 0o600)
        os.ftruncate(coverage_fd, raw_bytes)
        os.ftruncate(uncertainty_fd, raw_bytes)
        os.ftruncate(pose_only_fd, image_bytes)
        os.ftruncate(difference_fd, image_bytes)
        initial_pass = _write_image_pass(
            temporary / 'mosaic_pose_only.tif', 'initial', grid,
            initial, optimized, inputs.camera.width, inputs.camera.height, jobs,
            image_raw_fd=pose_only_fd)
        optimized_pass = _write_image_pass(
            temporary / 'mosaic_optimized.tif', 'optimized', grid,
            initial, optimized, inputs.camera.width, inputs.camera.height, jobs,
            coverage_fd, uncertainty_fd,
            difference_fd=difference_fd, previous_raw_fd=pose_only_fd)
        os.close(coverage_fd)
        coverage_fd = None
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
            'mosaic_format_version': 1,
            'input_summary': input_summary(inputs),
            'grid': {'frame': 'wall', 'min_x_m': grid.min_x_m, 'min_y_m': grid.min_y_m,
                     'max_x_m': grid.max_x_m, 'max_y_m': grid.max_y_m,
                     'resolution_m_per_pixel': grid.resolution_m,
                     'width_px': grid.width_px, 'height_px': grid.height_px,
                     'tile_size_px': grid.tile_size_px},
            'fusion': {'method': ('single-image hard cut by maximum interior distance; '
                                  'stable input-frame order resolves ties'),
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
                       }},
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
             'preview_max_side_px': preview_max_side_px},
            {**processed_run_inputs(manifest['input_summary']),
             'pose_graph': artifact(pose_graph_dir / 'pose_graph.json'),
             'optimized_poses': artifact(pose_graph_dir / 'optimized_poses.json')},
            ('mosaic_manifest.json',))
        temporary.replace(output_dir)
        return manifest
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    finally:
        resource_monitor.stop()
        if coverage_fd is not None:
            os.close(coverage_fd)
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
    plus 98 MB each, so the 96 MB below was right and only the base was
    missing.  Fusion at 4 GB now runs 22 workers where it ran 8.
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
