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

"""Post-mosaic comparison against the immutable diagnostic-wall texture."""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import threading
from typing import Any
from uuid import uuid4

from climbot_mosaic.stage_provenance import artifact
from climbot_mosaic.stage_provenance import write_stage_provenance
import cv2
import numpy as np
from PIL import Image
import tifffile


class DiagnosticTruthError(ValueError):
    """A mosaic or diagnostic texture cannot produce trustworthy evidence."""


@dataclass(frozen=True)
class TruthGrid:
    """Metric-to-pixel mapping for the immutable wall texture."""

    origin_x_m: float
    origin_y_m: float
    width_m: float
    height_m: float
    scale_m_per_px: float
    width_px: int
    height_px: int


@dataclass(frozen=True)
class MosaicGrid:
    """Metric-to-pixel mapping declared by a completed wall mosaic."""

    min_x_m: float
    min_y_m: float
    max_x_m: float
    max_y_m: float
    resolution_m_per_pixel: float
    width_px: int
    height_px: int


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _document(path: Path, description: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise DiagnosticTruthError(f'{description} is invalid: {error}') from error
    if not isinstance(value, dict):
        raise DiagnosticTruthError(f'{description} must be a JSON object.')
    return value


def _finite(value: Any, description: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise DiagnosticTruthError(f'{description} must be numeric.') from error
    if not math.isfinite(result):
        raise DiagnosticTruthError(f'{description} must be finite.')
    return result


def _positive_integer(value: Any, description: str) -> int:
    if isinstance(value, bool):
        raise DiagnosticTruthError(f'{description} must be a positive integer.')
    try:
        result = int(value)
    except (TypeError, ValueError) as error:
        raise DiagnosticTruthError(f'{description} must be a positive integer.') from error
    if result <= 0:
        raise DiagnosticTruthError(f'{description} must be a positive integer.')
    return result


def _pair(value: Any, description: str) -> tuple[float, float]:
    if not isinstance(value, list) or len(value) != 2:
        raise DiagnosticTruthError(f'{description} must contain two finite values.')
    return (_finite(value[0], f'{description}[0]'), _finite(value[1], f'{description}[1]'))


def _feature_bounds(feature: dict[str, Any]) -> tuple[float, float, float, float]:
    points: list[tuple[float, float]] = []
    for key in ('points_m', 'polygon_m'):
        for point in feature.get(key, []):
            points.append(_pair(point, f'{feature.get("id", "feature")}.{key}'))
    for branch in feature.get('branches_m', []):
        for point in branch:
            points.append(_pair(point, f'{feature.get("id", "feature")}.branches_m'))
    if 'center_m' in feature:
        center_x, center_y = _pair(feature['center_m'], 'feature.center_m')
        if 'size_m' in feature:
            radius = math.hypot(*_pair(feature['size_m'], 'feature.size_m')) / 2.0
        else:
            radius = _finite(feature.get('radius_m', 0.0), 'feature.radius_m')
        points.extend(((center_x - radius, center_y - radius),
                       (center_x + radius, center_y + radius)))
    if not points:
        raise DiagnosticTruthError('diagnostic feature has no metric geometry.')
    half_width = _finite(feature.get('width_m', 0.0), 'feature.width_m') / 2.0
    return (min(point[0] for point in points) - half_width,
            min(point[1] for point in points) - half_width,
            max(point[0] for point in points) + half_width,
            max(point[1] for point in points) + half_width)


def _expand(bounds: tuple[float, float, float, float], padding_m: float):
    return (bounds[0] - padding_m, bounds[1] - padding_m,
            bounds[2] + padding_m, bounds[3] + padding_m)


def _inside(bounds: tuple[float, float, float, float], grid: TruthGrid | MosaicGrid) -> bool:
    if isinstance(grid, TruthGrid):
        min_x, min_y = grid.origin_x_m, grid.origin_y_m
        max_x, max_y = min_x + grid.width_m, min_y + grid.height_m
    else:
        min_x, min_y, max_x, max_y = (
            grid.min_x_m, grid.min_y_m, grid.max_x_m, grid.max_y_m)
    return bounds[0] >= min_x and bounds[1] >= min_y and \
        bounds[2] <= max_x and bounds[3] <= max_y


def _wall_grid(document: dict[str, Any]) -> TruthGrid:
    origin_x, origin_y = _pair(document.get('region_origin_m'), 'region_origin_m')
    width_m, height_m = _pair(document.get('region_m'), 'region_m')
    scale = _finite(document.get('scale_m_per_px'), 'scale_m_per_px')
    if min(width_m, height_m, scale) <= 0.0:
        raise DiagnosticTruthError('diagnostic wall geometry must be positive.')
    width = _positive_integer(document.get('width_px'), 'width_px')
    height = _positive_integer(document.get('height_px'), 'height_px')
    if abs(width * scale - width_m) > scale or abs(height * scale - height_m) > scale:
        raise DiagnosticTruthError('diagnostic wall pixels disagree with its metric extent.')
    return TruthGrid(origin_x, origin_y, width_m, height_m, scale, width, height)


def _mosaic_grid(document: dict[str, Any]) -> MosaicGrid:
    value = document.get('grid')
    if not isinstance(value, dict):
        raise DiagnosticTruthError('mosaic manifest lacks grid.')
    min_x = _finite(value.get('min_x_m'), 'mosaic min_x_m')
    min_y = _finite(value.get('min_y_m'), 'mosaic min_y_m')
    max_x = _finite(value.get('max_x_m'), 'mosaic max_x_m')
    max_y = _finite(value.get('max_y_m'), 'mosaic max_y_m')
    resolution = _finite(value.get('resolution_m_per_pixel'), 'mosaic resolution')
    width = _positive_integer(value.get('width_px'), 'mosaic width_px')
    height = _positive_integer(value.get('height_px'), 'mosaic height_px')
    if min(max_x - min_x, max_y - min_y, resolution) <= 0.0:
        raise DiagnosticTruthError('mosaic grid geometry must be positive.')
    if abs(width * resolution - (max_x - min_x)) > resolution or \
            abs(height * resolution - (max_y - min_y)) > resolution:
        raise DiagnosticTruthError('mosaic pixels disagree with its metric extent.')
    return MosaicGrid(min_x, min_y, max_x, max_y, resolution, width, height)


def _blocks(document: dict[str, Any]) -> tuple[dict[str, Any], ...]:
    try:
        blocks = document['maps']['albedo']['blocks']
    except (KeyError, TypeError) as error:
        raise DiagnosticTruthError('diagnostic wall lacks albedo blocks.') from error
    if not isinstance(blocks, list) or not blocks:
        raise DiagnosticTruthError('diagnostic wall has no albedo blocks.')
    return tuple(blocks)


def _reference_crop(wall_dir: Path, blocks: tuple[dict[str, Any], ...], grid: TruthGrid,
                    bounds: tuple[float, float, float, float]) -> np.ndarray:
    if not _inside(bounds, grid):
        raise DiagnosticTruthError('reference crop is outside the diagnostic wall.')
    x0 = int(round((bounds[0] - grid.origin_x_m) / grid.scale_m_per_px))
    x1 = int(round((bounds[2] - grid.origin_x_m) / grid.scale_m_per_px))
    y0 = int(round((grid.origin_y_m + grid.height_m - bounds[3]) / grid.scale_m_per_px))
    y1 = int(round((grid.origin_y_m + grid.height_m - bounds[1]) / grid.scale_m_per_px))
    if x1 <= x0 or y1 <= y0:
        raise DiagnosticTruthError('reference crop is empty.')
    output = np.zeros((y1 - y0, x1 - x0), np.uint8)
    complete = np.zeros(output.shape, bool)
    for block in blocks:
        try:
            bx = _positive_integer(block['x_px'] + 1, 'block.x_px') - 1
            by = _positive_integer(block['y_px'] + 1, 'block.y_px') - 1
            bw = _positive_integer(block['width_px'], 'block.width_px')
            bh = _positive_integer(block['height_px'], 'block.height_px')
            name = block['file']
            sample_x = _positive_integer(block.get('sample_x_px', bx) + 1, 'sample_x_px') - 1
            sample_y = _positive_integer(block.get('sample_y_px', by) + 1, 'sample_y_px') - 1
        except (KeyError, TypeError) as error:
            raise DiagnosticTruthError('diagnostic albedo block is malformed.') from error
        ix0, iy0 = max(x0, bx), max(y0, by)
        ix1, iy1 = min(x1, bx + bw), min(y1, by + bh)
        if ix1 <= ix0 or iy1 <= iy0:
            continue
        if not isinstance(name, str) or Path(name).name != name:
            raise DiagnosticTruthError('diagnostic albedo file name is unsafe.')
        path = wall_dir / name
        try:
            with Image.open(path) as image:
                rgb = np.asarray(image.convert('RGB'), dtype=np.uint8)
        except (OSError, ValueError) as error:
            raise DiagnosticTruthError(
                f'cannot decode diagnostic albedo block {name}: {error}') from error
        sx0, sy0 = ix0 - sample_x, iy0 - sample_y
        sx1, sy1 = sx0 + ix1 - ix0, sy0 + iy1 - iy0
        if sx0 < 0 or sy0 < 0 or sx1 > rgb.shape[1] or sy1 > rgb.shape[0]:
            raise DiagnosticTruthError('diagnostic albedo block sample bounds are invalid.')
        target_y0, target_x0 = iy0 - y0, ix0 - x0
        target_y1, target_x1 = target_y0 + iy1 - iy0, target_x0 + ix1 - ix0
        output[target_y0:target_y1, target_x0:target_x1] = cv2.cvtColor(
            rgb[sy0:sy1, sx0:sx1], cv2.COLOR_RGB2GRAY)
        complete[target_y0:target_y1, target_x0:target_x1] = True
    if not bool(complete.all()):
        raise DiagnosticTruthError('reference crop contains an uncovered albedo pixel.')
    return output


def _mosaic_crop(image: np.ndarray, grid: MosaicGrid,
                 bounds: tuple[float, float, float, float]) -> np.ndarray:
    if image.dtype != np.uint8 or image.ndim != 2:
        raise DiagnosticTruthError('mosaic master must be mono8.')
    if image.shape != (grid.height_px, grid.width_px):
        raise DiagnosticTruthError('mosaic master dimensions disagree with its manifest.')
    if not _inside(bounds, grid):
        raise DiagnosticTruthError('mosaic crop is outside the render grid.')
    x0 = int(round((bounds[0] - grid.min_x_m) / grid.resolution_m_per_pixel))
    x1 = int(round((bounds[2] - grid.min_x_m) / grid.resolution_m_per_pixel))
    y0 = int(round((grid.max_y_m - bounds[3]) / grid.resolution_m_per_pixel))
    y1 = int(round((grid.max_y_m - bounds[1]) / grid.resolution_m_per_pixel))
    return image[y0:y1, x0:x1]


def estimate_translation(reference: np.ndarray, observed: np.ndarray) -> tuple[
        float, float, float]:
    """Estimate the observed-image translation in reference pixel coordinates."""
    if reference.ndim != 2 or observed.ndim != 2 or reference.shape != observed.shape:
        raise DiagnosticTruthError('phase-correlation inputs must be same-size mono images.')
    if min(reference.shape) < 32:
        raise DiagnosticTruthError('diagnostic feature crop is too small.')
    reference_edges = cv2.Laplacian(reference, cv2.CV_32F, ksize=3)
    observed_edges = cv2.Laplacian(observed, cv2.CV_32F, ksize=3)
    window = cv2.createHanningWindow((reference.shape[1], reference.shape[0]), cv2.CV_32F)
    shift, response = cv2.phaseCorrelate(reference_edges, observed_edges, window)
    if not all(math.isfinite(float(value)) for value in (*shift, response)):
        raise DiagnosticTruthError('phase correlation produced a non-finite result.')
    return float(shift[0]), float(shift[1]), float(response)


def fit_similarity(expected_xy_m: np.ndarray, observed_xy_m: np.ndarray) -> dict[str, Any]:
    """Fit all trusted anchors without hiding local wall-registration residuals."""
    if expected_xy_m.shape != observed_xy_m.shape or expected_xy_m.ndim != 2 or \
            expected_xy_m.shape[1] != 2 or len(expected_xy_m) < 2:
        raise DiagnosticTruthError('at least two paired two-dimensional anchors are required.')
    matrix = np.zeros((2 * len(expected_xy_m), 4), np.float64)
    target = observed_xy_m.reshape(-1)
    for index, (x_value, y_value) in enumerate(expected_xy_m):
        matrix[2 * index] = (x_value, -y_value, 1.0, 0.0)
        matrix[2 * index + 1] = (y_value, x_value, 0.0, 1.0)
    try:
        a_value, b_value, tx_value, ty_value = np.linalg.lstsq(
            matrix, target, rcond=None)[0]
    except np.linalg.LinAlgError as error:
        raise DiagnosticTruthError(
            'diagnostic anchors cannot fit a similarity transform.') from error
    rotation_scale = np.asarray(((a_value, -b_value), (b_value, a_value)), np.float64)
    scale = math.hypot(float(a_value), float(b_value))
    if scale <= 0.0:
        raise DiagnosticTruthError('diagnostic similarity scale is invalid.')
    yaw = math.atan2(float(b_value), float(a_value))
    translation = np.asarray((tx_value, ty_value), np.float64)
    predicted = expected_xy_m @ rotation_scale.T + translation
    residuals = np.linalg.norm(observed_xy_m - predicted, axis=1)
    inlier_mask = np.ones(len(expected_xy_m), bool)
    return {
        'scale': scale,
        'scale_error_ppm': (scale - 1.0) * 1_000_000.0,
        'yaw_error_deg': math.degrees(yaw),
        'translation_m': [float(tx_value), float(ty_value)],
        'inlier_count': int(inlier_mask.sum()),
        'residuals_m': [float(value) for value in residuals],
        'inlier_mask': [bool(value) for value in inlier_mask],
    }


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    values = sorted(values)
    return values[max(0, math.ceil(fraction * len(values)) - 1)]


def _histogram_summary(histogram: np.ndarray, value_scale: float = 1.0) -> dict[str, Any] | None:
    """Summarize a bounded non-negative integer distribution without retaining samples."""
    count = int(histogram.sum())
    if not count:
        return None
    cumulative = np.cumsum(histogram)

    def percentile(fraction: float) -> float:
        return float(np.searchsorted(cumulative, math.ceil(fraction * count)) * value_scale)

    values = np.arange(len(histogram), dtype=np.float64)
    return {
        'count': count,
        'mean': float((values * histogram).sum() / count * value_scale),
        'median': percentile(0.50), 'p95': percentile(0.95),
        'p99': percentile(0.99),
        'maximum': float(np.flatnonzero(histogram)[-1] * value_scale),
    }


def _seam_adjacencies(path: Path, grid: MosaicGrid) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Load and validate the sparse hard-cut source transitions from fusion."""
    try:
        with np.load(path, allow_pickle=False) as values:
            version = int(values['seam_format_version'])
            rows = np.asarray(values['row_px'], np.uint32)
            columns = np.asarray(values['column_px'], np.uint32)
            axes = np.asarray(values['axis'], np.uint8)
            width = int(values['grid_width_px'])
            height = int(values['grid_height_px'])
    except (OSError, KeyError, TypeError, ValueError) as error:
        raise DiagnosticTruthError(f'seam adjacency product is invalid: {error}') from error
    if version != 1 or width != grid.width_px or height != grid.height_px:
        raise DiagnosticTruthError('seam adjacency product does not match the mosaic grid.')
    if rows.ndim != 1 or columns.ndim != 1 or axes.ndim != 1 or \
            not (len(rows) == len(columns) == len(axes)):
        raise DiagnosticTruthError('seam adjacency arrays must be equally-sized vectors.')
    valid = ((axes <= 1) & (rows < grid.height_px) & (columns < grid.width_px) &
             ~((axes == 0) & (columns + 1 >= grid.width_px)) &
             ~((axes == 1) & (rows + 1 >= grid.height_px)))
    if not bool(valid.all()):
        raise DiagnosticTruthError('seam adjacency contains an out-of-grid neighbour.')
    return rows, columns, axes


class _ReferenceTileReader:
    """Read bounded grayscale truth windows with a small decoded-DDS cache."""

    def __init__(self, wall_dir: Path, blocks: tuple[dict[str, Any], ...],
                 truth_grid: TruthGrid, mosaic_grid: MosaicGrid, cache_size: int = 8) -> None:
        if abs(truth_grid.scale_m_per_px - mosaic_grid.resolution_m_per_pixel) > 1e-12:
            raise DiagnosticTruthError(
                'truth and mosaic resolutions must match for diagnostic comparison.')
        self.wall_dir = wall_dir
        self.blocks = blocks
        self.cache_size = cache_size
        self.x_offset = int(round(
            (mosaic_grid.min_x_m - truth_grid.origin_x_m) / truth_grid.scale_m_per_px))
        self.y_offset = int(round(
            (truth_grid.origin_y_m + truth_grid.height_m - mosaic_grid.max_y_m) /
            truth_grid.scale_m_per_px))
        self._cache: OrderedDict[str, np.ndarray] = OrderedDict()

    def _block_image(self, block: dict[str, Any]) -> np.ndarray:
        name = block.get('file')
        if not isinstance(name, str) or Path(name).name != name:
            raise DiagnosticTruthError('diagnostic albedo file name is unsafe.')
        if name in self._cache:
            image = self._cache.pop(name)
            self._cache[name] = image
            return image
        try:
            with Image.open(self.wall_dir / name) as source:
                image = cv2.cvtColor(
                    np.asarray(source.convert('RGB'), dtype=np.uint8), cv2.COLOR_RGB2GRAY)
        except (OSError, ValueError) as error:
            raise DiagnosticTruthError(
                f'cannot decode diagnostic albedo block {name}: {error}') from error
        self._cache[name] = image
        while len(self._cache) > self.cache_size:
            self._cache.popitem(last=False)
        return image

    def tile(self, row: int, column: int, height: int, width: int) -> np.ndarray:
        """Return one mosaic-aligned truth tile without allocating the full wall crop."""
        x0, y0 = self.x_offset + column, self.y_offset + row
        x1, y1 = x0 + width, y0 + height
        output = np.zeros((height, width), np.uint8)
        complete = np.zeros((height, width), bool)
        for block in self.blocks:
            try:
                bx = _positive_integer(block['x_px'] + 1, 'block.x_px') - 1
                by = _positive_integer(block['y_px'] + 1, 'block.y_px') - 1
                bw = _positive_integer(block['width_px'], 'block.width_px')
                bh = _positive_integer(block['height_px'], 'block.height_px')
                sample_x = _positive_integer(
                    block.get('sample_x_px', bx) + 1, 'sample_x_px') - 1
                sample_y = _positive_integer(
                    block.get('sample_y_px', by) + 1, 'sample_y_px') - 1
            except (KeyError, TypeError) as error:
                raise DiagnosticTruthError('diagnostic albedo block is malformed.') from error
            ix0, iy0 = max(x0, bx), max(y0, by)
            ix1, iy1 = min(x1, bx + bw), min(y1, by + bh)
            if ix1 <= ix0 or iy1 <= iy0:
                continue
            image = self._block_image(block)
            sx0, sy0 = ix0 - sample_x, iy0 - sample_y
            sx1, sy1 = sx0 + ix1 - ix0, sy0 + iy1 - iy0
            if sx0 < 0 or sy0 < 0 or sx1 > image.shape[1] or sy1 > image.shape[0]:
                raise DiagnosticTruthError('diagnostic albedo block sample bounds are invalid.')
            target_y0, target_x0 = iy0 - y0, ix0 - x0
            target_y1, target_x1 = target_y0 + iy1 - iy0, target_x0 + ix1 - ix0
            output[target_y0:target_y1, target_x0:target_x1] = image[sy0:sy1, sx0:sx1]
            complete[target_y0:target_y1, target_x0:target_x1] = True
        if not bool(complete.all()):
            raise DiagnosticTruthError('reference tile contains an uncovered albedo pixel.')
        return output


class _SeamGradientAccumulator:
    """Accumulate source-transition and same-owner gradient distributions tile by tile."""

    def __init__(self, rows: np.ndarray, columns: np.ndarray, axes: np.ndarray,
                 grid: MosaicGrid, tile_size_px: int) -> None:
        self.grid = grid
        self.tile_size_px = tile_size_px
        self.tile_columns = math.ceil(grid.width_px / tile_size_px)
        target_rows = rows.astype(np.uint64) + (axes == 1)
        target_columns = columns.astype(np.uint64) + (axes == 0)
        tile_ids = ((target_rows // tile_size_px) * self.tile_columns +
                    target_columns // tile_size_px)
        order = np.argsort(tile_ids, kind='stable')
        self.tile_ids = tile_ids[order]
        self.rows = target_rows[order].astype(np.uint32)
        self.columns = target_columns[order].astype(np.uint32)
        self.axes = axes[order]
        self.on_observed = np.zeros(256, np.int64)
        self.on_truth = np.zeros(256, np.int64)
        self.on_excess = np.zeros(256, np.int64)
        self.off_excess = np.zeros(256, np.int64)
        self.axis_counts = {
            'rightward': int((axes == 0).sum()),
            'downward': int((axes == 1).sum()),
        }

    @staticmethod
    def _add(histogram: np.ndarray, values: np.ndarray) -> None:
        if values.size:
            histogram += np.bincount(values, minlength=256)

    def add_tile(self, row: int, column: int, image: np.ndarray, reference: np.ndarray,
                 coverage: np.ndarray, previous_right: np.ndarray | None,
                 previous_bottom: np.ndarray | None,
                 previous_reference_right: np.ndarray | None,
                 previous_reference_bottom: np.ndarray | None,
                 previous_coverage_right: np.ndarray | None,
                 previous_coverage_bottom: np.ndarray | None) -> None:
        """Accumulate a tile, including adjacencies crossing its top and left edges."""
        height, width = image.shape
        if reference.shape != image.shape or coverage.shape != image.shape:
            raise DiagnosticTruthError(
                'seam tiles must share image, truth, and coverage dimensions.')
        tile_id = (row // self.tile_size_px) * self.tile_columns + column // self.tile_size_px
        start = int(np.searchsorted(self.tile_ids, tile_id, side='left'))
        end = int(np.searchsorted(self.tile_ids, tile_id, side='right'))
        horizontal_seams = np.zeros((height, width), bool)
        vertical_seams = np.zeros((height, width), bool)
        if end > start:
            local_rows = self.rows[start:end].astype(np.intp) - row
            local_columns = self.columns[start:end].astype(np.intp) - column
            axes = self.axes[start:end]
            horizontal = axes == 0
            horizontal_seams[local_rows[horizontal], local_columns[horizontal]] = True
            vertical_seams[local_rows[~horizontal], local_columns[~horizontal]] = True

        def gradients(axis: int):
            if axis == 0:
                if previous_right is None:
                    valid_edge = np.zeros(height, bool)
                    left_image = np.zeros(height, np.uint8)
                    left_truth = np.zeros(height, np.uint8)
                    left_coverage = np.zeros(height, coverage.dtype)
                else:
                    valid_edge = np.ones(height, bool)
                    left_image, left_truth, left_coverage = (
                        previous_right, previous_reference_right, previous_coverage_right)
                image_previous = np.empty_like(image)
                truth_previous = np.empty_like(reference)
                coverage_previous = np.empty_like(coverage)
                image_previous[:, 0], truth_previous[:, 0] = left_image, left_truth
                coverage_previous[:, 0] = left_coverage
                image_previous[:, 1:] = image[:, :-1]
                truth_previous[:, 1:] = reference[:, :-1]
                coverage_previous[:, 1:] = coverage[:, :-1]
                valid = (coverage > 0) & (coverage_previous > 0)
                valid[:, 0] &= valid_edge
                seams = horizontal_seams
            else:
                if previous_bottom is None:
                    valid_edge = np.zeros(width, bool)
                    top_image = np.zeros(width, np.uint8)
                    top_truth = np.zeros(width, np.uint8)
                    top_coverage = np.zeros(width, coverage.dtype)
                else:
                    valid_edge = np.ones(width, bool)
                    top_image, top_truth, top_coverage = (
                        previous_bottom, previous_reference_bottom, previous_coverage_bottom)
                image_previous = np.empty_like(image)
                truth_previous = np.empty_like(reference)
                coverage_previous = np.empty_like(coverage)
                image_previous[0, :], truth_previous[0, :] = top_image, top_truth
                coverage_previous[0, :] = top_coverage
                image_previous[1:, :] = image[:-1, :]
                truth_previous[1:, :] = reference[:-1, :]
                coverage_previous[1:, :] = coverage[:-1, :]
                valid = (coverage > 0) & (coverage_previous > 0)
                valid[0, :] &= valid_edge
                seams = vertical_seams
            invalid_seams = seams & ~valid
            if bool(invalid_seams.any()):
                local_row, local_column = np.argwhere(invalid_seams)[0]
                raise DiagnosticTruthError(
                    'hard-cut seam is not covered by both adjacent images at '
                    f'({row + local_row}, {column + local_column}), axis {axis}.')
            observed = np.abs(image.astype(np.int16) - image_previous.astype(np.int16))
            truth = np.abs(reference.astype(np.int16) - truth_previous.astype(np.int16))
            excess = np.maximum(0, observed - truth)
            on = seams
            off = valid & ~seams
            self._add(self.on_observed, observed[on])
            self._add(self.on_truth, truth[on])
            self._add(self.on_excess, excess[on])
            self._add(self.off_excess, excess[off])

        gradients(0)
        gradients(1)

    def summary(self) -> dict[str, Any]:
        """Return only interpretable hard-cut gradient evidence and its deterministic baseline."""
        on_excess = _histogram_summary(self.on_excess)
        off_excess = _histogram_summary(self.off_excess)
        ratio = None
        if on_excess is not None and off_excess is not None and off_excess['p95'] > 0.0:
            ratio = on_excess['p95'] / off_excess['p95']
        return {
            'seam_adjacency_count': int(self.on_excess.sum()),
            'axis_counts': self.axis_counts,
            'gradient_excess_gray_per_pixel': {
                'definition': ('adjacent-pixel grayscale discontinuity beyond immutable truth at '
                               'hard-cut source transitions; the baseline uses every covered '
                               'same-owner adjacency in the same raster and directions'),
                'on_hard_cut': {
                    'observed': _histogram_summary(self.on_observed),
                    'immutable_truth': _histogram_summary(self.on_truth),
                    'excess_over_truth': on_excess,
                },
                'off_hard_cut_baseline': {
                    'excess_over_truth': off_excess,
                },
                'on_to_off_excess_p95_ratio': ratio,
            },
        }


def _crop_pixels(grid: MosaicGrid, bounds: tuple[float, float, float, float]) -> tuple[
        int, int, int, int]:
    """Map metric bounds to a mosaic pixel rectangle using the raster contract."""
    if not _inside(bounds, grid):
        raise DiagnosticTruthError('mosaic crop is outside the render grid.')
    x0 = int(round((bounds[0] - grid.min_x_m) / grid.resolution_m_per_pixel))
    x1 = int(round((bounds[2] - grid.min_x_m) / grid.resolution_m_per_pixel))
    y0 = int(round((grid.max_y_m - bounds[3]) / grid.resolution_m_per_pixel))
    y1 = int(round((grid.max_y_m - bounds[1]) / grid.resolution_m_per_pixel))
    if x1 <= x0 or y1 <= y0:
        raise DiagnosticTruthError('mosaic anchor crop is empty.')
    return y0, y1, x0, x1


class _AnchorCropCollector:
    """Collect small anchor windows while the full mosaic is decoded once by tiles."""

    def __init__(self, anchors: list[dict[str, Any]], grid: MosaicGrid) -> None:
        self._windows: dict[str, tuple[int, int, int, int]] = {}
        self._crops: dict[str, np.ndarray] = {}
        for anchor in anchors:
            identifier = anchor.get('id')
            if not isinstance(identifier, str) or identifier in self._windows:
                raise DiagnosticTruthError(
                    'diagnostic anchors must have unique string identifiers.')
            window = _crop_pixels(grid, tuple(float(value) for value in anchor['bounds_m']))
            self._windows[identifier] = window
            self._crops[identifier] = np.zeros(
                (window[1] - window[0], window[3] - window[2]), np.uint8)

    def add_tile(self, row: int, column: int, image: np.ndarray) -> None:
        """Copy every overlap from one decoded mosaic tile into its small target crop."""
        bottom, right = row + image.shape[0], column + image.shape[1]
        for identifier, (y0, y1, x0, x1) in self._windows.items():
            iy0, iy1, ix0, ix1 = max(row, y0), min(bottom, y1), max(column, x0), min(right, x1)
            if iy1 <= iy0 or ix1 <= ix0:
                continue
            self._crops[identifier][iy0 - y0:iy1 - y0, ix0 - x0:ix1 - x0] = image[
                iy0 - row:iy1 - row, ix0 - column:ix1 - column]

    def crops(self) -> dict[str, np.ndarray]:
        """Return fully assembled anchor crops after the tile stream finishes."""
        return self._crops


def _tile_segments(path: Path, grid: MosaicGrid, tile_size_px: int,
                   dtype: np.dtype = np.dtype(np.uint8)):
    """
    Yield valid, unpadded mono8 TIFF tiles in deterministic raster order.

    ``segments`` sorts by file offset, which is raster order only because the
    writer emits tiles that way.  The seam accumulator carries a single edge
    vector between neighbouring tiles and assigns seams by manifest tile size,
    so both invariants are checked here rather than assumed: a different order
    or tile size would silently mispair tile borders instead of failing.
    """
    try:
        with tifffile.TiffFile(path) as document:
            page = document.pages[0]
            if page.shape != (grid.height_px, grid.width_px) or page.dtype != dtype:
                raise DiagnosticTruthError('mosaic master dimensions disagree with its manifest.')
            expected_row, expected_column = 0, 0
            for values, index, _ in page.segments(
                    maxworkers=1, sort=True, buffersize=1024 * 1024):
                if values is None or values.dtype != dtype or values.ndim != 4:
                    raise DiagnosticTruthError('mosaic TIFF has an invalid tile segment.')
                if values.shape[1] != tile_size_px or values.shape[2] != tile_size_px:
                    raise DiagnosticTruthError(
                        'mosaic TIFF tile size disagrees with its manifest.')
                if (int(index[2]), int(index[3])) != (expected_row, expected_column):
                    raise DiagnosticTruthError('mosaic TIFF tiles are not in raster order.')
                row, column = expected_row, expected_column
                height = min(tile_size_px, grid.height_px - row)
                width = min(tile_size_px, grid.width_px - column)
                if height <= 0 or width <= 0:
                    raise DiagnosticTruthError('mosaic TIFF tile is outside its manifest grid.')
                yield row, column, values[0, :height, :width, 0]
                expected_column += tile_size_px
                if expected_column >= grid.width_px:
                    expected_column, expected_row = 0, expected_row + tile_size_px
            if expected_row < grid.height_px or expected_column:
                raise DiagnosticTruthError('mosaic TIFF stops before its last manifest tile.')
    except (OSError, tifffile.TiffFileError) as error:
        raise DiagnosticTruthError(f'cannot read mosaic TIFF tiles: {error}') from error


def _process_pss_bytes() -> int:
    """Read this evaluator's Linux proportional resident set without another dependency."""
    try:
        rollup = Path(f'/proc/{os.getpid()}/smaps_rollup').read_text(encoding='ascii')
        pss = next(line for line in rollup.splitlines() if line.startswith('Pss:'))
        return int(pss.split()[1]) * 1024
    except (OSError, StopIteration, IndexError, ValueError):
        return 0


class _PssMonitor:
    """Sample evaluator PSS while tiled truth evaluation is running."""

    def __init__(self, interval_s: float = 0.25) -> None:
        self.interval_s = interval_s
        self.peak_bytes = 0
        self._finished = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _run(self) -> None:
        while not self._finished.is_set():
            self.peak_bytes = max(self.peak_bytes, _process_pss_bytes())
            self._finished.wait(self.interval_s)
        self.peak_bytes = max(self.peak_bytes, _process_pss_bytes())

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._finished.set()
        if self._thread.is_alive():
            self._thread.join()


def _stream_variant(path: Path, coverage_path: Path, anchors: list[dict[str, Any]],
                    reference: _ReferenceTileReader, seam_rows: np.ndarray,
                    seam_columns: np.ndarray, seam_axes: np.ndarray,
                    grid: MosaicGrid, tile_size_px: int) -> tuple[
                        dict[str, Any], dict[str, np.ndarray]]:
    """Evaluate one master in bounded memory while retaining only anchor crops."""
    collector = _AnchorCropCollector(anchors, grid)
    accumulator = _SeamGradientAccumulator(
        seam_rows, seam_columns, seam_axes, grid, tile_size_px)
    previous_right = previous_reference_right = previous_coverage_right = None
    previous_bottom: dict[int, np.ndarray] = {}
    previous_reference_bottom: dict[int, np.ndarray] = {}
    previous_coverage_bottom: dict[int, np.ndarray] = {}
    coverage_tiles = _tile_segments(coverage_path, grid, tile_size_px, np.dtype(np.uint16))
    for row, column, image in _tile_segments(path, grid, tile_size_px):
        try:
            coverage_row, coverage_column, coverage = next(coverage_tiles)
        except StopIteration as error:
            raise DiagnosticTruthError('coverage TIFF is missing a mosaic tile.') from error
        if (coverage_row, coverage_column, coverage.shape) != (row, column, image.shape):
            raise DiagnosticTruthError('coverage TIFF tile layout differs from the mosaic master.')
        if column == 0:
            previous_right = previous_reference_right = previous_coverage_right = None
        truth = reference.tile(row, column, image.shape[0], image.shape[1])
        accumulator.add_tile(
            row, column, image, truth, coverage, previous_right, previous_bottom.get(column),
            previous_reference_right, previous_reference_bottom.get(column),
            previous_coverage_right, previous_coverage_bottom.get(column))
        collector.add_tile(row, column, image)
        previous_right = image[:, -1].copy()
        previous_reference_right = truth[:, -1].copy()
        previous_coverage_right = coverage[:, -1].copy()
        previous_bottom[column] = image[-1, :].copy()
        previous_reference_bottom[column] = truth[-1, :].copy()
        previous_coverage_bottom[column] = coverage[-1, :].copy()
    try:
        next(coverage_tiles)
        raise DiagnosticTruthError('coverage TIFF has extra mosaic tiles.')
    except StopIteration:
        pass
    return accumulator.summary(), collector.crops()


def _anchors(features: list[dict[str, Any]], truth_grid: TruthGrid, mosaic_grid: MosaicGrid,
             padding_m: float) -> list[dict[str, Any]]:
    result = []
    for feature in features:
        if feature.get('kind') not in ('repair_patch', 'graffiti_decal'):
            continue
        bounds = _expand(_feature_bounds(feature), padding_m)
        if not _inside(bounds, truth_grid) or not _inside(bounds, mosaic_grid):
            continue
        center = feature.get('center_m')
        if not isinstance(center, list):
            raw = _feature_bounds(feature)
            center = [(raw[0] + raw[2]) / 2.0, (raw[1] + raw[3]) / 2.0]
        result.append({
            'id': feature.get('id'), 'kind': feature.get('kind'),
            'center_m': list(_pair(center, 'feature center')), 'bounds_m': list(bounds),
        })
    if len(result) < 2:
        raise DiagnosticTruthError('fewer than two diagnostic anchors are visible in the mosaic.')
    return result


def _variant_matches(image: np.ndarray, anchors: list[dict[str, Any]], wall_dir: Path,
                     blocks: tuple[dict[str, Any], ...], truth_grid: TruthGrid,
                     mosaic_grid: MosaicGrid) -> list[dict[str, Any]]:
    if abs(truth_grid.scale_m_per_px - mosaic_grid.resolution_m_per_pixel) > 1e-12:
        raise DiagnosticTruthError(
            'truth and mosaic resolutions must match for diagnostic comparison.')
    matches = []
    for anchor in anchors:
        bounds = tuple(float(value) for value in anchor['bounds_m'])
        reference = _reference_crop(wall_dir, blocks, truth_grid, bounds)
        observed = _mosaic_crop(image, mosaic_grid, bounds)
        # The two metric grids have the same resolution but need not share a
        # pixel origin.  Rounding each crop endpoint can differ by one pixel;
        # resample that bounded difference to the immutable truth crop.
        if observed.shape != reference.shape:
            observed = cv2.resize(
                observed, (reference.shape[1], reference.shape[0]),
                interpolation=cv2.INTER_LINEAR)
        shift_x, shift_y, response = estimate_translation(reference, observed)
        expected = tuple(float(value) for value in anchor['center_m'])
        observed_xy = (
            expected[0] + shift_x * mosaic_grid.resolution_m_per_pixel,
            expected[1] - shift_y * mosaic_grid.resolution_m_per_pixel)
        matches.append({
            'id': anchor['id'], 'kind': anchor['kind'], 'expected_center_m': list(expected),
            'observed_center_m': list(observed_xy),
            'offset_m': [observed_xy[0] - expected[0], observed_xy[1] - expected[1]],
            'offset_norm_m': math.dist(expected, observed_xy),
            'phase_response': response,
        })
    return matches


def _variant_matches_from_crops(crops: dict[str, np.ndarray], anchors: list[dict[str, Any]],
                                wall_dir: Path, blocks: tuple[dict[str, Any]],
                                truth_grid: TruthGrid,
                                mosaic_grid: MosaicGrid) -> list[dict[str, Any]]:
    """Measure anchors retained from a bounded tiled decode of one mosaic master."""
    matches = []
    for anchor in anchors:
        identifier = anchor['id']
        if identifier not in crops:
            raise DiagnosticTruthError('mosaic tile stream did not collect every anchor crop.')
        bounds = tuple(float(value) for value in anchor['bounds_m'])
        reference = _reference_crop(wall_dir, blocks, truth_grid, bounds)
        observed = crops[identifier]
        if observed.shape != reference.shape:
            observed = cv2.resize(
                observed, (reference.shape[1], reference.shape[0]), interpolation=cv2.INTER_LINEAR)
        shift_x, shift_y, response = estimate_translation(reference, observed)
        expected = tuple(float(value) for value in anchor['center_m'])
        observed_xy = (
            expected[0] + shift_x * mosaic_grid.resolution_m_per_pixel,
            expected[1] - shift_y * mosaic_grid.resolution_m_per_pixel)
        matches.append({
            'id': identifier, 'kind': anchor['kind'], 'expected_center_m': list(expected),
            'observed_center_m': list(observed_xy),
            'offset_m': [observed_xy[0] - expected[0], observed_xy[1] - expected[1]],
            'offset_norm_m': math.dist(expected, observed_xy),
            'phase_response': response,
        })
    return matches


def _summarize_variant(matches: list[dict[str, Any]], accepted_ids: set[str]) -> dict[str, Any]:
    accepted = [dict(match) for match in matches if match['id'] in accepted_ids]
    if len(accepted) < 2:
        raise DiagnosticTruthError('fewer than two anchors meet the truth-response contract.')
    expected_array = np.asarray([match['expected_center_m'] for match in accepted], np.float64)
    observed_array = np.asarray([match['observed_center_m'] for match in accepted], np.float64)
    fit = fit_similarity(expected_array, observed_array)
    residuals = fit.pop('residuals_m')
    inliers = fit.pop('inlier_mask')
    for match, residual, inlier in zip(accepted, residuals, inliers):
        match['similarity_residual_m'] = residual
        match['similarity_inlier'] = inlier
    offsets = [match['offset_norm_m'] for match in accepted]
    responses = [match['phase_response'] for match in matches]
    accepted_responses = [match['phase_response'] for match in accepted]
    local_observable = len(residuals) >= 3
    return {
        'anchors': accepted,
        'candidate_anchor_count': len(matches),
        'accepted_anchor_count': len(accepted),
        'rejected_anchor_ids': sorted(
            match['id'] for match in matches if match['id'] not in accepted_ids),
        'absolute_anchor_offset_m': {
            'median': _percentile(offsets, 0.50), 'p95': _percentile(offsets, 0.95),
            'maximum': max(offsets),
        },
        'phase_response': {
            'candidate_median': _percentile(responses, 0.50),
            'candidate_minimum': min(responses),
            'accepted_median': _percentile(accepted_responses, 0.50),
            'accepted_minimum': min(accepted_responses),
        },
        'similarity': {
            **fit,
            'local_residual_observable': local_observable,
            'local_residual_median': _percentile(residuals, 0.50) if local_observable else None,
            'local_residual_p95': _percentile(residuals, 0.95) if local_observable else None,
            'local_residual_maximum': max(residuals) if local_observable else None,
        },
    }


def evaluate_diagnostic_mosaic(mosaic_dir: Path, wall_manifest: Path,
                               output_dir: Path, anchor_padding_m: float = 0.10,
                               minimum_phase_response: float = 0.10) -> dict[str, Any]:
    """Write independent metric evidence for pose-only and optimized mosaics."""
    mosaic_dir = mosaic_dir.resolve()
    wall_manifest = wall_manifest.resolve()
    output_dir = output_dir.resolve()
    if output_dir.exists() or not output_dir.parent.is_dir():
        raise DiagnosticTruthError('output directory must be new with an existing parent.')
    if not math.isfinite(anchor_padding_m) or anchor_padding_m <= 0.0:
        raise DiagnosticTruthError('anchor padding must be positive and finite.')
    if not math.isfinite(minimum_phase_response) or not 0.0 < minimum_phase_response <= 1.0:
        raise DiagnosticTruthError('minimum phase response must be within (0, 1].')
    mosaic_document = _document(mosaic_dir / 'mosaic_manifest.json', 'mosaic manifest')
    if mosaic_document.get('mosaic_format_version') != 3:
        raise DiagnosticTruthError('diagnostic seam evaluation requires mosaic format version 3.')
    wall_document = _document(wall_manifest, 'diagnostic wall manifest')
    diagnostic = wall_document.get('diagnostic_wall')
    if not isinstance(diagnostic, dict) or not isinstance(diagnostic.get('features'), list):
        raise DiagnosticTruthError('wall manifest is not a diagnostic-wall truth source.')
    fusion = mosaic_document.get('fusion')
    if not isinstance(fusion, dict) or 'hard cut' not in str(fusion.get('method', '')).lower():
        raise DiagnosticTruthError('P2.7 diagnostic evaluation requires a hard-cut mosaic.')
    truth_grid, mosaic_grid = _wall_grid(wall_document), _mosaic_grid(mosaic_document)
    tile_size = _positive_integer(
        mosaic_document.get('grid', {}).get('tile_size_px'), 'mosaic tile_size_px')
    anchors = _anchors(diagnostic['features'], truth_grid, mosaic_grid, anchor_padding_m)
    paths = {
        'pose_only': mosaic_dir / 'mosaic_pose_only.tif',
        'optimized': mosaic_dir / 'mosaic_optimized.tif',
    }
    seam_paths = {
        'pose_only': mosaic_dir / 'seams_pose_only.npz',
        'optimized': mosaic_dir / 'seams_optimized.npz',
    }
    coverage_paths = {
        'pose_only': mosaic_dir / 'coverage_pose_only_count.tif',
        'optimized': mosaic_dir / 'coverage_count.tif',
    }
    if any(not path.is_file() for path in coverage_paths.values()):
        raise DiagnosticTruthError('mosaic variant coverage product is absent.')
    blocks = _blocks(wall_document)
    reference = _ReferenceTileReader(
        wall_manifest.parent, blocks, truth_grid, mosaic_grid)
    monitor = _PssMonitor()
    temporary = Path(tempfile.mkdtemp(prefix=f'.{output_dir.name}.tmp-{uuid4().hex}-',
                                      dir=output_dir.parent))
    try:
        raw_matches = {}
        seam_quality = {}
        monitor.start()
        for name, path in paths.items():
            if not path.is_file():
                raise DiagnosticTruthError(f'mosaic product is absent: {path.name}')
            if not seam_paths[name].is_file():
                raise DiagnosticTruthError(
                    f'hard-cut seam product is absent: {seam_paths[name].name}')
            seam_rows, seam_columns, seam_axes = _seam_adjacencies(
                seam_paths[name], mosaic_grid)
            seam_quality[name], crops = _stream_variant(
                path, coverage_paths[name], anchors, reference, seam_rows, seam_columns, seam_axes,
                mosaic_grid, tile_size)
            raw_matches[name] = _variant_matches_from_crops(
                crops, anchors, wall_manifest.parent, blocks, truth_grid, mosaic_grid)
        monitor.stop()
        accepted_ids = set.intersection(*(
            {match['id'] for match in matches
             if match['phase_response'] >= minimum_phase_response}
            for matches in raw_matches.values()))
        if len(accepted_ids) < 2:
            raise DiagnosticTruthError(
                'fewer than two common anchors meet the truth-response contract.')
        variants = {
            name: {**_summarize_variant(matches, accepted_ids),
                   'seam_quality': seam_quality[name]}
            for name, matches in raw_matches.items()
        }
        pose_p95 = variants['pose_only']['absolute_anchor_offset_m']['p95']
        optimized_p95 = variants['optimized']['absolute_anchor_offset_m']['p95']
        summary = {
            'diagnostic_truth_format_version': 3,
            'purpose': ('Post-mosaic visual truth evaluation only; diagnostic wall truth is never '
                        'available to candidate generation, matching, or pose optimization.'),
            'mosaic_dir': str(mosaic_dir),
            'mosaic_manifest_sha256': _sha256(mosaic_dir / 'mosaic_manifest.json'),
            'diagnostic_wall_manifest': str(wall_manifest),
            'diagnostic_wall_manifest_sha256': _sha256(wall_manifest),
            'anchor_rule': ('repair_patch and graffiti_decal features fully inside both grids; '
                            'each comparison crop adds %.3f m padding' % anchor_padding_m),
            'candidate_anchor_count': len(anchors),
            'minimum_phase_response': minimum_phase_response,
            'seam_quality_rule': ('hard-cut source-owner adjacencies are compared with the '
                                  'immutable wall at the same metric-grid pixels; covered '
                                  'same-owner adjacencies in the same raster are the baseline. '
                                  'No structural-edge displacement proxy is reported.'),
            'resource_usage': {
                'peak_process_pss_bytes': monitor.peak_bytes,
                'pss_sample_interval_s': monitor.interval_s,
                'mosaic_decode': 'tiled; no full-wall truth or mosaic raster is retained',
                'reference_tile_cache_blocks': reference.cache_size,
            },
            'common_accepted_anchor_ids': sorted(accepted_ids),
            'variants': variants,
            'optimized_minus_pose_only_p95_anchor_offset_m': optimized_p95 - pose_p95,
            'optimized_not_worse_p95_anchor_offset': optimized_p95 <= pose_p95,
        }
        (temporary / 'diagnostic_truth_summary.json').write_text(
            json.dumps(
                summary, ensure_ascii=False, allow_nan=False, indent=2,
                sort_keys=True) + '\n',
            encoding='utf-8')
        write_stage_provenance(
            temporary, 'diagnostic_truth',
            {'anchor_padding_m': anchor_padding_m,
             'minimum_phase_response': minimum_phase_response},
            {'mosaic_manifest': artifact(mosaic_dir / 'mosaic_manifest.json'),
             'diagnostic_wall_manifest': artifact(wall_manifest)},
            ('diagnostic_truth_summary.json',))
        temporary.replace(output_dir)
        return summary
    except Exception:
        monitor.stop()
        import shutil
        shutil.rmtree(temporary, ignore_errors=True)
        raise
