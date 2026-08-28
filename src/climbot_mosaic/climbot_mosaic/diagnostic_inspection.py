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

"""Native-resolution visual inspection evidence for a diagnostic-wall mosaic."""

from __future__ import annotations

import json
import math
from pathlib import Path
import re
import shutil
import tempfile
from typing import Any
from uuid import uuid4

from climbot_mosaic.diagnostic_truth import (
    _blocks,
    _document,
    _feature_bounds,
    _finite,
    _mosaic_crop,
    _mosaic_grid,
    _reference_crop,
    _sha256,
    _wall_grid,
    DiagnosticTruthError,
    MosaicGrid,
    TruthGrid,
)
import cv2
import numpy as np
import tifffile


class DiagnosticInspectionError(ValueError):
    """A native-resolution diagnostic inspection cannot be made safely."""


Bounds = tuple[float, float, float, float]


def _safe_feature_id(value: Any) -> str:
    """Return a bounded portable directory component for one declared feature."""
    if not isinstance(value, str) or len(value) > 128 or re.fullmatch(
            r'[A-Za-z0-9][A-Za-z0-9._-]*', value) is None:
        raise DiagnosticInspectionError('diagnostic feature id is not a safe file name.')
    return value


def _register_feature_id(feature: Any, existing: set[str]) -> str:
    """Validate and reserve one feature id so output directories cannot collide."""
    if not isinstance(feature, dict):
        raise DiagnosticInspectionError('diagnostic feature is malformed.')
    feature_id = _safe_feature_id(feature.get('id'))
    if feature_id in existing:
        raise DiagnosticInspectionError(f'diagnostic feature id is duplicated: {feature_id}.')
    existing.add(feature_id)
    return feature_id


def _intersection(first: Bounds, second: Bounds) -> Bounds | None:
    result = (max(first[0], second[0]), max(first[1], second[1]),
              min(first[2], second[2]), min(first[3], second[3]))
    return result if result[0] < result[2] and result[1] < result[3] else None


def _grid_bounds(grid: TruthGrid | MosaicGrid) -> Bounds:
    if isinstance(grid, TruthGrid):
        return (grid.origin_x_m, grid.origin_y_m,
                grid.origin_x_m + grid.width_m, grid.origin_y_m + grid.height_m)
    return (grid.min_x_m, grid.min_y_m, grid.max_x_m, grid.max_y_m)


def _expand(bounds: Bounds, padding_m: float) -> Bounds:
    return (bounds[0] - padding_m, bounds[1] - padding_m,
            bounds[2] + padding_m, bounds[3] + padding_m)


def _pad_to_shape(image: np.ndarray, height: int, width: int) -> np.ndarray:
    """Pad a crop for side-by-side display without moving or resampling a pixel."""
    result = np.zeros((height, width), np.uint8)
    result[:image.shape[0], :image.shape[1]] = image
    return result


def _write_native_tiles(output_dir: Path, feature_id: str, reference: np.ndarray,
                        pose_only: np.ndarray, optimized: np.ndarray,
                        tile_size_px: int) -> list[dict[str, Any]]:
    """Write unscaled truth|pose|optimized comparison tiles for one feature."""
    height = max(reference.shape[0], pose_only.shape[0], optimized.shape[0])
    width = max(reference.shape[1], pose_only.shape[1], optimized.shape[1])
    reference = _pad_to_shape(reference, height, width)
    pose_only = _pad_to_shape(pose_only, height, width)
    optimized = _pad_to_shape(optimized, height, width)
    feature_dir = output_dir / 'native_tiles' / feature_id
    feature_dir.mkdir(parents=True, exist_ok=True)
    tiles = []
    for row in range(0, height, tile_size_px):
        for column in range(0, width, tile_size_px):
            path = feature_dir / f'r{row // tile_size_px:04d}_c{column // tile_size_px:04d}.png'
            tile = np.hstack((
                reference[row:row + tile_size_px, column:column + tile_size_px],
                pose_only[row:row + tile_size_px, column:column + tile_size_px],
                optimized[row:row + tile_size_px, column:column + tile_size_px]))
            if not cv2.imwrite(str(path), tile, (cv2.IMWRITE_PNG_COMPRESSION, 3)):
                raise DiagnosticInspectionError(f'cannot write native inspection tile: {path}')
            tiles.append({
                'file': str(path.relative_to(output_dir)),
                'row_px': row,
                'column_px': column,
                'height_px': int(tile.shape[0]),
                'panel_width_px': int(tile.shape[1] // 3),
            })
    return tiles


def _feature_mask(feature: dict[str, Any], grid: MosaicGrid, bounds: Bounds) -> np.ndarray:
    """Return native pixel centres that belong to a declared diagnostic feature."""
    x0 = int(round((bounds[0] - grid.min_x_m) / grid.resolution_m_per_pixel))
    x1 = int(round((bounds[2] - grid.min_x_m) / grid.resolution_m_per_pixel))
    y0 = int(round((grid.max_y_m - bounds[3]) / grid.resolution_m_per_pixel))
    y1 = int(round((grid.max_y_m - bounds[1]) / grid.resolution_m_per_pixel))
    if x0 < 0 or y0 < 0 or x1 > grid.width_px or y1 > grid.height_px or x1 <= x0 or y1 <= y0:
        raise DiagnosticInspectionError('requested feature mask is outside the mosaic grid.')
    height, width = y1 - y0, x1 - x0
    x_values = bounds[0] + (np.arange(width, dtype=np.float32) + 0.5) * \
        grid.resolution_m_per_pixel
    y_values = bounds[3] - (np.arange(height, dtype=np.float32) + 0.5) * \
        grid.resolution_m_per_pixel
    kind = feature.get('kind')
    if kind in ('crack_decal', 'graffiti_decal'):
        center = feature.get('center_m')
        size = feature.get('size_m')
        if not isinstance(center, list) or not isinstance(size, list) or \
                len(center) != 2 or len(size) != 2:
            raise DiagnosticInspectionError('decal feature geometry is malformed.')
        center_x, center_y = float(center[0]), float(center[1])
        half_x, half_y = float(size[0]) / 2.0, float(size[1]) / 2.0
        angle = math.radians(_finite(feature.get('angle_deg', 0.0), 'decal angle_deg'))
        cosine, sine = math.cos(angle), math.sin(angle)
        dx = x_values[None, :] - center_x
        dy = y_values[:, None] - center_y
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        return (np.abs(local_x) <= half_x) & (np.abs(local_y) <= half_y)
    if kind == 'repair_patch':
        polygon = feature.get('polygon_m')
        if not isinstance(polygon, list) or len(polygon) < 3:
            raise DiagnosticInspectionError('repair patch polygon is malformed.')
        points = np.asarray(polygon, np.float64)
        if points.shape != (len(polygon), 2) or not np.isfinite(points).all():
            raise DiagnosticInspectionError('repair patch polygon is malformed.')
        mask = np.zeros((height, width), np.uint8)
        pixels = np.column_stack(((points[:, 0] - bounds[0]) / grid.resolution_m_per_pixel,
                                  (bounds[3] - points[:, 1]) / grid.resolution_m_per_pixel))
        cv2.fillPoly(mask, [np.rint(pixels).astype(np.int32)], 1)
        return mask.astype(bool)
    if kind == 'construction_seam':
        points = feature.get('points_m')
        width_m = _finite(feature.get('width_m'), 'construction seam width_m')
        if not isinstance(points, list) or len(points) < 2 or width_m <= 0.0:
            raise DiagnosticInspectionError('construction seam geometry is malformed.')
        mask = np.zeros((height, width), np.uint8)
        pixels = np.asarray([
            ((float(point[0]) - bounds[0]) / grid.resolution_m_per_pixel,
             (bounds[3] - float(point[1])) / grid.resolution_m_per_pixel)
            for point in points], np.float64)
        if pixels.shape != (len(points), 2) or not np.isfinite(pixels).all():
            raise DiagnosticInspectionError('construction seam points are malformed.')
        cv2.polylines(mask, [np.rint(pixels).astype(np.int32)], False, 1,
                      max(1, int(round(width_m / grid.resolution_m_per_pixel))))
        return mask.astype(bool)
    raise DiagnosticInspectionError(f'unsupported diagnostic feature kind: {kind!r}.')


def _coverage_summary(coverage: np.ndarray, grid: MosaicGrid, bounds: Bounds,
                      mask: np.ndarray | None = None) -> dict[str, int]:
    values = _grid_crop(coverage, grid, bounds)
    if values.dtype != np.uint16:
        raise DiagnosticInspectionError('coverage raster must be uint16.')
    if mask is not None:
        if mask.shape != values.shape:
            raise DiagnosticInspectionError('feature geometry mask does not match coverage crop.')
        values = values[mask]
    return {
        'pixel_count': int(values.size),
        'uncovered_pixel_count': int(np.count_nonzero(values == 0)),
        'single_source_pixel_count': int(np.count_nonzero(values == 1)),
        'overlap_pixel_count': int(np.count_nonzero(values >= 2)),
        'maximum_source_count': int(values.max()) if values.size else 0,
    }


def _grid_crop(image: np.ndarray, grid: MosaicGrid, bounds: Bounds) -> np.ndarray:
    """Crop any one-channel raster on a declared mosaic grid without resampling."""
    if image.ndim != 2 or image.shape != (grid.height_px, grid.width_px):
        raise DiagnosticInspectionError('raster dimensions disagree with its grid.')
    x0 = int(round((bounds[0] - grid.min_x_m) / grid.resolution_m_per_pixel))
    x1 = int(round((bounds[2] - grid.min_x_m) / grid.resolution_m_per_pixel))
    y0 = int(round((grid.max_y_m - bounds[3]) / grid.resolution_m_per_pixel))
    y1 = int(round((grid.max_y_m - bounds[1]) / grid.resolution_m_per_pixel))
    if x0 < 0 or y0 < 0 or x1 > grid.width_px or y1 > grid.height_px or x1 <= x0 or y1 <= y0:
        raise DiagnosticInspectionError('requested crop is outside the mosaic grid.')
    return image[y0:y1, x0:x1]


def _quality(manifest: dict[str, Any]) -> dict[str, Any]:
    value = manifest.get('quality')
    if not isinstance(value, dict) or not isinstance(value.get('optimized'), dict) or \
            not isinstance(value.get('pose_only'), dict):
        raise DiagnosticInspectionError('mosaic manifest lacks full-resolution overlap quality.')
    return value


def _validate_mosaic(manifest: dict[str, Any]) -> MosaicGrid:
    fusion = manifest.get('fusion')
    if not isinstance(fusion, dict) or 'hard cut' not in str(fusion.get('method', '')).lower():
        raise DiagnosticInspectionError('diagnostic inspection requires a hard-cut mosaic.')
    return _mosaic_grid(manifest)


def inspect_diagnostic_mosaic(mosaic_dir: Path, wall_manifest: Path, output_dir: Path,
                              padding_m: float = 0.05,
                              tile_size_px: int = 2048) -> dict[str, Any]:
    """
    Write 100%-scale visual evidence for every diagnostic feature intersecting a mosaic.

    This is deliberately a post-mosaic operation.  It only reads finished products and
    immutable wall truth, retains a one-to-one pixel scale, and does not report a visual
    crop outside the actual mosaic domain as though it had been inspected.
    """
    mosaic_dir = mosaic_dir.resolve()
    wall_manifest = wall_manifest.resolve()
    output_dir = output_dir.resolve()
    if output_dir.exists() or not output_dir.parent.is_dir():
        raise DiagnosticInspectionError('output directory must be new with an existing parent.')
    if not math.isfinite(padding_m) or padding_m < 0.0:
        raise DiagnosticInspectionError('padding must be finite and non-negative.')
    if tile_size_px < 128 or tile_size_px % 16:
        raise DiagnosticInspectionError('tile size must be at least 128 and divisible by 16.')
    try:
        manifest = _document(mosaic_dir / 'mosaic_manifest.json', 'mosaic manifest')
        wall_document = _document(wall_manifest, 'diagnostic wall manifest')
        diagnostic = wall_document.get('diagnostic_wall')
        if not isinstance(diagnostic, dict) or not isinstance(diagnostic.get('features'), list):
            raise DiagnosticInspectionError('wall manifest is not a diagnostic-wall truth source.')
        mosaic_grid = _validate_mosaic(manifest)
        truth_grid = _wall_grid(wall_document)
        if abs(truth_grid.scale_m_per_px - mosaic_grid.resolution_m_per_pixel) > 1e-12:
            raise DiagnosticInspectionError('truth and mosaic resolutions must match.')
        paths = {
            'pose_only': mosaic_dir / 'mosaic_pose_only.tif',
            'optimized': mosaic_dir / 'mosaic_optimized.tif',
            'coverage': mosaic_dir / 'coverage_count.tif',
        }
        if any(not path.is_file() for path in paths.values()):
            raise DiagnosticInspectionError('mosaic product required for inspection is absent.')
        temporary = Path(tempfile.mkdtemp(prefix=f'.{output_dir.name}.tmp-{uuid4().hex}-',
                                          dir=output_dir.parent))
        try:
            coverage = tifffile.imread(paths['coverage'])
            expected_shape = (mosaic_grid.height_px, mosaic_grid.width_px)
            if coverage.shape != expected_shape:
                raise DiagnosticInspectionError(
                    'coverage raster dimensions disagree with its grid.')
            common_bounds = _intersection(_grid_bounds(truth_grid), _grid_bounds(mosaic_grid))
            if common_bounds is None:
                raise DiagnosticInspectionError(
                    'diagnostic wall and mosaic have no common extent.')
            feature_specs = []
            feature_ids: set[str] = set()
            visible_core_pixels = 0
            uncovered_core_pixels = 0
            for raw_feature in diagnostic['features']:
                feature_id = _register_feature_id(raw_feature, feature_ids)
                core_bounds = _feature_bounds(raw_feature)
                visible_core = _intersection(core_bounds, common_bounds)
                record: dict[str, Any] = {
                    'id': feature_id,
                    'kind': raw_feature.get('kind'),
                    'feature_bounds_m': list(core_bounds),
                }
                if visible_core is None:
                    record['visibility'] = 'outside_mosaic_domain'
                    feature_specs.append((record, None, None))
                    continue
                crop_bounds = _intersection(_expand(core_bounds, padding_m), common_bounds)
                if crop_bounds is None:  # Protected by visible_core, retained as a clear contract.
                    raise DiagnosticInspectionError('visible feature has no crop extent.')
                coverage_result = _coverage_summary(
                    coverage, mosaic_grid, visible_core,
                    _feature_mask(raw_feature, mosaic_grid, visible_core))
                visible_core_pixels += coverage_result['pixel_count']
                uncovered_core_pixels += coverage_result['uncovered_pixel_count']
                record.update({
                    'visibility': ('fully_visible' if visible_core == core_bounds
                                   else 'partially_visible'),
                    'visible_feature_bounds_m': list(visible_core),
                    'crop_bounds_m': list(crop_bounds),
                    'coverage': coverage_result,
                })
                feature_specs.append((record, crop_bounds, feature_id))
            del coverage
            pose_only = tifffile.imread(paths['pose_only'])
            optimized = tifffile.imread(paths['optimized'])
            if pose_only.dtype != np.uint8 or optimized.dtype != np.uint8 or \
                    pose_only.shape != expected_shape or optimized.shape != expected_shape:
                raise DiagnosticInspectionError('mosaic albedo products disagree with their grid.')
            records = []
            for record, crop_bounds, feature_id in feature_specs:
                if crop_bounds is not None and feature_id is not None:
                    reference = _reference_crop(
                        wall_manifest.parent, _blocks(wall_document), truth_grid, crop_bounds)
                    pose_crop = _mosaic_crop(pose_only, mosaic_grid, crop_bounds)
                    optimized_crop = _mosaic_crop(optimized, mosaic_grid, crop_bounds)
                    record['native_tiles'] = _write_native_tiles(
                        temporary, feature_id, reference, pose_crop, optimized_crop, tile_size_px)
                records.append(record)
            visible = [record for record in records
                       if record['visibility'] != 'outside_mosaic_domain']
            summary = {
                'diagnostic_inspection_format_version': 1,
                'purpose': ('Post-mosaic 100%-scale visual inspection only; immutable diagnostic '
                            'wall truth is never available to candidate generation, matching, or '
                            'pose optimization.'),
                'mosaic_dir': str(mosaic_dir),
                'mosaic_manifest_sha256': _sha256(mosaic_dir / 'mosaic_manifest.json'),
                'diagnostic_wall_manifest': str(wall_manifest),
                'diagnostic_wall_manifest_sha256': _sha256(wall_manifest),
                'pixel_scale_m_per_pixel': mosaic_grid.resolution_m_per_pixel,
                'native_tile_contract': {
                    'layout': 'left-to-right: immutable_truth, pose_only, optimized',
                    'resampled': False,
                    'tile_size_px': tile_size_px,
                    'padding_m': padding_m,
                },
                'full_resolution_overlap_quality': _quality(manifest),
                'feature_counts': {
                    'declared': len(records),
                    'intersecting_mosaic_domain': len(visible),
                    'fully_visible': sum(
                        record['visibility'] == 'fully_visible' for record in visible),
                    'partially_visible': sum(
                        record['visibility'] == 'partially_visible' for record in visible),
                    'outside_mosaic_domain': len(records) - len(visible),
                },
                'visible_feature_coverage': {
                    'pixel_count': visible_core_pixels,
                    'uncovered_pixel_count': uncovered_core_pixels,
                    'all_visible_feature_pixels_covered': uncovered_core_pixels == 0,
                },
                'features': records,
            }
            (temporary / 'diagnostic_inspection_summary.json').write_text(
                json.dumps(summary, ensure_ascii=False, allow_nan=False, indent=2,
                           sort_keys=True) + '\n', encoding='utf-8')
            temporary.replace(output_dir)
            return summary
        except Exception:
            shutil.rmtree(temporary, ignore_errors=True)
            raise
    except DiagnosticTruthError as error:
        raise DiagnosticInspectionError(str(error)) from error
