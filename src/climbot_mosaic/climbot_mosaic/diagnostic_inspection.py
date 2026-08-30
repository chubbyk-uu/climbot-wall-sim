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
from climbot_mosaic.stage_provenance import artifact
from climbot_mosaic.stage_provenance import write_stage_provenance
import cv2
import numpy as np
import tifffile


class DiagnosticInspectionError(ValueError):
    """A native-resolution diagnostic inspection cannot be made safely."""


Bounds = tuple[float, float, float, float]
#: A convex region in wall metres, as the frozen task publishes it.
Polygon = tuple[tuple[float, float], ...]


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
                'bytes': path.stat().st_size,
                'sha256': _sha256(path),
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


def _polygon_json(polygon: Polygon) -> list[list[float]]:
    """Render a polygon as plain JSON numbers, whatever it is made of."""
    return [[float(x), float(y)] for x, y in polygon]


def _bounding_box(polygon: Polygon) -> Bounds:
    """Return the axis-aligned extent of a polygon, for cheap overlap rejection."""
    xs = [x for x, _ in polygon]
    ys = [y for _, y in polygon]
    return (min(xs), min(ys), max(xs), max(ys))


def _counter_clockwise(polygon: Polygon) -> Polygon:
    """Return the polygon wound counter-clockwise, so "inside" has one sign."""
    area = 0.0
    for (first_x, first_y), (second_x, second_y) in zip(polygon, polygon[1:] + polygon[:1]):
        area += first_x * second_y - second_x * first_y
    # Scaled rather than compared to zero: three collinear vertices cancel to a
    # rounding residue, not to 0.0, and a region with no area masks no pixels at
    # all -- which the gate would then read as nothing uncovered.
    box = _bounding_box(polygon)
    reference = max(1.0, (box[2] - box[0]) * (box[3] - box[1]))
    if abs(area) <= 1e-12 * reference:
        raise DiagnosticInspectionError('inspection region has no area.')
    return polygon if area > 0.0 else tuple(reversed(polygon))


def _polygon_mask(grid: MosaicGrid, bounds: Bounds, polygon: Polygon) -> np.ndarray:
    """
    Mark crop pixels whose centre lies inside a convex polygon.

    The region used to be four numbers, which cannot express the trapezoid
    tasks this planner already accepts.  A polygon can, and the frozen task
    publishes one, so nothing has to be flattened to a bounding box on the way
    in -- a bounding box would silently accept ground the task excluded.
    """
    resolution = grid.resolution_m_per_pixel
    x0 = int(round((bounds[0] - grid.min_x_m) / resolution))
    x1 = int(round((bounds[2] - grid.min_x_m) / resolution))
    y0 = int(round((grid.max_y_m - bounds[3]) / resolution))
    y1 = int(round((grid.max_y_m - bounds[1]) / resolution))
    columns = grid.min_x_m + (np.arange(x0, x1, dtype=np.float64) + 0.5) * resolution
    rows = grid.max_y_m - (np.arange(y0, y1, dtype=np.float64) + 0.5) * resolution
    wound = _counter_clockwise(polygon)
    inside = np.ones((rows.size, columns.size), bool)
    for (ax, ay), (bx, by) in zip(wound, wound[1:] + wound[:1]):
        # Left of every directed edge.  The boundary counts as inside, so a
        # pixel centre sitting exactly on the region edge is judged rather than
        # quietly excused.
        inside &= ((bx - ax) * (rows[:, None] - ay)
                   - (by - ay) * (columns[None, :] - ax)) >= 0.0
    return inside


def _union_mask(grid: MosaicGrid, bounds: Bounds, polygons: tuple[Polygon, ...]) -> np.ndarray:
    """Mark crop pixels whose centre lies inside any of several convex polygons."""
    mask = _polygon_mask(grid, bounds, polygons[0])
    for polygon in polygons[1:]:
        mask = mask | _polygon_mask(grid, bounds, polygon)
    return mask


def _convex_hull(points: tuple[tuple[float, float], ...]) -> Polygon:
    """Return the counter-clockwise hull of a point set by monotone chain."""
    ordered = sorted(set(points))
    if len(ordered) < 3:
        raise DiagnosticInspectionError('convex hull needs three distinct points.')

    def half(sequence):
        chain: list[tuple[float, float]] = []
        for point in sequence:
            while len(chain) >= 2:
                (ax, ay), (bx, by) = chain[-2], chain[-1]
                if (bx - ax) * (point[1] - ay) - (by - ay) * (point[0] - ax) > 0.0:
                    break
                chain.pop()
            chain.append(point)
        return chain[:-1]

    return tuple(half(ordered) + half(list(reversed(ordered))))


def _minkowski_rectangle(polygon: Polygon, minimum_x: float, minimum_y: float,
                         maximum_x: float, maximum_y: float) -> Polygon:
    """Grow a convex polygon by an axis-aligned rectangle of camera footprint."""
    corners = ((minimum_x, minimum_y), (maximum_x, minimum_y),
               (maximum_x, maximum_y), (minimum_x, maximum_y))
    return _convex_hull(tuple((x + dx, y + dy) for x, y in polygon for dx, dy in corners))


def _camera_footprint(camera_footprint_m: tuple[float, float, float]
                      ) -> tuple[float, float, float]:
    width, length, offset = camera_footprint_m
    for name, value in (('detection_width_m', width), ('detection_length_m', length)):
        if not math.isfinite(value) or value <= 0.0:
            raise DiagnosticInspectionError(f'{name} must be finite and positive.')
    # Zero is explicitly valid for centred/contact tools. A negative offset is
    # also meaningful for a camera mounted behind base_link.
    if not math.isfinite(offset):
        raise DiagnosticInspectionError('detection_forward_offset_m must be finite.')
    return width, length, offset


def _swept_footprint(first: tuple[float, float], second: tuple[float, float],
                     camera_footprint_m: tuple[float, float, float]) -> Polygon:
    """Exact continuous camera-footprint sweep along one directed SCAN segment."""
    width, length, offset = _camera_footprint(camera_footprint_m)
    dx, dy = second[0] - first[0], second[1] - first[1]
    distance = math.hypot(dx, dy)
    if not math.isfinite(distance) or distance <= 0.0:
        raise DiagnosticInspectionError('planned SCAN segment must have positive finite length.')
    ux, uy = dx / distance, dy / distance
    nx, ny = -uy, ux
    start = (first[0] + offset * ux, first[1] + offset * uy)
    end = (second[0] + offset * ux, second[1] + offset * uy)
    half_length, half_width = length / 2.0, width / 2.0
    return _convex_hull(tuple(
        (point[0] + along * ux + across * nx,
         point[1] + along * uy + across * ny)
        for point, along in ((start, -half_length), (end, half_length))
        for across in (-half_width, half_width)))


def planned_scan_footprints(frozen_tasks: tuple[dict[str, Any], ...]) -> tuple[Polygon, ...]:
    """Return the exact footprint union components of the frozen planned SCANs."""
    polygons = []
    for task in frozen_tasks:
        waypoints = task.get('waypoints_m', ())
        segment_types = task.get('segment_types', ())
        if len(waypoints) != len(segment_types) + 1:
            raise DiagnosticInspectionError('frozen task SCAN geometry is incomplete.')
        for index, kind in enumerate(segment_types):
            if kind == 1:
                polygons.append(_swept_footprint(
                    waypoints[index], waypoints[index + 1], task['camera_footprint_m']))
    if not polygons:
        raise DiagnosticInspectionError('frozen tasks contain no planned SCAN footprint.')
    return tuple(polygons)


def motion_region_camera_envelopes(
        frozen_tasks: tuple[dict[str, Any], ...]) -> tuple[Polygon, ...]:
    """
    Cover camera pixels reachable from a safe centre pose at a task SCAN heading.

    Unlike the former upper bound grown from coverage_region, membership here
    constructs a witness: a base_link centre in the frozen motion_region and a
    forward or reverse heading on that task's sweep axis.
    """
    polygons = []
    for task in frozen_tasks:
        width, length, offset = _camera_footprint(task['camera_footprint_m'])
        half_width, half_length = width / 2.0, length / 2.0
        region = task['motion_region_m']
        sweep = task['sweep_direction']
        for direction in (-1.0, 1.0):
            centre = direction * offset
            if sweep == 1:  # Horizontal SCAN heading.
                rectangle = (centre - half_length, -half_width,
                             centre + half_length, half_width)
            elif sweep == 2:  # Vertical SCAN heading.
                rectangle = (-half_width, centre - half_length,
                             half_width, centre + half_length)
            else:
                raise DiagnosticInspectionError('frozen task has an unsupported sweep direction.')
            polygons.append(_minkowski_rectangle(region, *rectangle))
    if not polygons:
        raise DiagnosticInspectionError('frozen tasks define no safe camera-pose envelope.')
    return tuple(polygons)


def _coverage_summary(coverage: np.ndarray, grid: MosaicGrid, bounds: Bounds,
                      mask: np.ndarray | None = None,
                      inside_region: np.ndarray | None = None,
                      planned_observable: np.ndarray | None = None,
                      safe_pose_observable: np.ndarray | None = None) -> dict[str, int]:
    values = _grid_crop(coverage, grid, bounds)
    if values.dtype != np.uint16:
        raise DiagnosticInspectionError('coverage raster must be uint16.')
    keep = np.ones(values.shape, bool)
    if mask is not None:
        if mask.shape != values.shape:
            raise DiagnosticInspectionError('feature geometry mask does not match coverage crop.')
        keep = mask
    selected = values[keep]
    summary = {
        'pixel_count': int(selected.size),
        'uncovered_pixel_count': int(np.count_nonzero(selected == 0)),
        'single_source_pixel_count': int(np.count_nonzero(selected == 1)),
        'overlap_pixel_count': int(np.count_nonzero(selected >= 2)),
        'maximum_source_count': int(selected.max()) if selected.size else 0,
    }
    if inside_region is not None:
        # Declared feature geometry may extend past the inspection region,
        # and three of this wall's seams span the wall end to end by design, so
        # a total over all declared pixels is false for this wall however well
        # the run went. The gate is the split, reported here rather than
        # derived afterwards. What lies outside the region is not thereby
        # unphotographable; frozen SCAN and safe-pose geometry classify it.
        if inside_region.shape != values.shape:
            raise DiagnosticInspectionError('inspection-region mask does not match coverage crop.')
        uncovered = (values == 0) & keep
        summary['feature_pixels_inside_inspection_region'] = int(
            np.count_nonzero(keep & inside_region))
        summary['uncovered_inside_inspection_region'] = int(
            np.count_nonzero(uncovered & inside_region))
        summary['uncovered_outside_inspection_region'] = int(
            np.count_nonzero(uncovered & ~inside_region))
        if planned_observable is not None:
            if planned_observable.shape != values.shape:
                raise DiagnosticInspectionError(
                    'planned SCAN footprint mask does not match coverage crop.')
            outside = uncovered & ~inside_region
            summary['uncovered_outside_region_inside_planned_scan_footprint'] = int(
                np.count_nonzero(outside & planned_observable))
            summary['uncovered_outside_region_outside_planned_scan_footprint'] = int(
                np.count_nonzero(outside & ~planned_observable))
        if safe_pose_observable is not None:
            if safe_pose_observable.shape != values.shape:
                raise DiagnosticInspectionError(
                    'safe-pose camera envelope mask does not match coverage crop.')
            outside = uncovered & ~inside_region
            summary['uncovered_outside_region_inside_safe_pose_envelope'] = int(
                np.count_nonzero(outside & safe_pose_observable))
            summary['uncovered_outside_region_outside_safe_pose_envelope'] = int(
                np.count_nonzero(outside & ~safe_pose_observable))
    return summary


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
                              tile_size_px: int = 2048,
                              inspection_region_m: Polygon | None = None,
                              camera_footprint_m: tuple[float, float, float] | None = None,
                              frozen_tasks: tuple[dict[str, Any], ...] | None = None,
                              ) -> dict[str, Any]:
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
    if inspection_region_m is not None:
        # A region is what the run is judged against, so every way of supplying
        # one that measures nothing has to fail loudly. A NaN vertex compares
        # false against every pixel and a degenerate ring encloses none, and
        # both used to end in an empty mask -- which the gate then reported as
        # a pass, because nothing uncovered had been found inside nothing.
        if len(inspection_region_m) < 3:
            raise DiagnosticInspectionError('inspection region needs at least three points.')
        if not all(math.isfinite(value) for point in inspection_region_m for value in point):
            raise DiagnosticInspectionError('inspection region has a non-finite vertex.')
        _counter_clockwise(inspection_region_m)
    planned_footprints: tuple[Polygon, ...] | None = None
    safe_pose_envelopes: tuple[Polygon, ...] | None = None
    if camera_footprint_m is not None:
        if inspection_region_m is None:
            raise DiagnosticInspectionError(
                'a camera footprint classifies pixels outside the inspection region, '
                'so the region is required with it.')
        _camera_footprint(camera_footprint_m)
        if not frozen_tasks:
            raise DiagnosticInspectionError(
                'camera reach classification requires the frozen task geometry.')
        planned_footprints = planned_scan_footprints(frozen_tasks)
        safe_pose_envelopes = motion_region_camera_envelopes(frozen_tasks)
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
            if inspection_region_m is not None and _intersection(
                    _bounding_box(inspection_region_m), common_bounds) is None:
                raise DiagnosticInspectionError(
                    'inspection region does not overlap the inspected mosaic.')
            feature_specs = []
            feature_ids: set[str] = set()
            visible_core_pixels = 0
            region_feature_pixels = 0
            uncovered_inside_region = 0
            uncovered_outside_inside_planned = 0
            uncovered_outside_outside_planned = 0
            uncovered_outside_inside_safe_pose = 0
            uncovered_outside_outside_safe_pose = 0
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
                    _feature_mask(raw_feature, mosaic_grid, visible_core),
                    None if inspection_region_m is None else _polygon_mask(
                        mosaic_grid, visible_core, inspection_region_m),
                    None if planned_footprints is None else _union_mask(
                        mosaic_grid, visible_core, planned_footprints),
                    None if safe_pose_envelopes is None else _union_mask(
                        mosaic_grid, visible_core, safe_pose_envelopes))
                visible_core_pixels += coverage_result['pixel_count']
                uncovered_core_pixels += coverage_result['uncovered_pixel_count']
                region_feature_pixels += coverage_result.get(
                    'feature_pixels_inside_inspection_region', 0)
                uncovered_inside_region += coverage_result.get(
                    'uncovered_inside_inspection_region', 0)
                uncovered_outside_inside_planned += coverage_result.get(
                    'uncovered_outside_region_inside_planned_scan_footprint', 0)
                uncovered_outside_outside_planned += coverage_result.get(
                    'uncovered_outside_region_outside_planned_scan_footprint', 0)
                uncovered_outside_inside_safe_pose += coverage_result.get(
                    'uncovered_outside_region_inside_safe_pose_envelope', 0)
                uncovered_outside_outside_safe_pose += coverage_result.get(
                    'uncovered_outside_region_outside_safe_pose_envelope', 0)
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
            coverage_totals: dict[str, Any] = {
                'pixel_count': visible_core_pixels,
                'uncovered_pixel_count': uncovered_core_pixels,
                'all_visible_feature_pixels_covered': uncovered_core_pixels == 0,
            }
            if inspection_region_m is not None:
                if region_feature_pixels == 0:
                    # Zero uncovered pixels inside a region holding no feature
                    # pixels is not a pass, it is a measurement that never
                    # happened. Saying so here is what stops a mistyped or
                    # misplaced region from certifying a run by accident.
                    raise DiagnosticInspectionError(
                        'inspection region contains no declared feature pixels, '
                        'so the coverage gate would pass without measuring anything.')
                # The gate. Only this number can be zero for a wall whose
                # seams run past the region by design; the total never can.
                coverage_totals.update({
                    'feature_pixels_inside_inspection_region': region_feature_pixels,
                    'uncovered_inside_inspection_region': uncovered_inside_region,
                    'uncovered_outside_inspection_region':
                        uncovered_core_pixels - uncovered_inside_region,
                    'all_inspection_region_feature_pixels_covered': uncovered_inside_region == 0,
                })
                if planned_footprints is not None and safe_pose_envelopes is not None:
                    coverage_totals.update({
                        'uncovered_outside_region_inside_planned_scan_footprint':
                            uncovered_outside_inside_planned,
                        'uncovered_outside_region_outside_planned_scan_footprint':
                            uncovered_outside_outside_planned,
                        'uncovered_outside_region_inside_safe_pose_envelope':
                            uncovered_outside_inside_safe_pose,
                        'uncovered_outside_region_outside_safe_pose_envelope':
                            uncovered_outside_outside_safe_pose,
                    })
            summary = {
                'diagnostic_inspection_format_version': 4,
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
                'inspection_region_m': (
                    None if inspection_region_m is None
                    else _polygon_json(inspection_region_m)),
                'camera_footprint_m': (
                    None if camera_footprint_m is None
                    else {'detection_width_m': float(camera_footprint_m[0]),
                          'detection_length_m': float(camera_footprint_m[1]),
                          'detection_forward_offset_m': float(camera_footprint_m[2])}),
                # Where the region and the footprint above came from, named
                # with the digests that tie them to specific archives. Without
                # this the summary states a domain but not its authority.
                'frozen_tasks': [
                    {'task_id': task.get('task_id'),
                     'source_run_id': task.get('source_run_id'),
                     'archive_manifest_sha256': task.get('archive_manifest_sha256'),
                     'processing_manifest_sha256': task.get('processing_manifest_sha256')}
                    for task in (frozen_tasks or ())],
                'planned_scan_footprints_m': (
                    None if planned_footprints is None
                    else [_polygon_json(polygon) for polygon in planned_footprints]),
                'safe_pose_camera_envelopes_m': (
                    None if safe_pose_envelopes is None
                    else [_polygon_json(polygon) for polygon in safe_pose_envelopes]),
                'visible_feature_coverage': coverage_totals,
                'features': records,
            }
            (temporary / 'diagnostic_inspection_summary.json').write_text(
                json.dumps(summary, ensure_ascii=False, allow_nan=False, indent=2,
                           sort_keys=True) + '\n', encoding='utf-8')
            write_stage_provenance(
                temporary, 'diagnostic_inspection',
                {'padding_m': padding_m, 'tile_size_px': tile_size_px,
                 'inspection_region_m': summary['inspection_region_m'],
                 'camera_footprint_m': summary['camera_footprint_m']},
                {'mosaic_manifest': artifact(mosaic_dir / 'mosaic_manifest.json'),
                 'diagnostic_wall_manifest': artifact(wall_manifest),
                 'frozen_tasks': list(frozen_tasks or ())},
                ('diagnostic_inspection_summary.json',))
            temporary.replace(output_dir)
            return summary
        except Exception:
            shutil.rmtree(temporary, ignore_errors=True)
            raise
    except DiagnosticTruthError as error:
        raise DiagnosticInspectionError(str(error)) from error
