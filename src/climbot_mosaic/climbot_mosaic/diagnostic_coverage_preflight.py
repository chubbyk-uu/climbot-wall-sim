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

"""Offline, discrete-exposure coverage preflight for a diagnostic wall."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any

from climbot_mosaic.diagnostic_inspection import _feature_mask, _intersection
from climbot_mosaic.diagnostic_truth import (
    _document,
    _feature_bounds,
    MosaicGrid,
)
import numpy as np
import yaml


class DiagnosticCoveragePreflightError(ValueError):
    """A task cannot yield a trustworthy diagnostic coverage forecast."""


Bounds = tuple[float, float, float, float]


@dataclass(frozen=True)
class ScanSegment:
    """One planned base-link SCAN segment in execution order."""

    start: tuple[float, float]
    end: tuple[float, float]


@dataclass(frozen=True)
class Exposure:
    """One contractual camera-centre footprint predicted before a run."""

    center: tuple[float, float]
    heading_rad: float
    segment_index: int
    trigger_index: int


def _number(value: Any, name: str, *, positive: bool = False) -> float:
    if isinstance(value, bool):
        raise DiagnosticCoveragePreflightError(f'{name} must be numeric.')
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise DiagnosticCoveragePreflightError(f'{name} must be numeric.') from error
    if not math.isfinite(result) or (positive and result <= 0.0):
        qualifier = 'positive and finite' if positive else 'finite'
        raise DiagnosticCoveragePreflightError(f'{name} must be {qualifier}.')
    return result


def _pair(value: Any, name: str) -> tuple[float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise DiagnosticCoveragePreflightError(f'{name} must contain two coordinates.')
    return (_number(value[0], f'{name}[0]'), _number(value[1], f'{name}[1]'))


def _rectangle_parameters(task: dict[str, Any]) -> tuple[Bounds, str, str, float]:
    if task.get('region_type') != 'rectangle':
        raise DiagnosticCoveragePreflightError('only rectangular diagnostic tasks are supported.')
    lower_left = _pair(task.get('lower_left'), 'lower_left')
    upper_right = _pair(task.get('upper_right'), 'upper_right')
    if upper_right[0] <= lower_left[0] or upper_right[1] <= lower_left[1]:
        raise DiagnosticCoveragePreflightError(
            'rectangle upper_right must be above and right of lower_left.')
    direction = task.get('sweep_direction')
    if direction not in ('horizontal', 'vertical'):
        raise DiagnosticCoveragePreflightError('sweep_direction must be horizontal or vertical.')
    corner = task.get('start_corner', 'lower_left')
    if corner not in ('lower_left', 'lower_right', 'upper_left', 'upper_right'):
        raise DiagnosticCoveragePreflightError('start_corner is unsupported.')
    overlap = _number(task.get('overlap_ratio', 0.20), 'overlap_ratio')
    if not 0.0 <= overlap < 1.0:
        raise DiagnosticCoveragePreflightError('overlap_ratio must be within [0, 1).')
    return ((lower_left[0], lower_left[1], upper_right[0], upper_right[1]),
            direction, corner, overlap)


def _safe_bounds(robot: dict[str, Any], wall: dict[str, Any]) -> Bounds:
    footprint = robot.get('robot', {}).get('footprint', {})
    surface = wall.get('wall', {}).get('surface', {})
    robot_length = _number(footprint.get('length_m'), 'robot footprint length_m', positive=True)
    robot_width = _number(footprint.get('width_m'), 'robot footprint width_m', positive=True)
    clearance = _number(footprint.get('edge_clearance_m'), 'robot footprint edge_clearance_m')
    if clearance < 0.0:
        raise DiagnosticCoveragePreflightError(
            'robot footprint edge_clearance_m must be non-negative.')
    width = _number(surface.get('width_m'), 'wall width_m', positive=True)
    height = _number(surface.get('height_m'), 'wall height_m', positive=True)
    margin = 0.5 * math.hypot(robot_length, robot_width) + clearance
    if 2.0 * margin >= min(width, height):
        raise DiagnosticCoveragePreflightError('robot safety margin removes the wall work region.')
    return (margin, margin, width - margin, height - margin)


def _contains(container: Bounds, candidate: Bounds) -> bool:
    return all((candidate[0] >= container[0] - 1e-9,
                candidate[1] >= container[1] - 1e-9,
                candidate[2] <= container[2] + 1e-9,
                candidate[3] <= container[3] + 1e-9))


def _maneuver_safe_route_bounds(task: dict[str, Any], target: Bounds,
                                safe: Bounds) -> Bounds:
    """Mirror the planner's symmetric translation envelope for rectangles."""
    margin = _number(
        task.get('maneuver_boundary_margin_m', 0.10),
        'maneuver_boundary_margin_m')
    if margin < 0.0:
        raise DiagnosticCoveragePreflightError(
            'maneuver_boundary_margin_m must be non-negative.')
    direction = _pair(
        task.get('maneuver_drift_direction', [0.0, -1.0]),
        'maneuver_drift_direction')
    norm = math.hypot(*direction)
    if margin > 0.0 and norm <= 1e-9:
        raise DiagnosticCoveragePreflightError(
            'maneuver_drift_direction must be non-zero when the margin is positive.')
    translation = (0.0, 0.0) if margin == 0.0 else (
        margin * direction[0] / norm, margin * direction[1] / norm)
    maneuver_safe = (
        safe[0] + abs(translation[0]), safe[1] + abs(translation[1]),
        safe[2] - abs(translation[0]), safe[3] - abs(translation[1]))
    route = (
        max(target[0], maneuver_safe[0]), max(target[1], maneuver_safe[1]),
        min(target[2], maneuver_safe[2]), min(target[3], maneuver_safe[3]))
    if route[2] <= route[0] or route[3] <= route[1]:
        raise DiagnosticCoveragePreflightError(
            'selected region has no area inside the maneuver-safe envelope.')
    return route


def planned_scan_segments(task: dict[str, Any],
                          detection_width_m: float,
                          route_bounds: Bounds | None = None) -> tuple[ScanSegment, ...]:
    """Mirror the rectangular branch of coverage_geometry.cpp exactly."""
    bounds, direction, corner, overlap = _rectangle_parameters(task)
    if route_bounds is not None:
        bounds = route_bounds
    width = _number(detection_width_m, 'detection_width_m', positive=True)
    horizontal = direction == 'horizontal'
    cross_low, cross_high = (bounds[1], bounds[3]) if horizontal else (bounds[0], bounds[2])
    span = cross_high - cross_low
    row_spacing = width * (1.0 - overlap)
    cross_inset = min(0.5 * span, 0.5 * width)
    usable_span = max(0.0, span - 2.0 * cross_inset)
    line_count = max(1, math.ceil(usable_span / row_spacing - 1e-9) + 1)
    spacing = 0.0 if line_count == 1 else usable_span / (line_count - 1)
    start_low = corner.startswith('lower_')
    start_left = corner.endswith('left')
    reverse_order = (not start_low) if horizontal else (not start_left)
    segments = []
    for line_index in range(line_count):
        ordered = line_count - 1 - line_index if reverse_order else line_index
        coordinate = 0.5 * (cross_low + cross_high) if line_count == 1 else \
            cross_low + cross_inset + spacing * ordered
        if horizontal:
            start, end = (bounds[0], coordinate), (bounds[2], coordinate)
            forward = start_left
        else:
            start, end = (coordinate, bounds[1]), (coordinate, bounds[3])
            forward = start_low
        if line_index % 2:
            forward = not forward
        segments.append(ScanSegment(start, end) if forward else ScanSegment(end, start))
    return tuple(segments)


def planned_exposures(segments: tuple[ScanSegment, ...], effective_length_m: float,
                      image_overlap_ratio: float, forward_offset_m: float) -> tuple[Exposure, ...]:
    """Mirror automatic_capture_node's count, interval, and first target contract."""
    length = _number(effective_length_m, 'effective_length_m', positive=True)
    overlap = _number(image_overlap_ratio, 'image_overlap_ratio')
    offset = _number(forward_offset_m, 'forward_offset_m')
    if not 0.0 <= overlap < 1.0 or offset < 0.0:
        raise DiagnosticCoveragePreflightError('capture overlap/forward offset is invalid.')
    maximum_spacing = length * (1.0 - overlap)
    exposures = []
    for segment_index, segment in enumerate(segments):
        dx, dy = segment.end[0] - segment.start[0], segment.end[1] - segment.start[1]
        route_length = math.hypot(dx, dy)
        if route_length <= 1e-6:
            raise DiagnosticCoveragePreflightError('planned SCAN has zero length.')
        count = max(1, math.ceil(route_length / maximum_spacing))
        interval = route_length / count
        tangent = (dx / route_length, dy / route_length)
        heading = math.atan2(tangent[1], tangent[0])
        for trigger_index in range(count):
            progress = offset + interval * trigger_index
            exposures.append(Exposure(
                (segment.start[0] + progress * tangent[0],
                 segment.start[1] + progress * tangent[1]),
                heading, segment_index, trigger_index))
    return tuple(exposures)


def _coverage_counts(bounds: Bounds, resolution_m: float, exposures: tuple[Exposure, ...],
                     footprint_width_m: float, footprint_length_m: float) -> np.ndarray:
    width = max(1, int(round((bounds[2] - bounds[0]) / resolution_m)))
    height = max(1, int(round((bounds[3] - bounds[1]) / resolution_m)))
    x = bounds[0] + (np.arange(width, dtype=np.float64) + 0.5) * resolution_m
    y = bounds[3] - (np.arange(height, dtype=np.float64) + 0.5) * resolution_m
    counts = np.zeros((height, width), np.uint16)
    half_width, half_length = 0.5 * footprint_width_m, 0.5 * footprint_length_m
    for exposure in exposures:
        cosine, sine = math.cos(exposure.heading_rad), math.sin(exposure.heading_rad)
        extent_x = abs(cosine) * half_length + abs(sine) * half_width
        extent_y = abs(sine) * half_length + abs(cosine) * half_width
        disjoint = (
            exposure.center[0] + extent_x < bounds[0] or
            exposure.center[0] - extent_x > bounds[2] or
            exposure.center[1] + extent_y < bounds[1] or
            exposure.center[1] - extent_y > bounds[3])
        if disjoint:
            continue
        dx = x[None, :] - exposure.center[0]
        dy = y[:, None] - exposure.center[1]
        covered = ((np.abs(cosine * dx + sine * dy) <= half_length + 1e-12) &
                   (np.abs(-sine * dx + cosine * dy) <= half_width + 1e-12))
        counts += covered
    return counts


def _feature_coverage(feature: dict[str, Any], target: Bounds, exposures: tuple[Exposure, ...],
                      footprint_width_m: float, footprint_length_m: float,
                      resolution_m: float) -> dict[str, Any]:
    bounds = _intersection(_feature_bounds(feature), target)
    record = {'id': feature.get('id'), 'kind': feature.get('kind')}
    if bounds is None:
        record.update({'intersects_target': False, 'sample_count': 0,
                       'uncovered_sample_count': 0, 'fully_covered': True})
        return record
    width = int(round((bounds[2] - bounds[0]) / resolution_m))
    height = int(round((bounds[3] - bounds[1]) / resolution_m))
    if min(width, height) <= 0:
        raise DiagnosticCoveragePreflightError(
            f"feature {feature.get('id')!r} is too small to sample.")
    grid = MosaicGrid(bounds[0], bounds[1], bounds[2], bounds[3], resolution_m, width, height)
    mask = _feature_mask(feature, grid, bounds)
    counts = _coverage_counts(
        bounds, resolution_m, exposures, footprint_width_m, footprint_length_m)
    values = counts[mask]
    if values.size == 0:
        raise DiagnosticCoveragePreflightError(
            f"feature {feature.get('id')!r} has empty declared geometry.")
    record.update({
        'intersects_target': True,
        'target_bounds_m': [float(value) for value in bounds],
        'sample_count': int(values.size),
        'uncovered_sample_count': int(np.count_nonzero(values == 0)),
        'single_exposure_sample_count': int(np.count_nonzero(values == 1)),
        'overlap_sample_count': int(np.count_nonzero(values >= 2)),
        'fully_covered': bool(np.all(values > 0)),
    })
    return record


def preflight_diagnostic_coverage(task: dict[str, Any], wall_manifest: dict[str, Any],
                                  camera: dict[str, Any], robot: dict[str, Any],
                                  wall: dict[str, Any],
                                  resolution_m: float = 0.001) -> dict[str, Any]:
    """Predict per-feature coverage from the exact planned, discrete camera exposures."""
    resolution = _number(resolution_m, 'resolution_m', positive=True)
    target, direction, _, _ = _rectangle_parameters(task)
    safe = _safe_bounds(robot, wall)
    if not _contains(safe, target):
        raise DiagnosticCoveragePreflightError(
            'task rectangle lies outside the green wall-safe region.')
    route = _maneuver_safe_route_bounds(task, target, safe)
    footprint = camera.get('inspection_camera', {}).get('footprint', {})
    mount = camera.get('inspection_camera', {}).get('optical_mount', {})
    effective_width = _number(
        footprint.get('effective_width_m'), 'camera effective_width_m', positive=True)
    effective_length = _number(
        footprint.get('effective_length_m'), 'camera effective_length_m', positive=True)
    mount_xyz = mount.get('center_xyz_m')
    if not isinstance(mount_xyz, (list, tuple)) or len(mount_xyz) != 3:
        raise DiagnosticCoveragePreflightError(
            'camera optical_mount.center_xyz_m must contain three coordinates.')
    forward_offset = _number(mount_xyz[0], 'camera forward mount offset')
    segments = planned_scan_segments(task, effective_width, route)
    exposures = planned_exposures(segments, effective_length, 0.20, forward_offset)
    diagnostic = wall_manifest.get('diagnostic_wall')
    if not isinstance(diagnostic, dict) or not isinstance(diagnostic.get('features'), list):
        raise DiagnosticCoveragePreflightError(
            'wall manifest is not a diagnostic-wall truth source.')
    records = [_feature_coverage(
        feature, target, exposures, effective_width, effective_length, resolution)
               for feature in diagnostic['features']]
    intersecting = [record for record in records if record['intersects_target']]
    uncovered = sum(record['uncovered_sample_count'] for record in intersecting)
    return {
        'schema_version': 1,
        'model': {
            'description': (
                'planned rectangular SCANs plus automatic_capture_node discrete trigger '
                'contract; declared feature support sampled at %.6f m' % resolution),
            'feature_resolution_m': resolution,
            'camera_footprint_m': [effective_width, effective_length],
            'camera_forward_offset_m': forward_offset,
            'image_overlap_ratio': 0.20,
        },
        'task': {
            'task_id': task.get('task_id'), 'sweep_direction': direction,
            'drive_rectangle_m': list(target), 'green_safe_rectangle_m': list(safe),
            'maneuver_safe_route_rectangle_m': list(route),
            'scan_segment_count': len(segments), 'exposure_count': len(exposures),
        },
        'feature_coverage': {
            'intersecting_feature_count': len(intersecting),
            'all_intersecting_feature_samples_covered': uncovered == 0,
            'uncovered_sample_count': uncovered,
            'features': records,
        },
    }


def preflight_diagnostic_coverage_set(tasks: tuple[dict[str, Any], ...],
                                      wall_manifest: dict[str, Any], camera: dict[str, Any],
                                      robot: dict[str, Any], wall: dict[str, Any],
                                      resolution_m: float = 0.001) -> dict[str, Any]:
    """Evaluate the union of complementary tasks over one identical target rectangle."""
    if not tasks:
        raise DiagnosticCoveragePreflightError(
            'a diagnostic preflight set needs at least one task.')
    resolution = _number(resolution_m, 'resolution_m', positive=True)
    target, _, _, _ = _rectangle_parameters(tasks[0])
    if any(_rectangle_parameters(task)[0] != target for task in tasks[1:]):
        raise DiagnosticCoveragePreflightError(
            'all complementary tasks must use the same drive rectangle.')
    safe = _safe_bounds(robot, wall)
    if not _contains(safe, target):
        raise DiagnosticCoveragePreflightError(
            'task rectangle lies outside the green wall-safe region.')
    footprint = camera.get('inspection_camera', {}).get('footprint', {})
    mount = camera.get('inspection_camera', {}).get('optical_mount', {})
    effective_width = _number(
        footprint.get('effective_width_m'), 'camera effective_width_m', positive=True)
    effective_length = _number(
        footprint.get('effective_length_m'), 'camera effective_length_m', positive=True)
    mount_xyz = mount.get('center_xyz_m')
    if not isinstance(mount_xyz, (list, tuple)) or len(mount_xyz) != 3:
        raise DiagnosticCoveragePreflightError(
            'camera optical_mount.center_xyz_m must contain three coordinates.')
    forward_offset = _number(mount_xyz[0], 'camera forward mount offset')
    task_records, all_exposures = [], []
    for task in tasks:
        _, direction, _, _ = _rectangle_parameters(task)
        route = _maneuver_safe_route_bounds(task, target, safe)
        segments = planned_scan_segments(task, effective_width, route)
        exposures = planned_exposures(segments, effective_length, 0.20, forward_offset)
        task_records.append({
            'task_id': task.get('task_id'), 'sweep_direction': direction,
            'maneuver_safe_route_rectangle_m': list(route),
            'scan_segment_count': len(segments), 'exposure_count': len(exposures),
        })
        all_exposures.extend(exposures)
    diagnostic = wall_manifest.get('diagnostic_wall')
    if not isinstance(diagnostic, dict) or not isinstance(diagnostic.get('features'), list):
        raise DiagnosticCoveragePreflightError(
            'wall manifest is not a diagnostic-wall truth source.')
    records = [_feature_coverage(feature, target, tuple(all_exposures), effective_width,
                                 effective_length, resolution)
               for feature in diagnostic['features']]
    intersecting = [record for record in records if record['intersects_target']]
    uncovered = sum(record['uncovered_sample_count'] for record in intersecting)
    return {
        'schema_version': 1,
        'model': {
            'description': (
                'union of planned rectangular SCANs plus automatic_capture_node discrete '
                'trigger contract; declared feature support sampled at %.6f m' % resolution),
            'feature_resolution_m': resolution,
            'camera_footprint_m': [effective_width, effective_length],
            'camera_forward_offset_m': forward_offset,
            'image_overlap_ratio': 0.20,
        },
        'task_set': {
            'drive_rectangle_m': list(target), 'green_safe_rectangle_m': list(safe),
            'exposure_count': len(all_exposures), 'tasks': task_records,
        },
        'feature_coverage': {
            'intersecting_feature_count': len(intersecting),
            'all_intersecting_feature_samples_covered': uncovered == 0,
            'uncovered_sample_count': uncovered,
            'features': records,
        },
    }


def load_preflight_inputs(task_config: Path, wall_manifest: Path, camera_config: Path,
                          robot_config: Path, wall_config: Path) -> tuple[dict[str, Any], ...]:
    """Read only declared static inputs; no simulator or diagnostic image is opened."""
    try:
        task_document = yaml.safe_load(task_config.read_text(encoding='utf-8'))
        camera = yaml.safe_load(camera_config.read_text(encoding='utf-8'))
        robot = yaml.safe_load(robot_config.read_text(encoding='utf-8'))
        wall = yaml.safe_load(wall_config.read_text(encoding='utf-8'))
    except (OSError, UnicodeDecodeError, yaml.YAMLError) as error:
        raise DiagnosticCoveragePreflightError(f'cannot read preflight input: {error}') from error
    if not isinstance(task_document, dict) or not isinstance(
            task_document.get('coverage_planner'), dict):
        raise DiagnosticCoveragePreflightError('task config lacks coverage_planner.')
    task = task_document['coverage_planner'].get('ros__parameters')
    if not isinstance(task, dict) or not all(
            isinstance(value, dict) for value in (camera, robot, wall)):
        raise DiagnosticCoveragePreflightError('preflight YAML input is malformed.')
    return task, _document(wall_manifest, 'diagnostic wall manifest'), camera, robot, wall


def write_preflight_report(output: Path, report: dict[str, Any]) -> None:
    """Write one reproducible JSON report, rejecting accidental relative destinations."""
    if not output.is_absolute():
        raise DiagnosticCoveragePreflightError('output report path must be absolute.')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2,
                                 sort_keys=True) + '\n', encoding='utf-8')
