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

"""Deterministic first-plane projection from rectified images and EKF poses."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import tempfile
from typing import Any, Iterable
from uuid import uuid4

from climbot_mosaic.mosaic_inputs import (
    FrameKey,
    input_summary,
    MosaicInputs,
    ProcessedFrame,
)
from climbot_mosaic.stage_provenance import processed_run_inputs
from climbot_mosaic.stage_provenance import write_stage_provenance
import cv2
import numpy as np


class ProjectionError(ValueError):
    """A camera ray cannot form a physically valid intersection with the wall."""


@dataclass(frozen=True)
class FrameProjection:
    """One image's finite pixel-to-wall homography and ordered wall footprint."""

    key: FrameKey
    image_sha256: str
    homography_image_to_wall: tuple[float, ...]
    footprint_xy_m: tuple[tuple[float, float], ...]
    optical_center_xy_m: tuple[float, float]
    camera_plane_distance_m: float


def _finite(value: float, description: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ProjectionError(f'{description} must be finite.')
    return result


def _quaternion_rotation(orientation: dict[str, Any]) -> np.ndarray:
    """Return the active optical-frame-to-wall rotation for a unit quaternion."""
    try:
        x, y, z, w = (_finite(orientation[name], f'orientation.{name}')
                      for name in ('x', 'y', 'z', 'w'))
    except (KeyError, TypeError, ValueError) as error:
        raise ProjectionError('camera pose orientation is incomplete.') from error
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if abs(norm - 1.0) > 1e-3:
        raise ProjectionError('camera pose orientation is not a unit quaternion.')
    x, y, z, w = (value / norm for value in (x, y, z, w))
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def _camera_center(label: dict[str, Any]) -> np.ndarray:
    try:
        position = label['camera_pose']['pose']['position']
        return np.array([
            _finite(position['x'], 'camera position.x'),
            _finite(position['y'], 'camera position.y'),
            _finite(position['z'], 'camera position.z'),
        ], dtype=np.float64)
    except (KeyError, TypeError, ValueError) as error:
        raise ProjectionError('camera pose position is incomplete.') from error


def _camera_rotation(label: dict[str, Any]) -> np.ndarray:
    try:
        return _quaternion_rotation(label['camera_pose']['pose']['orientation'])
    except (KeyError, TypeError) as error:
        raise ProjectionError('camera pose is incomplete.') from error


def _ray_to_wall(point_px: np.ndarray, matrix: tuple[float, ...],
                 center: np.ndarray, rotation: np.ndarray,
                 wall_plane_z_m: float) -> np.ndarray:
    fx, fy = matrix[0], matrix[4]
    cx, cy = matrix[2], matrix[5]
    ray = rotation @ np.array([
        (float(point_px[0]) - cx) / fx,
        (float(point_px[1]) - cy) / fy,
        1.0,
    ], dtype=np.float64)
    if not np.all(np.isfinite(ray)) or abs(ray[2]) < 1e-12:
        raise ProjectionError('a camera ray is parallel to the wall plane.')
    distance = (wall_plane_z_m - center[2]) / ray[2]
    if not math.isfinite(distance) or distance <= 0.0:
        raise ProjectionError('the wall plane lies behind or at the camera.')
    intersection = center + distance * ray
    if not np.all(np.isfinite(intersection)):
        raise ProjectionError('camera ray intersection is not finite.')
    return intersection


def project_frame(frame: ProcessedFrame, inputs: MosaicInputs,
                  wall_plane_z_m: float = 0.0) -> FrameProjection:
    """Project one rectified image into the named wall coordinate plane."""
    plane = _finite(wall_plane_z_m, 'wall_plane_z_m')
    camera = inputs.camera
    center = _camera_center(frame.label)
    rotation = _camera_rotation(frame.label)
    if abs(center[2] - plane) < 1e-9:
        raise ProjectionError('camera optical center lies on the wall plane.')
    source = np.array([
        [0.0, 0.0],
        [float(camera.width - 1), 0.0],
        [float(camera.width - 1), float(camera.height - 1)],
        [0.0, float(camera.height - 1)],
    ], dtype=np.float64)
    intersections = np.array([
        _ray_to_wall(point, camera.matrix, center, rotation, plane)
        for point in source
    ], dtype=np.float64)
    footprint = intersections[:, :2]
    homography = cv2.getPerspectiveTransform(source.astype(np.float32),
                                             footprint.astype(np.float32))
    if (homography.shape != (3, 3) or not np.all(np.isfinite(homography)) or
            abs(float(np.linalg.det(homography))) < 1e-15):
        raise ProjectionError('image-to-wall homography is singular or non-finite.')
    footprint_area = 0.5 * abs(float(np.dot(
        footprint[:, 0], np.roll(footprint[:, 1], -1)) - np.dot(
        footprint[:, 1], np.roll(footprint[:, 0], -1))))
    if footprint_area <= 1e-12:
        raise ProjectionError('image footprint has zero wall-plane area.')
    optical_center = _ray_to_wall(
        np.array([camera.matrix[2], camera.matrix[5]], dtype=np.float64),
        camera.matrix, center, rotation, plane)
    return FrameProjection(
        frame.key,
        frame.image_sha256,
        tuple(float(value) for value in homography.reshape(-1)),
        tuple((float(point[0]), float(point[1])) for point in footprint),
        (float(optical_center[0]), float(optical_center[1])),
        abs(float(center[2] - plane)),
    )


def project_inputs(inputs: MosaicInputs, wall_plane_z_m: float = 0.0) -> tuple[
        FrameProjection, ...]:
    """Project all already-validated frames in globally stable frame-key order."""
    return tuple(project_frame(frame, inputs, wall_plane_z_m) for frame in inputs.frames)


def projection_extent(projections: Iterable[FrameProjection]) -> tuple[float, float, float, float]:
    """Return min-x, min-y, max-x, max-y for non-empty projected footprints."""
    points = [point for projection in projections for point in projection.footprint_xy_m]
    if not points:
        raise ProjectionError('at least one projected footprint is required.')
    values = np.asarray(points, dtype=np.float64)
    if not np.all(np.isfinite(values)):
        raise ProjectionError('projected footprint extent is not finite.')
    return (float(values[:, 0].min()), float(values[:, 1].min()),
            float(values[:, 0].max()), float(values[:, 1].max()))


def projection_manifest(inputs: MosaicInputs, projections: Iterable[FrameProjection],
                        wall_plane_z_m: float = 0.0) -> dict[str, Any]:
    """Create the strict, host-path-free P2.3 output manifest."""
    values = tuple(projections)
    extent = projection_extent(values)
    return {
        'initial_projection_format_version': 1,
        'input_summary': input_summary(inputs),
        'wall_plane': {'frame': 'wall', 'z_m': _finite(wall_plane_z_m, 'wall_plane_z_m')},
        'footprint_extent_xy_m': {
            'min_x': extent[0], 'min_y': extent[1], 'max_x': extent[2], 'max_y': extent[3],
        },
        'frames': [{
            'source_run_id': projection.key.source_run_id,
            'frame_index': projection.key.frame_index,
            'processed_image_sha256': projection.image_sha256,
            'homography_image_to_wall': list(projection.homography_image_to_wall),
            'footprint_xy_m': [list(point) for point in projection.footprint_xy_m],
            'optical_center_xy_m': list(projection.optical_center_xy_m),
            'camera_plane_distance_m': projection.camera_plane_distance_m,
        } for projection in values],
    }


def render_footprint_preview(projections: Iterable[FrameProjection],
                             max_side_px: int = 1600) -> np.ndarray:
    """Render an outline-only initial-projection preview; source pixels are untouched."""
    if isinstance(max_side_px, bool) or not isinstance(max_side_px, int) or max_side_px < 64:
        raise ProjectionError('preview max_side_px must be an integer of at least 64.')
    values = tuple(projections)
    min_x, min_y, max_x, max_y = projection_extent(values)
    span_x, span_y = max_x - min_x, max_y - min_y
    padding = max(0.01, 0.02 * max(span_x, span_y))
    min_x, min_y = min_x - padding, min_y - padding
    span_x, span_y = max_x - min_x + padding, max_y - min_y + padding
    scale = min(float(max_side_px) / span_x, float(max_side_px) / span_y)
    width, height = max(1, math.ceil(span_x * scale)), max(1, math.ceil(span_y * scale))
    image = np.full((height, width, 3), 32, dtype=np.uint8)
    for projection in values:
        polygon = np.asarray([
            [(point[0] - min_x) * scale, (max_y + padding - point[1]) * scale]
            for point in projection.footprint_xy_m
        ], dtype=np.int32)
        cv2.polylines(image, [polygon], True, (48, 202, 255), 1, cv2.LINE_AA)
    return image


def write_initial_projection(output_dir: Path, inputs: MosaicInputs,
                             wall_plane_z_m: float = 0.0,
                             preview_max_side_px: int = 1600) -> dict[str, Any]:
    """Atomically publish P2.3 projection evidence to a new independent directory."""
    if not output_dir.is_absolute():
        raise ProjectionError('projection output directory must be an absolute path.')
    if output_dir.exists():
        raise ProjectionError('projection output directory must not already exist.')
    parent = output_dir.parent
    if not parent.is_dir():
        raise ProjectionError('projection output parent directory does not exist.')
    for run in inputs.runs:
        try:
            output_dir.relative_to(run.root)
        except ValueError:
            continue
        raise ProjectionError('projection output directory cannot be inside an input run.')
    projections = project_inputs(inputs, wall_plane_z_m)
    manifest = projection_manifest(inputs, projections, wall_plane_z_m)
    preview = render_footprint_preview(projections, preview_max_side_px)
    temporary = Path(tempfile.mkdtemp(
        prefix=f'.{output_dir.name}.tmp-{uuid4().hex}-', dir=parent))
    try:
        text = json.dumps(
            manifest, allow_nan=False, ensure_ascii=False, indent=2, sort_keys=True) + '\n'
        (temporary / 'initial_projection.json').write_text(text, encoding='utf-8')
        if not cv2.imwrite(str(temporary / 'initial_footprints_preview.png'), preview):
            raise ProjectionError('failed to write initial footprint preview.')
        write_stage_provenance(
            temporary, 'initial_projection',
            {'wall_plane_z_m': wall_plane_z_m, 'preview_max_side_px': preview_max_side_px},
            processed_run_inputs(manifest['input_summary']),
            ('initial_projection.json', 'initial_footprints_preview.png'))
        temporary.replace(output_dir)
    except Exception:
        # A temporary diagnostic directory may remain after a filesystem fault, but it can never
        # be mistaken for a published result because only the final rename uses output_dir.
        raise
    return manifest
