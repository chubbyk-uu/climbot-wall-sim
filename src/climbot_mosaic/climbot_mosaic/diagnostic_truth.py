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

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import tempfile
from typing import Any
from uuid import uuid4

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
    wall_document = _document(wall_manifest, 'diagnostic wall manifest')
    diagnostic = wall_document.get('diagnostic_wall')
    if not isinstance(diagnostic, dict) or not isinstance(diagnostic.get('features'), list):
        raise DiagnosticTruthError('wall manifest is not a diagnostic-wall truth source.')
    fusion = mosaic_document.get('fusion')
    if not isinstance(fusion, dict) or 'hard cut' not in str(fusion.get('method', '')).lower():
        raise DiagnosticTruthError('P2.7 diagnostic evaluation requires a hard-cut mosaic.')
    truth_grid, mosaic_grid = _wall_grid(wall_document), _mosaic_grid(mosaic_document)
    anchors = _anchors(diagnostic['features'], truth_grid, mosaic_grid, anchor_padding_m)
    paths = {
        'pose_only': mosaic_dir / 'mosaic_pose_only.tif',
        'optimized': mosaic_dir / 'mosaic_optimized.tif',
    }
    temporary = Path(tempfile.mkdtemp(prefix=f'.{output_dir.name}.tmp-{uuid4().hex}-',
                                      dir=output_dir.parent))
    try:
        raw_matches = {}
        for name, path in paths.items():
            if not path.is_file():
                raise DiagnosticTruthError(f'mosaic product is absent: {path.name}')
            image = tifffile.imread(path)
            raw_matches[name] = _variant_matches(
                image, anchors, wall_manifest.parent, _blocks(wall_document),
                truth_grid, mosaic_grid)
            del image
        accepted_ids = set.intersection(*(
            {match['id'] for match in matches
             if match['phase_response'] >= minimum_phase_response}
            for matches in raw_matches.values()))
        if len(accepted_ids) < 2:
            raise DiagnosticTruthError(
                'fewer than two common anchors meet the truth-response contract.')
        variants = {
            name: _summarize_variant(matches, accepted_ids)
            for name, matches in raw_matches.items()
        }
        pose_p95 = variants['pose_only']['absolute_anchor_offset_m']['p95']
        optimized_p95 = variants['optimized']['absolute_anchor_offset_m']['p95']
        summary = {
            'diagnostic_truth_format_version': 1,
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
        temporary.replace(output_dir)
        return summary
    except Exception:
        import shutil
        shutil.rmtree(temporary, ignore_errors=True)
        raise
