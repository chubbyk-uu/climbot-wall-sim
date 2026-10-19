#!/usr/bin/env python3
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

"""Offline D4 quilting with conservative, orientation-independent source exclusion.

The source-rectangle guard is deliberately conservative at selection time.
Actual reuse measurements must replay retained alpha weights, not count this
candidate guard or gutter texels as final wall duplication.
"""

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np

ALGORITHM = 'd4-source-rectangle-exclusion-v3'
DISJOINT_ALGORITHM = 'd4-disjoint-source-seam-recolouring-v5'
LOCAL_ALGORITHM = 'd4-local-disjoint-seam-selection-v6'


@dataclass
class Placement:
    top: int
    left: int
    sy: int
    sx: int
    orientation: int
    mask: np.ndarray
    has_left: bool | None = None
    has_top: bool | None = None


def orient(array, orientation):
    """Rotate counter-clockwise then mirror horizontally, without interpolation."""
    if not isinstance(orientation, (int, np.integer)) or not 0 <= orientation < 8:
        raise ValueError('orientation must be an integer in [0, 7]')
    rotated = np.rot90(array, int(orientation) // 2)
    return np.fliplr(rotated) if orientation % 2 else rotated


def source_coordinates(x, y, patch, orientation):
    """Map output pixel centres back through exactly the same D4 operation."""
    if orientation % 2:
        x = patch - 1 - x
    k = orientation // 2
    if k == 0:
        return x, y
    if k == 1:
        return patch - 1 - y, x
    if k == 2:
        return patch - 1 - x, patch - 1 - y
    if k == 3:
        return y, patch - 1 - x
    raise ValueError('orientation must be in [0, 7]')


def rectangle_distance(first, second):
    """Minimum physical distance between closed XY rectangles."""
    dx = max(first[0] - second[2], second[0] - first[2], 0)
    dy = max(first[1] - second[3], second[1] - first[3], 0)
    return math.hypot(dx, dy)


def source_overlap(sx, sy, old_x, old_y, patch):
    """Source area fraction; orientation never makes repeated content new."""
    w = np.maximum(0, patch - np.abs(old_x - sx))
    h = np.maximum(0, patch - np.abs(old_y - sy))
    return w * h / float(patch * patch)


def source_catalog(shape, patch):
    """Build a finite staggered source-crop pool without inventing pixels.

    Adjacent catalogue rectangles overlap at most 25%. Reusing a catalogue
    entry still passes the very same metric wall-neighbourhood guard; the
    catalogue is not a permission to repeat content locally.
    """
    dx, dy = math.ceil(3 * patch / 4), math.ceil(5 * patch / 8)
    shift = math.ceil(3 * patch / 8)
    entries = []
    for row, sy in enumerate(range(0, shape[0] - patch + 1, dy)):
        for sx in range(shift if row % 2 else 0, shape[1] - patch + 1, dx):
            entries.append((sy, sx))
    return np.asarray(entries, dtype=np.int64).reshape(-1, 2)


def source_schedule(capacity, patch, overlap, radius_px):
    """Find a metric-safe staggered reuse lattice without greedy dead ends.

    A globally seeded permutation assigns actual source crops to colours.
    Repeated colours have patch rectangles farther apart than the exclusion
    radius. Distinct colours still undergo the source-overlap guard.
    """
    step = patch - overlap
    minimum_x = math.floor((radius_px + patch) / step) + 1
    feasible = []
    for nx in range(minimum_x, capacity + 1):
        for ny in range(1, capacity // nx + 1):
            for shift in range(nx // 2 + 1):
                valid = True
                for n in range(1, math.floor((radius_px + patch) / (ny * step)) + 2):
                    x = min((n * shift) % nx, nx - (n * shift) % nx) * step
                    y = n * ny * step
                    if math.hypot(max(0, x - patch), max(0, y - patch)) <= radius_px:
                        valid = False
                        break
                if valid:
                    feasible.append(
                        (nx * ny, -abs(math.log(nx / ny)), nx, ny, shift))
    if not feasible:
        raise ValueError('source catalogue cannot supply a metric-safe reuse schedule; '
                         'change patch scale or add traceable sources; thresholds unchanged')
    _, _, nx, ny, shift = max(feasible)
    return {'period_columns': nx, 'period_rows': ny, 'row_period_column_shift': shift,
            'colours': nx * ny,
            'proof_scope': 'minimum wall-patch rectangle distance of same colour'}


def disjoint_plan(source_shape, height, width, patch, overlap, radius_px, seed, passes=4):
    """Colour a finite wall with nonoverlapping source crops, then destroy its period.

    Every retained alpha contributor is a subset of a disjoint full source crop.
    Equal colours must remain farther apart than the metric rectangle guard.
    Recolouring uses all wall positions, including future quilt placements;
    it therefore cannot create a greedy allocation dead end during quilting.
    """
    step = patch - overlap
    source_rows, source_columns = source_shape[0] // patch, source_shape[1] // patch
    if (min(source_rows, source_columns) * step - patch <= radius_px or
            passes < 0):
        raise ValueError('disjoint source catalogue cannot supply the metric-safe initial '
                         'colouring; change patch/overlap scale or add traceable sources')
    catalog = np.array([(y * patch, x * patch) for y in range(source_rows)
                        for x in range(source_columns)], np.int64)
    rows = max(1, math.ceil((height - overlap) / step))
    columns = max(1, math.ceil((width - overlap) / step))
    yy, xx = np.indices((rows, columns))
    colours = ((yy % source_rows) * source_columns + xx %
               source_columns).ravel()
    initial = colours.copy()
    x, y = xx.ravel() * step, yy.ravel() * step
    rng = np.random.default_rng(seed)
    population = np.bincount(colours, minlength=len(catalog))
    changed = 0
    minimum_options = len(catalog)
    for _ in range(passes):
        for index in rng.permutation(len(colours)):
            dx = np.maximum(np.abs(x - x[index]) - patch, 0)
            dy = np.maximum(np.abs(y - y[index]) - patch, 0)
            near = dx * dx + dy * dy <= radius_px ** 2
            near[index] = False
            available = np.ones(len(catalog), dtype=bool)
            available[colours[near]] = False
            options = np.flatnonzero(available)
            if not len(options):
                raise RuntimeError(
                    'invalid source colouring: no safe recolour including itself')
            minimum_options = min(minimum_options, len(options))
            population[colours[index]] -= 1
            probability = 1. / (1. + population[options]) ** 2
            probability /= probability.sum()
            selected = int(rng.choice(options, p=probability))
            changed += int(selected != colours[index])
            colours[index] = selected
            population[selected] += 1
    # A separate final proof, independent of update order and seam decisions.
    minimum_repeat = None
    for colour in np.flatnonzero(population):
        positions = np.flatnonzero(colours == colour)
        for i, first in enumerate(positions):
            previous = positions[:i]
            distance = np.hypot(np.maximum(np.abs(x[previous] - x[first]) - patch, 0),
                                np.maximum(np.abs(y[previous] - y[first]) - patch, 0))
            if len(distance):
                current = float(distance.min())
                if current <= radius_px:
                    raise RuntimeError(
                        'final disjoint source colouring violates metric exclusion')
                minimum_repeat = current if minimum_repeat is None else min(
                    minimum_repeat, current)
    palette = rng.permutation(len(catalog))
    return catalog, palette[colours], (rows * step + overlap, columns * step + overlap), {
        'algorithm': DISJOINT_ALGORITHM,
        'source_catalog': 'disjoint',
        'source_catalog_origins_yx_px': catalog.tolist(),
        'recolour_passes': passes, 'recolour_updates': changed,
        'positions_changed_from_periodic_initialization': int(
            np.count_nonzero(initial != colours)),
        'minimum_recolour_options': minimum_options,
        'source_crops_used': int(np.count_nonzero(population)),
        'minimum_repeat_rectangle_distance_px': minimum_repeat,
        'initial_period_columns': source_columns, 'initial_period_rows': source_rows,
        'final_source_assignment_sha256': hashlib.sha256(
            np.asarray(palette[colours], dtype='<i8').tobytes()).hexdigest(),
        'source_exclusion_scope': 'all retained alpha contributors, by disjoint full-crop proof',
    }


def verify_disjoint_colouring(assignment, x, y, patch, radius_px):
    """Verify the final finite-wall source identities independently of seam scoring."""
    minimum_repeat = None
    for colour in np.unique(assignment):
        positions = np.flatnonzero(assignment == colour)
        for i, first in enumerate(positions):
            previous = positions[:i]
            distance = np.hypot(np.maximum(np.abs(x[previous] - x[first]) - patch, 0),
                                np.maximum(np.abs(y[previous] - y[first]) - patch, 0))
            if len(distance):
                current = float(distance.min())
                if current <= radius_px:
                    raise RuntimeError(
                        'final source assignment violates metric exclusion')
                minimum_repeat = current if minimum_repeat is None else min(
                    minimum_repeat, current)
    return minimum_repeat


def disjoint_layout(source_grey, height, width, patch, overlap, seed, candidates,
                    tolerance, feather, scale, radius_m, max_overlap, log):
    """Select seam-qualified sources while preserving the entire finite-wall guard."""
    import bake_wall_texture as bake

    if max_overlap != 0:
        raise ValueError(
            'disjoint catalogue requires explicit max_source_overlap=0')
    catalog, assignment, shape, stats = disjoint_plan(
        source_grey.shape, height, width, patch, overlap, radius_m / scale, seed)
    rng = np.random.default_rng(seed)
    canvas = np.zeros(shape, np.float32)
    step = patch - overlap
    columns = (shape[1] - overlap) // step
    x = np.arange(len(assignment)) % columns * step
    y = np.arange(len(assignment)) // columns * step
    population = np.bincount(assignment, minlength=len(catalog))
    placements, eligible_counts, source_counts = [], [], []
    for index in range(len(assignment)):
        row, column = divmod(index, columns)
        top, left = row * step, column * step
        window = canvas[top:top + patch, left:left + patch]
        dx = np.maximum(np.abs(x - left) - patch, 0)
        dy = np.maximum(np.abs(y - top) - patch, 0)
        near = dx * dx + dy * dy <= (radius_m / scale) ** 2
        near[index] = False
        available = np.ones(len(catalog), dtype=bool)
        available[assignment[near]] = False
        options = np.flatnonzero(available)
        if not len(options):
            raise RuntimeError(
                'source assignment lost its safe current colour')
        source_counts.append(len(options))
        pool = []
        # Sample distinct (source, D4) pairs; transformations do not add source capacity.
        samples = rng.choice(
            len(options) * 8, min(candidates, len(options) * 8), replace=False)
        for sample in samples:
            colour, orientation = int(options[sample // 8]), int(sample % 8)
            sy, sx = map(int, catalog[colour])
            block = orient(
                source_grey[sy:sy + patch, sx:sx + patch], int(orientation))
            error = 0.
            if column:
                delta = block[:, :overlap] - window[:, :overlap]
                error += float(np.einsum('ij,ij->', delta, delta))
            if row:
                delta = block[:overlap, :] - window[:overlap, :]
                error += float(np.einsum('ij,ij->', delta, delta))
            pool.append((error, colour, orientation))
        best = min(v[0] for v in pool)
        allowed = [(colour, k) for error, colour, k in pool if error <=
                   best * (1 + tolerance) + 1e-9]
        probability = np.array(
            [1. / (1. + population[colour]) ** 2 for colour, _ in allowed])
        probability /= probability.sum()
        colour, orientation = allowed[int(
            rng.choice(len(allowed), p=probability))]
        population[assignment[index]] -= 1
        assignment[index] = colour
        population[colour] += 1
        sy, sx = map(int, catalog[colour])
        block = orient(source_grey[sy:sy + patch, sx:sx + patch], orientation)
        mask = bake.patch_mask(block, window, 0, 0, patch,
                               overlap, column > 0, row > 0)
        alpha = bake.feather_mask(mask, feather)
        alpha[~bake.overlap_mask(patch, overlap, column > 0, row > 0)] = 1
        window[:] = window * (1 - alpha) + block * alpha
        placements.append(Placement(top, left, sy, sx,
                          orientation, np.packbits(mask)))
        eligible_counts.append(len(pool))
        if column == columns - 1:
            log(f'  disjoint row {row + 1}/{(shape[0] - overlap) // step}')
    stats.update({'candidate_attempts': sum(eligible_counts), 'source_rejections': 0,
                  'eligible_candidates_per_placement': eligible_counts,
                  'seam_eligible_source_crops_per_placement': source_counts,
                  'source_crops_used': int(np.count_nonzero(population)),
                  'minimum_repeat_rectangle_distance_px': verify_disjoint_colouring(
                      assignment, x, y, patch, radius_m / scale),
                  'final_source_assignment_sha256': hashlib.sha256(
                      np.asarray(assignment, dtype='<i8').tobytes()).hexdigest()})
    return placements, shape, stats


def layout(source_grey, height, width, patch, overlap, seed, candidates,
           tolerance, feather, scale, radius_m, max_overlap, log,
           attempt_multiplier=64, catalog_mode='free'):
    """Choose eligible candidates, score transformed seams, and keep seeded diversity."""
    import bake_wall_texture as bake

    numbers = (scale, radius_m, max_overlap, tolerance, feather)
    if not all(math.isfinite(value) for value in numbers):
        raise ValueError('randomization parameters must be finite')
    if (scale <= 0 or radius_m <= 0 or not 0 <= max_overlap < 1 or
            tolerance < 0 or not 0 < overlap < patch or candidates < 1 or
            attempt_multiplier < 1 or feather < 0 or feather > overlap / 2):
        raise ValueError('invalid randomization geometry or thresholds')
    if min(source_grey.shape) < patch or min(height, width) <= 0:
        raise ValueError('source/output dimensions cannot supply a patch')
    rng = np.random.default_rng(seed)
    if catalog_mode == 'disjoint':
        return disjoint_layout(source_grey, height, width, patch, overlap, seed, candidates,
                               tolerance, feather, scale, radius_m, max_overlap, log)
    if catalog_mode not in ('free', 'packed', 'scheduled', 'local'):
        raise ValueError(
            'source catalog must be free, packed, scheduled, disjoint or local')
    if catalog_mode == 'local':
        if max_overlap != 0:
            raise ValueError(
                'local disjoint catalogue requires max_source_overlap=0')
        catalog = np.array([(y, x) for y in range(0, source_grey.shape[0] - patch + 1, patch)
                            for x in range(0, source_grey.shape[1] - patch + 1, patch)], np.int64)
    else:
        catalog = source_catalog(
            source_grey.shape, patch) if catalog_mode != 'free' else None
    schedule = source_schedule(len(catalog), patch, overlap, radius_m /
                               scale) if catalog_mode == 'scheduled' else None
    palette = rng.permutation(len(catalog)) if schedule is not None else None
    step = patch - overlap
    rows = max(1, math.ceil((height - overlap) / step))
    columns = max(1, math.ceil((width - overlap) / step))
    shape = (rows * step + overlap, columns * step + overlap)
    canvas = np.zeros(shape, dtype=np.float32)
    placements = []
    history = np.empty((0, 4), dtype=np.int64)  # top, left, sy, sx
    attempts = rejected = 0
    eligible_counts = []
    source_counts = []
    started = time.monotonic()
    for row in range(rows):
        for column in range(columns):
            top, left = row * step, column * step
            window = canvas[top:top + patch, left:left + patch]
            current_rect = (left, top, left + patch, top + patch)
            near = np.asarray([
                rectangle_distance(current_rect, (v[1], v[0], v[1] + patch,
                                                  v[0] + patch)) * scale <= radius_m
                for v in history], dtype=bool)
            neighbours = history[near]
            pool = []
            eligible = catalog
            if schedule is not None:
                nx, ny = schedule['period_columns'], schedule['period_rows']
                colour = (row % ny) * nx + (column -
                                            schedule['row_period_column_shift'] * (row // ny)) % nx
                eligible = catalog[[palette[colour]]]
            if catalog is not None and len(neighbours):
                allowed = np.ones(len(eligible), dtype=bool)
                for previous in neighbours:
                    reused = source_overlap(
                        eligible[:, 1], eligible[:, 0], previous[3], previous[2], patch)
                    allowed &= reused <= max_overlap + 1e-12
                eligible = eligible[allowed]
            source_counts.append(
                len(eligible) if eligible is not None else None)
            draws = (rng.choice(len(eligible) * 8, min(candidates, len(eligible) * 8),
                                replace=False) if catalog_mode == 'local' else None)
            # Bounded attempts, with explicit exhaustion and no constraint fallback.
            attempt_limit = len(draws) if draws is not None else candidates * attempt_multiplier
            for attempt in range(attempt_limit):
                if eligible is not None and not len(eligible):
                    break
                attempts += 1
                if draws is not None:
                    sy, sx = map(int, eligible[draws[attempt] // 8])
                    orientation = int(draws[attempt] % 8)
                elif eligible is None:
                    sy = int(rng.integers(source_grey.shape[0] - patch + 1))
                    sx = int(rng.integers(source_grey.shape[1] - patch + 1))
                else:
                    sy, sx = map(
                        int, eligible[int(rng.integers(len(eligible)))])
                if draws is None:
                    orientation = int(rng.integers(8))
                if len(neighbours) and np.any(source_overlap(
                        sx, sy, neighbours[:, 3], neighbours[:, 2], patch) >
                        max_overlap + 1e-12):
                    rejected += 1
                    continue
                block = orient(
                    source_grey[sy:sy + patch, sx:sx + patch], orientation)
                error = 0.0
                if column:
                    delta = block[:, :overlap] - window[:, :overlap]
                    error += float(np.einsum('ij,ij->', delta, delta))
                if row:
                    delta = block[:overlap, :] - window[:overlap, :]
                    error += float(np.einsum('ij,ij->', delta, delta))
                usage = float(source_overlap(
                    sx, sy, history[:, 3], history[:, 2], patch).sum())
                pool.append((error, usage, sy, sx, orientation))
                if len(pool) == candidates:
                    break
            if not pool:
                raise ValueError(
                    f'source diversity exhausted at row={row} column={column}; '
                    f'radius_m={radius_m} max_source_overlap={max_overlap} '
                    f'attempts_per_patch={candidates * attempt_multiplier}; '
                    'add traceable sources or change candidates/patch scale explicitly')
            eligible_counts.append(len(pool))
            if len(pool) < candidates:
                log(f'  candidate pool row={row} column={column}: '
                    f'{len(pool)}/{candidates}; exclusion thresholds unchanged')
            best = min(value[0] for value in pool)
            allowed = [value for value in pool if value[0]
                       <= best * (1 + tolerance) + 1e-9]
            # Among seam-qualified candidates prefer less globally used source areas.
            probability = np.array([1 / (1 + value[1]) for value in allowed])
            probability /= probability.sum()
            _, _, sy, sx, orientation = allowed[int(
                rng.choice(len(allowed), p=probability))]
            block = orient(
                source_grey[sy:sy + patch, sx:sx + patch], orientation)
            mask = bake.patch_mask(block, window, 0, 0, patch, overlap,
                                   column > 0, row > 0)
            alpha = bake.feather_mask(mask, feather)
            alpha[~bake.overlap_mask(patch, overlap, column > 0, row > 0)] = 1
            window[:] = window * (1 - alpha) + block * alpha
            placements.append(Placement(top, left, sy, sx,
                              orientation, np.packbits(mask)))
            history = np.vstack((history, (top, left, sy, sx)))
        log(f'  randomized row {row + 1}/{rows} ({time.monotonic() - started:.0f} s)')
    return placements, shape, {
        'candidate_attempts': attempts, 'source_rejections': rejected,
        'eligible_candidates_per_placement': eligible_counts,
        'source_catalog': catalog_mode,
        'source_catalog_origins_yx_px': catalog.tolist() if catalog is not None else None,
        'source_schedule': schedule,
        **({'algorithm': LOCAL_ALGORITHM,
            'seam_eligible_source_crops_per_placement': source_counts,
            'future_sources_reserved': False,
            'selection_scope': 'past metric neighbours only; distant source reuse allowed'}
           if catalog_mode == 'local' else {}),
    }


def alpha_mask(placement, patch, overlap, feather):
    """Recreate precisely the blend weight used in grey and RGB replay."""
    import bake_wall_texture as bake

    mask = np.unpackbits(placement.mask, count=patch *
                         patch).reshape(patch, patch).astype(bool)
    alpha = bake.feather_mask(mask, feather)
    left = placement.left > 0 if placement.has_left is None else placement.has_left
    top = placement.top > 0 if placement.has_top is None else placement.has_top
    alpha[~bake.overlap_mask(patch, overlap, left, top)] = 1
    return alpha


def replay(source, placements, shape, patch, overlap, feather, height, width):
    """Render immutable offline source transforms into one global RGB canvas."""
    canvas = np.zeros(shape + (3,), dtype=np.uint8)
    for item in placements:
        block = orient(source[item.sy:item.sy + patch, item.sx:item.sx + patch],
                       item.orientation)
        alpha = alpha_mask(item, patch, overlap, feather)[:, :, None]
        window = canvas[item.top:item.top + patch, item.left:item.left + patch]
        window[:] = np.clip(np.round(window * (1 - alpha) + block * alpha),
                            0, 255).astype(np.uint8)
    return canvas[:height, :width]


def write_trace(directory, placements, patch, source_native_size, source_resampled_side,
                *, algorithm=ALGORITHM):
    """Store exact cuts plus compact, hashed source and transform records."""
    directory = Path(directory)
    coordinates = np.array([(p.top, p.left, p.sy, p.sx, p.orientation)
                            for p in placements], dtype=np.int32)
    mask_path = directory / 'quilt_masks.npz'
    np.savez_compressed(mask_path, coordinates=coordinates,
                        masks=np.array([p.mask for p in placements]), patch_px=patch)
    records = []
    for index, item in enumerate(placements):
        ratio = source_native_size / source_resampled_side
        records.append({
            'index': index, 'wall_top_left_px': [item.left, item.top],
            'source_top_left_resampled_px': [item.sx, item.sy],
            'source_rect_resampled_px': [item.sx, item.sy, item.sx + patch, item.sy + patch],
            'source_center_resampled_px': [item.sx + (patch - 1) / 2,
                                           item.sy + (patch - 1) / 2],
            'source_center_native_px': [
                (item.sx + patch / 2) * ratio - 0.5,
                (item.sy + patch / 2) * ratio - 0.5],
            'orientation': item.orientation,
            'rotation_ccw_deg': 90 * (item.orientation // 2),
            'mirror_axis_after_rotation': 'image_y' if item.orientation % 2 else None,
            'resampled_to_native_scale': ratio,
        })
    trace_path = directory / 'quilt_sources.json'
    trace_path.write_text(json.dumps({
        'algorithm': algorithm, 'transform_order': 'crop, rot90, fliplr',
        'source_interpolation': 'Pillow LANCZOS once before lossless D4',
        'pixel_coordinates': 'pixel centres; source rows down; wall conversion in manifest',
        'alpha_replay': 'unpack mask, Pillow GaussianBlur, force non-overlap to one; '
                        'each later alpha multiplies each earlier contribution by (1-alpha)',
        'placements': records,
    }, indent=2, allow_nan=False) + '\n')
    return [{'file': path.name, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
            for path in (trace_path, mask_path)]
