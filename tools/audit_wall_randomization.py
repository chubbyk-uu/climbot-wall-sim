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

"""Measure retained source reuse and D4 copies in final BC1 wall pixels.

This is an offline asset audit, not rendered-camera or mosaic acceptance.
The sampled source-weight area is an approximation at a recorded grid spacing;
NCC uses full-resolution decoded DDS pixels, including quilt/block boundaries.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np
from PIL import Image
from wall_texture_randomization import alpha_mask
from wall_texture_randomization import Placement
from wall_texture_randomization import source_coordinates

Image.MAX_IMAGE_PIXELS = None


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_trace(manifest_path):
    path = Path(manifest_path)
    manifest = json.loads(path.read_text())
    for item in manifest['quilt']['trace_files']:
        if Path(item['file']).name != item['file'] or digest(
                path.parent / item['file']) != item['sha256']:
            raise ValueError('quilt trace hash/path mismatch')
    with np.load(path.parent / 'quilt_masks.npz', allow_pickle=False) as data:
        placements = [Placement(*map(int, row), mask)
                      for row, mask in zip(data['coordinates'], data['masks'], strict=True)]
    return manifest, placements


def read_image(manifest, directory, region_px=None):
    """Decode nominal ownership once; duplicate gutter pixels are not area."""
    rx, ry, rw, rh = region_px or (
        0, 0, manifest['width_px'], manifest['height_px'])
    output = np.zeros((rh, rw, 3), np.uint8)
    for block in manifest['maps']['albedo']['blocks']:
        x, y = block['x_px'], block['y_px']
        w, h = block['width_px'], block['height_px']
        x0, y0, x1, y1 = max(x, rx), max(y, ry), min(
            x + w, rx + rw), min(y + h, ry + rh)
        if x0 >= x1 or y0 >= y1:
            continue
        path = Path(directory) / block['file']
        if 'sha256' in block and digest(path) != block['sha256']:
            raise ValueError('DDS checksum mismatch')
        with Image.open(path) as file:
            decoded = np.asarray(file.convert('RGB'))
        sx, sy = x0 - block['sample_x_px'], y0 - block['sample_y_px']
        output[y0 - ry:y1 - ry, x0 - rx:x1 -
               rx] = decoded[sy:sy + y1 - y0, sx:sx + x1 - x0]
    return cv2.cvtColor(output, cv2.COLOR_RGB2GRAY)


def gutter_continuity(manifest, directory):
    """Compare the same global BC1 blocks in neighbouring mip-0..5 samples."""
    blocks = manifest['maps']['albedo']['blocks']
    levels = []
    for block in blocks:
        data = (Path(directory) / block['file']).read_bytes()
        offset = 128
        width, height = block['sample_width_px'], block['sample_height_px']
        encoded = []
        for _ in range(6):
            columns, rows = math.ceil(width / 4), math.ceil(height / 4)
            size = columns * rows * 8
            encoded.append(np.frombuffer(data[offset:offset + size],
                           dtype=np.uint8).reshape(rows, columns, 8))
            offset += size
            width, height = max(width // 2, 1), max(height // 2, 1)
        levels.append(encoded)
    comparisons = mismatches = 0
    for i, first in enumerate(blocks):
        for j in range(i):
            second = blocks[j]
            if abs(first['row'] - second['row']) + abs(first['column'] - second['column']) != 1:
                continue
            for level in range(6):
                alignment = 4 * 2 ** level
                ax, ay = first['sample_x_px'], first['sample_y_px']
                bx, by = second['sample_x_px'], second['sample_y_px']
                if any(v % alignment for v in (ax, ay, bx, by)):
                    raise ValueError(
                        'DDS source origins do not preserve BC1 mip phase')
                x0, y0 = max(ax, bx), max(ay, by)
                x1 = min(ax + first['sample_width_px'],
                         bx + second['sample_width_px'])
                y1 = min(ay + first['sample_height_px'],
                         by + second['sample_height_px'])
                x0, y0 = math.ceil(x0 / alignment), math.ceil(y0 / alignment)
                x1, y1 = x1 // alignment, y1 // alignment
                if x1 <= x0 or y1 <= y0:
                    continue
                a = levels[i][level][y0 - ay // alignment:y1 - ay // alignment,
                                     x0 - ax // alignment:x1 - ax // alignment]
                b = levels[j][level][y0 - by // alignment:y1 - by // alignment,
                                     x0 - bx // alignment:x1 - bx // alignment]
                comparisons += 1
                mismatches += int(not np.array_equal(a, b))
    return {'mip_levels': [0, 1, 2, 3, 4, 5], 'comparisons': comparisons,
            'mismatched_shared_regions': mismatches}


def seam_gradient(image, xs, ys, weights):
    """Compare final-pixel gradients at blended seams and other sampled wall points."""
    x = np.clip(xs, 1, image.shape[1] - 2)
    y = np.clip(ys, 1, image.shape[0] - 2)
    gx = (image[np.ix_(y, x + 1)].astype(np.float32) -
          image[np.ix_(y, x - 1)]) / 2
    gy = (image[np.ix_(y + 1, x)].astype(np.float32) -
          image[np.ix_(y - 1, x)]) / 2
    magnitude = np.hypot(gx, gy)
    seam = (weights > .05).sum(axis=2) > 1
    if not seam.any() or seam.all():
        raise ValueError(
            'seam audit requires both blended and unblended samples')
    seam_p95, off_p95 = float(np.percentile(magnitude[seam], 95)), float(
        np.percentile(magnitude[~seam], 95))
    return {'method': 'central pixel gradients; seam has at least two retained weights >0.05',
            'seam_samples': int(seam.sum()), 'off_seam_samples': int((~seam).sum()),
            'seam_gradient_p95_dn_per_px': seam_p95,
            'off_seam_gradient_p95_dn_per_px': off_p95,
            'ratio': seam_p95 / off_p95 if off_p95 > 0 else None}


def retained_weights(manifest, placements, stride):
    """Replay all contributors on a grid; preserve even small feather weights."""
    q = manifest['quilt']
    patch, overlap = q['patch_px'], q['overlap_px']
    # With 25% overlaps at most four placements cover any physical sample.
    slots = math.ceil(patch / (patch - overlap)) ** 2
    xs = np.arange(stride // 2, manifest['width_px'], stride)
    ys = np.arange(stride // 2, manifest['height_px'], stride)
    owners = np.full((len(ys), len(xs), slots), -1, np.int32)
    weights = np.zeros(owners.shape, np.float32)
    counts = np.zeros(owners.shape[:2], np.int16)
    for index, item in enumerate(placements):
        xi = np.flatnonzero((xs >= item.left) & (xs < item.left + patch))
        yi = np.flatnonzero((ys >= item.top) & (ys < item.top + patch))
        if not len(xi) or not len(yi):
            continue
        alpha = alpha_mask(item, patch, overlap, q['seam_feather_px'])[
            np.ix_(ys[yi] - item.top, xs[xi] - item.left)]
        yy, xx = np.meshgrid(yi, xi, indexing='ij')
        weights[yy, xx] *= (1 - alpha[:, :, None])
        slot = counts[yy, xx]
        owners[yy, xx, slot] = index
        weights[yy, xx, slot] = alpha
        counts[yy, xx] += 1
    if not np.allclose(weights.sum(axis=2), 1, atol=1e-6):
        raise ValueError(
            'retained source weights do not cover every wall sample')
    return xs, ys, owners, weights


def source_weight_reuse(manifest, placements, xs, ys, owners, weights, stride, radius):
    """Count each retained source contribution once if it has a distinct local copy."""
    scale = manifest['scale_m_per_px']
    patch = manifest['quilt']['patch_px']
    records = []
    for owner, item in enumerate(placements):
        yy, xx, slot = np.where((owners == owner) & (weights > 0))
        sx, sy = source_coordinates(xs[xx] - item.left, ys[yy] - item.top,
                                    patch, item.orientation)
        # Spatial source bins approximate area, not exact per-pixel duplicate identity.
        records.append(np.column_stack((
            (sx + item.sx) // stride, (sy + item.sy) // stride,
            xs[xx], ys[yy], weights[yy, xx, slot])))
    values = np.concatenate(records)
    source_side = max(manifest['source_resampled_dimensions_px']) // stride + 1
    keys = values[:, 1].astype(np.int64) * source_side + \
        values[:, 0].astype(np.int64)
    order = np.argsort(keys, kind='stable')
    values, keys = values[order], keys[order]
    ends = np.r_[0, np.flatnonzero(np.diff(keys)) + 1, len(keys)]
    repeated_weight = 0.0
    distances = []
    for start, end in zip(ends[:-1], ends[1:]):
        group = values[start:end]
        if len(group) < 2:
            continue
        delta = group[:, None, 2:4] - group[None, :, 2:4]
        distance = np.linalg.norm(delta, axis=2) * scale
        # Same wall position's seam contributors are never a duplicate defect.
        eligible = (distance > .015) & (distance <= radius)
        duplicated = eligible.any(axis=1)
        repeated_weight += float(group[duplicated, 4].sum())
        if duplicated.any():
            distances.extend(distance[np.triu(eligible, k=1)].tolist())
    cell_area = (stride * scale) ** 2
    return {
        'method': ('all final alpha contributions; source bins at recorded stride; '
                   'approximate area'),
        'sample_spacing_mm': stride * scale * 1000,
        'weighted_repeated_area_m2': repeated_weight * cell_area,
        'weighted_repeated_fraction': repeated_weight / float(weights.sum()),
        'repeat_distance_m': {'count': len(distances),
                              'p50': float(np.median(distances)) if distances else None,
                              'p95': float(np.percentile(distances, 95)) if distances else None},
    }


def opaque_core_pairs(manifest, placements, radius):
    """Exact conservative core check, independent of rotations and gutter layout."""
    return source_rectangle_pairs(manifest, placements, radius, core_only=True)


def source_rectangle_pairs(manifest, placements, radius, core_only=False):
    """Independently enumerate same-source intersections within the wall guard.

    Vectorize source filtering so small-crop experiments do not require a
    Python wall-distance calculation for every unrelated source pair.
    A zero full-rectangle result proves exclusion of all retained fragments.
    """
    patch, overlap = (manifest['quilt'][k] for k in ('patch_px', 'overlap_px'))
    scale = manifest['scale_m_per_px']
    side = patch - 2 * overlap if core_only else patch
    coordinates = np.array([(p.left, p.top, p.sx, p.sy)
                           for p in placements], np.int64)
    failures = []
    for i, a in enumerate(placements):
        previous = coordinates[:i]
        w = np.maximum(0, side - np.abs(previous[:, 2] - a.sx))
        h = np.maximum(0, side - np.abs(previous[:, 3] - a.sy))
        matches = np.flatnonzero((w > 0) & (h > 0))
        dx = np.maximum(0, np.abs(previous[matches, 0] - a.left) - patch)
        dy = np.maximum(0, np.abs(previous[matches, 1] - a.top) - patch)
        matches = matches[np.hypot(dx, dy) * scale <= radius]
        for j in matches:
            key = 'source_core_area_m2' if core_only else 'source_rectangle_area_m2'
            failures.append(
                {'placements': [int(j), i], key: float(w[j] * h[j] * scale ** 2)})
    return failures


def inverse_orient(array, orientation):
    unmirrored = np.fliplr(array) if orientation % 2 else array
    return np.rot90(unmirrored, -(orientation // 2))


def output_coordinates(sx, sy, patch, orientation):
    k = orientation // 2
    if k == 0:
        x, y = sx, sy
    elif k == 1:
        x, y = sy, patch - 1 - sx
    elif k == 2:
        x, y = patch - 1 - sx, patch - 1 - sy
    else:
        x, y = patch - 1 - sy, sx
    return (patch - 1 - x if orientation % 2 else x), y


def ncc_windows(manifest, placements, image, count, radius, seed):
    """Source-indexed D4-copy search, including final seams and compressed pixels."""
    scale, patch = manifest['scale_m_per_px'], manifest['quilt']['patch_px']
    window = round(.1 / scale)
    rng = np.random.default_rng(seed)
    h, w = image.shape
    if min(h, w) < window or count < 1:
        raise ValueError('NCC audit requires nonempty windows inside the wall')
    rows = []
    for query in range(count):
        x0, y0 = int(rng.integers(w - window + 1)
                     ), int(rng.integers(h - window + 1))
        cx, cy = x0 + (window - 1) / 2, y0 + (window - 1) / 2
        first = image[y0:y0 + window, x0:x0 + window]
        best = None
        candidates = set()
        for a in placements:
            if not (a.left <= cx < a.left + patch and a.top <= cy < a.top + patch):
                continue
            sx, sy = source_coordinates(
                cx - a.left, cy - a.top, patch, a.orientation)
            sx, sy = sx + a.sx, sy + a.sy
            for b in placements:
                if not (b.sx <= sx < b.sx + patch and b.sy <= sy < b.sy + patch):
                    continue
                bx, by = output_coordinates(
                    sx - b.sx, sy - b.sy, patch, b.orientation)
                bx, by = bx + b.left, by + b.top
                distance = math.hypot(cx - bx, cy - by) * scale
                if not .015 < distance <= radius:
                    continue
                left, top = round(bx - (window - 1) /
                                  2), round(by - (window - 1) / 2)
                if not (0 <= left <= w - window and 0 <= top <= h - window):
                    continue
                identity = (left, top, a.orientation, b.orientation)
                if identity in candidates:
                    continue
                candidates.add(identity)
                second = image[top:top + window, left:left + window]
                source_a = np.ascontiguousarray(
                    inverse_orient(first, a.orientation))
                source_b = np.ascontiguousarray(
                    inverse_orient(second, b.orientation))
                if min(float(source_a.std()), float(source_b.std())) < 1e-6:
                    continue  # constant zero variance has undefined NCC
                score = float(cv2.matchTemplate(
                    source_a, source_b, cv2.TM_CCOEFF_NORMED)[0, 0])
                if best is None or score > best['ncc']:
                    best = {'ncc': score, 'distance_m': distance,
                            'query_px': [x0, y0], 'copy_px': [left, top],
                            'orientations': [a.orientation, b.orientation]}
        rows.append({'query': query, 'query_px': [x0, y0], 'std_dn': float(first.std()),
                     'source_indexed_candidates': len(candidates), 'best_copy': best})
    copied = sum(row['best_copy'] is not None and row['best_copy']
                 ['ncc'] > .9 for row in rows)
    return {'queries': count, 'window_px': window, 'seed': seed,
            'scope': 'source-indexed candidates at query centres; D4 tolerance; final DDS windows',
            'copied_windows_over_0_90': copied, 'copied_window_fraction': copied / count,
            'weak_texture_windows_std_below_2dn': sum(row['std_dn'] < 2 for row in rows),
            'windows': rows}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--sample-step-px', type=int, default=16)
    parser.add_argument('--windows', type=int, default=300)
    parser.add_argument('--query-seed', type=int, default=20261017)
    args = parser.parse_args()
    if args.sample_step_px < 1 or args.output.exists():
        parser.error('positive sample stride and new output path required')
    cv2.setNumThreads(1)
    manifest, placements = load_trace(args.manifest)
    radius = manifest['randomization']['reuse_radius_m']
    xs, ys, owners, weights = retained_weights(
        manifest, placements, args.sample_step_px)
    area = source_weight_reuse(manifest, placements, xs, ys, owners, weights,
                               args.sample_step_px, radius)
    area['scope'] = ('concrete substrate before diagnostic overpaint; not final visible area'
                     if 'diagnostic_wall' in manifest else 'all final concrete quilt weights')
    cores = opaque_core_pairs(manifest, placements, radius)
    full_pairs = (source_rectangle_pairs(manifest, placements, radius)
                  if manifest['randomization']['max_source_overlap_fraction'] == 0 else None)
    image = read_image(manifest, args.manifest.parent)
    gutter = gutter_continuity(manifest, args.manifest.parent)
    seam = seam_gradient(image, xs, ys, weights)
    ncc = ncc_windows(manifest, placements, image,
                      args.windows, radius, args.query_seed)
    result = {'evaluation': 'offline_final_dds_source_reuse',
              'manifest_sha256': digest(args.manifest), 'reuse_radius_m': radius,
              'dimensions_px': [manifest['width_px'], manifest['height_px']],
              'retained_weights': area, 'opaque_core_duplicate_pairs': cores, 'ncc': ncc,
              'full_source_rectangle_duplicate_pairs': full_pairs,
              'passed_full_source_exclusion': not full_pairs if full_pairs is not None else None,
              'gutter': gutter, 'seam': seam,
              'passed_core_and_ncc': not cores and not full_pairs and
              ncc['copied_window_fraction'] <= .01,
              'passed_gutter_and_seam': gutter['mismatched_shared_regions'] == 0 and
              seam['ratio'] is not None and seam['ratio'] <= 1.5,
              'limitation': 'Not rendered SIFT or natural-appearance acceptance.'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(
        result, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'passed_core_and_ncc': result['passed_core_and_ncc'],
                      'copied_window_fraction': ncc['copied_window_fraction'],
                      'retained_weight_repeat_fraction': area['weighted_repeated_fraction'],
                      'output': str(args.output)}))
    return 0 if result['passed_core_and_ncc'] and result['passed_gutter_and_seam'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
