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

"""Attribute false SIFT correspondences to actual retained source contributions."""

import argparse
import json
from pathlib import Path

from audit_wall_randomization import digest, load_trace
import numpy as np
from wall_texture_randomization import alpha_mask, source_coordinates


def point_sources(manifest, placements, wall_points):
    """Replay each point's final alpha contributions, retaining source coordinates."""
    points = np.asarray(wall_points, np.float64)
    scale = manifest['scale_m_per_px']
    origin = manifest['region_origin_m']
    x = (points[:, 0] - origin[0]) / scale
    y = (origin[1] + manifest['region_m'][1] - points[:, 1]) / scale
    q = manifest['quilt']
    patch = q['patch_px']
    result = [[] for _ in points]
    for item in placements:
        inside = np.flatnonzero((x >= item.left) & (x < item.left + patch) &
                                (y >= item.top) & (y < item.top + patch))
        if not len(inside):
            continue
        alpha = alpha_mask(item, patch, q['overlap_px'], q['seam_feather_px'])
        px, py = x[inside] - item.left, y[inside] - item.top
        ix, iy = px.astype(int), py.astype(int)
        ix1, iy1 = np.minimum(ix + 1, patch - 1), np.minimum(iy + 1, patch - 1)
        dx, dy = px - ix, py - iy
        weights = ((1 - dx) * (1 - dy) * alpha[iy, ix] +
                   dx * (1 - dy) * alpha[iy, ix1] +
                   (1 - dx) * dy * alpha[iy1, ix] + dx * dy * alpha[iy1, ix1])
        sx, sy = source_coordinates(px, py, patch, item.orientation)
        for index, source_x, source_y, weight in zip(inside, sx, sy, weights):
            result[index] = [(a, b, w * (1 - weight)) for a, b, w in result[index]]
            result[index].append((float(source_x + item.sx), float(source_y + item.sy),
                                  float(weight)))
    return result


def attribute(manifest_path, evaluation_path, output):
    manifest, placements = load_trace(manifest_path)
    evaluation = json.loads(Path(evaluation_path).read_text())
    groups, first_points, second_points = [], [], []
    for name, group in evaluation['negatives'].items():
        for score in group['scores']:
            if score['truth'] is None:
                continue
            truth = score['truth']
            count = len(truth['wrong_first_wall_points_m'])
            groups.append((name, score['match']['first'], score['match']['second'],
                           len(first_points), count))
            first_points.extend(truth['wrong_first_wall_points_m'])
            second_points.extend(truth['wrong_second_wall_points_m'])
    if not first_points:
        raise ValueError('no false correspondence points to attribute')
    first = point_sources(manifest, placements, first_points)
    second = point_sources(manifest, placements, second_points)
    rows = []
    for name, a, b, start, count in groups:
        copied = 0
        for index in range(start, start + count):
            same_source = any(
                min(wa, wb) >= .25 and np.hypot(ax - bx, ay - by) *
                manifest['scale_m_per_px'] <= .001
                for ax, ay, wa in first[index] for bx, by, wb in second[index])
            copied += int(same_source)
        rows.append({'group': name, 'first': a, 'second': b,
                     'false_correspondences': count,
                     'same_source_correspondences_with_both_weights_ge_0_25': copied})
    result = {
        'manifest_sha256': digest(manifest_path), 'evaluation_sha256': digest(evaluation_path),
        'source_sha256': manifest['source_sha256'],
        'method': 'final alpha >=0.25 in both locations; source-coordinate distance <=1mm',
        'false_correspondences': len(first_points),
        'source_confirmed_false_correspondences': sum(
            row['same_source_correspondences_with_both_weights_ge_0_25'] for row in rows),
        'pairs': rows,
        'limitation': ('Conservative source-index attribution; remaining errors '
                       'are not proven unrelated to copies.'),
    }
    Path(output).write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps({key: result[key] for key in (
        'false_correspondences', 'source_confirmed_false_correspondences')}))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--evaluation', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output must be new')
    attribute(args.manifest, args.evaluation, args.output)


if __name__ == '__main__':
    main()
