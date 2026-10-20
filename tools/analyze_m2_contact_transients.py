#!/usr/bin/env python3
"""Inspect saved M2 contact transients without rerunning or filtering acceptance data."""

import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path

from climbot_description.geometry import quaternion_from_rpy, rotate_vector
import numpy as np


def longest_run(mask):
    """Count consecutive raw frames satisfying a condition."""
    best = current = 0
    for value in mask:
        current = current + 1 if value else 0
        best = max(best, current)
    return best


def analyze(root):
    """Keep raw loss/spike counts alongside short-event impulse diagnostics."""
    path = root / 'native/frames.csv.gz'
    with gzip.open(path, 'rt') as stream:
        c = np.genfromtxt(stream, delimiter=',', names=True)
    resolved = json.loads((root / 'resolved.json').read_text())
    t = c['sim_time_s']
    released = c['vac_fixture_active'] == 0
    release = float(c['vac_release_time_s'][released][0])
    ids = np.flatnonzero(released & (t >= release + 1) & (t <= release + 6))
    step = float(np.median(np.diff(t)))
    q = quaternion_from_rpy(*resolved['wall_surface_pose'][3:])
    normal = np.array(rotate_vector(q, [0, 0, 1]))
    forward = np.array(rotate_vector(q, [0, 1, 0]))
    normals = np.column_stack([
        np.column_stack([c[side + '_f' + axis] for axis in 'xyz']) @ normal
        for side in ['left', 'right', 'ball']])
    i = int(ids[np.argmax(normals[ids, 2])])
    ball = normals[ids, 2]
    median = float(np.median(ball))
    pair = float(normals[i-1:i+1, 2].sum() * step)
    position = np.column_stack([c['ball_' + axis] for axis in 'xyz'])
    local = position - np.array(resolved['wall_surface_pose'][:3])
    support = {}
    for j, side in enumerate(['left', 'right', 'ball']):
        values = normals[ids, j]
        support[side] = {
            'raw_normal_quantiles_n': np.percentile(values, [0, 50, 95, 99, 100]).tolist(),
            'zero_normal_frames': int(np.count_nonzero(values == 0)),
            'maximum_zero_normal_run_steps': longest_run(values == 0),
        }
    # This diagnostic threshold is deliberately separate from traction margins.
    # Impulse averaging never erases raw zero-force frames or supplies a pass gate.
    return {
        'frames_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'scope': 'Existing B contact data only; not a T2 load or fan/leak pass',
        'step_s': step, 'release_s': release, 'scored_frames': len(ids),
        'supports': support,
        'ball_spike_threshold_n': 750,
        'ball_frames_over_threshold': int(np.count_nonzero(ball > 750)),
        'ball_maximum_spike_run_steps': longest_run(ball > 750),
        'peak': {
            'time_s': float(t[i]), 'after_release_s': float(t[i] - release),
            'source_v_m': float(local[i] @ forward),
            'normal_n': float(normals[i, 2]),
            'previous_normal_n': float(normals[i-1, 2]),
            'previous_normal_velocity_mps': float(
                np.array([c['ball_v' + axis][i-1] for axis in 'xyz']) @ normal),
            'peak_and_previous_normal_impulse_ns': pair,
            'median_reference_two_step_impulse_ns': 2 * step * median,
            'pair_impulse_relative_difference': pair / (2 * step * median) - 1,
            'maximum_one_step_outward_displacement_m': float(
                np.max(np.abs(np.diff(local[ids] @ normal)))),
            'contact_points': int(c['ball_points'][i]),
        },
        'maximum_simultaneously_zero_normals': int(
            np.count_nonzero(normals[ids] == 0, axis=1).max()),
        'all_support_normals_simultaneously_zero_frames': int(
            np.count_nonzero(np.all(normals[ids] == 0, axis=1))),
        'runtime_failed': bool(np.any(c['vac_runtime_failed'])),
        'maximum_stop_reaction_n': float(c['vac_skirt_stop_previous_normal_n'][ids].max()),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-root', type=Path,
                        default=Path(os.environ.get('CLIMBOT_DATA_ROOT',
                                                    Path.home() / 'climbot_data')))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    names = ['m2_robot_patch_20261019_drive_bridge',
             'm2_robot_patch_20261019_heightmap_control']
    result = {'scope': 'T2 entry: raw contact/impulse diagnosis; no new simulation',
              'cases': {name: analyze(args.data_root / name) for name in names}}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    for name, case in result['cases'].items():
        print(json.dumps({'case': name, 'peak_n': case['peak']['normal_n'],
                          'zero_frames': case['supports']['ball']['zero_normal_frames'],
                          'longest_zero_run': case['supports']['ball'][
                              'maximum_zero_normal_run_steps'],
                          'pair_impulse_relative_difference': case['peak'][
                              'pair_impulse_relative_difference']}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
