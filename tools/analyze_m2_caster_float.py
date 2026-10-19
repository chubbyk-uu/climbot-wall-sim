#!/usr/bin/env python3
"""Compare saved fixed/floating caster traces without changing raw load gates."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from validate_vacuum_motion import frame_data


def metrics(root):
    c = frame_data(root / 'native/frames.csv')
    t = c['sim_time_s']
    release = float(c['vac_release_time_s'][c['vac_fixture_active'] == 0][0])
    normal = np.column_stack([c[s + '_fx'] for s in ['left', 'right', 'ball']])
    tangent = np.column_stack([np.hypot(c[s + '_fy'], c[s + '_fz'])
                               for s in ['left', 'right']])
    margin = 1.1 * normal[:, :2] / np.maximum(tangent, 1e-12)
    # Inverse rotation of the ball centre relative to the chassis. Its body-y
    # coordinate is the housing prismatic displacement; ball spin changes no centre.
    v = np.column_stack([c['ball_' + a] - c['pose_' + a] for a in 'xyz'])
    q = np.column_stack([c['q' + a] for a in 'xyz'])
    w = c['qw'][:, None]
    v_body = v - 2 * w * np.cross(q, v) + 2 * np.cross(q, np.cross(q, v))
    travel = v_body[:, 1]
    windows = {}
    for name, start, end in [('motion_and_braking', 1, 5.2),
                             ('parking_first_two_seconds', 5.2, 7.2),
                             ('hold_after_two_seconds', 7.2, float(t[-1] - release))]:
        ids = (t >= release + start) & (t <= release + end)
        if not ids.any():
            continue
        windows[name] = {
            'duration_s': float(t[ids][-1] - t[ids][0]),
            'minimum_drive_margin': margin[ids].min(axis=0).tolist(),
            'drive_margin_below_1_5_frames': (margin[ids] < 1.5).sum(axis=0).tolist(),
            'minimum_normals_n': normal[ids].min(axis=0).tolist(),
            'maximum_absolute_ball_lateral_force_n': float(np.abs(c['ball_fy'][ids]).max()),
            'ball_lateral_force_abs_p50_p95_n': np.percentile(
                np.abs(c['ball_fy'][ids]), [50, 95]).tolist(),
            'housing_lateral_range_mm': (1000 * np.array([
                travel[ids].min(), travel[ids].max()])).tolist(),
            'pressure_range_pa': [float(c['vac_delta_pa'][ids].min()),
                                  float(c['vac_delta_pa'][ids].max())],
        }
    active = t >= release + 1
    resolved = json.loads((root / 'resolved.json').read_text())
    cfg = resolved.get('caster_compliance')
    within_limits = None if cfg is None else bool(
        travel[active].min() > cfg['lower_m'] and travel[active].max() < cfg['upper_m'])
    return {
        'release_s': release, 'windows': windows,
        'no_runtime_failure': not bool(np.any(c['vac_runtime_failed'])),
        'no_stop_contact': not bool(np.any(c['vac_skirt_stop_previous_normal_n'][active])),
        'housing_stays_inside_limits': within_limits,
        'frames_sha256': hashlib.sha256((root / 'native/frames.csv').read_bytes()).hexdigest(),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--before', type=Path, required=True)
    parser.add_argument('--after', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = {'scope': 'Raw native margins; no averaging or force redistribution for acceptance',
              'before': metrics(args.before), 'after': metrics(args.after)}
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
