#!/usr/bin/env python3
"""Approved M2 load criterion; retain all samples and all raw violations."""

import numpy as np


CRITERION = {
    'version': 'm2_load_v2_20261019',
    'mu': 1.1,
    'window_ms': 50,
    'minimum_window_margin': 1.5,
    'minimum_loaded_instant_margin_exclusive': 1.0,
    'loaded_drive_threshold_n': 5.0,
    'support_floors_n_left_right_ball': [5.0, 5.0, 10.0],
    'maximum_contact_or_low_load_run_steps': 2,
    'start_after_release_s': 1.0,
    'window_denominator': 'sum(norm(F_t) * dt); no signed-force cancellation',
    'sensitivity_windows_ms': [20, 50, 100],
}


def resolved_mu(resolved):
    """Old frozen resolved inputs predate friction metadata and used mu=1.1."""
    value = resolved.get('friction', {}).get('effective_drive_mu', 1.1)
    if not np.isfinite(value) or value <= 0:
        raise ValueError('invalid resolved friction coefficient')
    return float(value)


def longest_run(flags):
    """Return the longest consecutive true run, including either array boundary."""
    changes = np.diff(np.r_[False, flags, False].astype(int))
    start, end = np.flatnonzero(changes == 1), np.flatnonzero(changes == -1)
    return int((end - start).max()) if len(start) else 0


def rolling_sum(values, count):
    """Integrate complete consecutive sample windows, keeping low-load frames."""
    integral = np.vstack([np.zeros((1, values.shape[1])), np.cumsum(values, axis=0)])
    return integral[count:] - integral[:-count]


def evaluate(columns, release_s, *, mu=1.1):
    """Score the existing release+1 s boundary through the entire remaining trace."""
    if not np.isfinite(mu) or mu <= 0:
        raise ValueError('friction coefficient must be finite and positive')
    criterion = dict(CRITERION, mu=float(mu))
    if mu != CRITERION['mu']:
        criterion['version'] = 'm2_load_v3_explicit_mu_20261022'
    names = ['left', 'right', 'ball']
    times = columns['sim_time_s']
    if not np.isfinite(times).all() or not np.isfinite(release_s):
        raise ValueError('nonfinite time or release boundary')
    ids = np.flatnonzero(times >= release_s + CRITERION['start_after_release_s'])
    if len(ids) < 2 or release_s < 0:
        raise ValueError('load scoring requires a released, nonempty trace')
    t = columns['sim_time_s'][ids]
    step = float(np.median(np.diff(t)))
    if (not np.isfinite(t).all() or step <= 0 or
            not np.allclose(np.diff(t), step, rtol=0, atol=1e-9) or
            np.any(columns['paused'][ids] != 0)):
        raise ValueError('load windows require consecutive, unpaused physical steps')
    normal = np.column_stack([columns[s + '_fx'][ids] for s in names])
    tangent = np.column_stack([
        np.hypot(columns[s + '_fy'][ids], columns[s + '_fz'][ids]) for s in names[:2]])
    points = np.column_stack([columns[s + '_points'][ids] for s in names])
    if not all(np.isfinite(v).all() for v in [normal, tangent, points]):
        raise ValueError('nonfinite force or contact samples')
    floors = np.array(CRITERION['support_floors_n_left_right_ball'])
    absent = points == 0
    low = normal <= floors
    unsupported = absent | low
    gap = [longest_run(unsupported[:, i]) for i in range(3)]
    margin = mu * normal[:, :2] / np.maximum(tangent, 1e-12)
    loaded = normal[:, :2] > CRITERION['loaded_drive_threshold_n']
    loaded_min = [float(margin[:, i][loaded[:, i]].min())
                  if loaded[:, i].any() else None for i in range(2)]
    sensitivity = {}
    for ms in CRITERION['sensitivity_windows_ms']:
        count = round(ms / 1000 / step)
        if count < 1 or abs(count * step - ms / 1000) > 1e-9:
            raise ValueError('physical step must divide each window duration')
        if len(t) < count:
            sensitivity[str(ms)] = None
            continue
        # Uniform dt was checked above, so it cancels between the two impulses.
        ratio = mu * rolling_sum(normal[:, :2], count) / np.maximum(
            rolling_sum(tangent, count), 1e-12)
        sensitivity[str(ms)] = {'samples': count,
                                'minimum_margin_left_right': ratio.min(axis=0).tolist()}
    gate = sensitivity[str(CRITERION['window_ms'])]
    checks = {
        'sustained_50ms_margin_at_least_1_5': gate is not None and
        min(gate['minimum_margin_left_right']) >= CRITERION['minimum_window_margin'],
        'every_loaded_drive_frame_margin_above_1': all(
            x is not None and x > 1.0 for x in loaded_min),
        'contact_or_low_load_runs_at_most_two_steps': max(gap) <= 2,
    }
    return {
        'criterion': criterion, 'passed': all(checks.values()), 'checks': checks,
        'step_s': step, 'samples': len(t),
        'scoring_time_range_s': [float(t[0]), float(t[-1])],
        'minimum_loaded_instant_margin_left_right': loaded_min,
        'window_sensitivity_ms': sensitivity,
        'raw': {
            'minimum_drive_margin_left_right': margin.min(axis=0).tolist(),
            'drive_below_1_5_frames_left_right': (margin < 1.5).sum(axis=0).tolist(),
            'loaded_drive_below_1_5_frames_left_right': (
                (margin < 1.5) & loaded).sum(axis=0).tolist(),
            'loaded_drive_at_or_below_1_frames_left_right': (
                (margin <= 1) & loaded).sum(axis=0).tolist(),
            'maximum_contact_loss_steps_left_right_ball': [
                longest_run(absent[:, i]) for i in range(3)],
            'contact_loss_frames_left_right_ball': absent.sum(axis=0).tolist(),
            'low_load_frames_left_right_ball': low.sum(axis=0).tolist(),
            'contact_or_low_load_frames_left_right_ball': unsupported.sum(axis=0).tolist(),
            'maximum_low_load_steps_left_right_ball': [longest_run(low[:, i]) for i in range(3)],
            'maximum_contact_or_low_load_steps_left_right_ball': gap,
            'maximum_simultaneous_contact_losses': int(absent.sum(axis=1).max()),
        },
    }
