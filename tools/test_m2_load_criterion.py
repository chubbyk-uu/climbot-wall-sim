"""Guard against cancellation, hidden loss and averaging away friction exhaustion."""

from m2_load_criterion import evaluate
import numpy as np
import pytest


def trace(step=.001):
    times = 1 + step * np.arange(round(.2 / step))
    c = {'sim_time_s': times, 'paused': np.zeros(len(times))}
    for side in ['left', 'right', 'ball']:
        c[side + '_fx'] = np.full(len(times), 100.)
        c[side + '_fy'] = np.zeros(len(times))
        c[side + '_fz'] = np.full(len(times), 50.)
        c[side + '_points'] = np.ones(len(times))
    return c


def test_opposing_forces_cannot_cancel_into_a_window_pass():
    c = trace()
    c['left_fz'][:] = np.tile([90., -90.], len(c['left_fz']) // 2)
    result = evaluate(c, 0)
    assert result['checks']['every_loaded_drive_frame_margin_above_1']
    assert not result['passed']
    assert result['window_sensitivity_ms']['50']['minimum_margin_left_right'][0] == \
        pytest.approx(110 / 90)


@pytest.mark.parametrize('force,expected', [(100., True), (120., False)])
def test_short_peak_may_lose_design_margin_but_cannot_exhaust_friction(force, expected):
    c = trace()
    c['left_fz'][80] = force
    result = evaluate(c, 0)
    assert result['checks']['sustained_50ms_margin_at_least_1_5']
    assert result['passed'] is expected
    assert result['raw']['loaded_drive_below_1_5_frames_left_right'] == [1, 0]


@pytest.mark.parametrize('length,expected', [(2, True), (3, False)])
@pytest.mark.parametrize('points_absent', [True, False])
def test_contact_or_low_load_duration_cannot_be_averaged_away(length, expected, points_absent):
    c = trace()
    c['left_fx'][80:80 + length] = 0
    c['left_fz'][80:80 + length] = 0
    if points_absent:
        c['left_points'][80:80 + length] = 0
    result = evaluate(c, 0)
    assert result['passed'] is expected
    assert result['raw']['maximum_contact_or_low_load_steps_left_right_ball'] == [length, 0, 0]


def test_windows_retain_low_load_samples_and_their_tangential_impulses():
    c = trace()
    c['left_fx'][80] = 0
    c['left_fz'][80] = 90
    c['left_points'][80] = 0
    result = evaluate(c, 0)
    expected = 1.1 * (49 * 100) / (49 * 50 + 90)
    assert result['window_sensitivity_ms']['50']['minimum_margin_left_right'][0] == \
        pytest.approx(expected)
    assert result['samples'] == 200
    assert result['raw']['drive_below_1_5_frames_left_right'] == [1, 0]


def test_persistent_low_margin_still_fails():
    c = trace()
    c['left_fz'][:] = 100
    result = evaluate(c, 0)
    assert not result['passed']
    assert result['window_sensitivity_ms']['50']['minimum_margin_left_right'][0] == \
        pytest.approx(1.1)


def test_half_millisecond_step_uses_one_hundred_samples_for_fifty_ms():
    result = evaluate(trace(.0005), 0)
    assert result['passed']
    assert result['window_sensitivity_ms']['50']['samples'] == 100


def test_late_parking_failure_is_scored_after_the_old_motion_window():
    c = {key: np.tile(value, 40) for key, value in trace().items()}
    c['sim_time_s'] = 1 + .001 * np.arange(len(c['sim_time_s']))
    c['right_fz'][-200:] = 100
    result = evaluate(c, 0)
    assert result['scoring_time_range_s'][-1] > 7
    assert not result['passed']


def test_incomplete_fifty_ms_trace_cannot_pass():
    c = {key: value[:30] for key, value in trace().items()}
    result = evaluate(c, 0)
    assert result['window_sensitivity_ms']['50'] is None
    assert not result['passed']


@pytest.mark.parametrize('invalid', ['gap', 'pause', 'nan_force', 'nan_time'])
def test_invalid_sampling_or_force_is_rejected(invalid):
    c = trace()
    if invalid == 'gap':
        c['sim_time_s'][100:] += .001
    elif invalid == 'pause':
        c['paused'][100] = 1
    elif invalid == 'nan_time':
        c['sim_time_s'][-1] = np.nan
    else:
        c['left_fx'][100] = np.nan
    with pytest.raises(ValueError):
        evaluate(c, 0)
