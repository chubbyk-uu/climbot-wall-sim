"""Verify the turn-slip fit separates kinematic swing from real sliding."""

import math

from climbot_gazebo.turn_slip_model import fit
from climbot_gazebo.turn_slip_model import residual_rms
from climbot_gazebo.turn_slip_model import slip_per_degree_ignoring_swing
from climbot_gazebo.turn_slip_model import summarise
import pytest


def _turn(start_deg, angle_deg, offset=(0.0, 0.0), slip_per_degree=0.0):
    """Synthesise one turn: pure kinematic swing plus a downhill slide."""
    start = math.radians(start_deg)
    end = start + math.radians(angle_deg)
    swing_x = offset[0] * (math.cos(end) - math.cos(start)) - offset[1] * (
        math.sin(end) - math.sin(start))
    swing_y = offset[0] * (math.sin(end) - math.sin(start)) + offset[1] * (
        math.cos(end) - math.cos(start))
    return {
        'angle_deg': angle_deg,
        'start_heading_deg': start_deg,
        'end_heading_deg': start_deg + angle_deg,
        'horizontal_mm': swing_x,
        'vertical_mm': swing_y - slip_per_degree * abs(angle_deg),
    }


def _balanced(offset=(0.0, 0.0), slip_per_degree=0.0):
    """Build a both-directions sweep, as the calibration script drives."""
    return [
        _turn(start, angle, offset, slip_per_degree)
        for start, angle in (
            (90.0, 30.0), (120.0, -30.0), (90.0, 90.0), (180.0, -90.0),
            (90.0, 180.0), (-90.0, -180.0))
    ]


def _one_directional(offset=(0.0, 0.0), slip_per_degree=0.0):
    """Build a sweep that never reverses, so the swing cannot cancel out."""
    return [
        _turn(start, angle, offset, slip_per_degree)
        for start, angle in (
            (0.0, 30.0), (30.0, 45.0), (75.0, 90.0), (165.0, 45.0),
            (210.0, 90.0), (300.0, 30.0))
    ]


def test_a_pose_on_the_rotation_centre_fits_no_offset():
    offset, slip = fit(_balanced(slip_per_degree=0.5))
    assert math.hypot(*offset) == pytest.approx(0.0, abs=1e-9)
    assert slip == pytest.approx(0.0005, abs=1e-9)


def test_the_offset_of_a_pose_behind_the_axle_is_recovered():
    """The 2026-08-13 data set was taken about 79 mm behind the drive axle."""
    offset, slip = fit(_balanced(offset=(-79.0, 7.0), slip_per_degree=0.5))
    assert offset[0] == pytest.approx(-0.079, abs=1e-6)
    assert offset[1] == pytest.approx(0.007, abs=1e-6)
    # Fitting the offset and the slide together is what keeps the coefficient
    # right; solving for the offset first absorbs part of the slide into it.
    assert slip == pytest.approx(0.0005, abs=1e-9)


def test_a_one_directional_sweep_still_separates_the_two():
    offset, slip = fit(_one_directional(offset=(-79.0, 7.0), slip_per_degree=0.5))
    assert math.hypot(offset[0] + 0.079, offset[1] - 0.007) == pytest.approx(
        0.0, abs=1e-6)
    assert slip == pytest.approx(0.0005, abs=1e-9)


def test_ignoring_the_swing_survives_a_balanced_sweep_but_not_a_one_sided_one():
    """Why the hand-picked constant was right despite coming from bad data."""
    balanced = _balanced(offset=(-79.0, 7.0), slip_per_degree=0.5)
    assert slip_per_degree_ignoring_swing(balanced) == pytest.approx(
        0.0005, abs=2e-5)
    one_sided = _one_directional(offset=(-79.0, 7.0), slip_per_degree=0.5)
    assert slip_per_degree_ignoring_swing(one_sided) != pytest.approx(
        0.0005, abs=5e-5)


def test_a_fitted_model_explains_synthetic_data_exactly():
    records = _balanced(offset=(-79.0, 7.0), slip_per_degree=0.5)
    offset, slip = fit(records)
    assert residual_rms(records, offset, slip) == pytest.approx(0.0, abs=1e-9)


def test_summary_reports_both_the_check_and_the_coefficient():
    result = summarise(_balanced(offset=(-79.0, 7.0), slip_per_degree=0.5))
    assert result['turns'] == 6
    assert result['reference_offset_magnitude_m'] == pytest.approx(
        math.hypot(0.079, 0.007), abs=1e-6)
    assert result['turn_slip_per_degree_m'] == pytest.approx(0.0005, abs=1e-9)


def test_rejects_input_that_cannot_constrain_a_fit():
    with pytest.raises(ValueError):
        fit([_turn(90.0, 30.0)])
    with pytest.raises(ValueError):
        slip_per_degree_ignoring_swing([])
    with pytest.raises(ValueError):
        slip_per_degree_ignoring_swing([_turn(90.0, 0.0)])
