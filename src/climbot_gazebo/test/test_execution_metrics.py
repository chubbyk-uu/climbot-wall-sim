"""Verify complete-task execution quality metrics."""

import json
import math

from climbot_gazebo.execution_metrics import coefficient_of_variation
from climbot_gazebo.execution_metrics import count_visible_reversals
from climbot_gazebo.execution_metrics import execution_quality
from climbot_gazebo.execution_metrics import scan_line_spacing
import pytest


def _row(segment, x, y, yaw=0.0, heading_error=0.0, angular=0.0,
         filtered_yaw=None):
    return {
        'segment': segment,
        'truth_x_m': x,
        'truth_y_m': y,
        'truth_yaw_rad': yaw,
        'filtered_yaw_rad': yaw if filtered_yaw is None else filtered_yaw,
        'reference_start_x_m': 0.0,
        'reference_start_y_m': 0.0,
        'reference_end_x_m': 1.0,
        'reference_end_y_m': 0.0,
        'heading_error_rad': heading_error,
        'command_angular_rps': angular,
        'scored_line_sample': 1,
    }


def test_quality_reports_endpoint_length_drift_and_compensation():
    rows = [
        _row(0, 0.0, 0.010, yaw=0.10, heading_error=0.02, angular=0.04),
        _row(0, 0.5, 0.015, yaw=0.08, angular=-0.03),
        _row(0, 0.99, 0.020, yaw=0.06, angular=0.01),
    ]
    result = execution_quality(rows, [1], [1.0])
    assert result['actual_path_length_m'] == pytest.approx(0.99005, rel=1e-4)
    assert result['actual_to_planned_length_ratio'] == pytest.approx(0.99005, rel=1e-4)
    assert result['maximum_endpoint_error_m'] == pytest.approx(math.sqrt(0.0005))
    assert result['maximum_turn_end_heading_error_deg'] == pytest.approx(
        math.degrees(0.02))
    assert result['maximum_horizontal_height_drift_m'] == pytest.approx(0.010)
    assert result['maximum_heading_compensation_deg'] == pytest.approx(
        math.degrees(0.10))
    assert result['maximum_tracking_angular_speed_rps'] == pytest.approx(0.04)


def test_quality_excludes_approach_and_does_not_label_vertical_height_drift():
    approach = _row(-1, -1.0, 0.0)
    vertical = [_row(0, 0.0, 0.0), _row(0, 0.0, 1.0)]
    for row in vertical:
        row['reference_end_x_m'] = 0.0
        row['reference_end_y_m'] = 1.0
    result = execution_quality([approach, *vertical], [1], [1.0])
    assert result['actual_path_length_m'] == pytest.approx(1.0)
    assert result['maximum_horizontal_height_drift_m'] is None
    assert result['segments'][0]['horizontal_height_drift_m'] is None


def test_turn_end_heading_error_is_measured_from_truth_not_the_filter():
    """A drifting filter must widen the reported error, not hide inside it."""
    honest = execution_quality(
        [_row(0, 0.0, 0.0, yaw=0.10, heading_error=0.02)], [1], [1.0])
    assert honest['maximum_turn_end_heading_error_deg'] == pytest.approx(
        math.degrees(0.02))

    # Same controller report, but the filter believes the robot is 0.05 rad
    # further round than it truly is, so the true miss is 0.02 + 0.05.
    drifting = execution_quality(
        [_row(0, 0.0, 0.0, yaw=0.10, heading_error=0.02, filtered_yaw=0.15)],
        [1], [1.0])
    assert drifting['maximum_turn_end_heading_error_deg'] == pytest.approx(
        math.degrees(0.07))


def test_turn_end_heading_error_is_omitted_without_a_filtered_pose():
    result = execution_quality(
        [_row(0, 0.0, 0.0, yaw=0.10, heading_error=0.02,
              filtered_yaw=math.nan)], [1], [1.0])
    assert result['segments'][0]['turn_end_heading_error_deg'] is None
    assert result['maximum_turn_end_heading_error_deg'] is None


def _scan_row(segment, x, y):
    row = _row(segment, x, y)
    row['scored_line_sample'] = 1
    return row


def _vertical_boustrophedon(actual_x):
    """Two vertical scan lines 0.40 m apart, joined by one transition."""
    waypoints = [(0.0, 0.0), (0.0, 1.0), (0.40, 1.0), (0.40, 0.0)]
    types = [1, 2, 1]
    rows = [_scan_row(0, actual_x[0], 0.0), _scan_row(0, actual_x[0], 1.0),
            _scan_row(2, actual_x[1], 1.0), _scan_row(2, actual_x[1], 0.0)]
    return rows, types, waypoints


def test_scan_line_spacing_sees_an_offset_the_cross_track_error_cannot():
    """A line frozen 25 mm off nominal is reported even when tracked perfectly."""
    rows, types, waypoints = _vertical_boustrophedon([0.0, 0.425])
    result = scan_line_spacing(rows, types, waypoints)
    assert result['scan_line_offsets_m'][0] == pytest.approx(0.0)
    assert result['scan_line_offsets_m'][1] == pytest.approx(0.025)
    assert result['maximum_scan_line_spacing_error_m'] == pytest.approx(0.025)


def test_scan_line_spacing_uses_one_axis_for_alternating_directions():
    """Both lines drifting the same way is a small spacing error, not a large one."""
    rows, types, waypoints = _vertical_boustrophedon([0.020, 0.425])
    result = scan_line_spacing(rows, types, waypoints)
    assert result['maximum_scan_line_offset_m'] == pytest.approx(0.025)
    assert result['maximum_scan_line_spacing_error_m'] == pytest.approx(0.005)


def test_scan_line_spacing_is_undefined_for_a_single_line():
    rows = [_scan_row(0, 0.0, 0.0), _scan_row(0, 0.0, 1.0)]
    result = scan_line_spacing(rows, [1], [(0.0, 0.0), (0.0, 1.0)])
    assert result['scan_line_offsets_m'] == []
    assert result['maximum_scan_line_spacing_error_m'] is None
    assert result['applicable'] is False
    assert 'fewer than two parallel scan lines' in result['not_applicable_reason']


def test_every_metric_a_summary_carries_is_valid_json():
    """These summaries are meant to be machine-readable, and NaN is not JSON."""
    # json.dump writes the bare token NaN by default. Python reads it back, so
    # nothing in this repository noticed for twenty-six archived acceptance
    # summaries; Ruby, strict Java and Go, and every schema validator do not.
    single = scan_line_spacing(
        [_scan_row(0, 0.0, 0.0), _scan_row(0, 0.0, 1.0)], [1],
        [(0.0, 0.0), (0.0, 1.0)])
    nothing_measured = execution_quality([], [1], [1.0])
    for name, metrics in (('spacing', single), ('quality', nothing_measured)):
        # Raises ValueError on anything non-finite, which is the whole check.
        assert json.dumps(metrics, allow_nan=False), name


def test_scan_line_spacing_rejects_mismatched_waypoints():
    with pytest.raises(ValueError):
        scan_line_spacing([], [1, 2, 1], [(0.0, 0.0), (0.0, 1.0)])


def test_quality_rejects_inconsistent_planning_metadata():
    with pytest.raises(ValueError):
        execution_quality([], [1], [])


def test_spacing_ignores_a_scan_line_that_crosses_the_sweep():
    """A top-edge finishing scan is a SCAN, but not one spacing is defined over."""
    rows, types, waypoints = _vertical_boustrophedon([0.0, 0.400])
    plain = scan_line_spacing(rows, types, waypoints)
    # Append a horizontal finishing scan: transition up, then across the top.
    types = list(types) + [2, 1]
    waypoints = list(waypoints) + [(0.4, 1.0), (0.0, 1.0)]
    rows = list(rows) + [
        _scan_row(len(types) - 1, 0.4, 1.0), _scan_row(len(types) - 1, 0.0, 1.0)]
    result = scan_line_spacing(rows, types, waypoints)
    assert result['crossing_scan_lines'] == 1
    assert result['maximum_scan_line_offset_m'] == pytest.approx(
        plain['maximum_scan_line_offset_m'])
    assert result['maximum_scan_line_spacing_error_m'] == pytest.approx(
        plain['maximum_scan_line_spacing_error_m'])


def test_noise_around_zero_is_not_a_visible_reversal():
    """PROJECT_GUIDE 14.3: crossings inside the band are not snaking."""
    noise = [0.001, -0.002, 0.003, -0.001, 0.002, -0.003]
    assert count_visible_reversals(noise, 0.020) == 0


def test_one_excursion_to_each_side_is_one_reversal():
    assert count_visible_reversals([0.05, 0.0, -0.05], 0.020) == 1
    assert count_visible_reversals([0.05, -0.05, 0.05], 0.020) == 2


def test_a_single_sided_excursion_is_not_a_reversal():
    """A segment that drifts one way and stays there never reverses."""
    assert count_visible_reversals([0.0, 0.03, 0.06, 0.09], 0.020) == 0


def test_coefficient_of_variation_matches_the_population_definition():
    assert coefficient_of_variation([1.0, 1.0, 1.0]) == pytest.approx(0.0)
    # Population sigma of (9, 10, 11) is sqrt(2/3); the mean is 10.
    assert coefficient_of_variation([9.0, 10.0, 11.0]) == pytest.approx(
        math.sqrt(2.0 / 3.0) / 10.0)


def test_coefficient_of_variation_rejects_undefined_inputs():
    with pytest.raises(ValueError):
        coefficient_of_variation([])
    with pytest.raises(ValueError):
        coefficient_of_variation([1.0, -1.0])
