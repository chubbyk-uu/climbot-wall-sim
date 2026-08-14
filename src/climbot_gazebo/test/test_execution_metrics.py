"""Verify complete-task execution quality metrics."""

import math

from climbot_gazebo.execution_metrics import execution_quality
import pytest


def _row(segment, x, y, yaw=0.0, heading_error=0.0, angular=0.0):
    return {
        'segment': segment,
        'truth_x_m': x,
        'truth_y_m': y,
        'truth_yaw_rad': yaw,
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


def test_quality_rejects_inconsistent_planning_metadata():
    with pytest.raises(ValueError):
        execution_quality([], [1], [])
