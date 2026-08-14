"""Truth-based quality metrics derived from a complete coverage trajectory."""

# Every reported value is measured from the Gazebo truth pose. Controller
# estimates appear only where they define the target the robot was aiming at.

import math


def _finite(row, names):
    return all(math.isfinite(float(row[name])) for name in names)


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def execution_quality(rows, segment_types, planned_lengths, scan_type=1):
    """Summarize spatial execution quality without depending on ROS messages."""
    if len(segment_types) != len(planned_lengths):
        raise ValueError('Segment types and planned lengths must have equal size.')
    if any(length <= 0.0 or not math.isfinite(length)
           for length in planned_lengths):
        raise ValueError('Every planned segment length must be finite and positive.')

    grouped = {index: [] for index in range(len(segment_types))}
    actual_length = 0.0
    previous = None
    for row in rows:
        segment = int(row['segment'])
        if segment not in grouped or not _finite(row, ('truth_x_m', 'truth_y_m')):
            previous = None
            continue
        grouped[segment].append(row)
        point = (float(row['truth_x_m']), float(row['truth_y_m']))
        if previous is not None and previous[0] == segment:
            actual_length += math.hypot(
                point[0] - previous[1][0], point[1] - previous[1][1])
        previous = (segment, point)

    segment_metrics = []
    for segment, values in grouped.items():
        scored = [row for row in values if int(row['scored_line_sample'])]
        if not scored:
            continue
        last = scored[-1]
        reference_fields = (
            'reference_start_x_m', 'reference_start_y_m',
            'reference_end_x_m', 'reference_end_y_m')
        if not _finite(last, reference_fields):
            continue
        endpoint_error = math.hypot(
            float(last['truth_x_m']) - float(last['reference_end_x_m']),
            float(last['truth_y_m']) - float(last['reference_end_y_m']))
        # The controller reports its own error against the gravity-compensated
        # target using the filtered pose, which cannot reveal a drifting filter.
        # Recover that target and re-measure it with truth (PROJECT_GUIDE 14.5).
        entry = scored[0]
        turn_end_error = None
        if _finite(entry, ('truth_yaw_rad', 'filtered_yaw_rad', 'heading_error_rad')):
            target_yaw = _wrap(
                float(entry['filtered_yaw_rad']) + float(entry['heading_error_rad']))
            turn_end_error = abs(_wrap(float(entry['truth_yaw_rad']) - target_yaw))
        reference_heading = math.atan2(
            float(last['reference_end_y_m']) - float(last['reference_start_y_m']),
            float(last['reference_end_x_m']) - float(last['reference_start_x_m']))
        heading_offsets = [
            abs(_wrap(float(row['truth_yaw_rad']) - reference_heading))
            for row in scored]
        angular_speeds = [
            abs(float(row['command_angular_rps'])) for row in scored]
        horizontal_drift = None
        if (segment_types[segment] == scan_type and
                abs(math.sin(reference_heading)) < math.sin(math.radians(2.0))):
            horizontal_drift = abs(
                float(scored[-1]['truth_y_m']) -
                float(scored[0]['truth_y_m']))
        segment_metrics.append({
            'segment': segment,
            'endpoint_error_m': endpoint_error,
            'turn_end_heading_error_deg': (
                math.degrees(turn_end_error) if turn_end_error is not None else None),
            'horizontal_height_drift_m': horizontal_drift,
            'maximum_heading_compensation_deg': math.degrees(max(heading_offsets)),
            'maximum_tracking_angular_speed_rps': max(angular_speeds),
        })

    planned_length = sum(planned_lengths)
    horizontal_drifts = [
        value['horizontal_height_drift_m'] for value in segment_metrics
        if value['horizontal_height_drift_m'] is not None]
    turn_end_errors = [
        value['turn_end_heading_error_deg'] for value in segment_metrics
        if value['turn_end_heading_error_deg'] is not None]
    return {
        'actual_path_length_m': actual_length,
        'planned_path_length_m': planned_length,
        'actual_to_planned_length_ratio': actual_length / planned_length,
        'maximum_endpoint_error_m': max(
            (value['endpoint_error_m'] for value in segment_metrics), default=math.nan),
        'maximum_turn_end_heading_error_deg': max(
            turn_end_errors, default=math.nan),
        'maximum_horizontal_height_drift_m': (
            max(horizontal_drifts) if horizontal_drifts else None),
        'maximum_heading_compensation_deg': max(
            (value['maximum_heading_compensation_deg'] for value in segment_metrics),
            default=math.nan),
        'maximum_tracking_angular_speed_rps': max(
            (value['maximum_tracking_angular_speed_rps'] for value in segment_metrics),
            default=math.nan),
        'segments': segment_metrics,
    }
