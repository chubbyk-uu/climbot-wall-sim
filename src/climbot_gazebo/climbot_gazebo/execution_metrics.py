"""Truth-based quality metrics derived from a complete coverage trajectory."""

# Every reported value is measured from the Gazebo truth pose. Controller
# estimates appear only where they define the target the robot was aiming at.

import math


def _finite(row, names):
    return all(math.isfinite(float(row[name])) for name in names)


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _unit_direction(waypoints, segment):
    """Return the unit direction of one segment, or None when degenerate."""
    first, second = waypoints[segment], waypoints[segment + 1]
    delta_x = second[0] - first[0]
    delta_y = second[1] - first[1]
    length = math.hypot(delta_x, delta_y)
    if length <= 1e-9:
        return None
    return (delta_x / length, delta_y / length)


def _parallel_to(waypoints, segment, direction, tolerance_deg=2.0):
    """Return whether a segment runs along direction, either way round."""
    unit = _unit_direction(waypoints, segment)
    if unit is None:
        return False
    # The magnitude of the 2-D cross product is the sine of the angle between
    # them, so an anti-parallel line still counts as parallel.
    return abs(unit[0] * direction[1] - unit[1] * direction[0]) <= math.sin(
        math.radians(tolerance_deg))


def coefficient_of_variation(values):
    """Return population standard deviation divided by the absolute mean."""
    if not values:
        raise ValueError('At least one value is required.')
    mean = sum(values) / len(values)
    if abs(mean) <= 1e-12:
        raise ValueError('Coefficient of variation requires a non-zero mean.')
    variance = sum((value - mean) ** 2 for value in values) / len(values)
    return math.sqrt(variance) / abs(mean)


def count_visible_reversals(cross_errors, excursion):
    """Count cross-track sign changes that leave the noise band on both sides."""
    # PROJECT_GUIDE 14.3: centimetre-scale zero crossings near the noise floor
    # are not snaking. Only an excursion past the band on one side, then past
    # it on the other, counts as one visible reversal.
    signs = []
    for error in cross_errors:
        sign = 1 if error > excursion else -1 if error < -excursion else 0
        if sign and (not signs or sign != signs[-1]):
            signs.append(sign)
    return max(0, len(signs) - 1)


def scan_line_spacing(rows, segment_types, waypoints, scan_type=1):
    """Measure where each executed scan line sat relative to its nominal line."""
    # PROJECT_GUIDE 14.2 and 14.3 accept a coverage run partly on the spacing
    # between adjacent scan lines, which no other metric observes: cross-track
    # error is measured against the frozen line itself, so it stays small no
    # matter where that line ended up. Positions come from truth (14.5).
    if len(waypoints) != len(segment_types) + 1:
        raise ValueError('Waypoints must hold one more entry than segments.')

    scans = [index for index, kind in enumerate(segment_types)
             if kind == scan_type]
    # A top-edge finishing scan (PROJECT_GUIDE 10.7) is a SCAN that runs across
    # the sweep, not along it. Projecting it onto the sweep normal would report
    # its whole length as an offset, so spacing is measured only over the lines
    # this metric is defined for: the parallel ones.
    crossing = []
    if scans:
        sweep = _unit_direction(waypoints, scans[0])
        if sweep is not None:
            crossing = [index for index in scans[1:]
                        if not _parallel_to(waypoints, index, sweep)]
            scans = [index for index in scans if index not in set(crossing)]
    if len(scans) < 2:
        return {
            'scan_line_offsets_m': [],
            'scan_line_spacing_errors_m': [],
            'maximum_scan_line_offset_m': math.nan,
            'maximum_scan_line_spacing_error_m': math.nan,
            'crossing_scan_lines': len(crossing),
        }

    # Adjacent scan lines run in opposite directions, so each line's own normal
    # alternates. Project every line onto one shared axis instead.
    first, second = waypoints[scans[0]], waypoints[scans[0] + 1]
    length = math.hypot(second[0] - first[0], second[1] - first[1])
    if length <= 1e-9:
        raise ValueError('The first scan line must have non-zero length.')
    axis = (-(second[1] - first[1]) / length, (second[0] - first[0]) / length)
    # Orient the axis along sweep advance so a positive offset always means
    # "further along the sweep than nominal", whichever way line 0 runs.
    advance = waypoints[scans[1]]
    if ((advance[0] - first[0]) * axis[0] +
            (advance[1] - first[1]) * axis[1]) < 0.0:
        axis = (-axis[0], -axis[1])

    grouped = {}
    for row in rows:
        segment = int(row['segment'])
        if int(row['scored_line_sample']) and _finite(
                row, ('truth_x_m', 'truth_y_m')):
            grouped.setdefault(segment, []).append(row)

    nominal = []
    actual = []
    for segment in scans:
        samples = grouped.get(segment)
        if not samples:
            continue
        start = waypoints[segment]
        nominal.append(start[0] * axis[0] + start[1] * axis[1])
        actual.append(sum(
            float(row['truth_x_m']) * axis[0] +
            float(row['truth_y_m']) * axis[1]
            for row in samples) / len(samples))

    offsets = [a - n for a, n in zip(actual, nominal)]
    errors = [abs(actual[i + 1] - actual[i]) - abs(nominal[i + 1] - nominal[i])
              for i in range(len(actual) - 1)]
    return {
        'scan_line_offsets_m': offsets,
        'scan_line_spacing_errors_m': errors,
        'maximum_scan_line_offset_m': max(
            (abs(value) for value in offsets), default=math.nan),
        'maximum_scan_line_spacing_error_m': max(
            (abs(value) for value in errors), default=math.nan),
        'crossing_scan_lines': len(crossing),
    }


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
