"""Fit the in-place turn-slip coefficient the controller feeds forward."""

# PROJECT_GUIDE 10.4 and 10.7: the controller pre-compensates a turn by the
# drop it predicts from turn_slip_per_degree_m. That constant is wall-specific
# - it follows the friction and WheelSlip parameters - so a new surface has to
# be able to re-derive it rather than inherit a number someone once picked.

import math


def _solve(matrix, vector):
    """Solve a small dense system by Gaussian elimination with partial pivoting."""
    size = len(vector)
    rows = [list(matrix[index]) + [vector[index]] for index in range(size)]
    for column in range(size):
        pivot = max(range(column, size), key=lambda r: abs(rows[r][column]))
        if abs(rows[pivot][column]) <= 1e-12:
            raise ValueError('Turn set does not constrain the fit.')
        rows[column], rows[pivot] = rows[pivot], rows[column]
        for other in range(size):
            if other == column:
                continue
            factor = rows[other][column] / rows[column][column]
            for term in range(column, size + 1):
                rows[other][term] -= factor * rows[column][term]
    return [rows[index][size] / rows[index][index] for index in range(size)]


def _equations(record):
    """Return the two rows one turn contributes to the joint fit."""
    # The reported point is p = centre + R(yaw) * offset, so rotating in place
    # moves it by R(end)*offset - R(start)*offset even with no sliding at all.
    # Gravity slip adds a downhill displacement proportional to the turn size.
    # Both are unknown, and they are not separable one after the other: a fit
    # that solves for the offset alone absorbs part of the real slide into it.
    start = math.radians(float(record['start_heading_deg']))
    end = math.radians(float(record['end_heading_deg']))
    angle = abs(float(record['angle_deg']))
    return (
        ([math.cos(end) - math.cos(start), -(math.sin(end) - math.sin(start)), 0.0],
         float(record['horizontal_mm'])),
        ([math.sin(end) - math.sin(start), math.cos(end) - math.cos(start), -angle],
         float(record['vertical_mm'])),
    )


def fit(records):
    """Jointly fit the reference offset in metres and the slip per degree."""
    if len(records) < 2:
        raise ValueError('At least two turns are required.')
    rows = [row for record in records for row in _equations(record)]
    normal = [[sum(row[0][i] * row[0][j] for row in rows) for j in range(3)]
              for i in range(3)]
    right = [sum(row[0][i] * row[1] for row in rows) for i in range(3)]
    offset_x, offset_y, slip = _solve(normal, right)
    return (offset_x / 1000.0, offset_y / 1000.0), slip / 1000.0


def slip_per_degree_ignoring_swing(records):
    """Return the coefficient a naive fit gives, for comparison only."""
    if not records:
        raise ValueError('At least one turn is required.')
    numerator = 0.0
    denominator = 0.0
    for record in records:
        angle = abs(float(record['angle_deg']))
        if angle <= 1e-9:
            continue
        numerator += angle * -float(record['vertical_mm'])
        denominator += angle * angle
    if denominator <= 0.0:
        raise ValueError('Turn set contains no non-zero angle.')
    return numerator / denominator / 1000.0


def residual_rms(records, offset, slip):
    """Return the RMS displacement the fitted model fails to explain, in metres."""
    total = 0.0
    count = 0
    for record in records:
        for row, measured in _equations(record):
            predicted = (row[0] * offset[0] * 1000.0 +
                         row[1] * offset[1] * 1000.0 +
                         row[2] * slip * 1000.0)
            total += (measured - predicted) ** 2
            count += 1
    return math.sqrt(total / count) / 1000.0 if count else math.nan


def summarise(records):
    """Return the offset self-check and the coefficient a config should carry."""
    offset, slip = fit(records)
    return {
        'reference_offset_m': offset,
        'reference_offset_magnitude_m': math.hypot(*offset),
        'turn_slip_per_degree_m': slip,
        'turn_slip_per_degree_m_ignoring_swing': slip_per_degree_ignoring_swing(
            records),
        'residual_rms_m': residual_rms(records, offset, slip),
        'turns': len(records),
    }
