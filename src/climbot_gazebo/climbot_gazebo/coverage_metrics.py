# Copyright 2026 jerry
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Truth-based two-dimensional inspection-footprint coverage metrics."""

import math


def _point_in_polygon(x, y, polygon):
    """Return whether a point lies inside or on a simple polygon."""
    inside = False
    previous_x, previous_y = polygon[-1]
    for current_x, current_y in polygon:
        edge_x = current_x - previous_x
        edge_y = current_y - previous_y
        point_x = x - previous_x
        point_y = y - previous_y
        cross = edge_x * point_y - edge_y * point_x
        dot = point_x * edge_x + point_y * edge_y
        squared_length = edge_x * edge_x + edge_y * edge_y
        if abs(cross) <= 1e-12 and -1e-12 <= dot <= squared_length + 1e-12:
            return True
        if ((current_y > y) != (previous_y > y)):
            intersection_x = (
                (previous_x - current_x) * (y - current_y) /
                (previous_y - current_y) + current_x)
            if x < intersection_x:
                inside = not inside
        previous_x, previous_y = current_x, current_y
    return inside


def _interpolated_poses(path, maximum_step):
    """Yield densely spaced poses without bridging separate scan paths."""
    if not path:
        return
    yield path[0]
    for previous, current in zip(path, path[1:]):
        distance = math.hypot(current[0] - previous[0], current[1] - previous[1])
        steps = max(1, int(math.ceil(distance / maximum_step)))
        yaw_delta = math.atan2(
            math.sin(current[2] - previous[2]),
            math.cos(current[2] - previous[2]))
        for index in range(1, steps + 1):
            fraction = index / steps
            yield (
                previous[0] + fraction * (current[0] - previous[0]),
                previous[1] + fraction * (current[1] - previous[1]),
                previous[2] + fraction * yaw_delta,
            )


def footprint_coverage(polygon, scan_paths, width, length, resolution=0.01,
                       forward_offset=0.0):
    """Rasterize oriented rectangular footprints swept along actual scan paths."""
    if len(polygon) < 3:
        raise ValueError('coverage polygon requires at least three points')
    if (width <= 0.0 or length <= 0.0 or resolution <= 0.0 or
            not math.isfinite(forward_offset) or forward_offset < 0.0):
        raise ValueError('footprint dimensions and resolution must be positive')
    values = [coordinate for point in polygon for coordinate in point]
    if not all(math.isfinite(value) for value in values):
        raise ValueError('coverage polygon must be finite')

    minimum_x = min(point[0] for point in polygon)
    maximum_x = max(point[0] for point in polygon)
    minimum_y = min(point[1] for point in polygon)
    maximum_y = max(point[1] for point in polygon)
    columns = max(1, int(math.ceil((maximum_x - minimum_x) / resolution)))
    rows = max(1, int(math.ceil((maximum_y - minimum_y) / resolution)))
    region = bytearray(columns * rows)
    covered = bytearray(columns * rows)
    region_cells = 0
    for row in range(rows):
        y = minimum_y + (row + 0.5) * resolution
        for column in range(columns):
            x = minimum_x + (column + 0.5) * resolution
            if _point_in_polygon(x, y, polygon):
                region[row * columns + column] = 1
                region_cells += 1
    if region_cells == 0:
        raise ValueError('coverage polygon is smaller than the raster resolution')

    half_length = 0.5 * length
    half_width = 0.5 * width
    maximum_step = 0.5 * min(length, resolution)
    for path in scan_paths:
        for x, y, yaw in _interpolated_poses(path, maximum_step):
            cosine = math.cos(yaw)
            sine = math.sin(yaw)
            x += forward_offset * cosine
            y += forward_offset * sine
            extent_x = abs(cosine) * half_length + abs(sine) * half_width
            extent_y = abs(sine) * half_length + abs(cosine) * half_width
            first_column = max(
                0, int(math.floor((x - extent_x - minimum_x) / resolution)))
            last_column = min(
                columns - 1,
                int(math.floor((x + extent_x - minimum_x) / resolution)))
            first_row = max(
                0, int(math.floor((y - extent_y - minimum_y) / resolution)))
            last_row = min(
                rows - 1,
                int(math.floor((y + extent_y - minimum_y) / resolution)))
            for row in range(first_row, last_row + 1):
                cell_y = minimum_y + (row + 0.5) * resolution
                for column in range(first_column, last_column + 1):
                    offset = row * columns + column
                    if not region[offset] or covered[offset]:
                        continue
                    cell_x = minimum_x + (column + 0.5) * resolution
                    delta_x = cell_x - x
                    delta_y = cell_y - y
                    along = delta_x * cosine + delta_y * sine
                    cross = -delta_x * sine + delta_y * cosine
                    if (abs(along) <= half_length + 1e-12 and
                            abs(cross) <= half_width + 1e-12):
                        covered[offset] = 1

    covered_cells = sum(covered)
    ratio = covered_cells / region_cells
    return {
        'ratio': ratio,
        'missed_ratio': 1.0 - ratio,
        'covered_cells': covered_cells,
        'region_cells': region_cells,
        'resolution_m': resolution,
        'covered_area_m2': covered_cells * resolution * resolution,
        'region_area_m2': region_cells * resolution * resolution,
    }
