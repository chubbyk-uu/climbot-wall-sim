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

"""
Every built-in evaluation case must satisfy the executor's task contract.

The cases are built in a Python script and validated in C++ by
``coverage_execution.cpp``. Nothing connected the two, so a case could be
rejected at run time by a rule the script had never been checked against --
which is exactly how the straight_line case came to place both of its
waypoints outside its own coverage_region. This mirrors the C++ predicate
closely enough to catch that class of mistake before a run starts.
"""

import importlib.util
import math
import os

from ament_index_python.packages import get_package_prefix
import pytest

TOLERANCE = 1e-6


def _load_script():
    path = os.path.join(
        get_package_prefix('climbot_gazebo'),
        'lib', 'climbot_gazebo', 'evaluate_coverage_execution.py')
    spec = importlib.util.spec_from_file_location('evaluate_coverage_execution', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _point_in_polygon(x, y, polygon, tolerance=TOLERANCE):
    """Port of pointInPolygon from climbot_control/src/coverage_execution.cpp."""
    points = polygon.points
    if len(points) < 3:
        return False
    orientation = 0
    for index, first in enumerate(points):
        second = points[(index + 1) % len(points)]
        edge_x = second.x - first.x
        edge_y = second.y - first.y
        edge_length = math.hypot(edge_x, edge_y)
        if edge_length <= 1e-9:
            return False
        side = edge_x * (y - first.y) - edge_y * (x - first.x)
        if abs(side) / edge_length <= tolerance:
            continue
        current = 1 if side > 0.0 else -1
        if orientation == 0:
            orientation = current
        elif current != orientation:
            return False
    return True


class _Parameter:

    def __init__(self, value):
        self.value = value


class _Builder:
    """The task builders without the node: they touch only these two members."""

    def __init__(self, module, corners, parameters):
        self._module = module
        self.motion_region_corners = corners
        self._parameters = parameters

    def get_parameter(self, name):
        return _Parameter(self._parameters[name])

    def build(self, case, x, y):
        node_class = self._module.CoverageExecutionEvaluator
        builder = {
            'vertical_rectangle': node_class._vertical_rectangle,
            'short_top_trapezoid': node_class._short_top_trapezoid,
            'straight_line': node_class._straight_line,
        }[case]
        task = builder(self, x, y)
        task.motion_region.points = node_class._motion_region(self)
        return task


@pytest.fixture(scope='module')
def module():
    return _load_script()


@pytest.fixture
def builder(module):
    # A generous rectangle: these cases are checked for internal consistency,
    # not for fitting one particular wall.
    return _Builder(
        module, (0.0, 0.0, 10.0, 8.0),
        {
            'straight_line_bearing_deg': 0.0,
            'straight_line_length_m': 2.0,
            'straight_line_start_offset_m': 0.6,
            'straight_line_approach_bearing_deg': float('nan'),
        })


CASES = ['vertical_rectangle', 'short_top_trapezoid', 'straight_line']


@pytest.mark.parametrize('case', CASES)
def test_every_nominal_waypoint_lies_inside_coverage_region(builder, case):
    task = builder.build(case, 3.0, 3.0)
    for index, pose in enumerate(task.waypoints):
        assert _point_in_polygon(
            pose.position.x, pose.position.y, task.coverage_region), (
            f'{case} waypoint {index} at '
            f'({pose.position.x:.3f}, {pose.position.y:.3f}) is outside its coverage_region')


@pytest.mark.parametrize('case', CASES)
def test_coverage_region_lies_inside_motion_region(builder, case):
    task = builder.build(case, 3.0, 3.0)
    for index, point in enumerate(task.coverage_region.points):
        assert _point_in_polygon(point.x, point.y, task.motion_region), (
            f'{case} coverage_region corner {index} is outside motion_region')


@pytest.mark.parametrize('case', CASES)
def test_segment_counts_match_the_waypoints(builder, case):
    task = builder.build(case, 3.0, 3.0)
    assert len(task.waypoints) >= 2
    assert len(task.segment_types) + 1 == len(task.waypoints)


@pytest.mark.parametrize('case', CASES)
def test_no_segment_has_zero_length(builder, case):
    task = builder.build(case, 3.0, 3.0)
    for index in range(len(task.waypoints) - 1):
        first = task.waypoints[index].position
        second = task.waypoints[index + 1].position
        assert math.hypot(second.x - first.x, second.y - first.y) > 1e-9


@pytest.mark.parametrize('bearing_deg', [0.0, 37.0, 90.0, 180.0, -125.0])
def test_a_straight_line_stays_valid_at_any_bearing(builder, bearing_deg):
    # The region is built from the along/normal basis, so a bearing that is not
    # axis aligned is the case most likely to expose a sign or ordering error.
    builder._parameters['straight_line_bearing_deg'] = bearing_deg
    task = builder.build('straight_line', 5.0, 4.0)
    for pose in task.waypoints:
        assert _point_in_polygon(
            pose.position.x, pose.position.y, task.coverage_region)


def test_the_predicate_rejects_a_point_outside_the_region(builder):
    # Guards the port above: a check that accepts everything proves nothing.
    task = builder.build('straight_line', 3.0, 3.0)
    outside = task.waypoints[0].position
    assert not _point_in_polygon(outside.x - 0.05, outside.y, task.coverage_region)
