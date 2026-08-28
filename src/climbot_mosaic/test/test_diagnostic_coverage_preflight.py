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

"""Contracts for the P2-06 discrete diagnostic-coverage preflight."""

from climbot_mosaic.diagnostic_coverage_preflight import (
    DiagnosticCoveragePreflightError,
    planned_exposures,
    planned_scan_segments,
    preflight_diagnostic_coverage,
    preflight_diagnostic_coverage_set,
)
import pytest


def _task(**overrides):
    result = {
        'task_id': 'preflight-test', 'region_type': 'rectangle',
        'lower_left': [1.0, 1.0], 'upper_right': [3.0, 3.0],
        'overlap_ratio': 0.20, 'sweep_direction': 'horizontal',
        'start_corner': 'lower_left',
    }
    result.update(overrides)
    return result


def _camera():
    return {'inspection_camera': {
        'footprint': {'effective_width_m': 0.5, 'effective_length_m': 0.5},
        'optical_mount': {'center_xyz_m': [0.25, 0.0, 0.2]},
    }}


def _robot():
    return {'robot': {'footprint': {
        'length_m': 0.2, 'width_m': 0.2, 'edge_clearance_m': 0.0,
    }}}


def _wall():
    return {'wall': {'surface': {'width_m': 4.0, 'height_m': 4.0}}}


def test_planner_rectangle_rows_match_the_coverage_geometry_contract():
    segments = planned_scan_segments(_task(), 0.5)
    assert len(segments) == 5
    assert segments[0].start == (1.0, 1.25)
    assert segments[0].end == (3.0, 1.25)
    assert segments[1].start == (3.0, 1.625)
    assert segments[-1].end == (3.0, 2.75)


def test_exposure_targets_use_ceil_count_and_not_a_terminal_camera_pose():
    task = _task(lower_left=[1.0, 1.0], upper_right=[2.0, 1.5])
    segments = (planned_scan_segments(task, 0.5)[0],)
    exposures = planned_exposures(segments, 0.5, 0.0, 0.25)
    assert [(item.trigger_index, item.center) for item in exposures] == [
        (0, (1.25, 1.25)), (1, (1.75, 1.25))]


def test_preflight_reserves_the_planner_maneuver_envelope_at_a_hard_boundary():
    task = _task(
        lower_left=[0.15, 0.15], upper_right=[3.85, 3.85],
        sweep_direction='vertical', maneuver_boundary_margin_m=0.10,
        maneuver_drift_direction=[0.0, -1.0])
    truth = {'diagnostic_wall': {'features': []}}
    report = preflight_diagnostic_coverage(
        task, truth, _camera(), _robot(), _wall(), 0.01)
    route = report['task']['maneuver_safe_route_rectangle_m']
    safety_margin = 0.5 * (0.2 ** 2 + 0.2 ** 2) ** 0.5
    assert route == pytest.approx([
        0.15, safety_margin + 0.10, 3.85, 4.0 - safety_margin - 0.10])
    assert report['task']['exposure_count'] < len(
        planned_exposures(
            planned_scan_segments(task, 0.5), 0.5, 0.20, 0.25))


def test_preflight_measures_declared_feature_samples_with_discrete_photos():
    truth = {'diagnostic_wall': {'features': [{
        'id': 'patch', 'kind': 'repair_patch',
        'polygon_m': [[1.40, 1.40], [1.60, 1.40], [1.60, 1.60], [1.40, 1.60]],
    }]}}
    report = preflight_diagnostic_coverage(_task(), truth, _camera(), _robot(), _wall(), 0.01)
    coverage = report['feature_coverage']
    assert coverage['all_intersecting_feature_samples_covered']
    assert coverage['features'][0]['sample_count'] > 0
    assert coverage['features'][0]['uncovered_sample_count'] == 0


def test_preflight_rejects_a_task_outside_the_green_safe_region():
    with pytest.raises(DiagnosticCoveragePreflightError, match='green wall-safe'):
        preflight_diagnostic_coverage(
            _task(lower_left=[0.0, 0.0]), {'diagnostic_wall': {'features': []}},
            _camera(), _robot(), _wall())


def test_complementary_tasks_use_the_union_of_their_discrete_exposures():
    truth = {'diagnostic_wall': {'features': [{
        'id': 'patch', 'kind': 'repair_patch',
        'polygon_m': [[1.40, 1.40], [1.60, 1.40], [1.60, 1.60], [1.40, 1.60]],
    }]}}
    report = preflight_diagnostic_coverage_set(
        (_task(), _task(task_id='vertical', sweep_direction='vertical')),
        truth, _camera(), _robot(), _wall(), 0.01)
    assert report['task_set']['exposure_count'] > 0
    assert report['feature_coverage']['all_intersecting_feature_samples_covered']
