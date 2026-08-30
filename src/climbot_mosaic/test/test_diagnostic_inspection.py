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

"""Small, deterministic contracts for native diagnostic inspection output."""

import json

from climbot_mosaic.diagnostic_inspection import (
    _coverage_summary,
    _feature_mask,
    _intersection,
    _pad_to_shape,
    _polygon_json,
    _polygon_mask,
    _register_feature_id,
    _safe_feature_id,
    _union_mask,
    DiagnosticInspectionError,
    motion_region_camera_envelopes,
    planned_scan_footprints,
)
from climbot_mosaic.diagnostic_truth import MosaicGrid
import numpy as np
import pytest


def test_intersection_keeps_only_positive_area():
    assert _intersection((0.0, 0.0, 2.0, 2.0), (1.0, 1.0, 3.0, 3.0)) == (1.0, 1.0, 2.0, 2.0)
    assert _intersection((0.0, 0.0, 1.0, 1.0), (1.0, 0.0, 2.0, 1.0)) is None


def test_padding_preserves_pixels_without_resampling():
    image = np.asarray(((3, 4), (5, 6)), np.uint8)
    padded = _pad_to_shape(image, 3, 4)
    np.testing.assert_array_equal(padded[:2, :2], image)
    assert padded[2, 3] == 0


def test_feature_id_is_one_portable_directory_component():
    assert _safe_feature_id('crack_decal_01') == 'crack_decal_01'
    for value in ('', '.', '..', '../escape', '/tmp/escape', 'two/parts', 'white space'):
        with pytest.raises(DiagnosticInspectionError, match='safe file name'):
            _safe_feature_id(value)


def test_feature_ids_must_be_unique():
    existing = set()
    assert _register_feature_id({'id': 'repair_patch_01'}, existing) == 'repair_patch_01'
    with pytest.raises(DiagnosticInspectionError, match='duplicated'):
        _register_feature_id({'id': 'repair_patch_01'}, existing)


def test_coverage_summary_counts_each_native_pixel():
    grid = MosaicGrid(0.0, 0.0, 0.004, 0.004, 0.001, 4, 4)
    coverage = np.asarray(((0, 1, 2, 3), (1, 1, 2, 2),
                           (0, 0, 1, 1), (3, 2, 2, 1)), np.uint16)
    result = _coverage_summary(coverage, grid, (0.0, 0.0, 0.004, 0.004))
    assert result == {
        'pixel_count': 16,
        'uncovered_pixel_count': 3,
        'single_source_pixel_count': 6,
        'overlap_pixel_count': 7,
        'maximum_source_count': 3,
    }


def _flat(polygon):
    """Flatten a polygon so a tolerance can be applied to its coordinates."""
    return [coordinate for point in sorted(polygon) for coordinate in point]


def _rect(min_x, min_y, max_x, max_y):
    """Return a rectangle as the counter-clockwise polygon a frozen task publishes."""
    return ((min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y))


def test_polygon_mask_selects_pixel_centres_inside_the_region():
    grid = MosaicGrid(0.0, 0.0, 0.004, 0.004, 0.001, 4, 4)
    # Row 0 is the top of the raster, so y descends down the mask.
    mask = _polygon_mask(grid, (0.0, 0.0, 0.004, 0.004), _rect(0.001, 0.001, 0.003, 0.003))
    assert mask.tolist() == [
        [False, False, False, False],
        [False, True, True, False],
        [False, True, True, False],
        [False, False, False, False],
    ]


def test_polygon_mask_follows_a_region_a_bounding_box_would_widen():
    """Trapezoid tasks exist, and their bounding box accepts ground they exclude."""
    grid = MosaicGrid(0.0, 0.0, 0.004, 0.004, 0.001, 4, 4)
    triangle = ((0.0, 0.0), (0.004, 0.0), (0.0, 0.004))
    mask = _polygon_mask(grid, (0.0, 0.0, 0.004, 0.004), triangle)
    assert mask.tolist() == [
        [True, False, False, False],
        [True, True, False, False],
        [True, True, True, False],
        [True, True, True, True],
    ]


def test_a_region_wound_either_way_selects_the_same_pixels():
    grid = MosaicGrid(0.0, 0.0, 0.004, 0.004, 0.001, 4, 4)
    bounds = (0.0, 0.0, 0.004, 0.004)
    region = _rect(0.001, 0.001, 0.003, 0.003)
    np.testing.assert_array_equal(
        _polygon_mask(grid, bounds, region),
        _polygon_mask(grid, bounds, tuple(reversed(region))))


def test_a_region_without_area_is_refused_rather_than_masking_nothing():
    grid = MosaicGrid(0.0, 0.0, 0.004, 0.004, 0.001, 4, 4)
    with pytest.raises(DiagnosticInspectionError, match='no area'):
        _polygon_mask(grid, (0.0, 0.0, 0.004, 0.004),
                      ((0.001, 0.001), (0.003, 0.001), (0.002, 0.001)))


def test_coverage_split_reports_what_lay_inside_the_inspection_region():
    """A gap outside the inspection region is not a coverage failure."""
    grid = MosaicGrid(0.0, 0.0, 0.004, 0.004, 0.001, 4, 4)
    # Uncovered pixels sit in the outer ring only, which is what a feature
    # running past the region looks like.
    coverage = np.asarray(((0, 0, 0, 0), (0, 2, 2, 0),
                           (0, 2, 2, 0), (0, 0, 0, 0)), np.uint16)
    bounds = (0.0, 0.0, 0.004, 0.004)
    region = _rect(0.001, 0.001, 0.003, 0.003)
    result = _coverage_summary(
        coverage, grid, bounds, None, _polygon_mask(grid, bounds, region))
    assert result['uncovered_pixel_count'] == 12
    assert result['uncovered_inside_inspection_region'] == 0
    assert result['uncovered_outside_inspection_region'] == 12
    assert result['feature_pixels_inside_inspection_region'] == 4
    # And a gap inside the region is still counted against the run.
    coverage[1, 1] = 0
    inside = _coverage_summary(
        coverage, grid, bounds, None, _polygon_mask(grid, bounds, region))
    assert inside['uncovered_inside_inspection_region'] == 1


def test_coverage_split_is_absent_without_an_inspection_region():
    grid = MosaicGrid(0.0, 0.0, 0.004, 0.004, 0.001, 4, 4)
    coverage = np.zeros((4, 4), np.uint16)
    result = _coverage_summary(coverage, grid, (0.0, 0.0, 0.004, 0.004))
    assert 'uncovered_inside_inspection_region' not in result


def test_the_region_split_respects_the_feature_mask(tmp_path):
    """Pixels outside the declared geometry belong to neither side."""
    grid = MosaicGrid(0.0, 0.0, 0.004, 0.004, 0.001, 4, 4)
    coverage = np.zeros((4, 4), np.uint16)
    bounds = (0.0, 0.0, 0.004, 0.004)
    mask = np.zeros((4, 4), bool)
    mask[1, 1] = True
    result = _coverage_summary(
        coverage, grid, bounds, mask,
        _polygon_mask(grid, bounds, _rect(0.001, 0.001, 0.003, 0.003)))
    assert result['pixel_count'] == 1
    assert result['uncovered_inside_inspection_region'] == 1
    assert result['uncovered_outside_inspection_region'] == 0


def _task(sweep=1, offset=0.340):
    return {
        'motion_region_m': _rect(0.55, 0.55, 9.45, 7.45),
        'camera_footprint_m': (0.50, 0.28125, offset),
        'sweep_direction': sweep,
        'waypoints_m': ((0.8, 1.0), (9.2, 1.0)),
        'segment_types': (1,),
    }


def test_planned_scan_footprint_uses_the_directed_route_and_camera_offset():
    polygon, = planned_scan_footprints((_task(),))
    assert _flat(polygon) == pytest.approx(_flat(_rect(0.999375, 0.75, 9.680625, 1.25)))


def test_safe_pose_envelope_uses_motion_region_and_the_task_sweep_axis():
    negative, positive = motion_region_camera_envelopes((_task(),))
    assert _flat(negative) == pytest.approx(_flat(_rect(0.069375, 0.30, 9.250625, 7.70)))
    assert _flat(positive) == pytest.approx(_flat(_rect(0.749375, 0.30, 9.930625, 7.70)))


def test_camera_footprint_rejects_invalid_dimensions_but_accepts_zero_offset():
    for footprint in ((0.0, 0.28125, 0.340), (0.50, -1.0, 0.340)):
        task = _task()
        task['camera_footprint_m'] = footprint
        with pytest.raises(DiagnosticInspectionError, match='finite and positive'):
            motion_region_camera_envelopes((task,))
    task = _task(offset=float('nan'))
    with pytest.raises(DiagnosticInspectionError, match='offset.*finite'):
        motion_region_camera_envelopes((task,))
    assert len(motion_region_camera_envelopes((_task(offset=0.0),))) == 2


def test_negative_camera_offset_is_applied_behind_the_directed_scan():
    task = _task(offset=-0.340)
    polygon, = planned_scan_footprints((task,))
    assert _flat(polygon) == pytest.approx(_flat(_rect(0.319375, 0.75, 9.000625, 1.25)))


def test_union_mask_takes_every_polygon():
    grid = MosaicGrid(0.0, 0.0, 0.004, 0.004, 0.001, 4, 4)
    bounds = (0.0, 0.0, 0.004, 0.004)
    mask = _union_mask(grid, bounds, (_rect(0.0, 0.0, 0.001, 0.001),
                                      _rect(0.003, 0.003, 0.004, 0.004)))
    assert mask.tolist() == [
        [False, False, False, True],
        [False, False, False, False],
        [False, False, False, False],
        [True, False, False, False],
    ]


def test_reach_splits_distinguish_the_frozen_plan_from_any_safe_pose():
    grid = MosaicGrid(0.0, 0.0, 0.004, 0.004, 0.001, 4, 4)
    coverage = np.zeros((4, 4), np.uint16)
    bounds = (0.0, 0.0, 0.004, 0.004)
    region = _rect(0.001, 0.001, 0.003, 0.003)
    planned = _union_mask(grid, bounds, (_rect(0.001, 0.001, 0.003, 0.003),))
    safe = _union_mask(grid, bounds, (_rect(0.0, 0.001, 0.004, 0.003),))
    result = _coverage_summary(
        coverage, grid, bounds, None, _polygon_mask(grid, bounds, region),
        planned, safe)
    assert result['uncovered_outside_inspection_region'] == 12
    assert result['uncovered_outside_region_inside_planned_scan_footprint'] == 0
    assert result['uncovered_outside_region_outside_planned_scan_footprint'] == 12
    assert result['uncovered_outside_region_inside_safe_pose_envelope'] == 4
    assert result['uncovered_outside_region_outside_safe_pose_envelope'] == 8


def test_the_summary_renders_regions_as_plain_json_numbers():
    """
    The envelope became a tuple of polygons and the summary still wrote rectangles.

    Nothing caught it until a full run had spent nine minutes writing tiles and
    then failed on the last line, because the tests reached the geometry and
    the coverage split but never the summary that has to survive json.dumps.
    """
    region = _rect(0.55, 0.55, 9.45, 7.45)
    rendered = [_polygon_json(polygon)
                for polygon in motion_region_camera_envelopes((_task(),))]
    assert json.loads(json.dumps(rendered)) == rendered
    assert _polygon_json(region)[0] == [0.55, 0.55]


def test_reach_splits_are_absent_without_task_geometry():
    grid = MosaicGrid(0.0, 0.0, 0.004, 0.004, 0.001, 4, 4)
    coverage = np.zeros((4, 4), np.uint16)
    bounds = (0.0, 0.0, 0.004, 0.004)
    result = _coverage_summary(
        coverage, grid, bounds, None,
        _polygon_mask(grid, bounds, _rect(0.001, 0.001, 0.003, 0.003)))
    assert 'uncovered_outside_region_inside_planned_scan_footprint' not in result
    assert 'uncovered_outside_region_inside_safe_pose_envelope' not in result


def test_decal_mask_respects_rotation_at_native_pixel_centres():
    grid = MosaicGrid(0.0, 0.0, 0.004, 0.004, 0.001, 4, 4)
    mask = _feature_mask({
        'kind': 'graffiti_decal', 'center_m': [0.002, 0.002], 'size_m': [0.002, 0.002],
        'angle_deg': 45.0,
    }, grid, (0.0, 0.0, 0.004, 0.004))
    assert mask.shape == (4, 4)
    assert 0 < int(mask.sum()) < 16
