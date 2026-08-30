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

from climbot_mosaic.diagnostic_inspection import (
    _coverage_summary,
    _feature_mask,
    _intersection,
    _pad_to_shape,
    _polygon_mask,
    _register_feature_id,
    _safe_feature_id,
    _union_mask,
    DiagnosticInspectionError,
    observable_envelope,
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


def test_observable_envelope_leans_past_the_region_by_the_forward_offset():
    """The footprint is carried ahead of the centre, so it clears the boundary."""
    horizontal, vertical = observable_envelope(
        _rect(0.55, 0.55, 9.45, 7.45), 0.50, 0.28125, 0.340)
    # 0.340 + 0.28125 / 2 = 0.480625 along track, 0.50 / 2 across it.
    assert _flat(horizontal) == pytest.approx(_flat(_rect(0.069375, 0.30, 9.930625, 7.70)))
    assert _flat(vertical) == pytest.approx(_flat(_rect(0.30, 0.069375, 9.70, 7.930625)))


def test_observable_envelope_rejects_a_footprint_that_cannot_photograph_anything():
    region = _rect(0.55, 0.55, 9.45, 7.45)
    for footprint in ((0.0, 0.28125, 0.340), (0.50, -1.0, 0.340),
                      (0.50, 0.28125, float('nan'))):
        with pytest.raises(DiagnosticInspectionError, match='finite and positive'):
            observable_envelope(region, *footprint)


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


def test_envelope_split_separates_a_missed_pixel_from_an_unphotographable_one():
    """Outside the region is an excuse only where no permitted pose could look."""
    grid = MosaicGrid(0.0, 0.0, 0.004, 0.004, 0.001, 4, 4)
    coverage = np.zeros((4, 4), np.uint16)
    bounds = (0.0, 0.0, 0.004, 0.004)
    region = _rect(0.001, 0.001, 0.003, 0.003)
    envelope = observable_envelope(region, 0.0005, 0.0005, 0.00025)
    result = _coverage_summary(
        coverage, grid, bounds, None, _polygon_mask(grid, bounds, region),
        _union_mask(grid, bounds, envelope))
    assert result['uncovered_outside_inspection_region'] == 12
    # Only the four corners lie beyond both sweep axes; the rest of the ring is
    # reachable by a footprint leaning out of the region, so calling all twelve
    # unreachable would have overstated the excuse threefold.
    assert result['uncovered_outside_region_inside_envelope'] == 8
    assert result['uncovered_outside_region_outside_envelope'] == 4


def test_envelope_split_is_absent_without_a_footprint():
    grid = MosaicGrid(0.0, 0.0, 0.004, 0.004, 0.001, 4, 4)
    coverage = np.zeros((4, 4), np.uint16)
    bounds = (0.0, 0.0, 0.004, 0.004)
    result = _coverage_summary(
        coverage, grid, bounds, None,
        _polygon_mask(grid, bounds, _rect(0.001, 0.001, 0.003, 0.003)))
    assert 'uncovered_outside_region_inside_envelope' not in result


def test_decal_mask_respects_rotation_at_native_pixel_centres():
    grid = MosaicGrid(0.0, 0.0, 0.004, 0.004, 0.001, 4, 4)
    mask = _feature_mask({
        'kind': 'graffiti_decal', 'center_m': [0.002, 0.002], 'size_m': [0.002, 0.002],
        'angle_deg': 45.0,
    }, grid, (0.0, 0.0, 0.004, 0.004))
    assert mask.shape == (4, 4)
    assert 0 < int(mask.sum()) < 16
