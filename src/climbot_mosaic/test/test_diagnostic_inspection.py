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
    _register_feature_id,
    _safe_feature_id,
    DiagnosticInspectionError,
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


def test_decal_mask_respects_rotation_at_native_pixel_centres():
    grid = MosaicGrid(0.0, 0.0, 0.004, 0.004, 0.001, 4, 4)
    mask = _feature_mask({
        'kind': 'graffiti_decal', 'center_m': [0.002, 0.002], 'size_m': [0.002, 0.002],
        'angle_deg': 45.0,
    }, grid, (0.0, 0.0, 0.004, 0.004))
    assert mask.shape == (4, 4)
    assert 0 < int(mask.sum()) < 16
