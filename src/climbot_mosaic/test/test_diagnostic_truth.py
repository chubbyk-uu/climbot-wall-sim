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

"""Deterministic tests for P2.7 diagnostic visual-truth measurements."""

import math

from climbot_mosaic.diagnostic_truth import (
    _SeamGradientAccumulator,
    _summarize_variant,
    _tile_segments,
    DiagnosticTruthError,
    estimate_translation,
    fit_similarity,
    MosaicGrid,
)
import cv2
import numpy as np
import pytest
import tifffile


def test_phase_translation_reports_observed_shift_in_reference_pixels():
    reference = np.zeros((128, 128), np.uint8)
    cv2.circle(reference, (47, 73), 13, 255, 2)
    cv2.line(reference, (31, 23), (89, 105), 180, 3)
    transform = np.float32(((1.0, 0.0, 7.0), (0.0, 1.0, -5.0)))
    observed = cv2.warpAffine(reference, transform, (128, 128))
    shift_x, shift_y, response = estimate_translation(reference, observed)
    assert (shift_x, shift_y) == pytest.approx((7.0, -5.0), abs=0.25)
    assert response > 0.8


def test_similarity_reports_metric_scale_yaw_translation_and_local_residuals():
    expected = np.asarray(((1.0, 1.0), (3.0, 2.0), (2.0, 5.0), (5.0, 4.0)), np.float64)
    scale = 1.0008
    yaw = math.radians(0.35)
    rotation = scale * np.asarray(((math.cos(yaw), -math.sin(yaw)),
                                   (math.sin(yaw), math.cos(yaw))))
    translation = np.asarray((0.012, -0.017))
    observed = expected @ rotation.T + translation
    result = fit_similarity(expected, observed)
    assert result['scale_error_ppm'] == pytest.approx(800.0, abs=0.05)
    assert result['yaw_error_deg'] == pytest.approx(0.35, abs=1e-4)
    assert result['translation_m'] == pytest.approx(translation, abs=1e-6)
    assert result['inlier_count'] == 4
    assert max(result['residuals_m']) < 1e-6


def test_diagnostic_measurements_reject_incompatible_images_and_anchor_sets():
    with pytest.raises(DiagnosticTruthError, match='same-size mono'):
        estimate_translation(np.zeros((32, 32), np.uint8), np.zeros((31, 32), np.uint8))
    with pytest.raises(DiagnosticTruthError, match='at least two'):
        fit_similarity(np.zeros((1, 2)), np.zeros((1, 2)))


def test_two_anchors_cannot_claim_a_local_deformation_measurement():
    matches = [
        {
            'id': 'one', 'phase_response': 0.5,
            'expected_center_m': [1.0, 1.0], 'observed_center_m': [1.01, 1.0],
            'offset_norm_m': 0.01,
        },
        {
            'id': 'two', 'phase_response': 0.6,
            'expected_center_m': [3.0, 2.0], 'observed_center_m': [3.01, 2.0],
            'offset_norm_m': 0.01,
        },
    ]
    summary = _summarize_variant(matches, {'one', 'two'})
    assert summary['similarity']['local_residual_observable'] is False
    assert summary['similarity']['local_residual_p95'] is None


def test_seam_gradient_uses_same_raster_off_seam_baseline():
    grid = MosaicGrid(0.0, 0.0, 0.005, 0.005, 0.001, 5, 5)
    accumulator = _SeamGradientAccumulator(
        np.arange(5, dtype=np.uint32), np.full(5, 1, np.uint32),
        np.zeros(5, np.uint8), grid, 8)
    reference = np.zeros((5, 5), np.uint8)
    observed = reference.copy()
    observed[:, 2:] = 10
    accumulator.add_tile(0, 0, observed, reference, np.ones((5, 5), np.uint16),
                         None, None, None, None, None, None)
    summary = accumulator.summary()
    gradient = summary['gradient_excess_gray_per_pixel']
    assert summary['seam_adjacency_count'] == 5
    assert gradient['on_hard_cut']['excess_over_truth']['p95'] == 10.0
    assert gradient['off_hard_cut_baseline']['excess_over_truth']['count'] == 35
    assert gradient['off_hard_cut_baseline']['excess_over_truth']['p95'] == 0.0
    assert gradient['on_to_off_excess_p95_ratio'] is None


def _tiled_master(path, width, height, tile_size):
    tifffile.imwrite(path, np.zeros((height, width), np.uint8),
                     tile=(tile_size, tile_size), compression='deflate')
    return MosaicGrid(0.0, 0.0, width * 0.001, height * 0.001, 0.001, width, height)


def test_tile_stream_accepts_the_raster_order_the_seam_accumulator_assumes(tmp_path):
    path = tmp_path / 'master.tif'
    grid = _tiled_master(path, 40, 24, 16)
    positions = [(row, column) for row, column, _ in _tile_segments(path, grid, 16)]
    assert positions == [(0, 0), (0, 16), (0, 32), (16, 0), (16, 16), (16, 32)]


def test_tile_stream_rejects_a_tile_size_the_manifest_does_not_declare(tmp_path):
    path = tmp_path / 'master.tif'
    grid = _tiled_master(path, 40, 24, 16)
    with pytest.raises(DiagnosticTruthError, match='tile size'):
        list(_tile_segments(path, grid, 32))


def test_tile_stream_crops_the_padding_off_the_last_row_and_column(tmp_path):
    path = tmp_path / 'master.tif'
    grid = _tiled_master(path, 40, 24, 16)
    shapes = {(row, column): tile.shape for row, column, tile in _tile_segments(path, grid, 16)}
    assert shapes[(0, 0)] == (16, 16)
    assert shapes[(0, 32)] == (16, 8)
    assert shapes[(16, 32)] == (8, 8)
