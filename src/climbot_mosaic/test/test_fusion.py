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

"""Small deterministic geometry tests for tiled mosaic fusion."""

import os

import climbot_mosaic.fusion as fusion
from climbot_mosaic.fusion import (
    _process_tree_pss_bytes,
    _tile_count,
    _tile_seam_adjacencies,
    _tiles_from_raw,
    _write_raw_tile,
    build_wall_mosaic,
    common_grid,
    encode_uncertainty,
    feather_map,
    FUSION_BASE_MEMORY_GB,
    FUSION_WORKER_MEMORY_MB,
    FusionError,
    hard_cut_ownership,
    interior_distance_map,
    RenderFrame,
    RenderGrid,
    resolve_jobs,
)
from climbot_mosaic.mosaic_inputs import FrameKey
import cv2
import numpy as np
import pytest


def _frame(index, bbox):
    return RenderFrame(FrameKey('run', index), f'{index}.png',
                       (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
                       bbox, (0.0, 0.0), (0.001, 0.001, 0.001))


def test_common_grid_covers_both_pose_variants_exactly():
    grid = common_grid((_frame(0, (0.0, 0.0, 1.0, 1.0)),),
                       (_frame(0, (-0.1, 0.2, 1.1, 1.2)),), 0.1, 16)
    assert (grid.min_x_m, grid.min_y_m) == pytest.approx((-0.1, 0.0))
    assert grid.max_x_m >= 1.1 and grid.max_y_m >= 1.2
    assert (grid.width_px, grid.height_px) == (12, 12)


def test_feather_is_symmetric_and_has_full_weight_center():
    values = feather_map(20, 10, 0.2)
    np.testing.assert_array_equal(values, np.flipud(values))
    np.testing.assert_array_equal(values, np.fliplr(values))
    assert values[5, 10] == pytest.approx(1.0)
    assert 0.0 < values[0, 0] < 1.0


def test_hard_cut_prefers_interior_source_and_keeps_stable_ties():
    distance = interior_distance_map(7, 5)
    np.testing.assert_array_equal(distance, np.flipud(distance))
    np.testing.assert_array_equal(distance, np.fliplr(distance))
    owner = np.asarray(((3.0, 4.0), (2.0, 5.0)), np.float32)
    candidate = np.asarray(((4.0, 4.0), (1.0, 6.0)), np.float32)
    np.testing.assert_array_equal(
        hard_cut_ownership(owner, candidate),
        ((True, False), (False, True)))
    with pytest.raises(FusionError, match='share a shape'):
        hard_cut_ownership(owner, candidate[:, :1])


def test_hard_cut_seams_include_internal_and_tiled_neighbours():
    grid = RenderGrid(0.0, 0.0, 0.004, 0.004, 0.001, 4, 4, 2)
    owner = np.asarray(((1, 2), (3, 3)), np.uint16)
    rows, columns, axes = _tile_seam_adjacencies(
        owner, 0, 2, grid, np.asarray((1, 2), np.uint16), None)
    assert set(zip(rows.tolist(), columns.tolist(), axes.tolist())) == {
        (1, 1, 0),  # boundary from the preceding tile
        (0, 2, 0),  # internal horizontal owner transition
        (0, 2, 1),  # internal vertical owner transition
        (0, 3, 1),
    }


def test_worker_image_cache_is_byte_bounded(tmp_path, monkeypatch):
    first = tmp_path / 'first.png'
    second = tmp_path / 'second.png'
    assert cv2.imwrite(str(first), np.full((4, 4), 10, np.uint8))
    assert cv2.imwrite(str(second), np.full((4, 4), 20, np.uint8))
    monkeypatch.setattr(fusion, '_WORKER_IMAGE_CACHE_BYTES', 16)
    grid = RenderGrid(0.0, 0.0, 0.004, 0.004, 0.001, 4, 4, 16)
    fusion._init_worker((), (), grid, 4, 4)
    np.testing.assert_array_equal(fusion._image(str(first)), np.full((4, 4), 10, np.uint8))
    np.testing.assert_array_equal(fusion._image(str(second)), np.full((4, 4), 20, np.uint8))
    assert list(fusion._WORKER_IMAGES) == [str(second)]
    assert fusion._WORKER_IMAGE_CACHE_USED_BYTES == 16


def test_resource_and_grid_contracts_reject_invalid_values():
    with pytest.raises(FusionError, match='resolution'):
        common_grid((_frame(0, (0.0, 0.0, 1.0, 1.0)),), (), 0.0)
    # A budget that cannot cover the run's fixed cost plus one worker falls
    # back to a single worker and says so. It does not refuse: the fixed cost
    # was fitted on the largest mosaic here, so a small job would be turned
    # away for memory it never uses. Nor does it pretend to have obeyed the
    # budget, which is what returning 1 in silence amounted to.
    for budget in (0.1, FUSION_BASE_MEMORY_GB):
        with pytest.warns(RuntimeWarning, match='soft budget'):
            assert resolve_jobs(None, budget) == 1
    # Beyond that the budget pays per worker, and the machine bounds the rest.
    processors = os.cpu_count() or 1
    assert resolve_jobs(64, 1024.0) == processors
    assert resolve_jobs(2, 1024.0) == 2
    generous = FUSION_BASE_MEMORY_GB + 10 * FUSION_WORKER_MEMORY_MB / 1024.0
    assert resolve_jobs(None, generous) == min(10, processors)


def test_preview_size_contract_rejects_too_small_panels(tmp_path):
    with pytest.raises(FusionError, match='preview_max_side_px'):
        build_wall_mosaic(
            tmp_path / 'result', tmp_path / 'work', None, None,
            0.001, 1, 1.0, preview_max_side_px=511)


def test_uncertainty_encoding_has_precision_range_and_explicit_nodata():
    encoded = encode_uncertainty(np.asarray((0.0, 0.001, 1.0, np.nan), np.float32))
    np.testing.assert_array_equal(encoded, (0, 100, 65534, 65535))


def test_process_tree_memory_sampler_observes_current_process():
    assert _process_tree_pss_bytes(os.getpid()) > 0


def test_raw_auxiliary_tile_cache_preserves_tile_order(tmp_path):
    grid = RenderGrid(0.0, 0.0, 0.02, 0.017, 0.001, 20, 17, 16)
    path = tmp_path / 'tiles.dat'
    descriptor = os.open(path, os.O_RDWR | os.O_CREAT | os.O_TRUNC, 0o600)
    try:
        tile_bytes = grid.tile_size_px ** 2 * np.dtype(np.uint16).itemsize
        os.ftruncate(descriptor, _tile_count(grid) * tile_bytes)
        for index, (row, column) in enumerate(((0, 0), (0, 16), (16, 0), (16, 16))):
            _write_raw_tile(
                descriptor, np.full((16, 16), index + 1, np.uint16), grid, row, column)
    finally:
        os.close(descriptor)
    tiles = tuple(_tiles_from_raw(path, np.dtype(np.uint16), grid))
    assert [int(tile[0, 0]) for tile in tiles] == [1, 2, 3, 4]
