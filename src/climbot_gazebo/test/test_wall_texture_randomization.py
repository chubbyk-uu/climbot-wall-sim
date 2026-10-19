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

"""D4 source identity, explicit diversity exhaustion and reproducible RGB replay."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
from PIL import Image
import pytest

TOOLS = Path(__file__).resolve().parents[3] / 'tools'
sys.path.insert(0, str(TOOLS))
import wall_texture_randomization as randomizer  # noqa: E402


@pytest.mark.parametrize('orientation', range(8))
def test_d4_inverse_maps_each_pixel_to_its_original_source(orientation):
    original = np.arange(7 * 7).reshape(7, 7)
    x, y = np.meshgrid(np.arange(7), np.arange(7))
    sx, sy = randomizer.source_coordinates(x, y, 7, orientation)
    np.testing.assert_array_equal(randomizer.orient(
        original, orientation), original[sy, sx])
    assert sorted(randomizer.orient(
        original, orientation).ravel()) == list(range(49))


def make_layout(source, seed=73, width=64):
    return randomizer.layout(source, 64, width, 16, 4, seed, 12, .1, 1,
                             .001, .05, .25, lambda _: None, 16)


def test_all_orientations_share_one_source_reuse_guard():
    source = np.random.default_rng(4).uniform(
        0, 255, (128, 128)).astype(np.float32)
    placements, _, _ = make_layout(source)
    for i, first in enumerate(placements):
        for second in placements[:i]:
            a = (first.left, first.top, first.left + 16, first.top + 16)
            b = (second.left, second.top, second.left + 16, second.top + 16)
            if randomizer.rectangle_distance(a, b) * .001 <= .05:
                assert randomizer.source_overlap(
                    first.sx, first.sy, np.array([second.sx]),
                    np.array([second.sy]), 16)[0] <= .25
    assert len({p.orientation for p in placements}) > 1


def test_local_selection_protects_neighbours_without_reserving_future_or_banning_far_reuse():
    from audit_wall_randomization import source_rectangle_pairs

    source = np.random.default_rng(4).uniform(
        0, 255, (128, 128)).astype(np.float32)
    args = (source, 128, 256, 16, 4, 73, 32, .1,
            1, .001, .04, 0, lambda _: None)
    a, _, stats = randomizer.layout(*args, catalog_mode='local')
    b, _, other = randomizer.layout(*args, catalog_mode='local')
    assert stats == other and stats['algorithm'] == randomizer.LOCAL_ALGORITHM
    assert not stats['future_sources_reserved']
    assert stats['source_schedule'] is None
    assert len(a) > len({(p.sx, p.sy) for p in a})
    manifest = {'scale_m_per_px': .001, 'quilt': {
        'patch_px': 16, 'overlap_px': 4}}
    assert source_rectangle_pairs(manifest, a, .04) == []
    assert [(p.sx, p.sy, p.orientation) for p in a] == [
        (p.sx, p.sy, p.orientation) for p in b]


def test_packed_catalogue_does_not_treat_rotations_as_new_source_capacity():
    origins = randomizer.source_catalog((10000, 10000), 1280)
    assert len(origins) > 80
    for i, (sy, sx) in enumerate(origins):
        assert np.all(randomizer.source_overlap(sx, sy, origins[:i, 1],
                                                origins[:i, 0], 1280) <= .25)
    with pytest.raises(ValueError, match='source diversity exhausted'):
        randomizer.layout(np.ones((16, 16), np.float32), 16, 32, 16, 4,
                          1, 8, .1, 1, .001, .05, .25, lambda _: None,
                          catalog_mode='packed')


def test_scheduled_reuse_preserves_metric_exclusion_for_a_large_wall():
    catalogue = randomizer.source_catalog((10000, 10000), 1200)
    rule = randomizer.source_schedule(len(catalogue), 1200, 300, 2.1 / .00025)
    nx, ny, shift = rule['period_columns'], rule['period_rows'], rule['row_period_column_shift']
    assert rule['colours'] <= len(catalogue)
    for m in range(-4, 5):
        for n in range(-4, 5):
            if not (m or n):
                continue
            x, y = (m * nx + n * shift) * 900, n * ny * 900
            assert randomizer.rectangle_distance((0, 0, 1200, 1200),
                                                 (x, y, x + 1200, y + 1200)) * .00025 > 2.1
    with pytest.raises(ValueError, match='cannot supply'):
        randomizer.source_schedule(1, 1200, 300, 8400)


def test_disjoint_plan_is_deterministic_breaks_the_period_and_protects_all_contributors():
    options = ((128, 128), 160, 208, 16, 2, 40, 20261018)
    catalog, assignment, shape, stats = randomizer.disjoint_plan(*options)
    second = randomizer.disjoint_plan(*options)
    np.testing.assert_array_equal(assignment, second[1])
    assert stats == second[3]
    assert stats['positions_changed_from_periodic_initialization'] > len(
        assignment) / 2
    columns = (shape[1] - 2) // 14
    for i, colour in enumerate(assignment):
        row, column = divmod(i, columns)
        for j, previous in enumerate(assignment[:i]):
            other_row, other_column = divmod(j, columns)
            a = (column * 14, row * 14, column * 14 + 16, row * 14 + 16)
            b = (other_column * 14, other_row * 14,
                 other_column * 14 + 16, other_row * 14 + 16)
            if randomizer.rectangle_distance(a, b) <= 40:
                sy, sx = catalog[colour]
                py, px = catalog[previous]
                # Zero full-source intersection proves that even feather fragments
                # cannot reproduce the 23.4375% strip from the v3 failure.
                assert randomizer.source_overlap(sx, sy, np.array([px]),
                                                 np.array([py]), 16)[0] == 0


def test_disjoint_capacity_failure_never_relaxes_the_metric_guard():
    with pytest.raises(ValueError, match='cannot supply'):
        randomizer.disjoint_plan((32, 32), 160, 208, 16, 2, 40, 1)
    with pytest.raises(ValueError, match='explicit max_source_overlap=0'):
        randomizer.layout(np.ones((128, 128), np.float32), 64, 64, 16, 2,
                          1, 8, .1, 1, .001, .04, .25, lambda _: None,
                          catalog_mode='disjoint')


def test_full_source_audit_catches_the_strip_that_a_pure_core_audit_misses():
    from audit_wall_randomization import opaque_core_pairs, source_rectangle_pairs

    placements = [randomizer.Placement(0, 0, 0, 0, 0, np.array([], np.uint8)),
                  randomizer.Placement(0, 256, 80, 48, 7, np.array([], np.uint8))]
    manifest = {'scale_m_per_px': .00025, 'quilt': {
        'patch_px': 128, 'overlap_px': 32}}
    assert opaque_core_pairs(manifest, placements, 2.1) == []
    result = source_rectangle_pairs(manifest, placements, 2.1)
    assert len(result) == 1
    assert result[0]['source_rectangle_area_m2'] == pytest.approx(
        128 ** 2 * .234375 * .00025 ** 2)


def test_disjoint_quilt_exposes_its_actual_distinct_candidates_and_replays_identically():
    from audit_wall_randomization import source_rectangle_pairs

    source = np.random.default_rng(4).integers(0, 256, (128, 128, 3), np.uint8)
    grey = source.astype(np.float32).mean(axis=2)
    arguments = (grey, 64, 96, 16, 2, 1, 96, .1,
                 1, .001, .04, 0, lambda _: None)
    a, shape, stats = randomizer.layout(*arguments, catalog_mode='disjoint')
    b, _, second_stats = randomizer.layout(*arguments, catalog_mode='disjoint')
    assert stats == second_stats
    assert stats['algorithm'] == randomizer.DISJOINT_ALGORITHM
    assert set(stats['eligible_candidates_per_placement']) == {96}
    manifest = {'scale_m_per_px': .001, 'quilt': {
        'patch_px': 16, 'overlap_px': 2}}
    assert source_rectangle_pairs(manifest, a, .04) == []
    assert stats['source_crops_used'] == len({(p.sy, p.sx) for p in a})
    np.testing.assert_array_equal(
        randomizer.replay(source, a, shape, 16, 2, 1, 64, 96),
        randomizer.replay(source, b, shape, 16, 2, 1, 64, 96))


def test_final_disjoint_proof_rejects_a_corrupted_future_source_assignment():
    with pytest.raises(RuntimeError, match='violates metric exclusion'):
        randomizer.verify_disjoint_colouring(np.array([0, 0]), np.array([0, 14]),
                                             np.array([0, 0]), 16, 40)


def test_exhaustion_does_not_silently_reuse_a_rotated_source():
    with pytest.raises(ValueError, match='source diversity exhausted'):
        make_layout(np.ones((16, 16), dtype=np.float32))


def test_replay_and_trace_are_reproducible_and_expose_source_centres(tmp_path):
    source = np.random.default_rng(4).integers(
        0, 256, (128, 128, 3), dtype=np.uint8)
    grey = source.astype(np.float32).mean(axis=2)
    first, shape, stats = make_layout(grey)
    second, second_shape, second_stats = make_layout(grey)
    assert shape == second_shape and stats == second_stats
    a = randomizer.replay(source, first, shape, 16, 4, 1, 64, 64)
    b = randomizer.replay(source, second, shape, 16, 4, 1, 64, 64)
    np.testing.assert_array_equal(a, b)
    hashes = []
    for number, placements in enumerate((first, second)):
        directory = tmp_path / str(number)
        directory.mkdir()
        trace = randomizer.write_trace(directory, placements, 16, 64, 128)
        hashes.append(trace)
        for item in trace:
            assert hashlib.sha256(
                (directory / item['file']).read_bytes()).hexdigest() == item['sha256']
        document = json.loads((directory / 'quilt_sources.json').read_text())
        p = document['placements'][0]
        assert p['resampled_to_native_scale'] == .5
        assert p['source_center_native_px'][0] == (first[0].sx + 8) * .5 - .5
    assert hashes[0] == hashes[1]


def test_cli_refuses_existing_output_before_opening_source(tmp_path):
    output = tmp_path / 'old-wall'
    output.mkdir()
    sentinel = output / 'keep'
    sentinel.write_bytes(b'immutable old asset')
    result = subprocess.run([
        sys.executable, str(TOOLS / 'bake_wall_texture.py'),
        '--source-dir', str(tmp_path / 'absent'), '--source-size-m', '2.5',
        '--output-dir', str(output), '--randomize'], capture_output=True, text=True)
    assert result.returncode != 0 and 'must not already exist' in result.stderr
    assert sentinel.read_bytes() == b'immutable old asset'


def test_publication_does_not_replace_a_target_created_during_the_bake(tmp_path):
    from bake_wall_texture import publish_new_directory

    source = tmp_path / 'working'
    source.mkdir()
    (source / 'asset').write_bytes(b'new asset')
    destination = tmp_path / 'concurrently-created'
    destination.mkdir()
    with pytest.raises(FileExistsError):
        publish_new_directory(source, destination)
    assert (source / 'asset').read_bytes() == b'new asset'
    assert not list(destination.iterdir())


def test_cli_publishes_a_complete_new_traceable_asset(tmp_path):
    source = tmp_path / 'source'
    source.mkdir()
    Image.fromarray(np.random.default_rng(1).integers(
        0, 256, (128, 128, 3), dtype=np.uint8)).save(source / 'fixture_Color.jpg')
    output = tmp_path / 'new-wall'
    result = subprocess.run([
        sys.executable, str(TOOLS / 'bake_wall_texture.py'),
        '--source-dir', str(source), '--source-size-m', '.032',
        '--output-dir', str(output), '--randomize', '--region-m', '.016', '.016',
        '--patch-px', '16', '--seam-feather-px', '1', '--block-px', '32',
        '--gutter-px', '4', '--reuse-radius-m', '.02', '--candidates', '8',
        '--jobs', '1', '--quiet'], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    manifest = json.loads((output / 'wall_texture.json').read_text())
    assert manifest['randomization']['algorithm'] == randomizer.ALGORITHM
    assert manifest['source_native_dimensions_px'] == [128, 128]
    assert len(manifest['quilt']['trace_files']) == 2
    for entry in manifest['maps']['albedo']['blocks']:
        assert hashlib.sha256(
            (output / entry['file']).read_bytes()).hexdigest() == entry['sha256']
    assert not list(tmp_path.glob('.wall-bake-*'))


@pytest.mark.parametrize('orientation', range(8))
def test_final_pixel_copy_audit_recognizes_rotated_and_mirrored_content(orientation):
    from audit_wall_randomization import ncc_windows

    source = np.random.default_rng(9).integers(
        0, 256, (128, 128), dtype=np.uint8)
    image = np.concatenate(
        (source, randomizer.orient(source, orientation)), axis=1)
    placements = [randomizer.Placement(0, left, 0, 0, k, np.array([], np.uint8))
                  for left, k in ((0, 0), (128, orientation))]
    result = ncc_windows({'scale_m_per_px': .001, 'quilt': {'patch_px': 128}},
                         placements, image, 100, .5, 7)
    assert result['copied_window_fraction'] > .1


def test_retained_trace_tracks_the_weights_of_all_four_overlap_contributors():
    from audit_wall_randomization import retained_weights

    source = np.random.default_rng(4).uniform(
        0, 255, (128, 128)).astype(np.float32)
    placements, _, _ = make_layout(source)
    manifest = {'width_px': 64, 'height_px': 64, 'quilt': {
        'patch_px': 16, 'overlap_px': 4, 'seam_feather_px': 1}}
    xs, ys, owners, weights = retained_weights(manifest, placements, 1)
    np.testing.assert_allclose(weights.sum(axis=2), 1, atol=1e-6)
    expected = np.zeros((64, 64), np.float32)
    for item in placements:
        y1, x1 = min(64, item.top + 16), min(64, item.left + 16)
        alpha = randomizer.alpha_mask(item, 16, 4, 1)[
            :y1 - item.top, :x1 - item.left]
        block = randomizer.orient(source[item.sy:item.sy + 16, item.sx:item.sx + 16],
                                  item.orientation)[:y1 - item.top, :x1 - item.left]
        window = expected[item.top:y1, item.left:x1]
        window[:] = window * (1 - alpha) + block * alpha
    restored = np.zeros(expected.shape, np.float32)
    for i, item in enumerate(placements):
        yy, xx, slot = np.where(owners == i)
        sx, sy = randomizer.source_coordinates(xs[xx] - item.left, ys[yy] - item.top,
                                               16, item.orientation)
        np.add.at(restored, (yy, xx),
                  source[sy + item.sy, sx + item.sx] * weights[yy, xx, slot])
    np.testing.assert_allclose(restored, expected, atol=3e-5)


def test_visual_filled_joints_preserve_global_coordinates_and_other_diagnostic_kinds():
    import create_diagnostic_wall as diagnostic

    features = diagnostic.feature_layout([0, 0], [10, 8], 20260827, 'filled')
    kinds = {p['kind'] for p in features}
    assert {'construction_seam', 'crack_decal',
            'repair_patch', 'graffiti_decal'} <= kinds
    joints = [p for p in features if p['kind'] == 'construction_seam']
    assert all(p['joint_profile']['physical_depth_m'] == 0 for p in joints)
    # A compact crop through the first vertical joint at wall x=3.4 m.
    base = Image.new('RGB', (500, 1000), (128, 128, 128))
    left = diagnostic.render_sample(
        base, joints, [0, 0], [10, 8], .001, 3100, 1000)
    right = diagnostic.render_sample(
        base, joints, [0, 0], [10, 8], .001, 3350, 1000)
    np.testing.assert_array_equal(np.asarray(
        left)[:, 250:], np.asarray(right)[:, :250])
    assert not np.all(np.asarray(left) == 128)


def test_repair_polygon_clipping_preserves_the_first_pixel_of_shared_gutter():
    import create_diagnostic_wall as diagnostic

    feature = next(p for p in diagnostic.feature_layout([0, 0], [10, 8], 20260827)
                   if p['id'] == 'repair_patch_06')
    base = Image.new('RGB', (500, 400), (128, 128, 128))
    left = diagnostic.render_sample(base, [feature], [0, 0], [10, 8], .00025, 30400, 4500)
    right = diagnostic.render_sample(base, [feature], [0, 0], [10, 8], .00025, 30592, 4500)
    np.testing.assert_array_equal(np.asarray(left)[:, 192:], np.asarray(right)[:, :308])


def test_diagnostic_bc1_preserves_shared_gutters_beside_unpainted_tiles(tmp_path):
    from audit_wall_randomization import gutter_continuity
    from bc1 import write_dds_bc1
    from create_diagnostic_wall import _preserve_unpainted_bc1

    pixels = np.random.default_rng(90).integers(40, 190, (256, 1024, 3), dtype=np.uint8)
    for crosses_gutter in (False, True):
        blocks = []
        for column, left in enumerate((0, 384)):
            base = tmp_path / ('base%d.dds' % column)
            output = tmp_path / ('paint%d.dds' % column)
            write_dds_bc1(base, pixels[:, left:left + 640])
            with Image.open(base) as image:
                original = np.asarray(image.convert('RGB'))
            painted = original.copy()
            x = np.arange(left, left + 640)
            mask = ((x >= 500) & (x < 520) if crosses_gutter else
                    (x >= 50) & (x < 70))
            painted[70:190, mask] = [70, 70, 70]
            if mask.any():
                write_dds_bc1(output, painted)
                _preserve_unpainted_bc1(base, output, original, painted)
            else:
                output.write_bytes(base.read_bytes())
            blocks.append({'file': output.name, 'row': 0, 'column': column,
                           'sample_x_px': left, 'sample_y_px': 0,
                           'sample_width_px': 640, 'sample_height_px': 256})
        audit = gutter_continuity({'maps': {'albedo': {'blocks': blocks}}}, tmp_path)
        assert audit['comparisons'] == 6
        assert audit['mismatched_shared_regions'] == 0


def test_false_match_attribution_recognizes_d4_and_discards_overwritten_sources():
    from attribute_wall_texture_matches import point_sources

    mask = np.packbits(np.ones((16, 16), dtype=bool))
    placements = [randomizer.Placement(0, 0, 0, 0, 0, mask),
                  randomizer.Placement(0, 16, 0, 0, 2, mask)]
    manifest = {'scale_m_per_px': .001, 'region_origin_m': [0, 0],
                'region_m': [.032, .016], 'quilt': {
                    'patch_px': 16, 'overlap_px': 4, 'seam_feather_px': 0}}
    sources = point_sources(manifest, placements, [[.005, .010], [.022, .006]])
    for contributions in sources:
        visible = [p for p in contributions if p[2] > .25]
        np.testing.assert_allclose(visible, [(5, 6, 1)], atol=1e-12)
    placements.append(randomizer.Placement(0, 0, 20, 20, 0, mask))
    source = point_sources(manifest, placements, [[.005, .010]])[0]
    np.testing.assert_allclose([p for p in source if p[2] > .25], [
                               (25, 26, 1)], atol=1e-12)


def test_diagnostic_derivative_rejects_a_tampered_hashed_base_block(tmp_path):
    import create_diagnostic_wall as diagnostic

    block = tmp_path / 'albedo.dds'
    expected = hashlib.sha256(b'original block').hexdigest()
    block.write_bytes(b'changed block')
    manifest = {'scale_m_per_px': .00025, 'region_origin_m': [0, 0], 'region_m': [1, 1],
                'maps': {'albedo': {'blocks': [
                    {'file': block.name, 'sample_x_px': 0, 'sample_y_px': 0,
                     'sha256': expected}]}}}
    path = tmp_path / 'wall_texture.json'
    path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='base DDS checksum mismatch'):
        diagnostic._validate_base_manifest(str(path))
