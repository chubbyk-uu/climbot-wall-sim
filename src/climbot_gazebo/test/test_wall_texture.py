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

"""Check that baked texture blocks land where the bake says they do."""

import json
import os
import sys
import tempfile
from xml.dom import minidom

TOOLS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..',
                                     'tools'))
sys.path.insert(0, TOOLS)
import bake_wall_texture  # noqa: E402
from climbot_gazebo.wall_texture import (  # noqa: E402
    block_extent, load_manifest, sampled_block_extent, texture_visuals)
import create_diagnostic_wall  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402
import pytest  # noqa: E402

SCALE = 0.001
BLOCK = 100


def write_bake(directory, columns=3, rows=2, maps=('albedo',), gutter=0):
    """Write a small manifest and the block files it promises."""
    width, height = columns * BLOCK, rows * BLOCK
    entries = {}
    for name in maps:
        blocks = []
        for row in range(rows):
            for column in range(columns):
                filename = '%s_r%02d_c%02d.dds' % (name, row, column)
                open(os.path.join(directory, filename), 'wb').close()
                entry = {
                    'file': filename, 'row': row, 'column': column,
                    'x_px': column * BLOCK, 'y_px': row * BLOCK,
                    'width_px': BLOCK, 'height_px': BLOCK}
                if gutter:
                    left = max(0, column * BLOCK - gutter)
                    top = max(0, row * BLOCK - gutter)
                    right = min(width, (column + 1) * BLOCK + gutter)
                    bottom = min(height, (row + 1) * BLOCK + gutter)
                    entry.update(
                        sample_x_px=left, sample_y_px=top,
                        sample_width_px=right - left,
                        sample_height_px=bottom - top)
                blocks.append(entry)
        entries[name] = {'blocks': blocks}
    manifest = {
        'scale_m_per_px': SCALE,
        'region_m': [width * SCALE, height * SCALE],
        'region_origin_m': [2.0, 1.0],
        'width_px': width, 'height_px': height,
        'maps': entries,
    }
    if gutter:
        manifest['gutter_px'] = gutter
    path = os.path.join(directory, 'wall_texture.json')
    with open(path, 'w', encoding='utf-8') as handle:
        json.dump(manifest, handle)
    return path


def test_blocks_tile_the_region_without_gap_or_overlap():
    """The blocks together must cover exactly the region the bake claims."""
    with tempfile.TemporaryDirectory() as directory:
        manifest, _ = load_manifest(write_bake(directory))
        origin_x, origin_y = manifest['region_origin_m']
        width, height = manifest['region_m']
        covered = []
        for block in manifest['maps']['albedo']['blocks']:
            x, y, w, h = block_extent(manifest, block)
            covered.append((x - w / 2.0, y - h / 2.0, x + w / 2.0, y + h / 2.0))
        assert min(entry[0] for entry in covered) == pytest.approx(origin_x)
        assert min(entry[1] for entry in covered) == pytest.approx(origin_y)
        assert max(entry[2] for entry in covered) == pytest.approx(
            origin_x + width)
        assert max(entry[3] for entry in covered) == pytest.approx(
            origin_y + height)
        total = sum((entry[2] - entry[0]) * (entry[3] - entry[1])
                    for entry in covered)
        assert total == pytest.approx(width * height)


def test_sampled_block_extents_overlap_with_identical_wall_coordinates():
    """A gutter enlarges visuals without changing nominal wall ownership."""
    with tempfile.TemporaryDirectory() as directory:
        manifest, _ = load_manifest(write_bake(directory, gutter=20))
        blocks = {(entry['row'], entry['column']): entry
                  for entry in manifest['maps']['albedo']['blocks']}
        left = sampled_block_extent(manifest, blocks[(0, 0)])
        right = sampled_block_extent(manifest, blocks[(0, 1)])
        left_edges = (left[0] - left[2] / 2.0, left[0] + left[2] / 2.0)
        right_edges = (right[0] - right[2] / 2.0,
                       right[0] + right[2] / 2.0)
        assert left_edges == pytest.approx((2.0, 2.12))
        assert right_edges == pytest.approx((2.08, 2.22))
        assert left_edges[1] - right_edges[0] == pytest.approx(0.04)


def test_bake_copies_real_neighbours_into_each_render_block(monkeypatch):
    """Both sides of a visual boundary must store the same source pixels."""
    image = np.arange(8 * 12 * 3, dtype=np.uint8).reshape(8, 12, 3)
    encoded = {}

    def remember(path, rgb):
        encoded[os.path.basename(path)] = rgb.copy()
        return rgb.size

    monkeypatch.setattr(bake_wall_texture, 'write_dds_bc1', remember)
    with tempfile.TemporaryDirectory() as directory:
        _, _, blocks = bake_wall_texture.write_blocks(
            image, directory, 'albedo', block=8, gutter=4,
            log=lambda _: None)

    assert blocks[0]['sample_x_px'] == 0
    assert blocks[0]['sample_width_px'] == 12
    assert blocks[1]['sample_x_px'] == 4
    assert blocks[1]['sample_width_px'] == 8
    assert np.array_equal(encoded['albedo_r00_c00.dds'][:, 4:12],
                          encoded['albedo_r00_c01.dds'])
    assert bake_wall_texture.stored_texels(12, 8, 8, 4) == 160
    assert (bake_wall_texture.DEFAULT_RENDER_GUTTER_PX %
            (bake_wall_texture.BLOCK_ALIGNMENT * 2 ** 5)) == 0


@pytest.mark.skipif(os.name != 'posix', reason='parallel encoder uses POSIX fork')
def test_parallel_block_encoder_keeps_row_major_metadata():
    """Workers may finish out of order, but the manifest order must not change."""
    image = np.arange(16 * 16 * 3, dtype=np.uint8).reshape(16, 16, 3)
    with tempfile.TemporaryDirectory() as directory:
        rows, columns, blocks = bake_wall_texture.write_blocks(
            image, directory, 'albedo', block=8, gutter=0,
            log=lambda _: None, jobs=2)
        assert (rows, columns) == (2, 2)
        assert [(block['row'], block['column']) for block in blocks] == [
            (0, 0), (0, 1), (1, 0), (1, 1)]
        assert all(os.path.getsize(os.path.join(directory, block['file'])) > 128
                   for block in blocks)


def test_minimum_cut_feather_has_no_binary_render_edge():
    """The selected cut stays put but its rendered transition becomes soft."""
    mask = np.zeros((32, 32), dtype=bool)
    mask[:, 16:] = True
    alpha = bake_wall_texture.feather_mask(mask, 4.0)
    assert alpha[:, 0].max() == 0.0
    assert alpha[:, -1].min() == 1.0
    assert np.any((alpha > 0.0) & (alpha < 1.0))


def test_diagnostic_layout_is_seeded_sparse_and_recorded_in_wall_metres():
    """P2.7a details must be reproducible evidence, not untracked scenery."""
    first = create_diagnostic_wall.feature_layout([0.0, 0.0], [10.0, 8.0],
                                                  20260827)
    second = create_diagnostic_wall.feature_layout([0.0, 0.0], [10.0, 8.0],
                                                   20260827)
    assert first == second
    kinds = [feature['kind'] for feature in first]
    assert kinds.count('construction_seam') == 3
    assert kinds.count('crack_decal') == 8
    assert kinds.count('repair_patch') == 6
    assert kinds.count('graffiti_decal') == 4
    # The smallest centre separation rules out a dense calibration grid.
    centres = [feature['center_m'] for feature in first
               if feature['kind'] in ('repair_patch', 'graffiti_decal')]
    distances = [np.linalg.norm(np.subtract(left, right))
                 for index, left in enumerate(centres)
                 for right in centres[index + 1:]]
    assert min(distances) > 0.35
    seams = [feature for feature in first if feature['kind'] == 'construction_seam']
    for seam in seams:
        start, end = seam['points_m']
        assert start[0] == end[0] or start[1] == end[1]
        edge_tolerance = seam['width_m'] / 2.0 + 1e-12
        for point in (start, end):
            assert min(abs(point[0]), abs(point[0] - 10.0),
                       abs(point[1]), abs(point[1] - 8.0)) <= edge_tolerance
    cracks = [feature for feature in first if feature['kind'] == 'crack_decal']
    assert len({tuple(feature['atlas_box_fraction']) for feature in cracks}) == 8
    assert min(max(feature['size_m']) for feature in cracks) >= 0.78
    assert sum(max(feature['size_m']) >= 1.0 for feature in cracks) >= 6
    graffiti = [feature for feature in first
                if feature['kind'] == 'graffiti_decal']
    assert min(feature['opacity'] for feature in graffiti) >= 0.40
    assert min(max(feature['size_m']) for feature in graffiti) >= 0.50
    for feature in first:
        x0, y0, x1, y1 = create_diagnostic_wall._feature_bounds(feature)
        assert 0.0 <= x0 < x1 <= 10.0
        assert 0.0 <= y0 < y1 <= 8.0


def test_default_diagnostic_decal_atlas_is_transparent_and_complete():
    """The reproducible generator must ship its selected v2 source asset."""
    with Image.open(create_diagnostic_wall.DEFAULT_DECAL_ATLAS) as atlas:
        assert atlas.mode == 'RGBA'
        assert atlas.getchannel('A').getextrema() == (0, 255)
        for row in range(3):
            for column in range(4):
                cell = atlas.crop((column * atlas.width // 4,
                                   row * atlas.height // 3,
                                   (column + 1) * atlas.width // 4,
                                   (row + 1) * atlas.height // 3))
                assert cell.getchannel('A').getbbox() is not None


def test_diagnostic_feature_is_drawn_in_the_declared_world_location():
    """An overlap sample must place its paint mark from metres, not block row."""
    image = Image.new('RGB', (200, 200), (128, 128, 128))
    feature = {
        'id': 'paint_mark_01', 'kind': 'paint_mark',
        'center_m': [0.50, 1.50], 'radius_m': 0.08, 'angle_rad': 0.0,
        'rgba': [80, 40, 30, 255],
    }
    rendered = create_diagnostic_wall.render_sample(
        image, [feature], [0.0, 0.0], [2.0, 2.0], 0.01, 0, 0)
    # y is top-origin in texture pixels: wall y=1.5 maps to row 50.
    assert rendered.getpixel((50, 50)) != (128, 128, 128)
    assert rendered.getpixel((150, 150)) == (128, 128, 128)


def test_diagnostic_crack_decal_uses_alpha_and_metric_size():
    """A decal must composite at its declared wall position and remain local."""
    image = Image.new('RGB', (200, 200), (128, 128, 128))
    atlas = Image.new('RGBA', (20, 20), (20, 20, 20, 255))
    feature = {
        'id': 'crack_decal_01', 'kind': 'crack_decal',
        'center_m': [0.50, 1.50], 'size_m': [0.20, 0.20],
        'angle_deg': 0.0, 'opacity': 1.0,
        'atlas_box_fraction': [0.0, 0.0, 1.0, 1.0],
    }
    rendered = create_diagnostic_wall.render_sample(
        image, [feature], [0.0, 0.0], [2.0, 2.0], 0.01, 0, 0,
        atlas)
    assert rendered.getpixel((50, 50)) != (128, 128, 128)
    assert rendered.getpixel((80, 80)) == (128, 128, 128)


def test_first_block_is_the_top_of_the_wall_not_the_bottom():
    """Pixel rows run down while wall coordinates run up."""
    # Reading the row index as a height is the one mistake this conversion
    # invites, and it mirrors the reference a mosaic is scored against, which
    # then fails as a pose error rather than as a texture error.
    with tempfile.TemporaryDirectory() as directory:
        manifest, _ = load_manifest(write_bake(directory))
        blocks = {(entry['row'], entry['column']): entry
                  for entry in manifest['maps']['albedo']['blocks']}
        top = block_extent(manifest, blocks[(0, 0)])
        bottom = block_extent(manifest, blocks[(1, 0)])
        assert top[1] > bottom[1]
        origin_y, height = manifest['region_origin_m'][1], manifest['region_m'][1]
        assert top[1] + top[3] / 2.0 == pytest.approx(origin_y + height)


def test_missing_block_file_is_refused_rather_than_skipped():
    """A half-written bake must not render as a partly blank wall."""
    with tempfile.TemporaryDirectory() as directory:
        path = write_bake(directory)
        os.remove(os.path.join(directory, 'albedo_r00_c01.dds'))
        with pytest.raises(FileNotFoundError):
            load_manifest(path)


def test_visuals_are_flat_coplanar_and_carry_the_baked_map():
    """One plane at one depth, and the map actually attached."""
    with tempfile.TemporaryDirectory() as directory:
        manifest, resolved = load_manifest(write_bake(directory))
        visuals = texture_visuals(
            manifest, resolved, 0.10, (0.0, -5.0, 0.0), (0.0, 0.0, 4.0))
        assert len(visuals) == len(manifest['maps']['albedo']['blocks'])
        depths = {entry.split('<pose>')[1].split()[0] for entry in visuals}
        assert len(depths) == 1
        assert float(depths.pop()) > 0.05
        # The base colour multiplies the albedo map, and its default is dark
        # enough to render the textured blocks pure black while the untextured
        # wall beside them lights normally. Nothing in the load reports it, so
        # the only thing standing between that and a whole run of black
        # photographs is this assertion.
        assert all('<ambient>1 1 1 1</ambient>' in entry for entry in visuals)
        assert all('<diffuse>1 1 1 1</diffuse>' in entry for entry in visuals)
        assert all('<cast_shadows>false</cast_shadows>' in entry
                   for entry in visuals)
        assert all('<albedo_map>' in entry for entry in visuals)
        assert all(entry.count('.dds') == 1 for entry in visuals)
        # No normal or roughness map is baked, so none may be referenced: a tag
        # pointing at a file that was never written loads as a black surface.
        assert not any('<normal_map>' in entry for entry in visuals)
        assert not any('<roughness_map>' in entry for entry in visuals)


def test_a_block_lands_where_the_work_frame_says_it_does():
    """The one mapping that has already been got wrong."""
    # A visual's pose is relative to the link; a block's place is a work
    # coordinate. They differ by the work frame's origin in the world and by
    # where the wall body sits, and leaving either out shifts every block by a
    # constant. That is the failure with no symptom at load time: the first
    # version omitted the origin, which cost nothing while the origin was the
    # wall's centre and moved every block 5 m sideways the day it became the
    # corner. Half the wall rendered flat and nothing said so.
    #
    # The expected poses below are written as numbers rather than as the
    # formula, because a test that restates the implementation cannot fail
    # when the implementation is wrong.
    with tempfile.TemporaryDirectory() as directory:
        manifest, resolved = load_manifest(
            write_bake(directory, columns=3, rows=2))
        # 0.3 x 0.2 m of bake, its lower-left corner 1 m along and 2 m up an
        # 8 x 6 m wall that stands on the ground and is centred on world Y.
        manifest['region_origin_m'] = [1.0, 2.0]
        visuals = texture_visuals(
            manifest, resolved, 0.10, (0.0, -4.0, 0.0), (0.0, 0.0, 3.0))
        poses = {}
        for entry in visuals:
            name = entry.split('name="')[1].split('"')[0]
            values = entry.split('<pose>')[1].split('</pose>')[0].split()
            poses[name] = [float(value) for value in values[:3]]
        # Bottom-left block: work (1.05, 2.05) -> world (-2.95, 2.05).
        assert poses['wall_texture_r01_c00'][1] == pytest.approx(-2.95)
        assert poses['wall_texture_r01_c00'][2] == pytest.approx(-0.95)
        # Top-right block: work (1.25, 2.15) -> world (-2.75, 2.15).
        assert poses['wall_texture_r00_c02'][1] == pytest.approx(-2.75)
        assert poses['wall_texture_r00_c02'][2] == pytest.approx(-0.85)


def _mutated_bake(directory, mutate):
    """Write a valid bake, then break one thing about the manifest."""
    path = write_bake(directory)
    with open(path, encoding='utf-8') as handle:
        manifest = json.load(handle)
    mutate(manifest)
    with open(path, 'w', encoding='utf-8') as handle:
        json.dump(manifest, handle)
    return path


def test_a_manifest_that_does_not_describe_the_wall_is_refused():
    """The failures a bad manifest used to produce only in the rendered image."""
    # The work frame origin bug showed what silence costs here: every block
    # shifted half a wall sideways, half the surface rendered untextured, and
    # nothing in the load said a word. These are the same class of mistake,
    # made in the manifest rather than in the code.
    def drop_the_scale(manifest):
        del manifest['scale_m_per_px']

    def negative_scale(manifest):
        manifest['scale_m_per_px'] = -SCALE

    def infinite_scale(manifest):
        manifest['scale_m_per_px'] = float('inf')

    def block_off_the_region(manifest):
        manifest['maps']['albedo']['blocks'][0]['x_px'] = 100000

    def block_with_no_area(manifest):
        manifest['maps']['albedo']['blocks'][0]['width_px'] = 0

    def map_with_no_blocks(manifest):
        manifest['maps']['albedo']['blocks'] = []

    def overlapping_blocks(manifest):
        manifest['maps']['albedo']['blocks'][1]['x_px'] = 0

    def gap_between_blocks(manifest):
        manifest['maps']['albedo']['blocks'][0]['width_px'] -= 1

    def inconsistent_pixel_extent(manifest):
        manifest['width_px'] += 1

    for mutate, error in ((drop_the_scale, KeyError),
                          (negative_scale, ValueError),
                          (infinite_scale, ValueError),
                          (block_off_the_region, ValueError),
                          (block_with_no_area, ValueError),
                          (map_with_no_blocks, ValueError),
                          (overlapping_blocks, ValueError),
                          (gap_between_blocks, ValueError),
                          (inconsistent_pixel_extent, ValueError)):
        with tempfile.TemporaryDirectory() as directory:
            with pytest.raises(error):
                load_manifest(_mutated_bake(directory, mutate))


def test_a_manifest_region_must_fit_the_real_wall():
    """A self-consistent bake may still be mounted outside the configured wall."""
    with tempfile.TemporaryDirectory() as directory:
        path = _mutated_bake(
            directory,
            lambda manifest: manifest.update(region_origin_m=[9.8, 7.9]))
        with pytest.raises(ValueError, match='outside'):
            load_manifest(path, wall_size=(10.0, 8.0))

    with tempfile.TemporaryDirectory() as directory:
        manifest, _ = load_manifest(
            write_bake(directory), wall_size=(10.0, 8.0))
        assert manifest['region_origin_m'] == [2.0, 1.0]


def test_a_path_with_xml_in_it_still_produces_parseable_sdf():
    """The world file is parsed as XML, so the bake's own text has to be."""
    # Nothing about a checkout path is under this repository control. One with
    # an ampersand in it - somebody named a directory, or a Windows share seen
    # through WSL - would break minidom.parseString on the whole world file,
    # and the error would point at a line the bake wrote rather than at the
    # path that caused it.
    with tempfile.TemporaryDirectory() as parent:
        directory = os.path.join(parent, 'wall & mount <v2>')
        os.mkdir(directory)
        manifest, resolved = load_manifest(write_bake(directory))
        visuals = texture_visuals(
            manifest, resolved, 0.10, (0.0, -5.0, 0.0), (0.0, 0.0, 4.0))
        document = minidom.parseString('<link>' + '\n'.join(visuals) + '</link>')
        uris = document.getElementsByTagName('albedo_map')
        assert uris, 'no albedo map was emitted'
        assert '&' in uris[0].firstChild.data, (
            'the path was mangled rather than escaped')
