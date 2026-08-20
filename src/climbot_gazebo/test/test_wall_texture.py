"""Check that baked texture blocks land where the bake says they do."""

import json
import os
import tempfile

from climbot_gazebo.wall_texture import (
    block_extent, load_manifest, texture_visuals)

import pytest

SCALE = 0.001
BLOCK = 100


def write_bake(directory, columns=3, rows=2, maps=('albedo',)):
    """Write a small manifest and the block files it promises."""
    width, height = columns * BLOCK, rows * BLOCK
    entries = {}
    for name in maps:
        blocks = []
        for row in range(rows):
            for column in range(columns):
                filename = '%s_r%02d_c%02d.dds' % (name, row, column)
                open(os.path.join(directory, filename), 'wb').close()
                blocks.append({
                    'file': filename, 'row': row, 'column': column,
                    'x_px': column * BLOCK, 'y_px': row * BLOCK,
                    'width_px': BLOCK, 'height_px': BLOCK})
        entries[name] = {'blocks': blocks}
    manifest = {
        'scale_m_per_px': SCALE,
        'region_m': [width * SCALE, height * SCALE],
        'region_origin_m': [2.0, 1.0],
        'width_px': width, 'height_px': height,
        'maps': entries,
    }
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
