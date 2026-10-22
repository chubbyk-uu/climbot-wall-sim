# Copyright 2026 jerry
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Full-wall geometry contracts: cropped UVs and immutable cached same-source assets."""
from pathlib import Path

from climbot_gazebo.surface_mesh import write_feature_mesh
from climbot_gazebo.wall_assets import prepare
from climbot_gazebo.wall_surface import WallSurface
import numpy as np
import pytest
import yaml


def config():
    return {'roughness': {'amplitude_m': .002, 'scale_range_m': [.2, .8], 'seed': 20261019},
            'grooves': [],
            'channel_model': {'drape_m': .001, 'transition_m': .0001, 'refinement': 1}}


def test_gutter_uv_and_cropped_mesh_retain_global_source_coordinates(tmp_path):
    source = WallSurface(config())
    path = tmp_path/'visual.obj'
    result = write_feature_mesh(path, source, bounds=(2, 2.1, 3, 3.1),
                                uv_bounds=(1.99, 2.11, 2.99, 3.11))
    lines = path.read_text().splitlines()
    vertices = np.array([[float(x) for x in line.split()[1:]]
                         for line in lines if line.startswith('v ')])
    uv = np.array([[float(x) for x in line.split()[1:]]
                   for line in lines if line.startswith('vt ')])
    world = vertices[:, :2]+[2.05, 3.05]
    np.testing.assert_allclose(vertices[:, 2], source.height(world[:, 0], world[:, 1]), atol=1e-14)
    np.testing.assert_allclose(uv[:, 0], (world[:, 0]-1.99)/.12, atol=1e-13)
    # Match the baked image's global top-down pixel coordinates AFTER the
    # actual OBJLoader's 1-V conversion, not just the writer's formula.
    image_row_fraction = 1-uv[:, 1]
    np.testing.assert_allclose(image_row_fraction, (3.11-world[:, 1])/.12, atol=1e-13)
    assert np.all((uv > 0) & (uv < 1))  # Gutters are sampled, never overlapping geometry.
    assert result['sampled_height_error_max_m'] <= .00002


def test_cache_reuse_and_corruption_are_explicit(tmp_path, monkeypatch):
    monkeypatch.setenv('CLIMBOT_DATA_ROOT', str(tmp_path))
    directory, first = prepare(config(), [.08, .08])
    reused, second = prepare(config(), [.08, .08])
    assert reused == directory and first == second
    assert len(first['proxy']) == 1 and len(first['visuals']) == 1
    assert first['driver']['count'] == 1025
    assert first['proxy'][0]['centroid_error_m'] <= .00001
    Path(directory/'driver.png').write_bytes(b'broken')
    with pytest.raises(ValueError, match='checksum'):
        prepare(config(), [.08, .08])


def test_interrupted_build_does_not_block_the_next_launch(tmp_path, monkeypatch):
    from climbot_gazebo import wall_assets
    monkeypatch.setenv('CLIMBOT_DATA_ROOT', str(tmp_path))

    def interrupted(*args, **kwargs):
        raise KeyboardInterrupt

    monkeypatch.setattr(wall_assets, 'write_feature_mesh', interrupted)
    with pytest.raises(KeyboardInterrupt):
        prepare(config(), [.08, .08])
    monkeypatch.undo()
    monkeypatch.setenv('CLIMBOT_DATA_ROOT', str(tmp_path))
    directory, result = prepare(config(), [.08, .08])
    assert (directory/'manifest.json').is_file() and len(result['visuals']) == 1
    assert prepare(config(), [.08, .08])[0] == directory


def test_wall_tiles_share_one_static_link_per_role(tmp_path, monkeypatch):
    from xml.etree import ElementTree as ET

    from climbot_description.geometry import quaternion_from_rpy, rotate_vector
    from climbot_gazebo.wall_assets import apply
    monkeypatch.setenv('CLIMBOT_DATA_ROOT', str(tmp_path))
    size = [1.2, .3]
    directory, manifest = prepare(config(), size)
    world = ET.fromstring('<world/>')
    plugin = ET.fromstring('<plugin><sample><attachment>.1 .1 .02</attachment></sample>'
                           '<sample><attachment>.3 .2 .02</attachment></sample></plugin>')
    pose = [.4, -1., 2., 0., -1.2, .7]
    apply(world, plugin, directory, manifest, pose, size, config(), wall_mu=.7)
    assert all(float(e.text) == .7 for tag in ('mu', 'mu2')
               for e in world.findall('.//collision/surface/friction/ode/' + tag))
    # Separate static models each become a DART skeleton updated every step.
    assert sorted(m.get('name') for m in world.findall('model')) == [
        'm2_caster_proxy', 'm2_driver_wall', 'm2_optical_wall']
    q = quaternion_from_rpy(*pose[3:])
    caster = world.find('model[@name="m2_caster_proxy"]')
    np.testing.assert_allclose([float(x) for x in caster.findtext('pose').split()], pose)
    assert len(caster.findall('link/collision')) == 1
    # The merged caster mesh is the tiles shifted into wall coordinates, unchanged.
    merged = [line.split() for line in (directory/'caster.obj').read_text().splitlines()]
    expected, faces = [], 0
    for tile in manifest['proxy']:
        x0, x1, y0, y1 = tile['bounds_m']
        rows = [line.split() for line in (directory/tile['file']).read_text().splitlines()]
        expected += [[float(r[1])+(x0+x1)/2, float(r[2])+(y0+y1)/2, float(r[3])]
                     for r in rows if r and r[0] == 'v']
        faces += sum(1 for r in rows if r and r[0] == 'f')
    np.testing.assert_array_equal(
        [[float(x) for x in r[1:]] for r in merged if r and r[0] == 'v'], expected)
    assert sum(1 for r in merged if r and r[0] == 'f') == faces == \
        manifest['caster_mesh']['triangles']
    for name, tag, tiles in [('m2_optical_wall', 'visual', manifest['visuals'])]:
        model = world.find(f'model[@name="{name}"]')
        np.testing.assert_allclose([float(x) for x in model.findtext('pose').split()], pose)
        elements = model.findall(f'link/{tag}')
        assert len(elements) == len(tiles) == 2
        for element, tile in zip(elements, tiles):
            x0, x1, y0, y1 = tile['bounds_m']
            local = [float(x) for x in element.findtext('pose').split()]
            # Same world placement as the former one-model-per-tile layout.
            np.testing.assert_allclose(
                np.array(pose[:3])+rotate_vector(q, local[:3]),
                np.array(pose[:3])+rotate_vector(q, [(x0+x1)/2, (y0+y1)/2, 0]))
            assert local[3:] == [0, 0, 0]


def test_clipped_full_wall_joint_has_no_zero_area_optical_slivers(tmp_path):
    cfg = yaml.safe_load((Path(__file__).parents[1]/'config/m2_wall_surface.yaml').read_text())
    result = write_feature_mesh(tmp_path/'joint.obj', WallSurface(cfg),
                                bounds=(6.656, 7.168, 6.464, 6.976))
    assert result['min_projected_double_area_m2'] > 0
    assert result['sampled_height_error_max_m'] <= .00002


def test_m2_diagnostic_paint_matches_native_groove_openings_and_keeps_details():
    import sys
    sys.path.insert(0, str(Path(__file__).parents[3]/'tools'))
    import create_diagnostic_wall as diagnostic
    from PIL import Image
    cfg = yaml.safe_load((Path(__file__).parents[1]/'config/m2_wall_surface.yaml').read_text())
    source = WallSurface(cfg)
    features = diagnostic.surface_feature_layout([0, 0], [10, 8], 20260827, source)
    old_details = [f for f in diagnostic.feature_layout([0, 0], [10, 8], 20260827)
                   if f['kind'] != 'construction_seam']
    assert features[:len(old_details)] == old_details
    grooves = [f for f in features if f['kind'] == 'surface_groove']
    assert [f['geometry'] for f in grooves] == cfg['grooves']
    assert not any(f['kind'] == 'construction_seam' for f in features)
    # The eight M1 crack decals stay visual-only; they have no physical groove.
    assert sum(f['kind'] == 'crack_decal' for f in features) == 8
    # Pixel-centre mask around the real branching crack, not a copied atlas crack.
    x, y, size, scale = 16700, 22900, 600, .00025
    overlay = Image.new('RGBA', (size, size))
    for feature in grooves:
        diagnostic._draw_surface_groove(overlay, feature, [0, 0], [10, 8], scale, x, y)
    u, v = np.meshgrid((x+np.arange(size)+.5)*scale,
                       8-(y+np.arange(size)+.5)*scale)
    expected = np.rint(230*(1-np.exp(-source.groove_depth(u, v)/.0005))) > 0
    np.testing.assert_array_equal(np.asarray(overlay)[:, :, 3] > 0, expected)
    assert expected.any() and (~expected).any()
    # Adjacent DDS gutters see identical global pixels, including the crack tip.
    base = Image.new('RGB', (size, size), (120, 120, 120))
    left = diagnostic.render_sample(base, grooves, [0, 0], [10, 8], scale, x, y)
    right = diagnostic.render_sample(base, grooves, [0, 0], [10, 8], scale, x+300, y)
    np.testing.assert_array_equal(np.asarray(left)[:, 300:], np.asarray(right)[:, :300])


def test_native_surface_change_invalidates_the_cache(tmp_path, monkeypatch):
    """The C++ surface computes the meshes and the skirt query; a C++ fix must rebuild."""
    from types import SimpleNamespace

    from climbot_gazebo import wall_assets
    monkeypatch.setenv('CLIMBOT_DATA_ROOT', str(tmp_path/'data'))
    before, _ = prepare(config(), [.08, .08])
    rebuilt = tmp_path/'_climbot_surface.so'
    rebuilt.write_bytes(b'a rebuilt native surface')
    monkeypatch.setattr(wall_assets, '_climbot_surface', SimpleNamespace(__file__=str(rebuilt)))
    after, _ = prepare(config(), [.08, .08])
    assert after != before
