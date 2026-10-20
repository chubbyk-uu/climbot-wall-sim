# Copyright 2026 jerry
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Cached full-wall M2 assets, with separate same-source contact and optical meshes."""
import hashlib
import json
import os
from pathlib import Path
import tempfile
import time

import _climbot_surface
from climbot_description.geometry import quaternion_from_rpy, rotate_vector
from climbot_description.vacuum_preview import node, numbers
import numpy as np
from PIL import Image
import yaml

from .surface_mesh import write_feature_mesh
from .surface_patch import Envelope
from .wall_surface import WallSurface, write_tensor_mesh
from .wall_texture import load_manifest, sampled_block_extent


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def proxy_tile(path, surface, bounds):
    """Reuse T2's half-radius dilation, resolving joints locally, not wall-wide."""
    x0, x1, y0, y1 = bounds
    axes = []
    for axis, (lo, hi) in enumerate([(x0, x1), (y0, y1)]):
        values = np.linspace(lo, hi, round((hi-lo)/.01)+1)
        for g in surface.config['grooves']:
            if g['start_m'][axis] != g['end_m'][axis]:
                continue
            c, half = g['start_m'][axis], g['width_m']/2
            if not lo-half <= c <= hi+half:
                continue
            values = np.r_[values, c+np.linspace(-half-.0001, half+.0001, 53),
                           c+np.linspace(-.0002, .0002, 17), c-half, c, c+half]
        axes.append(np.unique(np.round(values[(values >= lo) & (values <= hi)], 12)))
    source_axes = []
    for a in axes:
        values = np.r_[np.arange(a[0]-.02, a[-1]+.0201, .001), a,
                       (2*a[:-1]+a[1:])/3, (a[:-1]+2*a[1:])/3]
        source_axes.append(np.unique(values))
    envelope = Envelope(surface, source_axes, .00625)
    result = write_tensor_mesh(path, envelope, *axes)
    u, v = np.meshgrid(*axes)
    height = envelope.height(u, v)
    errors = []
    for index, (a, b) in enumerate([(1/3, 2/3), (2/3, 1/3)]):
        x = a*u[:-1, :-1]+b*u[:-1, 1:]
        y = b*v[:-1, :-1]+a*v[1:, :-1]
        linear = ((height[:-1, :-1]+height[:-1, 1:]+height[1:, 1:])/3 if index == 0 else
                  (height[:-1, :-1]+height[1:, 1:]+height[1:, :-1])/3)
        errors.append(float(np.max(abs(envelope.height(x, y)-linear))))
    error = max(errors)
    if error > .00001:
        raise ValueError(f'full-wall caster facet error exceeds 10um: {error} at {bounds}')
    return {'bounds_m': list(bounds), 'triangles': result['triangles'],
            'centroid_error_m': error, 'file': path.name}


def prepare(config, size, texture_manifest=None):
    """Build persistent assets once; refuse incomplete/modified cached geometry."""
    width, height = size
    source = WallSurface(config)
    manifest, texture_directory = (load_manifest(texture_manifest, wall_size=size)
                                   if texture_manifest else (None, None))
    if manifest and manifest.get('diagnostic_wall', {}).get('joint_profile') == 'filled':
        raise ValueError('M2 open grooves require base v6 texture, not painted filled seams')
    details = manifest.get('diagnostic_wall', {}) if manifest else {}
    if details.get('joint_profile') == 'm2_open_source':
        expected = hashlib.sha256(source.resolved_yaml.encode()).hexdigest()
        if details.get('surface_source_sha256') != expected:
            raise ValueError('M2 diagnostic texture and physical wall source differ')
        if digest(Path(texture_directory)/details['surface_source']) != expected:
            raise ValueError('M2 diagnostic source checksum mismatch')
    identity = {'source': source.resolved_yaml, 'size_m': list(size),
                'texture_sha256': digest(texture_manifest) if texture_manifest else None,
                # The surface is computed by the native module the skirt query
                # also links; a C++-only fix must not reuse meshes from before it.
                'implementation': {p.name: digest(p) for p in [
                        Path(__file__), Path(__file__).with_name('surface_mesh.py'),
                        Path(__file__).with_name('surface_patch.py'),
                        Path(__file__).with_name('wall_surface.py'),
                        Path(_climbot_surface.__file__)]}}
    key = hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()
    data = Path(os.environ.get('CLIMBOT_DATA_ROOT', str(Path.home()/'climbot_data')))
    out = data/'assets/m2_wall'/key[:20]
    complete = out/'manifest.json'
    if complete.exists():
        result = json.loads(complete.read_text())
        if result['identity'] != identity or any(
                not (out/name).is_file() or digest(out/name) != sha
                for name, sha in result['files_sha256'].items()):
            raise ValueError('M2 cached asset checksum mismatch; rebuild explicitly')
        return out, result
    # Build beside the cache entry and publish it with one rename: a launch
    # interrupted mid-build must not leave a half-written entry blocking reuse.
    final = out
    final.parent.mkdir(parents=True, exist_ok=True)
    out = Path(tempfile.mkdtemp(prefix=final.name+'.partial-', dir=final.parent))
    start = time.monotonic()
    (out/'source.yaml').write_text(source.resolved_yaml)
    # Driver/body/stop collision: no grooves. Same vertical transform and
    # count/(count-1) scaling verified in T2 and 4W's full-area heightmap.
    count = 1025
    u, v = np.meshgrid(np.linspace(0, width, count), np.linspace(0, height, count))
    h = source.roughness(u, v)
    lo, hi = float(h.min()), float(h.max())
    if hi <= lo:
        raise ValueError('undulating full wall requires nonzero roughness')
    Image.fromarray(np.flipud(np.rint((h-lo)/(hi-lo)*65535).astype(np.uint16))).save(
        out/'driver.png')
    driver = {'count': count, 'low_m': lo, 'high_m': hi,
              'quantization_error_m': (hi-lo)/65535/2}
    proxy = []
    contact_source = source.contact_view('bridge_narrow_cracks')
    # One-metre collision tiles bound memory during the offline dilation.
    # Whole-wall 1mm rasterization would allocate >640MB before any envelope.
    rows = zip(np.arange(0, height, 1), np.minimum(np.arange(1, height+1, 1), height))
    for row, (y0, y1) in enumerate(rows):
        columns = zip(np.arange(0, width, 1), np.minimum(np.arange(1, width+1, 1), width))
        for col, (x0, x1) in enumerate(columns):
            proxy.append(proxy_tile(out/f'caster_{row}_{col}.obj', contact_source,
                                    (x0, x1, y0, y1)))
    # ODE's hash space compares every static geom's box each step; one merged
    # caster mesh (same triangles, OPCODE tree inside) removes that broadphase.
    merged = out/'caster.obj'
    with merged.open('x') as stream:
        stream.write('# M2 caster proxy: one-metre source tiles in wall coordinates\n')
        offset = 0
        for tile in proxy:
            x0, x1, y0, y1 = tile['bounds_m']
            count = 0
            for line in (out/tile['file']).read_text().splitlines():
                if line.startswith('v '):
                    x, y, z = map(float, line.split()[1:])
                    stream.write(f'v {x+(x0+x1)/2:.17g} {y+(y0+y1)/2:.17g} {z:.17g}\n')
                    count += 1
                elif line.startswith('vn '):
                    stream.write(line+'\n')
                elif line.startswith('f '):
                    stream.write('f '+' '.join(
                        f'{int(i.split("//")[0])+offset}//{int(i.split("//")[1])+offset}'
                        for i in line.split()[1:])+'\n')
            offset += count
    caster_mesh = {'file': merged.name, 'triangles': sum(p['triangles'] for p in proxy)}
    visuals = []
    blocks = manifest['maps']['albedo']['blocks'] if manifest else [
        {'row': r, 'column': c, 'bounds': (c, min(c+1, width), r, min(r+1, height))}
        for r in np.arange(0, height, 1) for c in np.arange(0, width, 1)]
    for block in blocks:
        if manifest:
            scale = manifest['scale_m_per_px']
            ox, oy = manifest['region_origin_m']
            x0 = ox+block['x_px']*scale
            x1 = x0+block['width_px']*scale
            y1 = oy+manifest['region_m'][1]-block['y_px']*scale
            y0 = y1-block['height_px']*scale
            cx, cy, w, h = sampled_block_extent(manifest, block)
            uv_bounds = (cx-w/2, cx+w/2, cy-h/2, cy+h/2)
        else:
            x0, x1, y0, y1 = block['bounds']
            uv_bounds = None
        path = out/f"visual_{block['row']}_{block['column']}.obj"
        mesh = write_feature_mesh(path, source, bounds=(x0, x1, y0, y1), uv_bounds=uv_bounds)
        visuals.append({'file': path.name, 'bounds_m': [x0, x1, y0, y1],
                        'uv_bounds_m': uv_bounds, 'triangles': mesh['triangles'],
                        'sampled_error_m': mesh['sampled_height_error_max_m'],
                        'texture': str(Path(texture_directory, block['file']).resolve())
                        if manifest else None})
    result = {'identity': identity, 'driver': driver, 'proxy': proxy,
              'caster_mesh': caster_mesh, 'visuals': visuals,
              'build_seconds': time.monotonic()-start,
              'omitted_contact_cracks': contact_source.config['omitted_cracks'],
              'files_sha256': {p.name: digest(p) for p in out.iterdir() if p.is_file()}}
    (out/'manifest.json').write_text(json.dumps(result, indent=2)+'\n')
    out.rename(final)
    return final, result


def apply(world, plugin, assets, manifest, surface_pose, size, config):
    """Install exact same-source optical surface, driver heightmap and ball proxy."""
    q = quaternion_from_rpy(*surface_pose[3:])
    origin = np.array(surface_pose[:3])
    for name in ['wall', 'caster_proxy_wall']:
        item = world.find(f'model[@name="{name}"]')
        if item is not None:
            world.remove(item)

    def link(name, bounds, low=0):
        x0, x1, y0, y1 = bounds
        centre = origin+rotate_vector(q, [(x0+x1)/2, (y0+y1)/2, low])
        model = node(world, 'model', name=name)
        node(model, 'static', 'true')
        node(model, 'pose', numbers([*map(float, centre), *surface_pose[3:]]))
        return node(model, 'link', name='surface')

    driver = manifest['driver']
    base = link('m2_driver_wall', (0, size[0], 0, size[1]), driver['low_m'])
    collision = node(base, 'collision', name='surface_collision')
    shape = node(node(collision, 'geometry'), 'heightmap')
    node(shape, 'uri', (assets/'driver.png').as_uri())
    n = driver['count']
    node(shape, 'size', numbers([
        size[0]*n/(n-1), size[1]*n/(n-1), driver['high_m']-driver['low_m']]))
    node(shape, 'pos', '0 0 0')

    def friction(collision, mask):
        surface = node(collision, 'surface')
        node(node(surface, 'contact'), 'collide_bitmask', mask)
        ode = node(node(surface, 'friction'), 'ode')
        node(ode, 'mu', 1.1)
        node(ode, 'mu2', 1.1)

    friction(collision, 1)

    def tile_pose(element, bounds):
        x0, x1, y0, y1 = bounds
        node(element, 'pose', numbers([(x0+x1)/2, (y0+y1)/2, 0, 0, 0, 0]))

    # One static link per role, as on the M1 wall: hundreds of separate static
    # models each become a DART skeleton whose pose bookkeeping runs every step
    # and cut the real-time factor to about 0.6 without touching any contact.
    caster = link('m2_caster_proxy', (0, 0, 0, 0))
    collision = node(caster, 'collision', name='surface_collision')
    node(node(node(collision, 'geometry'), 'mesh'), 'uri',
         (assets/manifest['caster_mesh']['file']).as_uri())
    friction(collision, 2)
    optical = link('m2_optical_wall', (0, 0, 0, 0))
    for i, tile in enumerate(manifest['visuals']):
        visual = node(optical, 'visual', name=f'tile_{i}')
        tile_pose(visual, tile['bounds_m'])
        # Same as the M1 wall: the camera lamp is in front, so wall shadows only
        # add a full-wall shadow pass and never shade the robot or the photo.
        node(visual, 'cast_shadows', 'false')
        node(node(node(visual, 'geometry'), 'mesh'), 'uri', (assets/tile['file']).as_uri())
        material = node(visual, 'material')
        node(material, 'ambient', '1 1 1 1')
        node(material, 'diffuse', '1 1 1 1')
        node(material, 'specular', '.1 .1 .1 1')
        if tile['texture']:
            metal = node(node(material, 'pbr'), 'metal')
            node(metal, 'albedo_map', Path(tile['texture']).as_uri())
            node(metal, 'metalness', '0.0')
    source = WallSurface(config)
    node(plugin, 'wall_surface_yaml', json.dumps(yaml.safe_load(source.resolved_yaml)))
    for key, value in config['channel_model'].items():
        node(plugin, 'channel_'+key, value)
    samples = np.array([list(map(float, p.findtext('attachment').split()))
                        for p in plugin.findall('sample')])
    if samples.ndim != 2 or samples.shape[1] != 3:
        raise ValueError('full wall requires generated seal sample geometry')
    x0, y0, gap = samples.min(axis=0)
    x1, y1, _ = samples.max(axis=0)
    for x, y in [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]:
        node(plugin, 'seal_corner', numbers([x, y, gap]))
    return {'cache_key': manifest['identity'], 'driver_samples': driver['count']**2,
            'caster_tiles': len(manifest['proxy']), 'visual_tiles': len(manifest['visuals']),
            'caster_maximum_error_m': max(p['centroid_error_m'] for p in manifest['proxy']),
            'driver_surface_policy': 'roughness_only',
            'caster_contact_policy': 'bridge_narrow_cracks',
            'optical_surface_policy': 'full_source',
            'triangle_counts': {'caster': sum(p['triangles'] for p in manifest['proxy']),
                                'visual': sum(p['triangles'] for p in manifest['visuals'])}}
