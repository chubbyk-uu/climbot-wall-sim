# Copyright 2026 jerry
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Small full-robot B-mode patch; source, contacts and runtime gap share one surface."""

import hashlib
import json
import time

from _climbot_surface import SphereEnvelope
from climbot_description.geometry import quaternion_from_rpy, rotate_vector
from climbot_description.vacuum_preview import node, numbers
import numpy as np
from PIL import Image
import yaml

from .surface_mesh import write_cross_joint_mesh
from .wall_surface import WallSurface, write_tensor_mesh


class Envelope:
    """Vector wrapper over native sampled dilation, used only during generation."""

    def __init__(self, source, axes, radius):
        u, v = np.meshgrid(*axes)
        self.native = SphereEnvelope(*axes, source.height(u, v))
        self.radius = radius

    def height(self, u, v):
        """Evaluate the native spherical dilation of the source."""
        u, v = np.broadcast_arrays(u, v)
        return self.native.evaluate(np.ascontiguousarray(u), np.ascontiguousarray(v), self.radius)

    def normal(self, u, v):
        """OBJ shading normals; native collision still uses geometric face normals."""
        e = 1e-6
        du = (self.height(u + e, v) - self.height(u - e, v)) / (2 * e)
        dv = (self.height(u, v + e) - self.height(u, v - e)) / (2 * e)
        n = np.stack([-du, -dv, np.ones_like(du)], axis=-1)
        return n / np.linalg.norm(n, axis=-1, keepdims=True)


def apply(world, plugin, out, config, surface_pose, wheel_radius, size=1.2):
    """Install a bounded patch near the robot, with no hidden flat collision plane."""
    channels = config.get('channel_model')
    if plugin.findtext('pressure_mode') != 'fixed_pressure' and channels is None:
        raise ValueError('surface patch requires fixed_pressure or explicit residual channels')
    if channels is not None:
        if (set(channels) != {'drape_m', 'transition_m', 'refinement'} or
                not np.isfinite([channels['drape_m'], channels['transition_m']]).all() or
                not 0 <= channels['drape_m'] <= .01 or channels['transition_m'] <= 0 or
                type(channels['refinement']) is not int or not 1 <= channels['refinement'] <= 8):
            raise ValueError('invalid residual channel parameters')
        for key, value in channels.items():
            node(plugin, 'channel_' + key, value)
    if len(config['grooves']) not in (0, 2):
        raise ValueError('patch requires two orthogonal panel joints or a smooth control')
    joint_width = max((g['width_m'] for g in config['grooves']), default=0.)
    if not np.isfinite(wheel_radius) or wheel_radius <= joint_width / np.sqrt(2):
        raise ValueError('invalid wheel radius for driver bridge bounds')
    start = time.monotonic()
    source = WallSurface(config)
    representation = config.get('contact_representation', 'mesh')
    if representation not in ('mesh', 'native_heightmap'):
        raise ValueError('unknown patch contact representation')
    native_heightmap = representation == 'native_heightmap'
    if native_heightmap and any(g.get('kind') != 'panel_joint' or g['width_m'] > .005
                                for g in config['grooves']):
        raise ValueError('driver bridge approximation only covers panel joints up to 5 mm')
    # Place the robot at (size/2, .45) in this patch, preserving its world pose.
    robot = world.find('model[@name="climbot"]')
    body = np.array(list(map(float, robot.findtext('pose').split()[:3])))
    q = quaternion_from_rpy(*surface_pose[3:])
    normal = np.array(rotate_vector(q, [0, 0, 1]))
    projection = body - normal * np.dot(body - np.array(surface_pose[:3]), normal)
    origin = projection - rotate_vector(q, [config.get('start_u_m', size / 2), .45, 0])
    patch_pose = [*map(float, origin), *surface_pose[3:]]
    center = origin + rotate_vector(q, [size / 2, size / 2, 0])
    out = (out / 'surface_patch').resolve()
    out.mkdir()
    (out / 'source.yaml').write_text(source.resolved_yaml)
    base = np.linspace(0, size, round(size/.01)+1)
    real = (write_cross_joint_mesh(out / 'real.obj', source, size=size)
            if config['grooves'] else
            write_tensor_mesh(out / 'real.obj', source, base, base))
    axes = []
    for axis in range(2):
        if not config['grooves']:
            axes.append(base)
            continue
        g = config['grooves'][1 - axis]
        c, half = g['start_m'][axis], g['width_m'] / 2
        values = np.r_[np.linspace(0, size, round(size/.01)+1),
                       c + np.linspace(-half-.0001, half+.0001, 53),
                       c + np.linspace(-.0002, .0002, 17), c-half, c, c+half]
        axes.append(np.unique(np.round(values, 12)))
    # Include the later facet-check coordinates in the sampled source. Otherwise
    # a 1 mm source grid introduces up to 40 um of sphere-cap sampling error at
    # unsampled centroids even on a locally flat panel, obscuring mesh accuracy.
    source_axes = []
    for a in axes:
        samples = np.r_[np.arange(-.02, size+.0201, .001), a,
                        (2*a[:-1]+a[1:])/3, (a[:-1]+2*a[1:])/3]
        source_axes.append(np.unique(samples))
    envelope = Envelope(source, source_axes, .00625)
    proxy = write_tensor_mesh(out / 'proxy.obj', envelope, *axes)
    # Sample each mesh cell at two triangle centroids against the native dilation.
    u, v = np.meshgrid(*axes)
    heights = envelope.height(u, v)
    errors = []
    for index, weights in enumerate([(1/3, 2/3), (2/3, 1/3)]):
        a, b = weights
        x = a * u[:-1, :-1] + b * u[:-1, 1:]
        y = b * v[:-1, :-1] + a * v[1:, :-1]
        if index == 0:
            linear = (heights[:-1, :-1] + heights[:-1, 1:] + heights[1:, 1:]) / 3
        else:
            linear = (heights[:-1, :-1] + heights[1:, 1:] + heights[1:, :-1]) / 3
        errors.append(float(np.max(np.abs(envelope.height(x, y) - linear))))
    interpolation_error = max(errors)
    if interpolation_error > .00001:
        raise ValueError(f'proxy facet geometry error exceeds 0.01 mm: {interpolation_error}')
    for name in ['wall', 'caster_proxy_wall']:
        old = world.find(f'model[@name="{name}"]')
        if old is not None:
            world.remove(old)
    driver_geometry = 'mesh'
    for name, filename, bits in [('m2_real_wall', 'real.obj', 1),
                                 ('m2_caster_wall', 'proxy.obj', 2)]:
        model = node(world, 'model', name=name)
        node(model, 'static', 'true')
        placement = center
        low = 0.
        if bits == 1 and native_heightmap:
            u, v = np.meshgrid(np.linspace(0, size, 257), np.linspace(0, size, 257))
            # Bridge narrow joints for drivers/body/stops only. The caster,
            # visual and runtime skirt continue to use the full source.
            height = source.roughness(u, v)
            low, high = float(height.min()), float(height.max())
            driver_geometry = 'heightmap' if high > low else 'box'
            if driver_geometry == 'heightmap':
                Image.fromarray(np.flipud(np.rint((height-low)/(high-low)*65535).astype(
                    np.uint16))).save(out / 'height.png')
            placement = center + normal * low
        node(model, 'pose', numbers([*map(float, placement), *surface_pose[3:]]))
        link = node(model, 'link', name='surface')
        collision = node(link, 'collision', name='surface_collision')
        geometry = node(collision, 'geometry')
        if bits == 1 and native_heightmap and driver_geometry == 'box':
            node(collision, 'pose', '0 0 -0.025 0 0 0')
            node(node(geometry, 'box'), 'size', numbers([size, size, .05]))
        elif bits == 1 and native_heightmap:
            shape = node(geometry, 'heightmap')
            node(shape, 'uri', (out / 'height.png').as_uri())
            node(shape, 'size', numbers([size*257/256, size*257/256, high-low]))
            node(shape, 'pos', '0 0 0')
        else:
            node(node(geometry, 'mesh'), 'uri', (out / filename).as_uri())
        surface = node(collision, 'surface')
        node(node(surface, 'contact'), 'collide_bitmask', bits)
        friction = node(node(surface, 'friction'), 'ode')
        node(friction, 'mu', 1.1)
        node(friction, 'mu2', 1.1)
        if bits == 1:
            visual = node(link, 'visual', name='surface_visual')
            node(visual, 'pose', numbers([0, 0, -low, 0, 0, 0]))
            node(node(node(visual, 'geometry'), 'mesh'), 'uri', (out / 'real.obj').as_uri())
            material = node(visual, 'material')
            node(material, 'ambient', '.4 .4 .38 1')
            node(material, 'diffuse', '.7 .7 .67 1')
    plugin.find('wall_pose').text = numbers(patch_pose)
    plugin.find('wall_size').text = numbers([size, size])
    # sdformat normalizes whitespace in string parameters. JSON is a one-line
    # YAML flow document, preserving the same coefficients through that path.
    node(plugin, 'wall_surface_yaml', json.dumps(yaml.safe_load(source.resolved_yaml)))
    return {'scope': ('M2 coupled patch; pass requires independent T2 checks' if channels else
                      'B-mode contact patch; residual channels and fan margin not validated'),
            'channel_model': channels,
            'size_m': size,
            'source_sha256': hashlib.sha256(source.resolved_yaml.encode()).hexdigest(),
            'wall_surface_pose': patch_pose, 'real': real, 'proxy': proxy,
            'contact_representation': 'native_heightmap' if native_heightmap else 'mesh',
            'driver_collision_geometry': driver_geometry,
            'driver_surface_policy': 'roughness_only' if native_heightmap else 'full_source',
            'bridge_scope': ('panel joints <=5 mm, no panel step; body/stops also bridged; '
                             'stop error is not bounded by wheel sag'
                             ) if native_heightmap else None,
            'driver_radius_m': wheel_radius,
            'ideal_driver_omitted_sag_m': {
                'single': wheel_radius - np.sqrt(wheel_radius**2 - (joint_width/2)**2),
                'cross_diagonal': wheel_radius - np.sqrt(wheel_radius**2 - joint_width**2/2)
            } if native_heightmap else None,
            'joint_count': len(config['grooves']),
            'proxy_centroid_error_m': interpolation_error,
            'generation_seconds': time.monotonic() - start}
