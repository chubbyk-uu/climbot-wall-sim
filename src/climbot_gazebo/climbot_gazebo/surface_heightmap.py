# Copyright 2026 jerry
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Offline overlapping native heightmaps with lowered support beneath fine joints."""

from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
from PIL import Image


def add_panel_heightmaps(world, out, surface):
    """Use two narrow strips and one intersection patch, rather than a full fine wall."""
    horizontal, vertical = surface.config['grooves']
    center_u, center_v = vertical['start_m'][0], horizontal['start_m'][1]
    # Align every patch boundary with both the 257-point base and 129-point
    # strip longitudinal grids. Lower strict interiors only: interpolation
    # ramps remain behind the fine surface and meet it on a shared boundary.
    coarse_step, half_width = .8/128, .0125
    if any(abs(c/coarse_step-round(c/coarse_step)) > 1e-9
           for c in [center_u, center_v]):
        raise ValueError('diagnostic heightmap joints must align with the shared grid')
    patches = [('base', (0., .8), (0., .8), 257),
               ('vertical', (center_u-half_width, center_u+half_width), (0., .8), 129),
               ('horizontal', (0., .8), (center_v-half_width, center_v+half_width), 129),
               ('intersection', (center_u-half_width, center_u+half_width),
                (center_v-half_width, center_v+half_width), 129)]
    results = []

    def node(parent, tag, text=None, **attrs):
        item = ET.SubElement(parent, tag, attrs)
        if text is not None:
            item.text = str(text)
        return item

    prepared = []
    for name, u_range, v_range, count in patches:
        u, v = np.meshgrid(np.linspace(*u_range, count), np.linspace(*v_range, count))
        height = surface.height(u, v)
        # Remove coarse support under fine features; never cover the groove mouth.
        lower = np.zeros(u.shape, dtype=bool)
        if name == 'base':
            lower = (np.abs(u-center_u) < half_width-1e-12) | (np.abs(v-center_v) < half_width-1e-12)
        elif name == 'vertical':
            lower = np.abs(v-center_v) < half_width-1e-12
        elif name == 'horizontal':
            lower = np.abs(u-center_u) < half_width-1e-12
        height = np.where(lower, surface.roughness(u, v)-.012, height)
        prepared.append((name, u_range, v_range, count, height, lower))
    # Identical quantization at shared edge nodes; per-tile ranges can create
    # small steps even when their original floating-point heights agree.
    lo = min(float(p[4].min()) for p in prepared)
    hi = max(float(p[4].max()) for p in prepared)
    for name, u_range, v_range, count, height, lower in prepared:
        encoded = np.rint((height-lo)/(hi-lo)*65535).astype(np.uint16)
        path = Path(out)/(name+'.png')
        Image.fromarray(np.flipud(encoded)).save(path)
        model = node(world, 'model', name='surface_'+name)
        node(model, 'static', 'true')
        node(model, 'pose', f'{lo:.17g} {sum(u_range)/2-.4:.17g} '
             f'{sum(v_range)/2+1:.17g} {np.pi/2:.17g} 0 {np.pi/2:.17g}')
        link = node(model, 'link', name='surface')
        collision = node(link, 'collision', name='surface_collision')
        shape = node(node(collision, 'geometry'), 'heightmap')
        node(shape, 'uri', str(path.resolve()))
        node(shape, 'size', f'{(u_range[1]-u_range[0])*count/(count-1):.17g} '
             f'{(v_range[1]-v_range[0])*count/(count-1):.17g} {hi-lo:.17g}')
        node(shape, 'pos', '0 0 0')
        friction = node(node(node(collision, 'surface'), 'friction'), 'ode')
        node(friction, 'mu', 0)
        node(friction, 'mu2', 0)
        results.append({'name': name, 'samples': count*count, 'u_range_m': u_range,
                        'v_range_m': v_range, 'lowered_samples': int(lower.sum()),
                        'max_quantization_error_m': (hi-lo)/65535/2})
    return {'patches': results, 'samples': sum(p['samples'] for p in results),
            'fine_width_m': 2*half_width, 'underlying_support_lowering_m': .012}
