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

"""Place the baked wall texture on the wall face as a grid of flat blocks."""

# One texture cannot hold the whole photographed area at the scale the camera
# resolves - the wall is 320 Mtexels at 0.50 mm per pixel - so the bake is cut
# into blocks and each becomes its own visual. The blocks are coplanar with one
# another and stand clear of the wall face by a single shared offset, which
# matters more than it sounds: a stitch fits a homography, and a homography
# describes one plane. Anything sitting at a different depth - the reference
# grid did, by 3 mm - is a second plane, and the mosaic pays for it as parallax
# that looks like a pose error.
#
# The wall keeps its collision box and its flat visual underneath. Only the
# appearance of the front face changes, which is the line PROJECT_GUIDE 5.1
# draws: the calibrated geometry and the friction and WheelSlip parameters
# measured against it must not move because a picture was applied to it.

import json
import os
from xml.sax.saxutils import quoteattr

#: How far the texture blocks stand off the wall face, in metres. Large enough
#: that the renderer does not have to choose between two surfaces at one depth,
#: small enough to stay far inside the 3 mm the reference grid used, so the
#: photographed plane and the collision plane remain the same plane to the
#: tolerances anything here is measured at.
SURFACE_OFFSET_M = 0.001


def load_manifest(path):
    """Read a bake manifest and check it describes a wall that can be built."""
    # Everything here is checked before a single visual is emitted, because the
    # alternative is finding out from the rendered wall. That is how the work
    # frame origin bug was found: blocks landed half a wall sideways, half the
    # surface rendered untextured, and nothing in the load reported anything.
    # A manifest written by hand, edited, or half-copied between machines can
    # go wrong in the same silent way, and it is cheap to refuse it here.
    with open(path, encoding='utf-8') as handle:
        manifest = json.load(handle)
    directory = os.path.dirname(os.path.abspath(path))

    for key in ('scale_m_per_px', 'region_origin_m', 'region_m', 'maps'):
        if key not in manifest:
            raise KeyError('the manifest has no %s; rebake with '
                           'tools/bake_wall_texture.py' % key)
    scale = _positive(manifest, 'scale_m_per_px')
    for key in ('region_origin_m', 'region_m'):
        pair = manifest[key]
        if not isinstance(pair, list) or len(pair) != 2:
            raise ValueError('%s must be two numbers' % key)
        for value in pair:
            if not isinstance(value, (int, float)) or value != value:
                raise ValueError('%s must be two finite numbers' % key)
    region_width, region_height = (float(value) for value in manifest['region_m'])
    if region_width <= 0.0 or region_height <= 0.0:
        raise ValueError('region_m must be positive')

    for name, group in manifest['maps'].items():
        blocks = group.get('blocks')
        if not blocks:
            raise ValueError('map %s lists no blocks' % name)
        for block in blocks:
            for key in ('file', 'row', 'column', 'x_px', 'y_px',
                        'width_px', 'height_px'):
                if key not in block:
                    raise KeyError('a block of map %s has no %s' % (name, key))
            if block['width_px'] <= 0 or block['height_px'] <= 0:
                raise ValueError('a block of map %s has no area' % name)
            candidate = os.path.join(directory, block['file'])
            if not os.path.exists(candidate):
                raise FileNotFoundError(
                    'manifest lists %s for map %s but it is missing; '
                    'rebake with tools/bake_wall_texture.py' % (candidate, name))
            # Off the region is off the wall. A block placed there renders
            # somewhere nobody is looking, and the surface it should have
            # covered stays blank - which reads as a camera fault rather than
            # as a bad manifest.
            right = (block['x_px'] + block['width_px']) * scale
            bottom = (block['y_px'] + block['height_px']) * scale
            if block['x_px'] < 0 or block['y_px'] < 0:
                raise ValueError(
                    'block %s of map %s starts outside the baked region'
                    % (block['file'], name))
            if right > region_width + scale or bottom > region_height + scale:
                raise ValueError(
                    'block %s of map %s runs %.3f x %.3f m past the %.3f x %.3f m '
                    'region it claims to tile'
                    % (block['file'], name, max(0.0, right - region_width),
                       max(0.0, bottom - region_height), region_width, region_height))
    return manifest, directory


def _positive(manifest, key):
    """Return a manifest number that has to be positive and finite."""
    value = manifest[key]
    if not isinstance(value, (int, float)) or value != value or value <= 0.0:
        raise ValueError('%s must be a positive number' % key)
    return float(value)


def block_extent(manifest, block):
    """Return a block's centre and size in the wall work frame, in metres."""
    scale = manifest['scale_m_per_px']
    origin_x, origin_y = manifest['region_origin_m']
    region_height = manifest['region_m'][1]
    width = block['width_px'] * scale
    height = block['height_px'] * scale
    # Pixel rows run down from the top of the bake, wall coordinates run up
    # from the bottom, so the vertical term is subtracted rather than added.
    centre_x = origin_x + (block['x_px'] + block['width_px'] / 2.0) * scale
    centre_y = (origin_y + region_height
                - (block['y_px'] + block['height_px'] / 2.0) * scale)
    return centre_x, centre_y, width, height


def texture_visuals(manifest, directory, thickness, wall_origin, link_centre,
                    offset_m=SURFACE_OFFSET_M):
    """Build the SDF <visual> elements for every baked block, as XML text."""
    # wall_origin is the work frame's origin in the Gazebo world, from
    # climbot_description/config/wall.yaml; link_centre is where the wall body
    # sits, from climbot_gazebo/config/simulation.yaml. Both are needed and
    # neither may be assumed: a block's place on the wall is a work coordinate,
    # a visual's pose is relative to the link, and the two differ by exactly
    # those. This was written once with the wall origin left out, which was
    # harmless only while the origin sat at the wall's centre. Once it moved to
    # the corner every block shifted half a wall sideways, and half the wall
    # rendered untextured with nothing in the load reporting a problem.
    if 'albedo' not in manifest['maps']:
        raise KeyError('the bake has no albedo map')

    depth = thickness / 2.0 + offset_m
    elements = []
    for block in manifest['maps']['albedo']['blocks']:
        work_x, work_y, width, height = block_extent(manifest, block)
        elements.append(
            '        <visual name="wall_texture_r%02d_c%02d">\n'
            '          <pose>%.6f %.6f %.6f 0 0 0</pose>\n'
            '          <geometry>\n'
            '            <box>\n'
            '              <size>0.0005 %.6f %.6f</size>\n'
            '            </box>\n'
            '          </geometry>\n'
            '          <material>\n'
            # The albedo map is modulated by the material base colour, which
            # defaults dark enough to multiply the texture away: the surface
            # renders pure black while the flat wall beside it lights normally,
            # and nothing in the load reports a problem. White here leaves the
            # baked pixels as the only thing deciding what the camera sees.
            '            <ambient>1 1 1 1</ambient>\n'
            '            <diffuse>1 1 1 1</diffuse>\n'
            '            <specular>0.1 0.1 0.1 1</specular>\n'
            '            <pbr>\n'
            '              <metal>\n'
            '                <albedo_map>%s</albedo_map>\n'
            '                <metalness>0.0</metalness>\n'
            '              </metal>\n'
            '            </pbr>\n'
            '          </material>\n'
            '        </visual>' % (
                block['row'], block['column'],
                depth,
                wall_origin[1] + work_x - link_centre[1],
                wall_origin[2] + work_y - link_centre[2],
                width, height,
                # Escaped, because this text is parsed as XML by the caller.
                # A directory with an ampersand or an angle bracket in it - a
                # checkout under a path somebody named, a Windows share seen
                # through WSL - would otherwise make minidom.parseString fail
                # on the whole world file, pointing at a line the bake wrote.
                _texture_uri(directory, block['file'])))
    return elements


def _texture_uri(directory, name):
    """Return a file:// URI for one block, safe to place in XML text."""
    escaped = quoteattr('file://' + os.path.join(directory, name))
    # quoteattr wraps in quotes for attribute use; this goes in element text.
    return escaped[1:-1]
