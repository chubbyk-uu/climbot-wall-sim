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
import math
import os
from xml.sax.saxutils import quoteattr

#: How far the texture blocks stand off the wall face, in metres. Large enough
#: that the renderer does not have to choose between two surfaces at one depth,
#: small enough to stay far inside the 3 mm the reference grid used, so the
#: photographed plane and the collision plane remain the same plane to the
#: tolerances anything here is measured at.
SURFACE_OFFSET_M = 0.001


def load_manifest(path, wall_size=None):
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

    for key in ('scale_m_per_px', 'region_origin_m', 'region_m',
                'width_px', 'height_px', 'maps'):
        if key not in manifest:
            raise KeyError('the manifest has no %s; rebake with '
                           'tools/bake_wall_texture.py' % key)
    scale = _positive(manifest, 'scale_m_per_px')
    for key in ('region_origin_m', 'region_m'):
        pair = manifest[key]
        if not isinstance(pair, list) or len(pair) != 2:
            raise ValueError('%s must be two numbers' % key)
        for value in pair:
            if not _finite_number(value):
                raise ValueError('%s must be two finite numbers' % key)
    origin_x, origin_y = (float(value)
                          for value in manifest['region_origin_m'])
    region_width, region_height = (float(value) for value in manifest['region_m'])
    if region_width <= 0.0 or region_height <= 0.0:
        raise ValueError('region_m must be positive')
    width_px = _positive_integer(manifest, 'width_px')
    height_px = _positive_integer(manifest, 'height_px')
    if not math.isclose(region_width, width_px * scale,
                        rel_tol=1e-9, abs_tol=1e-12):
        raise ValueError('region_m width does not match width_px and scale')
    if not math.isclose(region_height, height_px * scale,
                        rel_tol=1e-9, abs_tol=1e-12):
        raise ValueError('region_m height does not match height_px and scale')
    if wall_size is not None:
        if (not isinstance(wall_size, (list, tuple)) or len(wall_size) != 2 or
                not all(_finite_number(value) and value > 0.0
                        for value in wall_size)):
            raise ValueError('wall_size must be two positive finite numbers')
        wall_width, wall_height = (float(value) for value in wall_size)
        if (origin_x < 0.0 or origin_y < 0.0 or
                origin_x + region_width > wall_width + 1e-9 or
                origin_y + region_height > wall_height + 1e-9):
            raise ValueError(
                'the %.3f x %.3f m texture region at (%.3f, %.3f) lies '
                'outside the %.3f x %.3f m wall' %
                (region_width, region_height, origin_x, origin_y,
                 wall_width, wall_height))

    if not isinstance(manifest['maps'], dict) or not manifest['maps']:
        raise ValueError('maps must be a non-empty object')
    for name, group in manifest['maps'].items():
        if not isinstance(group, dict):
            raise ValueError('map %s must be an object' % name)
        blocks = group.get('blocks')
        if not isinstance(blocks, list) or not blocks:
            raise ValueError('map %s lists no blocks' % name)
        rectangles = []
        for block in blocks:
            if not isinstance(block, dict):
                raise ValueError('a block of map %s must be an object' % name)
            for key in ('file', 'row', 'column', 'x_px', 'y_px',
                        'width_px', 'height_px'):
                if key not in block:
                    raise KeyError('a block of map %s has no %s' % (name, key))
            if not isinstance(block['file'], str) or not block['file']:
                raise ValueError('a block of map %s has no valid file name' % name)
            for key in ('row', 'column', 'x_px', 'y_px'):
                _nonnegative_integer(block, key, 'a block of map %s' % name)
            block_width = _positive_integer(
                block, 'width_px', 'a block of map %s' % name)
            block_height = _positive_integer(
                block, 'height_px', 'a block of map %s' % name)
            candidate = os.path.join(directory, block['file'])
            if not os.path.exists(candidate):
                raise FileNotFoundError(
                    'manifest lists %s for map %s but it is missing; '
                    'rebake with tools/bake_wall_texture.py' % (candidate, name))
            # Off the region is off the wall. A block placed there renders
            # somewhere nobody is looking, and the surface it should have
            # covered stays blank - which reads as a camera fault rather than
            # as a bad manifest.
            left = block['x_px']
            top = block['y_px']
            right = left + block_width
            bottom = top + block_height
            if right > width_px or bottom > height_px:
                raise ValueError(
                    'block %s of map %s runs outside the %d x %d px region'
                    % (block['file'], name, width_px, height_px))
            rectangles.append((left, top, right, bottom, block['file']))
            _validate_sample_extent(manifest, block, name)
        _require_exact_tiling(name, rectangles, width_px, height_px)
    return manifest, directory


def _finite_number(value):
    """Report whether a value is a finite real JSON number other than bool."""
    return (isinstance(value, (int, float)) and not isinstance(value, bool) and
            math.isfinite(value))


def _positive(manifest, key):
    """Return a manifest number that has to be positive and finite."""
    value = manifest[key]
    if not _finite_number(value) or value <= 0.0:
        raise ValueError('%s must be a positive number' % key)
    return float(value)


def _positive_integer(values, key, owner='the manifest'):
    """Return a strictly positive JSON integer field."""
    value = values[key]
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ValueError('%s %s must be a positive integer' % (owner, key))
    return value


def _nonnegative_integer(values, key, owner):
    """Return a non-negative JSON integer field."""
    value = values[key]
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError('%s %s must be a non-negative integer' % (owner, key))
    return value


def _require_exact_tiling(name, rectangles, width_px, height_px):
    """Reject overlaps and gaps without allocating a full-resolution bitmap."""
    total_area = 0
    for index, first in enumerate(rectangles):
        total_area += (first[2] - first[0]) * (first[3] - first[1])
        for second in rectangles[index + 1:]:
            overlap_width = min(first[2], second[2]) - max(first[0], second[0])
            overlap_height = min(first[3], second[3]) - max(first[1], second[1])
            if overlap_width > 0 and overlap_height > 0:
                raise ValueError(
                    'blocks %s and %s overlap in map %s' %
                    (first[4], second[4], name))
    if total_area != width_px * height_px:
        raise ValueError(
            'blocks of map %s leave gaps in the %d x %d px region' %
            (name, width_px, height_px))


def _validate_sample_extent(manifest, block, name):
    """Validate the optional neighbouring pixels stored around one block."""
    sample_keys = ('sample_x_px', 'sample_y_px',
                   'sample_width_px', 'sample_height_px')
    present = [key in block for key in sample_keys]
    if any(present) and not all(present):
        raise KeyError('a block of map %s has an incomplete sample extent' % name)
    if not any(present):
        if manifest.get('gutter_px', 0) != 0:
            raise KeyError(
                'a block of map %s has no sampled extent for gutter_px' % name)
        return
    for key in ('sample_x_px', 'sample_y_px'):
        _nonnegative_integer(block, key, 'a block of map %s' % name)
    for key in ('sample_width_px', 'sample_height_px'):
        _positive_integer(block, key, 'a block of map %s' % name)

    sample_left, sample_top = block['sample_x_px'], block['sample_y_px']
    sample_right = sample_left + block['sample_width_px']
    sample_bottom = sample_top + block['sample_height_px']
    nominal_left, nominal_top = block['x_px'], block['y_px']
    nominal_right = nominal_left + block['width_px']
    nominal_bottom = nominal_top + block['height_px']
    if (sample_left > nominal_left or sample_top > nominal_top or
            sample_right < nominal_right or sample_bottom < nominal_bottom):
        raise ValueError(
            'the sampled pixels of block %s in map %s do not contain its '
            'nominal wall area' % (block['file'], name))
    if (sample_right > manifest['width_px'] or
            sample_bottom > manifest['height_px']):
        raise ValueError(
            'the sampled pixels of block %s in map %s run outside the bake' %
            (block['file'], name))

    gutter = manifest.get('gutter_px')
    if gutter is not None:
        if not isinstance(gutter, int) or isinstance(gutter, bool) or gutter < 0:
            raise ValueError('gutter_px must be a non-negative integer')
        expected = (
            max(0, nominal_left - gutter),
            max(0, nominal_top - gutter),
            min(manifest['width_px'], nominal_right + gutter),
            min(manifest['height_px'], nominal_bottom + gutter))
        if ((sample_left, sample_top, sample_right, sample_bottom) != expected):
            raise ValueError(
                'the sampled pixels of block %s in map %s do not match '
                'gutter_px' % (block['file'], name))


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


def sampled_block_extent(manifest, block):
    """
    Return the wall extent represented by a block's stored texture.

    New bakes include real neighbouring pixels around each nominal block so
    texture filtering does not clamp at visual boundaries.  Old manifests
    remain loadable and simply use their nominal, non-overlapping extent.
    """
    sampled = dict(block)
    if 'sample_x_px' in block:
        sampled.update(
            x_px=block['sample_x_px'], y_px=block['sample_y_px'],
            width_px=block['sample_width_px'],
            height_px=block['sample_height_px'])
    return block_extent(manifest, sampled)


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
        work_x, work_y, width, height = sampled_block_extent(manifest, block)
        elements.append(
            '        <visual name="wall_texture_r%02d_c%02d">\n'
            '          <pose>%.6f %.6f %.6f 0 0 0</pose>\n'
            # The collision wall underneath already casts the physical wall's
            # shadow. Overlapping gutter visuals must not shadow one another
            # at the same nominal surface or their borders become dark lines.
            '          <cast_shadows>false</cast_shadows>\n'
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
