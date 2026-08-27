#!/usr/bin/env python3
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

"""Create a reproducible diagnostic variant of a baked concrete wall.

The precision concrete wall remains the baseline.  This tool takes its DDS
blocks as immutable input and puts a small number of construction seams,
cracks, repair patches, and paint marks on a *new* wall.  Every feature is
stored in wall metres, with its seed and the source-manifest digest, so it is
both an easily inspected scene and independent ground truth for P2.7.

Features are deliberately sparse and non-periodic.  They are plausible wall
details, not a calibration chessboard or a repeated motif that can make a
matcher look good for the wrong reason.  The same world-space feature is drawn
into every overlapping DDS gutter that contains it; Gazebo filtering therefore
cannot create a block seam at a diagnostic feature.
"""

import argparse
import copy
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import math
import os
import shutil
import sys
import tempfile
import time

import cv2
import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bc1 import write_dds_bc1  # noqa: E402

Image.MAX_IMAGE_PIXELS = None

PROFILE_NAME = 'p2.7a-sparse-wall-details'
PROFILE_VERSION = 6
CRACK_SOURCE_LUMINANCE_CUTOFF = 180.0
CRACK_SOURCE_TRANSITION = 130.0
CRACK_SOURCE_BLUR_PX = 0.35
DEFAULT_DECAL_ATLAS = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'assets',
    'diagnostic_wall_decals_v2.png')

_ATLAS_CACHE = {}
_SPRITE_CACHE = {}


def sha256(path):
    """Return a file digest without loading the whole file at once."""
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _point(origin, region, fraction):
    """Convert a normalised wall coordinate into its measured wall position."""
    return [origin[0] + region[0] * fraction[0],
            origin[1] + region[1] * fraction[1]]


def _repair_polygon(center, width, height, angle_rad, jitter):
    """Return a hand-trowelled repair outline rather than a regular polygon."""
    # These are uneven points along four roughly straight patch edges, not
    # equal-angle polar samples.  The result reads as a rough mortar repair
    # instead of a conspicuous octagonal target.
    outline = ((-0.50, -0.42), (-0.18, -0.53), (0.33, -0.47),
               (0.53, -0.16), (0.46, 0.33), (0.12, 0.51),
               (-0.37, 0.43), (-0.55, 0.08))
    points = []
    for index, (x_factor, y_factor) in enumerate(outline):
        x = width * (x_factor + jitter[index] * 0.08)
        y = height * (y_factor + jitter[index] * 0.08)
        points.append([
            center[0] + x * math.cos(angle_rad) - y * math.sin(angle_rad),
            center[1] + x * math.sin(angle_rad) + y * math.cos(angle_rad)])
    return points


def feature_layout(origin, region, seed):
    """Build the complete deterministic P2.7a feature specification.

    The positions are intentionally expressed relative to the requested wall
    region: the same profile can be created for a reduced test wall, while the
    emitted manifest still contains only physical metres.  ``seed`` controls
    the subtle patch outline perturbations and marker orientations, not a
    blind scatter that could accidentally cluster features.
    """
    if len(origin) != 2 or len(region) != 2 or min(region) <= 0.0:
        raise ValueError('origin and positive two-dimensional region are required')
    rng = np.random.default_rng(seed)
    features = []

    # These are deliberately true panel joints: straight, wall-edge to
    # wall-edge, and axis aligned.  They should never impersonate cracks.
    seam_paths = (
        ((0.34, 0.0), (0.34, 1.0)),
        ((0.70, 0.0), (0.70, 1.0)),
        ((0.0, 0.53), (1.0, 0.53)),
    )
    for index, path in enumerate(seam_paths, start=1):
        seam_width = 0.011
        points = [_point(origin, region, point) for point in path]
        # Keep the full painted width inside the declared wall.  This is only
        # 5.5 mm short of an edge, yet retains a valid metric feature extent.
        points = [[min(max(point[0], origin[0] + seam_width / 2.0),
                       origin[0] + region[0] - seam_width / 2.0),
                   min(max(point[1], origin[1] + seam_width / 2.0),
                       origin[1] + region[1] - seam_width / 2.0)]
                  for point in points]
        features.append({
            'id': 'construction_seam_%02d' % index,
            'kind': 'construction_seam',
            'points_m': points,
            'width_m': seam_width,
            'rgba': [64, 59, 53, 128],
        })

    # Eight distinct generated alpha decals replace the former procedural
    # polylines.  Their source geometry is already hairline-thin, branched and
    # continuously tapered instead of being a thick chipped-concrete cutout.
    crack_crops = (
        (0.00, 0.00, 0.25, 1 / 3), (0.25, 0.00, 0.50, 1 / 3),
        (0.50, 0.00, 0.75, 1 / 3), (0.75, 0.00, 1.00, 1 / 3),
        (0.00, 1 / 3, 0.25, 2 / 3), (0.25, 1 / 3, 0.50, 2 / 3),
        (0.50, 1 / 3, 0.75, 2 / 3), (0.75, 1 / 3, 1.00, 2 / 3),
    )
    crack_specs = (
        ((0.13, 0.18), (0.55, 1.45), -18.0, 0.64),
        ((0.25, 0.79), (0.75, 1.25), 64.0, 0.56),
        ((0.43, 0.35), (0.62, 1.10), -42.0, 0.50),
        ((0.57, 0.72), (0.52, 1.35), 21.0, 0.59),
        ((0.64, 0.17), (0.88, 0.85), -73.0, 0.48),
        ((0.79, 0.40), (0.58, 1.30), 9.0, 0.60),
        ((0.89, 0.76), (0.48, 1.05), -31.0, 0.47),
        ((0.92, 0.14), (0.82, 0.78), 78.0, 0.52),
    )
    for index, (spec, crop) in enumerate(zip(crack_specs, crack_crops), start=1):
        fraction, size_m, angle_deg, opacity = spec
        features.append({
            'id': 'crack_decal_%02d' % index,
            'kind': 'crack_decal',
            'center_m': _point(origin, region, fraction),
            'size_m': list(size_m),
            'angle_deg': angle_deg,
            'opacity': opacity,
            'atlas_box_fraction': list(crop),
        })

    patch_specs = (
        ((0.20, 0.44), 0.43, 0.29, -0.20, [133, 128, 118, 105]),
        ((0.44, 0.23), 0.58, 0.37, 0.12, [101, 97, 90, 98]),
        ((0.64, 0.62), 0.34, 0.54, -0.31, [150, 142, 128, 92]),
        ((0.88, 0.27), 0.49, 0.25, 0.26, [118, 112, 103, 112]),
        ((0.16, 0.70), 0.29, 0.22, 0.08, [145, 138, 125, 88]),
        ((0.78, 0.84), 0.37, 0.31, -0.17, [108, 104, 97, 96]),
    )
    for index, (fraction, width, height, angle, rgba) in enumerate(patch_specs, start=1):
        center = _point(origin, region, fraction)
        # Scale detail sizes only when making a deliberately smaller test wall;
        # the production 10 x 8 m wall keeps the declared metric dimensions.
        scale = min(1.0, min(region) / 4.0)
        jitter = rng.uniform(-0.12, 0.12, size=8).tolist()
        polygon = _repair_polygon(center, width * scale, height * scale,
                                  angle, jitter)
        features.append({
            'id': 'repair_patch_%02d' % index,
            'kind': 'repair_patch',
            'center_m': center,
            'polygon_m': polygon,
            'rgba': rgba,
            'edge_rgba': [79, 74, 68, 116],
        })

    graffiti_crops = (
        (0.00, 2 / 3, 0.25, 1.00), (0.25, 2 / 3, 0.50, 1.00),
        (0.50, 2 / 3, 0.75, 1.00), (0.75, 2 / 3, 1.00, 1.00),
    )
    graffiti_specs = (
        ((0.09, 0.58), (0.55, 0.55), -12.0, 0.49),
        ((0.37, 0.87), (0.78, 0.38), 7.0, 0.43),
        ((0.72, 0.31), (0.70, 0.43), -9.0, 0.46),
        ((0.94, 0.61), (0.50, 0.50), 14.0, 0.41),
    )
    for index, (spec, crop) in enumerate(
            zip(graffiti_specs, graffiti_crops), start=1):
        fraction, size_m, angle_deg, opacity = spec
        features.append({
            'id': 'graffiti_decal_%02d' % index,
            'kind': 'graffiti_decal',
            'center_m': _point(origin, region, fraction),
            'size_m': list(size_m),
            'angle_deg': angle_deg,
            'opacity': opacity,
            'atlas_box_fraction': list(crop),
        })
    return features


def _world_to_pixel(point, origin, region, scale):
    """Map wall metres to top-left-origin global texture pixels."""
    return ((point[0] - origin[0]) / scale,
            (origin[1] + region[1] - point[1]) / scale)


def _feature_bounds(feature):
    """Return wall-space extent for cheap block-intersection rejection."""
    points = []
    for key in ('points_m', 'polygon_m'):
        points.extend(feature.get(key, []))
    for branch in feature.get('branches_m', []):
        points.extend(branch)
    if 'center_m' in feature:
        center = feature['center_m']
        if 'size_m' in feature:
            radius = math.hypot(*feature['size_m']) / 2.0
        else:
            radius = feature.get('radius_m', 0.0)
        points.extend(((center[0] - radius, center[1] - radius),
                       (center[0] + radius, center[1] + radius)))
    width = feature.get('width_m', 0.0) / 2.0
    return (min(point[0] for point in points) - width,
            min(point[1] for point in points) - width,
            max(point[0] for point in points) + width,
            max(point[1] for point in points) + width)


def _intersects(bounds, sample_bounds):
    return not (bounds[2] < sample_bounds[0] or bounds[0] > sample_bounds[2] or
                bounds[3] < sample_bounds[1] or bounds[1] > sample_bounds[3])


def _decal_sprite(feature, atlas, scale):
    """Extract and style one transparent atlas cell at its metric size."""
    cache_key = (PROFILE_VERSION, feature['id'], float(scale), id(atlas))
    cached = _SPRITE_CACHE.get(cache_key)
    if cached is not None:
        return cached
    box_fraction = feature['atlas_box_fraction']
    box = (round(box_fraction[0] * atlas.width),
           round(box_fraction[1] * atlas.height),
           round(box_fraction[2] * atlas.width),
           round(box_fraction[3] * atlas.height))
    sprite = atlas.crop(box).convert('RGBA')
    content = sprite.getbbox()
    if content is None:
        raise ValueError('diagnostic decal atlas cell is empty: %s' % feature['id'])
    sprite = sprite.crop(content)
    width = max(1, round(feature['size_m'][0] / scale))
    height = max(1, round(feature['size_m'][1] / scale))
    if feature['kind'] == 'crack_decal':
        pixels = np.asarray(sprite, dtype=np.uint8).copy()
        original_alpha = pixels[:, :, 3].astype(np.float32) / 255.0
        luminance = pixels[:, :, :3].astype(np.float32).mean(axis=2)
        darkness = np.clip(
            (CRACK_SOURCE_LUMINANCE_CUTOFF - luminance) /
            CRACK_SOURCE_TRANSITION, 0.0, 1.0)
        source_alpha = original_alpha * darkness
        centerline = cv2.ximgproc.thinning(
            (source_alpha >= 4.0 / 255.0).astype(np.uint8) * 255)
        strength = np.maximum(source_alpha, 0.55)
        alpha = centerline.astype(np.float32) / 255.0 * strength
        alpha = cv2.GaussianBlur(alpha, (0, 0), CRACK_SOURCE_BLUR_PX)
        pixels[:, :, :3] = np.array([43, 40, 38], dtype=np.uint8)
        pixels[:, :, 3] = np.round(
            np.clip(alpha * feature['opacity'], 0.0, 1.0) * 255.0).astype(np.uint8)
        sprite = Image.fromarray(pixels, 'RGBA').resize(
            (width, height), Image.Resampling.LANCZOS)
    else:
        sprite = sprite.resize((width, height), Image.Resampling.LANCZOS)
        pixels = np.asarray(sprite, dtype=np.uint8).copy()
        original_alpha = pixels[:, :, 3].astype(np.float32) / 255.0
        pixels[:, :, :3] = np.array([220, 215, 196], dtype=np.uint8)
        alpha = original_alpha * feature['opacity']
        pixels[:, :, 3] = np.round(alpha * 255.0).astype(np.uint8)
        sprite = Image.fromarray(pixels, 'RGBA')
    sprite = sprite.rotate(feature['angle_deg'],
                           resample=Image.Resampling.BICUBIC, expand=True)
    _SPRITE_CACHE[cache_key] = sprite
    return sprite


def _draw_feature(overlay, feature, origin, region, scale, sample_x, sample_y,
                  decal_atlas=None):
    """Draw one feature into a transparent local DDS-sample overlay."""
    draw = ImageDraw.Draw(overlay, 'RGBA')

    def local(point):
        x, y = _world_to_pixel(point, origin, region, scale)
        return (round(x - sample_x), round(y - sample_y))

    kind = feature['kind']
    if kind in ('construction_seam', 'crack'):
        def draw_path(points, widths):
            if widths:
                for start, end, width_m in zip(points, points[1:], widths):
                    draw.line((local(start), local(end)), fill=tuple(feature['rgba']),
                              width=max(1, round(width_m / scale)))
            else:
                width = max(1, round(feature['width_m'] / scale))
                draw.line([local(point) for point in points],
                          fill=tuple(feature['rgba']), width=width, joint='curve')

        draw_path(feature['points_m'], feature.get('segment_widths_m', []))
        for branch, widths in zip(feature.get('branches_m', []),
                                  feature.get('branch_widths_m', [])):
            draw_path(branch, widths)
    elif kind == 'repair_patch':
        polygon = [local(point) for point in feature['polygon_m']]
        draw.polygon(polygon, fill=tuple(feature['rgba']))
        draw.line(polygon + [polygon[0]], fill=tuple(feature['edge_rgba']),
                  width=max(1, round(0.003 / scale)), joint='curve')
    elif kind == 'paint_mark':
        cx, cy = local(feature['center_m'])
        radius = max(1, round(feature['radius_m'] / scale))
        rgba = tuple(feature['rgba'])
        draw.ellipse((cx - radius, cy - radius, cx + radius, cy + radius),
                     outline=rgba, width=max(1, radius // 5))
        angle = feature['angle_rad']
        dx, dy = math.cos(angle) * radius, math.sin(angle) * radius
        draw.line((cx - dx, cy - dy, cx + dx, cy + dy), fill=rgba,
                  width=max(1, radius // 4))
    elif kind in ('crack_decal', 'graffiti_decal'):
        if decal_atlas is None:
            raise ValueError('diagnostic decal feature requires an atlas image')
        sprite = _decal_sprite(feature, decal_atlas, scale)
        cx, cy = local(feature['center_m'])
        layer = Image.new('RGBA', overlay.size, (0, 0, 0, 0))
        layer.paste(sprite, (round(cx - sprite.width / 2.0),
                             round(cy - sprite.height / 2.0)))
        overlay.alpha_composite(layer)
    else:
        raise ValueError('unsupported diagnostic feature kind %s' % kind)


def render_sample(image, features, origin, region, scale, sample_x, sample_y,
                  decal_atlas=None):
    """Overlay intersecting feature detail on one decoded DDS sample image."""
    sample_width, sample_height = image.size
    x0 = origin[0] + sample_x * scale
    x1 = origin[0] + (sample_x + sample_width) * scale
    y1 = origin[1] + region[1] - sample_y * scale
    y0 = origin[1] + region[1] - (sample_y + sample_height) * scale
    sample_bounds = (x0, y0, x1, y1)
    overlay = Image.new('RGBA', image.size, (0, 0, 0, 0))
    visible = 0
    for feature in features:
        if _intersects(_feature_bounds(feature), sample_bounds):
            _draw_feature(overlay, feature, origin, region, scale,
                          sample_x, sample_y, decal_atlas)
            visible += 1
    if visible:
        return Image.alpha_composite(image.convert('RGBA'), overlay).convert('RGB')
    return image.convert('RGB')


def _sample_bounds(origin, region, scale, block):
    """Return the wall-space extent of one DDS block including its gutter."""
    x0 = origin[0] + block['sample_x_px'] * scale
    x1 = x0 + block['sample_width_px'] * scale
    y1 = origin[1] + region[1] - block['sample_y_px'] * scale
    y0 = y1 - block['sample_height_px'] * scale
    return (x0, y0, x1, y1)


def _block_has_feature(block, features, origin, region, scale):
    """Whether a block needs a new DDS instead of a read-only base link."""
    sample_bounds = _sample_bounds(origin, region, scale, block)
    return any(_intersects(_feature_bounds(feature), sample_bounds)
               for feature in features)


def _encode_modified_block(task):
    """Read, draw, encode, and hash one independent diagnostic DDS block."""
    (source, destination, block, features, origin, region, scale,
     decal_atlas_path) = task
    atlas = _ATLAS_CACHE.get(decal_atlas_path)
    if atlas is None:
        with Image.open(decal_atlas_path) as atlas_file:
            atlas = atlas_file.convert('RGBA')
        _ATLAS_CACHE[decal_atlas_path] = atlas
    with Image.open(source) as decoded:
        sample = render_sample(decoded, features, origin, region, scale,
                               int(block['sample_x_px']),
                               int(block['sample_y_px']), atlas)
    write_dds_bc1(destination, np.asarray(sample, dtype=np.uint8))
    return block['file'], sha256(destination)


def _validate_base_manifest(path):
    """Load only the contract this writer needs, failing before output exists."""
    with open(path, encoding='utf-8') as handle:
        manifest = json.load(handle)
    required = ('scale_m_per_px', 'region_origin_m', 'region_m', 'maps')
    absent = [key for key in required if key not in manifest]
    if absent:
        raise ValueError('base manifest lacks %s' % ', '.join(absent))
    if 'albedo' not in manifest['maps']:
        raise ValueError('base manifest has no albedo DDS blocks')
    blocks = manifest['maps']['albedo'].get('blocks', [])
    if not blocks:
        raise ValueError('base manifest lists no albedo blocks')
    directory = os.path.dirname(os.path.abspath(path))
    for block in blocks:
        for key in ('file', 'sample_x_px', 'sample_y_px'):
            if key not in block:
                raise ValueError('base block lacks %s; rebake with gutters first' % key)
        if not os.path.isfile(os.path.join(directory, block['file'])):
            raise FileNotFoundError('base DDS block is missing: %s' % block['file'])
    return manifest, directory


def write_preview(manifest, directory, path, width_px=1600):
    """Create a compact whole-wall preview from nominal, non-gutter pixels."""
    wall_width = int(manifest['width_px'])
    wall_height = int(manifest['height_px'])
    height_px = max(1, round(width_px * wall_height / wall_width))
    preview = Image.new('RGB', (width_px, height_px))
    for block in manifest['maps']['albedo']['blocks']:
        with Image.open(os.path.join(directory, block['file'])) as decoded:
            local_x = block['x_px'] - block['sample_x_px']
            local_y = block['y_px'] - block['sample_y_px']
            nominal = decoded.convert('RGB').crop((
                local_x, local_y, local_x + block['width_px'],
                local_y + block['height_px']))
        target = (round(block['x_px'] * width_px / wall_width),
                  round(block['y_px'] * height_px / wall_height),
                  round((block['x_px'] + block['width_px']) * width_px / wall_width),
                  round((block['y_px'] + block['height_px']) * height_px / wall_height))
        preview.paste(nominal.resize((target[2] - target[0], target[3] - target[1]),
                                    Image.Resampling.LANCZOS), target[:2])
    preview.save(path, optimize=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base-manifest', required=True,
                        help='immutable concrete-wall manifest with DDS gutters')
    parser.add_argument('--output-dir', required=True,
                        help='new directory for diagnostic DDS blocks')
    parser.add_argument('--seed', type=int, default=20260827,
                        help='feature-layout seed written into the manifest')
    parser.add_argument('--decal-atlas', default=DEFAULT_DECAL_ATLAS,
                        help='transparent crack and graffiti atlas')
    parser.add_argument('--jobs', type=int, default=4,
                        help='parallel DDS encoders for affected blocks')
    parser.add_argument('--preview', default='',
                        help='optional PNG path for a whole-wall preview')
    parser.add_argument('--preview-width-px', type=int, default=1600)
    arguments = parser.parse_args()
    if arguments.preview_width_px < 200:
        parser.error('--preview-width-px must be at least 200')
    if arguments.jobs < 1:
        parser.error('--jobs must be positive')
    decal_atlas_path = os.path.abspath(arguments.decal_atlas)
    if not os.path.isfile(decal_atlas_path):
        parser.error('--decal-atlas does not exist: %s' % decal_atlas_path)
    if os.path.exists(arguments.output_dir):
        parser.error('--output-dir must not already exist: %s' % arguments.output_dir)

    base_path = os.path.abspath(arguments.base_manifest)
    manifest, base_dir = _validate_base_manifest(base_path)
    scale = float(manifest['scale_m_per_px'])
    origin = [float(value) for value in manifest['region_origin_m']]
    region = [float(value) for value in manifest['region_m']]
    features = feature_layout(origin, region, arguments.seed)
    output_dir = os.path.abspath(arguments.output_dir)
    parent = os.path.dirname(output_dir)
    os.makedirs(parent, exist_ok=True)
    temporary = tempfile.mkdtemp(prefix='.diagnostic-wall-', dir=parent)
    try:
        blocks = manifest['maps']['albedo']['blocks']
        changed = [block for block in blocks
                   if _block_has_feature(block, features, origin, region, scale)]
        print('overlaying %d sparse diagnostic features onto %d/%d DDS blocks; '
              'linking the unchanged base blocks' %
              (len(features), len(changed), len(blocks)), flush=True)
        tasks = []
        for block in blocks:
            source = os.path.join(base_dir, block['file'])
            written = os.path.join(temporary, block['file'])
            if block not in changed:
                # The base wall is immutable.  A hard link makes the new
                # manifest self-contained without duplicating nearly a GiB of
                # identical BC1 data; the writer never opens such a file for
                # writing, so this cannot change the baseline.
                os.link(source, written)
                block['sha256'] = sha256(written)
                continue
            tasks.append((source, written, block, features, origin, region, scale,
                          decal_atlas_path))

        digests = {}
        with ProcessPoolExecutor(max_workers=arguments.jobs) as executor:
            for index, (name, digest) in enumerate(executor.map(
                    _encode_modified_block, tasks), start=1):
                digests[name] = digest
                if index == 1 or index == len(tasks) or index % 20 == 0:
                    print('  encoded %d/%d modified blocks' %
                          (index, len(tasks)), flush=True)
        for block in changed:
            block['sha256'] = digests[block['file']]

        output = copy.deepcopy(manifest)
        output['recorded_utc'] = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
        output['diagnostic_wall'] = {
            'profile': PROFILE_NAME,
            'profile_version': PROFILE_VERSION,
            'seed': arguments.seed,
            'base_manifest': os.path.relpath(base_path, parent),
            'base_manifest_sha256': sha256(base_path),
            'decal_atlas': os.path.relpath(decal_atlas_path, parent),
            'decal_atlas_sha256': sha256(decal_atlas_path),
            'crack_rendering': 'source hairline alpha recoloured charcoal',
            'crack_source_luminance_cutoff': CRACK_SOURCE_LUMINANCE_CUTOFF,
            'crack_source_transition': CRACK_SOURCE_TRANSITION,
            'crack_source_blur_px': CRACK_SOURCE_BLUR_PX,
            'features': features,
            'feature_counts': {
                kind: sum(feature['kind'] == kind for feature in features)
                for kind in sorted({feature['kind'] for feature in features})},
            'invariant': 'Sparse details are visual ground truth only; they do not alter wall collision, friction, or control parameters.',
        }
        output_path = os.path.join(temporary, 'wall_texture.json')
        with open(output_path, 'w', encoding='utf-8') as handle:
            json.dump(output, handle, indent=2)
            handle.write('\n')
        os.replace(temporary, output_dir)
        temporary = None
        if arguments.preview:
            preview_path = os.path.abspath(arguments.preview)
            os.makedirs(os.path.dirname(preview_path), exist_ok=True)
            write_preview(output, output_dir, preview_path, arguments.preview_width_px)
            print('preview %s' % preview_path)
        print('manifest %s' % os.path.join(output_dir, 'wall_texture.json'))
    finally:
        if temporary is not None:
            shutil.rmtree(temporary, ignore_errors=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
