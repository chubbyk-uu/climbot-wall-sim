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
# See the License for the specific language governing permissions and
# limitations under the License.

"""Derive a coarser BC1 wall bake from exactly the same rendered content.

This is an evaluation tool, not the normal wall-generation path.  Independent
quilts contain different concrete features and therefore confound a texture
resolution comparison.  Resampling one canonical fine bake holds content,
seams, seed, camera locations, and BC1 rendering constant while changing only
the texel scale.
"""

import argparse
import hashlib
import json
import math
import os
import subprocess
import sys
import time

import numpy as np
from PIL import Image

import bake_wall_texture
from bc1 import decode_bc1

Image.MAX_IMAGE_PIXELS = None


def sha256(path):
    """Return the digest of the canonical manifest."""
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b''):
            digest.update(chunk)
    return digest.hexdigest()


def git_commit():
    """Describe the source revision that performed the derivation."""
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return subprocess.run(
        ['git', '-C', root, 'rev-parse', 'HEAD'], check=True,
        capture_output=True, text=True).stdout.strip()


def decode_top_level(path, expected_width, expected_height):
    """Decode only mip zero from one BC1 DDS block."""
    with open(path, 'rb') as handle:
        data = handle.read()
    if data[:4] != b'DDS ' or len(data) < 128:
        raise ValueError('%s is not a DDS file' % path)
    header = np.frombuffer(data[4:128], dtype='<u4')
    height, width = int(header[2]), int(header[3])
    if (width, height) != (expected_width, expected_height):
        raise ValueError(
            '%s is %d x %d, expected %d x %d' %
            (path, width, height, expected_width, expected_height))
    padded_width = max(math.ceil(width / 4) * 4, 4)
    padded_height = max(math.ceil(height / 4) * 4, 4)
    byte_count = padded_width * padded_height // 2
    decoded = decode_bc1(data[128:128 + byte_count],
                         padded_width, padded_height)
    return decoded[:height, :width]


def reconstruct(manifest, directory):
    """Reassemble nominal, non-overlapping pixels from guttered blocks."""
    canvas = np.empty(
        (manifest['height_px'], manifest['width_px'], 3), dtype=np.uint8)
    for block in manifest['maps']['albedo']['blocks']:
        sample_x = block.get('sample_x_px', block['x_px'])
        sample_y = block.get('sample_y_px', block['y_px'])
        sample_width = block.get('sample_width_px', block['width_px'])
        sample_height = block.get('sample_height_px', block['height_px'])
        image = decode_top_level(
            os.path.join(directory, block['file']),
            sample_width, sample_height)
        crop_x = block['x_px'] - sample_x
        crop_y = block['y_px'] - sample_y
        nominal = image[
            crop_y:crop_y + block['height_px'],
            crop_x:crop_x + block['width_px']]
        x, y = block['x_px'], block['y_px']
        canvas[y:y + block['height_px'],
               x:x + block['width_px']] = nominal
    return canvas


def main():
    """Reconstruct, resize, and emit a normal wall-texture manifest."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--scale-mm-per-px', type=float, required=True)
    parser.add_argument('--block-px', type=int, default=2048)
    parser.add_argument('--gutter-px', type=int,
                        default=bake_wall_texture.DEFAULT_RENDER_GUTTER_PX)
    arguments = parser.parse_args()
    if (not math.isfinite(arguments.scale_mm_per_px) or
            arguments.scale_mm_per_px <= 0.0):
        parser.error('--scale-mm-per-px must be positive and finite')

    manifest_path = os.path.abspath(arguments.manifest)
    with open(manifest_path, encoding='utf-8') as handle:
        source = json.load(handle)
    source_directory = os.path.dirname(manifest_path)
    source_scale = float(source['scale_m_per_px'])
    target_scale = arguments.scale_mm_per_px / 1000.0
    if target_scale <= source_scale:
        parser.error('target scale must be coarser than the canonical bake')

    width = bake_wall_texture.align_up(
        source['region_m'][0] / target_scale,
        bake_wall_texture.BLOCK_ALIGNMENT)
    height = bake_wall_texture.align_up(
        source['region_m'][1] / target_scale,
        bake_wall_texture.BLOCK_ALIGNMENT)
    canonical = reconstruct(source, source_directory)
    resized = np.asarray(Image.fromarray(canonical).resize(
        (width, height), Image.Resampling.LANCZOS))
    del canonical

    os.makedirs(arguments.output_dir, exist_ok=True)
    rows, columns, files = bake_wall_texture.write_blocks(
        resized, arguments.output_dir, 'albedo', arguments.block_px,
        arguments.gutter_px, lambda message: print(message, flush=True))
    del resized
    region = [width * target_scale, height * target_scale]
    video = bake_wall_texture.video_memory_gb(
        width, height, arguments.block_px, arguments.gutter_px)
    derived = dict(source)
    derived.update({
        'recorded_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        'git_commit': git_commit(),
        'scale_m_per_px': target_scale,
        'region_m': region,
        'width_px': width, 'height_px': height,
        'block_px': arguments.block_px, 'gutter_px': arguments.gutter_px,
        'estimated_video_memory_gb': round(video, 3),
        'derived_from': {
            'purpose': 'controlled rendered-resolution comparison',
            'canonical_manifest_sha256': sha256(manifest_path),
            'canonical_scale_m_per_px': source_scale,
            'resampling': 'Pillow LANCZOS from decoded BC1 mip zero',
        },
        'maps': {'albedo': {
            'block_rows': rows, 'block_columns': columns, 'blocks': files}},
    })
    output_path = os.path.join(arguments.output_dir, 'wall_texture.json')
    with open(output_path, 'w', encoding='utf-8') as handle:
        json.dump(derived, handle, indent=2, allow_nan=False)
        handle.write('\n')
    print('manifest %s' % output_path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
