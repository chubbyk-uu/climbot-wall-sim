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

"""Bake one non-repeating wall texture from a tileable concrete sample.

The wall is 80 m2 and the sample covers 6.25 m2, so the sample has to be
stretched over roughly thirteen times its own area. Doing that by tiling is the
one thing that must not happen: a stitcher matches local appearance, and a
pattern that repeats gives it several equally good answers, so it picks a wrong
one confidently rather than failing visibly. The pose prior keeps the primary
matching path out of that trap, but the global fallback has no prior by
definition, and the fallback is exactly the path that runs when the prior is
wrong.

So the sample is quilted rather than tiled (Efros and Freeman 2001): patches
are drawn from anywhere in the source, chosen to agree with what they overlap,
and joined along the minimum-error path through that overlap. Cuts are hard,
never blended, because a blend invents texture that was never photographed.

Only the colour map is baked. A normal map at this size would cost as much
video memory as the colour map and carries nothing a stitch can match, and the
same is true of roughness. Drawn cracks were considered and left out: the
texture libraries have none worth using, but drawing them is a separate claim
about what the wall looks like and it belongs in its own change.

The bake is seeded and its inputs are recorded, because the baked image is not
only scenery: it is the reference a stitched mosaic gets compared against, and
a reference nobody can reproduce cannot support a measurement. It is also not
committed - 200 MB of generated blocks - so this script, the checksum in
tools/fetch_wall_texture.sh and the recorded seed are the whole of what the
repository carries.
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

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bc1 import write_dds_bc1  # noqa: E402  (needs the path set above)

Image.MAX_IMAGE_PIXELS = None

#: BC1 stores a 4x4 block in 8 bytes. Mip levels add a third again, and gz runs
#: the server and the GUI as two processes that each build their own scene.
BC1_BYTES_PER_TEXEL = 0.5
MIP_OVERHEAD = 1.33
RENDERING_PROCESSES = 2

#: BC1 addresses whole 4x4 blocks, so every stored edge has to be a multiple of
#: four. Padding at write time would work but would shift the last block's
#: texels off the metres they represent, so the bake is sized to fit instead.
BLOCK_ALIGNMENT = 4


def sha256(path):
    """Digest a file without holding it in memory."""
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b''):
            digest.update(chunk)
    return digest.hexdigest()


def git_commit():
    """Describe the tree this bake was produced from, or None without git."""
    try:
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        return subprocess.run(
            ['git', '-C', root, 'rev-parse', 'HEAD'], check=True,
            capture_output=True, text=True, timeout=5.0).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return None


def described_wall():
    """Read the wall the whole project agrees on, straight from the source."""
    # Not through ament: this runs before and independently of a built
    # workspace, and it must not be able to bake for a different wall than the
    # one climbot_description describes.
    import yaml
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = os.path.join(
        root, 'src', 'climbot_description', 'config', 'wall.yaml')
    with open(path, encoding='utf-8') as handle:
        surface = yaml.safe_load(handle)['wall']['surface']
    return float(surface['width_m']), float(surface['height_m'])


def align_up(value, multiple):
    """Round a pixel count up to the next whole block."""
    return int(math.ceil(value / multiple) * multiple)


def load_source(path, side):
    """Read the source map and resample it to the baked pixel scale."""
    image = Image.open(path).convert('RGB')
    if image.size != (side, side):
        image = image.resize((side, side), Image.LANCZOS)
    return np.asarray(image)


def minimum_cut(error):
    """Return, per row, the column where the seam should cross.

    error is the squared difference across the overlap strip. The seam is the
    cheapest top-to-bottom path through it, which is what keeps a join from
    showing: a straight edge cuts through whatever happens to be there, while
    this one goes around it.
    """
    rows, columns = error.shape
    total = error.astype(np.float64).copy()
    back = np.zeros((rows, columns), dtype=np.int32)
    for row in range(1, rows):
        previous = total[row - 1]
        left = np.concatenate(([np.inf], previous[:-1]))
        right = np.concatenate((previous[1:], [np.inf]))
        stacked = np.vstack((left, previous, right))
        choice = np.argmin(stacked, axis=0)
        back[row] = choice - 1
        total[row] += stacked[choice, np.arange(columns)]
    seam = np.empty(rows, dtype=np.int32)
    seam[-1] = int(np.argmin(total[-1]))
    for row in range(rows - 2, -1, -1):
        seam[row] = seam[row + 1] + back[row + 1, seam[row + 1]]
    return seam


def choose_patch(source_grey, canvas_grey, has_left, has_top, patch, overlap,
                 rng, candidates, tolerance):
    """Pick a source patch that agrees with the neighbours already placed."""
    limit = source_grey.shape[0] - patch
    ys = rng.integers(0, limit + 1, size=candidates)
    xs = rng.integers(0, limit + 1, size=candidates)
    if not has_left and not has_top:
        return int(ys[0]), int(xs[0])

    errors = np.empty(candidates, dtype=np.float64)
    for index in range(candidates):
        sy, sx = int(ys[index]), int(xs[index])
        block = source_grey[sy:sy + patch, sx:sx + patch]
        error = 0.0
        if has_left:
            difference = block[:, :overlap] - canvas_grey[:, :overlap]
            error += float(np.einsum('ij,ij->', difference, difference))
        if has_top:
            difference = block[:overlap, :] - canvas_grey[:overlap, :]
            error += float(np.einsum('ij,ij->', difference, difference))
        errors[index] = error
    best = errors.min()
    # A tolerance band rather than the single best match: the best match for a
    # nearly featureless overlap is close to arbitrary, and always taking it
    # pulls the whole bake toward a handful of source neighbourhoods, which is
    # the repetition this is here to avoid.
    allowed = np.flatnonzero(errors <= best * (1.0 + tolerance) + 1e-9)
    pick = int(rng.choice(allowed))
    return int(ys[pick]), int(xs[pick])


def patch_mask(source_grey, canvas_grey, sy, sx, patch, overlap,
               has_left, has_top):
    """Build the hard mask that says which pixels the new patch owns."""
    mask = np.ones((patch, patch), dtype=bool)
    block = source_grey[sy:sy + patch, sx:sx + patch]
    if has_left:
        difference = block[:, :overlap] - canvas_grey[:, :overlap]
        seam = minimum_cut(difference * difference)
        columns = np.arange(overlap)[None, :]
        mask[:, :overlap] &= columns >= seam[:, None]
    if has_top:
        difference = block[:overlap, :] - canvas_grey[:overlap, :]
        seam = minimum_cut((difference * difference).T)
        rows = np.arange(overlap)[:, None]
        mask[:overlap, :] &= rows >= seam[None, :]
    return mask


def quilt_layout(source_grey, height, width, patch, overlap, seed,
                 candidates, tolerance, log):
    """Lay the whole output out once, on a grey copy, and record the layout."""
    rng = np.random.default_rng(seed)
    step = patch - overlap
    rows = max(1, math.ceil((height - overlap) / step))
    columns = max(1, math.ceil((width - overlap) / step))
    canvas = np.zeros((rows * step + overlap, columns * step + overlap),
                      dtype=np.float32)
    placements = []
    started = time.monotonic()
    for row in range(rows):
        for column in range(columns):
            top, left = row * step, column * step
            window = canvas[top:top + patch, left:left + patch]
            has_left, has_top = column > 0, row > 0
            sy, sx = choose_patch(
                source_grey, window, has_left, has_top, patch, overlap,
                rng, candidates, tolerance)
            mask = patch_mask(
                source_grey, window, sy, sx, patch, overlap, has_left, has_top)
            window[mask] = source_grey[sy:sy + patch, sx:sx + patch][mask]
            placements.append((top, left, sy, sx, np.packbits(mask)))
        log('  quilted row %d/%d  (%.0f s)'
            % (row + 1, rows, time.monotonic() - started))
    return placements, canvas.shape, patch


def apply_layout(source, placements, canvas_shape, patch, height, width):
    """Replay the layout onto the colour map."""
    canvas = np.zeros(canvas_shape + (3,), dtype=np.uint8)
    for top, left, sy, sx, packed in placements:
        mask = np.unpackbits(packed, count=patch * patch).reshape(patch, patch)
        mask = mask.astype(bool)
        block = source[sy:sy + patch, sx:sx + patch]
        window = canvas[top:top + patch, left:left + patch]
        window[mask] = block[mask]
    return canvas[:height, :width]


def write_blocks(image, directory, name, block, log):
    """Cut the baked map into GPU-sized blocks and write them as BC1 DDS."""
    height, width = image.shape[:2]
    rows = math.ceil(height / block)
    columns = math.ceil(width / block)
    written = []
    for row in range(rows):
        for column in range(columns):
            y0, x0 = row * block, column * block
            y1, x1 = min(y0 + block, height), min(x0 + block, width)
            filename = '%s_r%02d_c%02d.dds' % (name, row, column)
            stored = write_dds_bc1(
                os.path.join(directory, filename),
                np.ascontiguousarray(image[y0:y1, x0:x1]))
            written.append({
                'file': filename, 'row': row, 'column': column,
                'x_px': x0, 'y_px': y0,
                'width_px': x1 - x0, 'height_px': y1 - y0,
                'bytes': stored})
        log('  wrote block row %d/%d' % (row + 1, rows))
    return rows, columns, written


def video_memory_gb(width, height):
    """Estimate what the finished bake will cost on the GPU.

    This number is the whole reason the tool refuses some bakes: exhausting
    video memory under WSLg does not fail the run, it takes the session down
    with it. That is not hypothetical - an earlier RGBA attempt at this size
    asked for 5.31 GB and did exactly that.
    """
    return (width * height * BC1_BYTES_PER_TEXEL * MIP_OVERHEAD
            * RENDERING_PROCESSES / 1e9)


def host_memory_gb(width, height, source_side):
    """Estimate the peak the bake itself needs, which is not the GPU's.

    The float32 grey canvas and the uint8 colour canvas are both held whole,
    because the quilt is laid out globally: a patch is chosen to agree with
    neighbours that may have been placed thousands of patches ago.
    """
    return (width * height * (4 + 3) + source_side * source_side * (4 + 3)) / 1e9


def main():
    """Bake the wall texture and record what it was baked from."""
    wall_width, wall_height = described_wall()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-dir', required=True,
                        help='directory of maps from tools/fetch_wall_texture.sh')
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--source-size-m', type=float, required=True,
                        help='real-world extent of the source sample. Required '
                             'and deliberately not defaulted: Concrete044D '
                             'records no size, so this is a declaration about '
                             'what the wall is made of, and it decides every '
                             'measured length in the bake. The project '
                             'declares 2.5.')
    parser.add_argument('--region-m', type=float, nargs=2,
                        default=(wall_width, wall_height),
                        help='width and height of the textured patch; defaults '
                             'to the whole described wall')
    parser.add_argument('--region-origin-m', type=float, nargs=2,
                        default=(0.0, 0.0),
                        help='lower-left corner of that patch in the wall work '
                             'frame, whose origin is the wall corner')
    parser.add_argument('--scale-mm-per-px', type=float, default=0.50,
                        help='not the camera scale of 0.2604: 0.50 measured as '
                             'the optimum for stitching, with 11x the RANSAC '
                             'inliers of camera-native and 1.6x those of 0.75')
    parser.add_argument('--block-px', type=int, default=2048)
    parser.add_argument('--patch-px', type=int, default=512)
    parser.add_argument('--overlap-fraction', type=float, default=1.0 / 6.0)
    parser.add_argument('--candidates', type=int, default=96)
    parser.add_argument('--tolerance', type=float, default=0.10)
    parser.add_argument('--seed', type=int, default=20260820)
    parser.add_argument('--video-memory-budget-gb', type=float, default=3.0,
                        help='refuse to bake past this estimated video memory')
    parser.add_argument('--host-memory-budget-gb', type=float, default=8.0,
                        help='refuse to bake past this estimated peak RAM')
    parser.add_argument('--quiet', action='store_true')
    arguments = parser.parse_args()

    def log(message):
        if not arguments.quiet:
            print(message, flush=True)

    if arguments.block_px % BLOCK_ALIGNMENT:
        parser.error('--block-px must be a multiple of %d' % BLOCK_ALIGNMENT)

    scale = arguments.scale_mm_per_px / 1000.0
    # Sized up to whole BC1 blocks rather than padded at write time, so every
    # stored texel still stands for the metre it was baked at. The region grows
    # by under 2 mm, which is why region_m below is derived and not echoed.
    width = align_up(arguments.region_m[0] / scale, BLOCK_ALIGNMENT)
    height = align_up(arguments.region_m[1] / scale, BLOCK_ALIGNMENT)
    region = (width * scale, height * scale)
    patch = arguments.patch_px
    overlap = max(4, int(round(patch * arguments.overlap_fraction)))

    matches = [entry for entry in sorted(os.listdir(arguments.source_dir))
               if entry.endswith('_Color.jpg')]
    if not matches:
        parser.error('no _Color.jpg map in %s' % arguments.source_dir)
    source_path = os.path.join(arguments.source_dir, matches[0])

    native = Image.open(source_path).size[0]
    side = int(round(arguments.source_size_m * 1000.0 / arguments.scale_mm_per_px))
    if side < patch:
        parser.error('source is smaller than one patch at this scale')
    log('source %dpx over %.2f m (declared) -> %dpx at %.4f mm/px'
        % (native, arguments.source_size_m, side, arguments.scale_mm_per_px))

    video = video_memory_gb(width, height)
    host = host_memory_gb(width, height, side)
    log('estimated video memory %.2f GB (server and GUI together)' % video)
    log('estimated peak host memory %.2f GB' % host)
    if video > arguments.video_memory_budget_gb:
        parser.error(
            'this bake needs about %.2f GB of video memory, over the %.2f GB '
            'budget. Coarsen --scale-mm-per-px or shrink --region-m. Do not '
            'raise the budget without checking what the GPU already has free: '
            'running it out under WSLg ends the session, it does not just drop '
            'frames.' % (video, arguments.video_memory_budget_gb))
    if host > arguments.host_memory_budget_gb:
        parser.error(
            'this bake needs about %.2f GB of RAM, over the %.2f GB budget. '
            'The quilt is laid out globally, so the canvases cannot be '
            'streamed; coarsen --scale-mm-per-px or shrink --region-m.'
            % (host, arguments.host_memory_budget_gb))
    log('baking %.2f x %.2f m  =  %d x %d px  (%.0f Mpx), reuse %.1f x'
        % (region[0], region[1], width, height, width * height / 1e6,
           region[0] * region[1] / arguments.source_size_m ** 2))

    source = load_source(source_path, side)
    grey = source.astype(np.float32).mean(axis=2)
    log('laying out patches of %d px with %d px overlap' % (patch, overlap))
    placements, canvas_shape, patch = quilt_layout(
        grey, height, width, patch, overlap, arguments.seed,
        arguments.candidates, arguments.tolerance, log)
    del grey

    os.makedirs(arguments.output_dir, exist_ok=True)
    baked = apply_layout(source, placements, canvas_shape, patch, height, width)
    del source
    rows, columns, files = write_blocks(
        baked, arguments.output_dir, 'albedo', arguments.block_px, log)
    del baked
    log('wrote albedo: %d x %d blocks, %.0f MiB'
        % (rows, columns, sum(entry['bytes'] for entry in files) / (1 << 20)))

    manifest = {
        'recorded_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        'git_commit': git_commit(),
        'source_asset': 'ambientCG Concrete044D 8K-JPG (CC0)',
        'source_file': os.path.basename(source_path),
        'source_sha256': sha256(source_path),
        # A declaration, not a property of the asset. The library records no
        # size for Concrete044D, and every length in this manifest is derived
        # from this one number, so it is written down as what it is.
        'source_size_m': arguments.source_size_m,
        'source_size_is_declared': True,
        # Everything a wall coordinate needs to become a pixel. The bake is the
        # reference a mosaic is scored against, so this mapping is the
        # measurement's own datum and belongs next to the pixels, not in a
        # launch file that may have moved on.
        'scale_m_per_px': scale,
        'region_m': list(region),
        'region_requested_m': list(arguments.region_m),
        # Where the baked pixels sit on the wall. A mosaic is scored by
        # projecting it into this frame, so without the origin the bake is a
        # picture of a wall rather than a measurement of one, and every pose
        # label attached to a photograph would have nothing to be checked
        # against.
        'region_origin_m': list(arguments.region_origin_m),
        # Written as the conversion itself rather than as a named convention.
        # Pixel rows run down from the top and wall coordinates run up from the
        # bottom, so any wording short of the formula leaves the reader to
        # guess which of the two the origin belongs to, and a mosaic scored
        # against a vertically mirrored reference fails in a way that looks
        # like a control problem.
        'wall_x_m_of_px': 'region_origin_m[0] + x_px * scale_m_per_px',
        'wall_y_m_of_px':
            'region_origin_m[1] + region_m[1] - y_px * scale_m_per_px',
        'width_px': width, 'height_px': height,
        'block_px': arguments.block_px,
        'format': 'BC1 DDS with baked mip chain',
        'estimated_video_memory_gb': round(video, 3),
        'quilt': {
            'patch_px': patch, 'overlap_px': overlap,
            'candidates': arguments.candidates,
            'tolerance': arguments.tolerance, 'seed': arguments.seed,
            'placements': len(placements)},
        'maps': {
            'albedo': {
                'block_rows': rows, 'block_columns': columns, 'blocks': files},
        },
    }
    path = os.path.join(arguments.output_dir, 'wall_texture.json')
    with open(path, 'w', encoding='utf-8') as handle:
        json.dump(manifest, handle, indent=2)
        handle.write('\n')
    log('manifest %s' % path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
