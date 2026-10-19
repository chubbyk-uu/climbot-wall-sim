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

"""Paired rotation sharpness at one physical scale, before and after BC1.

Continuous-angle probes resample once directly from native source pixels.
Metrics use the same inscribed physical source disk for all angles, not
different random crops. This is an optional asset probe, not rendered approval
to turn on continuous rotations in production baking.
"""

import argparse
import json
from pathlib import Path
import tempfile

from audit_wall_randomization import digest
from bake_wall_texture import load_source
from bc1 import write_dds_bc1
import cv2
import numpy as np
from PIL import Image
from wall_texture_randomization import orient


def metrics(rgb):
    grey = cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2GRAY).astype(np.float32)
    side = grey.shape[0]
    y, x = np.mgrid[:side, :side]
    radius = np.hypot(x - (side - 1) / 2, y - (side - 1) / 2)
    disk = radius < side * .4
    gx = cv2.Sobel(grey, cv2.CV_32F, 1, 0)
    gy = cv2.Sobel(grey, cv2.CV_32F, 0, 1)
    gradient = float(np.sqrt(np.mean((gx * gx + gy * gy)[disk])))
    taper = np.clip((side * .4 - radius) / (side * .04), 0, 1)
    spectrum = np.fft.fft2((grey - grey[disk].mean()) * taper)
    frequency = np.hypot(np.fft.fftfreq(side)[:, None], np.fft.fftfreq(side)[None, :])
    band = (frequency >= .25) & (frequency <= .45)
    energy = float(np.mean(np.abs(spectrum[band]) ** 2))
    return {'gradient_rms': gradient, 'high_frequency_energy': energy}


def run(source_path, size_m, scale_mm, patch, output):
    if output.exists():
        raise ValueError('rotation output must be new')
    cv2.setNumThreads(1)
    native = np.asarray(Image.open(source_path).convert('RGB'))
    side = round(size_m * 1000 / scale_mm)
    source = load_source(source_path, side)
    start = (side - patch) // 2
    d4_base = source[start:start + patch, start:start + patch]
    ratio = native.shape[0] / side
    yy, xx = np.mgrid[:patch, :patch].astype(np.float32)
    xx, yy = (xx - (patch - 1) / 2) * ratio, (yy - (patch - 1) / 2) * ratio
    centre = (native.shape[0] - 1) / 2
    rows = []
    references = {}
    with tempfile.TemporaryDirectory(prefix='climbot_rotation_') as directory:
        cases = [('d4', k, orient(d4_base, k)) for k in range(8)]
        for angle in (0, 17, 37, 83):
            theta = np.deg2rad(angle)
            map_x = centre + np.cos(theta) * xx - np.sin(theta) * yy
            map_y = centre + np.sin(theta) * xx + np.cos(theta) * yy
            if min(float(map_x.min()), float(map_y.min())) < 4 or max(
                    float(map_x.max()), float(map_y.max())) >= native.shape[0] - 4:
                raise ValueError('rotated footprint lacks valid native Lanczos support')
            cases.append(('continuous_probe', angle, cv2.remap(
                native, map_x, map_y, cv2.INTER_LANCZOS4, borderMode=cv2.BORDER_CONSTANT)))
        for mode, transform, image in cases:
            path = Path(directory) / 'pair.dds'
            write_dds_bc1(path, np.ascontiguousarray(image))
            with Image.open(path) as encoded:
                compressed = np.asarray(encoded.convert('RGB'))
            row = {'mode': mode, 'transform': transform,
                   'uncompressed': metrics(image), 'compressed': metrics(compressed)}
            if transform == 0:
                references[mode] = row
            ratios = []
            for stage in ('uncompressed', 'compressed'):
                row[stage + '_ratios'] = {
                    key: value / references[mode][stage][key] for key, value in row[stage].items()}
                ratios.extend(row[stage + '_ratios'].values())
            row['passed_paired_tolerance'] = all(.9 <= v <= 1.1 for v in ratios)
            rows.append(row)
    result = {'source_sha256': digest(source_path), 'producer_sha256': digest(__file__),
              'native_dimensions_px': list(native.shape[1::-1]), 'source_size_m_declared': size_m,
              'target_mm_per_px': scale_mm, 'patch_px': patch,
              'physical_source_region': ('same central inscribed disk; '
                                         'radius=0.4*patch at target scale'),
              'd4_interpolation': 'Pillow LANCZOS to physical scale, then lossless D4',
              'continuous_interpolation': 'OpenCV LANCZOS4, single native-to-output remap',
              'frequency_band_cycles_per_px': [.25, .45], 'allowed_ratio': [.9, 1.1],
              'cases': rows, 'passed': all(row['passed_paired_tolerance'] for row in rows),
              'scope': ('asset-only probe; continuous rotations are not enabled '
                        'by the baker or rendered-accepted')}
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    failures = [[r['mode'], r['transform']] for r in rows if not r['passed_paired_tolerance']]
    print(json.dumps({'passed': result['passed'], 'failed_cases': failures,
                      'output': str(output)}))
    return 0 if result['passed'] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-image', type=Path, required=True)
    parser.add_argument('--source-size-m', type=float, default=2.5)
    parser.add_argument('--scale-mm-per-px', type=float, default=.25)
    parser.add_argument('--patch-px', type=int, default=1280)
    parser.add_argument('--output', type=Path, required=True)
    a = parser.parse_args()
    return run(a.source_image, a.source_size_m, a.scale_mm_per_px, a.patch_px, a.output)


if __name__ == '__main__':
    raise SystemExit(main())
