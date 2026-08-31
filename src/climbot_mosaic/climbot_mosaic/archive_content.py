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

"""
Publish what an archive's images actually look like, as a checkable product.

Two whole collections once passed every acquisition guard while every frame
was ruined by z-fighting, because the guards count frames and check timing
and nothing looked at the pixels. The comparison that caught it lived in a
throwaway script and a sentence in a document, which is the same kind of
unciteable claim this chain exists to remove. So the content statistics are
a published, hashed product with their own provenance, and the comparison
against a reference collection is a gate with stated tolerances rather than
a number someone reads.

Global scalars alone would not have been enough on their own terms: mean and
variance happened to move enormously under z-fighting, but a subtler spatial
artefact need not move them at all. The fingerprint therefore also carries
coarse spatial structure -- a block downsample and the row and column mean
profiles -- because that is where banding, striping and vignetting live.
"""

from __future__ import annotations

import math
from pathlib import Path
import tempfile
from typing import Any
from uuid import uuid4

from climbot_common.atomic import write_json
from climbot_mosaic.stage_provenance import artifact, write_stage_provenance
import cv2
import numpy as np

#: Block downsample per axis; coarse on purpose, so it survives the millimetre
#: pose differences between two runs of the same plan and still shows layout.
BLOCK_GRID = 16
#: Bins in the row and column mean profiles. Periodic rendering artefacts show
#: here long before they move a global scalar.
PROFILE_BINS = 32
CONTENT_FORMAT_VERSION = 2


class ArchiveContentError(ValueError):
    """An archive's images cannot produce a trustworthy content record."""


def _resize_mean(image: np.ndarray, width: int, height: int) -> np.ndarray:
    """Average-pool to a fixed grid, independent of the source resolution."""
    return cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA).astype(np.float32)


def frame_fingerprint(image: np.ndarray) -> dict[str, Any]:
    """Return the scalar and coarse-spatial description of one frame."""
    if image.ndim != 2 or image.dtype != np.uint8:
        raise ArchiveContentError('archive frames must be mono8.')
    values = image.astype(np.float32)
    return {
        'mean': float(values.mean()),
        'std': float(values.std()),
        'laplacian_variance': float(cv2.Laplacian(image, cv2.CV_64F).var()),
        'block': _resize_mean(image, BLOCK_GRID, BLOCK_GRID).ravel(),
        'column_profile': _resize_mean(image, PROFILE_BINS, 1).ravel(),
        'row_profile': _resize_mean(image, 1, PROFILE_BINS).ravel(),
    }


def _distribution(values: np.ndarray) -> dict[str, float]:
    return {
        'minimum': float(values.min()), 'median': float(np.median(values)),
        'mean': float(values.mean()), 'p95': float(np.percentile(values, 95)),
        'maximum': float(values.max()),
    }


def _raw_images(run: Path) -> list[Path]:
    images = sorted((run / 'images' / 'raw').glob('*.png'))
    if not images:
        raise ArchiveContentError(f'archive run has no raw images: {run}')
    return images


def measure_run(run: Path) -> dict[str, Any]:
    """Read every raw frame of one archive run and summarize what it contains."""
    scalars: dict[str, list[float]] = {'mean': [], 'std': [], 'laplacian_variance': []}
    blocks, columns, rows = [], [], []
    for path in _raw_images(run):
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ArchiveContentError(f'cannot decode archive frame: {path.name}')
        fingerprint = frame_fingerprint(image)
        for key in scalars:
            scalars[key].append(fingerprint[key])
        blocks.append(fingerprint['block'])
        columns.append(fingerprint['column_profile'])
        rows.append(fingerprint['row_profile'])
    return {
        'frame_count': len(blocks),
        'scalars': {key: np.asarray(value, np.float32) for key, value in scalars.items()},
        'block': np.asarray(blocks, np.float32),
        'column_profile': np.asarray(columns, np.float32),
        'row_profile': np.asarray(rows, np.float32),
    }


def _normalized_mean_profile(values: np.ndarray) -> np.ndarray:
    """Average a per-frame profile over the run and divide out overall brightness."""
    profile = values.mean(axis=0)
    total = float(profile.mean())
    if not math.isfinite(total) or total <= 0.0:
        raise ArchiveContentError('a run profile has no positive mean brightness.')
    return profile / total


def _per_frame_deviation(observed: dict[str, Any], reference: dict[str, Any]) -> np.ndarray:
    """
    Return each frame's largest disagreement with the reference frame of the same index.

    Run-level statistics cannot see a single ruined frame: one black frame moves
    a 680-frame median by nothing and a mean profile by a seven-hundredth. Two
    collections of the same plan expose frame i at the same planned pose, so the
    frames correspond by index and can be compared one to one. Deviations are
    expressed as a fraction of the reference run's typical brightness, so a black
    frame scores about one and a swapped pair scores whatever the wall differs by.
    """
    scale = float(np.median(reference['scalars']['mean']))
    if not math.isfinite(scale) or scale <= 0.0:
        raise ArchiveContentError('reference run has no positive typical brightness.')
    count = min(len(observed['block']), len(reference['block']))
    if count == 0:
        raise ArchiveContentError('no frames to compare.')
    if len(observed['block']) != len(reference['block']):
        # A different frame count is itself a disagreement, not something to
        # silently truncate: the plan is frozen, so the counts must match.
        raise ArchiveContentError(
            'observed and reference runs have different frame counts: '
            f"{len(observed['block'])} and {len(reference['block'])}.")
    worst = np.zeros(count, np.float64)
    for key in ('block', 'column_profile', 'row_profile'):
        difference = np.abs(observed[key][:count] - reference[key][:count]).max(axis=1)
        worst = np.maximum(worst, difference / scale)
    for key in ('mean', 'std'):
        difference = np.abs(observed['scalars'][key][:count] -
                            reference['scalars'][key][:count])
        worst = np.maximum(worst, difference / scale)
    return worst


def compare(observed: dict[str, Any], reference: dict[str, Any],
            scalar_tolerance: float, profile_tolerance: float,
            frame_tolerance: float = 0.30,
            frame_p95_tolerance: float = 0.10) -> dict[str, Any]:
    """
    Gate one run's content against a reference run of the same plan.

    Both ratios and profile deviations are reported whether or not they pass,
    because a gate that only records its verdict cannot be re-judged against a
    tolerance someone later decides was wrong.
    """
    if not 0.0 < scalar_tolerance < 1.0 or not 0.0 < profile_tolerance < 1.0 \
            or not 0.0 < frame_tolerance < 1.0 or not 0.0 < frame_p95_tolerance < 1.0:
        raise ArchiveContentError('content tolerances must lie in (0, 1).')
    ratios, failures = {}, []
    for key, values in observed['scalars'].items():
        other = reference['scalars'][key]
        divisor = float(np.median(other))
        if not math.isfinite(divisor) or divisor == 0.0:
            raise ArchiveContentError(f'reference {key} has no usable median.')
        ratio = float(np.median(values)) / divisor
        ratios[key] = ratio
        if abs(ratio - 1.0) > scalar_tolerance:
            failures.append(f'{key} median ratio {ratio:.4f}')
    deviations = {}
    for key in ('column_profile', 'row_profile', 'block'):
        deviation = float(np.abs(_normalized_mean_profile(observed[key]) -
                                 _normalized_mean_profile(reference[key])).max())
        deviations[key] = deviation
        # The block grid is reported but not gated. Averaged over a run the row
        # and column profiles are insensitive to the millimetre pose differences
        # between two collections of the same plan; a 34 mm block is not, so a
        # tolerance tight enough to be useful there would fail on pose noise.
        if key != 'block' and deviation > profile_tolerance:
            failures.append(f'{key} max deviation {deviation:.4f}')
    frames = _per_frame_deviation(observed, reference)
    frame_stats = {
        'median': float(np.median(frames)), 'p95': float(np.percentile(frames, 95)),
        'maximum': float(frames.max()),
        'worst_frame_index': int(np.argmax(frames)),
        'frames_over_tolerance': int((frames > frame_tolerance).sum()),
    }
    # Two gates, because they catch different things. The maximum refuses one
    # ruined frame; the p95 refuses a population of degraded frames that each
    # stay under it. Both come from measurement: across 5340 frames of eight
    # matched collections the per-frame deviation held a median of 0.027 and a
    # p95 of 0.055 in every run, with a worst single frame of 0.164, so these
    # sit about 1.8 times above what agreement actually looks like. A black
    # frame scores near one, six times the maximum gate.
    if frame_stats['maximum'] > frame_tolerance:
        failures.append('frame %d deviates %.4f'
                        % (frame_stats['worst_frame_index'], frame_stats['maximum']))
    if frame_stats['p95'] > frame_p95_tolerance:
        failures.append('per-frame deviation p95 %.4f' % frame_stats['p95'])
    return {
        'scalar_median_ratio': ratios,
        'normalized_profile_max_deviation': deviations,
        'gated_profiles': ['column_profile', 'row_profile'],
        'per_frame_deviation': frame_stats,
        'scalar_tolerance': scalar_tolerance,
        'profile_tolerance': profile_tolerance,
        'frame_tolerance': frame_tolerance,
        'frame_p95_tolerance': frame_p95_tolerance,
        'passed': not failures,
        'failures': failures,
    }


def summarize_archive_content(runs: list[Path], output_dir: Path,
                              reference_run: Path | None = None,
                              scalar_tolerance: float = 0.05,
                              profile_tolerance: float = 0.02,
                              frame_tolerance: float = 0.30,
                              frame_p95_tolerance: float = 0.10) -> dict[str, Any]:
    """Write one published, hashed content record for the given archive runs."""
    runs = [run.resolve() for run in runs]
    output_dir = output_dir.resolve()
    if output_dir.exists() or not output_dir.parent.is_dir():
        raise ArchiveContentError('output directory must be new with an existing parent.')
    if not runs:
        raise ArchiveContentError('at least one archive run is required.')
    measured = {run.name: measure_run(run) for run in runs}
    reference = measure_run(reference_run.resolve()) if reference_run else None
    temporary = Path(tempfile.mkdtemp(prefix=f'.{output_dir.name}.tmp-{uuid4().hex}-',
                                      dir=output_dir.parent))
    try:
        arrays = {}
        for name, values in measured.items():
            for key in ('block', 'column_profile', 'row_profile'):
                arrays[f'{name}.{key}'] = values[key]
            for key, series in values['scalars'].items():
                arrays[f'{name}.{key}'] = series
        np.savez_compressed(temporary / 'archive_content.npz', **arrays)
        summary: dict[str, Any] = {
            'archive_content_format_version': CONTENT_FORMAT_VERSION,
            'purpose': ('Published record of the content of an archive\u2019s raw frames, '
                        'so a claim that two collections match is checkable, not asserted.'),
            'block_grid': BLOCK_GRID,
            'profile_bins': PROFILE_BINS,
            'runs': {},
        }
        for name, values in measured.items():
            entry: dict[str, Any] = {
                'frame_count': values['frame_count'],
                'scalars': {key: _distribution(series)
                            for key, series in values['scalars'].items()},
            }
            if reference is not None:
                entry['against_reference'] = compare(
                    values, reference, scalar_tolerance, profile_tolerance,
                    frame_tolerance, frame_p95_tolerance)
            summary['runs'][name] = entry
        if reference_run is not None:
            summary['reference_run'] = reference_run.resolve().name
            summary['all_runs_match_reference'] = all(
                entry['against_reference']['passed'] for entry in summary['runs'].values())
        write_json(temporary / 'archive_content_summary.json', summary)
        write_stage_provenance(
            temporary, 'archive_content',
            {'block_grid': BLOCK_GRID, 'profile_bins': PROFILE_BINS,
             'scalar_tolerance': scalar_tolerance, 'profile_tolerance': profile_tolerance,
             'frame_tolerance': frame_tolerance,
             'frame_p95_tolerance': frame_p95_tolerance},
            # Named, not positional: a consumer has to be able to say which run
            # a hash belongs to without relying on list order lining up with a
            # dict's insertion order.
            {'archive_manifests': [{'run': run.name, 'manifest': artifact(run / 'manifest.json')}
                                   for run in runs],
             'reference_manifest': (
                 {'run': reference_run.resolve().name,
                  'manifest': artifact(reference_run.resolve() / 'manifest.json')}
                 if reference_run else None)},
            ('archive_content_summary.json', 'archive_content.npz'))
        temporary.replace(output_dir)
        return summary
    except Exception:
        import shutil
        shutil.rmtree(temporary, ignore_errors=True)
        raise
