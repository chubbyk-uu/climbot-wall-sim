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

"""The content record must catch what frame counting cannot."""

import json

from climbot_mosaic.archive_content import (
    ArchiveContentError,
    compare,
    measure_run,
    summarize_archive_content,
)
import cv2
import numpy as np
import pytest

RNG = np.random.default_rng(11)


#: Two collections of the same plan photograph the same wall from poses that
#: differ by millimetres, so their frames share content and differ by noise.
#: Brightness climbs along the sweep, as it does on a real wall under a moving
#: light, so swapping two frames is a real change rather than a relabelling.
WALL = [cv2.GaussianBlur(np.clip(np.random.default_rng(5 + index).integers(
    40, 120, (240, 320)) + 22 * index, 0, 255).astype(np.uint8), (0, 0), 1.2)
    for index in range(6)]


def _run(root, name, transform=None):
    """Write a minimal archive run whose raw frames are a known pattern."""
    directory = root / name
    (directory / 'images' / 'raw').mkdir(parents=True)
    (directory / 'manifest.json').write_text(json.dumps({'run_id': name}))
    for index, wall in enumerate(WALL):
        base = np.clip(wall.astype(np.float32) + RNG.normal(0, 1.5, wall.shape),
                       0, 255).astype(np.uint8)
        cv2.imwrite(str(directory / 'images' / 'raw' / f'{index:06d}.png'),
                    base if transform is None else transform(base))
    return directory


def test_matching_collections_pass_the_content_gate(tmp_path):
    reference = _run(tmp_path, 'reference')
    observed = _run(tmp_path, 'observed')
    summary = summarize_archive_content(
        [observed], tmp_path / 'out', reference_run=reference)
    assert summary['all_runs_match_reference'] is True
    assert (tmp_path / 'out' / 'archive_content.npz').is_file()
    assert (tmp_path / 'out' / 'stage_provenance.json').is_file()


def test_striping_that_leaves_the_mean_alone_still_fails(tmp_path):
    """
    The artefact that got through was periodic banding, not a brightness shift.

    Scalars alone would pass this: the stripes are added and subtracted in
    equal measure, so the frame mean barely moves. The column profile is what
    refuses it.
    """
    reference = _run(tmp_path, 'reference')

    def stripe(image):
        columns = np.arange(image.shape[1])
        wave = (40.0 * np.sign(np.sin(columns / 6.0))).astype(np.float32)
        return np.clip(image.astype(np.float32) + wave, 0, 255).astype(np.uint8)

    observed = _run(tmp_path, 'striped', transform=stripe)
    a, b = measure_run(observed), measure_run(reference)
    verdict = compare(a, b, scalar_tolerance=0.05, profile_tolerance=0.02)
    assert abs(verdict['scalar_median_ratio']['mean'] - 1.0) < 0.05
    assert verdict['passed'] is False
    assert any('column_profile' in reason for reason in verdict['failures'])


def test_an_existing_output_directory_is_refused(tmp_path):
    observed = _run(tmp_path, 'observed')
    (tmp_path / 'out').mkdir()
    with pytest.raises(ArchiveContentError, match='must be new'):
        summarize_archive_content([observed], tmp_path / 'out')


def test_a_run_without_raw_frames_is_refused(tmp_path):
    empty = tmp_path / 'empty'
    (empty / 'images' / 'raw').mkdir(parents=True)
    with pytest.raises(ArchiveContentError, match='no raw images'):
        summarize_archive_content([empty], tmp_path / 'out')


def test_one_ruined_frame_in_a_run_is_refused(tmp_path):
    """
    Run-level statistics cannot see this, which is the whole point.

    A single black frame moves a six-frame median a little and a six-frame mean
    profile by a sixth; in a 680-frame collection it moves neither measurably.
    The per-frame comparison is what refuses it.
    """
    reference = _run(tmp_path, 'reference')
    observed = _run(tmp_path, 'one_black')
    black = observed / 'images' / 'raw' / '000003.png'
    cv2.imwrite(str(black), np.zeros((240, 320), np.uint8))
    a, b = measure_run(observed), measure_run(reference)
    verdict = compare(a, b, scalar_tolerance=0.05, profile_tolerance=0.02)
    assert verdict['per_frame_deviation']['worst_frame_index'] == 3
    assert verdict['passed'] is False
    assert any('frame 3' in reason for reason in verdict['failures'])


def test_two_swapped_frames_are_refused(tmp_path):
    """Order matters: averaged statistics are blind to it, indexed frames are not."""
    reference = _run(tmp_path, 'reference')
    observed = _run(tmp_path, 'swapped')
    first = observed / 'images' / 'raw' / '000001.png'
    second = observed / 'images' / 'raw' / '000004.png'
    one, two = cv2.imread(str(first), 0), cv2.imread(str(second), 0)
    cv2.imwrite(str(first), two)
    cv2.imwrite(str(second), one)
    a, b = measure_run(observed), measure_run(reference)
    run_level = compare(a, b, 0.05, 0.02, frame_tolerance=0.99,
                        frame_p95_tolerance=0.99)
    assert run_level['passed'] is True, 'averaged statistics should not notice a swap'
    verdict = compare(a, b, scalar_tolerance=0.05, profile_tolerance=0.02)
    assert verdict['passed'] is False
    assert verdict['per_frame_deviation']['frames_over_tolerance'] >= 2


def test_a_different_frame_count_is_refused_rather_than_truncated(tmp_path):
    reference = _run(tmp_path, 'reference')
    observed = _run(tmp_path, 'short')
    (observed / 'images' / 'raw' / '000005.png').unlink()
    with pytest.raises(ArchiveContentError, match='different frame counts'):
        compare(measure_run(observed), measure_run(reference), 0.05, 0.02)
