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

"""Unit tests for independent-frame flat-field calibration."""

from climbot_inspection.flat_field import apply_calibration, compute_calibration
import numpy as np
import pytest


def _frames(count=30):
    rng = np.random.default_rng(42)
    y, x = np.mgrid[-1:1:48j, -1:1:64j]
    illumination = 110.0 - 35.0 * (x * x + y * y)
    return [
        np.clip(illumination + rng.normal(0.0, 1.0, illumination.shape), 0, 255)
        .astype(np.uint8)
        for _ in range(count)
    ]


def test_thirty_noisy_frames_flatten_the_led_field():
    frames = _frames()
    calibration = compute_calibration(frames)
    corrected = apply_calibration(frames[0], calibration.gain)
    assert calibration.unique_hashes == 30
    assert calibration.temporal_noise_dn > 0.5
    assert calibration.saturated_fraction == 0.0
    # A single corrected exposure retains sensor noise; only the illumination
    # gradient (about 70 DN here) is expected to disappear.
    assert float(np.std(corrected.astype(float))) < 3.5
    assert float(np.mean(corrected)) == pytest.approx(128.0, abs=7.0)


def test_duplicate_frame_is_rejected():
    frame = _frames(1)[0]
    with pytest.raises(ValueError, match='duplicate'):
        compute_calibration([frame.copy() for _ in range(30)])


def test_noise_free_sequence_is_rejected_even_if_hashes_differ_slightly():
    frames = [np.full((12, 16), 100, dtype=np.uint8) for _ in range(30)]
    for index, frame in enumerate(frames):
        frame.flat[index] = 101
    with pytest.raises(ValueError, match='temporal noise'):
        compute_calibration(frames)


def test_saturated_flat_field_is_rejected_before_gain_is_computed():
    frames = _frames()
    for frame in frames:
        frame[:, :8] = 255
    with pytest.raises(ValueError, match='saturated'):
        compute_calibration(frames)
