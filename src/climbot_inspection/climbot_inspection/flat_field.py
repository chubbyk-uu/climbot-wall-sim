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

"""Patent-compatible mono8 LED flat-field calibration math."""

from dataclasses import dataclass
import hashlib

import cv2
import numpy as np


@dataclass(frozen=True)
class Calibration:
    """Computed correction and evidence that source frames were independent."""

    gain: np.ndarray
    mean_image: np.ndarray
    unique_hashes: int
    temporal_noise_dn: float
    trimmed_mean_dn: float
    saturated_fraction: float


def image_hash(image):
    """Return a stable content digest for duplicate-frame rejection."""
    return hashlib.sha256(np.ascontiguousarray(image).tobytes()).hexdigest()


def compute_calibration(images, target_mean_dn=128.0, blur_sigma_px=2.0,
                        max_saturated_fraction=0.0001):
    """Compute multiplicative flat-field gain from at least 11 mono8 images."""
    if len(images) < 11:
        raise ValueError('flat-field calibration requires at least 11 images')
    frames = np.asarray(images)
    if frames.ndim != 3 or frames.dtype != np.uint8:
        raise ValueError('images must be equally sized mono8 arrays')
    if not np.isfinite(target_mean_dn) or not 1.0 <= target_mean_dn <= 254.0:
        raise ValueError('target_mean_dn must be finite and in [1, 254]')
    if (not np.isfinite(max_saturated_fraction) or
            not 0.0 <= max_saturated_fraction < 1.0):
        raise ValueError('max_saturated_fraction must be finite and in [0, 1)')
    hashes = {image_hash(frame) for frame in frames}
    if len(hashes) != len(frames):
        raise ValueError('duplicate calibration frame detected')
    source = frames.astype(np.float32)
    saturated_fraction = float(np.mean(frames >= 254))
    if saturated_fraction > max_saturated_fraction:
        raise ValueError(
            'flat-field source is saturated: %.6f exceeds %.6f' % (
                saturated_fraction, max_saturated_fraction))
    temporal_noise = float(np.mean(np.std(source, axis=0, ddof=1)))
    if temporal_noise < 0.10:
        raise ValueError('calibration frames lack measurable temporal noise')
    if blur_sigma_px > 0.0:
        source = np.asarray([
            cv2.GaussianBlur(frame, (0, 0), blur_sigma_px)
            for frame in source
        ])
    mean_image = np.mean(source, axis=0)
    if float(np.min(mean_image)) < 1.0:
        raise ValueError('flat-field target contains invalid dark pixels')
    spatial_gain = float(np.max(mean_image)) / mean_image
    means = np.sort(np.mean(source, axis=(1, 2)))
    trim = int(np.floor(0.20 * len(means)))
    trimmed = means[trim:len(means) - trim]
    trimmed_mean = float(np.mean(trimmed))
    # Patent global coefficient: b=d/(a*V), where a is mean spatial gain.
    global_gain = target_mean_dn / (float(np.mean(spatial_gain)) * trimmed_mean)
    gain = (spatial_gain * global_gain).astype(np.float32)
    return Calibration(
        gain=gain,
        mean_image=mean_image.astype(np.float32),
        unique_hashes=len(hashes),
        temporal_noise_dn=temporal_noise,
        trimmed_mean_dn=trimmed_mean,
        saturated_fraction=saturated_fraction)


def apply_calibration(image, gain):
    """Apply gain to one mono8 image with saturating conversion."""
    if image.dtype != np.uint8 or image.ndim != 2:
        raise ValueError('image must be mono8')
    if gain.shape != image.shape or not np.all(np.isfinite(gain)):
        raise ValueError('gain must be finite and match the image')
    return np.clip(image.astype(np.float32) * gain, 0, 255).astype(np.uint8)


def validate_gain(gain, expected_shape):
    """Reject an incompatible or non-physical stored flat-field gain."""
    if gain.shape != tuple(expected_shape):
        raise ValueError(
            'calibration gain shape must be %s, got %s' % (tuple(expected_shape), gain.shape))
    if not np.all(np.isfinite(gain)) or not np.all(gain > 0.0):
        raise ValueError('calibration gain must be finite and positive')
