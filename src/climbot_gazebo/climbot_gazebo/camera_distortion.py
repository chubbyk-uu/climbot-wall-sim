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

"""Brown-Conrady maps for an Ogre2 overscan image and calibrated raw image."""

import math

import cv2
import numpy as np
import yaml


def load_calibration(path):
    """Load and validate the shared nominal inspection-camera calibration."""
    with open(path) as handle:
        camera = yaml.safe_load(handle)['inspection_camera']
    image = camera['image']
    calibration = camera['calibration']
    intrinsics = calibration['intrinsics']
    values = [
        intrinsics['fx_px'], intrinsics['fy_px'], intrinsics['cx_px'],
        intrinsics['cy_px'], intrinsics['skew'],
        *calibration['distortion'],
    ]
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError('camera calibration values must be finite')
    width = int(image['width_px'])
    height = int(image['height_px'])
    if width <= 0 or height <= 0:
        raise ValueError('camera image dimensions must be positive')
    if intrinsics['fx_px'] <= 0.0 or intrinsics['fy_px'] <= 0.0:
        raise ValueError('camera focal lengths must be positive')
    if calibration['distortion_model'] != 'plumb_bob':
        raise ValueError('only plumb_bob distortion is supported')
    if len(calibration['distortion']) != 5:
        raise ValueError('plumb_bob distortion must have five coefficients')
    return camera


def matrices(camera):
    """Return OpenCV K and D arrays in ROS plumb_bob coefficient order."""
    intrinsics = camera['calibration']['intrinsics']
    matrix = np.array([
        [intrinsics['fx_px'], intrinsics['skew'], intrinsics['cx_px']],
        [0.0, intrinsics['fy_px'], intrinsics['cy_px']],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    distortion = np.asarray(
        camera['calibration']['distortion'], dtype=np.float64)
    return matrix, distortion


def make_distortion_maps(camera, render_focal_scale):
    """Map each distorted output pixel to the wider ideal rendered source."""
    scale = float(render_focal_scale)
    if not math.isfinite(scale) or scale <= 0.0 or scale > 1.0:
        raise ValueError('render_focal_scale must be finite and in (0, 1]')
    width = int(camera['image']['width_px'])
    height = int(camera['image']['height_px'])
    matrix, distortion = matrices(camera)
    columns, rows = np.meshgrid(
        np.arange(width, dtype=np.float32),
        np.arange(height, dtype=np.float32))
    distorted_pixels = np.stack((columns, rows), axis=-1).reshape(-1, 1, 2)
    undistorted = cv2.undistortPoints(
        distorted_pixels, matrix, distortion).reshape(height, width, 2)
    render_fx = matrix[0, 0] * scale
    render_fy = matrix[1, 1] * scale
    map_x = undistorted[:, :, 0] * render_fx + matrix[0, 2]
    map_y = undistorted[:, :, 1] * render_fy + matrix[1, 2]
    return map_x.astype(np.float32), map_y.astype(np.float32)


def maps_fit_source(map_x, map_y, width, height, tolerance_px=0.5):
    """Report whether overscan supplies every distorted output pixel."""
    return (
        float(np.min(map_x)) >= -tolerance_px and
        float(np.max(map_x)) <= width - 1 + tolerance_px and
        float(np.min(map_y)) >= -tolerance_px and
        float(np.max(map_y)) <= height - 1 + tolerance_px)
