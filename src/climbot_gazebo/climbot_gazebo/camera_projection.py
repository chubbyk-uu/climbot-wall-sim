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

"""Project the fixed camera optical centre from a wall-plane robot pose."""

import math


def camera_centre(position, yaw, optical_offset):
    """Apply planar robot yaw to a 3-D base_link-to-camera translation."""
    if len(position) != 3 or len(optical_offset) != 3:
        raise ValueError('position and optical_offset must have three values')
    values = [*position, yaw, *optical_offset]
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError('camera projection inputs must be finite')
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return [
        position[0] + cosine * optical_offset[0] - sine * optical_offset[1],
        position[1] + sine * optical_offset[0] + cosine * optical_offset[1],
        position[2] + optical_offset[2],
    ]


def optical_axis_footprint(camera, axis, height=None, height_bound_m=0.05):
    """
    Intersect a wall-frame ray with z=height(x,y), checking the final residual.

    The M2 source is bounded within +/-30 mm. The explicit bracket avoids the
    old three-iteration fixed-point approximation at steep groove shoulders.
    ``height=None`` denotes the reference plane. Return (point, ray distance).
    """
    import numpy as np
    from scipy.optimize import brentq

    camera, axis = np.asarray(camera, float), np.asarray(axis, float)
    if (camera.shape != (3,) or axis.shape != (3,) or
            not np.isfinite([*camera, *axis, height_bound_m]).all() or
            height_bound_m <= 0 or axis[2] >= -1e-9 or
            abs(np.linalg.norm(axis) - 1.0) > 1e-6):
        raise ValueError('footprint needs a finite camera and a unit ray toward the wall')
    if camera[2] <= height_bound_m:
        raise ValueError('camera must be outside the bounded wall surface')
    if height is None:
        distance = -camera[2] / axis[2]
    else:
        def residual(distance):
            point = camera + distance * axis
            return float(point[2] - height(point[0], point[1]))

        lower = (camera[2] - height_bound_m) / -axis[2]
        upper = (camera[2] + height_bound_m) / -axis[2]
        distance = brentq(residual, lower, upper, xtol=1e-12)
        if abs(residual(distance)) > 1e-9:
            raise ValueError('optical footprint did not converge on the surface')
    return camera + distance * axis, float(distance)
