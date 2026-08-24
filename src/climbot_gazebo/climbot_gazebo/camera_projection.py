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
