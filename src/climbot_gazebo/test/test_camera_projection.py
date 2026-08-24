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

"""Verify the front camera lever arm follows every wall-plane heading."""

import math

from climbot_gazebo.camera_projection import camera_centre
import pytest


@pytest.mark.parametrize(
    'yaw, expected',
    [(0.0, [5.3, 2.0, 0.276]),
     (math.pi / 2.0, [5.0, 2.3, 0.276]),
     (math.pi, [4.7, 2.0, 0.276]),
     (-math.pi / 2.0, [5.0, 1.7, 0.276])])
def test_camera_centre_rotates_the_front_offset(yaw, expected):
    actual = camera_centre(
        [5.0, 2.0, 0.001], yaw, [0.3, 0.0, 0.275])
    assert actual == pytest.approx(expected)


@pytest.mark.parametrize(
    'position, yaw, offset',
    [([float('nan'), 0.0, 0.0], 0.0, [0.3, 0.0, 0.275]),
     ([0.0, 0.0, 0.0], float('inf'), [0.3, 0.0, 0.275]),
     ([0.0, 0.0, 0.0], 0.0, [0.3, float('nan'), 0.275])])
def test_nonfinite_projection_input_is_rejected(position, yaw, offset):
    with pytest.raises(ValueError):
        camera_centre(position, yaw, offset)
