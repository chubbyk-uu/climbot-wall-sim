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

"""Verify the simulation-only Brown distortion map and overscan guard."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from climbot_gazebo.camera_distortion import (
    load_calibration,
    make_distortion_maps,
    maps_fit_source,
    matrices,
)
import numpy as np
import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def shared_camera():
    description = Path(get_package_share_directory('climbot_description'))
    return load_calibration(description / 'config' / 'inspection_camera.yaml')


def test_shared_camera_keeps_ros_distortion_order():
    camera = shared_camera()
    matrix, distortion = matrices(camera)
    assert matrix.tolist() == [
        [960.0, 0.0, 959.5], [0.0, 960.0, 539.5], [0.0, 0.0, 1.0]]
    assert distortion.tolist() == pytest.approx(
        [-0.120, 0.025, 0.0005, -0.0003, -0.004])


def test_configured_overscan_covers_every_distorted_pixel():
    camera = shared_camera()
    simulation = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'simulation.yaml').read_text())['simulation']
    scale = simulation['inspection_camera']['render_overscan_focal_scale']
    map_x, map_y = make_distortion_maps(camera, scale)
    assert map_x.shape == (1080, 1920)
    assert map_y.shape == (1080, 1920)
    assert np.isfinite(map_x).all()
    assert np.isfinite(map_y).all()
    assert maps_fit_source(map_x, map_y, 1920, 1080)


@pytest.mark.parametrize('scale', [0.0, -1.0, 1.01, float('nan')])
def test_bad_render_focal_scale_is_rejected(scale):
    with pytest.raises(ValueError):
        make_distortion_maps(shared_camera(), scale)
