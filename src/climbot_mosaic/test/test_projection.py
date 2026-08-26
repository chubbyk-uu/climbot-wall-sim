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

"""Synthetic metric checks for P2.3 optical ray projection."""

from pathlib import Path

from climbot_mosaic.mosaic_inputs import (
    CameraModel,
    FrameKey,
    MosaicInputs,
    ProcessedFrame,
    ProcessedRun,
)
from climbot_mosaic.projection import (
    project_frame,
    project_inputs,
    projection_extent,
    projection_manifest,
    ProjectionError,
    render_footprint_preview,
    write_initial_projection,
)
import cv2
import numpy as np
import pytest


def make_inputs(tmp_path: Path, orientation=None) -> MosaicInputs:
    """Create a rectified 3x3 camera that looks from z=1 onto the z=0 wall."""
    if orientation is None:
        orientation = {'x': 1.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}
    camera = CameraModel(
        3, 3, (2.0, 0.0, 1.0, 0.0, 2.0, 1.0, 0.0, 0.0, 1.0),
        (0.0, 0.0, 1.0), (0.0, 0.0, 0.0), 'camera-signature')
    label = {
        'task_id': 'synthetic',
        'camera_pose': {'pose': {
            'position': {'x': 1.0, 'y': 2.0, 'z': 1.0},
            'orientation': orientation,
        }},
    }
    frame = ProcessedFrame(
        FrameKey('run-1', 0), tmp_path / 'image.png', 'a' * 64,
        tmp_path / 'label.json', label)
    run = ProcessedRun('run-1', 'b' * 64, tmp_path / 'input', camera, (frame,))
    return MosaicInputs(camera, (run,), (frame,))


def test_project_frame_maps_center_and_corners_to_metric_wall(tmp_path):
    """A known fronto-parallel optical pose has the expected metre footprint."""
    projection = project_frame(make_inputs(tmp_path).frames[0], make_inputs(tmp_path))

    assert projection.optical_center_xy_m == pytest.approx((1.0, 2.0))
    assert projection.camera_plane_distance_m == pytest.approx(1.0)
    assert np.asarray(projection.footprint_xy_m) == pytest.approx(np.asarray(
        ((0.5, 2.5), (1.5, 2.5), (1.5, 1.5), (0.5, 1.5))))
    homography = np.asarray(projection.homography_image_to_wall).reshape(3, 3)
    mapped = cv2.perspectiveTransform(
        np.asarray([[[1.0, 1.0]]], dtype=np.float32), homography)[0, 0]
    assert mapped == pytest.approx((1.0, 2.0))


def test_rejects_camera_that_does_not_face_wall(tmp_path):
    """A ray parallel to the wall is not a usable planar projection."""
    inputs = make_inputs(tmp_path, {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0})

    with pytest.raises(ProjectionError, match='behind'):
        project_inputs(inputs)


def test_projection_manifest_is_stable_and_host_path_free(tmp_path):
    """Projection records are ordered, finite and never leak archive paths."""
    inputs = make_inputs(tmp_path)
    manifest = projection_manifest(inputs, project_inputs(inputs))

    assert manifest['initial_projection_format_version'] == 1
    assert manifest['input_summary']['frame_count'] == 1
    assert manifest['footprint_extent_xy_m'] == pytest.approx(
        {'min_x': 0.5, 'min_y': 1.5, 'max_x': 1.5, 'max_y': 2.5})
    assert str(tmp_path) not in str(manifest)


def test_preview_and_atomic_output_are_created_for_new_absolute_directory(tmp_path):
    """The preview is an outline diagnostic and final output is atomically named."""
    inputs = make_inputs(tmp_path)
    output = tmp_path / 'projection-result'
    manifest = write_initial_projection(output, inputs, preview_max_side_px=64)

    assert manifest['input_summary']['frame_count'] == 1
    assert (output / 'initial_projection.json').is_file()
    preview = cv2.imread(str(output / 'initial_footprints_preview.png'), cv2.IMREAD_COLOR)
    assert preview is not None
    assert preview.shape[0] <= 64
    assert preview.shape[1] <= 64
    with pytest.raises(ProjectionError, match='must not already exist'):
        write_initial_projection(output, inputs)


def test_extent_rejects_empty_projection_sequence():
    """There is no valid preview or manifest extent without one projected image."""
    with pytest.raises(ProjectionError, match='at least one'):
        projection_extent(())


def test_preview_requires_sane_maximum_size(tmp_path):
    """Preview limits are explicit rather than silently allocating an invalid image."""
    projections = project_inputs(make_inputs(tmp_path))
    with pytest.raises(ProjectionError, match='at least 64'):
        render_footprint_preview(projections, 63)
