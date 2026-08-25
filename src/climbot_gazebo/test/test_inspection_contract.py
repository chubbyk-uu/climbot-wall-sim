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

"""Cross-package geometry contracts for online archive and G2 acceptance."""

from pathlib import Path

from climbot_gazebo.inspection_contract import DEFAULT_MINIMUM_ACTUAL_OVERLAP_RATIO
import pytest
import yaml


WORKSPACE = Path(__file__).resolve().parents[3]


def test_archive_spacing_guard_fits_within_the_g2_measured_overlap_limit():
    """A nominally valid archive must be eligible to pass the formal evaluator."""
    description = WORKSPACE / 'src' / 'climbot_description'
    inspection = WORKSPACE / 'src' / 'climbot_inspection'
    camera = yaml.safe_load(
        (description / 'config' / 'inspection_camera.yaml').read_text())['inspection_camera']
    parameters = yaml.safe_load(
        (inspection / 'config' / 'inspection.yaml').read_text())
    automatic = parameters['automatic_capture_node']['ros__parameters']
    recorder = parameters['archive_recorder_node']['ros__parameters']
    effective_length = float(camera['footprint']['effective_length_m'])
    spacing = effective_length * (1.0 - float(recorder['image_overlap_ratio']))
    archive_limit = spacing + float(recorder['actual_spacing_tolerance_m'])
    evaluator_limit = effective_length * (1.0 - DEFAULT_MINIMUM_ACTUAL_OVERLAP_RATIO)
    assert float(automatic['image_overlap_ratio']) == pytest.approx(
        float(recorder['image_overlap_ratio']))
    assert archive_limit <= evaluator_limit


def test_capture_gate_lag_is_stricter_than_the_archive_lag_limit():
    """The control barrier must never permit a capture the recorder rejects."""
    inspection = WORKSPACE / 'src' / 'climbot_inspection'
    parameters = yaml.safe_load(
        (inspection / 'config' / 'inspection.yaml').read_text())
    automatic = parameters['automatic_capture_node']['ros__parameters']
    recorder = parameters['archive_recorder_node']['ros__parameters']
    assert 0.0 < float(automatic['capture_gate_max_lag_m']) < float(
        recorder['maximum_target_lag_m'])
