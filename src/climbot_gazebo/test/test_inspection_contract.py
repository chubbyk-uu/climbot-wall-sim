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

from ament_index_python.packages import get_package_share_directory
from climbot_gazebo.inspection_contract import DEFAULT_MINIMUM_ACTUAL_OVERLAP_RATIO
import pytest
import yaml


def _config(package: str, name: str) -> dict:
    """
    Read one installed package config.

    Resolved through the ament index rather than by walking up from this file
    to a sibling package's source tree: that arithmetic encodes a workspace
    layout this test has no reason to know, and it breaks as soon as the test
    runs anywhere other than a src/<pkg>/test checkout. The share directory is
    also what the nodes themselves load, so this compares the deployed values.
    """
    path = Path(get_package_share_directory(package)) / 'config' / name
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def test_archive_spacing_guard_fits_within_the_g2_measured_overlap_limit():
    """A nominally valid archive must be eligible to pass the formal evaluator."""
    camera = _config(
        'climbot_description', 'inspection_camera.yaml')['inspection_camera']
    parameters = _config('climbot_inspection', 'inspection.yaml')
    automatic = parameters['automatic_capture_node']['ros__parameters']
    recorder = parameters['archive_recorder_node']['ros__parameters']
    effective_length = float(camera['footprint']['effective_length_m'])
    spacing = effective_length * (1.0 - float(recorder['image_overlap_ratio']))
    archive_limit = spacing + float(recorder['actual_spacing_tolerance_m'])
    evaluator_limit = effective_length * (1.0 - DEFAULT_MINIMUM_ACTUAL_OVERLAP_RATIO)
    assert float(automatic['image_overlap_ratio']) == pytest.approx(
        float(recorder['image_overlap_ratio']))
    assert archive_limit <= evaluator_limit


def test_capture_supervision_outlasts_the_execution_reference_heartbeat():
    """
    A stalled execution reference must stop the robot, not just the camera.

    Two independent timers watch the same signal. automatic_capture stops
    triggering once the reference is older than reference_timeout_s, and
    line_tracker stops the robot once the capture gate that reference produces
    has been stale for capture_gate_timeout_s and then unanswered for another
    capture_gate_timeout_s. If the capture timer is the shorter of the two, a
    stall opens a window in which the robot still drives a scan line while no
    exposure is being taken, and the first exposure after recovery lands past
    its target and is rejected by the archive's longitudinal contract.

    The reference is not a sampled measurement whose value decays: line_tracker
    publishes immediately whenever any field changes, so segment boundaries,
    frozen geometry, pause and resume are never delayed by the heartbeat. The
    beat in between only asserts that the executor is alive, which is why
    widening this timeout costs nothing and closing the window is worth it.
    """
    control = _config('climbot_control', 'control.yaml')['line_tracker']['ros__parameters']
    automatic = _config(
        'climbot_inspection', 'inspection.yaml')['automatic_capture_node']['ros__parameters']
    gate_timeout = float(control['capture_gate_timeout_s'])
    reference_timeout = float(automatic['reference_timeout_s'])
    heartbeat_period = 1.0 / float(control['execution_reference_heartbeat_hz'])
    assert reference_timeout >= 2.0 * gate_timeout
    # The heartbeat is what both timers are measured against, so it has to be
    # comfortably shorter than the tighter of them rather than merely shorter.
    assert heartbeat_period <= 0.5 * gate_timeout
