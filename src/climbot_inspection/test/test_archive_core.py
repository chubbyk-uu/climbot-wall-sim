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

"""Pure G4 archive invariants, kept independent of a running ROS graph."""

from pathlib import Path
from types import SimpleNamespace

from climbot_inspection.archive_core import (
    ArchiveError,
    atomic_write_json,
    capture_count_for_length,
    estimated_archive_bytes,
    expected_image_count,
    resolved_output_root,
    run_directory,
    safe_task_id,
)
import pytest


def point(x, y):
    return SimpleNamespace(position=SimpleNamespace(x=x, y=y))


def task():
    return SimpleNamespace(
        SEGMENT_SCAN=1,
        segment_types=[1, 2, 1],
        waypoints=[point(0.0, 0.0), point(1.0, 0.0), point(1.0, 0.2), point(1.0, 0.8)],
    )


def test_task_component_rejects_path_injection():
    assert safe_task_id('wall-A_02.demo') == 'wall-A_02.demo'
    for value in ('../outside', 'wall/side', '', '.hidden', '..', 'bad space'):
        with pytest.raises(ArchiveError):
            safe_task_id(value)


def test_run_directory_is_unique_per_run_and_stays_under_root(tmp_path):
    root = resolved_output_root(str(tmp_path))
    first = run_directory(root, 'wall-a', 12, '20260825T103015Z', 'a' * 32)
    second = run_directory(root, 'wall-a', 12, '20260825T103015Z', 'b' * 32)
    assert first != second
    assert first.parent.parent == root
    assert first.name.startswith('r000012_20260825T103015Z_')


def test_expected_images_matches_per_scan_capture_rule():
    # The nominal task estimate reserves enough archive space before dynamic
    # SCAN references exist. It is deliberately not a final capture contract.
    # effective length 0.28125, overlap 25% => 0.2109375 m spacing. The
    # first 1 m SCAN estimates five frames; the final 0.6 m SCAN estimates three.
    assert expected_image_count(task(), 0.28125, 0.25) == 8
    assert estimated_archive_bytes(8, 1920, 1080) > 8 * 1920 * 1080


def test_frozen_reference_count_keeps_final_target_inside_arrival_tolerance():
    # With 0.2 m spacing, a 1 m frozen line has five captures at base-route
    # progress 0.0, 0.2, ... 0.8.  A sixth capture exactly at 1.0 m is not a
    # valid contract because the tracker may finish inside its endpoint band.
    assert capture_count_for_length(1.0, 0.25, 0.20) == 5
    assert capture_count_for_length(0.01, 0.25, 0.20) == 1


def test_atomic_json_rejects_nonfinite_values_without_creating_destination(tmp_path):
    destination = Path(tmp_path) / 'label.json'
    with pytest.raises(ValueError):
        atomic_write_json(destination, {'bad': float('inf')})
    assert not destination.exists()
