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

"""Hold the executable-file contract required by ``ros2 run``."""

import os
from pathlib import Path


def test_installed_program_sources_are_executable():
    scripts = Path(__file__).parents[1] / 'scripts'
    expected = {
        'build_initial_projection',
        'build_local_matches',
        'build_overlap_candidates',
        'build_pose_graph',
        'build_wall_mosaic',
        'evaluate_diagnostic_mosaic',
        'inspect_diagnostic_mosaic',
        'preflight_diagnostic_coverage',
        'summarize_archive_content',
        'validate_mosaic_inputs',
    }
    assert {path.name for path in scripts.iterdir()} == expected
    for name in expected:
        path = scripts / name
        assert os.access(path, os.X_OK), f'{path} is not executable'
        assert path.read_bytes().startswith(b'#!/usr/bin/env python3\n')
