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

"""
Say what produced one stage's output, in the same shape for every stage.

A formal summary once carried a single provenance block for a chain that ran
across several commits, and named a commit that did not contain the code which
produced the summary's own fields.  One block cannot describe a chain: each
stage has its own code version, its own parameters, and its own view of what it
read.  So each stage writes its own record, next to the products it describes
and inside the same directory publish, and the summary generator reads the
chain rather than being told about it.
"""

from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from climbot_common.atomic import write_json
from climbot_common.hashing import sha256_file
from climbot_common.provenance import git_state

#: The file each stage leaves in its own output directory.
STAGE_PROVENANCE_FILENAME = 'stage_provenance.json'

STAGE_PROVENANCE_FORMAT_VERSION = 1


def artifact(path: Path) -> dict[str, str]:
    """
    Name a file by content, not by where it happened to sit.

    Host paths are deliberately absent: a digest is what lets the next stage
    prove it read this exact file, and a path only says where somebody's disk
    had it that afternoon.
    """
    path = Path(path)
    return {'name': path.name, 'sha256': sha256_file(path)}


def processed_run_inputs(summary: dict[str, Any]) -> dict[str, Any]:
    """Name the processed runs a stage read, by identity and manifest digest."""
    return {'processed_runs': [
        {'source_run_id': run_id, 'processing_manifest_sha256': digest}
        for run_id, digest in zip(summary['source_run_ids'],
                                  summary['processing_manifest_sha256'])]}


def stage_record(stage: str, parameters: dict[str, Any],
                 inputs: dict[str, Any], outputs: dict[str, Any]) -> dict[str, Any]:
    """Describe one stage run: its code, its settings, and what it read and wrote."""
    return {
        'stage_provenance_format_version': STAGE_PROVENANCE_FORMAT_VERSION,
        'stage': stage,
        'recorded_utc': datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
        'git': git_state(),
        'parameters': parameters,
        'inputs': inputs,
        'outputs': outputs,
    }


def write_stage_provenance(directory: Path, stage: str, parameters: dict[str, Any],
                           inputs: dict[str, Any], output_names: tuple[str, ...]) -> Path:
    """
    Write the record into a directory that is about to be published atomically.

    Called on the temporary directory rather than the final one, so the record
    and the products it hashes become visible in the same rename.  A stage that
    fails after writing its products leaves neither.
    """
    directory = Path(directory)
    outputs = {name: artifact(directory / name) for name in output_names}
    return write_json(directory / STAGE_PROVENANCE_FILENAME,
                      stage_record(stage, parameters, inputs, outputs))
