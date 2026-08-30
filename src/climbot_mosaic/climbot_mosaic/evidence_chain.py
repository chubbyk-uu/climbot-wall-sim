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
Recover the acceptance domain from the frozen task instead of from an operator.

The gate asks whether every feature pixel inside the inspection region was
photographed.  While that region arrived as four numbers on a command line, the
person running the checker chose what the run would be judged against, and a
joint mosaic made of two archives could be judged against a region belonging to
neither.  So the region is walked out of the evidence itself:

    mosaic manifest -> processed runs -> source archives -> frozen task

Every link is checked by hash.  A processed run whose manifest digest is not
among the ones the mosaic recorded did not go into that mosaic, and an archive
whose digest is not the one the processed run recorded is not the archive that
run came from.  Either way the answer is a refusal, not a region.
"""

import json
import math
from pathlib import Path
from typing import Any

from climbot_common.hashing import sha256_file
from climbot_mosaic.stage_provenance import STAGE_PROVENANCE_FILENAME


class EvidenceChainError(Exception):
    """Raised when the chain from mosaic back to frozen task cannot be closed."""


def _document(path: Path, description: str) -> dict[str, Any]:
    try:
        document = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, ValueError) as error:
        raise EvidenceChainError(f'{description} is unreadable: {error}') from error
    if not isinstance(document, dict):
        raise EvidenceChainError(f'{description} is not a JSON object.')
    return document


def _polygon(points: Any, description: str) -> tuple[tuple[float, float], ...]:
    if not isinstance(points, list) or len(points) < 3:
        raise EvidenceChainError(f'{description} needs at least three points.')
    polygon = []
    for point in points:
        if not isinstance(point, dict):
            raise EvidenceChainError(f'{description} has a non-object point.')
        try:
            x, y = float(point['x']), float(point['y'])
        except (KeyError, TypeError, ValueError) as error:
            raise EvidenceChainError(
                f'{description} has a point without an x and a y.') from error
        # A NaN vertex compares false against every bound, so a region built
        # from one excuses the whole wall while reporting that it checked it.
        if not math.isfinite(x) or not math.isfinite(y):
            raise EvidenceChainError(f'{description} has a non-finite point.')
        polygon.append((x, y))
    return tuple(polygon)


def _archive_for_run(archive_root: Path, run_id: str) -> Path:
    matches = sorted(archive_root.glob(f'*/r*_{run_id}/manifest.json'))
    if len(matches) != 1:
        raise EvidenceChainError(
            f'archive root holds {len(matches)} archives for run {run_id}, expected exactly one.')
    return matches[0]


def resolve_frozen_tasks(mosaic_dir: Path, input_runs: tuple[Path, ...],
                         archive_root: Path) -> dict[str, Any]:
    """
    Walk a finished mosaic back to the frozen tasks that produced its frames.

    The processed-run directories have to be named because a mosaic manifest
    deliberately records run identity without host paths.  Naming the wrong one
    is caught: the set of processing-manifest digests must equal the set the
    mosaic recorded, so a run that was not in this mosaic cannot supply the
    region this mosaic is judged against.
    """
    mosaic = _document(mosaic_dir / 'mosaic_manifest.json', 'mosaic manifest')
    summary = mosaic.get('input_summary')
    if not isinstance(summary, dict):
        raise EvidenceChainError('mosaic manifest has no input summary.')
    recorded = summary.get('processing_manifest_sha256')
    if not isinstance(recorded, list) or not recorded:
        raise EvidenceChainError('mosaic manifest records no processed-run digests.')
    if not input_runs:
        raise EvidenceChainError('no processed run was named for the mosaic.')

    tasks = []
    seen = []
    for run in input_runs:
        manifest_path = Path(run).resolve() / 'processing_manifest.json'
        if not manifest_path.is_file():
            raise EvidenceChainError(f'processed run has no processing manifest: {run}.')
        digest = sha256_file(manifest_path)
        seen.append(digest)
        processing = _document(manifest_path, 'processing manifest')
        source = processing.get('source_archive')
        if not isinstance(source, dict):
            raise EvidenceChainError('processing manifest names no source archive.')
        run_id, expected = source.get('run_id'), source.get('manifest_sha256')
        if not isinstance(run_id, str) or not isinstance(expected, str):
            raise EvidenceChainError('source archive record is incomplete.')
        archive_path = _archive_for_run(Path(archive_root).resolve(), run_id)
        if sha256_file(archive_path) != expected:
            raise EvidenceChainError(
                f'archive {run_id} does not match the digest its processed run recorded.')
        archive = _document(archive_path, 'archive manifest')
        task = archive.get('task')
        if not isinstance(task, dict):
            raise EvidenceChainError(f'archive {run_id} carries no frozen task snapshot.')
        try:
            footprint = (float(task['detection_width_m']), float(task['detection_length_m']),
                         float(task['detection_forward_offset_m']))
        except (KeyError, TypeError, ValueError) as error:
            raise EvidenceChainError(
                f'archive {run_id} task snapshot has no camera footprint.') from error
        tasks.append({
            'task_id': task.get('task_id'),
            'source_run_id': run_id,
            'archive_manifest_sha256': expected,
            'processing_manifest_sha256': digest,
            'coverage_region_m': _polygon(task.get('coverage_region'), 'frozen coverage region'),
            'camera_footprint_m': footprint,
        })

    if sorted(seen) != sorted(recorded):
        raise EvidenceChainError(
            'the processed runs named are not the ones this mosaic was built from.')
    return {'tasks': tuple(tasks)}


def _artifacts(section: Any) -> list[tuple[str, str]]:
    """Collect (name, digest) pairs from an inputs or outputs section."""
    found = []
    if isinstance(section, dict):
        values = section.values()
    elif isinstance(section, list):
        values = section
    else:
        return found
    for value in values:
        if isinstance(value, dict) and isinstance(value.get('name'), str) \
                and isinstance(value.get('sha256'), str):
            found.append((value['name'], value['sha256']))
        elif isinstance(value, (dict, list)):
            found.extend(_artifacts(list(value.values()) if isinstance(value, dict) else value))
    return found


def verify_stage_chain(stage_directories: dict[str, Path]) -> dict[str, Any]:
    """
    Read each stage's own provenance and check that the links between them hold.

    Nothing here is told what should match what.  A stage records the digest of
    every artifact it read and every artifact it wrote, so a link is simply a
    name that one stage produced and another consumed, and the check is that
    the two digests agree.  A file replaced between stages breaks that equality
    and is reported rather than summarised over -- which is the whole reason a
    chain gets its own record per stage instead of one block at the end.
    """
    records: dict[str, dict[str, Any]] = {}
    for stage, directory in stage_directories.items():
        path = Path(directory) / STAGE_PROVENANCE_FILENAME
        if not path.is_file():
            raise EvidenceChainError(
                f'stage {stage} published no provenance record; it predates the chain '
                'and cannot be cited.')
        records[stage] = _document(path, f'{stage} provenance')

    produced: dict[str, tuple[str, str]] = {}
    for stage, record in records.items():
        for name, digest in _artifacts(record.get('outputs')):
            produced[name] = (stage, digest)

    links = []
    for stage, record in records.items():
        for name, digest in _artifacts(record.get('inputs')):
            if name not in produced:
                continue
            upstream, expected = produced[name]
            if digest != expected:
                raise EvidenceChainError(
                    f'{stage} read a different {name} than {upstream} published; '
                    'the chain is broken between them.')
            links.append({'artifact': name, 'from': upstream, 'to': stage, 'sha256': digest})
    return {'stages': records, 'links': sorted(
        links, key=lambda link: (link['from'], link['to'], link['artifact']))}
