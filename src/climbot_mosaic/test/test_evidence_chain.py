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

"""The acceptance domain has to come out of the evidence, not off a command line."""

import json

from climbot_common.hashing import sha256_file
from climbot_mosaic.evidence_chain import (
    EvidenceChainError,
    resolve_frozen_tasks,
    verify_inspection_tiles,
    verify_manifest_outputs,
    verify_stage_chain,
)
from climbot_mosaic.stage_provenance import artifact, write_stage_provenance
import pytest

REGION = [{'x': 0.55, 'y': 0.55, 'z': 0.0}, {'x': 9.45, 'y': 0.55, 'z': 0.0},
          {'x': 9.45, 'y': 7.45, 'z': 0.0}, {'x': 0.55, 'y': 7.45, 'z': 0.0}]


def _write(path, document):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, sort_keys=True), encoding='utf-8')
    return path


def _chain(root, run_id='a' * 32, task_id='diagnostic-horizontal', region=None):
    """Build one archive, one processed run and a mosaic that names its digest."""
    archive = _write(root / task_id / f'r000001_20260830T000000Z_{run_id}' / 'manifest.json', {
        'archive_format_version': 1,
        'task': {'task_id': task_id, 'coverage_region': region or REGION,
                 'motion_region': region or REGION, 'sweep_direction': 1,
                 'waypoints': [
                     {'position': {'x': 0.6, 'y': 0.6}},
                     {'position': {'x': 9.4, 'y': 0.6}}],
                 'segment_types': [1],
                 'detection_width_m': 0.50, 'detection_length_m': 0.28125,
                 'detection_forward_offset_m': 0.340}})
    run = root / f'processed-{task_id}'
    _write(run / 'processing_manifest.json', {
        'processing_format_version': 1,
        'source_archive': {'run_id': run_id, 'manifest_sha256': sha256_file(archive)}})
    return archive, run


def _mosaic(root, runs, name='mosaic'):
    directory = root / name
    _write(directory / 'mosaic_manifest.json', {
        'input_summary': {'processing_manifest_sha256': [
            sha256_file(run / 'processing_manifest.json') for run in runs]}})
    return directory


def test_the_region_and_footprint_come_out_of_the_frozen_task(tmp_path):
    _, run = _chain(tmp_path)
    result = resolve_frozen_tasks(_mosaic(tmp_path, [run]), (run,), tmp_path)
    task = result['tasks'][0]
    assert task['coverage_region_m'] == (
        (0.55, 0.55), (9.45, 0.55), (9.45, 7.45), (0.55, 7.45))
    assert task['camera_footprint_m'] == (0.50, 0.28125, 0.340)
    assert task['motion_region_m'] == task['coverage_region_m']
    assert task['waypoints_m'] == ((0.6, 0.6), (9.4, 0.6))
    assert task['segment_types'] == (1,)
    assert task['task_id'] == 'diagnostic-horizontal'


def test_a_joint_mosaic_resolves_every_archive_it_was_built_from(tmp_path):
    _, horizontal = _chain(tmp_path, 'a' * 32, 'diagnostic-horizontal')
    _, vertical = _chain(tmp_path, 'b' * 32, 'diagnostic-vertical')
    runs = (horizontal, vertical)
    result = resolve_frozen_tasks(_mosaic(tmp_path, runs), runs, tmp_path)
    assert [task['task_id'] for task in result['tasks']] == [
        'diagnostic-horizontal', 'diagnostic-vertical']


def test_a_run_the_mosaic_was_not_built_from_is_refused(tmp_path):
    """Otherwise the checker could be pointed at a region belonging to another run."""
    _, used = _chain(tmp_path, 'a' * 32, 'diagnostic-horizontal')
    _, unused = _chain(tmp_path, 'b' * 32, 'diagnostic-vertical')
    mosaic = _mosaic(tmp_path, [used])
    with pytest.raises(EvidenceChainError, match='not the ones this mosaic'):
        resolve_frozen_tasks(mosaic, (unused,), tmp_path)


def test_an_archive_edited_after_processing_is_refused(tmp_path):
    archive, run = _chain(tmp_path)
    document = json.loads(archive.read_text(encoding='utf-8'))
    document['task']['coverage_region'][2]['x'] = 20.0
    archive.write_text(json.dumps(document, sort_keys=True), encoding='utf-8')
    with pytest.raises(EvidenceChainError, match='does not match the digest'):
        resolve_frozen_tasks(_mosaic(tmp_path, [run]), (run,), tmp_path)


def test_an_archive_without_a_task_snapshot_is_refused(tmp_path):
    run_id = 'c' * 32
    archive = _write(tmp_path / 'task' / f'r000001_20260830T000000Z_{run_id}' / 'manifest.json',
                     {'archive_format_version': 1})
    run = tmp_path / 'processed-task'
    _write(run / 'processing_manifest.json', {
        'source_archive': {'run_id': run_id, 'manifest_sha256': sha256_file(archive)}})
    with pytest.raises(EvidenceChainError, match='no frozen task snapshot'):
        resolve_frozen_tasks(_mosaic(tmp_path, [run]), (run,), tmp_path)


def test_a_non_finite_region_vertex_is_refused(tmp_path):
    """A NaN bound excuses the whole wall while reporting that it checked it."""
    region = [dict(point) for point in REGION]
    region[1]['x'] = float('nan')
    _, run = _chain(tmp_path, region=region)
    with pytest.raises(EvidenceChainError, match='non-finite point'):
        resolve_frozen_tasks(_mosaic(tmp_path, [run]), (run,), tmp_path)


def test_naming_no_processed_run_is_refused(tmp_path):
    _, run = _chain(tmp_path)
    with pytest.raises(EvidenceChainError, match='no processed run was named'):
        resolve_frozen_tasks(_mosaic(tmp_path, [run]), (), tmp_path)


def _stage(directory, stage, inputs, outputs):
    """Publish one stage directory with its products and its own provenance."""
    directory.mkdir(parents=True, exist_ok=True)
    for name, text in outputs.items():
        (directory / name).write_text(text, encoding='utf-8')
    write_stage_provenance(directory, stage, {}, inputs, tuple(outputs))
    return directory


def test_a_chain_whose_digests_agree_reports_every_link(tmp_path):
    matches = _stage(tmp_path / 'matches', 'local_matches', {},
                     {'local_matches.json': '{"matches": []}\n'})
    graph = _stage(tmp_path / 'graph', 'pose_graph',
                   {'local_matches': artifact(matches / 'local_matches.json')},
                   {'pose_graph.json': '{"edges": []}\n'})
    result = verify_stage_chain({'local_matches': matches, 'pose_graph': graph})
    assert [(link['from'], link['to'], link['artifact']) for link in result['links']] == [
        ('local_matches', 'pose_graph', 'local_matches.json')]


def test_an_artifact_replaced_between_stages_breaks_the_chain(tmp_path):
    """A summary that averages over this is exactly the unciteable kind."""
    matches = _stage(tmp_path / 'matches', 'local_matches', {},
                     {'local_matches.json': '{"matches": []}\n'})
    graph = _stage(tmp_path / 'graph', 'pose_graph',
                   {'local_matches': artifact(matches / 'local_matches.json')},
                   {'pose_graph.json': '{"edges": []}\n'})
    (matches / 'local_matches.json').write_text('{"matches": [1]}\n', encoding='utf-8')
    write_stage_provenance(matches, 'local_matches', {}, {}, ('local_matches.json',))
    with pytest.raises(EvidenceChainError, match='chain is broken'):
        verify_stage_chain({'local_matches': matches, 'pose_graph': graph})


def test_a_stage_product_changed_without_rewriting_provenance_is_refused(tmp_path):
    matches = _stage(tmp_path / 'matches', 'local_matches', {},
                     {'local_matches.json': '{"matches": []}\n'})
    (matches / 'local_matches.json').write_text('{"matches": [1]}\n', encoding='utf-8')
    with pytest.raises(EvidenceChainError, match='no longer matches its provenance'):
        verify_stage_chain({'local_matches': matches})


def test_a_stage_name_must_match_its_record(tmp_path):
    directory = _stage(tmp_path / 'wrong', 'local_matches', {}, {'matches.json': '{}\n'})
    with pytest.raises(EvidenceChainError, match='contains provenance for local_matches'):
        verify_stage_chain({'pose_graph': directory})


def test_formal_chain_rejects_an_untraceable_stage(tmp_path, monkeypatch):
    monkeypatch.setattr('climbot_mosaic.stage_provenance.git_state', lambda: {
        'commit': None, 'source_modified': True, 'traceable': False})
    stage = _stage(tmp_path / 'matches', 'local_matches', {}, {'matches.json': '{}\n'})
    with pytest.raises(EvidenceChainError, match='not produced from a traceable'):
        verify_stage_chain({'local_matches': stage}, require_traceable=True)


def test_formal_chain_requires_the_exact_stage_set_and_links(tmp_path):
    matches = _stage(tmp_path / 'matches', 'local_matches', {},
                     {'local_matches.json': '{}\n'})
    graph = _stage(tmp_path / 'graph', 'pose_graph',
                   {'local_matches': artifact(matches / 'local_matches.json')},
                   {'pose_graph.json': '{}\n'})
    required = (('local_matches', 'pose_graph', 'local_matches.json'),)
    result = verify_stage_chain(
        {'local_matches': matches, 'pose_graph': graph},
        required_stages=('local_matches', 'pose_graph'), required_links=required)
    assert len(result['links']) == 1
    with pytest.raises(EvidenceChainError, match='stage set'):
        verify_stage_chain({'local_matches': matches},
                           required_stages=('local_matches', 'pose_graph'))
    with pytest.raises(EvidenceChainError, match='stage links'):
        verify_stage_chain(
            {'local_matches': matches, 'pose_graph': graph},
            required_stages=('local_matches', 'pose_graph'), required_links=())


def test_duplicate_output_producers_are_refused(tmp_path):
    first = _stage(tmp_path / 'first', 'first', {}, {'same.json': '1\n'})
    second = _stage(tmp_path / 'second', 'second', {}, {'same.json': '1\n'})
    with pytest.raises(EvidenceChainError, match='ambiguously produced'):
        verify_stage_chain({'first': first, 'second': second})


def test_mosaic_manifest_products_are_rehashed(tmp_path):
    product = _write(tmp_path / 'mosaic.tif', {'pixels': [1]})
    manifest = _write(tmp_path / 'mosaic_manifest.json', {'outputs': {
        product.name: {'bytes': product.stat().st_size, 'sha256': sha256_file(product)}}})
    assert verify_manifest_outputs(manifest)[product.name]['bytes'] == product.stat().st_size
    product.write_text('{}', encoding='utf-8')
    with pytest.raises(EvidenceChainError, match='no longer matches the mosaic manifest'):
        verify_manifest_outputs(manifest)


def test_native_inspection_tiles_are_rehashed(tmp_path):
    tile = _write(tmp_path / 'native_tiles' / 'feature' / 'tile.json', {'pixels': [1]})
    summary = _write(tmp_path / 'diagnostic_inspection_summary.json', {'features': [{
        'native_tiles': [{'file': str(tile.relative_to(tmp_path)),
                          'bytes': tile.stat().st_size, 'sha256': sha256_file(tile)}]}]})
    assert verify_inspection_tiles(summary) == 1
    tile.write_text('{}', encoding='utf-8')
    with pytest.raises(EvidenceChainError, match='no longer matches its summary'):
        verify_inspection_tiles(summary)


def test_a_stage_predating_the_chain_cannot_be_cited(tmp_path):
    directory = tmp_path / 'old'
    directory.mkdir()
    (directory / 'pose_graph.json').write_text('{}\n', encoding='utf-8')
    with pytest.raises(EvidenceChainError, match='predates the chain'):
        verify_stage_chain({'pose_graph': directory})


def test_an_input_no_stage_produced_is_recorded_without_a_link(tmp_path):
    """Immutable wall truth enters from outside; it has a digest but no upstream."""
    wall = tmp_path / 'wall_texture.json'
    wall.write_text('{"diagnostic_wall": {}}\n', encoding='utf-8')
    inspection = _stage(tmp_path / 'inspection', 'diagnostic_inspection',
                        {'diagnostic_wall_manifest': artifact(wall)},
                        {'diagnostic_inspection_summary.json': '{}\n'})
    assert verify_stage_chain({'diagnostic_inspection': inspection})['links'] == []


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__]))
