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

"""The formal summary must prove its own assembly as well as its inputs."""

import importlib.util
import json
from pathlib import Path
import sys

from climbot_common.hashing import sha256_file


def _load_tool():
    path = Path(__file__).resolve().parents[3] / 'tools' / 'build_mosaic_evidence_summary.py'
    spec = importlib.util.spec_from_file_location('build_mosaic_evidence_summary', path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _write(path, document):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, sort_keys=True) + '\n', encoding='utf-8')
    return path


def _stage(directory, name, inputs, outputs):
    directory.mkdir(parents=True)
    for filename, document in outputs.items():
        _write(directory / filename, document)
    _write(directory / 'stage_provenance.json', {
        'stage_provenance_format_version': 1,
        'stage': name,
        'git': {'commit': 'a' * 40, 'source_modified': False, 'traceable': True},
        'parameters': {},
        'inputs': inputs,
        'outputs': {filename: {'name': filename, 'sha256': sha256_file(directory / filename)}
                    for filename in outputs},
    })
    return directory


def _artifact(path):
    return {'name': path.name, 'sha256': sha256_file(path)}


def test_formal_summary_records_its_generator_and_only_the_causal_chain(
        tmp_path, monkeypatch):
    tool = _load_tool()
    run_id = 'b' * 32
    region = [{'x': 0.0, 'y': 0.0}, {'x': 1.0, 'y': 0.0},
              {'x': 1.0, 'y': 1.0}, {'x': 0.0, 'y': 1.0}]
    archive = _write(
        tmp_path / 'archives' / 'task' / f'r000001_20260830T000000Z_{run_id}' / 'manifest.json',
        {'task': {'task_id': 'task', 'coverage_region': region, 'motion_region': region,
                  'sweep_direction': 1,
                  'waypoints': [{'position': {'x': 0.1, 'y': 0.5}},
                                {'position': {'x': 0.9, 'y': 0.5}}],
                  'segment_types': [1], 'detection_width_m': 0.5,
                  'detection_length_m': 0.25, 'detection_forward_offset_m': 0.0}})
    run = tmp_path / 'processed'
    processing = _write(run / 'processing_manifest.json', {
        'source_archive': {'run_id': run_id, 'manifest_sha256': sha256_file(archive)}})
    processed_digest = sha256_file(processing)

    matches = _stage(tmp_path / 'matches', 'local_matches', {}, {
        'local_matches.json': {'match_summary': {'candidate_count': 1}}})
    graph = _stage(tmp_path / 'graph', 'pose_graph',
                   {'local_matches': _artifact(matches / 'local_matches.json')}, {
                       'pose_graph.json': {}, 'optimized_poses.json': {}})

    mosaic = tmp_path / 'mosaic'
    mosaic.mkdir()
    product = _write(mosaic / 'tiny_product.json', {'pixels': [1]})
    manifest = _write(mosaic / 'mosaic_manifest.json', {
        'input_summary': {'frame_count': 1,
                          'processing_manifest_sha256': [processed_digest]},
        'outputs': {product.name: {'bytes': product.stat().st_size,
                                   'sha256': sha256_file(product)}},
        'quality': {'optimized': {'gray_std_mean': 1.0},
                    'pose_only': {'gray_std_mean': 2.0}},
    })
    _write(mosaic / 'stage_provenance.json', {
        'stage_provenance_format_version': 1, 'stage': 'wall_mosaic',
        'git': {'traceable': True}, 'parameters': {},
        'inputs': {'pose_graph': _artifact(graph / 'pose_graph.json'),
                   'optimized_poses': _artifact(graph / 'optimized_poses.json')},
        'outputs': {'mosaic_manifest.json': _artifact(manifest)}})

    variant = {'accepted_anchor_count': 1,
               'absolute_anchor_offset_m': {'median': 0.001, 'p95': 0.002},
               'similarity': {'local_residual_p95': 0.0002, 'scale_error_ppm': 1.0,
                              'yaw_error_deg': 0.01}}
    truth = _stage(tmp_path / 'truth', 'diagnostic_truth',
                   {'mosaic_manifest': _artifact(manifest)}, {
                       'diagnostic_truth_summary.json': {
                           'variants': {'optimized': variant, 'pose_only': variant},
                           'optimized_not_worse_p95_anchor_offset': True}})
    inspection = _stage(tmp_path / 'inspection', 'diagnostic_inspection',
                        {'mosaic_manifest': _artifact(manifest)}, {
                            'diagnostic_inspection_summary.json': {
                                'pixel_scale_m_per_pixel': 0.001,
                                'feature_counts': {'declared': 1},
                                'inspection_region_m': [[0.0, 0.0], [1.0, 0.0],
                                                        [1.0, 1.0], [0.0, 1.0]],
                                'planned_scan_footprints_m': [],
                                'safe_pose_camera_envelopes_m': [],
                                'features': [],
                                'visible_feature_coverage': {
                                    'all_visible_feature_pixels_covered': True,
                                    'all_inspection_region_feature_pixels_covered': True}}})

    def assembly(stage, parameters, inputs, outputs):
        return {'stage': stage, 'git': {'commit': 'c' * 40, 'source_modified': False,
                                        'traceable': True},
                'parameters': parameters, 'inputs': inputs, 'outputs': outputs}

    monkeypatch.setattr(tool, 'stage_record', assembly)
    output = tmp_path / 'summary.json'
    monkeypatch.setattr(sys, 'argv', [
        str(Path(tool.__file__)), '--matches-dir', str(matches),
        '--pose-graph-dir', str(graph), '--mosaic-dir', str(mosaic),
        '--truth-dir', str(truth), '--inspection-dir', str(inspection),
        '--input-run', str(run), '--archive-root', str(tmp_path / 'archives'),
        '--output', str(output), '--status', 'test-pass'])
    assert tool.main() == 0
    summary = json.loads(output.read_text(encoding='utf-8'))
    assert summary['schema_version'] == 3
    assert summary['provenance']['summary_generation']['git']['traceable'] is True
    assert set(summary['provenance']['stages']) == set(tool.FORMAL_STAGES)
    assert 'overlap_candidates' not in summary['provenance']['stages']
    assert summary['evidence']['mosaic_products_rehashed'] == 1
    assert summary['evidence']['inspection_tiles_rehashed'] == 0
