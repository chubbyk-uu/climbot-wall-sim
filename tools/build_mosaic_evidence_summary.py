#!/usr/bin/env python3
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
Assemble a formal mosaic evidence summary out of the chain instead of by hand.

The summary this replaces was typed.  Its provenance block named one commit for
a chain that ran across several, and the commit it named did not contain the
code that produced the summary's own fields -- a claim no program could have
made, and none had.  So every number here is read out of a stage's published
output, and the provenance is the stages' own records with their links checked.

Only the limitations are written by a person, because judging what a single
collection does not yet show is not something the files can do.
"""

import argparse
import json
from pathlib import Path
import sys

from climbot_common.atomic import write_json
from climbot_common.hashing import sha256_file
from climbot_mosaic.evidence_chain import (
    EvidenceChainError,
    resolve_frozen_tasks,
    verify_inspection_tiles,
    verify_manifest_outputs,
    verify_stage_chain,
)
from climbot_mosaic.stage_provenance import (
    artifact,
    stage_record,
    STAGE_PROVENANCE_FORMAT_VERSION,
)

#: The stages that actually produced the measurements quoted by a formal P2-06
#: summary. overlap_candidates is intentionally absent: its JSON is a parallel
#: human-readable diagnostic, while local_matches rederives candidates itself.
STAGE_ARGUMENTS = (
    ('matches_dir', 'local_matches'),
    ('pose_graph_dir', 'pose_graph'),
    ('mosaic_dir', 'wall_mosaic'),
    ('truth_dir', 'diagnostic_truth'),
    ('inspection_dir', 'diagnostic_inspection'),
)

FORMAL_STAGES = tuple(stage for _, stage in STAGE_ARGUMENTS)
FORMAL_LINKS = (
    ('local_matches', 'pose_graph', 'local_matches.json'),
    ('pose_graph', 'wall_mosaic', 'optimized_poses.json'),
    ('pose_graph', 'wall_mosaic', 'pose_graph.json'),
    ('wall_mosaic', 'diagnostic_inspection', 'mosaic_manifest.json'),
    ('wall_mosaic', 'diagnostic_truth', 'mosaic_manifest.json'),
)


def _document(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def _variant(truth, name):
    """Quote one mosaic variant's independently measured geometry."""
    variant = truth['variants'][name]
    similarity = variant['similarity']
    seam = variant['seam_quality']
    gradient = seam['gradient_excess_gray_per_pixel']
    on_excess = gradient['on_hard_cut']['excess_over_truth']
    off_excess = gradient['off_hard_cut_baseline']['excess_over_truth']
    result = {
        'accepted_anchor_count': variant['accepted_anchor_count'],
        'absolute_anchor_offset_median_mm': variant['absolute_anchor_offset_m']['median'] * 1000.0,
        'absolute_anchor_offset_p95_mm': variant['absolute_anchor_offset_m']['p95'] * 1000.0,
        'local_residual_p95_mm': similarity['local_residual_p95'] * 1000.0,
        'scale_error_ppm': similarity['scale_error_ppm'],
        'yaw_error_deg': similarity['yaw_error_deg'],
        'seam_quality': {
            'seam_adjacency_count': seam['seam_adjacency_count'],
            'gradient_excess_p95_gray_per_pixel': on_excess['p95'],
            'off_seam_gradient_excess_p95_gray_per_pixel': off_excess['p95'],
            'on_to_off_gradient_excess_p95_ratio': gradient[
                'on_to_off_excess_p95_ratio'],
        },
    }
    return result


def _require_current_seam_contract(mosaic, truth):
    """Refuse old evidence that lacks the required, interpretable seam measurements."""
    if mosaic.get('mosaic_format_version') != 3:
        raise ValueError('mosaic is not format version 3 with per-variant hard-cut coverage')
    if truth.get('diagnostic_truth_format_version') != 3:
        raise ValueError('truth summary is not format version 3 with the seam baseline')
    for name in ('pose_only', 'optimized'):
        try:
            seam = truth['variants'][name]['seam_quality']
            gradient = seam['gradient_excess_gray_per_pixel']
            on = gradient['on_hard_cut']['excess_over_truth']
            off = gradient['off_hard_cut_baseline']['excess_over_truth']
            if seam['seam_adjacency_count'] <= 0 or on['count'] <= 0 or off['count'] <= 0:
                raise ValueError('empty seam or off-seam gradient distribution')
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f'{name} lacks the required seam gradient contract: {error}') from error


def _verify_archive_content(directories, frozen):
    """
    Bind the acquisition content records to the archives this summary rests on.

    The content gate is not one of the five measurement stages, but P2-06 leans
    on it: it is the only checkable basis for saying two collections photograph
    the same wall. A record nobody verifies against the chain is the same kind
    of claim as a hand-typed provenance block, so every frozen task's archive
    manifest must appear in a supplied record -- either as a gated input, in
    which case that run has to have passed, or as the record's own reference,
    in which case it is the baseline the others were judged against.
    """
    if not directories:
        raise EvidenceChainError('at least one archive content record is required')
    wanted = {task['archive_manifest_sha256']: task['task_id'] for task in frozen['tasks']}
    seen = {}
    records = {}
    for directory in directories:
        directory = Path(directory)
        # Checked here rather than through verify_stage_chain, which requires
        # distinct input artifact names: every archive contributes a file called
        # manifest.json, so the names collide by construction. The same
        # conditions are enforced -- stage identity, provenance format, a clean
        # source tree, and the declared outputs rehashed off disk.
        record = _document(directory / 'stage_provenance.json')
        if record.get('stage') != 'archive_content':
            raise EvidenceChainError(f'{directory} is not an archive content record')
        if record.get('stage_provenance_format_version') != STAGE_PROVENANCE_FORMAT_VERSION:
            raise EvidenceChainError(f'{directory} has an unsupported provenance format')
        if not record.get('git', {}).get('traceable'):
            raise EvidenceChainError(
                f'{directory} was not produced from a traceable source tree')
        for name, declared in record['outputs'].items():
            path = directory / name
            if not path.is_file() or sha256_file(path) != declared['sha256']:
                raise EvidenceChainError(f'{directory} output {name} does not match its record')
        summary = _document(directory / 'archive_content_summary.json')
        if summary.get('archive_content_format_version') != 2:
            raise EvidenceChainError(
                f'{directory} is not an archive content record of format version 2')
        if summary.get('all_runs_match_reference') is not True:
            raise EvidenceChainError(f'{directory} does not report all runs matching')
        inputs = record['inputs']
        for entry in inputs['archive_manifests']:
            digest = entry['manifest']['sha256']
            if digest not in wanted:
                continue
            verdict = summary['runs'][entry['run']]['against_reference']
            if not verdict['passed']:
                raise EvidenceChainError(
                    f"archive {entry['run']} failed its content gate: {verdict['failures']}")
            seen[digest] = {'run': entry['run'], 'role': 'gated', 'passed': True}
        reference = inputs.get('reference_manifest')
        if reference and reference['manifest']['sha256'] in wanted:
            seen.setdefault(reference['manifest']['sha256'],
                            {'run': reference['run'], 'role': 'reference'})
        records[directory.name] = {
            'stage_provenance': record, 'all_runs_match_reference':
            summary.get('all_runs_match_reference')}
    missing = sorted(task for digest, task in wanted.items() if digest not in seen)
    if missing:
        raise EvidenceChainError(
            'no archive content record covers the frozen archives for: ' + ', '.join(missing))
    return {'records': records,
            'frozen_archives': {wanted[digest]: value for digest, value in seen.items()}}


def main() -> int:
    """Write one formal summary whose every number came out of the chain."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mosaic-dir', required=True, type=Path)
    parser.add_argument('--truth-dir', required=True, type=Path)
    parser.add_argument('--inspection-dir', required=True, type=Path)
    parser.add_argument(
        '--candidates-dir', type=Path,
        help='optional standalone diagnostic; verified but not represented as a pipeline link')
    parser.add_argument('--matches-dir', type=Path)
    parser.add_argument('--pose-graph-dir', type=Path)
    parser.add_argument('--input-run', action='append', required=True, type=Path)
    parser.add_argument(
        '--archive-content-dir', action='append', required=True, type=Path,
        help='published summarize_archive_content record covering the frozen archives')
    parser.add_argument('--archive-root', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--status', required=True)
    parser.add_argument('--limitation', action='append', default=[],
                        help='a thing this evidence does not show; repeatable')
    arguments = parser.parse_args()

    stages = {stage: getattr(arguments, name)
              for name, stage in STAGE_ARGUMENTS if getattr(arguments, name) is not None}
    try:
        chain = verify_stage_chain(
            stages, required_stages=FORMAL_STAGES, required_links=FORMAL_LINKS,
            require_traceable=True)
        mosaic_products = verify_manifest_outputs(
            arguments.mosaic_dir / 'mosaic_manifest.json')
        inspection_tiles = verify_inspection_tiles(
            arguments.inspection_dir / 'diagnostic_inspection_summary.json')
        candidate_diagnostic = None
        if arguments.candidates_dir is not None:
            candidate_diagnostic = verify_stage_chain(
                {'overlap_candidates': arguments.candidates_dir},
                required_stages=('overlap_candidates',), required_links=(),
                require_traceable=True)['stages']['overlap_candidates']
        frozen = resolve_frozen_tasks(
            arguments.mosaic_dir, tuple(arguments.input_run), arguments.archive_root)
        archive_content = _verify_archive_content(arguments.archive_content_dir, frozen)
    except EvidenceChainError as error:
        print(f'evidence chain refused: {error}', file=sys.stderr)
        return 2

    mosaic = _document(arguments.mosaic_dir / 'mosaic_manifest.json')
    truth = _document(arguments.truth_dir / 'diagnostic_truth_summary.json')
    inspection = _document(arguments.inspection_dir / 'diagnostic_inspection_summary.json')
    try:
        _require_current_seam_contract(mosaic, truth)
    except ValueError as error:
        print(f'evidence chain refused: {error}', file=sys.stderr)
        return 2
    coverage = inspection['visible_feature_coverage']

    assembly = stage_record(
        'evidence_summary',
        {'schema_version': 4, 'status': arguments.status,
         'limitations': list(arguments.limitation),
         'formal_stages': list(FORMAL_STAGES),
         'formal_links': [list(link) for link in FORMAL_LINKS]},
        {'stage_provenance': {
            stage: artifact(Path(stages[stage]) / 'stage_provenance.json')
            for stage in FORMAL_STAGES},
         'mosaic_manifest': artifact(arguments.mosaic_dir / 'mosaic_manifest.json'),
         'truth_summary': artifact(
             arguments.truth_dir / 'diagnostic_truth_summary.json'),
         'inspection_summary': artifact(
             arguments.inspection_dir / 'diagnostic_inspection_summary.json')},
        {})
    if assembly['git'].get('traceable') is not True:
        print('evidence chain refused: summary generator source tree is not traceable.',
              file=sys.stderr)
        return 2

    summary = {
        'schema_version': 4,
        'status': arguments.status,
        'acquisition': {
            'frozen_tasks': [
                {'task_id': task['task_id'], 'source_run_id': task['source_run_id'],
                 'archive_manifest_sha256': task['archive_manifest_sha256'],
                 'processing_manifest_sha256': task['processing_manifest_sha256'],
                 'coverage_region_m': [list(point) for point in task['coverage_region_m']],
                 'camera_footprint_m': list(task['camera_footprint_m'])}
                for task in frozen['tasks']],
            'frame_count': mosaic['input_summary']['frame_count'],
        },
        'joint': {
            'frame_count': mosaic['input_summary']['frame_count'],
            'mosaic_manifest_sha256': sha256_file(
                arguments.mosaic_dir / 'mosaic_manifest.json'),
            'truth_summary_sha256': sha256_file(
                arguments.truth_dir / 'diagnostic_truth_summary.json'),
            'optimized': _variant(truth, 'optimized'),
            'pose_only': _variant(truth, 'pose_only'),
            'optimized_not_worse_p95_anchor_offset':
                truth['optimized_not_worse_p95_anchor_offset'],
            'overlap_gray_std_mean': {
                'optimized': mosaic['quality']['optimized']['gray_std_mean'],
                'pose_only': mosaic['quality']['pose_only']['gray_std_mean'],
            },
        },
        'inspection': {
            'inspection_summary_sha256': sha256_file(
                arguments.inspection_dir / 'diagnostic_inspection_summary.json'),
            'pixel_scale_mm': inspection['pixel_scale_m_per_pixel'] * 1000.0,
            'feature_counts': inspection['feature_counts'],
            'inspection_region_m': inspection['inspection_region_m'],
            'planned_scan_footprints_m': inspection['planned_scan_footprints_m'],
            'safe_pose_camera_envelopes_m': inspection['safe_pose_camera_envelopes_m'],
            'visible_feature_coverage': coverage,
        },
        'evidence': {
            'all_visible_feature_pixels_covered':
                coverage['all_visible_feature_pixels_covered'],
            'all_inspection_region_feature_pixels_covered':
                coverage['all_inspection_region_feature_pixels_covered'],
            'optimized_not_worse_p95_anchor_offset':
                truth['optimized_not_worse_p95_anchor_offset'],
            'every_stage_published_its_own_provenance': True,
            'stage_links_verified': len(chain['links']),
            'mosaic_products_rehashed': len(mosaic_products),
            'inspection_tiles_rehashed': inspection_tiles,
        },
        'provenance': {
            'stages': {stage: chain['stages'][stage] for stage in sorted(chain['stages'])},
            'links': chain['links'],
            'summary_generation': assembly,
            'archive_content': archive_content,
            'standalone_diagnostics': ({
                'overlap_candidates': candidate_diagnostic,
            } if candidate_diagnostic is not None else {}),
        },
        'limitations': list(arguments.limitation),
    }
    write_json(arguments.output, summary)
    print(json.dumps({'status': 'written', 'output': str(arguments.output),
                      'stages': sorted(stages), 'links': len(chain['links'])},
                     ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
