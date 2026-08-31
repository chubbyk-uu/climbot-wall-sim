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
from climbot_mosaic.stage_provenance import artifact, stage_record

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
    result = {
        'accepted_anchor_count': variant['accepted_anchor_count'],
        'absolute_anchor_offset_median_mm': variant['absolute_anchor_offset_m']['median'] * 1000.0,
        'absolute_anchor_offset_p95_mm': variant['absolute_anchor_offset_m']['p95'] * 1000.0,
        'local_residual_p95_mm': similarity['local_residual_p95'] * 1000.0,
        'scale_error_ppm': similarity['scale_error_ppm'],
        'yaw_error_deg': similarity['yaw_error_deg'],
    }
    if 'seam_quality' in variant:
        seam = variant['seam_quality']
        edge = seam['double_image_edge_displacement_proxy'][
            'dominant_normal_edge_displacement_m']
        result['seam_quality'] = {
            'seam_adjacency_count': seam['seam_adjacency_count'],
            'gradient_excess_p95_gray_per_pixel': seam['gradient_jump_gray_per_pixel'][
                'excess_over_truth']['p95'],
            'double_image_edge_displacement_p95_mm': (
                edge['p95'] * 1000.0 if edge is not None else None),
            'eligible_structural_edge_count': seam['double_image_edge_displacement_proxy'][
                'eligible_structural_edge_count'],
        }
    return result


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
    except EvidenceChainError as error:
        print(f'evidence chain refused: {error}', file=sys.stderr)
        return 2

    mosaic = _document(arguments.mosaic_dir / 'mosaic_manifest.json')
    truth = _document(arguments.truth_dir / 'diagnostic_truth_summary.json')
    inspection = _document(arguments.inspection_dir / 'diagnostic_inspection_summary.json')
    coverage = inspection['visible_feature_coverage']

    assembly = stage_record(
        'evidence_summary',
        {'schema_version': 3, 'status': arguments.status,
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
        'schema_version': 3,
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
