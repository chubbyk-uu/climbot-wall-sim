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
    verify_stage_chain,
)

#: Stage directories in pipeline order.  The first three are required because
#: the summary quotes their numbers; the rest are optional only so a partial
#: rerun can still be summarised, and every one given is checked.
STAGE_ARGUMENTS = (
    ('candidates_dir', 'overlap_candidates'),
    ('matches_dir', 'local_matches'),
    ('pose_graph_dir', 'pose_graph'),
    ('mosaic_dir', 'wall_mosaic'),
    ('truth_dir', 'diagnostic_truth'),
    ('inspection_dir', 'diagnostic_inspection'),
)


def _document(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def _variant(truth, name):
    """Quote one mosaic variant's independently measured geometry."""
    variant = truth['variants'][name]
    similarity = variant['similarity']
    return {
        'accepted_anchor_count': variant['accepted_anchor_count'],
        'absolute_anchor_offset_median_mm': variant['absolute_anchor_offset_m']['median'] * 1000.0,
        'absolute_anchor_offset_p95_mm': variant['absolute_anchor_offset_m']['p95'] * 1000.0,
        'local_residual_p95_mm': similarity['local_residual_p95'] * 1000.0,
        'scale_error_ppm': similarity['scale_error_ppm'],
        'yaw_error_deg': similarity['yaw_error_deg'],
    }


def main() -> int:
    """Write one formal summary whose every number came out of the chain."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mosaic-dir', required=True, type=Path)
    parser.add_argument('--truth-dir', required=True, type=Path)
    parser.add_argument('--inspection-dir', required=True, type=Path)
    parser.add_argument('--candidates-dir', type=Path)
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
        chain = verify_stage_chain(stages)
        frozen = resolve_frozen_tasks(
            arguments.mosaic_dir, tuple(arguments.input_run), arguments.archive_root)
    except EvidenceChainError as error:
        print(f'evidence chain refused: {error}', file=sys.stderr)
        return 2

    mosaic = _document(arguments.mosaic_dir / 'mosaic_manifest.json')
    truth = _document(arguments.truth_dir / 'diagnostic_truth_summary.json')
    inspection = _document(arguments.inspection_dir / 'diagnostic_inspection_summary.json')
    coverage = inspection['visible_feature_coverage']

    summary = {
        'schema_version': 2,
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
            'observable_envelope_m': inspection['observable_envelope_m'],
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
        },
        'provenance': {
            'stages': {stage: chain['stages'][stage] for stage in sorted(chain['stages'])},
            'links': chain['links'],
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
