#!/usr/bin/env python3
# Copyright 2026 jerry
# Licensed under the Apache License, Version 2.0

"""Diagnostic-only per-variant truth when the strict paired evaluator refuses.

Preserves the native 0.10 response threshold and scoring functions, reports all
raw matches, and explicitly records missing paired evidence. This does not
replace evaluate_diagnostic_mosaic or promote its failure to acceptance.
"""

import argparse
import json
from pathlib import Path

from climbot_common.hashing import sha256_file
from climbot_mosaic.diagnostic_truth import (
    _AnchorCropCollector, _anchors, _blocks, _document, _mosaic_grid,
    _ReferenceTileReader, _seam_adjacencies, _stream_variant, _summarize_variant,
    _tile_segments, _variant_matches_from_crops, _wall_grid,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mosaic-dir', type=Path, required=True)
    parser.add_argument('--wall-manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--include-seams', action='store_true')
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError('diagnostic output must be new')
    manifest = _document(args.mosaic_dir / 'mosaic_manifest.json', 'mosaic')
    wall = _document(args.wall_manifest, 'wall')
    grid, truth = _mosaic_grid(manifest), _wall_grid(wall)
    blocks = _blocks(wall)
    anchors = _anchors(wall['diagnostic_wall']['features'], truth, grid, 0.10)
    variants, accepted_sets = {}, []
    tile_size = manifest['grid']['tile_size_px']
    reference = _ReferenceTileReader(args.wall_manifest.parent, blocks, truth, grid)
    for name in ('pose_only', 'optimized'):
        path = args.mosaic_dir / f'mosaic_{name}.tif'
        seam_quality = None
        if args.include_seams:
            seams = _seam_adjacencies(args.mosaic_dir / f'seams_{name}.npz', grid)
            coverage = 'coverage_pose_only_count.tif' if name == 'pose_only' else 'coverage_count.tif'
            seam_quality, crops = _stream_variant(
                path, args.mosaic_dir / coverage, anchors, reference, *seams, grid, tile_size)
        else:
            collector = _AnchorCropCollector(anchors, grid)
            for row, column, image in _tile_segments(path, grid, tile_size):
                collector.add_tile(row, column, image)
            crops = collector.crops()
        matches = _variant_matches_from_crops(crops, anchors, args.wall_manifest.parent,
                                              blocks, truth, grid)
        accepted = {match['id'] for match in matches if match['phase_response'] >= 0.10}
        accepted_sets.append(accepted)
        variants[name] = {'raw_matches': matches, 'accepted_ids': sorted(accepted),
                          'summary': _summarize_variant(matches, accepted)
                          if len(accepted) >= 2 else None,
                          'seam_quality': seam_quality,
                          'master_sha256': sha256_file(path)}
    common = set.intersection(*accepted_sets)
    summary = {'diagnostic_only': True, 'paired_acceptance': False,
               'reason': 'per-variant diagnostics do not substitute for strict common-anchor evidence',
               'minimum_phase_response': 0.10, 'anchor_padding_m': 0.10,
               'common_accepted_ids': sorted(common), 'variants': variants,
               'mosaic_manifest_sha256': sha256_file(args.mosaic_dir / 'mosaic_manifest.json'),
               'wall_manifest_sha256': sha256_file(args.wall_manifest)}
    args.output.write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'diagnostic_only': True, 'common_anchor_count': len(common),
                      'variant_anchor_counts': [len(values) for values in accepted_sets]}))


if __name__ == '__main__':
    main()
