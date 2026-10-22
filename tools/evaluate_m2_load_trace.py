#!/usr/bin/env python3
"""Evaluate every recorded physics step in the frozen final-generation score window."""
import argparse
import hashlib
import json
from pathlib import Path

from m2_load_criterion import evaluate
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--generation', type=int, default=0,
                        help='frozen expected generation; reject unexpected resets')
    parser.add_argument('--mu', type=float,
                        help='scoring coefficient; does not change recorded physics')
    args = parser.parse_args()
    meta = json.loads(Path(str(args.input)+'.json').read_text())
    recorded_mu = meta.get('recorded_physics_mu')
    if args.mu is None and recorded_mu is None:
        raise ValueError('historical trace has no friction metadata; specify --mu explicitly')
    mu = recorded_mu if args.mu is None else args.mu
    count = len(meta['columns'])
    if args.input.stat().st_size % (count*8):
        raise ValueError('truncated binary load record')
    raw = np.memmap(args.input, dtype=meta['dtype'], mode='r').reshape(-1, count)
    columns = {name: raw[:, i] for i, name in enumerate(meta['columns'])}
    generation = args.generation
    if columns['vac_generation'].max() != generation:
        raise ValueError('unexpected world reset or missing frozen generation')
    selected = columns['vac_generation'] == generation
    release = columns['vac_release_time_s'][selected].max()
    if release < 0:
        raise ValueError('fixture never released')
    selected &= columns['sim_time_s'] >= release+1
    c = {name: value[selected] for name, value in columns.items()}
    result = {'scope': 'All final-generation physics steps from release+1 to recorder end',
              'generation': int(generation), 'raw_rows': len(raw),
              'scored_rows': int(selected.sum()),
              'recorded_physics_mu': recorded_mu,
              'scoring_mu': mu,
              'coefficient_source': 'explicit_argument' if args.mu is not None else 'trace_metadata',
              'counterfactual': None if recorded_mu is None else mu != recorded_mu,
              'load_acceptance': evaluate(c, release, mu=mu),
              'minimum_pressure_pa': float(c['vac_delta_pa'].min()),
              'runtime_failed': bool(np.any(c['vac_runtime_failed'])),
              'startup_failed': bool(np.any(c['vac_startup_failed'])),
              'maximum_stop_normal_n': float(c['vac_skirt_stop_previous_normal_n'].max()),
              'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    # Streaming hash avoids copying a full large mission into memory.
    h = hashlib.sha256()
    with args.input.open('rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            h.update(block)
    result['trace_sha256'] = h.hexdigest()
    result['passed'] = (result['load_acceptance']['passed'] and
                        not result['runtime_failed'] and not result['startup_failed'] and
                        result['maximum_stop_normal_n'] == 0)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: result[k] for k in ['passed', 'scored_rows', 'minimum_pressure_pa']}))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
