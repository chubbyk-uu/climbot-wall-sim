#!/usr/bin/env python3
"""Independent raw-data audit and a static load figure for an S3 evidence batch."""
import argparse
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def audit_frames(path):
    """Check exact paused values, with the documented first refresh separate."""
    previous = None
    stable = True
    comparisons = 0
    resets = set()
    count = 0
    with gzip.open(path, 'rt') as stream:
        for row in csv.DictReader(stream):
            count += 1
            resets.add(row['vac_generation'])
            if row['paused'] == '1':
                # CSV has 17-digit round-trip values. String equality here
                # checks every recorded scalar, not Pose3d's tolerance-based ==.
                snapshot = {k: v for k, v in row.items() if k not in ('iteration', 'paused')}
                if previous is not None:
                    comparisons += 1
                    stable &= snapshot == previous
                previous = snapshot
            else:
                previous = None
    return {'frames': count, 'generations': sorted(resets),
            'exact_paused_comparisons': comparisons, 'exact_pause_values_unchanged': stable}


def audit_contacts(path):
    """Keep native geometry and force conventions visible without new thresholds."""
    maximum_angle = maximum_depth = maximum_pair_error = 0.
    minimum_depth = math.inf
    rows = 0
    with gzip.open(path, 'rt') as stream:
        for row in csv.DictReader(stream):
            rows += 1
            normal = [float(row['normal_' + axis]) for axis in 'xyz']
            norm = math.sqrt(sum(x*x for x in normal))
            angle = math.degrees(math.acos(min(1., abs(normal[0])/norm))) if norm else 180.
            maximum_angle = max(maximum_angle, angle)
            depth = float(row['depth'])
            minimum_depth = min(minimum_depth, depth)
            maximum_depth = max(maximum_depth, depth)
            error = math.sqrt(sum((float(row['body1_f' + axis]) +
                                   float(row['body2_f' + axis]))**2 for axis in 'xyz'))
            maximum_pair_error = max(maximum_pair_error, error)
    return {'raw_contact_points': rows, 'maximum_normal_deviation_deg': maximum_angle,
            'minimum_depth_m': minimum_depth if rows else None,
            'maximum_depth_m': maximum_depth, 'maximum_action_reaction_error_n': maximum_pair_error}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batch', type=Path)
    args = parser.parse_args();root = args.batch
    summary = json.loads((root / 's3_summary.json').read_text())
    output = {'audit_script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'native_run_source_revision': json.loads((root / 'source_manifest.json').read_text())['revision'],
              'cases': {}}
    for directory in sorted(root.iterdir()):
        frames = directory / 'native/frames.csv.gz'
        if frames.exists():
            output['cases'][directory.name] = {
                **audit_frames(frames), **audit_contacts(directory / 'native/contacts.csv.gz')}
    native = output['cases']['lifecycle']
    guard = json.loads((root / 'mutual_guard/guard_summary.json').read_text())
    output['checks'] = {
        'all_paused_scalars_exactly_frozen': native['exact_pause_values_unchanged'] and
            native['exact_paused_comparisons'] == 99,
        'both_reset_generations_observed': native['generations'] == ['0', '1'],
        'mutual_force_path_rejected': guard['exit_code'] == 2 and guard['explicit_mutual_rejection'],
    }
    headings = list(range(0, 360, 45))
    fig, axes = plt.subplots(2, 1, figsize=(9, 7), constrained_layout=True)
    colors = ['#0067a6', '#d65f00', '#217a42']
    for i, name in enumerate(['Left drive', 'Right drive', 'Ball caster']):
        for mode, style in [('fixed_pressure', '--'), ('fan_ode', '-')]:
            values = [summary[f'{mode}_h{h}']['steady_normal_mean_n'][i] for h in headings]
            axes[0].plot(headings, values, style, color=colors[i], marker='o' if mode == 'fan_ode' else None,
                         label=name + (' C' if mode == 'fan_ode' else ' B'))
    axes[0].set(ylabel='Mean native normal load [N]',
                title='Flat wall, rigid wheels: release +2 to +62 s')
    axes[0].legend(ncol=3, fontsize=8);axes[0].grid(alpha=.25)
    axes[1].plot(headings, [summary[f'fan_ode_h{h}']['minimum_pressure_pa']/1000 for h in headings],
                 'o-', label='Static per-wheel requirement')
    axes[1].plot(headings, [summary[f'fan_ode_h{h}']['steady_delta_pa']/1000 for h in headings],
                 'o-', label='C measured steady pressure')
    revised = json.loads((root / 'revised_design.json').read_text())
    axes[1].axhline(revised['worst']['conservative']['minimum_pressure_pa']/1000,
                    color='#d65f00', linestyle=':', label='Offline motion envelope (S5 pending)')
    axes[1].set(xlabel='Heading in wall coordinates [deg]', ylabel='Pressure difference [kPa]')
    axes[1].legend(fontsize=8);axes[1].grid(alpha=.25)
    fig.savefig(root / 'static_loads.svg')
    fig.savefig(root / 'static_loads.png', dpi=150);plt.close(fig)
    (root / 'raw_audit.json').write_text(json.dumps(output, indent=2, allow_nan=False)+'\n')
    print(json.dumps(output['checks']))
    return 0 if all(output['checks'].values()) else 2


if __name__ == '__main__':
    raise SystemExit(main())
