#!/usr/bin/env python3
"""Freeze M2 leak-area bounds with the unchanged M1 fan curve; no fit to outcomes."""
import argparse
import hashlib
import json
from pathlib import Path

from validate_vacuum_physics import read_physics, reference_pressure, reference_rate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    physics = read_physics()
    source = root / physics['calibration']['requirement_source']
    requirement = json.loads(source.read_text())['nominal']['rounded_pressure_pa']
    cal = physics['calibration']
    baseline = cal['residual_area_m2'] + cal['reference_area_m2'] * cal['maximum_normal_opening_multiplier']
    pumping = -reference_rate(requirement, [1, 1, 1], 0, physics)
    per_area = reference_rate(requirement, [1, 1, 1], 1, physics) + pumping
    maximum_area = pumping/per_area
    cases = []
    # Declared trapezoid: 5 mm opening, 4 mm floor. Uncovered channel bound;
    # finite skirt conformity may reduce it, but this budget claims no credit.
    for crossings in [0, 2, 4]:
        for depth in ([0] if crossings == 0 else [3, 5]):
            channel_mm2 = crossings * 4.5 * depth
            area = baseline + channel_mm2*1e-6
            cases.append(dict(crossings=crossings, depth_mm=depth,
                              geometric_channel_mm2=channel_mm2, total_area_mm2=area*1e6,
                              equilibrium_delta_pa=reference_pressure([1, 1, 1], area, physics),
                              remaining_area_at_requirement_mm2=(maximum_area-area)*1e6))
    report = dict(
        scope='Pre-T2 area budget, not coupled rough-wall or hardware validation',
        decision='Keep existing fan curves; report boundary rather than retune per case',
        pressure_requirement_pa=requirement, baseline_area_mm2=baseline*1e6,
        maximum_area_at_requirement_mm2=maximum_area*1e6,
        pressure_budget_passed=all(c['remaining_area_at_requirement_mm2'] >= 0 for c in cases),
        assumptions=[
            'Baseline 180 mm2 is the conservative normal M1 opening, not measured hardware leakage.',
            'Add connected groove channels to baseline; no silent absorption into its existing allowance.',
            'Trapezoid section is depth*(opening+floor)/2; shared openings must not be counted twice.',
            'Mild roughness adds no arbitrary leakage while the seal follows it; actual loss of conformity must be computed.',
            'Cracks, extra lift, transients and changed wheel/skirt loads are not certified by this static budget.',
            '6100 Pa is the old rigid-flat nominal capability envelope, pending M2 load recomputation.'],
        cases=cases, input_sha256={
            'vacuum_physics.yaml': hashlib.sha256((root/'src/climbot_gazebo/config/vacuum_physics.yaml').read_bytes()).hexdigest(),
            'requirement_summary.json': hashlib.sha256(source.read_bytes()).hexdigest()})
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: report[k] for k in ['pressure_requirement_pa', 'maximum_area_at_requirement_mm2', 'pressure_budget_passed']}))


if __name__ == '__main__':
    main()
