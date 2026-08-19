#!/usr/bin/env python3
"""Compare repeated coverage runs of the same case and report their spread.

PROJECT_GUIDE 15.11 asks for repeated runs at a fixed random seed and a check
that the results agree. What "agree" can mean here is bounded by the plant:
Gazebo's physics step and the ROS executors are not sample-synchronous with
the noise timers, so two runs at the same seed are not the same run and no
metric is expected to repeat bit for bit. What is expected is that the spread
stays far below the acceptance thresholds the same numbers are judged against,
and that changing the seed moves the numbers by more than that spread - a
group that repeats perfectly because no noise was applied would pass the first
half of that and fail the second.

So this reports the spread and checks the preconditions that make it mean
something: the same commit, the same seeds, and every run having completed.
Tolerances are optional and absolute, because the interesting metrics sit near
zero, where a relative bound is noise on a small denominator.

Usage:
  tools/compare_repeatability.py results/coverage_horizontal_rep*_summary.json
  tools/compare_repeatability.py results/coverage_horizontal_rep*_summary.json
      --against results/coverage_horizontal_seedB_summary.json
"""

import argparse
import json
import math
import os
import sys

#: Metric path in the summary, label, and the unit multiplier used to print it.
#: These are the numbers the acceptance criteria are stated in, plus the two
#: schedule figures, which are the ones that would move first if the timing of
#: the run were what varied.
METRICS = (
    ('coverage/ratio', 'coverage ratio', 1.0, ''),
    ('execution_quality/maximum_endpoint_error_m',
     'max endpoint error', 1000.0, 'mm'),
    ('execution_quality/maximum_horizontal_height_drift_m',
     'max height drift', 1000.0, 'mm'),
    ('execution_quality/maximum_turn_end_heading_error_deg',
     'max turn-end heading', 1.0, 'deg'),
    ('execution_quality/maximum_heading_compensation_deg',
     'max heading compensation', 1.0, 'deg'),
    ('execution_quality/actual_to_planned_length_ratio',
     'actual/planned length', 1.0, ''),
    ('scan_line_spacing/maximum_scan_line_spacing_error_m',
     'max spacing error', 1000.0, 'mm'),
    ('elapsed_time_s', 'elapsed', 1.0, 's'),
    ('schedule/schedule_lag_max_s', 'schedule lag max', 1000.0, 'ms'),
    ('schedule/schedule_lag_min_s', 'schedule lag min', 1000.0, 'ms'),
)


#: Maximum range a group of repeats may show, per metric, in summary units.
#: Derived from the acceptance thresholds the same metrics are judged against
#: rather than from measured spread, so this is a bound the runs have to meet
#: and not a restatement of what they happened to do: a quarter of the
#: threshold, which leaves a regression three quarters of the budget before it
#: reaches the criterion itself. Metrics with no acceptance threshold get a
#: bound in the same spirit - a few percent of the quantity. The spread these
#: were measured against is in results/README.md.
DEFAULT_TOLERANCES = {
    # Acceptance: >= 0.95. A ratio that varies at all across repeats is worth
    # seeing, so this one is far tighter than a quarter of its own budget.
    'coverage/ratio': 0.005,
    # Acceptance: <= 0.030 m.
    'execution_quality/maximum_endpoint_error_m': 0.0075,
    # Acceptance: <= 0.030 m.
    'execution_quality/maximum_horizontal_height_drift_m': 0.0075,
    # Acceptance: <= 2.0 deg.
    'execution_quality/maximum_turn_end_heading_error_deg': 0.5,
    # Acceptance: <= 0.020 m.
    'scan_line_spacing/maximum_scan_line_spacing_error_m': 0.005,
    # No acceptance threshold on these three; bounded as a fraction of the
    # quantity itself, at the scale the schedule is planned and judged on.
    'execution_quality/maximum_heading_compensation_deg': 0.5,
    'execution_quality/actual_to_planned_length_ratio': 0.01,
    'elapsed_time_s': 5.0,
    'schedule/schedule_lag_max_s': 0.020,
    'schedule/schedule_lag_min_s': 0.020,
}


def lookup(summary, path):
    """Return a slash-separated field, or None where any level is missing."""
    node = summary
    for key in path.split('/'):
        if not isinstance(node, dict) or key not in node:
            return None
        node = node[key]
    return node if isinstance(node, (int, float)) else None


def load(paths):
    """Read every summary, keeping the file name for the report."""
    runs = []
    for path in paths:
        with open(path, encoding='utf-8') as handle:
            runs.append((os.path.basename(path), json.load(handle)))
    return runs


def seeds(summary):
    """Return the seeds the noise sources reported, or None where unknown."""
    sources = (summary.get('provenance') or {}).get('noise_sources') or {}
    if not sources:
        return None
    found = {}
    for name, values in sources.items():
        found[name] = None if values is None else values.get('random_seed')
    return found


def spread(values):
    """Return the mean, the sample standard deviation, and the range."""
    count = len(values)
    mean = sum(values) / count
    if count > 1:
        variance = sum((value - mean) ** 2 for value in values) / (count - 1)
    else:
        variance = 0.0
    return mean, math.sqrt(variance), max(values) - min(values)


def check_preconditions(runs, report):
    """State whether this group is a repeatability group at all."""
    ok = True
    tasks = {run.get('task_id') for _, run in runs}
    if len(tasks) != 1:
        report.append(
            '  FAIL  these are different cases, not repeats: %s' % sorted(
                str(task) for task in tasks))
        ok = False
    else:
        report.append('  case      %s' % tasks.pop())
    commits = {
        (run.get('provenance') or {}).get('git', {}).get('commit')
        for _, run in runs}
    if len(commits) != 1 or None in commits:
        report.append('  FAIL  runs do not share one commit: %s' % sorted(
            str(commit)[:12] for commit in commits))
        ok = False
    else:
        report.append('  commit    %s' % str(commits.pop())[:12])

    traceable = all(
        (run.get('provenance') or {}).get('git', {}).get('traceable')
        for _, run in runs)
    if not traceable:
        report.append(
            '  FAIL  at least one run was produced from a modified tree')
        ok = False

    seed_sets = [seeds(run) for _, run in runs]
    if any(entry is None for entry in seed_sets):
        report.append(
            '  WARN  at least one run predates noise-source provenance; '
            'the seeds it used are not recorded')
    elif any(None in entry.values() for entry in seed_sets):
        report.append(
            '  FAIL  a noise source could not be asked what seed it used')
        ok = False
    elif any(entry != seed_sets[0] for entry in seed_sets):
        report.append('  FAIL  runs used different seeds: %s' % seed_sets)
        ok = False
    else:
        report.append('  seeds     %s' % ', '.join(
            '%s=%s' % item for item in sorted(seed_sets[0].items())))

    incomplete = [name for name, run in runs if not run.get('completed')]
    if incomplete:
        report.append('  FAIL  did not complete: %s' % ', '.join(incomplete))
        ok = False
    failed = [name for name, run in runs if not run.get('passed')]
    if failed:
        report.append('  FAIL  did not pass acceptance: %s' % ', '.join(failed))
        ok = False
    return ok


def compare(runs, tolerances, control):
    """Report per-metric spread, and how far a control seed sits from it."""
    report = []
    ok = check_preconditions(runs, report)
    report.append('')
    header = '  %-30s %9s %9s %9s %9s' % (
        'metric', 'mean', 'sd', 'range', 'tolerance')
    if control:
        header += ' %9s' % 'other seed'
    report.append(header)

    for path, label, scale, unit in METRICS:
        values = [lookup(run, path) for _, run in runs]
        if any(value is None for value in values):
            report.append('  %-30s %s' % (label, 'not recorded'))
            continue
        mean, deviation, extent = spread(
            [value * scale for value in values])
        limit = tolerances.get(path)
        line = '  %-30s %9.4f %9.4f %9.4f %9s' % (
            '%s%s' % (label, ' [%s]' % unit if unit else ''),
            mean, deviation, extent,
            '%.4f' % (limit * scale) if limit is not None else '-')
        if limit is not None and extent > limit * scale:
            line += '  OVER'
            ok = False
        if control:
            other = lookup(control, path)
            line += ' %9s' % (
                '-' if other is None else '%.4f' % (other * scale))
            if other is not None and extent > 0.0:
                line += '  (%.1f x spread)' % (
                    abs(other * scale - mean) / extent)
        report.append(line)
    return ok, report


def main():
    """Compare one group of repeats, optionally against a different seed."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('summaries', nargs='+', help='summary JSON files')
    parser.add_argument(
        '--against', default=None,
        help='a summary from a different seed, reported alongside the spread')
    parser.add_argument(
        '--tolerance', action='append', default=[], metavar='PATH=VALUE',
        help='override the allowed range for one metric, in the summary unit')
    parser.add_argument(
        '--no-default-tolerances', action='store_true',
        help='report the spread without judging it')
    arguments = parser.parse_args()

    tolerances = dict(DEFAULT_TOLERANCES)
    if arguments.no_default_tolerances:
        tolerances = {}
    for entry in arguments.tolerance:
        path, _, value = entry.partition('=')
        tolerances[path] = float(value)

    runs = load(arguments.summaries)
    if len(runs) < 2:
        parser.error('repeatability needs at least two runs')
    control = None
    if arguments.against:
        control = load([arguments.against])[0][1]

    print('runs: %s' % ', '.join(name for name, _ in runs))
    ok, report = compare(runs, tolerances, control)
    print('\n'.join(report))
    print('\n%s' % ('OK' if ok else 'FAILED'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
