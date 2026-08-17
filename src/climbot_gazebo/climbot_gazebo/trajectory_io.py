"""Archiving for recorded truth trajectories: gzip-aware and noise-free."""

import csv
import gzip
import math
import os


def compact(value):
    """Round a float to micrometre resolution, leaving other types alone."""
    # Full float repr writes 17 significant digits, and everything past the
    # sixth decimal is round-trip noise rather than measurement: positions are
    # metres, angles radians, times seconds, and every acceptance threshold in
    # PROJECT_GUIDE 14 is stated in millimetres or degrees. Keeping the noise
    # doubled the archive for nothing.
    if not isinstance(value, float) or not math.isfinite(value):
        return value
    return round(value, 6)


def open_trajectory(path, mode='rt'):
    """Open a trajectory CSV, transparently gzipped when the name says so."""
    if str(path).endswith('.gz'):
        return gzip.open(path, mode, newline='', encoding='utf-8')
    return open(path, mode, newline='', encoding='utf-8')


def write_trajectory(path, fieldnames, rows):
    """Write recorded rows to a trajectory CSV and return the path used."""
    expanded = os.path.abspath(os.path.expanduser(str(path)))
    directory = os.path.dirname(expanded)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open_trajectory(expanded, 'wt') as handle:
        writer = csv.DictWriter(
            handle, fieldnames=fieldnames, lineterminator='\n')
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {name: compact(row[name]) for name in fieldnames})
    return expanded


def read_trajectory(path):
    """Read a trajectory CSV, gzipped or not, as a list of dict rows."""
    with open_trajectory(path) as handle:
        return list(csv.DictReader(handle))
