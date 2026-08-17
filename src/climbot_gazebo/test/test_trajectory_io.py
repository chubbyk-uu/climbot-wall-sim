"""Verify trajectory archiving keeps the measurement and drops the noise."""

import gzip
import math

from climbot_gazebo.trajectory_io import compact
from climbot_gazebo.trajectory_io import read_trajectory
from climbot_gazebo.trajectory_io import write_trajectory
import pytest


FIELDS = ['time_s', 'truth_x_m', 'segment', 'state']


def _rows():
    return [
        {'time_s': 40.57, 'truth_x_m': -1.7060779467392706e-05,
         'segment': 0, 'state': 1},
        {'time_s': 40.58, 'truth_x_m': 2.0007945229123130,
         'segment': 1, 'state': 3},
    ]


def test_rounding_keeps_micrometre_resolution():
    """Every acceptance threshold is millimetres, so a micrometre is plenty."""
    assert compact(2.0007945229123130) == pytest.approx(2.000795, abs=5e-7)
    assert compact(-1.7060779467392706e-05) == pytest.approx(-1.7e-05, abs=5e-7)


def test_rounding_leaves_non_floats_and_non_finite_alone():
    assert compact(7) == 7
    assert compact('approach') == 'approach'
    assert math.isnan(compact(math.nan))
    assert math.isinf(compact(math.inf))


def test_a_plain_csv_round_trips(tmp_path):
    path = tmp_path / 'trajectory.csv'
    write_trajectory(path, FIELDS, _rows())
    assert not path.read_bytes().startswith(b'\x1f\x8b')
    rows = read_trajectory(path)
    assert [row['segment'] for row in rows] == ['0', '1']
    assert float(rows[1]['truth_x_m']) == pytest.approx(2.000795, abs=5e-7)


def test_a_gz_suffix_writes_and_reads_gzip(tmp_path):
    """The suffix is the switch, so an archive needs no separate flag."""
    path = tmp_path / 'trajectory.csv.gz'
    write_trajectory(path, FIELDS, _rows())
    assert path.read_bytes().startswith(b'\x1f\x8b')
    with gzip.open(path, 'rt') as handle:
        assert handle.readline().strip() == ','.join(FIELDS)
    assert [row['state'] for row in read_trajectory(path)] == ['1', '3']


def test_rounding_shrinks_the_archive(tmp_path):
    """Full float repr is the reason an archived run was megabytes wide."""
    noisy = tmp_path / 'noisy.csv'
    rows = [{'time_s': index * 0.01,
             'truth_x_m': 2.0 + index * 1.7060779467392706e-09,
             'segment': 0, 'state': 3} for index in range(500)]
    with open(noisy, 'w', newline='', encoding='utf-8') as handle:
        handle.write(','.join(FIELDS) + '\n')
        for row in rows:
            handle.write(','.join(repr(row[name]) for name in FIELDS) + '\n')
    compacted = tmp_path / 'compact.csv'
    write_trajectory(compacted, FIELDS, rows)
    assert compacted.stat().st_size < noisy.stat().st_size
