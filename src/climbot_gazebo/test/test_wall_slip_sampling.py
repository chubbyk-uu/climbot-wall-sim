"""Verify wall-slip trajectories record each truth message exactly once."""

import importlib.util
import os

_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'scripts', 'calibrate_wall_slip.py')
_SPEC = importlib.util.spec_from_file_location('calibrate_wall_slip', _PATH)
calibration = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(calibration)


def test_first_truth_sample_is_recorded():
    assert calibration.is_new_truth_sample(1_000_000, None)


def test_repeated_truth_timestamp_is_not_recorded_again():
    assert not calibration.is_new_truth_sample(1_000_000, 1_000_000)


def test_next_truth_timestamp_is_recorded():
    assert calibration.is_new_truth_sample(1_010_000, 1_000_000)


def test_reset_truth_timestamp_is_recorded():
    """A simulation reset must not suppress the first new-world sample."""
    assert calibration.is_new_truth_sample(0, 1_000_000)
