"""Verify normal-load summaries preserve physics-step lift-off samples."""

import importlib.util
import os

import pytest

_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'scripts', 'measure_normal_loads.py')
_SPEC = importlib.util.spec_from_file_location('measure_normal_loads', _PATH)
measurement = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(measurement)


def test_empty_load_series_has_no_statistics():
    assert measurement.load_statistics([]) is None


def test_zero_load_steps_reduce_contact_ratio():
    result = measurement.load_statistics([40.0, 0.0, 20.0, 0.0])
    assert result['mean'] == pytest.approx(15.0)
    assert result['min'] == 0.0
    assert result['max'] == 40.0
    assert result['samples'] == 4
    assert result['zero_samples'] == 2
    assert result['contact_ratio'] == pytest.approx(0.5)


def test_positive_load_at_every_step_is_full_contact():
    result = measurement.load_statistics([35.9, 37.2, 40.1])
    assert result['zero_samples'] == 0
    assert result['contact_ratio'] == pytest.approx(1.0)
