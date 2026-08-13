"""Verify the in-place turn profile respects its rate and acceleration limits."""

import importlib.util
import math
import os

import pytest

_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'scripts', 'measure_turn_slip.py')
_SPEC = importlib.util.spec_from_file_location('measure_turn_slip', _PATH)
turn = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(turn)


def integrate(profile, steps=20000):
    """Return the angle swept by numerically integrating the rate."""
    total = 0.0
    step = profile['duration'] / steps
    for index in range(steps):
        _, rate = turn.sample_turn(profile, (index + 0.5) * step)
        total += rate * step
    return total


def test_small_angle_uses_a_triangular_profile():
    """Below the rate limit the profile never reaches its coast phase."""
    profile = turn.plan_turn(math.radians(10.0), 0.6, 1.5)
    assert profile['shape'] == 'triangle'
    assert profile['coast'] == 0.0
    assert profile['peak'] < 0.6


def test_large_angle_uses_a_trapezoidal_profile():
    """Above the rate limit the profile coasts at exactly that limit."""
    profile = turn.plan_turn(math.radians(180.0), 0.6, 1.5)
    assert profile['shape'] == 'trapezoid'
    assert profile['coast'] > 0.0
    assert profile['peak'] == pytest.approx(0.6)


@pytest.mark.parametrize('degrees', [5.0, 30.0, 90.0, 180.0, -45.0, -135.0])
def test_profile_sweeps_the_requested_angle(degrees):
    """The integral of the commanded rate is the commanded turn."""
    profile = turn.plan_turn(math.radians(degrees), 0.6, 1.5)
    assert integrate(profile) == pytest.approx(math.radians(degrees), abs=1e-6)


@pytest.mark.parametrize('degrees', [5.0, 90.0, 180.0])
def test_reference_angle_matches_the_integrated_rate(degrees):
    """The reference the controller tracks agrees with its feedforward."""
    profile = turn.plan_turn(math.radians(degrees), 0.6, 1.5)
    angle, rate = turn.sample_turn(profile, profile['duration'])
    assert angle == pytest.approx(math.radians(degrees), abs=1e-9)
    assert rate == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize('degrees', [5.0, 30.0, 90.0, 180.0])
def test_rate_never_exceeds_the_limit(degrees):
    """Guide section 10.6 requires the angular rate to stay clamped."""
    profile = turn.plan_turn(math.radians(degrees), 0.6, 1.5)
    peak = max(abs(turn.sample_turn(profile, profile['duration'] * index / 500.0)[1])
               for index in range(501))
    assert peak <= 0.6 + 1e-9


def test_negative_turns_mirror_positive_ones():
    """A turn and its opposite differ only in sign."""
    forward = turn.plan_turn(math.radians(90.0), 0.6, 1.5)
    backward = turn.plan_turn(math.radians(-90.0), 0.6, 1.5)
    assert forward['duration'] == pytest.approx(backward['duration'])
    for fraction in (0.1, 0.5, 0.9):
        moment = forward['duration'] * fraction
        assert turn.sample_turn(forward, moment)[0] == pytest.approx(
            -turn.sample_turn(backward, moment)[0])
