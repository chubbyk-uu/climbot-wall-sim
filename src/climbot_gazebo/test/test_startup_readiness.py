# Copyright 2026 jerry
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Do not dispatch GUI missions merely because discovery completed."""

from copy import deepcopy
import importlib.util
from pathlib import Path

import pytest

SPEC = importlib.util.spec_from_file_location(
    'startup', Path(__file__).resolve().parents[3] / 'tools/wait_gazebo_ready.py')
STARTUP = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(STARTUP)
BOUNDS = [.56, .56, 9.44, 7.44]


def streams():
    """Represent a healthy, discovered clock and sensor set."""
    return {name: {'stamp': 1., 'receipt': 1., 'position': [2., 2., 0.]}
            for name in ['clock', 'truth', 'filtered', 'imu_wall', 'joints']}


def test_clock_zero_is_not_runtime_readiness():
    """Advertised topics cannot admit a task while simulation time is zero."""
    values = streams()
    for value in values.values():
        value['stamp'] = 0.
    result = STARTUP.readiness_checks(values, 0., 1., BOUNDS)
    assert not result['positive_clock']


def test_initial_filter_origin_prevents_mission_admission():
    """A fresh first orientation update is not an initialized global position."""
    values = streams()
    values['filtered']['position'] = [0., 0., 0.]
    assert not STARTUP.readiness_checks(values, 1., 1., BOUNDS)[
        'filtered_inside_safe_region']


@pytest.mark.parametrize('field,value', [('receipt', .7), ('stamp', .7), ('stamp', 1.1)])
def test_stale_or_future_sources_are_not_ready(field, value):
    """Retain existing 250 ms freshness and 50 ms future limits."""
    values = deepcopy(streams())
    values['filtered'][field] = value
    assert not STARTUP.readiness_checks(values, 1., 1., BOUNDS)['filtered_fresh']


def test_clock_must_advance_throughout_valid_window():
    """A frozen clock cannot satisfy a duration by repeated callbacks."""
    progress = STARTUP.StableProgress()
    for _ in range(20):
        assert not progress.update(1., True)
    assert progress.update(3., True)


def test_bad_source_and_clock_reset_restart_observation_window():
    """Reject resets and interrupted validity instead of joining two windows."""
    progress = STARTUP.StableProgress()
    assert not progress.update(1., True)
    assert not progress.update(2., False)
    assert not progress.update(2.1, True)
    assert not progress.update(.1, True)
    assert not progress.update(.2, True)
    assert not progress.update(2.1, True)
    assert progress.update(2.2, True)


def test_finite_healthy_sources_inside_safe_region_are_ready():
    """The barrier must release for normal input without changing any pose."""
    assert all(STARTUP.readiness_checks(streams(), 1., 1., BOUNDS).values())
