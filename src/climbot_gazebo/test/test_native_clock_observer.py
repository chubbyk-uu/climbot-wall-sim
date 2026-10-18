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

"""Validate throughput measurements independently of transport latency."""

import importlib.util
from pathlib import Path

SPEC = importlib.util.spec_from_file_location(
    'native_clock', Path(__file__).resolve().parents[3] / 'tools/observe_native_stats.py')
OBSERVER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(OBSERVER)


def test_transport_jitter_does_not_change_publisher_clock_ratio():
    """Callback scheduling must not masquerade as slower simulation."""
    points = [[21, 10, 20, 0, False, 10000, 1, .001],
              [22, 11, 21, 0, False, 11000, 1, .001]]
    assert OBSERVER.summarize(points)['native_clock_ratio'] == 1
    points[-1][0] = 29
    assert OBSERVER.summarize(points)['native_clock_ratio'] == 1


def test_reset_inside_window_is_explicitly_invalid():
    """Do not hide a clock reset by using only endpoints."""
    points = [[21, 10, 20, 0, False, 10000, 1, .001],
              [22, 0, 21, 0, False, 0, 1, .001],
              [23, 12, 22, 0, False, 12000, 1, .001]]
    result = OBSERVER.summarize(points)
    assert result['native_clock_ratio'] is None
    assert not result['checks']['source_clocks_monotonic']


def test_pause_after_warmup_is_not_silently_removed():
    """Retain pauses as failures instead of selecting a passing subwindow."""
    points = [[21, 10, 20, 0, False, 10000, 1, .001],
              [22, 10, 21, 1, True, 10000, 0, .001]]
    result = OBSERVER.summarize(points)
    assert result['native_clock_ratio'] is None
    assert not result['checks']['no_pause_after_warmup']
