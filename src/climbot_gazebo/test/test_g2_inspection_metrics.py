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

"""Keep the reported exposure-time P95 definition deterministic."""

import importlib.util
from pathlib import Path

import pytest


def _evaluator_module():
    path = Path(__file__).resolve().parents[1] / 'scripts' / 'evaluate_g2_inspection.py'
    spec = importlib.util.spec_from_file_location('g2_inspection_evaluator', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_p95_uses_nearest_rank_so_small_capture_runs_are_not_interpolated():
    percentile = _evaluator_module().nearest_rank_percentile
    assert percentile([0.001, 0.002, 0.003, 0.004], 0.95) == pytest.approx(0.004)
    assert percentile([0.001, 0.002, 0.003, 0.004, 0.005], 0.80) == pytest.approx(0.004)


def test_percentile_rejects_invalid_fraction_and_empty_sample_set_is_explicit():
    percentile = _evaluator_module().nearest_rank_percentile
    assert percentile([], 0.95) is None
    with pytest.raises(ValueError):
        percentile([0.001], 0.0)
