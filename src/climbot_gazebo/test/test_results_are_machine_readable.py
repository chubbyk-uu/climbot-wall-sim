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

"""Every archived acceptance summary must be readable by something but Python."""

# json.dump writes the bare tokens NaN and Infinity by default. Python reads
# them straight back, so nothing here noticed while twenty-six archived
# summaries carried NaN and could not be parsed by Ruby, by strict Java or Go,
# by any schema validator, or by most data warehouses. Evidence that only its
# own producer can read is not machine-readable evidence.
#
# The writer now passes allow_nan=False, so this cannot come back through the
# evaluator. It could still come back through a file written by hand or by a
# one-off script, which is why the check is over the archive rather than over
# the writer.

import json
from pathlib import Path

import pytest


def _repository_root():
    # src/climbot_gazebo/test/this_file.py
    root = Path(__file__).resolve().parents[3]
    return root if (root / 'results').is_dir() else None


def _summaries():
    root = _repository_root()
    return sorted((root / 'results').rglob('*_summary.json')) if root else []


def _reject_constants(name):
    raise ValueError('%s is not valid JSON' % name)


@pytest.mark.skipif(_repository_root() is None,
                    reason='the results archive is not beside this checkout')
def test_every_summary_parses_under_a_strict_json_reader():
    """parse_constant fires on exactly the tokens RFC 8259 does not allow."""
    unreadable = []
    for path in _summaries():
        try:
            json.loads(path.read_text(encoding='utf-8'),
                       parse_constant=_reject_constants)
        except ValueError as error:
            unreadable.append('%s: %s' % (path.name, error))
    assert not unreadable, (
        'these acceptance summaries are not valid JSON:\n  ' +
        '\n  '.join(unreadable))


@pytest.mark.skipif(_repository_root() is None,
                    reason='the results archive is not beside this checkout')
def test_a_spacing_metric_that_does_not_apply_says_so():
    """A bare null leaves "does not apply" and "not measured" indistinguishable."""
    # Only nulls have to explain themselves. A summary carrying a real number
    # is unambiguous whether or not it also carries the flag, and demanding one
    # would mean rewriting a hundred and forty-five archived files that say
    # nothing wrong.
    for path in _summaries():
        spacing = json.loads(path.read_text(encoding='utf-8')).get('scan_line_spacing')
        if not isinstance(spacing, dict):
            continue
        if spacing.get('maximum_scan_line_spacing_error_m') is not None:
            continue
        assert spacing.get('applicable') is False, (
            '%s reports no spacing error without saying the metric does not '
            'apply, so a reader cannot tell that from a measurement that was '
            'never taken' % path.name)
        assert spacing.get('not_applicable_reason'), (
            '%s says the spacing metric does not apply but not why' % path.name)
