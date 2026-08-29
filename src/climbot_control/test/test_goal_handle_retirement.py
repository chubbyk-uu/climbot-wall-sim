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

"""
A coverage goal handle must never be released while it is still active.

rclcpp_action's ~ServerGoalHandle cancels a goal that reaches its destructor
without a terminal state, and publishes the result from inside that destructor
with no try/catch of its own. Once shutdown has begun the server has usually
retired the goal, so that publish throws out of a destructor and terminates
line_tracker_node with SIGABRT -- observed as `Proc line_tracker_node-1 exited
with code -6` roughly one run in three.

Three separate shutdown paths had each been written to drop the handle rather
than risk a publish failure, which is precisely what armed the destructor.
Reproducing it needs a race, so the invariant is asserted against the source
instead: the handle is dropped in exactly one place, and that place terminates
the goal first.
"""

import re

import pytest

SOURCE = (
    re.sub(
        r'^\s*//.*$', '',
        open(
            __file__.rsplit('/test/', 1)[0] + '/src/line_tracker_node.cpp').read(),
        flags=re.MULTILINE))


def _body_of(name):
    """Return the text of a member function, brace matched."""
    start = SOURCE.index(name + '(')
    opening = SOURCE.index('{', start)
    depth = 0
    for index in range(opening, len(SOURCE)):
        if SOURCE[index] == '{':
            depth += 1
        elif SOURCE[index] == '}':
            depth -= 1
            if depth == 0:
                return SOURCE[opening:index + 1]
    raise AssertionError('unbalanced braces after ' + name)


def test_the_handle_is_released_in_exactly_one_place():
    # clearActiveGoal is what performs the release; more than one caller means
    # a path exists that has not been checked for a live handle.
    calls = len(re.findall(r'\bclearActiveGoal\(\)\s*;', SOURCE))
    assert calls == 1, (
        'clearActiveGoal() is called %d times; every release must go through '
        'retireActiveGoal so the goal is terminal first' % calls)


def test_that_place_terminates_the_goal_first():
    body = _body_of('void retireActiveGoal')
    release = body.index('clearActiveGoal()')
    for terminator in ('succeed(', 'canceled(', 'abort('):
        assert terminator in body, 'retireActiveGoal does not call ' + terminator
        assert body.index(terminator) < release, (
            terminator + ' must run before the handle is released')


def test_termination_failure_never_escapes():
    # The publish can fail whenever the server has already retired the goal.
    # Rethrowing there is what turned a bookkeeping race into a process abort.
    body = _body_of('void retireActiveGoal')
    assert 'catch (const std::exception' in body
    assert 'throw;' not in body


def test_no_shutdown_path_drops_a_live_handle():
    # Each of these once read `if (!rclcpp::ok()) { clearActiveGoal(); }`.
    for name in ('void finishGoal', 'void controlStep'):
        try:
            body = _body_of(name)
        except ValueError:
            continue
        assert 'clearActiveGoal()' not in body, (
            name + ' releases the handle directly instead of retiring it')


@pytest.mark.parametrize('symbol', ['active_goal_.reset()', 'active_goal_ ='])
def test_the_handle_is_only_reset_where_expected(symbol):
    assert SOURCE.count(symbol) == 1, (
        '%s appears more than once; a second assignment can drop a live handle'
        % symbol)


def test_the_source_read_is_the_coverage_action_server():
    # Guards the path above: if it ever pointed at a different node the
    # assertions would pass vacuously.
    assert 'rclcpp_action::ServerGoalHandle<ExecuteCoverage>' in SOURCE
    assert 'handleAccepted' in SOURCE
