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

"""The render provenance parameters must survive launch expansion as strings."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch_ros.actions import Node

LAUNCH = Path(__file__).parents[1] / 'launch' / 'inspection.launch.py'


def _render_headless_parameter():
    """Expand inspection.launch.py and return the recorder's render_headless value."""
    spec = importlib.util.spec_from_file_location('inspection_launch', str(LAUNCH))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, Node) and entity._Node__node_name == 'archive_recorder_node':
            for item in entity._Node__parameters:
                if not isinstance(item, dict):
                    continue
                for key, value in item.items():
                    # launch stores parameter names as substitution tuples.
                    name = ''.join(getattr(part, 'text', '') for part in key)
                    if name == 'render_headless':
                        return value
    raise AssertionError('the recorder node has no render_headless parameter')


def test_headless_true_reaches_the_recorder_as_a_string():
    """
    launch_ros infers a parameter type from the resolved substitution text.

    Unwrapped, ``headless:=true`` arrived as BOOL and the recorder died on
    startup, rejecting it because it expected a string. A literal string in a
    test parameter dict is never coerced, so only launch expansion catches it.
    """
    context = LaunchContext()
    context.launch_configurations['render_headless'] = 'true'
    evaluated = _render_headless_parameter().evaluate(context)
    assert isinstance(evaluated, str), type(evaluated)
    assert evaluated == 'true'


def test_unknown_is_still_a_string_when_nothing_declares_the_condition():
    """A recorder started outside the simulation launch must not fail on type."""
    context = LaunchContext()
    context.launch_configurations['render_headless'] = 'unknown'
    assert _render_headless_parameter().evaluate(context) == 'unknown'
