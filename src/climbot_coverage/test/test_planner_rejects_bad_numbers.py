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

"""A parameter that is not a number must stop the planner, not be accepted."""

# Every physical check in validatePhysicalParameters is a comparison, and NaN
# fails all of them rather than any of them: `detection_width <= 0.0` is false
# for NaN, so it passes as valid. Measured before this test existed, with
# detection_width:=.nan - the planner started, planned nothing, published an
# empty task and reported 0% coverage. It fails closed, which is the right
# direction, but it presents as a planning fault and sends whoever is looking
# at the region geometry rather than at the number they typed.

import subprocess

import pytest


BAD_NUMBERS = [
    ('detection_width', '.nan'),
    ('detection_length', '.nan'),
    ('overlap_ratio', '.nan'),
    ('wall_width', '.nan'),
    ('wall_height', '.nan'),
    ('edge_clearance', '.inf'),
    ('robot_width', '.nan'),
]


@pytest.mark.parametrize('name,value', BAD_NUMBERS)
def test_a_non_finite_parameter_is_refused_at_startup(name, value):
    """Refused by name, so the message names what has to be changed."""
    result = subprocess.run(
        ['ros2', 'run', 'climbot_coverage', 'coverage_planner_node',
         '--ros-args', '-p', '%s:=%s' % (name, value)],
        capture_output=True, text=True, timeout=60)
    assert result.returncode != 0, (
        '%s=%s was accepted; the planner will run and report an empty task '
        'instead of the bad number it was given' % (name, value))
    output = result.stdout + result.stderr
    assert name in output, output[-500:]
    assert 'finite' in output, output[-500:]
