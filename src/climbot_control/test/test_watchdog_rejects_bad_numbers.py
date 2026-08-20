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

"""The actuator-facing watchdog must reject non-finite safety parameters."""

import subprocess

import pytest


BAD_NUMBERS = [
    ('command_timeout_s', '.nan'),
    ('command_timeout_s', '.inf'),
    ('publish_rate_hz', '.nan'),
    ('publish_rate_hz', '.inf'),
]


@pytest.mark.parametrize('name,value', BAD_NUMBERS)
def test_a_non_finite_safety_parameter_is_refused_at_startup(name, value):
    """A bad last-hop parameter must stop startup and identify itself."""
    result = subprocess.run(
        ['ros2', 'run', 'climbot_control', 'cmd_vel_watchdog_node',
         '--ros-args', '-p', '%s:=%s' % (name, value)],
        capture_output=True, text=True, timeout=30)
    assert result.returncode != 0, (
        '%s=%s was accepted; the wheel command watchdog is not fail-safe'
        % (name, value))
    output = result.stdout + result.stderr
    assert name in output or 'Watchdog timeout' in output, output[-500:]
    assert 'finite' in output, output[-500:]
