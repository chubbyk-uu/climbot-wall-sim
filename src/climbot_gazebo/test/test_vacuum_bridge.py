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

"""Real launch and bidirectional bridge acceptance, with isolated DDS domain."""

from pathlib import Path
import subprocess
import sys
import uuid

import pytest


@pytest.mark.parametrize('mode', ['fixed_pressure', 'fan_ode'])
def test_pressure_bridge_and_all_live_fault_inputs(tmp_path, mode):
    """No mocked transport: require actual ROS commands and pressure messages."""
    tool = Path(__file__).resolve().parents[3] / 'tools/validate_vacuum_bridge.py'
    log = Path('/tmp') / ('climbot_s4_bridge_test_' + uuid.uuid4().hex + '.log')
    with log.open('w') as stream:
        result = subprocess.run([sys.executable, str(tool), '--mode', mode, '--output',
                                 str(tmp_path / 'evidence')], stdout=stream,
                                stderr=subprocess.STDOUT, timeout=90)
    assert result.returncode == 0, (
        f'Bridge acceptance failed; inspect {log}: {log.read_text()[-3000:]}')
