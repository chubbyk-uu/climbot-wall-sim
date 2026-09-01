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

"""The backend controller stays testable without a CUDA device."""

import sys

from climbot_common.acceleration import (
    BackendConfigurationError,
    BackendIndependentError,
    BackendInputError,
    BackendProtocolError,
    BackendRuntimeError,
    BackendUnavailableError,
    cpu_child_environment,
    execute_backend,
    parse_backend,
    run_json_child,
)
import pytest


def _worker(tmp_path, body: str) -> list[str]:
    path = tmp_path / 'worker.py'
    path.write_text(body, encoding='utf-8')
    return [sys.executable, str(path)]


def test_backend_spelling_is_closed():
    assert parse_backend('cpu') == 'cpu'
    with pytest.raises(BackendConfigurationError, match='one of'):
        parse_backend('gpu')


def test_child_environment_is_an_independent_copy():
    environment = {'PYTHONPATH': 'workspace', 'MARKER': 'parent'}
    child = cpu_child_environment(environment)
    child['MARKER'] = 'child'
    assert environment['MARKER'] == 'parent'


def test_json_child_only_accepts_one_finite_completed_object(tmp_path):
    completed = _worker(tmp_path, """import json, sys
assert json.loads(sys.stdin.read()) == {'request': 1}
print(json.dumps({'status': 'completed', 'answer': 2}))
""")
    assert run_json_child(completed, {'request': 1})['answer'] == 2
    noisy = _worker(tmp_path, """print('log line')
print('{\"status\": \"completed\"}')
""")
    with pytest.raises(BackendProtocolError, match='exactly one'):
        run_json_child(noisy, {})
    nonfinite = _worker(tmp_path, """print('{\"status\": \"completed\", \"value\": NaN}')
""")
    with pytest.raises(BackendProtocolError, match='non-finite'):
        run_json_child(nonfinite, {})
    unsuccessful = _worker(tmp_path, """print(
    '{\"status\": \"failed\", \"error\": {\"message\": \"nope\"}}')
raise SystemExit(2)
""")
    with pytest.raises(BackendRuntimeError, match='exited 2: nope'):
        run_json_child(unsuccessful, {})
    invalid = _worker(tmp_path, """print(
    '{\"status\": \"failed\", \"error\": '
    '{\"category\": \"input_contract\", \"message\": \"bad archive\"}}')
raise SystemExit(2)
""")
    with pytest.raises(BackendInputError, match='bad archive'):
        run_json_child(invalid, {})
    shared = _worker(tmp_path, """print(
    '{\"status\": \"failed\", \"error\": '
    '{\"category\": \"backend_independent_failure\", \"message\": \"disk full\"}}')
raise SystemExit(2)
""")
    with pytest.raises(BackendIndependentError, match='disk full'):
        run_json_child(shared, {})


def test_auto_falls_back_only_for_cuda_availability_or_runtime_failure():
    calls = []

    def cpu():
        calls.append('cpu')
        return 'cpu-result'

    def unavailable():
        calls.append('cuda')
        raise BackendUnavailableError('probe failed')

    result = execute_backend('auto', cpu, unavailable)
    assert result.value == 'cpu-result'
    assert result.provenance['effective'] == 'cpu'
    assert result.provenance['fallback'] is True
    assert result.provenance['fallback_reason']['category'] == 'cuda_unavailable'
    assert calls == ['cuda', 'cpu']
    with pytest.raises(BackendUnavailableError):
        execute_backend('cuda', cpu, unavailable)

    def invalid_input():
        raise BackendInputError('bad archive')

    with pytest.raises(BackendInputError):
        execute_backend('auto', cpu, invalid_input)
    assert calls == ['cuda', 'cpu', 'cuda']

    def shared_failure():
        raise BackendIndependentError('output disk failed')

    with pytest.raises(BackendIndependentError):
        execute_backend('auto', cpu, shared_failure)
    assert calls == ['cuda', 'cpu', 'cuda']
