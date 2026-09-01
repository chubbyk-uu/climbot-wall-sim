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

"""The controller stays testable without a CUDA device or CUDA OpenCV."""

from pathlib import Path
import sys

from climbot_common.acceleration import (
    BackendConfigurationError,
    BackendInputError,
    BackendProtocolError,
    BackendRuntimeError,
    BackendUnavailableError,
    cpu_child_environment,
    cuda_child_environment,
    execute_backend,
    parse_backend,
    resolve_cuda_opencv_root,
    run_json_child,
)
import pytest


def _worker(tmp_path, body: str) -> list[str]:
    path = tmp_path / 'worker.py'
    path.write_text(body, encoding='utf-8')
    return [sys.executable, str(path)]


def _cuda_prefix(tmp_path) -> Path:
    prefix = tmp_path / 'cuda-opencv'
    extension = prefix / 'python' / 'cv2' / 'python-3.12' / 'cv2.test.so'
    extension.parent.mkdir(parents=True)
    extension.write_bytes(b'not loaded')
    (prefix / 'lib').mkdir()
    return prefix


def test_backend_spelling_is_closed():
    assert parse_backend('cpu') == 'cpu'
    with pytest.raises(BackendConfigurationError, match='one of'):
        parse_backend('gpu')


def test_cuda_environment_is_child_local_and_cpu_strips_it(tmp_path):
    prefix = _cuda_prefix(tmp_path)
    environment = {'PYTHONPATH': 'workspace', 'LD_LIBRARY_PATH': 'system'}
    cuda = cuda_child_environment(prefix, environment)
    assert cuda['PYTHONPATH'].split(':')[0] == str(prefix / 'python')
    assert cuda['LD_LIBRARY_PATH'].split(':')[0] == str(prefix / 'lib')
    assert 'CLIMBOT_CUDA_OPENCV_ROOT' not in environment
    cpu = cpu_child_environment(prefix, cuda)
    assert cpu['PYTHONPATH'] == 'workspace'
    assert cpu['LD_LIBRARY_PATH'] == 'system'
    assert 'CLIMBOT_CUDA_OPENCV_ROOT' not in cpu


def test_cuda_prefix_is_required_and_structurally_validated(tmp_path):
    with pytest.raises(BackendUnavailableError, match='not configured'):
        resolve_cuda_opencv_root(environment={})
    with pytest.raises(BackendUnavailableError, match='incomplete'):
        resolve_cuda_opencv_root(tmp_path / 'missing')
    prefix = _cuda_prefix(tmp_path)
    assert resolve_cuda_opencv_root(prefix) == prefix.resolve()


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
