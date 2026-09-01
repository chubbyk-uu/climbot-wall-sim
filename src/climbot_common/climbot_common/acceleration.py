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

"""Run an optional acceleration backend through one strict child protocol."""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import subprocess
from typing import Any, Callable, Mapping, Sequence


BACKENDS = ('cpu', 'cuda', 'auto')


class AccelerationError(RuntimeError):
    """A backend request cannot safely produce its requested result."""

    category = 'runtime_failure'


class BackendConfigurationError(AccelerationError):
    """The caller supplied an invalid backend setting."""

    category = 'configuration'


class BackendInputError(AccelerationError):
    """Inputs are invalid, so changing execution backend cannot help."""

    category = 'input_contract'


class BackendIndependentError(AccelerationError):
    """A shared operation failed, so retrying another backend cannot help."""

    category = 'backend_independent_failure'


class BackendUnavailableError(AccelerationError):
    """CUDA cannot be used before work starts on this host."""

    category = 'cuda_unavailable'


class BackendRuntimeError(AccelerationError):
    """A selected backend failed after its preflight had succeeded."""

    category = 'cuda_runtime_failure'


class BackendProtocolError(BackendRuntimeError):
    """A child did not honour the one-document JSON protocol."""

    category = 'child_protocol'


@dataclass(frozen=True)
class BackendExecution:
    """One completed selection decision, safe to put in a manifest."""

    value: Any
    provenance: dict[str, Any]


def parse_backend(value: str) -> str:
    """Validate one explicit backend spelling."""
    if value not in BACKENDS:
        raise BackendConfigurationError(
            f"backend must be one of {', '.join(BACKENDS)}, not {value!r}.")
    return value


def cpu_child_environment(environment: Mapping[str, str] | None = None) -> dict[str, str]:
    """Copy the environment for a clean worker subprocess."""
    return dict(os.environ if environment is None else environment)


def _reject_non_finite(token: str) -> None:
    raise ValueError(f'JSON protocol forbids non-finite number {token!r}.')


def _strict_json(text: str, description: str) -> dict[str, Any]:
    """Read exactly one finite JSON object, never a log line plus JSON."""
    lines = text.splitlines()
    if len(lines) != 1:
        raise BackendProtocolError(
            f'{description} must write exactly one JSON line to stdout, got {len(lines)} lines.')
    try:
        value = json.loads(lines[0], parse_constant=_reject_non_finite)
    except (TypeError, ValueError, json.JSONDecodeError) as error:
        raise BackendProtocolError(f'{description} wrote invalid JSON: {error}') from error
    if not isinstance(value, dict):
        raise BackendProtocolError(f'{description} JSON result must be an object.')
    return value


def run_json_child(command: Sequence[str], request: Mapping[str, Any], *,
                   environment: Mapping[str, str] | None = None,
                   timeout_s: float = 60.0) -> dict[str, Any]:
    """Run one clean worker and validate its JSON result and exit status."""
    try:
        request_text = json.dumps(request, allow_nan=False, sort_keys=True) + '\n'
    except (TypeError, ValueError) as error:
        raise BackendProtocolError(f'controller request is not finite JSON: {error}') from error
    try:
        completed = subprocess.run(
            list(command), input=request_text, text=True, capture_output=True,
            env=None if environment is None else dict(environment), timeout=timeout_s,
            check=False)
    except subprocess.TimeoutExpired as error:
        raise BackendRuntimeError(f'backend child exceeded {timeout_s:.1f} s.') from error
    result = _strict_json(completed.stdout, 'backend child')
    if completed.returncode != 0:
        error = result.get('error')
        message = error.get('message') if isinstance(error, dict) else None
        category = error.get('category') if isinstance(error, dict) else None
        error_types = {
            BackendInputError.category: BackendInputError,
            BackendIndependentError.category: BackendIndependentError,
            BackendUnavailableError.category: BackendUnavailableError,
            BackendRuntimeError.category: BackendRuntimeError,
        }
        error_type = error_types.get(category, BackendRuntimeError)
        raise error_type(
            f'backend child exited {completed.returncode}' + (f': {message}' if message else '.'))
    if result.get('status') != 'completed':
        raise BackendProtocolError("successful backend child did not report status 'completed'.")
    return result


def _failure_record(backend: str, error: AccelerationError) -> dict[str, Any]:
    return {'backend': backend, 'outcome': 'failed',
            'error': {'category': error.category, 'message': str(error)}}


def execute_backend(requested: str, cpu_attempt: Callable[[], Any],
                    cuda_attempt: Callable[[], Any]) -> BackendExecution:
    """Apply the fallback contract without ever masking an input failure."""
    requested = parse_backend(requested)
    attempts: list[dict[str, Any]] = []
    if requested == 'cpu':
        value = cpu_attempt()
        attempts.append({'backend': 'cpu', 'outcome': 'completed'})
        return BackendExecution(value, {
            'requested': requested, 'effective': 'cpu', 'fallback': False,
            'attempts': attempts,
        })
    try:
        value = cuda_attempt()
    except (BackendInputError, BackendIndependentError):
        raise
    except (BackendUnavailableError, BackendRuntimeError) as error:
        attempts.append(_failure_record('cuda', error))
        if requested == 'cuda':
            raise
        value = cpu_attempt()
        attempts.append({'backend': 'cpu', 'outcome': 'completed'})
        return BackendExecution(value, {
            'requested': requested, 'effective': 'cpu', 'fallback': True,
            'fallback_reason': {'category': error.category, 'message': str(error)},
            'attempts': attempts,
        })
    attempts.append({'backend': 'cuda', 'outcome': 'completed'})
    return BackendExecution(value, {
        'requested': requested, 'effective': 'cuda', 'fallback': False,
        'attempts': attempts,
    })


def opencv_provenance(cv2_module: Any, *, backend: str,
                      cuda: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Describe a loaded OpenCV build without storing its private install path."""
    from climbot_common.hashing import sha256_bytes, sha256_file

    module_path = Path(cv2_module.__file__)
    if module_path.suffix != '.so':
        candidates = tuple(module_path.parent.glob('python-*/cv2*.so'))
        if len(candidates) == 1:
            module_path = candidates[0]
    try:
        module_sha256 = sha256_file(module_path)
    except OSError as error:
        raise BackendRuntimeError('loaded OpenCV module cannot be hashed.') from error
    build_information = cv2_module.getBuildInformation()
    result: dict[str, Any] = {
        'backend': backend,
        'opencv_version': str(cv2_module.__version__),
        'opencv_module_sha256': module_sha256,
        'opencv_build_information_sha256': sha256_bytes(build_information.encode('utf-8')),
    }
    if cuda is not None:
        result['cuda'] = dict(cuda)
    return result
