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
Run an optional CUDA backend without contaminating the CPU process.

The controller deliberately has no OpenCV import. A CUDA OpenCV build is a
private, optional toolchain; loading it in the ROS/system OpenCV process makes
both the CPU fallback and the provenance claim ambiguous.  Backends therefore
communicate with this module through one strict, small JSON document.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Any, Callable, Mapping, Sequence


BACKENDS = ('cpu', 'cuda', 'auto')
CUDA_OPENCV_ROOT_ENV = 'CLIMBOT_CUDA_OPENCV_ROOT'


class AccelerationError(RuntimeError):
    """A backend request cannot safely produce its requested result."""

    category = 'runtime_failure'


class BackendConfigurationError(AccelerationError):
    """The caller supplied an invalid backend setting or CUDA prefix."""

    category = 'configuration'


class BackendInputError(AccelerationError):
    """Inputs are invalid, so changing execution backend cannot help."""

    category = 'input_contract'


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


def _path_entries(value: str | None) -> list[str]:
    return [entry for entry in (value or '').split(os.pathsep) if entry]


def _prepend_path(value: str, existing: str | None) -> str:
    entries = [value] + [entry for entry in _path_entries(existing) if entry != value]
    return os.pathsep.join(entries)


def resolve_cuda_opencv_root(explicit_root: str | Path | None = None,
                             environment: Mapping[str, str] | None = None) -> Path:
    """Resolve and validate the isolated CUDA OpenCV prefix without importing it."""
    environment = os.environ if environment is None else environment
    value = explicit_root if explicit_root is not None else environment.get(CUDA_OPENCV_ROOT_ENV)
    if value is None or not str(value).strip():
        raise BackendUnavailableError(
            f'CUDA OpenCV prefix is not configured; pass --cuda-opencv-root or set '
            f'{CUDA_OPENCV_ROOT_ENV}.')
    root = Path(value).expanduser().resolve(strict=False)
    python_root = root / 'python'
    library_root = root / 'lib'
    extension = next(python_root.glob('cv2/python-*/cv2*.so'), None)
    if not python_root.is_dir() or extension is None or not library_root.is_dir():
        raise BackendUnavailableError(
            'CUDA OpenCV prefix is incomplete; expected python/cv2/python-*/cv2*.so '
            'and lib beneath the configured prefix.')
    return root


def cuda_child_environment(root: Path,
                           environment: Mapping[str, str] | None = None) -> dict[str, str]:
    """Return an environment that loads CUDA OpenCV only in its child process."""
    child = dict(os.environ if environment is None else environment)
    root = Path(root)
    child[CUDA_OPENCV_ROOT_ENV] = str(root)
    child['PYTHONPATH'] = _prepend_path(str(root / 'python'), child.get('PYTHONPATH'))
    child['LD_LIBRARY_PATH'] = _prepend_path(str(root / 'lib'), child.get('LD_LIBRARY_PATH'))
    return child


def cpu_child_environment(cuda_root: Path | None,
                          environment: Mapping[str, str] | None = None) -> dict[str, str]:
    """Remove this controller's CUDA prefix before starting a CPU child."""
    child = dict(os.environ if environment is None else environment)
    child.pop(CUDA_OPENCV_ROOT_ENV, None)
    if cuda_root is None:
        return child
    root = Path(cuda_root).resolve(strict=False)
    for key in ('PYTHONPATH', 'LD_LIBRARY_PATH'):
        child[key] = os.pathsep.join(
            entry for entry in _path_entries(child.get(key))
            if not Path(entry).resolve(strict=False).is_relative_to(root))
    return child


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
        raise BackendRuntimeError(
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
    except BackendInputError:
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


def probe_cuda_opencv(root: Path, *, environment: Mapping[str, str] | None = None,
                      timeout_s: float = 30.0) -> dict[str, Any]:
    """Run the real CUDA probe in an isolated interpreter."""
    try:
        result = run_json_child(
            (sys.executable, '-m', 'climbot_common.cuda_probe'), {},
            environment=cuda_child_environment(root, environment), timeout_s=timeout_s)
    except BackendRuntimeError as error:
        raise BackendUnavailableError(str(error)) from error
    if result.get('probe') != 'cuda_opencv':
        raise BackendUnavailableError('CUDA probe returned an unexpected result type.')
    return result


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
