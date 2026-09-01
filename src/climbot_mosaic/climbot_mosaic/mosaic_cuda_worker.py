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

"""Custom-CUDA fusion worker launched by the OpenCV-free controller."""

from __future__ import annotations

import json
from pathlib import Path
import sys

from climbot_common.acceleration import opencv_provenance
from climbot_common.hashing import sha256_file
from climbot_mosaic.fusion import build_wall_mosaic, FusionError, resolve_jobs
from climbot_mosaic.fusion_cuda import (
    cuda_device_info, cuda_render_pool_factory, CudaRuntimeError, CudaUnavailableError,
)
from climbot_mosaic.mosaic_inputs import MosaicInputError, validate_processed_runs
from climbot_mosaic.projection import ProjectionError
import cv2


def _path(value: object, name: str) -> Path:
    if not isinstance(value, str):
        raise ValueError(f'{name} must be a string path.')
    return Path(value)


def _execution(value: object) -> dict:
    if not isinstance(value, dict) or value.get('requested') not in ('cuda', 'auto') \
            or value.get('effective') != 'cuda':
        raise ValueError('CUDA worker received an invalid backend record.')
    from climbot_mosaic import _fusion_cuda
    result = dict(value)
    result['opencv'] = opencv_provenance(cv2, backend='cpu_support')
    result['cuda'] = {
        **cuda_device_info(),
        'implementation': 'climbot_custom_hardcut_kernel',
        'sampling': 'opencv-compatible-1/32-pixel',
        'extension_sha256': sha256_file(Path(_fusion_cuda.__file__)),
    }
    return result


def _failure(category: str, error: Exception) -> int:
    print(json.dumps({'status': 'failed', 'error': {
        'category': category, 'message': str(error)}}, allow_nan=False, sort_keys=True))
    return 2


def main() -> int:
    try:
        request = json.loads(sys.stdin.read())
        if not isinstance(request, dict):
            raise ValueError('worker request must be a JSON object.')
        input_runs = request.get('input_runs')
        if not isinstance(input_runs, list) or not input_runs:
            raise ValueError('input_runs must be a non-empty list.')
        jobs_value = request.get('jobs')
        if jobs_value is not None and (isinstance(jobs_value, bool) or
                                       not isinstance(jobs_value, int) or jobs_value <= 0):
            raise ValueError('jobs must be null or a positive integer.')
        memory_budget_gb = float(request['memory_budget_gb'])
        preview_max_side_px = request['preview_max_side_px']
        if isinstance(preview_max_side_px, bool) or not isinstance(preview_max_side_px, int):
            raise ValueError('preview_max_side_px must be an integer.')
        inputs = validate_processed_runs(
            tuple(_path(value, 'input_run') for value in input_runs))
    except (KeyError, TypeError, ValueError, MosaicInputError, ProjectionError,
            json.JSONDecodeError) as error:
        return _failure('input_contract', error)
    try:
        execution = _execution(request.get('execution'))
    except ValueError as error:
        return _failure('input_contract', error)
    except (ImportError, CudaUnavailableError) as error:
        return _failure('cuda_unavailable', error)
    except (FusionError, OSError, RuntimeError) as error:
        return _failure('backend_independent_failure', error)
    try:
        manifest = build_wall_mosaic(
            _path(request.get('output_dir'), 'output_dir'),
            _path(request.get('work_dir'), 'work_dir'),
            inputs,
            _path(request.get('pose_graph_dir'), 'pose_graph_dir'),
            float(request['resolution_m_per_pixel']),
            resolve_jobs(jobs_value, memory_budget_gb), memory_budget_gb,
            preview_max_side_px, execution, cuda_render_pool_factory())
    except CudaUnavailableError as error:
        return _failure('cuda_unavailable', error)
    except CudaRuntimeError as error:
        return _failure('cuda_runtime_failure', error)
    except (KeyError, TypeError, ValueError, MosaicInputError, ProjectionError) as error:
        return _failure('input_contract', error)
    except (FusionError, OSError, RuntimeError) as error:
        return _failure('backend_independent_failure', error)
    print(json.dumps({'status': 'completed', 'grid': manifest['grid'],
                      'outputs': manifest['outputs']}, allow_nan=False, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
