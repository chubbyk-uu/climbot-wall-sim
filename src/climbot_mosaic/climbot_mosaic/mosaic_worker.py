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

"""CPU fusion worker launched by the OpenCV-free backend controller."""

from __future__ import annotations

import json
from pathlib import Path
import sys

from climbot_common.acceleration import opencv_provenance
from climbot_mosaic.fusion import build_wall_mosaic, FusionError, resolve_jobs
from climbot_mosaic.mosaic_inputs import MosaicInputError, validate_processed_runs
from climbot_mosaic.projection import ProjectionError
import cv2


def _request() -> dict:
    value = json.loads(sys.stdin.read())
    if not isinstance(value, dict):
        raise ValueError('worker request must be a JSON object.')
    return value


def _path(value: object, name: str) -> Path:
    if not isinstance(value, str):
        raise ValueError(f'{name} must be a string path.')
    return Path(value)


def _execution(value: object) -> dict:
    if not isinstance(value, dict):
        raise ValueError('execution backend record must be an object.')
    requested = value.get('requested')
    effective = value.get('effective')
    if requested not in ('cpu', 'cuda', 'auto') or effective != 'cpu':
        raise ValueError('CPU worker received an invalid backend record.')
    result = dict(value)
    result['opencv'] = opencv_provenance(cv2, backend='cpu')
    return result


def main() -> int:
    try:
        request = _request()
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
        manifest = build_wall_mosaic(
            _path(request.get('output_dir'), 'output_dir'),
            _path(request.get('work_dir'), 'work_dir'),
            validate_processed_runs(tuple(_path(value, 'input_run') for value in input_runs)),
            _path(request.get('pose_graph_dir'), 'pose_graph_dir'),
            float(request['resolution_m_per_pixel']),
            resolve_jobs(jobs_value, memory_budget_gb), memory_budget_gb,
            preview_max_side_px, _execution(request.get('execution')))
    except (KeyError, TypeError, ValueError, MosaicInputError, ProjectionError, FusionError,
            OSError, json.JSONDecodeError) as error:
        print(json.dumps({'status': 'failed', 'error': {'message': str(error)}},
                         allow_nan=False, sort_keys=True))
        return 2
    print(json.dumps({'status': 'completed', 'grid': manifest['grid'],
                      'outputs': manifest['outputs']}, allow_nan=False, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
