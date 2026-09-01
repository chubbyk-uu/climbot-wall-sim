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

"""The public mosaic entry point stays OpenCV-free until it starts a child."""

import json
import os
from pathlib import Path
import re
import subprocess
import sys

from climbot_mosaic.mosaic_inputs import input_summary, validate_processed_runs
from test_mosaic_inputs import make_processed_run, write_json


def test_backend_controller_does_not_import_opencv_before_its_worker():
    source = (Path(__file__).parents[1] / 'scripts' / 'build_wall_mosaic').read_text(
        encoding='utf-8')
    assert not re.search(r'^import cv2\b|^from cv2\b', source, flags=re.MULTILINE)
    assert "'climbot_mosaic.mosaic_worker'" in source
    assert "'climbot_mosaic.mosaic_cuda_worker'" in source
    assert "'--backend'" in source
    assert "'--cuda-opencv-root'" not in source


def test_cpu_worker_is_the_only_module_that_imports_cpu_fusion():
    source = (Path(__file__).parents[1] / 'climbot_mosaic' / 'mosaic_worker.py').read_text(
        encoding='utf-8')
    assert 'from climbot_mosaic.fusion import' in source
    assert 'from climbot_common.acceleration import opencv_provenance' in source


def test_cuda_worker_labels_invalid_inputs_before_probing_the_gpu(tmp_path):
    """Auto mode must not hide an invalid archive behind a CPU retry."""
    completed = subprocess.run(
        [sys.executable, '-m', 'climbot_mosaic.mosaic_cuda_worker'],
        input=json.dumps({
            'input_runs': [str(tmp_path / 'missing')],
            'pose_graph_dir': str(tmp_path / 'graph'),
            'output_dir': str(tmp_path / 'output'),
            'work_dir': str(tmp_path / 'work'),
            'resolution_m_per_pixel': 0.005,
            'jobs': 1,
            'memory_budget_gb': 1.0,
            'preview_max_side_px': 512,
            'execution': {'requested': 'auto', 'effective': 'cuda'},
        }),
        check=False, capture_output=True, text=True, env=os.environ.copy(), timeout=10.0)
    assert completed.returncode == 2
    result = json.loads(completed.stdout)
    assert result['status'] == 'failed'
    assert result['error']['category'] == 'input_contract'


def test_default_cpu_backend_runs_in_a_clean_child_and_records_provenance(tmp_path):
    """The controller path, not only direct fusion(), publishes a valid CPU run."""
    run = make_processed_run(tmp_path, 'run', 'controller-cpu-run')
    for path in (run / 'metadata').glob('*.json'):
        label = json.loads(path.read_text(encoding='utf-8'))
        label['camera_pose']['pose']['orientation'] = {
            'x': 1.0, 'y': 0.0, 'z': 0.0, 'w': 0.0,
        }
        write_json(path, label)
    inputs = validate_processed_runs([run])
    initial, optimized = [], []
    for frame in inputs.frames:
        position = frame.label['camera_pose']['pose']['position']
        initial.append({
            'source_run_id': frame.key.source_run_id, 'frame_index': frame.key.frame_index,
            'x_m': position['x'], 'y_m': position['y'],
            'wall_heading_rad': frame.label['wall_heading_rad'],
        })
        optimized.append({
            'source_run_id': frame.key.source_run_id, 'frame_index': frame.key.frame_index,
            'correction': {'dx_m': 0.0, 'dy_m': 0.0, 'dyaw_rad': 0.0},
            'posterior_std': {'x_m': 0.001, 'y_m': 0.001, 'yaw_rad': 0.001},
        })
    graph = tmp_path / 'graph'
    graph.mkdir()
    for name, value in {
            'pose_graph.json': {
                'pose_graph_format_version': 1, 'input_summary': input_summary(inputs)},
            'initial_poses.json': {'poses': initial},
            'optimized_poses.json': {'poses': optimized},
    }.items():
        (graph / name).write_text(json.dumps(value) + '\n', encoding='utf-8')
    script = Path(__file__).parents[1] / 'scripts' / 'build_wall_mosaic'
    output = tmp_path / 'output'
    completed = subprocess.run(
        [sys.executable, str(script), '--input-run', str(run), '--pose-graph-dir', str(graph),
         '--output-dir', str(output), '--work-dir', str(tmp_path / 'work'),
         '--resolution-mm-per-pixel', '5', '--jobs', '1'],
        check=False, capture_output=True, text=True, env=os.environ.copy(), timeout=30.0)
    assert completed.returncode == 0, completed.stderr
    response = json.loads(completed.stdout)
    assert response['status'] == 'completed'
    assert response['output_dir'] == str(output)
    manifest = json.loads((output / 'mosaic_manifest.json').read_text(encoding='utf-8'))
    execution = manifest['execution']
    assert execution['requested'] == execution['effective'] == 'cpu'
    assert execution['fallback'] is False
    assert len(execution['attempts']) == 1
    assert execution['attempts'][0]['outcome'] == 'completed'
    assert len(execution['opencv']['opencv_module_sha256']) == 64
    assert str(tmp_path) not in json.dumps(execution)
