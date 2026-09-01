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

"""Optional real-GPU integration checks for the custom fusion backend."""

import json
import os
from pathlib import Path
import subprocess
import sys

from climbot_mosaic.fusion_cuda import cuda_device_info
from climbot_mosaic.mosaic_inputs import input_summary, validate_processed_runs
import numpy as np
import pytest
from test_mosaic_inputs import make_processed_run, write_json
import tifffile


def _require_cuda():
    pytest.importorskip('climbot_mosaic._fusion_cuda')
    try:
        cuda_device_info()
    except Exception as error:  # A built extension can run on a GPU-less CI host.
        pytest.skip(f'custom CUDA fusion is unavailable: {error}')


def _graph(tmp_path, run):
    inputs = validate_processed_runs([run])
    initial = []
    optimized = []
    for frame in inputs.frames:
        position = frame.label['camera_pose']['pose']['position']
        initial.append({
            'source_run_id': frame.key.source_run_id,
            'frame_index': frame.key.frame_index,
            'x_m': position['x'], 'y_m': position['y'],
            'wall_heading_rad': frame.label['wall_heading_rad'],
        })
        optimized.append({
            'source_run_id': frame.key.source_run_id,
            'frame_index': frame.key.frame_index,
            'correction': {'dx_m': 0.0, 'dy_m': 0.0, 'dyaw_rad': 0.0},
            'posterior_std': {'x_m': 0.001, 'y_m': 0.001, 'yaw_rad': 0.001},
        })
    graph = tmp_path / 'graph'
    graph.mkdir()
    write_json(graph / 'pose_graph.json', {
        'pose_graph_format_version': 1, 'input_summary': input_summary(inputs)})
    write_json(graph / 'initial_poses.json', {'poses': initial})
    write_json(graph / 'optimized_poses.json', {'poses': optimized})
    return graph


def _run(script, run, graph, output, work, backend):
    return subprocess.run(
        [sys.executable, str(script), '--input-run', str(run),
         '--pose-graph-dir', str(graph), '--output-dir', str(output),
         '--work-dir', str(work), '--resolution-mm-per-pixel', '5',
         '--jobs', '1', '--backend', backend],
        check=False, capture_output=True, text=True, env=os.environ.copy(), timeout=60.0)


def test_cuda_controller_preserves_full_small_mosaic_contract(tmp_path):
    _require_cuda()
    run = make_processed_run(tmp_path, 'run', 'cuda-controller-run')
    for path in (run / 'metadata').glob('*.json'):
        label = json.loads(path.read_text(encoding='utf-8'))
        label['camera_pose']['pose']['orientation'] = {
            'x': 1.0, 'y': 0.0, 'z': 0.0, 'w': 0.0,
        }
        write_json(path, label)
    graph = _graph(tmp_path, run)
    script = Path(__file__).parents[1] / 'scripts' / 'build_wall_mosaic'
    cpu = tmp_path / 'cpu'
    cuda = tmp_path / 'cuda'
    cpu_result = _run(script, run, graph, cpu, tmp_path / 'cpu-work', 'cpu')
    cuda_result = _run(script, run, graph, cuda, tmp_path / 'cuda-work', 'cuda')
    assert cpu_result.returncode == 0, cpu_result.stderr
    assert cuda_result.returncode == 0, cuda_result.stderr
    manifest = json.loads((cuda / 'mosaic_manifest.json').read_text(encoding='utf-8'))
    assert manifest['execution']['effective'] == 'cuda'
    assert manifest['execution']['cuda']['implementation'] == 'climbot_custom_hardcut_kernel'
    assert manifest['fusion']['cuda_renderer']['sampling'] == \
        'opencv-compatible-1/32-pixel'
    for name in ('coverage_count.tif', 'coverage_pose_only_count.tif'):
        np.testing.assert_array_equal(tifffile.imread(cpu / name), tifffile.imread(cuda / name))
    cpu_uncertainty = tifffile.imread(cpu / 'uncertainty.tif').astype(np.int32)
    cuda_uncertainty = tifffile.imread(cuda / 'uncertainty.tif').astype(np.int32)
    uncertainty_difference = np.abs(cpu_uncertainty - cuda_uncertainty)
    assert int(uncertainty_difference.max()) <= 1
    assert np.count_nonzero(uncertainty_difference) / uncertainty_difference.size <= 1.0e-5
    for name in ('mosaic_pose_only.tif', 'mosaic_optimized.tif'):
        difference = np.abs(
            tifffile.imread(cpu / name).astype(np.int16) -
            tifffile.imread(cuda / name).astype(np.int16))
        assert int(difference.max()) <= 1
    for name in ('seams_pose_only.npz', 'seams_optimized.npz'):
        with np.load(cpu / name) as expected, np.load(cuda / name) as actual:
            assert expected.files == actual.files
            for key in expected.files:
                np.testing.assert_array_equal(expected[key], actual[key])
