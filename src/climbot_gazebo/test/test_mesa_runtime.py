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

"""The WSL D3D12 launch must select a complete patched Mesa explicitly."""

import hashlib
import importlib.util
import json
from pathlib import Path

from climbot_gazebo.mesa_runtime import environment, GALLIUM, validate

from launch import LaunchContext

import pytest


def _fake_install(root):
    prefix = root / 'install'
    for name in (f'lib/{GALLIUM}', 'lib/libgbm.so.1',
                 'lib/libGLX_mesa.so.0', 'lib/libEGL_mesa.so.0',
                 'share/glvnd/egl_vendor.d/50_mesa.json'):
        path = prefix / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b'patched test library')
    for name in ('lib/dri', 'lib/gbm'):
        (prefix / name).mkdir()
    digest = hashlib.sha256(
        (prefix / 'lib' / GALLIUM).read_bytes()).hexdigest()
    (root / 'build-result.json').write_text(
        json.dumps({
            'source_version': '25.2.8-0ubuntu0.24.04.2',
            'libraries_sha256': {GALLIUM: digest},
        }), encoding='utf-8')
    return prefix


def test_missing_or_changed_patch_fails_before_launch(tmp_path):
    """Missing and tampered patched libraries must be rejected."""
    prefix = _fake_install(tmp_path)
    assert validate(prefix)['gallium_sha256']
    (prefix / 'lib' / GALLIUM).write_bytes(b'wrong build')
    with pytest.raises(ValueError, match='does not match'):
        validate(prefix)
    with pytest.raises(ValueError, match='missing'):
        validate(tmp_path / 'absent')


def test_explicit_system_restores_inherited_environment(tmp_path):
    """An explicit system run must not inherit a nested private preload."""
    prefix = _fake_install(tmp_path)
    original = {'LD_PRELOAD': '/opt/profiler.so', 'DISPLAY': ':0'}
    selected = environment('private', prefix, original)
    assert selected['LD_PRELOAD'].startswith(str(prefix / 'lib' / GALLIUM))
    assert selected['LD_PRELOAD'].endswith(':/opt/profiler.so')
    assert environment('system', prefix, selected) == original


def test_wsl_launch_rejects_missing_patch_before_rendering_world(
        tmp_path, monkeypatch):
    """Fail before world rendering when D3D12 has no patched Mesa."""
    launch_file = (Path(__file__).resolve().parents[1] /
                   'launch/climbot_wall.launch.py')
    spec = importlib.util.spec_from_file_location(
        'climbot_wall_mesa_test', launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setenv('CLIMBOT_MESA_PREFIX', str(tmp_path / 'absent'))
    monkeypatch.setattr(
        module, 'render_world', lambda *args: pytest.fail('render started'))
    context = LaunchContext()
    context.launch_configurations.update(
        gpu_backend='wsl_d3d12', mesa='private')
    with pytest.raises(ValueError, match='Patched Mesa is missing'):
        module.launch_setup(context)
