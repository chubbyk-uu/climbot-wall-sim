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
import subprocess
import sys
from types import SimpleNamespace

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


def _builder():
    script = (Path(__file__).resolve().parents[3] /
              'tools/build_private_mesa.py')
    spec = importlib.util.spec_from_file_location('mesa_builder_test', script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_builder_help_needs_no_ros_or_other_workspace():
    """The standalone recipe is usable before colcon or ROS environment setup."""
    script = _builder().REPO / 'tools/build_private_mesa.py'
    result = subprocess.run(
        [sys.executable, '-I', str(script), '--help'],
        capture_output=True, text=True, check=True)
    assert '--root' in result.stdout
    assert '--cache' in result.stdout
    assert '--jobs' in result.stdout


def test_build_lock_matches_runtime_and_local_patch():
    """The locally shipped patch and recipe must match runtime expectations."""
    builder = _builder()
    lock = json.loads((builder.REPO / 'tools/patches/mesa-build-lock.json').read_text())
    assert lock['ubuntu_source_version'] == '25.2.8-0ubuntu0.24.04.2'
    builder.verify(builder.REPO / lock['project_patch'], lock['project_patch_sha256'])
    assert '-Dgallium-drivers=d3d12' in lock['configuration']
    assert '-Dllvm=disabled' in lock['configuration']
    assert '--libdir=lib' in lock['configuration']
    for digest in [*lock['downloads'].values(), *lock['dependency_packages'].values()]:
        assert len(digest) == 64
        int(digest, 16)


@pytest.mark.parametrize('bad_root', ['existing', 'has space', 'has:colon'])
def test_builder_rejects_unsafe_or_existing_root(tmp_path, monkeypatch, bad_root):
    """No downloads, overwrite or deletion before root validation."""
    builder = _builder()
    monkeypatch.setattr(builder.platform, 'freedesktop_os_release',
                        lambda: {'ID': 'ubuntu', 'VERSION_ID': '24.04'})
    monkeypatch.setattr(builder.platform, 'machine', lambda: 'x86_64')
    monkeypatch.setattr(builder, 'urlopen', lambda *a, **k: pytest.fail('download'))
    root = tmp_path / bad_root
    if bad_root == 'existing':
        root.mkdir()
        (root / 'keep').write_text('existing shared runtime')
    args = SimpleNamespace(root=root, jobs=8, cache=None)
    with pytest.raises(ValueError, match='already exists|whitespace or colon'):
        builder.build(args)
    if bad_root == 'existing':
        assert (root / 'keep').read_text() == 'existing shared runtime'
    else:
        assert not root.exists()


def test_builder_rejects_changed_cache_before_extracting(tmp_path, monkeypatch):
    """A supplied cache never disables the pinned SHA-256 check."""
    builder = _builder()
    monkeypatch.setattr(builder.platform, 'freedesktop_os_release',
                        lambda: {'ID': 'ubuntu', 'VERSION_ID': '24.04'})
    monkeypatch.setattr(builder.platform, 'machine', lambda: 'x86_64')
    monkeypatch.setattr(builder.shutil, 'which', lambda command: command)
    monkeypatch.setattr(builder, 'urlopen', lambda *a, **k: pytest.fail('download'))
    monkeypatch.setattr(builder.subprocess, 'run',
                        lambda *a, **k: pytest.fail('subprocess'))
    cache = tmp_path / 'cache'
    (cache / 'downloads').mkdir(parents=True)
    (cache / 'downloads/mesa.dsc').write_bytes(b'wrong cached source')
    root = tmp_path / 'new-build'
    with pytest.raises(ValueError, match='SHA256 mismatch: mesa.dsc'):
        builder.build(SimpleNamespace(root=root, jobs=8, cache=cache))
    assert (root / 'COLCON_IGNORE').exists()
    assert not (root / 'install').exists()


def test_builder_publishes_runtime_compatible_record(tmp_path, monkeypatch):
    """Exercise build orchestration without network or a real Mesa compilation."""
    builder = _builder()
    monkeypatch.setattr(builder.platform, 'freedesktop_os_release',
                        lambda: {'ID': 'ubuntu', 'VERSION_ID': '24.04'})
    monkeypatch.setattr(builder.platform, 'machine', lambda: 'x86_64')
    monkeypatch.setattr(builder.shutil, 'which', lambda command: command)
    monkeypatch.setattr(builder, 'urlopen', lambda *a, **k: pytest.fail('download'))
    repo = tmp_path / 'recipe'
    (repo / 'tools/patches').mkdir(parents=True)
    patch = repo / 'tools/patches/fix.patch'
    patch.write_bytes(b'locked patch')
    cache = tmp_path / 'cache'
    (cache / 'downloads').mkdir(parents=True)
    (cache / 'deps').mkdir()
    source = cache / 'downloads/mesa.dsc'
    source.write_bytes(b'locked source')
    dep = cache / 'deps/example_1_amd64.deb'
    dep.write_bytes(b'locked dependency')

    def digest(path):
        return hashlib.sha256(path.read_bytes()).hexdigest()

    lock = {
        'ubuntu_source_version': '25.2.8-0ubuntu0.24.04.2',
        'downloads': {'mesa.dsc': digest(source)},
        'dependency_packages': {dep.name: digest(dep)},
        'project_patch': str(patch.relative_to(repo)),
        'project_patch_sha256': digest(patch),
        'configuration': ['--libdir=lib', '-Dgallium-drivers=d3d12'],
    }
    (repo / 'tools/patches/mesa-build-lock.json').write_text(json.dumps(lock))
    monkeypatch.setattr(builder, 'REPO', repo)
    root = tmp_path / 'new-build'
    calls = []

    def fake_run(command, **kwargs):
        calls.append(command)
        if command[0] == 'dpkg-source':
            (root / 'source').mkdir()
        if 'install' in command:
            _fake_install(root)
        return SimpleNamespace(returncode=0, stdout='dependencies resolved\n', stderr='')

    monkeypatch.setattr(builder.subprocess, 'run', fake_run)
    builder.build(SimpleNamespace(root=root, jobs=4, cache=cache))
    record = json.loads((root / 'build-result.json').read_text())
    assert validate(root / 'install')['gallium_sha256'] == record['libraries_sha256'][GALLIUM]
    assert record['runtime_tested'] is False
    assert (root / 'COLCON_IGNORE').is_file()
    assert ['ninja', '-C', str(root / 'build'), '-j4'] in calls
    assert any(command[0] == 'patch' and '--fuzz=0' in command for command in calls)
    assert not any(command[0] in ('sudo', 'apt-get') for command in calls)
