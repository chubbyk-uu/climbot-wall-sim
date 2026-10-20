# Copyright 2026 jerry
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Reject mismatched physics libraries without disturbing the Mesa selection."""

import hashlib
import json
from pathlib import Path

from climbot_gazebo.ode_runtime import environment, PATCH_SHA256, SOURCE_SHA256
import pytest


def build(tmp_path):
    prefix = tmp_path / 'install'
    (prefix / 'lib').mkdir(parents=True)
    library = prefix / 'lib/libode.so.8'
    library.write_bytes(b'unit-test library identity')
    record = {'build_profile': 'ubuntu-noble-system-ccd-assertions',
              'cmake_options': ['-DODE_WITH_LIBCCD=ON', '-DODE_WITH_LIBCCD_SYSTEM=ON',
                                '-DODE_WITH_LIBCCD_BOX_CYL=OFF'],
              'version': '0.16.2', 'variant': 'hashfix', 'soname': 'libode.so.8',
              'regression_passed': True, 'source_sha256': SOURCE_SHA256,
              'patch_sha256': PATCH_SHA256,
              'library_sha256': hashlib.sha256(library.read_bytes()).hexdigest()}
    manifest = prefix.parent / 'build-result.json'
    manifest.write_text(json.dumps(record))
    return prefix, manifest, record


def test_child_only_loader_path_preserves_mesa_and_parent(tmp_path):
    """Private ODE must compose with the existing WSL Mesa workaround."""
    prefix, _, _ = build(tmp_path)
    original = {'LD_LIBRARY_PATH': '/mesa/lib:/ros/lib', 'LD_PRELOAD': '/mesa/lib/libEGL.so',
                'GLIBC_TUNABLES': 'glibc.cpu.hwcaps=-ERMS,-FSRM'}
    result, observed = environment(prefix, original)
    assert result['LD_LIBRARY_PATH'] == str(prefix / 'lib') + ':/mesa/lib:/ros/lib'
    assert original['LD_LIBRARY_PATH'] == '/mesa/lib:/ros/lib'
    assert result['LD_PRELOAD'] == original['LD_PRELOAD']
    assert result['GLIBC_TUNABLES'] == original['GLIBC_TUNABLES']
    assert observed['library'] == str(prefix / 'lib/libode.so.8')


@pytest.mark.parametrize('key,value', [
    ('build_profile', 'legacy-no-ccd'), ('cmake_options', []),
    ('variant', 'upstream'), ('regression_passed', False),
    ('source_sha256', 'wrong'), ('patch_sha256', 'wrong'), ('soname', 'libode.so.9')])
def test_wrong_build_is_rejected(tmp_path, key, value):
    """An existing file is not sufficient evidence of the intended fix."""
    prefix, manifest, record = build(tmp_path)
    record[key] = value
    manifest.write_text(json.dumps(record))
    with pytest.raises(ValueError, match='manifest or library'):
        environment(prefix, {})


def test_missing_build_names_the_fix(tmp_path):
    """A fresh machine meets this on the default wall; say how to build it."""
    with pytest.raises(FileNotFoundError, match='build_private_ode.py.*wall_surface:=flat'):
        environment(tmp_path / 'install', {})


def test_tampered_library_and_preload_conflict_are_rejected(tmp_path):
    """Detect replacement after validation and prevent a competing ODE preload."""
    prefix, _, _ = build(tmp_path)
    with pytest.raises(ValueError, match='preloaded ODE'):
        environment(prefix, {'LD_PRELOAD': '/other/libode.so.8'})
    (prefix / 'lib/libode.so.8').write_bytes(b'replaced')
    with pytest.raises(ValueError, match='checksum'):
        environment(prefix, {})


def test_runtime_pin_matches_build_recipe_patch():
    """Keep the deployed loader guard synchronized with the reviewed source patch."""
    root = Path(__file__).parents[3]
    patch = root / 'tools/patches/ode-0.16.2-hashspace-overflow.patch'
    assert hashlib.sha256(patch.read_bytes()).hexdigest() == PATCH_SHA256
