# Copyright 2026 jerry
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Explicit private ODE selection for M2 probes; never replace the system library."""

import hashlib
import json
from pathlib import Path

PATCH_SHA256 = 'fc35f0a3fc066f976f89557dbea39a7bced8fb06c8fe02cfa4754c2a441f2747'
SOURCE_SHA256 = '28b547ac9c3f2c8ba4c23a9cbe579667447616876c61b507a184f8607143050e'


def environment(prefix, inherited):
    """Validate source/fix/library identity, then set a child-only loader path."""
    prefix = Path(prefix).expanduser().resolve()
    if ':' in str(prefix):
        raise ValueError('ODE prefix cannot contain colon')
    record = prefix.parent / 'build-result.json'
    if not record.is_file():
        # The undulating wall is the default, so a fresh machine meets this first.
        raise FileNotFoundError(
            f'private ODE not built at {prefix}: run tools/build_private_ode.py --prefix '
            f'{prefix.parent} (see README), or launch with wall_surface:=flat')
    manifest = json.loads(record.read_text())
    library = prefix / 'lib/libode.so.8'
    if (manifest.get('variant') != 'hashfix' or manifest.get('version') != '0.16.2'
            or manifest.get('soname') != 'libode.so.8'
            or manifest.get('build_profile') != 'ubuntu-noble-system-ccd-assertions'
            or '-DODE_WITH_LIBCCD=ON' not in manifest.get('cmake_options', [])
            or '-DODE_WITH_LIBCCD_SYSTEM=ON' not in manifest.get('cmake_options', [])
            or '-DODE_WITH_LIBCCD_BOX_CYL=OFF' not in manifest.get('cmake_options', [])
            or manifest.get('regression_passed') is not True
            or manifest.get('source_sha256') != SOURCE_SHA256
            or manifest.get('patch_sha256') != PATCH_SHA256
            or hashlib.sha256(library.read_bytes()).hexdigest() != manifest.get('library_sha256')):
        raise ValueError('private ODE build manifest or library checksum mismatch')
    if any('libode' in item for item in inherited.get('LD_PRELOAD', '').split()):
        raise ValueError('preloaded ODE conflicts with explicit M2 loader selection')
    result = dict(inherited)
    result['LD_LIBRARY_PATH'] = ':'.join(filter(None, [
        str(prefix / 'lib'), inherited.get('LD_LIBRARY_PATH', '')]))
    return result, {'install': str(prefix), 'library': str(library.resolve()),
                    'library_sha256': manifest['library_sha256'], 'patch_sha256': PATCH_SHA256}
