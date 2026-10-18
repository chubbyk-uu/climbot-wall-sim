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

"""Select the shared, patched Mesa runtime for WSL D3D12 rendering."""

import hashlib
import json
import os
from pathlib import Path
import platform


VERSION = '25.2.8'
GALLIUM = f'libgallium-{VERSION}.so'
SNAPSHOT = 'CLIMBOT_MESA_ORIGINAL_ENV'
KEYS = ('LD_LIBRARY_PATH', 'LD_PRELOAD', 'LIBGL_DRIVERS_PATH',
        'GBM_BACKENDS_PATH', '__EGL_VENDOR_LIBRARY_FILENAMES',
        '__GLX_VENDOR_LIBRARY_NAME', 'GLIBC_TUNABLES')


def copy_tunables(value):
    """Avoid glibc REP MOVSB on WSL D3D12 upload mappings; preserve other tuning."""
    entries, caps = [], []
    for entry in value.split(':'):
        if entry.startswith('glibc.cpu.hwcaps='):
            caps.extend(cap for cap in entry.split('=', 1)[1].split(',')
                        if cap and cap.lstrip('-') not in ('ERMS', 'FSRM'))
        elif entry:
            entries.append(entry)
    entries.append('glibc.cpu.hwcaps=' + ','.join([*caps, '-ERMS', '-FSRM']))
    return ':'.join(entries)


def default_prefix(environ=None):
    """Find the shared Mesa install without embedding a user's home path."""
    env = os.environ if environ is None else environ
    chosen = env.get('CLIMBOT_MESA_PREFIX') or env.get('AGV_MESA_PREFIX')
    if chosen:
        return Path(chosen).expanduser()
    return Path.home() / 'opt' / 'agv-mesa-25.2.8' / 'install'


def libraries(prefix):
    """Return the private Mesa libraries that must be preloaded together."""
    directory = Path(prefix) / 'lib'
    return [directory / name for name in
            (GALLIUM, 'libgbm.so.1', 'libGLX_mesa.so.0', 'libEGL_mesa.so.0')]


def validate(prefix):
    """Fail before launch when the shared patch is incomplete."""
    prefix = Path(prefix).expanduser().resolve()
    if any(c.isspace() or c == ':' for c in str(prefix)):
        raise ValueError(
            'Mesa install prefix cannot contain whitespace or colon')
    required = [*libraries(prefix),
                prefix / 'share/glvnd/egl_vendor.d/50_mesa.json',
                prefix / 'lib/dri', prefix / 'lib/gbm']
    absent = [item for item in required if not item.exists()]
    if absent:
        raise ValueError(
            f'Patched Mesa is missing: {absent[0]}. Restore the shared '
            'Mesa install or build with python3 tools/build_private_mesa.py '
            '(docs/MESA_SETUP.md). Set CLIMBOT_MESA_PREFIX for another '
            'install; mesa:=system is only an explicit unpatched comparison.')
    record = prefix.parent / 'build-result.json'
    try:
        manifest = json.loads(record.read_text(encoding='utf-8'))
        expected = manifest['libraries_sha256'][GALLIUM]
    except (OSError, ValueError, KeyError, TypeError) as error:
        raise ValueError(
            f'Patched Mesa build record is missing or invalid: {record}'
        ) from error
    if manifest.get('source_version') != '25.2.8-0ubuntu0.24.04.2' or \
            not isinstance(expected, str) or len(expected) != 64:
        raise ValueError(
            'Patched Mesa build record has an unsupported version or digest')
    digest = hashlib.sha256(
        (prefix / 'lib' / GALLIUM).read_bytes()).hexdigest()
    if digest != expected:
        raise ValueError(
            'Patched Mesa Gallium library does not match its build record')
    return {'prefix': str(prefix), 'gallium_sha256': digest}


def environment(mode, prefix, inherited):
    """Return environment with validated private or explicit system Mesa."""
    env = dict(inherited)
    saved = env.pop(SNAPSHOT, None)
    if saved is not None:
        original = json.loads(saved)
        # Accept snapshots made before copy tuning was introduced.
        if set(original) not in (set(KEYS), set(KEYS) - {'GLIBC_TUNABLES'}):
            raise ValueError('Invalid inherited Mesa environment snapshot')
        for key, value in original.items():
            if value is None:
                env.pop(key, None)
            elif isinstance(value, str):
                env[key] = value
            else:
                raise ValueError('Invalid inherited Mesa environment value')
    if mode == 'system':
        return env
    if mode != 'private':
        raise ValueError('mesa must be private or system')
    identity = validate(prefix)
    directory = Path(identity['prefix']) / 'lib'
    env[SNAPSHOT] = json.dumps({key: env.get(key) for key in KEYS})
    env['LD_LIBRARY_PATH'] = str(directory) + (
        ':' + env['LD_LIBRARY_PATH'] if env.get('LD_LIBRARY_PATH') else '')
    env['LD_PRELOAD'] = ':'.join(map(str, libraries(identity['prefix']))) + \
        (':' + env['LD_PRELOAD'] if env.get('LD_PRELOAD') else '')
    env['LIBGL_DRIVERS_PATH'] = str(directory / 'dri')
    env['GBM_BACKENDS_PATH'] = str(directory / 'gbm')
    env['__EGL_VENDOR_LIBRARY_FILENAMES'] = str(
        Path(identity['prefix']) / 'share/glvnd/egl_vendor.d/50_mesa.json')
    env['__GLX_VENDOR_LIBRARY_NAME'] = 'mesa'
    # WSL upload buffers are very slow under REP MOVSB. Keep original assets
    # and pixels; select the vector copy path only for this private runtime.
    copy_mode = env.get('CLIMBOT_MEMCPY_MODE', 'auto')
    if copy_mode not in ('auto', 'default'):
        raise ValueError('CLIMBOT_MEMCPY_MODE must be auto or default')
    if (copy_mode == 'auto' and platform.machine() == 'x86_64' and
            'microsoft' in Path('/proc/version').read_text().lower()):
        env['GLIBC_TUNABLES'] = copy_tunables(env.get('GLIBC_TUNABLES', ''))
    return env


def rendering_environment(backend='auto', mode='private', inherited=None):
    """Resolve the shared runtime for new Gazebo entries and native probes."""
    original = os.environ if inherited is None else inherited
    if backend == 'auto':
        backend = 'wsl_d3d12' if 'microsoft' in Path('/proc/version').read_text().lower() \
            else 'native'
    if backend not in ('wsl_d3d12', 'software', 'native'):
        raise ValueError('invalid rendering backend')
    if mode not in ('private', 'system'):
        raise ValueError('mesa must be private or system')
    prefix = default_prefix(original)
    env = environment(mode if backend == 'wsl_d3d12' else 'system', prefix, original)
    identity = validate(prefix) if backend == 'wsl_d3d12' and mode == 'private' else None
    if backend == 'wsl_d3d12':
        env.update(GALLIUM_DRIVER='d3d12', MESA_D3D12_DEFAULT_ADAPTER_NAME='NVIDIA')
    elif backend == 'software':
        env['GALLIUM_DRIVER'] = 'llvmpipe'
        env.pop('MESA_D3D12_DEFAULT_ADAPTER_NAME', None)
    return env, {'backend': backend, 'mesa': mode if backend == 'wsl_d3d12' else 'system',
                 'gallium_sha256': identity['gallium_sha256'] if identity else None}
