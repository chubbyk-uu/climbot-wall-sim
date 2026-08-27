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

"""Keep committed project material free of machine-specific home directories."""

from pathlib import Path
import subprocess


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
FORBIDDEN_PATH_MARKERS = (
    b'/' + b'home/',
    b'/' + b'Users/',
    b'\\' + b'Users\\',
)


def test_tracked_files_do_not_embed_private_home_paths():
    tracked = subprocess.check_output(
        ['git', '-C', str(REPOSITORY_ROOT), 'ls-files', '-z'])
    offenders = []
    for raw_path in tracked.split(b'\0'):
        if not raw_path:
            continue
        path = REPOSITORY_ROOT / raw_path.decode('utf-8')
        if not path.is_file():
            continue
        contents = path.read_bytes()
        if any(marker in contents for marker in FORBIDDEN_PATH_MARKERS):
            offenders.append(str(raw_path, 'utf-8'))
    assert not offenders, f'machine-specific home paths found in: {offenders}'
