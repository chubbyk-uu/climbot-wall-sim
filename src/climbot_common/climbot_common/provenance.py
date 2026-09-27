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

"""
Record what a run was actually produced by, not what it was asked for.

Two archives were once filed as baselines from modified trees because the
field that would have said so was written and never read.  A formal mosaic
summary was then filed naming a commit that did not contain the code which
produced its own fields, because nothing generated that field at all -- it was
typed in.  Both failures are the same one: a provenance field that no program
computes cannot be wrong in a way anybody notices.

So this lives in a package with no ROS dependency, and every stage that writes
evidence calls it rather than describing itself.
"""

import hashlib
import os
from pathlib import Path
import subprocess
import sys

#: Where source that can change a result lives.  ``tools`` is not optional:
#: the summary generators run from there, and a pathspec of ``src`` alone
#: reports a dirty generator as a clean tree -- which is how an unciteable
#: summary got filed as formal evidence in the first place.
DEFAULT_PATHSPECS = ('src', 'tools')


def runtime_source_state(path=None, modules=None, entrypoint=None):
    """
    Check the bytes Python actually loaded against the current Git commit.

    A clean source tree alone does not show that an installed copy was rebuilt.
    Native extensions are hashed as executed artifacts; their build identity
    must be checked separately by an acceptance runner.
    """
    directory = Path(path or __file__)
    working_directory = directory if directory.is_dir() else directory.parent
    try:
        root = Path(subprocess.check_output(
            ['git', 'rev-parse', '--show-toplevel'], cwd=working_directory,
            text=True, timeout=5).strip())
    except (OSError, subprocess.SubprocessError):
        return {'source_matches_commit': False, 'files': []}
    selected = modules if modules is not None else sys.modules
    candidates = []
    for name, module in sorted(selected.items()):
        if not name.startswith(('climbot_common.', 'climbot_mosaic.')):
            continue
        location = getattr(module, '__file__', None)
        if location:
            package = name.split('.')[0]
            subpath = '/'.join(name.split('.')[1:])
            if Path(location).suffix == '.py':
                candidates.append((
                    f'src/{package}/{package}/{subpath}.py', Path(location)))
            elif '.so' in Path(location).name:
                candidates.append((None, Path(location)))
    command = Path(sys.argv[0] if entrypoint is None else entrypoint)
    if command.is_file():
        if command.name in ('build_mosaic_evidence_summary.py',):
            candidates.append((f'tools/{command.name}', command))
        elif (root / 'src/climbot_mosaic/scripts' / command.name).is_file():
            candidates.append((
                f'src/climbot_mosaic/scripts/{command.name}', command))
    files = []
    for source, actual in candidates:
        try:
            payload = actual.read_bytes()
            digest = hashlib.sha256(payload).hexdigest()
            expected = (subprocess.check_output(
                ['git', 'show', f'HEAD:{source}'], cwd=root, timeout=5)
                if source else None)
            matches = expected == payload if expected is not None else None
        except (OSError, subprocess.SubprocessError):
            digest, matches = None, False
        files.append({'source': source, 'runtime_file': actual.name,
                      'sha256': digest, 'matches_commit': matches})
    return {'source_matches_commit': bool(files) and all(
        item['matches_commit'] is not False for item in files), 'files': files}


def git_state(pathspecs=DEFAULT_PATHSPECS, path=None):
    """
    Describe the source revision, or nulls when git is unavailable.

    ``pathspecs`` bounds what counts as a modification, so untracked notes and
    build output elsewhere do not mark a reproducible run as modified.  It is
    reported back as ``checked_pathspecs``: a bare ``source_modified`` does not
    say what it looked at, and a reader cannot otherwise tell a clean tree from
    an unexamined one.
    """
    directory = path or os.path.dirname(os.path.abspath(__file__))
    checked = tuple(pathspecs)

    def capture(arguments):
        return subprocess.run(
            ['git'] + arguments, check=True, capture_output=True,
            text=True, timeout=5.0, cwd=directory).stdout.strip()

    try:
        root = capture(['rev-parse', '--show-toplevel'])
        # --porcelain lists untracked files too, so new uncommitted source
        # under these paths counts as a modification rather than passing as a
        # clean tree.
        modified = bool(capture(
            ['-C', root, 'status', '--porcelain', '--'] + list(checked)))
        return {
            'commit': capture(['rev-parse', 'HEAD']),
            'branch': capture(['rev-parse', '--abbrev-ref', 'HEAD']),
            'source_modified': modified,
            'checked_pathspecs': list(checked),
            # The question source_modified was added to answer, stated as the
            # answer rather than as its input. A field nobody reads cannot stop
            # an untraceable run from being filed as a baseline.
            'traceable': not modified,
        }
    except (OSError, subprocess.SubprocessError):
        return {
            'commit': None, 'branch': None, 'source_modified': None,
            'checked_pathspecs': list(checked), 'traceable': False,
        }
