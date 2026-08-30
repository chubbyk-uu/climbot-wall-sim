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

import os
import subprocess

#: Where source that can change a result lives.  ``tools`` is not optional:
#: the summary generators run from there, and a pathspec of ``src`` alone
#: reports a dirty generator as a clean tree -- which is how an unciteable
#: summary got filed as formal evidence in the first place.
DEFAULT_PATHSPECS = ('src', 'tools')


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
