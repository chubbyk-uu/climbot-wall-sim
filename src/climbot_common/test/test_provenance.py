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

"""What a provenance field claims has to be something a program checked."""

import subprocess

from climbot_common.provenance import DEFAULT_PATHSPECS, git_state
import pytest


def _repository(root):
    """Build a one-commit repository with source under src and notes under docs."""
    def run(*arguments):
        subprocess.run(['git'] + list(arguments), cwd=root, check=True,
                       capture_output=True, text=True)

    run('init', '--quiet')
    run('config', 'user.email', 'test@example.invalid')
    run('config', 'user.name', 'test')
    (root / 'src').mkdir()
    (root / 'docs').mkdir()
    (root / 'src' / 'module.py').write_text('value = 1\n', encoding='utf-8')
    run('add', 'src/module.py')
    run('commit', '--quiet', '-m', 'initial')
    return root


def test_clean_tree_is_traceable_and_says_what_it_checked(tmp_path):
    repository = _repository(tmp_path)
    state = git_state(path=repository)
    assert state['source_modified'] is False
    assert state['traceable'] is True
    assert state['checked_pathspecs'] == list(DEFAULT_PATHSPECS)
    assert len(state['commit']) == 40


def test_untracked_source_counts_as_modified(tmp_path):
    """A new uncommitted file changes the result, so it cannot read as clean."""
    repository = _repository(tmp_path)
    (repository / 'src' / 'added.py').write_text('value = 2\n', encoding='utf-8')
    state = git_state(path=repository)
    assert state['source_modified'] is True
    assert state['traceable'] is False


def test_the_generator_directory_is_inside_the_default_pathspec(tmp_path):
    """
    A summary generator lives in tools, and a pathspec of src alone reported it clean.

    That is exactly how a formal summary came to name a commit that could not
    have produced its own fields, so tools is part of the default.
    """
    repository = _repository(tmp_path)
    (repository / 'tools').mkdir()
    (repository / 'tools' / 'summarise.py').write_text('value = 3\n', encoding='utf-8')
    assert git_state(path=repository)['source_modified'] is True
    assert git_state(pathspecs=('src',), path=repository)['source_modified'] is False


def test_changes_outside_the_pathspec_do_not_mark_a_run_modified(tmp_path):
    repository = _repository(tmp_path)
    (repository / 'docs' / 'note.md').write_text('scratch\n', encoding='utf-8')
    assert git_state(path=repository)['source_modified'] is False


def test_a_directory_without_git_reports_nulls_rather_than_guessing(tmp_path):
    state = git_state(path=tmp_path / 'absent')
    assert state == {
        'commit': None, 'branch': None, 'source_modified': None,
        'checked_pathspecs': list(DEFAULT_PATHSPECS), 'traceable': False,
    }


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__]))
