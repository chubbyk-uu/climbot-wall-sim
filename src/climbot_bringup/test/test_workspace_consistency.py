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
Workspace-wide invariants no single package can check on its own.

climbot_bringup owns no node and nothing depends on it, so it is the one
place that may look across every package without creating an edge between
them. Both checks here are for mistakes that build cleanly, pass their own
package's tests, and only fail when something is actually launched.
"""

import ast
from pathlib import Path
import re

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
    PackageNotFoundError,
)
import pytest

PACKAGES = (
    'climbot_bringup',
    'climbot_control',
    'climbot_coverage',
    'climbot_description',
    'climbot_gazebo',
    'climbot_inspection',
)


def _source_root() -> Path:
    """
    Locate the checkout this test is running from.

    CMakeLists.txt is not installed anywhere, so unlike a config file there is
    no deployed copy to read and the source tree is the only source of truth
    for the domain assignments below.
    """
    root = Path(__file__).resolve().parents[3]
    packages = root / 'src'
    assert packages.is_dir(), (
        f'expected the workspace source tree at {packages}; this test reads '
        'CMakeLists.txt, which is never installed')
    return packages


def _literal_nodes(tree: ast.AST):
    """Yield (package, executable) for every Node(...) written as literals."""
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        name = node.func.attr if isinstance(node.func, ast.Attribute) else \
            getattr(node.func, 'id', None)
        if name != 'Node':
            continue
        fields = {}
        for keyword in node.keywords:
            if keyword.arg in ('package', 'executable') and \
                    isinstance(keyword.value, ast.Constant) and \
                    isinstance(keyword.value.value, str):
                fields[keyword.arg] = keyword.value.value
        if len(fields) == 2:
            yield fields['package'], fields['executable']


def test_every_launched_executable_is_installed():
    """
    A launch file naming an executable nobody installs fails only at run time.

    Renaming a target, or migrating a node from a Python script to a C++
    executable, leaves the launch file pointing at a name that no longer
    exists. Every package still builds and every unit test still passes; the
    first symptom is a mission that will not start.
    """
    missing = []
    checked = 0
    for package in PACKAGES:
        launch_dir = Path(get_package_share_directory(package)) / 'launch'
        if not launch_dir.is_dir():
            continue
        for launch_file in sorted(launch_dir.glob('*.launch.py')):
            tree = ast.parse(launch_file.read_text(encoding='utf-8'))
            for owner, executable in _literal_nodes(tree):
                checked += 1
                try:
                    prefix = Path(get_package_prefix(owner))
                except PackageNotFoundError:
                    missing.append(f'{launch_file.name}: package {owner} is not installed')
                    continue
                if not (prefix / 'lib' / owner / executable).exists():
                    missing.append(f'{launch_file.name}: {owner}/{executable}')
    assert checked > 0, 'no literal Node(package=, executable=) pair was found to check'
    assert missing == [], 'launch files name executables that are not installed: ' + \
        ', '.join(missing)


def test_launch_test_domain_ids_are_unique_across_the_workspace():
    """
    Two launch tests on one ROS domain see each other's topics.

    Each test starts real nodes on well-known names, so sharing a domain lets
    one test's task or odometry satisfy another's subscriber. The result is a
    failure in the innocent package, only under parallel test execution, and
    it reads as a regression in whatever that package tests. This has already
    happened twice: once between two of these packages, and once when an
    append past the end of one package's block landed on another's.
    """
    owners: dict[int, list[str]] = {}
    for cmake in sorted(_source_root().glob('*/CMakeLists.txt')):
        for domain in re.findall(r'ROS_DOMAIN_ID=(\d+)', cmake.read_text(encoding='utf-8')):
            owners.setdefault(int(domain), []).append(cmake.parent.name)
    assert owners, 'no ROS_DOMAIN_ID assignments were found to check'
    clashes = {domain: sorted(set(names)) for domain, names in owners.items() if len(names) > 1}
    assert clashes == {}, f'ROS_DOMAIN_ID reused by more than one launch test: {clashes}'


@pytest.mark.parametrize('package', PACKAGES)
def test_every_package_installs_its_launch_directory(package):
    """The checks above are only as good as their ability to find the files."""
    share = Path(get_package_share_directory(package))
    assert share.is_dir()
