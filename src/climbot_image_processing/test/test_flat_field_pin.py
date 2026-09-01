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

"""A processed run must not silently take the wrong flat field, or none."""

import hashlib
import os
from pathlib import Path
import re
import subprocess
import sys

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE_ROOT / 'scripts' / 'process_inspection_archive'
PIN_PATH = PACKAGE_ROOT / 'config' / 'inspection_flat_field.yaml'


def _run(*arguments):
    """Invoke the CLI far enough to reach its flat-field decision."""
    # The script imports its own package at module scope, which resolves from
    # the install space under colcon and from here when pytest runs bare.
    environment = dict(os.environ)
    environment['PYTHONPATH'] = os.pathsep.join(
        [str(PACKAGE_ROOT)] + ([environment['PYTHONPATH']]
                               if environment.get('PYTHONPATH') else []))
    return subprocess.run(
        [sys.executable, str(SCRIPT), *arguments],
        capture_output=True, text=True, cwd=str(PACKAGE_ROOT), env=environment)


def _pin():
    with open(PIN_PATH) as handle:
        return yaml.safe_load(handle)


def test_the_pin_names_a_calibration_and_a_digest():
    """A pin missing either field disables the check without saying so."""
    document = _pin()
    assert document['file_name'].endswith('.npz'), document['file_name']
    assert re.fullmatch(r'[0-9a-f]{64}', document['sha256']), document['sha256']
    assert document['measured_against'].strip(), 'the pin records no provenance'


def test_omitting_the_flat_field_is_refused(tmp_path):
    """
    Absence of a flat field must be a decision, not a default.

    P2-06 was processed for a week with a calibration that did not match the
    photographed material, and nothing in the chain objected.
    """
    result = _run('--input-run', str(tmp_path / 'run'),
                  '--output-dir', str(tmp_path / 'out'))
    assert result.returncode == 2, result.stdout
    assert 'no --flat-field-file given' in result.stderr, result.stderr
    assert _pin()['file_name'] in result.stderr


def test_an_unpinned_calibration_is_refused(tmp_path):
    """A stale calibration produces a normal-looking, several-percent-wrong run."""
    other = tmp_path / 'flat_field_someone_elses.npz'
    other.write_bytes(b'not the accepted calibration')
    result = _run('--input-run', str(tmp_path / 'run'),
                  '--output-dir', str(tmp_path / 'out'),
                  '--flat-field-file', str(other))
    assert result.returncode == 2, result.stdout
    assert 'is not the accepted calibration' in result.stderr, result.stderr
    assert hashlib.sha256(other.read_bytes()).hexdigest() in result.stderr


def test_both_escape_hatches_are_explicit(tmp_path):
    """Either override must get past the check and fail later, on real inputs."""
    other = tmp_path / 'flat_field_someone_elses.npz'
    other.write_bytes(b'not the accepted calibration')
    for arguments in (
            ('--no-flat-field',),
            ('--flat-field-file', str(other), '--allow-unpinned-flat-field')):
        result = _run('--input-run', str(tmp_path / 'missing_run'),
                      '--output-dir', str(tmp_path / 'out'), *arguments)
        assert 'accepted calibration' not in result.stderr, result.stderr
        assert 'no --flat-field-file given' not in result.stderr, result.stderr


def test_a_pin_whose_digest_is_all_digits_is_refused(tmp_path, monkeypatch):
    """
    A pin that cannot be read must refuse, never skip.

    YAML reads an unquoted 64-digit sha256 as the integer 0, which is falsy.
    An earlier version of this guard treated that as "no pin" and let every
    calibration through without saying anything.
    """
    package = tmp_path / 'climbot_image_processing'
    (package / 'config').mkdir(parents=True)
    (package / 'scripts').mkdir()
    (package / 'config' / 'inspection_flat_field.yaml').write_text(
        'file_name: something.npz\nsha256: ' + '0' * 64 + '\n')
    (package / 'scripts' / 'process_inspection_archive').write_text(SCRIPT.read_text())
    environment = dict(os.environ)
    environment['PYTHONPATH'] = os.pathsep.join(
        [str(PACKAGE_ROOT)] + ([environment['PYTHONPATH']]
                               if environment.get('PYTHONPATH') else []))
    # Nothing may resolve the real installed pin instead of this broken one.
    environment['AMENT_PREFIX_PATH'] = str(tmp_path / 'empty')
    result = subprocess.run(
        [sys.executable, str(package / 'scripts' / 'process_inspection_archive'),
         '--input-run', str(tmp_path / 'run'), '--output-dir', str(tmp_path / 'out'),
         '--flat-field-file', str(tmp_path / 'any.npz')],
        capture_output=True, text=True, env=environment)
    assert result.returncode == 2, result.stdout
    assert 'no usable sha256' in result.stderr, result.stderr


def test_the_two_flat_field_switches_are_exclusive(tmp_path):
    """Asking for none and for one at once is a mistake, not a precedence rule."""
    result = _run('--input-run', str(tmp_path / 'run'),
                  '--output-dir', str(tmp_path / 'out'),
                  '--no-flat-field', '--flat-field-file', str(tmp_path / 'f.npz'))
    assert result.returncode == 2, result.stdout
    assert 'exclusive' in result.stderr, result.stderr
