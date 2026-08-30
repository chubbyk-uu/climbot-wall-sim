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

"""A summary is published whole or not at all, in one byte-stable form."""

import json

from climbot_common.atomic import json_text, write_json
import pytest


def test_documents_are_rendered_in_one_stable_form():
    text = json_text({'b': 1, 'a': [2, 3]})
    assert text == '{\n  "a": [\n    2,\n    3\n  ],\n  "b": 1\n}\n'


def test_a_not_a_number_is_refused_rather_than_written():
    """A not-a-number reads back as a number and compares false against every threshold."""
    with pytest.raises(ValueError):
        json_text({'scale_error_ppm': float('nan')})


def test_writing_publishes_the_document_and_leaves_no_temporary(tmp_path):
    path = write_json(tmp_path / 'summary.json', {'status': 'completed'})
    assert json.loads(path.read_text(encoding='utf-8')) == {'status': 'completed'}
    assert [entry.name for entry in tmp_path.iterdir()] == ['summary.json']


def test_a_refused_document_leaves_the_previous_file_intact(tmp_path):
    path = write_json(tmp_path / 'summary.json', {'status': 'completed'})
    with pytest.raises(ValueError):
        write_json(path, {'status': float('inf')})
    assert json.loads(path.read_text(encoding='utf-8')) == {'status': 'completed'}
    assert [entry.name for entry in tmp_path.iterdir()] == ['summary.json']


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__]))
