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

"""Every stage says what produced it, in the same shape, beside its own products."""

import json

from climbot_common.hashing import sha256_file
from climbot_mosaic.stage_provenance import (
    artifact,
    processed_run_inputs,
    STAGE_PROVENANCE_FILENAME,
    write_stage_provenance,
)
import pytest


def test_an_artifact_is_named_by_content_not_by_where_it_sat(tmp_path):
    path = tmp_path / 'pose_graph.json'
    path.write_text('{}\n', encoding='utf-8')
    record = artifact(path)
    assert record == {'name': 'pose_graph.json', 'sha256': sha256_file(path)}
    assert 'path' not in record


def test_processed_run_inputs_pair_each_run_with_its_manifest_digest():
    summary = {'source_run_ids': ['first', 'second'],
               'processing_manifest_sha256': ['a' * 64, 'b' * 64]}
    assert processed_run_inputs(summary) == {'processed_runs': [
        {'source_run_id': 'first', 'processing_manifest_sha256': 'a' * 64},
        {'source_run_id': 'second', 'processing_manifest_sha256': 'b' * 64}]}


def test_the_record_hashes_the_products_it_sits_beside(tmp_path):
    """It is written into the temporary directory, so record and products publish together."""
    (tmp_path / 'local_matches.json').write_text('{"matches": []}\n', encoding='utf-8')
    write_stage_provenance(tmp_path, 'local_matches', {'ratio_test': 0.75},
                           {'processed_runs': []}, ('local_matches.json',))
    record = json.loads((tmp_path / STAGE_PROVENANCE_FILENAME).read_text(encoding='utf-8'))
    assert record['stage'] == 'local_matches'
    assert record['parameters'] == {'ratio_test': 0.75}
    assert record['outputs']['local_matches.json']['sha256'] == sha256_file(
        tmp_path / 'local_matches.json')
    # The point of the whole file: a stage that cannot say which source built
    # it cannot be cited, so these two fields are never absent.
    assert set(record['git']) >= {'commit', 'source_modified', 'checked_pathspecs'}


def test_a_missing_product_fails_the_stage_rather_than_being_recorded_as_absent(tmp_path):
    with pytest.raises(OSError):
        write_stage_provenance(tmp_path, 'local_matches', {}, {}, ('local_matches.json',))


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__]))
