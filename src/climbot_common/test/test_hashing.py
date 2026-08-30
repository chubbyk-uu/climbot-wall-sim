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

"""One digest definition, so a stage's output hash matches the next stage's input hash."""

import hashlib

from climbot_common.hashing import sha256_bytes, sha256_file
import pytest


def test_file_and_bytes_digests_agree_with_hashlib(tmp_path):
    payload = b'diagnostic wall truth\n'
    path = tmp_path / 'payload.bin'
    path.write_bytes(payload)
    expected = hashlib.sha256(payload).hexdigest()
    assert sha256_bytes(payload) == expected
    assert sha256_file(path) == expected


def test_a_file_larger_than_one_chunk_is_read_whole(tmp_path):
    """Chunked reading is why manifests can hash a mosaic, so it has to be exact."""
    payload = bytes(range(256)) * 8192
    path = tmp_path / 'large.bin'
    path.write_bytes(payload)
    assert len(payload) > (1 << 20)
    assert sha256_file(path) == hashlib.sha256(payload).hexdigest()


def test_an_empty_file_hashes_rather_than_failing(tmp_path):
    path = tmp_path / 'empty.bin'
    path.write_bytes(b'')
    assert sha256_file(path) == hashlib.sha256(b'').hexdigest()


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__]))
