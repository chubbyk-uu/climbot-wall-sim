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

"""One definition of the digest that ties a stage's output to the next stage's input."""

import hashlib

#: Large enough that hashing a multi-gigabyte mosaic is bound by the disk
#: rather than by Python, small enough not to matter for a manifest.
_CHUNK_BYTES = 1 << 20


def sha256_bytes(payload):
    """Return the hex digest of a bytes-like payload."""
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path):
    """Return the hex digest of a file, read in chunks so size does not matter."""
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        for chunk in iter(lambda: handle.read(_CHUNK_BYTES), b''):
            digest.update(chunk)
    return digest.hexdigest()
