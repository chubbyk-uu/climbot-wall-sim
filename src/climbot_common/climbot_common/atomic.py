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

"""Publish a JSON document whole or not at all, in one byte-stable form."""

import json
import os
from pathlib import Path
from uuid import uuid4


def json_text(document):
    """
    Render a document the one way every manifest in this repository renders it.

    ``allow_nan=False`` matters more than it looks: a NaN written here reads
    back as a number and compares false against everything, which is how an
    unusable bound can travel through a summary looking like a measurement.
    """
    return json.dumps(document, ensure_ascii=False, allow_nan=False,
                      indent=2, sort_keys=True) + '\n'


def write_json(path, document):
    """Write a JSON document atomically, leaving no partial file behind."""
    path = Path(path)
    temporary = path.with_name(f'.{path.name}.tmp-{uuid4().hex}')
    try:
        with open(temporary, 'w', encoding='utf-8') as handle:
            handle.write(json_text(document))
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
    return path
