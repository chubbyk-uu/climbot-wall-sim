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

"""The native-tile viewer must label its three panels and reject stale tiles."""

import hashlib
import importlib.util
import json
from pathlib import Path
import re

from climbot_mosaic.evidence_chain import EvidenceChainError
import pytest


def _viewer():
    path = Path(__file__).resolve().parents[3] / 'tools/prepare_native_tile_review.py'
    spec = importlib.util.spec_from_file_location('prepare_native_tile_review', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_viewer_has_labelled_panels_and_verifies_bytes(tmp_path):
    """A tile cannot enter the viewer without matching its pinned digest."""
    image = tmp_path / 'native_tiles/crack_01/r0000_c0000.png'
    image.parent.mkdir(parents=True)
    image.write_bytes(b'example image bytes')
    summary = tmp_path / 'diagnostic_inspection_summary.json'
    summary.write_text(json.dumps({'features': [{
        'id': 'crack_decal_01', 'kind': 'crack_decal',
        'native_tiles': [{
            'file': str(image.relative_to(tmp_path)),
            'bytes': image.stat().st_size,
            'sha256': hashlib.sha256(image.read_bytes()).hexdigest(),
            'panel_width_px': 2, 'height_px': 3,
        }],
    }]}), encoding='utf-8')
    html = _viewer().build_review_html(summary)
    match = re.search(r'const tiles = (\[.*?\]);', html)
    tiles = json.loads(match.group(1))
    assert len(tiles) == 1
    assert tiles[0]['width'] == 2
    assert tiles[0]['label'].startswith('裂缝')
    for heading in ('真值（理想墙面）', '仅位姿拼接（对照）',
                    '优化后拼接（重点检查）'):
        assert heading in html
    assert '原尺寸 100%' in html
    assert '断开、错位、重影或消失' in html
    assert str(tmp_path) not in html

    image.write_bytes(b'changed')
    with pytest.raises(EvidenceChainError):
        _viewer().build_review_html(summary)
