#!/usr/bin/env python3
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

"""Create an index for hash-verified native inspection tiles."""

import argparse
from html import escape
import json
from pathlib import Path

from climbot_mosaic.evidence_chain import verify_inspection_tiles


def build_review_html(summary_path):
    """Verify every tile and return an index; no visual verdict is inferred."""
    summary_path = Path(summary_path)
    count = verify_inspection_tiles(summary_path)
    summary = json.loads(summary_path.read_text(encoding='utf-8'))
    rows = []
    for feature in summary['features']:
        label = escape(f"{feature['id']} ({feature['kind']})")
        for tile in feature['native_tiles']:
            location = escape(tile['file'], quote=True)
            digest = escape(tile['sha256'], quote=True)
            rows.append(
                f'<li><label><input type="checkbox" data-sha="{digest}"> '
                f'{label}: {escape(Path(tile["file"]).name)}</label> '
                f'<a href="{location}" target="_blank">原尺寸打开</a>'
                '<details><summary>预览</summary>'
                f'<img loading="lazy" src="{location}" '
                f'alt="{label}"></details></li>')
    if len(rows) != count:
        raise ValueError('The tile count changed during review preparation')
    return ("""<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<title>原尺寸 tile 人工复核</title><style>
body{font:16px/1.5 sans-serif;max-width:1100px;margin:2rem auto;padding:0 1rem}
li{margin:1rem 0;border-bottom:1px solid #ddd;padding-bottom:.5rem}
img{max-width:100%;height:auto} summary{cursor:pointer}
#progress{position:sticky;top:0;background:white}
</style><h1>原尺寸 tile 人工复核</h1>
<p>每张图片横向依次为真值、pose-only、optimized。必须点击“原尺寸打开”查看细节。
勾选仅记录本机浏览器的查看进度，不代表已通过；发现缺陷应另记位置和现象。</p>
<p id="progress"></p><ol>""" + '\n'.join(rows) + """</ol><script>
const boxes=[...document.querySelectorAll('input[data-sha]')];
function refresh(){document.getElementById('progress').textContent=
  `已查看 ${boxes.filter(x=>x.checked).length} / ${boxes.length}（仅本机进度）`;}
for(const box of boxes){const key='climbot-review-'+box.dataset.sha;
 box.checked=localStorage.getItem(key)==='1';
 box.addEventListener('change',()=>{localStorage.setItem(key,box.checked?'1':'0');refresh();});}
refresh();</script></html>""")


def main():
    """Prepare the viewing index next to its immutable summary and tiles."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('summary', type=Path)
    args = parser.parse_args()
    output = args.summary.parent / 'native_tile_review.html'
    output.write_text(build_review_html(args.summary), encoding='utf-8')
    print(output)


if __name__ == '__main__':
    main()
