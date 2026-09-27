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

"""Create a labelled, one-tile-at-a-time native inspection viewer."""

import argparse
import json
from pathlib import Path

from climbot_mosaic.evidence_chain import verify_inspection_tiles


FEATURE_NAMES = {
    'construction_seam': '板缝',
    'crack_decal': '裂缝',
    'repair_patch': '修补块',
    'graffiti_decal': '涂鸦',
}

PAGE_HTML = """<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>诊断墙拼接人工复核</title>
<style>
*{box-sizing:border-box}
body{margin:0;background:#f2f4f7;color:#18202a;
 font:16px/1.5 system-ui,"Noto Sans CJK SC",sans-serif}
main{max-width:1500px;margin:auto;padding:16px}
h1{font-size:1.5rem;margin:0 0 8px} h2{font-size:1.05rem;margin:0}
.box{background:white;border:1px solid #d8dee6;border-radius:10px;
 padding:14px;margin:12px 0}
.guide{background:#fff8e8;border-color:#ead49d}
.guide ol{margin:6px 0;padding-left:23px}
.toolbar{position:sticky;top:0;z-index:2;box-shadow:0 2px 7px #0002}
.controls{display:flex;align-items:center;gap:9px;flex-wrap:wrap}
button,select{font:inherit;padding:6px 10px} button{cursor:pointer}
#position{font-weight:700;min-width:90px;text-align:center}
#name{font-weight:700} .hint{font-size:.9rem;color:#526071}
.scroll{overflow:auto;max-width:100%}
.comparison{display:flex;gap:10px;width:max-content;align-items:flex-start}
.column{flex:none}
.column header{padding:8px;background:#eaf0f7;border:1px solid #d8dee6}
.column:last-child header{background:#e4f4e8}
.column small{display:block;color:#526071}
.window{position:relative;overflow:hidden;background:#242932;
 border:1px solid #aeb8c4}
.window img{position:absolute;top:0;max-width:none;height:auto}
textarea{width:100%;min-height:65px;font:inherit;padding:8px}
</style></head><body><main>
<h1>诊断墙拼接人工复核</h1>
<section class="box guide"><strong>这三列怎么看？</strong><ol>
<li>左列是真值（理想墙面），中列是仅位姿拼接的对照，
右列是优化后拼接——重点看右列能否保留左列的图案。</li>
<li>板缝、裂缝、修补块和涂鸦在右列不应断开、错位、重影或消失。
中列差一些没关系；单纯明暗变化可能是 hard-cut 亮度接缝。</li>
<li>先用概览找到图案，再按“原尺寸 100%”看细节。
大图可横向滚动，或用“显示”下拉框只看一列。</li>
</ol><p class="hint">黑色裁剪边缘不一定是缺陷；
但左列有图案、右列同一位置却缺失时，应标记“需复查”。</p>
</section>
<section class="box toolbar"><div class="controls">
<button id="prev">← 上一张</button><span id="position"></span>
<button id="next">下一张 →</button>
<label>跳到特征 <select id="feature"></select></label>
<button id="fit">适合屏幕概览</button>
<button id="native">原尺寸 100%</button>
<label>显示 <select id="focus">
<option value="all">三列并排</option><option value="0">只看真值</option>
<option value="1">只看仅位姿</option><option value="2">只看优化后</option>
</select></label></div><p id="progress" class="hint"></p></section>
<section class="box"><div id="name"></div><p id="size" class="hint"></p>
<div class="scroll"><div class="comparison" id="comparison"></div></div>
<p><a id="raw" target="_blank" rel="noopener">打开原始三列图片</a></p>
</section>
<section class="box"><strong>本机查看记录（不是自动验收）</strong>
<div class="controls"><button id="ok">这张未见明显几何缺陷</button>
<button id="issue">这张需复查</button><button id="clear">清除标记</button>
</div><p id="decision"></p><label>问题位置和现象（可留空）
<textarea id="note" placeholder="例如：优化后右下角板缝断开"></textarea>
</label><p class="hint">标记只保存在本机浏览器，未逐张查看的图片仍是未复核。</p>
</section></main><script>
const tiles = __TILES__;
const labels = ['真值（理想墙面）', '仅位姿拼接（对照）',
                '优化后拼接（重点检查）'];
const subtitles = ['应该出现的图案', '不做优化时的样子',
                   '检查断线、重影、错位和缺失'];
const prefix = 'climbot-native-review-v2:';
const memory = new Map();
function read(key) {
  try { return localStorage.getItem(prefix + key); }
  catch (_) { return memory.get(key) || null; }
}
function save(key, value) {
  try { localStorage.setItem(prefix + key, value); }
  catch (_) { memory.set(key, value); }
}
let index = Number(read('last-index') || 0), scale = 0;
if (!Number.isInteger(index) || index < 0 || index >= tiles.length) index = 0;
function fitScale(tile) {
  const available = Math.max(210, window.innerWidth - 90);
  return Math.min(1, Math.max(.1, available / (3 * tile.width + 24)));
}
function progress() {
  let ok = 0, issue = 0;
  for (const tile of tiles) {
    const status = read(tile.sha256 + ':status');
    if (status === 'ok') ok++;
    if (status === 'issue') issue++;
  }
  document.getElementById('progress').textContent =
    `已标记 ${ok + issue} / ${tiles.length}：无明显问题 ${ok}，` +
    `需复查 ${issue}；其余未看。此计数不是正式验收。`;
}
function draw() {
  const tile = tiles[index];
  if (!scale) scale = fitScale(tile);
  document.getElementById('position').textContent =
    `${index + 1} / ${tiles.length}`;
  document.getElementById('name').textContent =
    `${tile.label} · ${tile.file.split('/').pop()}`;
  document.getElementById('size').textContent =
    `每列原图 ${tile.width} × ${tile.height} 像素，` +
    `当前显示 ${Math.round(scale * 100)}%。`;
  document.getElementById('raw').href = tile.file;
  document.getElementById('feature').value = tile.feature;
  const holder = document.getElementById('comparison');
  holder.replaceChildren();
  const focus = document.getElementById('focus').value;
  for (let part = 0; part < 3; part++) {
    if (focus !== 'all' && focus !== String(part)) continue;
    const column = document.createElement('div');
    column.className = 'column';
    const head = document.createElement('header');
    const title = document.createElement('h2');
    title.textContent = `${part + 1}. ${labels[part]}`;
    const subtitle = document.createElement('small');
    subtitle.textContent = subtitles[part];
    head.append(title, subtitle);
    const pane = document.createElement('div');
    pane.className = 'window';
    pane.style.width = `${tile.width * scale}px`;
    pane.style.height = `${tile.height * scale}px`;
    const image = document.createElement('img');
    image.src = tile.file;
    image.alt = labels[part];
    image.style.width = `${3 * tile.width * scale}px`;
    image.style.left = `${-part * tile.width * scale}px`;
    pane.append(image);
    column.append(head, pane);
    holder.append(column);
  }
  const status = read(tile.sha256 + ':status');
  document.getElementById('decision').textContent =
    status === 'issue' ? '已标记：需复查' :
    status === 'ok' ? '已标记：未见明显几何缺陷' : '尚未标记';
  document.getElementById('note').value =
    read(tile.sha256 + ':note') || '';
  document.getElementById('prev').disabled = index === 0;
  document.getElementById('next').disabled = index === tiles.length - 1;
  progress();
}
function move(next) {
  index = Math.min(tiles.length - 1, Math.max(0, next));
  save('last-index', String(index));
  scale = 0;
  draw();
  window.scrollTo(0, 0);
}
const feature = document.getElementById('feature');
for (let i = 0; i < tiles.length; i++) {
  if (i && tiles[i].feature === tiles[i - 1].feature) continue;
  const option = document.createElement('option');
  option.value = tiles[i].feature;
  option.textContent = tiles[i].label;
  option.dataset.first = i;
  feature.append(option);
}
feature.onchange = () => move(Number(feature.selectedOptions[0].dataset.first));
document.getElementById('prev').onclick = () => move(index - 1);
document.getElementById('next').onclick = () => move(index + 1);
document.getElementById('fit').onclick = () => {
  scale = fitScale(tiles[index]); draw();
};
document.getElementById('native').onclick = () => { scale = 1; draw(); };
document.getElementById('focus').onchange = draw;
function decide(value) {
  save(tiles[index].sha256 + ':status', value);
  draw();
}
document.getElementById('ok').onclick = () => decide('ok');
document.getElementById('issue').onclick = () => decide('issue');
document.getElementById('clear').onclick = () => decide('');
document.getElementById('note').oninput = event =>
  save(tiles[index].sha256 + ':note', event.target.value);
document.addEventListener('keydown', event => {
  if (['TEXTAREA', 'SELECT', 'INPUT'].includes(event.target.tagName)) return;
  if (event.key === 'ArrowLeft') move(index - 1);
  if (event.key === 'ArrowRight') move(index + 1);
});
draw();
</script></body></html>
"""


def build_review_html(summary_path):
    """Verify all tiles and return a self-contained viewer, not a verdict."""
    summary_path = Path(summary_path)
    count = verify_inspection_tiles(summary_path)
    summary = json.loads(summary_path.read_text(encoding='utf-8'))
    tiles = []
    for feature in summary['features']:
        kind = feature['kind']
        label = f"{FEATURE_NAMES.get(kind, kind)} {feature['id']}"
        for tile in feature['native_tiles']:
            width = tile['panel_width_px']
            height = tile['height_px']
            if not isinstance(width, int) or width <= 0 or \
                    not isinstance(height, int) or height <= 0:
                raise ValueError('Native tile has invalid panel dimensions')
            tiles.append({
                'feature': feature['id'], 'label': label,
                'file': tile['file'], 'sha256': tile['sha256'],
                'width': width, 'height': height,
            })
    if len(tiles) != count or not tiles:
        raise ValueError('The tile count changed during review preparation')
    payload = json.dumps(tiles, ensure_ascii=False).replace('<', '\\u003c')
    return PAGE_HTML.replace('__TILES__', payload)


def main():
    """Prepare the viewer next to its immutable summary and tiles."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('summary', type=Path)
    args = parser.parse_args()
    output = args.summary.parent / 'native_tile_review.html'
    output.write_text(build_review_html(args.summary), encoding='utf-8')
    print(output)


if __name__ == '__main__':
    main()
