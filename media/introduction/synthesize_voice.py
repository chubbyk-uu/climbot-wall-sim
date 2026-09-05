#!/usr/bin/env python3
"""Synthesize only the approved narration text; run inside the media venv."""
import argparse
import asyncio
import hashlib
import json
from pathlib import Path

import edge_tts
from build_video import SCENES

VOICE = 'zh-CN-YunxiNeural'
RATE = '+0%'


def spoken(text):
    for source, replacement in [('Climbot Sim', '这个项目'), ('pose-only', '仅按位姿拼接'),
                                ('Pause', '暂停'), ('Resume', '继续'), ('Stop', '停止'),
                                ('CUDA', '库达'), ('CPU', 'C P U'), ('GPU', 'G P U')]:
        text = text.replace(source, replacement)
    return text


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    manifest = []
    for i, scene in enumerate(SCENES):
        for j, caption in enumerate(scene[4:]):
            words = spoken(caption)
            digest = hashlib.sha256((VOICE + RATE + words).encode()).hexdigest()
            name = f'{i:02d}_{j}_{digest[:12]}.mp3'
            target = args.output / name
            if not target.exists():
                temporary = target.with_suffix('.part.mp3')
                for attempt in range(5):
                    try:
                        await edge_tts.Communicate(words, VOICE, rate=RATE).save(str(temporary))
                        break
                    except (edge_tts.exceptions.NoAudioReceived, asyncio.TimeoutError):
                        if attempt == 4:
                            raise
                        await asyncio.sleep(2 * (attempt + 1))
                temporary.replace(target)
                await asyncio.sleep(0.5)
            manifest.append(dict(scene=i, phrase=j, text=caption, spoken=words,
                                 voice=VOICE, rate=RATE, file=name, request_sha256=digest))
            print(f'voice {i * 2 + j + 1}/54', flush=True)
    (args.output / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    asyncio.run(main())
