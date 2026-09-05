"""Inspect recorded candidates near the diagnostic wall's board-joint crossings."""
import json
from pathlib import Path
import argparse
from PIL import Image, ImageDraw
from build_film import font


def main():
    p=argparse.ArgumentParser(); p.add_argument('--data-root',type=Path,required=True)
    args=p.parse_args(); data=args.data_root
    processed=data/'processed-p206-horizontal-20260831g5'
    run_id=json.loads((processed/'processing_manifest.json').read_text())['source_archive']['run_id']
    root=data/'inspection-diagnostic-full-horizontal-025mm-20260828'
    raw=next(p.parent for p in root.rglob('manifest.json') if json.loads(p.read_text()).get('run_id')==run_id)
    candidates=[]
    for path in (processed/'metadata').glob('*.json'):
        item=json.loads(path.read_text()); position=item['camera_pose']['pose']['position']
        x,y=position['x'],position['y']
        # Prefer a crossing off-centre: distortion is not visible on a central axis.
        dx=min(abs(x-3.4),abs(x-7.0)); dy=abs(y-4.24)
        candidates.append((abs(dx-.08)+abs(dy-.15),path.stem,x,y,item['image_file']))
    chosen=sorted(candidates)[:12]
    contact=Image.new('RGB',(1920,1560),'#111820'); d=ImageDraw.Draw(contact)
    for n,(_,stem,x,y,relative) in enumerate(chosen):
        with Image.open(raw/relative) as im:
            contact.paste(im.convert('RGB').resize((640,360)),((n%3)*640,(n//3)*390))
        d.text(((n%3)*640+8,(n//3)*390+360),f'{stem}   x={x:.3f} y={y:.3f}',font=font(21),fill='white')
    out=data/'video_introduction';contact.save(out/'board_candidates.jpg')
    print('raw run:',raw.name)
    print('candidates:',[x[1] for x in chosen])


if __name__=='__main__':main()
