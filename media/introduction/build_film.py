#!/usr/bin/env python3
"""V2: footage-led edit, Simplified Chinese typography, timed narration.

Uses locally cached speech from synthesize_voice.py. Rendering never contacts TTS.
"""
import argparse
import hashlib
import json
import math
import subprocess
from functools import lru_cache
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont, ImageOps
import numpy as np
import tifffile
import cv2
import yaml

from build_video import SCENES

W, H = 1920, 1080
FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
CYAN = '#62e2cd'


@lru_cache(maxsize=16)
def font(size):
    return ImageFont.truetype(FONT, size, index=2)


def run(command):
    subprocess.run(command, check=True)


def duration(path):
    return float(subprocess.check_output(['ffprobe', '-v', 'error', '-show_entries',
        'format=duration', '-of', 'csv=p=0', str(path)], text=True).strip())


def stamp(seconds):
    milliseconds = round(seconds * 1000)
    return f'{milliseconds//3600000:02d}:{milliseconds//60000%60:02d}:{milliseconds//1000%60:02d},{milliseconds%1000:03d}'


def fitted(path):
    with Image.open(path) as im:
        return ImageOps.fit(im.convert('RGB'), (W, H), Image.Resampling.LANCZOS)


def caption_image(text, note):
    image = Image.new('RGBA', (W, H))
    d = ImageDraw.Draw(image)
    # Gradients preserve the picture, instead of enclosing it in a slide frame.
    for y in range(820, H):
        d.line((0, y, W, y), fill=(0, 0, 0, int(200 * (y - 820) / 260)))
    d.text((65, 868), note, font=font(23), fill='#e1e9ef', stroke_width=1, stroke_fill='#151a20')
    lines = [text]
    if d.textlength(text,font=font(44))>1500:
        candidates=[]
        for cut in range(1,len(text)):
            if text[cut] in '，。？！；：、,.?!;:':continue
            left,right=text[:cut],text[cut:]
            if max(d.textlength(left,font=font(44)),d.textlength(right,font=font(44)))<=1670:
                score=abs(len(left)-len(right))+(0 if text[cut-1] in '，。；：' else 8)
                candidates.append((score,cut))
        if not candidates:raise ValueError('More than two caption lines')
        cut=min(candidates)[1]
        lines=[text[:cut],text[cut:]]
    for i, line in enumerate(lines):
        x = (W - d.textlength(line, font=font(44))) / 2
        d.text((x, 955 - (len(lines)-1)*27 + i*57), line, font=font(44),
               fill='white', stroke_width=2, stroke_fill='#141820')
    return image


def title_image(scene):
    im = Image.new('RGBA', (W, H))
    d = ImageDraw.Draw(im)
    for y in range(210):
        d.line((0, y, W, y), fill=(0, 0, 0, int(160 * (1-y/210))))
    title = SCENES[scene][1]
    d.rectangle((65, 67, 72, 119), fill=CYAN)
    d.text((96, 61), title, font=font(43), fill='white', stroke_width=1, stroke_fill='#151a20')
    return im


def accent_image(scene, phrase):
    im = Image.new('RGBA', (W, H)); d = ImageDraw.Draw(im)
    labels = {
        2: ('规划 → 执行 → 采集', '校正 → 匹配 → 墙面地图'),
        4: ('轮速  +  IMU  +  全站仪', '融合定位 → 控制与采集'),
        12: ('照片 + 曝光位姿', '原始归档保持不变'),
        13: ('原始照片 · 板缝弯曲', '只去畸变 · 尚未做平场'),
        14: ('去畸变后 · 未做平场', '相同几何 · 加入历史平场校正'),
        17: ('重叠匹配 → 全局优化', '真值只用于评价'),
        19: ('重叠区域，每个像素只取一个来源', '硬切保留细节，也暴露错位'),
        21: ('1.130 mm', 'P95 不是最大误差'),
        22: ('170 个原尺寸图块', '结论限定于已验证工况'),
        24: ('覆盖与来源选择一致', '灰度最大差异 1 DN'),
        25: ('真实吸附 · 真实传感器 · 硬件安全', '下一步：真实墙面'),
    }
    if scene in labels:
        label = labels[scene][phrase]
        size = 78 if scene == 21 and phrase == 0 else 42
        width = d.textlength(label, font=font(size))
        d.rounded_rectangle((65, 650, 115+width, 770), 14, fill=(12, 22, 30, 205))
        d.text((90, 663), label, font=font(size), fill=CYAN)
    if scene == 23:
        d.rounded_rectangle((160, 200, 1760, 730), 24, fill=(10, 18, 28, 225))
        values = [(91.86, 'CPU', '#a4b8c9'), (47.73, 'CUDA', CYAN)] if phrase == 0 else [
            (108.01, 'CPU', '#a4b8c9'), (64.02, 'CUDA', CYAN)]
        d.text((215, 236), '硬切融合' if phrase == 0 else '整个拼接命令', font=font(42), fill='white')
        for k, (value, name, color) in enumerate(values):
            y = 355 + k*150
            d.text((215, y), name, font=font(38), fill='white')
            right = 405 + value*9
            d.rounded_rectangle((405, y, right, y+65), 10, fill=color)
            d.text((right+25, y+4), f'{value:.2f} s', font=font(35), fill='white')
    if scene==3:
        d.rounded_rectangle((65,190,690,545),18,fill=(10,20,28,210))
        d.text((90,210),'方向关系示意',font=font(25),fill='white')
        # Horizontal progress is the result, not the body heading.
        d.line((120,390,565,390),fill=CYAN,width=5)
        d.polygon([(565,390),(546,380),(546,400)],fill=CYAN)
        d.text((315,404),'实际水平轨迹',font=font(28),fill=CYAN)
        d.line((120,390,495,285),fill='#ffbd78',width=5)
        d.polygon([(495,285),(472,280),(479,300)],fill='#ffbd78')
        d.text((185,267),'车头向上偏',font=font(28),fill='#ffbd78')
        d.line((575,275,575,375),fill='#ed8a85',width=4)
        d.polygon([(575,375),(566,357),(584,357)],fill='#ed8a85')
        d.text((599,309),'下滑',font=font(25),fill='#ed8a85')
        d.text((90,483),'上翘 ≠ 向上爬；用来抵消下滑',font=font(28),fill='white')
    return im


def tiff_crop(path, box):
    """Read only intersecting tiles of the project's compressed mono8 masters."""
    x0,y0,x1,y1=box
    with tifffile.TiffFile(path) as document:
        page=document.pages[0]
        if not page.is_tiled or len(page.shape)!=2 or page.dtype!=np.uint8:
            raise ValueError('Expected a tiled mono8 mosaic')
        h,w=page.shape; tw,th=page.tilewidth,page.tilelength
        if not (0<=x0<x1<=w and 0<=y0<y1<=h):raise ValueError('Crop outside mosaic')
        result=np.zeros((y1-y0,x1-x0),np.uint8)
        columns=math.ceil(w/tw)
        for ty in range(y0//th,(y1-1)//th+1):
            for tx in range(x0//tw,(x1-1)//tw+1):
                index=ty*columns+tx
                document.filehandle.seek(page.dataoffsets[index])
                values,_,_=page.decode(document.filehandle.read(page.databytecounts[index]),index)
                left,top=max(x0,tx*tw),max(y0,ty*th)
                right,bottom=min(x1,(tx+1)*tw),min(y1,(ty+1)*th)
                result[top-y0:bottom-y0,left-x0:right-x0]=values[0,top-ty*th:bottom-ty*th,left-tx*tw:right-tx*tw,0]
    return Image.fromarray(result)


def make_assets(data, out):
    assets = out/'assets'; assets.mkdir(parents=True, exist_ok=True)
    mosaic = data/'mosaic-p206-joint-20260831g5-hardcut'
    # High-resolution crops decode only intersecting tiles, not the full 1 GB wall.
    for variant in ['optimized', 'pose_only']:
        path=mosaic/f'mosaic_{variant}.tif'
        with tifffile.TiffFile(path) as document:h,w=document.pages[0].shape
        for k, (cx, cy) in enumerate([(0.50,0.47),(0.64,0.31),(0.28,0.67),(0.73,0.71)]):
            x, y = int(w*cx)-3000, int(h*cy)-1688
            crop = tiff_crop(path,(x,y,x+6000,y+3376)).convert('RGB')
            crop.resize((W,H), Image.Resampling.LANCZOS).save(assets/f'{variant}_{k}.jpg', quality=95)
    with Image.open(mosaic/'mosaic_preview.jpg') as im:
        # Full wall is retained inside a blurred version, avoiding a small inset.
        from PIL import ImageFilter
        back = ImageOps.fit(im.convert('RGB'), (W,H)).filter(ImageFilter.GaussianBlur(45))
        front = ImageOps.contain(im.convert('RGB'), (W,H))
        back.paste(front, ((W-front.width)//2,(H-front.height)//2))
        back.save(assets/'whole.jpg', quality=95)
    # One recorded crossing, no arbitrary crop or independent contrast adjustment.
    proc=data/'processed-p206-horizontal-20260831g5'
    manifest=json.loads((proc/'processing_manifest.json').read_text())
    run_id=manifest['source_archive']['run_id']
    raw_root=data/'inspection-diagnostic-full-horizontal-025mm-20260828'
    raw=next(p.parent for p in raw_root.rglob('manifest.json') if json.loads(p.read_text()).get('run_id')==run_id)
    label=json.loads((proc/'metadata/000369.json').read_text())
    raw_path=raw/label['image_file']; processed_path=proc/label['processed_image_file']
    if hashlib.sha256(raw_path.read_bytes()).hexdigest()!=label['image_sha256']:
        raise ValueError('Selected original frame hash mismatch')
    if hashlib.sha256(processed_path.read_bytes()).hexdigest()!=label['processed_image_sha256']:
        raise ValueError('Selected processed frame hash mismatch')
    info=yaml.safe_load((raw/'calibration/camera_info.yaml').read_text())
    rectified=yaml.safe_load((proc/'calibration/rectified_camera_info.yaml').read_text())
    matrix=np.array(info['k']).reshape(3,3)
    new_matrix=np.array(rectified['k']).reshape(3,3)
    original=cv2.imread(str(raw_path),cv2.IMREAD_GRAYSCALE)
    maps=cv2.initUndistortRectifyMap(matrix,np.array(info['d']),None,new_matrix,(W,H),cv2.CV_32FC1)
    geometry_only=cv2.remap(original,*maps,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
    processed=cv2.imread(str(processed_path),cv2.IMREAD_GRAYSCALE)
    for name,pixels in [('raw',original),('undistorted',geometry_only),('processed',processed)]:
        if pixels.shape!=(H,W):raise ValueError('Unexpected demonstration dimensions')
        Image.fromarray(pixels).save(assets/f'{name}.png')
    (out/'board_frame.json').write_text(json.dumps(dict(
        run_id=run_id,frame='000369',raw_sha256=label['image_sha256'],
        processed_sha256=label['processed_image_sha256'],
        geometry_only='original + archived K/D + processed rectified K; no flat field',
        processed='existing historical processed image; no new calibration applied'),indent=2))
    with Image.open(data/'video_introduction/raw/rviz_paused.png') as im:
        from PIL import ImageFilter
        back=ImageOps.fit(im.convert('RGB'),(W,H)).filter(ImageFilter.GaussianBlur(35))
        front=ImageOps.contain(im.convert('RGB'),(W,H))
        back.paste(front,((W-front.width)//2,0))
        back.save(assets/'paused.jpg',quality=95)
    a = fitted(assets/'pose_only_0.jpg'); b = fitted(assets/'optimized_0.jpg')
    pair = Image.new('RGB',(W,H))
    for x, source in [(0,a),(960,b)]:
        # Identical central crop on both sides, no independent contrast changes.
        pair.paste(source.crop((480,0,1440,1080)),(x,0))
    d = ImageDraw.Draw(pair)
    for x,label in [(60,'仅按位姿'),(1020,'全局优化')]:
        d.rounded_rectangle((x,165,x+245,230),10,fill='#13222d')
        d.text((x+20,171),label,font=font(35),fill=CYAN)
    d.line((960,0,960,H),fill=CYAN,width=3)
    pair.save(assets/'pair.jpg',quality=95)
    return assets


def footage(scene, phrase):
    if scene==10 and phrase==0:
        return None
    choices = {
      0:('gz',8,35), 2:('gz',23,43), 3:('gz',35,45), 4:('gz',5,40),
      5:('rviz',0,8), 6:('rviz',12,23), 7:('rviz',32,48), 8:('rviz',18,30),
      9:('rviz',46,59), 10:('rviz',83,114), 11:('rviz',135,150),
      25:('gz',40,50),
    }
    if scene in choices:
        name, first, second = choices[scene]
        return name, first if phrase == 0 else second
    return None


def still_frame(image, frame_index, scene_frames):
    """Continuous centred transform, no integer crop or per-phrase restart."""
    h,w=image.shape[:2]
    scale=1.015+0.045*frame_index/max(scene_frames-1,1)
    matrix=np.array([[scale,0,(1-scale)*(w-1)/2],
                     [0,scale,(1-scale)*(h-1)/2]],np.float64)
    return cv2.warpAffine(image,matrix,(w,h),flags=cv2.INTER_CUBIC,
                          borderMode=cv2.BORDER_REFLECT_101)


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--data-root',type=Path,required=True)
    parser.add_argument('--shots',type=int,nargs='*')
    parser.add_argument('--assemble-only',action='store_true',help='Validate existing shot lengths and assemble')
    args=parser.parse_args(); data=args.data_root
    root=data/'video_introduction'; out=root/'v3'; out.mkdir(exist_ok=True)
    clips=out/'clips'; clips.mkdir(exist_ok=True)
    assets=make_assets(data,out)
    voice=json.loads((root/'voice/manifest.json').read_text())
    lengths=[math.ceil((duration(root/'voice'/item['file'])+.35)*30)/30 for item in voice]
    cv2.setNumThreads(2)
    timeline=[]; cursor=0.; subtitles=[]
    for k,item in enumerate(voice):
        i,j=item['scene'],item['phrase']; scene=SCENES[i]
        audio=root/'voice'/item['file']
        speech_duration=duration(audio)
        length=math.ceil((speech_duration+.35)*30)/30
        still={1:'whole',10:'paused',12:'raw',13:'raw' if j==0 else 'undistorted',14:'undistorted' if j==0 else 'processed',
               15:'processed',16:'pose_only_0',17:f'optimized_{j}',18:'pair',19:f'optimized_{j+1}',
               20:'whole' if j==0 else 'optimized_2',21:'optimized_0',22:f'optimized_{j+2}',
               23:'whole',24:'pair',26:'optimized_0' if j==0 else 'whole'}.get(i,'whole')
        note=scene[3]
        if i==10:note='本次暂停状态截帧' if j==0 else 'RViz 实录 · 恢复后的任务执行'
        if i in [2,4,7,17,19,22,25]:
            note={2:'Gazebo 仿真实录',4:'Gazebo 实录 / 定位原理说明',7:'RViz 轨迹实录',
                  17:'真实拼接结果 / 对齐流程说明',19:'第 5 组 optimized 局部',
                  22:'第 5 组原尺寸产物裁切',25:'Gazebo 仿真实录 / 后续方向'}[i]
        if i in (13,14,15):note='第 5 组 / 第 369 帧 · 固定画幅 · 历史标定，不作显示增强'
        cap=assets/f'caption_{k:02d}.png'; caption_image(item['text'],note).save(cap)
        title=assets/f'title_{k:02d}.png'; title_image(i).save(title)
        accent=assets/f'accent_{k:02d}.png'; accent_image(i,j).save(accent)
        timeline.append(dict(shot=k,scene=i,phrase=j,start_s=cursor,duration_s=length,
             speech_duration_s=speech_duration,text=item['text'],voice_file=item['file'],
             picture=footage(i,j) or still))
        subtitles.append(f'{k+1}\n{stamp(cursor)} --> {stamp(cursor+speech_duration)}\n{item["text"]}\n')
        cursor+=length
        if args.assemble_only:continue
        if args.shots is not None and k not in args.shots:continue
        target=clips/f'{k:02d}.mp4'; wav=clips/f'{k:02d}.wav'
        # Cache depends on renderer, voice content and the selected source pixels.
        still_path=assets/f'{still}.png' if still in ('raw','undistorted','processed') else assets/f'{still}.jpg'
        key=hashlib.sha256(Path(__file__).read_bytes()+cap.read_bytes()+title.read_bytes()+
                           accent.read_bytes()+audio.read_bytes()+
                           (still_path.read_bytes() if not footage(i,j) else b'')).hexdigest()
        keypath=clips/f'{k:02d}.key'
        if target.exists() and wav.exists() and keypath.exists() and keypath.read_text()==key:continue
        cmd=['ffmpeg','-hide_banner','-loglevel','warning','-y','-threads','2','-filter_complex_threads','2']
        source=footage(i,j)
        if source:
            name,start=source
            video=root/'raw'/('gazebo_gpu_follow.mp4' if name=='gz' else 'rviz_demo.mp4')
            # Hold only the tail if narration runs beyond the available take.
            cmd+=['-ss',str(start),'-i',str(video)]
            if name=='gz':
                f='[0:v]scale=1920:1080:force_original_aspect_ratio=increase,crop=1920:1080,setsar=1,tpad=stop_mode=clone:stop_duration=20[b];'
            elif j==1 and i!=10:
                f='[0:v]crop=900:506:364:390,scale=1920:1080,setsar=1,tpad=stop_mode=clone:stop_duration=20[b];'
            else:
                f='[0:v]split[u][v];[u]scale=1920:1080:force_original_aspect_ratio=increase,crop=1920:1080,gblur=sigma=35,eq=brightness=-0.12[back];[v]scale=-2:1080[front];[back][front]overlay=(W-w)/2:0,setsar=1,tpad=stop_mode=clone:stop_duration=20[b];'
        else:
            cmd+=['-f','rawvideo','-pix_fmt','rgb24','-video_size','1920x1080','-framerate','30','-i','pipe:0']
            f='[0:v]setsar=1[b];'
        for path in [cap,title,accent]:cmd+=['-loop','1','-framerate','30','-i',str(path)]
        f+='[b][3:v]overlay[withaccent];[withaccent][1:v]overlay[withcaption];'
        f+="[withcaption][2:v]overlay=enable='lt(t,3.0)'" if j==0 else '[withcaption][2:v]overlay=enable=0'
        f+=',format=yuv420p[out]'
        cmd+=['-filter_complex',f,'-map','[out]','-t',str(length),'-r','30','-an',
              '-c:v','h264_nvenc','-preset','p4','-cq','20','-b:v','0',str(target)]
        if source:
            run(cmd)
        else:
            with Image.open(still_path) as picture:image=np.array(picture.convert('RGB'))
            fixed=still in ('raw','undistorted','processed','pair','paused')
            scene_frames=round(sum(lengths[i*2:i*2+2])*30)
            offset=round(lengths[i*2]*30) if j else 0
            with subprocess.Popen(cmd,stdin=subprocess.PIPE) as encoder:
                for frame_index in range(round(length*30)):
                    pixels=image if fixed else still_frame(image,offset+frame_index,scene_frames)
                    encoder.stdin.write(pixels.tobytes())
                encoder.stdin.close()
                if encoder.wait()!=0:raise RuntimeError(f'Frame encoder failed on shot {k}')
        run(['ffmpeg','-hide_banner','-loglevel','warning','-y','-i',str(audio),
             '-af','apad','-t',str(length),'-ar','48000','-ac','1','-c:a','pcm_s16le',str(wav)])
        keypath.write_text(key)
        print(f'shot {k+1}/54: {length:.2f}s',flush=True)
    (out/'timeline.json').write_text(json.dumps(timeline,ensure_ascii=False,indent=2))
    (out/'climbot_intro_zh_v3.srt').write_text('\n'.join(subtitles))
    if args.shots is not None:return
    for k,record in enumerate(timeline):
        if abs(duration(clips/f'{k:02d}.mp4')-record['duration_s'])>1/30+.001:
            raise ValueError(f'Shot {k} duration does not match updated narration; rerender it')
        if abs(duration(clips/f'{k:02d}.wav')-record['duration_s'])>.002:
            raise ValueError(f'Shot {k} audio does not match updated narration; rerender it')
    for suffix in ['mp4','wav']:
        (out/f'{suffix}_concat.txt').write_text(''.join(f"file 'clips/{k:02d}.{suffix}'\n" for k in range(54)))
    run(['ffmpeg','-hide_banner','-loglevel','warning','-y','-f','concat','-safe','1',
         '-i',str(out/'mp4_concat.txt'),'-c','copy',str(out/'picture.mp4')])
    run(['ffmpeg','-hide_banner','-loglevel','warning','-y','-f','concat','-safe','1',
         '-i',str(out/'wav_concat.txt'),'-af','loudnorm=I=-16:TP=-1.5:LRA=11',
         '-ar','48000','-c:a','aac','-b:a','192k',str(out/'narration.m4a')])
    final=root/'climbot_intro_zh_voiced_v3.mp4'
    run(['ffmpeg','-hide_banner','-loglevel','warning','-y','-i',str(out/'picture.mp4'),
         '-i',str(out/'narration.m4a'),'-map','0:v:0','-map','1:a:0','-c','copy',
         '-movflags','+faststart',str(final)])
    print(f'DONE {cursor:.3f}s {final.name}',flush=True)


if __name__=='__main__':main()
