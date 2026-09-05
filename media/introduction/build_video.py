#!/usr/bin/env python3
"""Render the Chinese, subtitle-led introduction from project and captured assets.

Usage: python3 media/introduction/build_video.py --data-root "$CLIMBOT_DATA_ROOT"
Generated media stays outside the repository. No speech service is contacted.
"""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont, ImageOps

ROOT = Path(__file__).resolve().parents[2]
FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
BG = '#101822'
FG = '#eef4fb'
MUTED = '#a5b8cc'
CYAN = '#50daca'
ORANGE = '#ffbb72'
W, H = 1920, 1080
DURATION = 12

# chapter, title, asset, source note, narration in two six-second phrases
SCENES = [
 ('开场', '让机器人，把墙面变成一张地图', 'gz:8', 'Gazebo / D3D12 · 跟随镜头实录 · 原速',
  '一台机器人沿着墙面移动，能不能把沿途拍到的照片，拼成一张可以查看细节的地图？',
  'Climbot Sim 把路线规划、运动控制、自动采集和墙面拼接，连接成了一条完整流程。'),
 ('开场', '先看最后得到的东西', 'mosaic', '第 5 组独立检验 · optimized 拼接图',
  '这张墙面图来自两次独立方向的扫描：横向六百八十张，纵向六百六十张。',
  '合计一千三百四十张照片。我们既要拼得完整，也要让墙上的细节对得齐。'),
 ('开场', '这期视频，走完一趟巡检', 'pipeline', '流程示意 · 非运行监控',
  '接下来，先看机器人怎样沿墙扫描，再看照片怎样变成墙面地图。',
  '所有机器人运行画面均来自仿真；真实硬件的吸附、安全和定位，还需要单独验证。'),
 ('01 / 机器人', '在重力作用下贴墙运动', 'gz:30', 'Gazebo / D3D12 · 跟随镜头实录 · 原速',
  '仿真用简化的法向力保持吸附，重力仍然存在。横向行走时，重力在车身横向上的分量，会让机器人有向下侧滑的趋势。',
  '所以车头需要略微上翘，也就是在墙面内朝上偏。用向上的运动分量抵消下滑，车身虽然斜着，实际轨迹仍保持水平。'),
 ('01 / 机器人', '知道自己在哪里，才能走得准', 'localization', '定位关系示意 · 非传感器实测曲线',
  '轮速、惯性测量和全站仪观测共同参与融合定位，为控制器提供运动状态。',
  '控制器根据估计位置修正轨迹。拼接也使用记录的相机位姿，而不是直接拿仿真真值。'),
 ('02 / 规划', '先定义要检查的区域', 'rviz:0', 'RViz 实录 · 本次从任务配置载入区域',
  '任务从一个工作区域开始。系统支持矩形和梯形，并可选择横向或纵向扫描。',
  '交互模式可以点选区域；这次演示提前载入了一块矩形，便于稳定展示后续流程。'),
 ('02 / 规划', '三种颜色，三种含义', 'rviz:5', '绿色：安全框 / 橙色：任务区 / 黄色：预测足迹',
  '绿色框表示运动安全边界，橙色轮廓是选定的任务区域，蓝线是机器人中心路线。',
  '黄色带表示相机预计拍到的范围。机器人能走的区域，与相机拍到的区域并不相同。'),
 ('02 / 规划', '弓字路径，把区域一行行扫过', 'route', '弓字覆盖示意 · 非本次精确规划坐标',
  '相邻扫描带保留重叠，一条直线结束后，机器人转向、换行，再扫描下一条。',
  '相机装在机器人前方，所以规划还要考虑相机偏移、画幅大小和边界处的运动裕量。'),
 ('03 / 执行', '规划完成，开始执行', 'rviz:16', 'RViz 仿真实录 · 原速',
  '任务启动后，管理器协调轨迹执行与采集归档，界面同时显示当前路段和完成进度。',
  '规划决定往哪里走；闭环控制不断根据当前位置，修正机器人实际怎样走。'),
 ('03 / 执行', '扫描与转场，各有职责', 'rviz:34', 'RViz 仿真实录 · 原速',
  '直线扫描段承担正式采集；转向和换行主要负责把机器人送到下一条扫描线。',
  '把这些状态分开，才能清楚地决定什么时候允许拍照，什么时候应该等待。'),
 ('03 / 执行', '暂停之后，继续同一趟任务', 'paused', '本次实录截帧 · Paused 状态',
  'Pause 会暂停当前任务，保留任务信息和进度。Resume 从暂停处继续执行。',
  'Stop 则结束当前执行。因此临时停下来观察时，不需要从头重新扫描。'),
 ('04 / 采集', '按位置拍照，而不是盲目连拍', 'rviz:135', 'RViz 仿真实录 · 原速',
  '正式扫描中，系统按沿轨道的位置触发拍照，以控制相邻照片之间的距离。',
  '这样，照片分布主要跟随机器人走过的位置，而不是简单依赖固定时间间隔。'),
 ('04 / 采集', '每张照片，都带着拍摄位置', 'archive', '归档内容示意 · 不展示私人保存路径',
  '一张图片还不够。系统同时保存曝光时刻的相机位姿、相机标定和任务信息。',
  '原始图片保持不变。后续处理使用独立目录，方便追溯，也方便换算法重新计算。'),
 ('05 / 预处理', '先把照片整理到同一标准', 'raw', '第 5 组原始采集帧 · 未做显示增强',
  '先看这张带横竖板缝的原始照片。靠近画面边缘的板缝出现弯曲，亮度也不均匀。',
  '现在只做去畸变。对照刚才的板缝，它更接近直线；这一张还没有做平场补偿。'),
 ('05 / 预处理', '平场校正，修正亮度不均', 'flatfield', '处理原理示意 · 不是标定前后实测图',
  '接着看平场。保持去畸变后的坐标不变，先看没有平场补偿时，中心和四角的亮度差异。',
  '再加入这组数据当时使用的平场校正。几何位置保持不变，主要改变的是画面各处的亮度。'),
 ('05 / 预处理', '校正结果，单独保存', 'processed', '第 5 组历史处理帧 · 使用该组原始标定',
  '这里展示的是同一帧的历史处理结果。它保留了当时使用的标定和处理记录。',
  '新数据使用更新后的平场标定；旧验收证据不会被悄悄替换成另一套处理结果。'),
 ('06 / 拼接', '第一步：按位姿摆放照片', 'pose', '第 5 组 · pose-only 全局拼接',
  '知道每张照片拍摄时相机的位置，就可以先把它们投影到统一墙面坐标中。',
  '这就是 pose-only。它提供初始布局，但定位误差仍可能在重叠处表现为错位。'),
 ('06 / 拼接', '第二步：让重叠内容相互对齐', 'matching', '算法流程示意 · 非本次匹配点可视化',
  '算法在有重叠的照片之间寻找对应内容，再把多张照片的约束放到一起优化。',
  '优化得到更一致的全局布局。仿真真值只在最后评价时使用，不参与拼接求解。'),
 ('06 / 拼接', '同一处墙面，放大对比', 'detail', '第 5 组 · 同坐标裁切 / 左 pose-only，右 optimized',
  '左边仅依赖初始位姿，右边加入图像匹配和全局优化。两侧显示同一个墙面位置。',
  '对比时要看结构是否连续、边缘是否错开，而不只是远看整张图是否好看。'),
 ('06 / 拼接', '重叠区域，采用硬切', 'hardcut', '硬切原理示意 · 色块代表不同来源照片',
  '最终重叠区域只选择其中一张照片的像素。当前实现使用硬切，不把两张直接混合。',
  '这样可以避免混合掩盖错位，但也更依赖前面的几何对齐和亮度校正。'),
 ('06 / 拼接', '从局部照片，回到整面墙', 'mosaic', '第 5 组独立检验 · optimized',
  '拼接完成后，可以从整体地图定位感兴趣的位置，再回到原尺寸检查细节。',
  '当前验收要求冻结巡检域内的特征没有漏拍，不等于墙面所有像素都已经覆盖。'),
 ('07 / 验证', '怎么判断，真的拼对了？', 'metrics', '来源：docs/STATUS.md · 第 5 组独立检验',
  '在这一组独立检验中，绝对锚点偏差的百分之九十五分位数约为一点一三毫米。',
  '也就是说，约百分之九十五的被测锚点偏差不超过这个数；它不是最大误差。'),
 ('07 / 验证', '有数据，也要看原尺寸', 'review', '原尺寸复核流程示意 · 170 个 tile 已人工复核',
  '除了数值门限，项目还对一百七十个原尺寸图块进行了人工复核，检查重影和拖影。',
  '这些结论来自特定诊断墙和仿真工况，不能直接外推到任意墙面或真实机器人。'),
 ('08 / 加速', '把最重的融合计算交给 GPU', 'speed', '同一 1,340 帧输入 · 三轮交替 A/B 中位数',
  'CUDA 加速针对硬切融合阶段：中位耗时从九十一点八六秒，降到四十七点七三秒。',
  '这一步约快一点九二倍；整个拼接命令从一百零八秒降到六十四秒，约快一点六九倍。'),
 ('08 / 加速', '加速之外，也要守住结果', 'quality', 'CUDA 开发验收 · 同输入 CPU / GPU 对比',
  'CPU 和 GPU 的覆盖、来源选择以及接缝位置一致，灰度最大差异为一个灰度级。',
  '所以这里比较的不只是秒表，还包括结果质量。CPU 后端也继续保留，方便回退和对照。'),
 ('09 / 下一步', '仿真闭环之后，走向真实墙面', 'future', '当前边界与后续方向',
  '下一步的重点，是把真实吸附、摩擦、传感器误差和硬件安全约束带进验证。',
  '仿真让我们先把流程跑通；真实世界仍需要新的数据、新的门限和独立测试。'),
 ('结尾', '从一次移动，到一张可检查的地图', 'mosaic', 'Climbot Sim · ROS 2 / Gazebo · 仿真项目介绍',
  '从规划、控制、采集，到预处理和拼接，这就是 Climbot Sim 当前完成的墙面巡检流程。',
  '感谢观看，我们下次再见。'),
]


def font(size):
    # TTC index 0 is Japanese; index 2 selects Simplified Chinese glyph forms.
    return ImageFont.truetype(FONT, size, index=2)


def text(draw, xy, value, size=36, fill=FG):
    draw.text(xy, value, font=font(size), fill=fill)


def fit(im, size):
    return ImageOps.contain(im.convert('RGB'), size, Image.Resampling.LANCZOS)


def diagram(kind):
    im = Image.new('RGB', (1792, 720), '#172330')
    d = ImageDraw.Draw(im)
    if kind == 'speed':
        text(d, (90, 55), '硬切融合耗时', 46)
        for y, label, value, color in [(210, 'CPU', 91.86, MUTED), (365, 'CUDA', 47.73, CYAN)]:
            text(d, (90, y), label, 40)
            d.rounded_rectangle((290, y, 290 + value * 12, y + 76), 12, fill=color)
            text(d, (310 + value * 12, y + 10), f'{value:.2f} s', 36)
        text(d, (90, 570), '融合 1.92×   /   命令全程 1.69×', 44, CYAN)
    elif kind == 'metrics':
        for x, big, title, note in [(80, '1.130 mm', '绝对锚点偏差 P95', '冻结门限 ≤ 2.5 mm'),
                                    (670, '0.143 mm', '局部残差 P95', '冻结门限 ≤ 0.5 mm'),
                                    (1260, '0', '巡检域内 feature 漏拍', '限定本组诊断墙')]:
            text(d, (x, 170), big, 65, CYAN)
            text(d, (x, 305), title, 32)
            text(d, (x, 385), note, 27, MUTED)
        text(d, (80, 590), '独立检验数据，不是实机性能承诺', 36, ORANGE)
    elif kind == 'route':
        d.rectangle((200, 85, 1580, 600), outline=CYAN, width=4)
        points=[]
        for i in range(6):
            y = 140 + i * 78
            a,b = (280,1500) if i % 2 == 0 else (1500,280)
            d.line((a,y,b,y), fill='#3d5962', width=52)
            points.extend([(a,y),(b,y)])
        d.line(points, fill=ORANGE, width=6)
        for x,y in points[::2]:
            d.ellipse((x-9,y-9,x+9,y+9), fill=FG)
        text(d, (610, 640), '扫描 → 换行 → 反向扫描', 30)
    elif kind == 'flatfield':
        text(d, (90, 65), '为什么边缘会偏暗？', 48)
        for cx,label,color in [(350,'画面响应不均',MUTED),(900,'平场补偿',ORANGE),(1450,'亮度更一致',CYAN)]:
            for r in range(175,0,-1):
                v = int(70+100*(1-r/175)) if cx==350 else 140
                d.ellipse((cx-r,310-r,cx+r,310+r), fill=(v,v,v))
            text(d,(cx-130,535),label,34,color)
        text(d,(565,280),'→',65,CYAN); text(d,(1120,280),'→',65,CYAN)
    elif kind == 'hardcut':
        for x,label in [(180,'照片 A'),(990,'照片 B')]:
            d.rounded_rectangle((x,150,x+600,500),20,fill='#334e60' if x==180 else '#45504c')
            text(d,(x+200,280),label,55)
        d.line((885,120,885,560),fill=ORANGE,width=6)
        text(d,(550,595),'每个输出像素，只选一个来源',42,CYAN)
    else:
        blocks={
          'pipeline':[('规划','定义区域与路线'),('执行','控制运动与拍照'),('处理','校正与全局拼接'),('验证','真值与视觉复核')],
          'localization':[('轮速','运动增量'),('IMU','惯性测量'),('全站仪','外部观测'),('融合定位','控制与采集使用')],
          'archive':[('原始图像','保持不可变'),('曝光位姿','与照片关联'),('相机标定','几何与光度'),('任务快照','支持追溯')],
          'matching':[('初始投影','提供布局'),('重叠匹配','寻找对应内容'),('全局优化','协调多图约束'),('墙面输出','统一坐标')],
          'review':[('170 个图块','原尺寸导出'),('人工检查','重影与拖影'),('数值门禁','几何与接缝'),('证据链','版本与哈希')],
          'quality':[('覆盖','逐位一致'),('像素来源','逐位一致'),('灰度','最大差 1 DN'),('CPU 后端','保留对照能力')],
          'future':[('真实吸附','压力与失效'),('真实定位','噪声与遮挡'),('硬件安全','急停与驱动'),('新墙面','独立验证')],
        }[kind]
        for i,(title,note) in enumerate(blocks):
            x=55+i*435
            d.rounded_rectangle((x,200,x+385,510),22,fill='#213444',outline='#3a5363',width=2)
            text(d,(x+28,235),f'0{i+1}',34,CYAN)
            text(d,(x+28,310),title,44)
            text(d,(x+28,407),note,29,MUTED)
            if i<3: text(d,(x+393,335),'›',36,CYAN)
    return im


def wrap(draw, value, max_width=1740):
    lines=['']
    for c in value:
        if draw.textlength(lines[-1]+c,font=font(38)) > max_width:
            lines.append('')
        lines[-1]+=c
    if len(lines)>2:
        raise ValueError('Subtitle exceeds two lines: '+value)
    return lines


def overlay(index, scene, caption):
    im=Image.new('RGBA',(W,H),(0,0,0,0)); d=ImageDraw.Draw(im)
    d.rectangle((0,0,W,139),fill=BG)
    d.rectangle((0,860,W,H),fill=BG)
    text(d,(64,21),'CLIMBOT SIM   /   '+scene[0],23,CYAN)
    text(d,(64,60),scene[1],48)
    text(d,(1670,34),f'{index+1:02d} / {len(SCENES):02d}',25,MUTED)
    text(d,(64,870),scene[3],24,MUTED)
    for j,line in enumerate(wrap(d,caption)):
        x=(W-d.textlength(line,font=font(38)))/2
        text(d,(x,925+j*53),line,38)
    d.rectangle((64,1055,1856,1058),fill='#334352')
    d.rectangle((64,1055,64+int(1792*(index+1)/len(SCENES)),1058),fill=CYAN)
    return im


def timestamp(seconds):
    return f'{seconds//3600:02d}:{seconds//60%60:02d}:{seconds%60:02d},000'


def main():
    p=argparse.ArgumentParser(); p.add_argument('--data-root',type=Path,required=True)
    p.add_argument('--prepare-only',action='store_true')
    p.add_argument('--rerender',type=int,nargs='*',default=[],help='Explicit scene indexes to replace')
    args=p.parse_args()
    data=args.data_root; out=data/'video_introduction'; assets=out/'assets'; clips=out/'clips'
    assets.mkdir(parents=True,exist_ok=True); clips.mkdir(exist_ok=True)
    mosaic=data/'mosaic-p206-joint-20260831g5-hardcut'
    images={}
    with Image.open(mosaic/'mosaic_preview.jpg') as im: images['mosaic']=im.copy()
    # The comparison preview has three equally sized panels: pose, optimized, difference.
    with Image.open(mosaic/'mosaic_comparison.jpg') as im:
        panel=im.width//3
        images['pose']=im.crop((0,0,panel,im.height))
        images['detail']=Image.new('RGB',(1792,720),BG)
        for i in range(2):
            box=(i*panel+int(panel*.36),int(im.height*.34),i*panel+int(panel*.64),int(im.height*.63))
            crop=fit(im.crop(box),(870,650))
            images['detail'].paste(crop,(i*896+(896-crop.width)//2,65))
        dd=ImageDraw.Draw(images['detail'])
        text(dd,(50,5),'POSE ONLY',28,ORANGE); text(dd,(946,5),'OPTIMIZED',28,CYAN)
    # Resolve the historical raw run by its recorded identity, not directory ordering.
    proc=data/'processed-p206-horizontal-20260831g5'
    manifest=json.loads((proc/'processing_manifest.json').read_text())
    run_id=manifest['source_archive']['run_id']
    candidates=list((data/'inspection-diagnostic-full-horizontal-025mm-20260828').rglob('manifest.json'))
    raw=next(q.parent for q in candidates if json.loads(q.read_text()).get('run_id')==run_id)
    for key,path in [('raw',raw/'images/raw/000020.png'),('processed',proc/'images/000020.png'),
                     ('paused',out/'raw/rviz_paused.png')]:
        with Image.open(path) as im: images[key]=im.copy()
    srt=[]; records=[]
    for index,scene in enumerate(SCENES):
        kind=scene[2]; base=Image.new('RGB',(W,H),BG)
        if ':' not in kind:
            visual=images[kind] if kind in images else diagram(kind)
            visual=fit(visual,(1792,720)); base.paste(visual,((W-visual.width)//2,140+(720-visual.height)//2))
        base.save(assets/f'{index:02d}_base.png')
        for j,caption in enumerate(scene[4:]):
            overlay(index,scene,caption).save(assets/f'{index:02d}_{j}.png')
            start=index*DURATION+j*6
            srt.append(f'{len(srt)+1}\n{timestamp(start)} --> {timestamp(start+6)}\n{caption}\n')
        records.append({'index':index,'title':scene[1],'source':kind,'note':scene[3],
                        'start_s':index*DURATION,'duration_s':DURATION,'captions':scene[4:]})
    (out/'climbot_intro_zh.srt').write_text('\n'.join(srt),encoding='utf-8')
    (out/'edit_manifest.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
    if args.prepare_only: return
    for index,scene in enumerate(SCENES):
        target=clips/f'{index:02d}.mp4'
        if target.exists() and index not in args.rerender: continue
        cmd=['ffmpeg','-hide_banner','-loglevel','warning','-y','-threads','2','-filter_complex_threads','2']
        kind=scene[2]
        if ':' in kind:
            name,start=kind.split(':'); filename='gazebo_gpu_follow.mp4' if name=='gz' else 'rviz_demo.mp4'
            cmd+=['-ss',start,'-i',str(out/'raw'/filename)]
            # RViz client capture excludes the title bar; Plan tab contains no paths.
            filt='[0:v]scale=1792:720:force_original_aspect_ratio=decrease,pad=1920:1080:(ow-iw)/2:140+(720-ih)/2:color=0x101822,setsar=1[b];'
        else:
            cmd+=['-loop','1','-framerate','30','-i',str(assets/f'{index:02d}_base.png')]
            filt='[0:v]setsar=1[b];'
        for j in range(2):cmd+=['-loop','1','-framerate','30','-i',str(assets/f'{index:02d}_{j}.png')]
        filt+="[b][1:v]overlay=enable='lt(t,6)'[a];[a][2:v]overlay=enable='gte(t,6)',fade=t=in:st=0:d=0.2,fade=t=out:st=11.8:d=0.2,format=yuv420p[v]"
        cmd+=['-filter_complex',filt,'-map','[v]','-t',str(DURATION),'-r','30','-an',
              '-c:v','h264_nvenc','-preset','p4','-cq','20','-b:v','0','-movflags','+faststart',str(target)]
        subprocess.run(cmd,check=True)
        print(f'rendered {index+1}/{len(SCENES)}',flush=True)
    listing=out/'concat.txt'
    listing.write_text(''.join(f"file 'clips/{i:02d}.mp4'\n" for i in range(len(SCENES))))
    final=out/'climbot_intro_zh_no_voice_v1.mp4'
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','warning','-y','-f','concat','-safe','1',
                    '-i',str(listing),'-c','copy','-movflags','+faststart',str(final)],check=True)
    digest=hashlib.file_digest(final.open('rb'),'sha256').hexdigest()
    (out/'video_sha256.txt').write_text(f'{digest}  {final.name}\n')
    print(f'DONE duration={len(SCENES)*DURATION}s file={final.name}',flush=True)


if __name__=='__main__':
    main()
