# 历史分册 3

### 扫描用的开关

| 开关 | 含义 | 默认 |
| --- | --- | --- |
| `-s` / `-i` | 全站仪 / IMU 噪声种子 | `42` / `17` |
| `-n` | 全站仪位置噪声 `stddev`，米 | `0.001` |
| `-r` | 全站仪发布频率，Hz | `12.0` |
| `-d` | 全站仪丢包率 | `0.0` |
| `-o` | 覆盖一个 `line_tracker` 参数，`name=value`，可重复 | 无 |

默认值全部照抄 launch 文件的，所以不带开关跑出来就是普通配置，每个扫描点与基线只差
一个数。`-o` 是拷一份 `control.yaml` 打补丁放进运行目录，不动工作树。

**实际生效的值不靠转述**：评价器启动后向 `total_station_sim`、`wall_imu_adapter` 和
`line_tracker` 的参数服务问回来，写进摘要的 `provenance.noise_sources` 和
`provenance.control_parameters`。传给一个没起来的节点的参数，在摘要里看得出来。

### 诊断墙的 realistic 定位 profile

默认 `precision` 保持既有 `12 Hz`、`1 mm` 位置白噪声和已知 `10 ms` 送达延迟。需要
重新采集诊断墙数据时，以 `realistic` 启动；它的初始可校准参数是机器人系
`[20, -10, 0] mm` 棱镜残差，以及仅作用于观测 header 的 `+20 ms` 时钟偏差和
`2 ms` 零均值抖动：

```bash
ros2 launch climbot_bringup coverage_mission.launch.py \
    wall_texture:=textures/wall_diagnostic_025/wall_texture.json \
    wall_grid_spacing:=0 \
    localization_profile:=realistic
```

棱镜残差按 Gazebo 真值 yaw 投影到墙面坐标，正反向扫描会得到相反的墙面内投影；它不改变
发布协方差。时间戳项不改变送达时间：已知 `10 ms` 队列仍按真实采样时刻送达，而 clock
残差只改 header stamp。若要分开做消融试验，可覆盖任一项：

```bash
# 保留 realistic 的时间戳残差，但关闭棱镜残差
ros2 launch climbot_gazebo climbot_wall.launch.py headless:=true \
    localization_profile:=realistic prism_extrinsic_error_mode:=disabled

# precision 基线中只启用棱镜残差，并指定另一个机器人系向量（m）
ros2 launch climbot_gazebo climbot_wall.launch.py headless:=true \
    localization_profile:=precision prism_extrinsic_error_mode:=enabled \
    prism_extrinsic_error_robot_m:='[0.010, 0.008, 0.0]'
```

`enabled` / `disabled` 显式覆盖 `auto`；时间戳项同理使用
`measurement_timestamp_error_mode`。摘要通过参数服务记录 profile、实际开关、残差、偏差、
抖动、种子和 Gazebo 真值/EKF 相机中心误差；不能把调参值事后写入归档标签冒充传感器输入。
当前正式证据及其有效范围见 [结果索引](../../../../results/README.md)。

### 诊断墙真值评价

拼接完成后，才可用独立评价器把 hard-cut BigTIFF 与诊断墙 DDS 母版的米制特征对齐。它只读取
正式拼接结果、`wall_texture.json` 和 DDS；不得接入候选生成、匹配或位姿图优化：

```bash
ros2 run climbot_mosaic evaluate_diagnostic_mosaic \
  --mosaic-dir "$CLIMBOT_DATA_ROOT/<mosaic-run>" \
  --wall-manifest "$PWD/textures/wall_diagnostic_025/wall_texture.json" \
  --output-dir "$CLIMBOT_DATA_ROOT/<mosaic-run>-truth-<new-id>" \
  --anchor-padding-m 0.10 --minimum-phase-response 0.10
```

评价器仅采用两个图上都完整可见的修补块／涂鸦锚点；相位相关响应低于门限的锚点被明确拒绝，
两个版本只用同一组保留锚点比较。输出记录绝对锚点偏差、尺度 ppm、航向、局部相似变换残差和
所有锚点的响应。少于三个保留锚点时仍报告整体相似变换，但 `local_residual_observable=false`，
不得把两点刚好拟合的零残差解读为局部无形变。

可用现成脚本采集横向和竖向诊断任务。脚本的 `PASS` 只表示任务、归档、照片绑定和几何采集
合同通过；绝对误差应从真值摘要读取，不能把脚本退出码解释为定位验收。

```bash
INSPECTION_OUTPUT_ROOT="$CLIMBOT_DATA_ROOT" \
WALL_TEXTURE=textures/wall_diagnostic_025/wall_texture.json \
LOCALIZATION_PROFILE=realistic G2_MAX_CAMERA_POSITION_ERROR_M=1.0 \
tools/run_g2_inspection_acceptance.sh <horizontal-case> <vertical-case>
```

### 原尺寸缺陷／接缝巡检

`inspect_diagnostic_mosaic` 是一条**后验**证据链。它只读取已完成的 hard-cut
拼接、覆盖次数图和诊断墙 DDS；不参与候选、局部匹配或位姿图优化。对每个诊断墙 feature，
它输出一组不缩放的 PNG tile，三个横向 panel 从左至右固定为 `immutable_truth`、`pose_only`、
`optimized`；每个 panel 保持母版的 `0.25 mm/px`，方便在图像查看器中以 100% 缩放逐项查看。

```bash
ros2 run climbot_mosaic inspect_diagnostic_mosaic \
  --mosaic-dir "$CLIMBOT_DATA_ROOT/<mosaic-run>" \
  --wall-manifest "$PWD/textures/wall_diagnostic_025/wall_texture.json" \
  --output-dir "$CLIMBOT_DATA_ROOT/<mosaic-run>-inspection-<new-id>" \
  --padding-m 0.05 --tile-size-px 2048
```

输出的 `diagnostic_inspection_summary.json` 逐项列出 feature 的完整／部分／网格外可见性、实际
`coverage_count.tif` 的零覆盖／单图／重叠像素数，以及渲染阶段已经对**所有**重叠像素计算的
灰度离散。这里“100%”只指每个与实际拼接域相交的诊断 feature 都导出了原尺寸 tile，绝不把
网格相交或 feature 的包围盒误写成相机足迹覆盖；零覆盖像素必须保留为失败边界，而不是由黑色
填充后声称已检查。该工具的输出是人工细节检查的可复核证据，不产生新的毫米门限。

### §14.5 定位对照

不走回归脚本，单独跑一次四方向闭环，逐段用真值同时测融合位姿误差和轮式航位推算
误差：

```bash
ros2 launch climbot_gazebo climbot_wall.launch.py use_sim_time:=true headless:=true
ros2 run climbot_gazebo evaluate_localization.py --ros-args -p use_sim_time:=true \
    -p summary_json:=results/localization_<tag>_summary.json
```

## 墙面贴图

墙面默认是平色的。贴图是**本地生成**的：源素材 `540 MB`、烘焙产物约 `250 MB`，
两者都不进仓库。仓库里的是脚本、源素材的 `sha256` 和烘焙种子——这三样一起把
产物钉死，因为这张图不只是布景，它是拼接结果要比对的基准，换一版素材就等于换了
一面墙而没人看得出来。

```bash
# 一次性：下载并校验源素材，只解出颜色图（约 540 MB）
tools/fetch_wall_texture.sh

# 烘焙整面墙：40000 × 32000 纹素（0.25 mm/texel），BC1 DDS，mip 链写在文件里。
# 四个编码 worker 保持相同像素与清单顺序；内存紧张时改为 --jobs 1。
python3 tools/bake_wall_texture.py \
    --source-dir "${TMPDIR:-/tmp}/climbot_wall_texture/maps" \
    --output-dir textures/wall --source-size-m 2.5 --jobs 4

# 用它启动
ros2 launch climbot_bringup coverage_mission.launch.py \
    wall_texture:=textures/wall/wall_texture.json wall_grid_spacing:=0
```

`--source-size-m 2.5` 没有默认值，必须显式给：ambientCG 没有记录 `Concrete044D`
的真实尺寸，这个数是**本项目的声明**，而烘焙里每一个长度都是从它推出来的。

默认烘焙使用 `1536 px` quilting patch、`25%` patch 重叠，并在每个 `2048 px`
渲染块四周携带 `128 px` 的真实相邻像素。后一个边框不是重复填充：相邻 visual 会在
同一段墙面坐标上显示同一段整墙图，使双线性过滤、mipmap 和 BC1 采样不会在 DDS
边缘各自钳位成一条接缝。`wall_texture.json` 中分别记录名义覆盖范围和带边框采样范围；
前者仍无重叠、无空洞地覆盖墙面，后者允许 visual 在外观上重叠。旧清单可以继续加载，
但不具备这项接缝保护，要获得新效果必须重新运行烘焙命令。

`128 px` 不是只为原始分辨率准备的。它使相邻块经过五级 2× 下采样及 BC1 的 `4×4`
编码后仍保持相同采样相位，覆盖 Gazebo GUI 从近景到整墙视角通常使用的 mip 范围。
代价是贴图纹素和压缩产物比无边框版本增加约 `24%`，仍在烘焙器的显存预算内。

### P2.7a 诊断墙

`wall_025` 是当前精度／混凝土基线，必须保持不变。以下命令从它的 DDS 块生成一个
**独立**的诊断墙：其中只有少量不规则施工缝、裂纹、修补块和喷涂标记，用来检查拼接
对可人工辨识细节的恢复能力；它不是用于特征算法的密集标定格。

```bash
python3 tools/create_diagnostic_wall.py \
  --base-manifest textures/wall_025/wall_texture.json \
  --output-dir textures/wall_diagnostic_025 \
  --seed 20260827 \
  --preview docs/images/wall_diagnostic_preview.png

ros2 launch climbot_bringup coverage_mission.launch.py \
  wall_texture:=textures/wall_diagnostic_025/wall_texture.json \
  wall_grid_spacing:=0
```

生成器拒绝覆盖已有输出目录。裂缝和白色手绘标记来自透明贴片图集
`tools/assets/diagnostic_wall_decals_v2.png`；v2 裂缝源图本身就是细长、自然分叉且末端渐细的
发丝裂纹。渲染器保留其走势和分叉，在图集原始分辨率上规范中心线后再统一颜色、尺寸、
角度和透明度；最终可见轮廓中位宽约 `4.0 mm`，较实核心中位宽约 `2.8 mm`。清单的 `diagnostic_wall` 段记录输入清单
SHA-256、贴片图集 SHA-256、生成 seed 和每一条特征的墙面米制坐标；所有特征按同一坐标写入相邻 DDS 的重叠 gutter，
因此块边界本身不能制造假接缝。诊断墙仍只是 wall 的视觉层，不改变碰撞、摩擦、
WheelSlip 或控制参数。`textures/` 产物不进 Git，脚本、seed 和 manifest 共同构成可复现
定义。

quilting 仍先在 `384 px` 重叠区寻找最小误差裁切路径，再只在该路径附近做默认
`96 px` 的窄羽化。羽化不会跨出两块真实照片共同覆盖的区域；它用于去掉硬裁切留下的
亮度边，而不是把整个 overlap 模糊成平均图。可用 `--seam-feather-px 0` 生成硬裁切
对照，但正式墙面不使用该模式。

拍照运行请一并加 `wall_grid_spacing:=0`（只关墙面那套，RViz 不受影响），理由见上面
的参考网格线一节。

贴图只改墙面**外观**：碰撞盒、摩擦系数和按它标定的 WheelSlip 参数都不动，贴图块
浮在墙面前 `1 mm`（参考网格线是 `3 mm`，那个距离足以让拼接的单应性把它当成第二个
平面）。

也可以把清单路径写进 `climbot_gazebo/config/simulation.yaml` 的
`wall.texture_manifest` 长期生效；launch 参数优先级更高。路径指向的文件不存在时
launch **直接报错退出**，不会退回平色墙——一整轮拍出来的白墙照片看起来像相机故障，
发现它的代价是整轮重跑。

### 渲染照片验收

`capture_wall_texture.py` 用 Gazebo server 的真实 Ogre2 相机传感器采图，不是从 DDS
离线裁图。相机位姿由共享 `wall.yaml` 与 `simulation.yaml` 推导，完整 Gazebo/bridge
日志和原始 PNG 应放在 `/tmp`，只有小型 JSON 摘要进入 `results/`。

整墙位置可区分性用 `5×4` 个参考位置；每个查询相对参考平移
`(60, 25) mm`、距离扰动 `0.5%`、航向扰动 `±0.6°`，然后对全部参考位置做 ORB +
RANSAC 检索：

```bash
source install/setup.bash
python3 tools/evaluate_wall_texture_photos.py global \
  --manifest textures/wall/wall_texture.json --columns 5 --rows 4 \
  --batch-size 12 --work-dir /tmp/climbot_wall_global_<tag> \
  --output results/wall_texture_global_<tag>_summary.json
```

贴图精度对照不能让三档各自独立 quilting，否则抽到的混凝土内容会与纹素精度混在
一起。先以最细档烘焙一份 canonical，再从其 BC1 mip 0 解码并 LANCZOS 下采样；三档
因此拥有同一内容、同一接缝和同一墙面坐标，只改变纹素尺度：

```bash
# 示例使用 3×2 m 局部墙，避免为一次对照生成三份整墙。
python3 tools/bake_wall_texture.py \
  --source-dir /tmp/climbot_wall_texture/maps \
  --output-dir /tmp/wall_resolution/0.26 --source-size-m 2.5 \
  --region-m 3.0 2.0 --region-origin-m 3.5 3.0 \
  --scale-mm-per-px 0.26 --patch-px 2954 \
  --overlap-fraction 0.25 --seam-feather-px 185
python3 tools/resample_wall_texture.py \
  --manifest /tmp/wall_resolution/0.26/wall_texture.json \
  --output-dir /tmp/wall_resolution/0.50 --scale-mm-per-px 0.50
python3 tools/resample_wall_texture.py \
  --manifest /tmp/wall_resolution/0.26/wall_texture.json \
  --output-dir /tmp/wall_resolution/0.75 --scale-mm-per-px 0.75

python3 tools/evaluate_wall_texture_photos.py resolution \
  --candidate 0.26=/tmp/wall_resolution/0.26/wall_texture.json \
  --candidate 0.50=/tmp/wall_resolution/0.50/wall_texture.json \
  --candidate 0.75=/tmp/wall_resolution/0.75/wall_texture.json \
  --columns 3 --rows 2 --batch-size 12 \
  --work-dir /tmp/wall_resolution/rendered \
  --output results/wall_texture_resolution_<tag>_summary.json
```

正式判读以 `ransac_inliers.median` 排序，同时检查最小值、内点率和描述子余量；不要
只看特征点数量，也不要把预期档位通过 `--expected-best` 写成结论。该选项只适合在排序
已由正式实验定案后作为回归门禁使用。

## 安全提示

Gazebo DiffDrive 会持续执行最后收到的 `/cmd_vel`，因此仿真 launch 始终启动速度
看门狗，并由它作为 `/cmd_vel` 的唯一发布者。键盘、实验脚本和自动控制统一发布到
`/control/cmd_vel`；当前一次只能运行一个上游控制源，不要同时启动键盘和自动任务。

控制环和看门狗的定时器**不使用节点默认时钟**。节点默认时钟在非仿真时间下就是
系统时钟，可以被设置、可以往回跳（WSL2 每约 30 s 回跳 1～2 s），建在它上面的
定时器在回跳期间不触发——控制器整段不发指令，而机器人还在按最后一条指令走。
仿真时间激活时跟 `/clock`，否则用单调时钟，见
[INTERFACES.md](../../../INTERFACES.md) 的“坐标、控制与定位”。
