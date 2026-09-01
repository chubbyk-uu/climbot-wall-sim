# 离线图像链 GPU 加速实施计划

更新：2026-09-01。本文记录已完成的 fusion CUDA 性能专项及仍可复用的验收合同，并标明被
否决的预处理/CUDA OpenCV 路线。当前拼接算法、P2-06 判据和证据边界仍以
[墙面拼接设计](MOSAIC_PLAN.md)为准；本文不重新定义几何或质量门限。

## 1. 目标、范围与非目标

目标是在保持 CPU 路径、不可变输入、原子发布和分阶段 provenance 的前提下，为离线图像链
增加可选 CUDA 后端，并降低完整 P2-06 重建时间。当前结论按已测瓶颈排列：

1. `build_wall_mosaic` 的 tile 投影与 hard-cut 融合已落地，fusion 中位加速 `1.92×`；
2. `process_inspection_archive` 的 CUDA 原型端到端更慢，已撤回；
3. 后续只有 CUDA SIFT 仍可能为整链节省超过 `15 s`，须另做可行性验证。

不在本专项内：

- 不修改 Gazebo、RViz、ROS 2 在线控制或相机采集的 GPU 路径；
- 不引入 Torch、CuPy、cuDNN 或 Conda；
- 不把 SIFT 换成 ORB，也不改变候选、RANSAC、位姿图或 hard-cut 算法；
- 不放宽 P2-06 质量门限，不用 JPEG 预览代替 BigTIFF 验收；
- 不覆盖现有 CPU 正式证据。GPU 产物必须以新的 backend provenance 独立发布。

## 2. 已冻结的 CPU 基线

CPU 解码复用基线提交为 `3b72f44`。同一组 P2-06 输入为横向 680 帧加纵向 660 帧，共
1,340 张 `1920×1080 mono8` 图像，输出网格为 `38169×29081 @ 0.25 mm/px`。

| 指标 | 解码复用前 | `3b72f44` 基线 |
| --- | ---: | ---: |
| fusion 总耗时 | `101.99 s` | `88.81 s` |
| pose-only pass | `37.80 s` | `30.44 s` |
| optimized pass | `44.40 s` | `37.73 s` |
| 两遍 PNG 解码未命中 | `57,145` | `15,730` |
| 图像缓存命中率 | 约 `0%` | 约 `72.5%` |
| 峰值进程树 PSS | `6.66 GiB` | `7.26 GiB` |

两版的十个正式图像、覆盖、不确定度和接缝产物 SHA-256 逐个相等。`88.81 s` 是当前单次实测，
作为功能与初始性能参考；最终性能结论必须按第 9 节重复测量，不能把一次运行当作分布。

当前完整冷缓存主要阶段约为：两组 P1 预处理 `9 s`、SIFT `32 s`、局部匹配 `8.4 s`、fusion
`88.8 s`。因此 fusion 是第一目标；P1 即使加倍也只节省约 4.5 秒。

## 3. 运行环境与隔离边界

已验证环境为系统 OpenCV `4.6.0`、CUDA Toolkit `12.8`、GPU 架构 `sm_120`。生产 fusion 使用
pybind11 编译的自定义 C++/CUDA 扩展，**不依赖 CUDA OpenCV、Torch、Conda 或 cuDNN**；此前
隔离安装的 OpenCV `4.14.0` 只属于被否定的原型。

CPU-only 主机必须仍能配置和运行：CMake 找不到 CUDA compiler 时不构建扩展；找到时默认生成
`sm_120`，其他设备通过 `CLIMBOT_CUDA_ARCHITECTURES` 覆盖。CLI controller 自身不导入 OpenCV，
CPU/CUDA 都在干净 child 中运行，CUDA context 不会被 fork。manifest 只记录扩展摘要、CUDA
版本和设备信息，不记录编译器、安装前缀或用户目录。

## 4. 公共后端合同

生产 `build_wall_mosaic` CLI 提供以下后端参数；P1 CUDA 原型已撤回，不提供后端选项：

| 参数 | 语义 |
| --- | --- |
| `--backend cpu` | 强制现有 CPU 路径；CUDA 环境完全不参与 |
| `--backend cuda` | 强制 CUDA；探测、显存或运行失败必须明确失败，不静默回退 |
| `--backend auto` | CUDA 自检通过才尝试；不可用或运行失败时清理 staging 后从头执行 CPU |

最终默认保持 `cpu`，让没有 CUDA 的机器与历史命令继续得到确定的 CPU 行为。本项目 CUDA
开发机的完整拼接、性能和验收运行则显式使用 `cuda`；`auto` 只用于允许回退的普通批处理。

错误分三类，控制器必须区分：

1. 输入、哈希、参数和输出合同错误：任何后端都应失败，`auto` 不得借 fallback 掩盖；
2. CUDA 环境不可用、设备不兼容或显存不足：`cuda` 失败，`auto` 转 CPU；
3. CUDA 执行中 OOM、worker 崩溃或协议损坏：清理未发布临时目录，`cuda` 失败，`auto` 从零
   重启 CPU，不能从半成品续跑。

最终 manifest 的 `execution.backend` 至少记录：

- `requested`、`effective`、是否 fallback、结构化 fallback reason；
- OpenCV 版本、`cv2` 模块 SHA-256、`getBuildInformation()` SHA-256；
- CUDA 构建版本、设备号、compute capability、总显存和启动时空闲显存；
- GPU 图像缓存峰值、host PSS、各阶段 wall time、上传/下载字节数；
- 每次 backend attempt 的开始、结果和错误类别。

stage provenance 同步记录影响结果或调度的后端参数，但不记录机器私有绝对路径。

## 5. 共享运行层改动

### 5.1 `climbot_common`

新增纯标准库模块 `climbot_common/acceleration.py`：

- 解析和校验 backend 参数；
- 构造不污染父进程的 child 环境；
- 启动 worker，校验严格 JSON 协议和退出码；
- 分类错误并编排 `auto` 的清理后重跑；
- 生成不含路径的 backend provenance。

早期 CUDA OpenCV 原型使用过独立 `cuda_probe.py` 和安装前缀隔离；原型否决后两者均已删除。
生产 CUDA worker 直接加载本包的自定义扩展，扩展负责设备、compute capability、显存和 kernel
执行检查，不能仅凭库或设备名称存在就报告可用。

单元测试全部使用伪 worker，不要求 CI 有 NVIDIA GPU。真实 GPU smoke test 是显式集成测试，没 GPU
时报告 skip，不能让普通 `colcon test` 依赖本机私有安装。

### 5.2 子进程协议

控制消息使用严格 JSON，只传参数、输入路径和 staging 身份；大图不通过 stdout、pickle 或管道
复制。worker 的 stdout 只允许最后一行机器可读结果，详细诊断写 stderr。所有数字禁止 NaN/Inf。

CUDA worker 仍使用现有“不存在的输出目录 + 同父目录随机 staging + `os.replace`”发布方式。
控制器只能在 child 成功且 manifest 回读通过后报告 completed。

## 6. Fusion CUDA 实现

### 6.1 CPU/GPU 边界

CPU 继续负责输入与 SHA-256 校验、投影/bbox、位姿图、共同网格、候选顺序、TIFF/PNG/JPEG、
稀疏接缝、产物哈希、provenance 和原子发布。CUDA 只替换每个 tile 的透视采样、coverage、
interior-distance hard-cut、owner、uncertainty 和重叠质量归约，因此失败时仍走同一套清理合同。

自定义 kernel 以 double 计算逆投影坐标，再按 OpenCV `INTER_LINEAR` 的 `1/32 px` 表量化采样点；
候选按稳定帧序循环，只有 `priority > best` 才换 owner，不用并发原子竞争决定归属。完整数据证明
coverage、owner 和接缝与 CPU 逐位一致。连续坐标变体会改变少量 owner，却没有改善锚点或接缝
等级指标，已删除而不是保留第二套语义。

### 6.2 驻留、显存与输出

1,340 张 `1920×1080 mono8` 由 8 个有界线程并行解码，每张只上传一次；约 `2.78 GiB` 图像在
pose-only 与 optimized 两遍之间共享。启动时实测空闲显存，计划占用不得超过其 `80%`，并至少
留 `2 GiB`；不足时 `cuda` 失败、`auto` 从零改跑 CPU。每个正式 `512×512` tile 只下载 image、
owner、coverage、uncertainty 和 4096-bin 质量直方图，不下载整幅浮点质量图。

TIFF 和派生产物仍是后半程主要成本，继续用 lossless deflate level 1。raw 缓存和单写入器没有
为了 GPU 改写；若以后优化 I/O，必须独立 A/B，不能把压缩或 schema 变化算成 CUDA 收益。

### 6.3 实际文件边界

| 文件 | 当前职责 |
| --- | --- |
| `scripts/build_wall_mosaic` | OpenCV-free controller、backend 选择与 fallback |
| `fusion.py` | CPU 权威实现和共享 grid、TIFF、接缝、发布合同 |
| `fusion_cuda.py` | 解码/上传、tile 任务适配和 CUDA 计时 |
| `mosaic_cuda_worker.py` | CUDA child、设备 provenance 与结构化错误 |
| `src/fusion_cuda.cu` | 自定义 hard-cut kernel 与 pybind11 session |
| `test_cuda_fusion.py` | 有 GPU时执行 CPU/CUDA 小型全链合同测试 |

## 7. P1 预处理 CUDA 实现

每帧保持现有顺序：`mono8 → float32 → dark subtract → max(0) → flat-field multiply → clip
→ uint8 → optional median3 → remap`。gain、dark、`remap_x/y` 在整次 run 中只上传一次。

GPU 只做像素计算，PNG 解码、编码、SHA-256、标签和原子写入仍在 CPU。单 CUDA context 配合
有界帧队列；CPU 解码、GPU 处理和 CPU 编码可流水重叠。不得为每个 CPU worker 创建 CUDA
context，也不得在创建 context 后使用 `fork`。若编码成为瓶颈，优先使用有界线程池并证明 OpenCV
调用释放 GIL；需要进程时使用 `spawn` 且子进程不接触 CUDA。

计划修改：

| 文件 | 计划改动 |
| --- | --- |
| `climbot_image_processing/scripts/process_inspection_archive` | 公共 backend controller |
| `climbot_image_processing/processing.py` | 保留 CPU 参考和共享校验/发布逻辑 |
| `climbot_image_processing/processing_cuda.py` | 常量驻留、帧流水和 CUDA 运算 |
| `climbot_image_processing/cuda_worker.py` | CUDA child 入口 |
| `climbot_image_processing/test/test_cuda_processing.py` | 顺序、数值、fallback、原子失败测试 |
| 两个包的 `package.xml` | P1 增加 `climbot_common` 运行依赖；不声明私有 CUDA OpenCV 为系统包 |

P1 当前两组只需约 `9 s`。只有 GPU 中位耗时至少降低 `15%` 且不降低吞吐稳定性，才保留
`auto` 候选；否则默认继续 CPU。未通过本节数值或性能验收的试验实现必须撤回，不保留一个
看似可用、实际没有收益的显式 `cuda` 路径。

## 8. 数值与数据合同验收

CPU 是权威参考。验收使用同一输入、位姿图、分辨率、帧序和参数，不比较不同采集。要求分层：

### 8.1 必须逐位一致

- 输入帧数、身份、SHA-256 和共同网格；
- coverage count、有效 mask；
- hard-cut owner 与稀疏 seam adjacency；
- 所有标签、目录结构、完成/失败语义和 provenance 链接。

任一分类栅格不一致都表示算法合同改变，不能用“GPU 浮点差异”豁免。先调整实现或把该步留在
CPU，不能放宽验收。

uncertainty 是 owner 与后验协方差推导出的连续量，不参与 owner 或覆盖判定。CPU/CUDA 分别执行
float32 开方后，允许编码最大差 `1 count = 0.01 mm`，差异比例不得超过 `1e-5`，nodata 必须
逐位一致。完整 P2-06 实测为 `2,446 / 1,109,992,689 ≈ 2.2e-6`，最大 `1 count`。强行搬回
Python/CPU 仍有 509 个边界舍入像素，却使 fusion 从约 `47 s` 增至 `71 s`，因此不采用。

### 8.2 灰度连续量

先用合成恒定图、斜坡、棋盘、单像素边、边界外推和随机图冻结 CPU/CUDA 插值误差；随后在完整
P2-06 上比较两张母版和差分。允许范围的上限为：绝对灰度差 `max ≤ 2 DN`，且 `p99.99 ≤ 1 DN`。
若合成测试表明可以逐位一致，就把正式合同收紧为逐位一致，不保留多余容差。

P1 对每帧执行同一比较，并额外要求 shape、dtype、饱和像素数、均值和角/中心比不退化。编码字节
可因 OpenCV 版本不同而变化，因此先解码比较像素，再分别校验输出文件自己的 SHA-256；不能把
PNG 文件摘要不同误判为像素不同。

### 8.3 下游 P2-06

GPU fusion 必须重跑当前全部自动后验：巡检域零漏拍、绝对锚点、局部残差、尺度、航向、接缝/
off-seam 比和 optimized 不劣于 pose-only。全部继续使用已经冻结的门限，不因加速重新推导。

CPU/GPU 原尺寸检查 tile 做自动像素差统计。分类栅格变化或灰度超过第 8.2 节容差才重新做受影响
tile 的人工检查；只有 `≤1 DN` 的已界定插值差时，以完整自动后验和既有人工基线收口，不制造
无法靠肉眼鉴别的重复任务。

## 9. 性能、稳定性与故障验收

### 9.1 测量方法

- 机器空闲，不同时运行 Gazebo、RViz 或其他 GPU 作业；
- 每种 backend 预热一次，不计入结果；
- CPU/GPU 交替各运行至少三次，报告 min/median/max，避免顺序和页缓存偏差；
- 输入和大型临时产物放数据根或 `/tmp`，只有最终小型摘要可进入 `results/`；
- 记录 wall time、各 pass、PNG decode、上传/下载、kernel、编码、磁盘写入、host PSS、GPU
  峰值显存与 fallback 次数。

### 9.2 性能门

fusion 以当前 `88.81 s` 单次结果为初始参考，最终以同轮重复 CPU 中位数为分母：

- 最低保留门：GPU 中位总耗时至少快 `20%`；
- 目标：GPU fusion 中位 `≤45 s`；
- 任一次不得因 OOM、worker 泄漏或临时文件写爆而使系统失去响应；
- host 峰值 PSS 不高于 `8 GiB`，GPU 计划占用不超过启动空闲显存 `80%` 且至少留 `2 GiB`。

若只加 CUDA warp 达不到最低门，先依据分项计时决定是做聚合 kernel、扩大 render block 还是处理
TIFF/raw I/O；不能凭感觉同时改三项。

2026-09-01 在机器空闲、CPU/CUDA 交替各三轮的完整 1,340 帧结果：

| 口径 | CPU min/median/max | CUDA min/median/max | 中位加速 |
| --- | ---: | ---: | ---: |
| fusion manifest | `91.13 / 91.86 / 91.92 s` | `46.89 / 47.73 / 47.82 s` | `1.92×` |
| CLI 全程 | `107.25 / 108.01 / 108.03 s` | `63.21 / 64.02 / 64.14 s` | `1.69×` |

最低保留门通过；`≤45 s` 的进取目标差 `2.73 s`，不为这点差距改写 TIFF 或输出合同。单次峰值
进程树 PSS 约 `1.78 GiB`，驻留图像 `2.78 GiB`，启动空闲显存约 `14.60 GiB`。

### 9.3 故障矩阵

必须覆盖：扩展缺失、无 CUDA device、不兼容架构、kernel 失败、预检显存不足、
运行中 OOM、child 非零退出、child 被信号终止、输出协议损坏、用户 Ctrl-C、CPU 输出目录已存在、
CUDA staging 清理失败。每项都验证：退出码、错误文本、是否 fallback、没有 completed 半成品、
原始输入未修改。

普通全仓测试继续使用 `colcon test --executor parallel --parallel-workers 8`。CUDA 集成测试另外显式
执行，不能因为 CI 无 GPU 就降低普通测试覆盖。

## 10. 实施顺序与提交边界

| 阶段 | 内容 | 完成条件 |
| --- | --- | --- |
| G0 | CPU 解码复用基线 | **已完成**：`3b72f44`、十产物哈希一致、全仓 1,342 tests |
| G1 | 公共 backend controller、worker 协议、provenance | **已完成**：CPU 默认仍走 system-OpenCV child；CUDA worker 严格失败分类；无 GPU 包级测试全绿 |
| G2 | 单 tile CUDA fusion 原型 | **不采用**：`cv2.cuda.warpPerspective` 不满足当前 CPU 像素合同，见第 10.1 节 |
| G3 | 完整自定义 CUDA fusion 与显存缓存 | **已完成**：完整 P2-06 自动后验通过，无半成品 |
| G4 | 分项性能收敛与 I/O 决策 | **已完成**：fusion 中位 `1.92×`；保留现有 lossless 输出 |
| G5 | P1 CUDA 流水 | **不采用**：端到端慢于 8-worker CPU，见第 10.2 节 |
| G6 | 可选 GPU 特征阶段 | 未实施；只有 CUDA SIFT 仍可能达到整链 `≥15 s` 收益 |
| G7 | 三轮交替长测、文档和默认策略决定 | 数值、性能、故障矩阵和全仓 `-j8` 全部通过 |

每阶段单独提交。观测、行为修改和门限/默认切换不能混在一个提交里；失败的实验记录结论后撤回，
不把试验代码留在主路径。CPU 正式 P2-06 证据仍是验收基线；CUDA A/B 证明执行后端等价，不
覆写既有证据目录。

### 10.1 G2：CUDA OpenCV fusion 原型的否定结论（2026-09-01）

G2 用三张确定性 `29×37` mono8 随机图、包含整数平移、半像素平移和小旋转/剪切的三张
homography，在 `64×64` tile 上对照当前 system OpenCV `4.6` 的 CPU 参考。CUDA OpenCV
`4.14` 已通过上传下载、warp、remap、算术和显存 probe，但其 `warpPerspective` 不能承担本项目的
fusion 合同：

- source mask 有 `96` 个 coverage 像素不同；因此 hard-cut owner 有 `36` 个分类像素不同；
- 有效源域内灰度最大差为 `5 DN`，超过 `max ≤ 2 DN` 的冻结上限；
- 把矩阵预求逆并使用 `WARP_INVERSE_MAP`，mask 差异仍为 `96`，没有改善；
- 用隔离 OpenCV `4.14` 的 CPU warp 复核，仍得到同样的 `36` 个 owner 差异，故不是 `4.6`/`4.14`
  的 CPU 版本差，而是 CUDA warp 的取样语义不同。

owner、coverage、接缝和不确定度是分类/合同产物，不能以“GPU 浮点误差”豁免。试验中的 CUDA
worker 和 tile 实现已撤回；不接入 CLI、不产生正式数据，也不改变 CPU 基线。

若未来仍要加速 fusion，唯一诚实的路线是编写并长期维护一个逐像素复现当前 CPU
`warpPerspective` 最近邻/线性取样规则的自定义 CUDA kernel，并先在 G2 重做本节的分类和灰度
验收。这已不再是低风险的 OpenCV 后端替换，不能在没有单独决策的情况下推进。当前可继续评估的
GPU 目标是 P1 预处理：它不依赖透视采样的分类归属，且有独立逐帧数值合同。

### 10.2 G5：P1 CUDA OpenCV 可行性原型的否定结论（2026-09-01）

原型覆盖 dark/gain 的四种开关组合、`median3`，以及随机图、棋盘、斜坡、边界外推和非整数
remap。得到三个可复用结论：

1. CUDA `remap` 使用连续坐标，而 CPU `remap` 使用 `1/32` 像素插值表。GPU 使用
   `round(map * 32) / 32` 的兼容坐标后，36 组合合成测试和 32 张真实 `1920×1080` 帧均满足
   `max = 1 DN`、`p99.99 = 1 DN`。这不是“GPU 更好”的证据，只是保证两个取样定义可比。
2. 隔离 OpenCV `4.14` 自行计算 `getOptimalNewCameraMatrix` 时，与生产 system OpenCV `4.6`
   的 new-camera matrix 最大相差 `0.922 px`，生成的 remap 最大相差 `1.027 px`，会导致真实帧
   最大 `115 DN` 的输出差。因此 CUDA 生产实现若重启，必须使用由 CPU 参考阶段冻结并传入的
   remap，不得在 CUDA 子进程重新标定；这个差异也没有被当作“画质提升”。
3. CUDA 12 的 wavelet-matrix median 与 CPU `medianBlur` 的边缘/并列值规则不同。可保留 CPU
   median 后再回 GPU remap，不能静默替换成不同的滤波器。

性能不满足保留条件。常量驻留后，在 100 张真实 `1920×1080` 帧上，GPU 的解码、上传、像素
计算、下载（尚未编码）为 `14.26 ms/frame`；单核 CPU 同口径为 `20.83 ms/frame`。但当前 CPU
归档以 8 个 worker 完整处理 680 帧仅 `4.769 s`，约 `7.01 ms/frame`，且已经包含 PNG 编码和
写入。GPU 的纯像素阶段已是该端到端基线两倍，无法达到 `15%` 加速门槛。多 stream 原型仅降至
`13.95 ms/frame`，不足以改变结论。

因此试验中的 CUDA session/child 已撤回：不接入 CLI、不生成正式数据、不改变 P1 CPU 基线。
若将来 GPU 需要用于提升图像质量，应先提出独立的、可量化的画质目标（例如独立靶标的几何误差
或平场均匀性），再与当前 CPU 输出和下游 P2-06 验收做盲测；不能仅以输出不同或新版本 OpenCV
为理由替换生产算法。

### 10.3 G3/G4：自定义 CUDA fusion 收口结果（2026-09-01）

自定义 kernel 避开了第 10.1 节的 OpenCV CUDA 取样差异。完整 P2-06 对比结果为：coverage
两个栅格逐位一致，pose-only/optimized 接缝数组逐项一致；两张母版分别约 `0.63%`、`0.62%`
像素相差 `1 DN`，最大也是 `1 DN`。uncertainty 最大差 `0.01 mm`，差异比例 `2.2e-6`。

后验评价接受 8 个共同锚点；optimized 锚点 P95 `1.130 mm`，巡检域内 feature 漏拍 `0`，
optimized/pose-only 接缝 excess P95 为 `7 / 16 DN`，全部保持原冻结判据。连续坐标变体把
optimized 接缝数改变 `2,015` 条，却没有改变 P95/P99、锚点等级或运行时间；均值改善只有约
`0.00065 DN`，不足以证明更好，因此对外参数和分支均删除。

默认最终保持 `cpu`：它是所有机器上的确定基线，也不会因有无 GPU 改变命令结果。`cuda` 是已
验收的显式加速路径；`auto` 供希望优先 GPU、失败再 CPU 的批处理使用。CUDA OpenCV 安装不再是
生产依赖。

## 11. 当前工作单

- [x] G0：CPU 解码复用与可测基线；
- [x] G1：公共 backend controller、结构化错误和 provenance；
- [x] G2：完成可行性原型；`cv2.cuda` fusion 因数值合同不通过而撤回；
- [x] G3：完整自定义 fusion CUDA 后端、显存守卫和真实 GPU 集成测试；
- [x] G4：完整 P2-06 数值/后验与三轮交替性能 A/B；
- [x] G5：完成 CUDA OpenCV 可行性原型；数值兼容可做，但端到端性能不达门槛，已撤回；
- [ ] G6：CUDA SIFT 可行性验证（独立后续，不属于本次 fusion 收口）；
- [x] G7：完整验收、文档和默认策略；全仓 `-j8` 为 1,370 tests、0 failures。
