# 离线图像链 GPU 加速实施计划

更新：2026-09-01。本文是当前进行中的性能专项，规定预处理与墙面拼接怎样引入 CUDA、哪些
合同不得改变、每一步修改什么以及如何验收。当前拼接算法、P2-06 判据和证据边界仍以
[墙面拼接设计](MOSAIC_PLAN.md)为准；本文不重新定义几何或质量门限。

## 1. 目标、范围与非目标

目标是在保持 CPU 路径、不可变输入、原子发布和分阶段 provenance 的前提下，为离线图像链
增加可选 CUDA 后端，并降低完整 P2-06 重建时间。优先级按已测瓶颈排列：

1. `build_wall_mosaic` 的 tile 投影与 hard-cut 融合；
2. `process_inspection_archive` 的平场、暗场、去噪与去畸变；
3. 只有前两项完成后仍值得优化，才评估描述子匹配。

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

已验证的开发环境为 OpenCV `4.14.0`、CUDA Toolkit `12.8`、GPU 架构 `sm_120`。CUDA OpenCV
安装在工作区之外，系统 OpenCV `4.6.0` 继续供 ROS 2 和 CPU 路径使用。

禁止把 CUDA OpenCV 写入全局 `PYTHONPATH`、`LD_LIBRARY_PATH`、系统 Python 或仓库私有路径。
运行时由 `CLIMBOT_CUDA_OPENCV_ROOT` 指向隔离安装前缀；manifest 只记录版本、构建指纹和设备
信息，不记录该绝对路径。仓库、文档和测试不得出现具体用户名目录。

一个 Python 进程不得同时加载系统 OpenCV 与 CUDA OpenCV。公共 CLI 先由不导入 `cv2` 的控制层
解析后端，再启动干净子进程：

```text
CLI controller（stdlib + climbot_common，不导入 cv2）
  ├─ cpu child  → 系统 OpenCV 4.6 → 现有实现
  └─ cuda child → 隔离 OpenCV 4.14 + CUDA 12.8 → CUDA 实现
```

这条进程边界同时解决三件事：ROS 不会误载私有 OpenCV、CPU fallback 可以从干净状态重跑、
CUDA context 不会被 `fork` 到多个进程。

## 4. 公共后端合同

P1 与 fusion 的 CLI 都增加同一组参数：

| 参数 | 语义 |
| --- | --- |
| `--backend cpu` | 强制现有 CPU 路径；CUDA 环境完全不参与 |
| `--backend cuda` | 强制 CUDA；探测、显存或运行失败必须明确失败，不静默回退 |
| `--backend auto` | CUDA 自检通过才尝试；不可用或运行失败时清理 staging 后从头执行 CPU |
| `--cuda-opencv-root` | 可选显式前缀；未给时只读取 `CLIMBOT_CUDA_OPENCV_ROOT` |

实施期默认仍为 `cpu`。只有第 9 节全部通过，才在单独提交中讨论把默认改成 `auto`；不能把默认
切换夹在算法实现提交里。

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
- 解析 CUDA OpenCV 前缀，构造只对子进程生效的环境；
- 启动 probe/worker，校验严格 JSON 协议和退出码；
- 分类错误并编排 `auto` 的清理后重跑；
- 生成不含路径的 backend provenance。

新增 `climbot_common/cuda_probe.py`，只在 CUDA 子进程中导入 `cv2`。probe 不以
`getCudaEnabledDeviceCount() > 0` 作为唯一判据，还要实际完成：`GpuMat` 上传/下载、
`warpPerspective`、`remap`、算术运算和一次显存查询。版本号存在但 kernel 不能执行时必须失败。

单元测试全部使用伪 worker，不要求 CI 有 NVIDIA GPU。真实 GPU smoke test 是显式集成测试，没 GPU
时报告 skip，不能让普通 `colcon test` 依赖本机私有安装。

### 5.2 子进程协议

控制消息使用严格 JSON，只传参数、输入路径和 staging 身份；大图不通过 stdout、pickle 或管道
复制。worker 的 stdout 只允许最后一行机器可读结果，详细诊断写 stderr。所有数字禁止 NaN/Inf。

CUDA worker 仍使用现有“不存在的输出目录 + 同父目录随机 staging + `os.replace`”发布方式。
控制器只能在 child 成功且 manifest 回读通过后报告 completed。

## 6. Fusion CUDA 实现

### 6.1 保持在 CPU 的部分

以下代码继续使用现有实现：输入和 SHA-256 校验、投影和 bbox、位姿图读取、共同网格、tile 候选
顺序、TIFF/PNG/JPEG 编码、稀疏接缝提取、产物哈希和 provenance。GPU 不能改变稳定帧序和
`candidate_priority > owner_priority` 的严格平局规则。

### 6.2 放到 GPU 的部分

每个 tile 内依次执行：

1. `source_mask` 的 nearest `warpPerspective`；
2. 源图的 linear `warpPerspective`；
3. interior-distance priority 的 linear `warpPerspective`；
4. mask 与 priority 比较，更新 hard-cut image、owner 和 owner priority；
5. coverage、重叠灰度 sum/square-sum 累加并输出 tile 级 quality。

最终 image、owner、coverage 和 quality 下载一次。uncertainty 继续在 CPU 按最终 owner 和现有公式
生成，以减少 GPU 分支并优先保证现有量化结果逐位一致；这项等价重构必须先有单元测试证明。

第一版使用已有 `cv2.cuda` primitives，候选帧仍按稳定顺序串行更新，tile 内像素并行；禁止用
并发原子竞争改变平局结果。若 profiler 证明 kernel launch/中间 GpuMat 成为主要瓶颈，第二版才
增加一个小型 CUDA 聚合 kernel；不能在没有基线时直接维护整套自定义 warp。

### 6.3 显存图像缓存

本组 1,340 张 mono8 解码后约 `2.78 GiB`，可在 16 GiB GPU 中一次驻留。CUDA worker：

- 本组显存充足时每张 PNG 最多解码一次、上传一次；
- pose-only 与 optimized 共用同一份 GPU 图像缓存；
- source mask、interior-distance map 和常量矩阵只上传一次；
- 其他数据集显存不够时使用按字节 LRU，允许有记录的重复解码/上传，但不创建多个 CUDA 进程；
- 启动前以 `DeviceInfo.queryMemory()` 量测可用显存，至少保留 `2 GiB` 且不计划占用超过启动时
  空闲显存的 `80%`；不满足时按 backend 合同失败或回退。

### 6.4 Tile 流水

正式 TIFF 仍为 `512×512` tile。先比较 GPU render block `512/1024/2048`，但存储 tile 和输出
网格不得改变。采用双缓冲：GPU 计算下一块时，CPU 下载、统计并压缩上一块。上传/下载和 kernel
分别计时，不能只报告一个总数。

初版保留现有 raw 辅助缓存，以便只比较 CPU/CUDA 计算差异；通过数值验收后，再单独做写入流水
A/B。后续可让多个有界 writer 队列直接消费 image/coverage/uncertainty/difference tile，目标是
消除除 pose-only difference 所需数据之外的 raw 写放大。这个 I/O 改造必须单独提交和测量，不能
与首个 CUDA 结果混在一起。

### 6.5 计划修改的文件

| 文件 | 计划改动 |
| --- | --- |
| `climbot_mosaic/scripts/build_wall_mosaic` | 变成不预先导入 `cv2` 的 backend controller |
| `climbot_mosaic/fusion.py` | 保留 CPU 权威实现，抽出共享 grid、tile 组装和发布合同 |
| `climbot_mosaic/fusion_cuda.py` | GPU cache、CUDA warp、hard-cut 与统计 |
| `climbot_mosaic/cuda_worker.py` | CUDA child 入口与结构化错误转换 |
| `climbot_mosaic/test/test_cuda_fusion.py` | synthetic 等价、故障和 fallback 测试 |
| `climbot_mosaic/README.md` | 完成后加入当前 backend 用法和 manifest 字段 |

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
`auto` 候选；否则功能可以保留为显式 `cuda` 实验路径，但默认继续 CPU。

## 8. 数值与数据合同验收

CPU 是权威参考。验收使用同一输入、位姿图、分辨率、帧序和参数，不比较不同采集。要求分层：

### 8.1 必须逐位一致

- 输入帧数、身份、SHA-256 和共同网格；
- coverage count、有效 mask；
- hard-cut owner 与稀疏 seam adjacency；
- 由 owner 产生的 uncertainty 编码和 nodata；
- 所有标签、目录结构、完成/失败语义和 provenance 链接。

任一分类栅格不一致都表示算法合同改变，不能用“GPU 浮点差异”豁免。先调整实现或把该步留在
CPU，不能放宽验收。

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

CPU/GPU 原尺寸检查 tile 做自动像素差统计；若存在非零差，再对所有受影响 tile 做人工 100%
检查。没有像素差时，沿用自动哈希证明，不制造无信息的重复人工任务。

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

### 9.3 故障矩阵

必须覆盖：前缀缺失、错误 OpenCV、无 CUDA device、不兼容架构、probe kernel 失败、预检显存不足、
运行中 OOM、child 非零退出、child 被信号终止、输出协议损坏、用户 Ctrl-C、CPU 输出目录已存在、
CUDA staging 清理失败。每项都验证：退出码、错误文本、是否 fallback、没有 completed 半成品、
原始输入未修改。

普通全仓测试继续使用 `colcon test --executor parallel --parallel-workers 8`。CUDA 集成测试另外显式
执行，不能因为 CI 无 GPU 就降低普通测试覆盖。

## 10. 实施顺序与提交边界

| 阶段 | 内容 | 完成条件 |
| --- | --- | --- |
| G0 | CPU 解码复用基线 | **已完成**：`3b72f44`、十产物哈希一致、全仓 1,342 tests |
| G1 | 公共 backend controller、probe、provenance | CPU 默认逐位不变；无 GPU CI 测试全绿 |
| G2 | 单 tile CUDA fusion 原型 | 分类输出逐位一致，灰度满足第 8 节 |
| G3 | 完整 CUDA fusion、显存缓存、双缓冲 | 完整 P2-06 自动后验通过，无半成品 |
| G4 | 分项性能收敛；必要时聚合 kernel 或 direct writer | 达到至少 20% 加速，单项提交有 A/B |
| G5 | P1 CUDA 流水 | 全帧数值合同通过；至少 15% 才进入 auto 候选 |
| G6 | 可选 CUDA BF matching 评估 | 仅在仍是显著瓶颈且边集合逐位一致时实施 |
| G7 | 三轮交替长测、文档和默认策略决定 | 数值、性能、故障矩阵和全仓 `-j8` 全部通过 |

每阶段单独提交。观测、行为修改和门限/默认切换不能混在一个提交里；失败的实验记录结论后撤回，
不把试验代码留在主路径。G7 完成前，现有 CPU 正式 P2-06 证据仍是唯一权威基线。

## 11. 当前工作单

- [x] G0：CPU 解码复用与可测基线；
- [ ] G1：公共 backend controller 和真实 CUDA probe；
- [ ] G2：单 tile CPU/CUDA 等价性原型；
- [ ] G3：完整 fusion CUDA 后端；
- [ ] G4：性能收敛与 I/O 决策；
- [ ] G5：P1 CUDA 后端；
- [ ] G6：是否继续匹配加速的证据化决策；
- [ ] G7：完整验收、文档和默认后端决策。
