# climbot_mosaic

离线链的末级：把一批 processed-run 投影、匹配、全局优化，再硬切渲染成米制墙面母版，
并在渲染完成后用诊断墙真值做后验检查。

本包没有 ROS 图依赖，`package.xml` 也不依赖任何项目包。构建母版的全过程**不读 Gazebo
真值、也不读墙面 DDS**：唯一的图像输入是已完成、已独立校验的 processed-run。

## 命令一览

| 命令 | 输入 | 主要输出 |
| --- | --- | --- |
| `validate_mosaic_inputs` | processed-run | 只打印严格 JSON 预检结果，**不创建目录** |
| `build_initial_projection` | processed-run | `initial_projection.json`、`initial_footprints_preview.png` |
| `build_overlap_candidates` | processed-run | `overlap_candidates.json` |
| `build_local_matches` | processed-run | `local_matches.json` |
| `build_pose_graph` | processed-run + 局部匹配 | `pose_graph.json` |
| `build_wall_mosaic` | processed-run + 位姿图 | 两张母版、差分、覆盖、不确定度、预览、manifest |
| `evaluate_diagnostic_mosaic` | 母版 + 墙面 manifest | `diagnostic_truth_summary.json` |
| `inspect_diagnostic_mosaic` | 母版 + 墙面 manifest | `diagnostic_inspection_summary.json`、原尺寸检查 tile |

所有输入/输出目录必须是绝对路径；输入只读，输出目录必须不存在且原子发布。
`--input-run` 可重复给出，把横向和竖向两次采集一起送进同一次拼接。
跨 run 的帧身份是 `(source_run_id, frame_index)`，不能只用文件名。

## 正常顺序

可复制的逐步命令见 [README 的离线墙面拼接](../../README.md#5-离线墙面拼接)，这里只说每一步
在做什么、为什么这么做。

**0. `validate_mosaic_inputs`** — 赶在昂贵的特征提取开始之前，逐项校验每张 processed PNG
的 SHA-256、每条处理标签、去畸变标定、冻结的安装快照，以及曝光位姿和协方差是否都是有限
数值（不含 NaN／Inf）。第一版只支持平面墙，因此所有输入 run 必须共享同一个去畸变相机
模型和同一份安装快照。

**1. `build_overlap_candidates`** — 先用扫描线索引缩小范围，再对凸足迹做精确裁剪，找出
可能拍到同一块墙的照片对，避免把所有照片两两比一遍。只有相交面积为正的候选才会进入视觉
匹配，仅仅**边缘相切**的足迹被排除。`--min-overlap-area-m2` 默认就是 `0.0`，也就是除了
“面积必须为正”之外不再设任何门限——不在默认值里藏一个没人量过的接受阈值。

它写出的 `overlap_candidates.json` 是给人查的候选图记录：下一步的 `build_local_matches`
会用同一套代码自己重算一遍候选，并不读这个文件。

**2. `build_local_matches`** — 在预测重叠区域做特征匹配、双向一致性、RANSAC 和固定尺度
SE(2) 估计。`--ratio-test`（`0.75`）、`--ransac-threshold-m`（`5 mm`）和
`--minimum-mutual-matches`（`4`）是接受判据；`--use-clahe` 用于弱纹理。
**接受和拒绝都要可追溯**——被拒的边和拒绝原因一并持久化。`--work-dir` 是可重建的特征缓存。

**3. `build_pose_graph`** — 以归档 `6×6` 协方差里显式提取出的 `x/y/yaw` 子空间为绝对先验，
配合视觉相对约束做鲁棒稀疏优化；优化变量只有 `δx, δy, δyaw`。EKF 初始位姿和优化后位姿都
会写出来，前者不可改写。**孤立帧保留自己的先验并被明确标出**，不伪装成视觉恢复的结果；
不连通的视觉分量也在质量报告里看得见。

优化中途会按数据本身复核一遍边，剔除的阈值取 `--edge-recheck-floor-m` 与
`median + 6·1.4826·MAD` 里的较大者——这个参数是那个阈值的下限，防止残差分布本身很窄时
把正常的边也一并剔掉。

**4. `build_wall_mosaic`** — 分块渲染 pose-only 与 optimized 两版母版，同时输出差分、
覆盖次数、不确定度和预览。渲染器内存有界，单写入器，产物原子发布；`--work-dir` 是可重建的
临时 tile 缓存。两版母版之间的比较规则见下节。

## 公平比较

pose-only 与 optimized 使用**同一批帧、同一网格、同一分辨率、同一插值和同一套硬切像素归属**，
唯一变量是位姿图修正。硬切规则：每个墙面像素只保留源图边缘距离最大的那张照片，平局取稳定
帧序较早者。这里不做融合平均——平均会把几何误差抹成一片模糊，看上去更“好”，实际上更没法
量测。

`<output-dir>` 里：

| 文件 | 含义 |
| --- | --- |
| `mosaic_pose_only.tif` | 直接按曝光位姿拼出的母版 |
| `mosaic_optimized.tif` | 经位姿图对齐后拼出的母版 |
| `mosaic_difference.tif` | 两者差分 |
| `coverage_count.tif` | 每个像素被几张照片覆盖；**为零表示相机从未拍到** |
| `uncertainty.tif` | 量化后验不确定度，`65535` 标记无覆盖 |
| `mosaic_preview.jpg` | 可比较的预览，**不能用于测量或缺陷判断** |
| `mosaic_manifest.json` | 哈希、共享网格、重叠分歧、每一遍的耗时、缓存统计和进程树 PSS 采样 |

正式母版是 `0.25 mm/px` 的无损 tiled BigTIFF（`--resolution-mm-per-pixel`）。
`--jobs` 和 `--memory-budget-gb` 只改变耗时：帧键、随机源、候选与边的排序、接受的边、
优化问题和输出像素必须保持确定。

## 后验评价

`evaluate_diagnostic_mosaic` 和 `inspect_diagnostic_mosaic` 是**渲染完成之后**才运行的
独立评价器，不参与候选生成、匹配、优化或渲染决策。只有它们可以读诊断墙 DDS 与 feature 真值。

- `evaluate_diagnostic_mosaic`：量测绝对锚点位置、尺度、航向和局部残差；
  `--anchor-padding-m`、`--minimum-phase-response` 控制锚点提取。
- `inspect_diagnostic_mosaic`：为每个与拼接域相交的 feature 导出**不缩放的**原尺寸
  truth / pose-only / optimized 检查 tile（`--tile-size-px`、`--padding-m`），
  并把零覆盖区域显式写进摘要，而不是用填充图掩盖。

原尺寸 tile 只能证明“已经导出”，不能证明相机拍到了它的每一个像素——这正是 P2-06 至今
未关闭的原因。

## 失败即不发布

任何预检失败、worker 缺失、缓存损坏或写入失败都不得产生 completed 结果。
`--work-dir`、特征缓存和临时 tile 是可再生的中间物，不是结果证据。

## 测试

```bash
colcon test --packages-select climbot_mosaic
colcon test-result --verbose
```

## 相关文档

上游是 [climbot_image_processing](../climbot_image_processing/README.md)。
设计合同与未关闭门禁见 [拼接计划](../../docs/MOSAIC_PLAN.md)，
接口与目录字段见 [接口合同](../../docs/INTERFACES.md)，
前置检查与故障处置见 [实验与故障处置手册](../../docs/OPERATION.md)，
各命令的完整参数以 `--help` 为准。
