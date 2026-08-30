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
| `preflight_diagnostic_coverage` | 任务 YAML + 相机/机器人/墙配置 + 墙面 manifest | 采集前的离散曝光 feature 覆盖 JSON |

所有输入/输出目录必须是绝对路径；输入只读，输出目录必须不存在且原子发布。
`--input-run` 可重复给出，把横向和竖向两次采集一起送进同一次拼接。
跨 run 的帧身份是 `(source_run_id, frame_index)`，不能只用文件名。

`preflight_diagnostic_coverage` 是 P2-06 的**采集前**工具，不参与任何拼接候选、匹配、
位姿优化或渲染。它精确复现 `automatic_capture_node` 的首张、计数和间隔合同，而不是把 SCAN
当作连续曝光；可重复给出横、竖任务以核对联合覆盖。它只以墙面 manifest 的 declared feature
几何作离线计划验证，完成采集后仍必须由 `coverage_count.tif` 和原尺寸检查重新证明实际覆盖。

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
| `mosaic_difference.tif` | 两者差分；由 optimized 那趟逐块相减得出，不重复渲染 |
| `coverage_count.tif` | 每个像素被几张照片覆盖；**为零表示相机从未拍到** |
| `uncertainty.tif` | 量化后验不确定度，`65535` 标记无覆盖 |
| `mosaic_preview.jpg` | optimized 母版的 JPEG 预览 |
| `mosaic_comparison.jpg` | pose-only、optimized 和差分的并排 JPEG；默认每个 panel 最长边 4096 px |
| `mosaic_manifest.json` | 哈希、共享网格、重叠分歧、每一遍的耗时、缓存统计和进程树 PSS 采样 |

正式母版是 `0.25 mm/px` 的无损 tiled BigTIFF（`--resolution-mm-per-pixel`）。
`--jobs` 和 `--memory-budget-gb` 只改变耗时：帧键、随机源、候选与边的排序、接受的边、
优化问题和输出像素必须保持确定。渲染进程数取三者最小——显式 `--jobs`、内存预算能负担的
数量、CPU 核数。预算先付一次运行的固定开销（渲染网格、帧表和 TIFF 写入器缓冲，实测
`1.87 GB`），余下的按每 worker `96 MB` 分配；预算不足以覆盖固定开销时退化为单进程。

`--preview-max-side-px` 只改变两份 JPEG 预览的尺寸，不会改变正式 BigTIFF 或位姿图。默认 `4096`
便于查看接缝和局部细节；想减小文件可设为 `2048`，最小值为 `512`。

## 后验评价

`evaluate_diagnostic_mosaic` 和 `inspect_diagnostic_mosaic` 是**渲染完成之后**才运行的
独立评价器，不参与候选生成、匹配、优化或渲染决策。只有它们可以读诊断墙 DDS 与 feature 真值。

- `evaluate_diagnostic_mosaic`：量测绝对锚点位置、尺度、航向和局部残差；
  `--anchor-padding-m`、`--minimum-phase-response` 控制锚点提取。
- `inspect_diagnostic_mosaic`：为每个与拼接域相交的 feature 导出**不缩放的**原尺寸
  truth / pose-only / optimized 检查 tile（`--tile-size-px`、`--padding-m`），
  并把零覆盖区域显式写进摘要，而不是用填充图掩盖。

`--drive-region-m MIN_X MIN_Y MAX_X MAX_Y` 给出机器人被允许行驶的矩形，摘要据此把未覆盖
像素拆成 `uncovered_inside_drive_region` 与 `uncovered_outside_drive_region`，并给出
`all_reachable_feature_pixels_covered`。判定覆盖时必须给：declared 几何可以伸到机器人到
不了的地方——这块诊断墙上就有按设计贯穿整墙的接缝——不裁剪可达范围的总数对这类墙面恒为
假，没有判定价值。拆分由评价器输出而不是事后另算，缺口才不会无声增长。

原尺寸 tile 只能证明“已经导出”，不能证明相机拍到了它的每一个像素。

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
