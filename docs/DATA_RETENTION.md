# 数据保留与清理清单

更新：2026-10-18。本文定义当前保留边界；**不授权未来删除**。所有已清理数据均视为永久
删除，不把废纸篓作为保留层或恢复方案。

## Git 与本地数据的边界

远端仅保留代码、文档、精简摘要和用户批准的 README 两张展示图。完整实验数据与其它验收
截图／视频留在本机，发布裁剪不等同于删除本地数据。旧未发布历史完整保留在本地归档分支；
远端使用干净提交，避免把已提交后又删除的大文件一并上传。详见[发布范围](../results/PUBLICATION.md)。

## 历史占用快照（2026-09-05）

| 位置 | 实测大小 | 性质 |
| --- | ---: | --- |
| `${CLIMBOT_DATA_ROOT}` | 约 13 GB，30 个顶层目录 | 原始采集、当前 P2-06 链、标定、审计材料及介绍视频 |
| `textures/` | 约 1.9 GB | Git 忽略、可重建但成本较高的 DDS 墙面贴图 |
| `results/` | 约 48 MB | Git 跟踪的正式摘要和证据 |
| `build/`、`install/` | 约 198 MB | 可重建；历史 `log/` 已清理 |

占用为 `du -h` 的取整显示；视频目录计入数据根总量，不另行相加。

## 必须保留

### 当前 P2-06 正式链

- 原始采集：`inspection-diagnostic-full-{horizontal,vertical}-025mm-20260828`；其中包含第
  1–5 组共 10 个 run。
- 第 1 组处理输入：`processed-p206-{horizontal,vertical}-20260830b`。
- 冻结上游：`mosaic-p206-joint-20260830c-{candidates,matches,pose-graph}`。
- 当前母版与检查：`mosaic-p206-joint-20260831b-hardcut`、
  `mosaic-p206-joint-20260831c-hardcut-truth`、
  `mosaic-p206-joint-20260831b-hardcut-inspection`。
- 第 5 组独立检验完整链：`processed-p206-*-20260831g5` 及
  `mosaic-p206-joint-20260831g5-*`。
- 内容与审计记录：`archive-content-p206-{horizontal,vertical}-20260831`、
  `acceptance_evidence`，以及保留的小型 `20260830b-{hardcut,hardcut-truth,
  hardcut-inspection,matches}` 和 `20260831b-hardcut-truth` 审计目录。

第 2–4 组的 processed 和 mosaic 已删除；原始 run、内容摘要和 Git 中的正式结果仍在，必要时
可由对应历史提交重建。第 1 组和第 5 组保留完整链，分别承担可直接复查基线和独立检验。

### 标定、历史输入和仓库证据

- `calibration/flat_field_sim_pbr_exp065_20260901.npz`：以后新仿真数据的当前平场；
- 旧 moonlight 平场：仍被历史 processing manifest 引用，不能删除；
- `p27d_blind_20260827` 与 2026-08-26 camera-zfix 横/竖原始归档：三套历史重建输入，约
  879 MB；其派生产物已删除；
- `results/` 的全部已跟踪文件，以及 `textures/wall*` 当前贴图资产。

### 项目介绍视频

- `video_introduction/climbot_intro_zh_voiced_v3.mp4`：当前发布成片。
- 保留 `raw/` 原始录屏、`voice/` 配音缓存、`v3/` 字幕、时间轴、来源记录和渲染素材，
  以及专用 `venv/`；制作脚本与说明位于仓库 `media/introduction/`。
- v1/v2 旧成片、渲染缓存及 v3 无声中间片已删除；旧版字幕和时间轴仍保留。
  重建中间片需要重新渲染/合成，不能再直接打开旧成片比较。

## S6 本轮临时数据的清理授权

用户于 2026-10-18 明确要求“验收通过后临时采集的数据要清理”。本轮验收完成后，
清理 S6 产生的诊断／失败采集、重复副本及无正式证据引用的中间产物；执行前记录相对路径、
用途、大小及所需摘要／哈希，执行后核对路径消失和释放空间。本节落实本轮授权，不扩大到
已有历史正式链、共享 Mesa、标定、贴图、源码或其他项目数据。

正式 S6 报告与复现入口、唯一原始输入及最终成果按证据链归档；仍被证据引用的数据须先
完成必要归档，不能把验证所需输入提前当缓存删除。最终清理清单区分保留和实际删除，
不能只给出待清理建议。验收未完成前先保留工作输入。

## 当前可清理项

共享 Mesa 工具链不是数据缓存：`$HOME/opt/agv-mesa-25.2.8/install` 和相邻的
`build-result.json` 是 WSL D3D12 默认启动所需，不能随采集数据、`build/` 或 `install/` 清理。
其源码构建和恢复方法见 [MESA_SETUP](MESA_SETUP.md)。这里的工作区 `install/` 与共享库
前缀中的 `install/` 不是同一个目录。

- `/tmp/climbot_*` 的诊断日志和 A/B 产物：确认不再人工查看后可直接永久删除；
- `build/`、`install/`、`log/`、`.pytest_cache/`：均可重建，但清理后需要重新构建或失去本地
  测试日志；
- 上述三套历史原始输入：只有在明确放弃历史重建能力后才可删除，不能与普通缓存一起清理。

2026-09-05 快照中的 30 个数据根目录已逐项归类（含视频制作目录）。后续新增正式采集时，先记录
run ID、上游/下游哈希和用途，再决定是否替换旧链。

## S6 已执行归档与清理（2026-10-18）

S6 功能、双 GUI 性能、CUDA／人工图像复核及最终完整回归通过后，按本轮用户授权执行：

- 正式逐步压力／接触和可重建场景共 2633 项、2466823880 逻辑字节，已按 SHA256 归档至
  `acceptance_evidence/suction_s6_20261018/`，随后删除仓库工作区中的相同字节副本；
  另删除 `/tmp` 已核对的 16 份压力副本。部分归档为硬链接，逻辑字节不等于释放空间。
- 删除本轮 `feature-work`（1340 项、50361216 字节）及空 `cuda-work` 工作目录。
- 九个精确诊断目录先将 193 份时序／日志材料无损压缩归档并复核，再移除诊断工作根。
- 零照片的失败启动组及 544 张照片的中断求解器重验组已删除；JSON／YAML 元数据逐字节保留，
  临时 PNG 仅保留哈希清单，不能声称已删除的 PNG 仍可恢复。
- 最终旧／新横纵 2680 张原图、成功的历史参考组、共享标定、处理结果、CUDA 母版、
  170 个原尺寸检查 tile、21 个人工复核窗口及仓库已跟踪历史文件均保留。

清理期间 Linux 文件系统可用空间观测增加 1033715712 字节（约 0.96 GiB），不宣称
WSL 虚拟磁盘文件因此缩小。精确清单、保留位置和哈希见
[S6 最终报告](../results/suction_s6_2026-10-18/REPORT.md)、
工作副本清理（本地附件：`results/suction_s6_2026-10-18/CLEANUP.json`）、
[诊断归档](../results/suction_s6_2026-10-18/diagnostic_archive.json)和
失败采集清理（本地附件：`results/suction_s6_2026-10-18/failed_capture_cleanup.json`）。

## S6 追加图片精简授权（2026-10-18）

用户明确要求最终旧结构的 1340 张原图先保留，同意清理上一轮建议的其余项目。
本次已删除下列 4190 个 PNG（3888161320 逻辑字节，约 3.62 GiB），所有保护检查通过。
范围、哈希与执行结果见
[追加清理记录](../results/suction_retention_2026-10-18/REPORT.md)：

- `suction_s6_legacy_reference_20261018_r2` 和 `suction_s6_candidate_20261018_r2`：
  早期 R2 两组共 2680 张，删除照片；原始 manifest、曝光元数据和标定副本原样保留。
- `processed-suction-s6-final-{horizontal,vertical}-20261018`：1340 张可重建预处理 PNG；
  保留 processing manifest、逐帧标签和标定，重建时使用保留的新结构原图及共享平场。
- `mosaic-suction-s6-final-joint-20261018-inspection/native_tiles`：170 个可重建检查 tile；
  保留检查摘要和 `manual_review_rois` 下 21 个已复核的原像素样本。

最终旧／新两组原图各 1340 张（共 2680 张）、曝光信息、标定、完整 CUDA 母版／覆盖图、
匹配／位姿图、压力／接触时序和 S7 正式成果均保留。此授权不扩大到其它数据。
本节更新此前“S6 成功 R2、processed 和 170 个 tile 均保留”的阶段记录；历史验收门禁不改写。
像素被精简的目录保留独立 IMAGE_RETENTION 标记，原 producer manifest 不修改：
R2 原照片只能查哈希／元数据，不能恢复；processed／tile 需要重建后才能完整复查。

## S7 保留与限定清理（2026-10-18）

S7 中等任务及 CUDA 拼接验收通过后，按用户已授权的“验收后清理临时数据”执行。
保留 `suction_s7_medium_20261018` 的 70 张原图、曝光信息、标定、processed、matches、pose-graph、
完整母版／覆盖图，以及 acceptance_evidence 下的压力、生成场景和 S6 展开等价原件。

可删除范围仅限本轮 `*_feature-work`／`*_cuda-work` 缓存、已逐字节归档的 `/tmp` 压力／生成场景
工作副本，以及最初因 RViz 显示失败被替代的 `suction_s7_small_20261018` 的 10 张临时原图。
首轮 manifest／元数据须先保留并记录原图哈希；删除后只保留哈希，不声称可重建这些 PNG。
不删除其他任务、S6 正式数据、共享标定或 Mesa 修复库。实际清理结果见
[S7 报告](../results/suction_s7_2026-10-18/REPORT.md)与同目录 CLEANUP.json。

## 已执行清理摘要

2026-08-27 至 2026-09-01 已永久清理开发期重复拼接、P2.7b/P2.7d 与 camera-zfix 派生产物、
失败/取消采集、P2-06 第 2–4 组可重建派生产物、空工作目录和无引用旧标定。最后一次对 75 个
顶层目录逐项核对后删除 46 个，数据根收敛为 29 个、约 12 GB；原先位于数据根之外的审计目录
已并入 `${CLIMBOT_DATA_ROOT}/acceptance_evidence`。

2026-09-05 经确认直接删除项目 `log/`、用户 ROS 日志目录、视频 v1/v2 旧成片与渲染缓存、
v3 的 `picture.mp4` 无声中间片，合计约 2 GB，未放入废纸篓。正式数据、标定、贴图、
构建安装产物及视频 v3 成片、录屏和配音均未删除。

已删除内容不可恢复；需要复查时只能依赖保留的原始输入、Git 历史和正式摘要重新计算。详细的
逐路径删除记录留在 Git 历史，不在当前保留清单重复维护。


本页以反引号标注的本地验收附件不随 Git 发布，见[发布范围说明](../results/PUBLICATION.md)。
