# 数据保留与清理清单

更新：2026-08-27。此清单是清理前的只读盘点；**本文不授权删除任何目录**。

## 当前占用

| 位置 | 大小 | 说明 |
| --- | ---: | --- |
| `${CLIMBOT_DATA_ROOT}` | 约 11 GB | 原始归档、处理结果、拼接产品与开发中间产物 |
| `textures/` | 约 1.9 GB | 忽略的 DDS 墙面贴图；可由脚本重建，但重建成本高 |
| `results/` | 约 47 MB | Git 跟踪的正式摘要和轨迹证据 |
| `log/` | 约 301 MB | 可再生的本机构建/测试日志 |
| `build/` / `install/` | 约 233 MB | 可再生的本地构建产物 |

## 必须保留

这些目录是当前 P2 证据链或重建环境的一部分，在 P2 最终门限冻结前不得删除：

- `${CLIMBOT_DATA_ROOT}/calibration`；
- `textures/wall`、`textures/wall_025`、`textures/wall_diagnostic_025`；
- `${CLIMBOT_DATA_ROOT}/p27d_blind_20260827`；
- `${CLIMBOT_DATA_ROOT}/processed-p27d-blind-horizontal-20260827` 与
  `processed-p27d-blind-vertical-20260827`；
- 三个 `${CLIMBOT_DATA_ROOT}/mosaic-p27d-hardcut-{horizontal,vertical,joint}-20260827`；
- 三个对应的 `mosaic-p27d-hardcut-*-truth-20260827`，以及
  `mosaic-p27d-hardcut-joint-inspection-v2-20260827`；
- `results/` 的已跟踪文件；
- 2026-08-26 当前正式相对基线的原始/processed 输入及
  `mosaic-horizontal-vertical-camera-zfix-traceable-final-025mm-20260826`。

## 已清理：第一批重复与缓存

2026-08-27 已逐项核对后，将以下 16 个目录移入系统废纸篓：五份与保留
`traceable-final` 母版的 pose-only/optimized SHA-256 相同的联合产品、一份已被 `-v2`
替代的 P2.7d 检查包、五份 2 mm 调试预览和五份 P2 工作缓存，共约 `5.0 GB`。它们已不在
`${CLIMBOT_DATA_ROOT}` 下，但在清空系统废纸篓前仍可恢复。

`build/`、`install/`、`log/` 与 `.pytest_cache/` 仍是可再生产物；清理会要求完整重建，暂不纳入
数据根清理。

## 第二批候选：已被洁净树盲测替代的开发数据

P2.7b 由脏工作树产生，只作为评价器开发证据，已被 P2.7d 洁净树盲测替代。确认不再需要
逐轮调试图后，可归档其小型 JSON 摘要并清理以下 raw、processed 与 product 目录，预计约
4.1 GB：

- `inspection-diagnostic-realistic-{horizontal,vertical}-025mm-20260827`；
- `processed-inspection-diagnostic-realistic-{horizontal,vertical}-025mm-20260827`；
- 所有前缀为 `mosaic-p27b-` 的目录及其 truth/work/candidate/match/pose-graph 子产品。

## 尚未分类，不自动清理

早期 2026-08 原始采集、处理结果和单向拼接虽有重复风险，但部分仍被历史结果说明引用。
在为其建立“当前基线 / 历史对照 / 可删除”索引前，不自动删除。所有清理操作应先将精确
目录、大小、引用检查结果和保留理由写入一次性清理记录，再由操作者确认。
