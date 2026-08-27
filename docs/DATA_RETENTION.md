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

## 已清理：第二批 P2.7b 开发数据

2026-08-27 已将不干净源码树产生的 P2.7b raw、processed 及全部 `mosaic-p27b-*` 产品移入
系统废纸篓，共约 `4.4 GB`。P2.7d 洁净树盲测已替代其作为当前证据；同名任务 YAML 保留，
可用新输入重新运行。清空系统废纸篓前仍可恢复。

## 尚未分类，不自动清理

早期 2026-08 原始采集、处理结果和单向拼接虽有重复风险，但部分仍被当前基线或历史结果说明
引用。特别是 `camera-zfix` 与非修正数据、以及横/竖单向最终母版，尚未完成逐文件哈希和证据
映射；在建立“当前基线 / 历史对照 / 可删除”索引前不自动删除。所有后续清理都必须先记录精确
目录、大小、引用检查和保留理由，再由操作者确认。
