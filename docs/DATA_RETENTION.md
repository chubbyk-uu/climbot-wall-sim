# 数据保留与清理清单

更新：2026-08-27。此清单是清理前的只读盘点；**本文不授权删除任何目录**。

## 当前占用

| 位置 | 大小 | 说明 |
| --- | ---: | --- |
| `/home/jerry/climbot_data` | 约 16 GB | 原始归档、处理结果、拼接产品与开发中间产物 |
| `textures/` | 约 1.9 GB | 忽略的 DDS 墙面贴图；可由脚本重建，但重建成本高 |
| `results/` | 约 47 MB | Git 跟踪的正式摘要和轨迹证据 |
| `log/` | 约 301 MB | 可再生的本机构建/测试日志 |
| `build/` / `install/` | 约 233 MB | 可再生的本地构建产物 |

## 必须保留

这些目录是当前 P2 证据链或重建环境的一部分，在 P2 最终门限冻结前不得删除：

- `/home/jerry/climbot_data/calibration`；
- `textures/wall`、`textures/wall_025`、`textures/wall_diagnostic_025`；
- `/home/jerry/climbot_data/p27d_blind_20260827`；
- `/home/jerry/climbot_data/processed-p27d-blind-horizontal-20260827` 与
  `processed-p27d-blind-vertical-20260827`；
- 三个 `/home/jerry/climbot_data/mosaic-p27d-hardcut-{horizontal,vertical,joint}-20260827`；
- 三个对应的 `mosaic-p27d-hardcut-*-truth-20260827`，以及
  `mosaic-p27d-hardcut-joint-inspection-v2-20260827`；
- `results/` 的已跟踪文件；
- 2026-08-26 当前正式相对基线的原始/processed 输入及
  `mosaic-horizontal-vertical-camera-zfix-traceable-final-025mm-20260826`。

## 第一批候选：可再生或已确认重复

执行前仍须逐项核对路径、哈希和文档引用；预计可释放约 5.8 GB。

| 组别 | 目录规则 | 预计大小 | 依据 |
| --- | --- | ---: | --- |
| P2.7d 旧检查包 | `mosaic-p27d-hardcut-joint-inspection-20260827` | 297 MB | `-v2` 是当前被结果摘要引用的版本 |
| 联合拼接重复产品 | `mosaic-horizontal-vertical-camera-zfix-025mm-20260826`，以及同前缀的 `bounded-final`、`final`、`pss-final`、`u16` 产品 | 约 4.6 GB | 与保留的 `traceable-final` 拥有相同 optimized 产品 SHA-256 |
| 2 mm 调试预览 | `mosaic-preview-horizontal-vertical-camera-zfix-*` | 约 146 MB | 可由保留输入重新生成，非正式证据 |
| P2 工作缓存 | `mosaic-work-*`、`mosaic-*-work-fusion-*` | 约 96 MB | 临时 feature/tile cache，不是发布证据 |
| 本地构建日志 | `log/`、`.pytest_cache/` | 约 301 MB | 常规可再生产物 |

`build/` 和 `install/` 也可清理，但会要求完整重建，默认不纳入第一批。

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
