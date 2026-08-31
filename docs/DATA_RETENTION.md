# 数据保留与清理清单

更新：2026-08-31（第二次）。此清单记录当前保留边界和已经执行的清理；**本文本身不授权未来删除**。

## 当前占用

| 位置 | 大小 | 说明 |
| --- | ---: | --- |
| `${CLIMBOT_DATA_ROOT}` | 约 21 GB | 原始归档、处理结果、拼接产品与开发中间产物；2026-08-31 实测，含新增的第 2、3 组 |
| `textures/` | 约 1.9 GB | 忽略的 DDS 墙面贴图；可由脚本重建，但重建成本高 |
| `results/` | 约 48 MB | Git 跟踪的正式摘要和轨迹证据 |
| `log/` | 约 834 MB | 可再生的本机构建/测试日志 |
| `build/` / `install/` | 约 379 MB | 可再生的本地构建产物 |

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

当前 P2-06 严格证据链还必须保留：

- `inspection-diagnostic-full-{horizontal,vertical}-025mm-20260828` 下摘要记录的两份原始归档；
- `processed-p206-{horizontal,vertical}-20260830b`；后缀虽为 `b`，它们仍是当前链的输入；
- `mosaic-p206-joint-20260830c-{candidates,matches,pose-graph}`；其中 candidates 是独立诊断，
  matches 与 pose-graph 是当前链的冻结上游；
- 当前正式链的 `mosaic-p206-joint-20260831b-hardcut`、
  `mosaic-p206-joint-20260831c-hardcut-truth` 与
  `mosaic-p206-joint-20260831b-hardcut-inspection`；hardcut 含两份稀疏硬切接缝和两份覆盖图，
  inspection 含严格几何分类和 170 个逐文件哈希原尺寸 tile；
- 完整的 `20260830c-hardcut`（约 1.3 GB）、`20260830c-hardcut-truth` 和
  `20260830d-hardcut-inspection`（约 492 MB），作为新增接缝记录前的逐位复现审计材料；
- 完整的 `20260831a-{hardcut,hardcut-truth,hardcut-inspection}` 与
  `20260831b-hardcut-truth`，作为 schema 2 证据和现行 schema 3 流式真值评价之间的审计材料；
- P2-06 门限推导所依据的第 2、3 组重复采集，每组约 3.4 GB：
  `inspection-diagnostic-full-{horizontal,vertical}-025mm-20260828` 下 `20260831T0721`、
  `20260831T0746`（第 2 组）与 `20260831T0807`、`20260831T0838`（第 3 组）四个 run；
  `processed-p206-{horizontal,vertical}-20260831{g2,g3}`；
  `mosaic-p206-joint-20260831{g2,g3}-{matches,pose-graph,hardcut,hardcut-truth,hardcut-inspection}`。
  门限由这三组共同推导，任何一组删除都会使推导无法复核；
- `archive-content-p206-{horizontal,vertical}-20260831`（共约 770 KB）：三组采集的图像内容
  记录与判定，是"三组内容一致"这一说法的唯一可复核依据；
- 旧 `20260830b` 的 `mosaic_manifest.json`、truth/inspection summary 以及 matches JSON，和旧
  `20260830c-hardcut-inspection` 的 summary/provenance，作为撤销与逐位复现审计材料。

`mosaic-p206-joint-20260831a-work-fusion` 是空目录，已在本次核对中移除；其余上述完整目录均为
有意保留，不能从“只在正式摘要中引用 summary/provenance”推断为可删除。
`mosaic-p206-joint-20260831{g2,g3}-work-{matches,fusion}` 是可随时重建的缓存，共约 98 MB。

2026-08-31 另有一次作废清理：第一次重采的两组（四个 raw 归档、两套 processed、匹配缓存，
共约 5.0 GB）因 `wall_visual` 与贴图块共面导致整幅图 z-fighting 而不可用，已移入系统废纸篓
（可恢复）。同批还有一个因仿真渲染卡顿被采集守卫拒收的残缺 run（341/660，280 MB）一并移入。
根因与修复见提交 `f849af1`。

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

## 已清理：第三批早期非修正数据

2026-08-27 已将早期非 `camera-zfix` 横/竖 raw 与 processed 数据，以及一份未被任何
processed-run 引用的失败归档移入系统废纸篓，共约 `866 MB`。当前 P2 相对基线使用的是
`camera-zfix` 数据和横/竖/联合最终母版，未受影响。

## 尚未分类，不自动清理

早期 2026-08 原始采集、处理结果和单向拼接虽有重复风险，但部分仍被当前基线或历史结果说明
引用。特别是 `camera-zfix` 与非修正数据、以及横/竖单向最终母版，尚未完成逐文件哈希和证据
映射；在建立“当前基线 / 历史对照 / 可删除”索引前不自动删除。所有后续清理都必须先记录精确
目录、大小、引用检查和保留理由，再由操作者确认。

## 已清理：2026-08-28 长跑测试采集

定位、机动边界、GUI/RViz 和相机 trigger A/B 共生成 14 个 `${CLIMBOT_DATA_ROOT}` 顶层测试
目录及 21 个 `/tmp/climbot_g2_*` 临时归档，合计约 `6.4 GB`。逐项确认它们未被当前 P2.7d、
8 月 26 日相对基线或 `results/` 正式证据引用后，已移入系统废纸篓；本轮诊断日志保留。
上述“必须保留”目录均未移动。清空系统废纸篓前这些测试图片仍可恢复，也仍占用宿主磁盘空间。

## 已清理：2026-08-30 P2-06 重复产物

严格证据发布后逐文件确认：`20260830b` 与当前 `20260830c` 的七个拼接产物哈希完全相同，
旧 `b/c` inspection 的 170 个原尺寸 tile 也与当前 `20260830d` 逐文件相同。随后永久删除旧
`b` 拼接大图、`b/c` 旧 inspection tiles、重复的 `b` candidates/pose-graph，以及 `b/c`
work 缓存，共 `2,425,317,078 bytes`（约 2.26 GiB）。旧 manifest、truth/inspection summary、
`b` matches 和 `c` inspection provenance 按上面的审计边界保留；当前 raw、processed、
`20260830c` 正式链及 `20260830d` inspection 均未删除。本次使用精确路径永久删除，不在废纸篓，
不能恢复；可由保留的输入重新计算。
