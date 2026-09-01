# 数据保留与清理清单

更新：2026-09-01。此清单记录当前保留边界和已经执行的清理；**本文本身不授权未来删除**。

## 当前占用

| 位置 | 大小 | 说明 |
| --- | ---: | --- |
| `${CLIMBOT_DATA_ROOT}` | 约 12 GB | 当前 P2-06 链、三套历史原始输入、标定与小型审计证据；2026-09-01 最终清理后实测 |
| `textures/` | 约 1.9 GB | 忽略的 DDS 墙面贴图；可由脚本重建，但重建成本高 |
| `results/` | 约 48 MB | Git 跟踪的正式摘要和轨迹证据 |
| `log/` | 约 834 MB | 可再生的本机构建/测试日志 |
| `build/` / `install/` | 约 379 MB | 可再生的本地构建产物 |

## 必须保留

这些目录是当前 P2-06 证据链或有意保留的重建输入：

- `${CLIMBOT_DATA_ROOT}/calibration`；
- `textures/wall`、`textures/wall_025`、`textures/wall_diagnostic_025`；
- `${CLIMBOT_DATA_ROOT}/p27d_blind_20260827`；
- `results/` 的已跟踪文件；
- 2026-08-26 camera-zfix 横、竖两份原始归档。其 processed 和拼接结果已由后续绝对真值
  P2-06 取代并删除，必要时可从原始归档和 Git 历史重建。

当前 P2-06 严格证据链还必须保留：

- `inspection-diagnostic-full-{horizontal,vertical}-025mm-20260828` 下摘要记录的两份原始归档；
- `processed-p206-{horizontal,vertical}-20260830b`；后缀虽为 `b`，它们仍是当前链的输入；
- `mosaic-p206-joint-20260830c-{candidates,matches,pose-graph}`；其中 candidates 是独立诊断，
  matches 与 pose-graph 是当前链的冻结上游；
- 当前正式链的 `mosaic-p206-joint-20260831b-hardcut`、
  `mosaic-p206-joint-20260831c-hardcut-truth` 与
  `mosaic-p206-joint-20260831b-hardcut-inspection`；hardcut 含两份稀疏硬切接缝和两份覆盖图，
  inspection 含严格几何分类和 170 个逐文件哈希原尺寸 tile；
- P2-06 门限推导所依据的第 2、3 组重复采集原始归档：
  `inspection-diagnostic-full-{horizontal,vertical}-025mm-20260828` 下 `20260831T0721`、
  `20260831T0746`（第 2 组）与 `20260831T0807`、`20260831T0838`（第 3 组）四个 run。
  对应 processed 与 mosaic 已在 P2-06 关闭后删除；可由这些原始归档、保留的旧平场和 Git
  历史重建，正式数值与哈希保留在 `results/`；
- `archive-content-p206-{horizontal,vertical}-20260831` 与
  `archive-content-p206-{horizontal,vertical}-20260831g4`（共约 1.2 MB）：四组采集的图像内容
  记录与判定，是"各组内容一致"这一说法的唯一可复核依据；
- 第 4 组（`20260831T1215`、`20260831T1234` 两个 run）：它带完整渲染记录并通过内容闸门，
  是门限推导的三组之一；processed 与 mosaic 与第 2、3 组同样已删除并可由原始归档重建；
- 第 5 组（`20260831T1309`、`20260831T1354` 两个 run，`processed-p206-*-20260831g5`、
  `mosaic-p206-joint-20260831g5-*` 与 `archive-content-p206-*-20260831g5`，约 3.4 GB）：
  门限的唯一独立检验，删除会使"已检验"这个说法失去依据；
- 旧 `20260830b` 的 `mosaic_manifest.json`、truth/inspection summary 与 matches JSON，以及
  `${CLIMBOT_DATA_ROOT}/acceptance_evidence` 的小型 JSON 副本，作为撤销与逐位复现审计材料。

`mosaic-p206-joint-20260831a-work-fusion` 是空目录，已在本次核对中移除；其余上述完整目录均为
有意保留，不能从“只在正式摘要中引用 summary/provenance”推断为可删除。
`mosaic-p206-joint-20260831{g2,g3,g4,g5}-work-{matches,fusion}` 是可随时重建的缓存；已于
2026-08-31 移入系统废纸篓，共约 196 MB。同批移入的还有 `processed_inspection_horizontal_20260826`
（179 MB），它不被任何已发布结果或文档引用，是 P2.7d 之前的早期处理产物。

2026-08-31 另有一次作废清理：第一次重采的两组（四个 raw 归档、两套 processed、匹配缓存，
共约 5.0 GB）因 `wall_visual` 与贴图块共面导致整幅图 z-fighting 而不可用，已移入系统废纸篓
（可恢复）。同批还有两个被采集守卫拒收的残缺 run（341/660 共 280 MB，536/660 共 441 MB）一并移入；停顿签名见 `Slow capture … source pair ≈1.91 s`。当时归因为「仿真渲染卡顿」，2026-09-01 查明并非渲染，而是 Fast DDS 共享内存段装不下整帧后的分片丢失，见 [STATUS](STATUS.md) 第 10 条。
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
系统废纸篓，共约 `4.4 GB`。P2.7d 洁净树盲测已替代其作为当时证据；同名任务 YAML 保留，
可用新输入重新运行。清空系统废纸篓前仍可恢复。

## 已清理：第三批早期非修正数据

2026-08-27 已将早期非 `camera-zfix` 横/竖 raw 与 processed 数据，以及一份未被任何
processed-run 引用的失败归档移入系统废纸篓，共约 `866 MB`。当时的 P2 相对基线使用
camera-zfix 数据；其派生产物后来在绝对真值 P2-06 关闭后清理，只保留两份原始输入。

## 历史原始输入

为避免所有历史阶段都只剩摘要，当前保留三套可重建输入：`p27d_blind_20260827` 与 2026-08-26
camera-zfix 横、竖原始归档，共约 `879 MB`。对应派生产物已删除；它们不是当前 P2-06 产品，
以后若决定放弃历史重建能力，可再单独清理。

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

## 已清理：2026-09-01 P2-06 重复组派生产物

P2-06 关闭后永久删除第 2、3、4 组的六个 `processed-p206-*` 和十五个
`mosaic-p206-joint-20260831g{2,3,4}-*` 目录，共约 `8.3 GB`。六份原始归档、旧 moonlight
平场、`archive-content-*`、Git 中的正式摘要和第 5 组独立检验完整链均保留，因此仍可重建，
但第 2～4 组不再支持免计算直接浏览。无引用的
`flat_field_sim_led2_exp065_20260825.npz` 同时删除；被历史处理清单引用的 moonlight 平场和当前
PBR 平场保留。

home 下独立的 `climbot_acceptance_evidence` 仅有约 `344 KB` JSON 审计副本，不是大数据来源；
已整体移入 `${CLIMBOT_DATA_ROOT}/acceptance_evidence`，内容不变，避免数据根之外再维护一个目录。

## 已清理：2026-09-01 历史派生产物收敛

逐项核对全部 75 个顶层目录后，永久删除另外 46 个目录，共约 `8.3 GB`：取消／失败的
`rviz-selection` 试采、2026-08-25 非 camera-zfix 原始数据、P2.7d 与 2026-08-26 camera-zfix
的 processed/匹配/位姿图/拼接产品，以及已被当前 `31b/31c` 链和小型审计摘要替代的
`20260830c/30d/31a` 大图与原尺寸 tile。数据根最终为 29 个顶层目录、约 `12 GB`；当前
P2-06 和 g5 独立检验完整链未动，P2.7d 与 camera-zfix 各保留原始输入。
