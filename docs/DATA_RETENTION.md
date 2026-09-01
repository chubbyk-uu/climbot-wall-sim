# 数据保留与清理清单

更新：2026-09-01。本文定义当前保留边界；**不授权未来删除**。所有已清理数据均视为永久
删除，不把废纸篓作为保留层或恢复方案。

## 当前占用

| 位置 | 实测大小 | 性质 |
| --- | ---: | --- |
| `${CLIMBOT_DATA_ROOT}` | 约 12 GB，29 个顶层目录 | 原始采集、当前 P2-06 链、标定和审计材料 |
| `textures/` | 约 1.9 GB | Git 忽略、可重建但成本较高的 DDS 墙面贴图 |
| `results/` | 约 48 MB | Git 跟踪的正式摘要和证据 |
| `build/`、`install/`、`log/` | 约 1.1 GB | 可由构建/测试重新生成，不是项目证据 |

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

## 当前可清理项

- `/tmp/climbot_*` 的诊断日志和 A/B 产物：确认不再人工查看后可直接永久删除；
- `build/`、`install/`、`log/`、`.pytest_cache/`：均可重建，但清理后需要重新构建或失去本地
  测试日志；
- 上述三套历史原始输入：只有在明确放弃历史重建能力后才可删除，不能与普通缓存一起清理。

当前 29 个数据根目录均已逐项归类，没有未确认的大型缓存目录。后续新增正式采集时，先记录
run ID、上游/下游哈希和用途，再决定是否替换旧链。

## 已执行清理摘要

2026-08-27 至 2026-09-01 已永久清理开发期重复拼接、P2.7b/P2.7d 与 camera-zfix 派生产物、
失败/取消采集、P2-06 第 2–4 组可重建派生产物、空工作目录和无引用旧标定。最后一次对 75 个
顶层目录逐项核对后删除 46 个，数据根收敛为 29 个、约 12 GB；原先位于数据根之外的审计目录
已并入 `${CLIMBOT_DATA_ROOT}/acceptance_evidence`。

已删除内容不可恢复；需要复查时只能依赖保留的原始输入、Git 历史和正式摘要重新计算。详细的
逐路径删除记录留在 Git 历史，不在当前保留清单重复维护。
