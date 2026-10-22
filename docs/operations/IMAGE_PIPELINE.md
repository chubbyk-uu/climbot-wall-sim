# 图像处理、拼接与后验评价

前置环境见[操作入口](../OPERATION.md#运行前准备)。

## 采集内容核对

计数守卫（`expected == saved`、`failed=0`、`staged=0`）不看像素。把新采集与一次已知良好的
同计划采集比对，产出可引用的记录：

```bash
ros2 run climbot_mosaic summarize_archive_content \
  --input-run "$CLIMBOT_DATA_ROOT/<新采集 run 目录>" \
  --reference-run "$CLIMBOT_DATA_ROOT/<基准 run 目录>" \
  --output-dir "$CLIMBOT_DATA_ROOT/archive-content-<id>"
```

横向只能对横向、纵向只能对纵向；比对按同序帧对齐，所以帧数必须与参考一致，不一致直接拒绝。
判定不通过时退出码为 `3`，不要把它当成可以忽略的告警——两整组 z-fighting 数据就是在计数全绿的
情况下走到拼接阶段的。这些记录是正式摘要的强制输入：`build_mosaic_evidence_summary.py` 要求
每个冻结任务的归档都被某份记录覆盖并通过，缺一份就拒绝出摘要。

## 预处理与拼接的前置检查

逐步命令见 [README 的离线图像处理](../QUICKSTART.md#5-离线图像处理)和
[离线墙面拼接](../QUICKSTART.md#6-离线墙面拼接)。开始之前先确认三件事：

1. G4 manifest 的 `outcome` 为 `completed`；半个 run 的计数本来就不可信，
   `--allow-incomplete` 只用于明确的取证运行。
2. 每一级的输出目录都是**新的、不存在的绝对路径**，且不在输入 run 内部。
3. 横向和竖向两次采集要一起拼时，把多个 `--input-run` 传给同一条命令，而不是各拼各的
   再去合并两张成品。

原始归档、processed-run 和正式 mosaic 一经发布不可覆盖；`--work-dir` 下的缓存可以随时删除重建。
输入输出契约见 [接口合同](../INTERFACES.md)，设计与门禁见 [拼接计划](../MOSAIC_PLAN.md)，
各命令的完整参数以 [`climbot_image_processing`](../../src/climbot_image_processing/README.md) 和
[`climbot_mosaic`](../../src/climbot_mosaic/README.md) 的 README 及 `--help` 为准。

本项目 CUDA 开发机上的完整拼接、性能和验收运行，`build_wall_mosaic` 必须显式传
`--backend cuda`，结束后核对 `mosaic_manifest.json` 的 `execution.effective` 为 `cuda` 且 `execution.fallback=false`。
不要用 `auto` 生成 GPU 性能或验收证据；它可能按设计回退 CPU。CPU 只用于明确的基线或 A/B。

## 诊断墙后验检查

只有诊断墙数据才能做这一步，且必须在 mosaic 渲染完成之后运行——评价器是独立的，不参与
候选生成、匹配、优化或渲染决策。

`MOSAIC_ROOT` 就是上一步 `build_wall_mosaic` 的 `--output-dir`。下面采用新任务的默认墙；
复查旧数据时必须改为该采集归档记录的原始清单，不能用新墙评价旧照片：

```bash
MOSAIC_ROOT="$CLIMBOT_DATA_ROOT/mosaic-<id>-hardcut"

ros2 run climbot_mosaic evaluate_diagnostic_mosaic \
  --mosaic-dir "$MOSAIC_ROOT" \
  --wall-manifest "$CLIMBOT_DATA_ROOT/assets/wall_local_m2_v6/wall_texture.json" \
  --output-dir "$MOSAIC_ROOT-truth-<new-id>"

ros2 run climbot_mosaic inspect_diagnostic_mosaic \
  --mosaic-dir "$MOSAIC_ROOT" \
  --wall-manifest "$CLIMBOT_DATA_ROOT/assets/wall_local_m2_v6/wall_texture.json" \
  --output-dir "$MOSAIC_ROOT-inspection-<new-id>" \
  --input-run "$CLIMBOT_DATA_ROOT/processed-<horizontal-id>" \
  --input-run "$CLIMBOT_DATA_ROOT/processed-<vertical-id>"
```

锚点真值评估和原尺寸检查均支持默认 M2 墙，包括有渐缩端部的 `surface_groove`。
原尺寸检查导出不等同逐张人工复核；覆盖缺口必须保留并单独报告。

**巡检域不再由命令行给出。** `--input-run` 指出这幅拼接是由哪些 processed-run 建的，
检查器沿

```
mosaic manifest → processing_manifest.json → 归档 manifest.json → 冻结任务
```

一路回溯，每一环都用 SHA-256 校验，然后从归档里的 `task.coverage_region` 取巡检域、
从 `task.detection_*` 取相机足迹。指错 run 会因为摘要里的 digest 对不上而被拒绝，所以
运行检查器的人不能再选择这次运行被拿什么来判。`--archive-root` 默认取
`$CLIMBOT_DATA_ROOT`。

2026-10-22 起，G2 验收入口按 `WALL_SURFACE` 使用真实光轴与平墙／原生起伏表面的交点
评测曝光 XY 误差，保留原平面评测旁证；5 mm 最大值门槛不变。入口默认在 case 目录的
`localization_bag/` 记录仿真时钟、轮速、全站仪、真实 IMU、滤波位姿、底盘真值和命令；
可用 `G2_RECORD_LOCALIZATION=false` 显式关闭。离线真值评测见
`tools/evaluate_m2_camera_pose.py --help`，真实输入回放见
`tools/replay_localization_bag.py --help`；历史缺少 IMU 的包不能伪装为真实输入回放。
当前 EKF 默认已显式设置按 M2 数据标定的 Q 及延迟历史处理；原始默认参数位于标定报告
所列数据目录的 `baseline_parameters.yaml`，复现旧基线须显式传 `--params`，不能把新默认
当成旧基线。`tools/analyze_localization_noise.py` 核查消息协方差及运动残差，
`tools/score_localization_replay.py` 报告误差与协方差一致性；真值仅用于离线分析。
`tools/derive_replay_processed_run.py` 从真实回放输出创建独立 processed-run，保留图像字节和
原位姿，记录参数／输出哈希；新数据先使用实际 EKF 先验，不自动覆盖为 σ=1.5 mm。
G2 可显式使用 `G2_EKF_CONFIG_FILE`、`G2_TOTAL_STATION_SEED`、`G2_IMU_SEED` 做冻结参数验证，
未设置时保持产品默认。适用范围与缺测边界见
[本轮标定报告](../../results/m2_localization_calibration_2026-10-22/REPORT.md)。

摘要因此输出 `all_inspection_region_feature_pixels_covered`（门禁）和逐 feature 的域内／域外
缺口。域外再按两套冻结几何拆分：任务实际连续 SCAN 足迹，以及 `motion_region` 内正反扫描
方向的安全位姿相机包络。前者回答计划是否本应扫到，后者回答是否存在安全中心位姿可拍；
不能再从保守上包络的成员关系推断位姿存在。域内 feature 像素数为零时检查器直接报错，
不会输出一个什么都没量到的 `true`。

重点看真值摘要里的共同锚点、局部残差、覆盖计数和未覆盖 feature。原尺寸 tile 只能证明
**已经导出**，不能证明相机拍到了它的每一个像素。当前门禁状态以 [STATUS](../STATUS.md) 为准。

## 墙面贴图

素材来源、校验、普通墙／诊断墙的完整生成命令及资源预算统一见
[README：从源码生成资产](../INSTALLATION.md#4-从源码生成资产)。墙面贴图由 `.gitignore` 排除，
不进仓库；实验时显式用 `wall_texture` 指向生成的清单。

