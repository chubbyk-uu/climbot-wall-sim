# 实验结果索引

本目录保存可追溯的仿真与验收证据。每份正式摘要应包含任务与配置、软件溯源、随机源、
评价口径和严格 JSON 结果。大体积原始图像与拼接母版保存在 `${CLIMBOT_DATA_ROOT}`，
不进入 Git。

本页只列当前仍可引用的基线和入口。过程试验、已替代基线、参数探索和长篇分析保留在
[结果归档](archive/RESULTS_2026-08-27.md)，不得当作当前验收结论。

## 当前正式证据

| 范围 | 证据 | 地位 |
| --- | --- | --- |
| 覆盖任务与控制 | `coverage_{bigH,bigTH,bigTV,bigV,horizontal,trapezoid_horizontal,trapezoid_vertical,vertical}_2026-08-20_summary.json` | 默认 `time` 控制的八工况正式仿真基线 |
| 距离控制对照 | 对应的 `*_2026-08-20d_summary.json` | 与默认控制律同口径的对照，不替代默认基线 |
| 定位 | [localization_2026-08-19_summary.json](localization_2026-08-19_summary.json) | 融合定位与轮式里程计的四方向独立真值对照 |
| 侧滑与转向 | `normal_loads_400N.csv`、`turn_slip.csv`、`turn_map.csv`、`wall_slip_trajectory.csv.gz` | 当前 400 N 几何与侧滑参考 |
| G1 相机 | `g1_camera_*`、`target_*`、`localization-g1-*` | 相机接口、轴向/畸变和运动投影中心证据 |
| G2/G4 采集 | `g2_accept_{horizontal,vertical,trapezoid}_2026-08-26_summary.json` | 异步匀速位置触发、位姿绑定与原始归档正式结果 |
| P2 相对拼接 | [mosaic_p2_baseline_2026-08-26_summary.json](mosaic_p2_baseline_2026-08-26_summary.json) | 0.25 mm 相对拼接基线；尚未冻结验收门槛 |
| P2 clean blind | [mosaic_p27d_blind_2026-08-27_summary.json](mosaic_p27d_blind_2026-08-27_summary.json) | 诊断墙绝对真值与检查切片；仍有覆盖缺口 |
| P2-06 采集前预检 | [p206_diagnostic_coverage_preflight_2026-08-28.json](p206_diagnostic_coverage_preflight_2026-08-28.json) | 绿框内横／竖互补任务的离散曝光预测；不是实际拼接验收 |

完整的验收要求、证据路径和缺口请看 [验收矩阵](../docs/ACCEPTANCE.md)。

## 当前有效范围

- 正式覆盖与控制结论仅适用于表中的仿真工况；它们不是实机验收结果。
- P2.7d 的三锚点绝对 P95 已从 Pose 的 19.48 mm 降至优化后的 10.40 mm，但诊断特征
  几何区域仍有 22.99% 未被相机覆盖，P2-06 不能据此关闭。
- P2-06 新任务的采集前预检预测横、竖联合 1,362 张能覆盖绿框目标域内全部 21 个 declared
  feature；它不读取新采图像，不能替代 `coverage_count.tif` 或关闭 P2-06。
- 2026-08-18b 之前的法向载荷口径为 220 N；与当前 400 N 基线不能直接混用。
- 仅带 `source_modified` 的记录用于趋势观察；可复现实验须有完整源码版本、命令与输入溯源。

## 复现与新增规则

| 目标 | 入口 |
| --- | --- |
| 运行机器人仿真、规划、控制、采集与拼接 | [项目 README](../README.md) 的快速启动 |
| 查看拼接阶段、输入输出与当前实施顺序 | [拼接计划](../docs/MOSAIC_PLAN.md) |
| 查看验收门槛及正式证据要求 | [验收矩阵](../docs/ACCEPTANCE.md) |
| 判断原始数据、母版与结果是否可清理 | [数据保留清单](../docs/DATA_RETENTION.md) |

新增正式结果时，先产出严格 JSON 摘要，再更新验收矩阵和本索引。被替代的结果应移入
`results/archive/` 并注明替代关系，不覆盖或删除历史证据。归档不等于可以删除其外部数据；
是否清理仍以数据保留清单为准。
