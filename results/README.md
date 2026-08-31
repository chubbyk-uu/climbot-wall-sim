# 实验结果索引

本目录保存可追溯的仿真与验收证据。每份正式摘要应包含任务与配置、软件溯源、随机源、
评价口径和严格 JSON 结果。大体积原始图像与拼接母版保存在 `${CLIMBOT_DATA_ROOT}`，
不进入 Git。

本页只列当前仍可引用的基线和入口。过程试验、已替代基线、参数探索和长篇分析保留在
[结果归档](archive/RESULTS_2026-08-27.md)，不得当作当前验收结论。

## 已撤销

| 证据 | 撤销原因 |
| --- | --- |
| `mosaic_p206_blind_2026-08-30_summary.json` | **provenance 不成立，不可引用。** 它声明 `commit=ec62a54`、`source_modified=false`，但产生它自身覆盖字段的代码直到 `139a21e` 才存在——那个 provenance 块是手填的，没有任何程序算过。文件保留在原处以便对照，测量本身没有问题（同一批数据在 `20260830c` 上逐位复现），可引用的版本是 `mosaic_p206_blind_2026-08-30b_summary.json`。 |
| `mosaic_p206_blind_2026-08-31_summary.json` | 已由 `2026-08-31c` 取代：旧摘要缺少 off-seam 分母，并包含在本墙面数据上不具判别力的结构边位移代理；其余测量并未被否定。 |

## 当前正式证据

| 范围 | 证据 | 地位 |
| --- | --- | --- |
| 覆盖任务与控制 | `coverage_{bigH,bigTH,bigTV,bigV,horizontal,trapezoid_horizontal,trapezoid_vertical,vertical}_2026-08-20_summary.json` | 默认 `time` 控制的八工况正式仿真基线 |
| 距离控制对照 | 对应的 `*_2026-08-20d_summary.json` | 与默认控制律同口径的对照，不替代默认基线 |
| 定位 | [localization_2026-08-19_summary.json](localization_2026-08-19_summary.json) | 融合定位与轮式里程计的四方向独立真值对照 |
| 侧滑与转向 | `normal_loads_400N.csv`、`turn_slip_2026-08-30.csv`、`turn_map_2026-08-30.csv`（含 `_summary.json`）、`wall_slip_trajectory.csv.gz` | 当前 400 N 几何与侧滑参考；`turn_slip_per_degree_m` 由 24 次转向拟合为 `0.00046`，整圈 48 点 `0.4309--0.5018 mm/deg` 且圆平坦。旧的 `turn_slip.csv`、`turn_map.csv` 与 `turn_map_g1_camera_2026-08-24.*` 保留为历史对照 |
| G1 相机 | `g1_camera_*`、`target_*`、`localization-g1-*` | 相机接口、轴向/畸变和运动投影中心证据 |
| G2/G4 采集 | `g2_accept_{horizontal,vertical,trapezoid}_2026-08-26_summary.json` | 异步匀速位置触发、位姿绑定与原始归档正式结果 |
| P2 相对拼接 | [mosaic_p2_baseline_2026-08-26_summary.json](mosaic_p2_baseline_2026-08-26_summary.json) | 0.25 mm 相对拼接基线；尚未冻结验收门槛 |
| P2 clean blind | [mosaic_p27d_blind_2026-08-27_summary.json](mosaic_p27d_blind_2026-08-27_summary.json) | 诊断墙绝对真值与检查切片；仍有覆盖缺口 |
| P2-06 采集前预检 | [p206_diagnostic_coverage_preflight_2026-08-28.json](p206_diagnostic_coverage_preflight_2026-08-28.json) | 含 100 mm 机动裕量的横 680／竖 660 张离散曝光联合预测；不是实际拼接验收 |
| P2-06 联合盲测与接缝诊断 第 1 组 | [mosaic_p206_blind_2026-08-31c_summary.json](mosaic_p206_blind_2026-08-31c_summary.json) | `3f422f5` 采集 1,340 帧；生成器 provenance 为干净 `2dfaf22`；巡检域内零漏拍，锚点 P95 `1.670 mm`，接缝跳变 P95 `16→7`、比值 `3.50×` |
| P2-06 联合盲测与接缝诊断 第 2 组 | [mosaic_p206_blind_2026-08-31g2_summary.json](mosaic_p206_blind_2026-08-31g2_summary.json) | `f849af1` 无头采集 1,340 帧；巡检域内零漏拍，锚点 P95 `0.630 mm`，接缝跳变 P95 `16→7`、比值 `3.50×` |
| P2-06 联合盲测与接缝诊断 第 3 组 | [mosaic_p206_blind_2026-08-31g3_summary.json](mosaic_p206_blind_2026-08-31g3_summary.json) | `f849af1` 无头采集 1,340 帧；巡检域内零漏拍，锚点 P95 `1.360 mm`，接缝跳变 P95 `16→7`、比值 `3.50×` |

完整的验收要求、证据路径和缺口请看 [验收矩阵](../docs/ACCEPTANCE.md)。

## 当前有效范围

- 正式覆盖与控制结论仅适用于表中的仿真工况；它们不是实机验收结果。
- P2-06 的覆盖已闭合：三组重复采集，巡检域内均零漏拍，绝对锚点 P95 `1.670 / 0.630 /
  1.360 mm`。全局 hard-cut 接缝的真值扣除跳变 P95 三组同为 pose-only `16`、optimized `7`
  灰度/像素；同图 off-seam 基线均为 `2`，即接缝额外不连续从 `8×` 降至 `3.5×`。此前结构边
  位移代理不具判别力，已撤回。**门限已按三组冻结，但尚未被独立数据检验**——它们由这三组
  推导又用于判定这三组，且第 1 组参与过实现过程。门限值与推导见
  [拼接计划](../docs/MOSAIC_PLAN.md)。
- 尺度误差含已知流水线系统偏置 `+21 ppm`（10 m 上 `0.21 mm`），三组复现到 `±2 ppm`，
  来源已定位到位姿图上游的相机模型与重采样链，不是采集工况效应；门限 `≤ 50 ppm` 包住它。
- 巡检域外三组分别有 `439,116 / 445,544 / 434,165` 个未覆盖像素，不作门禁要求；其中当前
  计划 SCAN 足迹内为 `32,504 / 32,970 / 33,646`，冻结 `motion_region` 安全位姿相机包络外
  恒为 `0`（属构造必然，不作独立证据）。
- P2.7d 及更早的绝对毫米数（三锚点 P95 19.48 → 10.40 mm）受一处墙面几何缺陷影响：
  贴图块曾高出碰撞面 1.25 mm，使拼接带上 +4566 ppm 尺度误差。这些值只能用于当时的
  相对比较，不能与 `3f422f5` 之后的数据混用。
- P2-06 新任务的采集前预检把 feature 裁到巡检域再判断，与采集后检查器的口径不同；
  它不读取新采图像，不能替代 `coverage_count.tif`。
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
