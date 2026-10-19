# 实验结果索引

远端保留精简报告／摘要；完整验收附件留在本机，见[发布范围](PUBLICATION.md)。

本目录保存可追溯的仿真与验收证据。每份正式摘要应包含任务与配置、软件溯源、随机源、
评价口径和严格 JSON 结果。大体积原始图像与拼接母版保存在 `${CLIMBOT_DATA_ROOT}`，
不进入 Git。路径统一用 `$CLIMBOT_DATA_ROOT`、`$HOME` 和 `$TMPDIR` 表示相应根目录，
临时日志只记文件名；这些引用不表示临时文件永久保留。路径写法规范化不改变原始证据哈希。

本页只列当前仍可引用的基线和入口。过程试验、已替代基线、参数探索和长篇分析保留在
[结果归档](archive/RESULTS_2026-08-27.md)，不得当作当前验收结论。

新机器部署的独立克隆、资产下载／生成与平场标定检查见
[2026-10-18 复现检查](fresh_clone_2026-10-18/REPORT.md)；它不替代运动或拼接验收。

## 已撤销

| 证据 | 撤销原因 |
| --- | --- |
| `mosaic_p206_blind_2026-08-30_summary.json` | **provenance 不成立，不可引用。** 它声明 `commit=ec62a54`、`source_modified=false`，但产生它自身覆盖字段的代码直到 `139a21e` 才存在——那个 provenance 块是手填的，没有任何程序算过。文件保留在原处以便对照，测量本身没有问题（同一批数据在 `20260830c` 上逐位复现），可引用的版本是 `mosaic_p206_blind_2026-08-30b_summary.json`。 |
| `mosaic_p206_blind_2026-08-31_summary.json` | 已由 `2026-08-31c` 取代：旧摘要缺少 off-seam 分母，并包含在本墙面数据上不具判别力的结构边位移代理；其余测量并未被否定。 |

## 当前正式证据

| 范围 | 证据 | 地位 |
| --- | --- | --- |
| 新墙面独立审核与默认切换 | [报告](wall_texture_review_2026-10-19/REPORT.md)、[摘要](wall_texture_review_2026-10-19/summary.json) | 640 DDS／两套各 3624 次共区通过；采集默认新版，模型预览纯色；旧资产保留供历史复查 |
| 新墙面代表性及全墙采集 | [报告](wall_texture_capture_2026-10-19/REPORT.md)、[摘要](wall_texture_capture_2026-10-19/summary.json) | **专项完成并收尾**：70＋680 张、全墙 RTF 0.988472、CUDA 无回退，验证与清理完成；边缘 feature 0.484% 未覆盖另列后续 |
| 墙面局部减复用当前配置 | [报告](wall_texture_local_2026-10-18/REPORT.md)、[摘要](wall_texture_local_2026-10-18/summary.json) | 单源三种子各 40/40 正重叠通过；最终覆盖物 gutter 修复通过（3624 次共区零差异），完整 -j8 1779 项零失败；10-18 离线修复完成；10-19 采集进度见上行 |
| 墙面随机变换首版试验 | [报告](wall_texture_randomization_2026-10-18/REPORT.md)、[精简摘要](wall_texture_randomization_2026-10-18/summary.json) | **未通过专项验收**；局部明显改善，两个独立种子仍有局部错配，远距离压力测试未达标；不替换旧资产或引用旧 P2/S6/S7 通过结论 |
| S6 追加数据精简 | [清理范围与记录](suction_retention_2026-10-18/REPORT.md) | 最终旧／新各 1340 张原图保留；R2 照片、可重建 processed PNG 与 170 个 tile 按用户授权清理；21 个已复核样本及元数据保留 |
| 三风机吸附 S7 默认收口 | [S7 报告](suction_s7_2026-10-18/REPORT.md)、[运行门禁](suction_s7_2026-10-18/RUNTIME_GATE.json) | 新模型为默认，旧恒力路径删除；双 GUI reset、70/70 张中等采集及 CUDA 拼接通过，RTF 0.988452，已合并 main |
| 三风机吸附 S6 最终验收 | [最终报告](suction_s6_2026-10-18/REPORT.md)、[总门禁](suction_s6_2026-10-18/FINAL_GATE.json) | 新质量 91 组、九组双 GUI 控制、四项传感、2680 张原精度照片与 CUDA 链通过；最终 1727 项零失败，总门禁 40 项通过；新结构拍照 RTF 0.986212／0.986974 |
| 三风机吸附 S6 物理预算 | [提手质量与逐轮重算](suction_s6_2026-10-18/physical_model/REPORT.md)、[故障可观测性红绿](suction_s6_2026-10-18/failure_observability/REPORT.md) | 15.518481 kg，解析包络约 6.10 kPa／801 N；阶段物理预算；最终新质量运动与双 GUI 全链已由 S6 最终报告关闭 |
| 三风机吸附 S6 显示／失压演示 | [风扇旋转](suction_s6_2026-10-18/fan_visual_trial/REPORT.md)、[全停跌落](suction_s6_2026-10-18/all_fans_off_trial/REPORT.md) | 实际窗口录像；旋转为慢速显示代理，跌落暴露的故障停报已有独立红绿修复，不代表完整撞地吸附物理验收 |
| 三风机吸附 S6 控制／GUI | [旧／新八工况实际 GUI 配对](suction_s6_2026-10-18/controls/REPORT.md) | 16 组／128 项通过，实测 RTF 约 1，四组额外原生统计；阶段控制记录；最终 S6 已关闭，引用最终报告的冻结证据 |
| 三风机吸附 S6 长采集 | [传感器与首轮完整 GUI 长采集](suction_s6_2026-10-18/CAPTURE_R2_REPORT.md) | 四组／2680 张及压力／清理通过；首轮性能／GUI 配置历史，未作为最终门禁；后续双 GUI 全链已通过 |
| 三风机吸附 S5 | [运动／停机边界](suction_s5_2026-10-18/REPORT.md)、[追加压力验证](suction_s5_pressure_2026-10-18/REPORT.md) | 91 组／698 项与独立审计 139 项通过；追加 32 组明确低压失败，旧质量下 7500 Pa 的 24 组运动通过；新质量下限不外推，当前依据 S6 复验 |
| 三风机吸附 S4 | [压力测量与故障接口报告](suction_s4_2026-10-18/REPORT.md) | 原生九组 63 项、B/C 真实 ROS bridge 各 31 项通过；1590 项完整回归零失败，后续运动／故障边界另见 S5 |
| 三风机吸附 S3 | [平墙原生集成报告](suction_s3_2026-10-17/REPORT.md) | 八朝向 B/C、60 s 保持、逐轮轮载与暂停/reset；231 项主检查及 3 项原始复核通过；传感器后续证据另见 S4，当时的集成阶段记录，后续 S5～S7 已通过，M2 待验收 |
| 控制反馈时戳专项 | [受控红绿对照](cruise_timestamp_2026-10-17/REPORT.md) | 延迟里程计引起的虚假追赶已修复，原速度门槛不变；Start 拒绝诊断补齐，两轮完整 -j8 各 1555 项零失败；ROS 测试不替代 S5/S6 整车验收 |
| 覆盖任务与控制 | `coverage_{bigH,bigTH,bigTV,bigV,horizontal,trapezoid_horizontal,trapezoid_vertical,vertical}_2026-08-20_summary.json` | 默认 `time` 控制的八工况正式仿真基线 |
| 距离控制对照 | 对应的 `*_2026-08-20d_summary.json` | 与默认控制律同口径的对照，不替代默认基线 |
| 定位 | [localization_2026-08-19_summary.json](localization_2026-08-19_summary.json) | 融合定位与轮式里程计的四方向独立真值对照 |
| 侧滑与转向 | `normal_loads_400N.csv`、`turn_slip_2026-08-30.csv`、`turn_map_2026-08-30.csv`（含 `_summary.json`）、`wall_slip_trajectory.csv.gz` | 旧 400 N 结构历史侧滑参考；新结构适用性由 S6 24／48 次转向复验确认；`turn_slip_per_degree_m` 由 24 次转向拟合为 `0.00046`，整圈 48 点 `0.4309--0.5018 mm/deg` 且圆平坦。旧的 `turn_slip.csv`、`turn_map.csv` 与 `turn_map_g1_camera_2026-08-24.*` 保留为历史对照 |
| M2 T0 接触漏检与首条板缝 | [报告](m2_t0_hashspace_2026-10-19/REPORT.md)、[摘要](m2_t0_hashspace_2026-10-19/summary.json) | ODE 跨零哈希漏检已用局部补丁与受控 A/B 修复；平滑面及单条真实板缝探针通过，T0／M2 和 GUI 性能未整体验收 |
| M2 T0 高度场边界 | [报告](m2_t0_extent_2026-10-19/REPORT.md)、[摘要](m2_t0_extent_2026-10-19/summary.json) | 按引擎采样跨度补偿 SDF 尺寸；四组近边界检查及原有五组接触回归通过，整墙与 GUI 尚未验收 |
| M2 T0 裂缝接触简化 | [报告](m2_t0_crack_bridge_2026-10-19/REPORT.md)、[摘要](m2_t0_crack_bridge_2026-10-19/summary.json) | 已纠正量规互撞；真实网格法向仍未通过，显式窄裂缝轮接触近似的静态支撑及隔离接触通过，T0 未整体验收 |
| M2 T0 板缝实际轮径 | [报告](m2_t0_joint_contact_2026-10-19/REPORT.md)、[摘要](m2_t0_joint_contact_2026-10-19/summary.json) | 共网格交汇与主动轮短扫描通过；球轮未通过，圆角／高度场／FCL 均未完全解决，球轮按用户要求挂起；非 T0 通过证据 |
| M2 T0 球径／局部倒角 | [报告](m2_t0_caster_size_chamfer_2026-10-19/REPORT.md)、[摘要](m2_t0_caster_size_chamfer_2026-10-19/summary.json) | 25／40 mm 球径×原槽／短倒角四组完成；倒角改善断触和支撑，加大球径改善小，均未满足全部门槛；未改整车默认 |
| M2 review 基础修正 | [报告](m2_review_2026-10-19/REPORT.md)、[干净需求](m2_review_2026-10-19/physical_model/summary.json) | 发行版 ODE 配置及三组 M1 对照、原生裙边止挡、C++ 唯一表面、漏气预算与旧模型整理；球轮跨缝挂起，M2 未验收 |
| M2 / T2入口接触复核 | [报告](m2_t2_entry_2026-10-19/REPORT.md)、[摘要](m2_t2_entry_2026-10-19/summary.json) | 只离线复用已有时序；真实缝／无缝峰值均紧接单步失触，逐轮需求与漏气／风机耦合仍待验收 |
| M2 / T2球座侧向柔顺试验 | [报告](m2_t2_caster_float_2026-10-19/REPORT.md)、[摘要](m2_t2_caster_float_2026-10-19/summary.json) | 平墙配对通过；±0.5 mm／±2 mm板缝试验均失败，默认关闭，1.5门槛不变，T2未验收 |
| M2 / T2轮胎侧向弹性 | [报告](m2_t2_tyre_lateral_2026-10-19/REPORT.md)、[摘要](m2_t2_tyre_lateral_2026-10-19/summary.json) | 停车锁死已消除；用户确认的v2轮载判据回算：新两组及平墙通过、旧两组失败，原始违规保留；50 ms事后选择，独立验证及T2其余项目待完成，默认关闭 |
| M2 整车起伏接触定位 | [报告](m2_robot_patch_2026-10-19/REPORT.md)、[摘要](m2_robot_patch_2026-10-19/summary.json) | 三角网降速已定位；驱动轮仅起伏高度场＋真实缝球轮代理的9 s短试验通过，行程0.613485 m；漏气／载荷与整墙性能待T2/T3 |
| M2 T1 名义柔顺轮与状态链 | [报告](m2_wheel_compliance_2026-10-19/REPORT.md)、[摘要](m2_wheel_compliance_2026-10-19/summary.json)、[协议](m2_wheel_compliance_2026-10-19/protocol.json) | 150 N／1 mm 实测标定及卸载回零通过；真实 ROS 直行／转向／reset／动态 TF 18 项通过，默认关闭；非 T2/T3 验收 |
| M2 代理球整车平墙滚动 | [报告](m2_caster_rolling_2026-10-19/REPORT.md)、[摘要](m2_caster_rolling_2026-10-19/summary.json) | 原球／代理球直行＋转向两次短动作通过，惯量／滚阻等效；起伏墙、整墙性能及 T1～T3 待验收 |
| M2 T0-F 同心代理球 | [报告](m2_t0_caster_proxy_2026-10-19/REPORT.md)、[首轮摘要](m2_t0_caster_proxy_2026-10-19/summary.json)、[修订摘要](m2_t0_caster_proxy_2026-10-19/summary_revised.json) | 首轮 1 ms 单槽未通过；按离散步长修订门槛后，1 ms 下掩码／平面／单槽／交汇通过，整墙网格规格 0.1×10 mm；隔离台架，非整车或 T0 整体验收 |
| 吸附专项 S0 | [基线与诊断报告](suction_s0_2026-10-17/REPORT.md)、[输入摘要](suction_s0_2026-10-17/baseline_summary.json)、[A 组复测](suction_s0_2026-10-17/a_loads_summary.json) | 冻结旧模型的 34 工况轮压复现与旧质量静力诊断，不是新底盘／负压模型通过；首次回归异常、夹具修复与剩余问题分别保留 |
| 吸附专项 S1 | [新结构与逐轮设计](suction_s1_rearward_2026-10-17/REPORT.md)、[质量与几何](suction_s1_rearward_2026-10-17/layout.json)、[计算摘要](suction_s1_rearward_2026-10-17/summary.json) | 平墙刚性轮离线候选；质量和材料假设、静态／运动需求与交叉敏感性分列。阶段离线设计；运行模型与新提手质量的最终结果见 S6／S7 |
| 吸附专项 S2 | [纯核心隔离报告](suction_s2_2026-10-17/REPORT.md)、[检查与原始数据索引](suction_s2_2026-10-17/summary.json) | C++ 单腔风机／裙边数学参考、留出与数值检查通过；名义漏气范围与不利参数失败分列，不是实机曲线或 Gazebo 整车验收 |
| G1 相机 | `g1_camera_*`、`target_*`、`localization-g1-*` | 相机接口、轴向/畸变和运动投影中心证据 |
| G2/G4 采集 | `g2_accept_{horizontal,vertical,trapezoid}_2026-08-26_summary.json` | 异步匀速位置触发、位姿绑定与原始归档正式结果 |
| P2 相对拼接 | [mosaic_p2_baseline_2026-08-26_summary.json](mosaic_p2_baseline_2026-08-26_summary.json) | 0.25 mm 相对拼接历史基线；其最低改善比例未单独冻结，当前绝对门限见 P2-06 |
| P2 clean blind | [mosaic_p27d_blind_2026-08-27_summary.json](mosaic_p27d_blind_2026-08-27_summary.json) | 诊断墙绝对真值与检查切片；仍有覆盖缺口 |
| P2-06 采集前预检 | [p206_diagnostic_coverage_preflight_2026-08-28.json](p206_diagnostic_coverage_preflight_2026-08-28.json) | 含 100 mm 机动裕量的横 680／竖 660 张离散曝光联合预测；不是实际拼接验收 |
| P2-06 联合盲测与接缝诊断 第 1 组 | [mosaic_p206_blind_2026-08-31c_summary.json](mosaic_p206_blind_2026-08-31c_summary.json) | `3f422f5` 采集 1,340 帧；生成器 provenance 为干净 `2dfaf22`；巡检域内零漏拍，锚点 P95 `1.670 mm`，接缝跳变 P95 `16→7`、比值 `3.50×` |
| P2-06 联合盲测与接缝诊断 第 2 组 | [mosaic_p206_blind_2026-08-31g2_summary.json](mosaic_p206_blind_2026-08-31g2_summary.json) | `f849af1` 无头采集 1,340 帧；巡检域内零漏拍，锚点 P95 `0.630 mm`，接缝跳变 P95 `16→7`、比值 `3.50×` |
| P2-06 联合盲测与接缝诊断 第 3 组 | [mosaic_p206_blind_2026-08-31g3_summary.json](mosaic_p206_blind_2026-08-31g3_summary.json) | `f849af1` 无头采集 1,340 帧；巡检域内零漏拍，锚点 P95 `1.360 mm`，接缝跳变 P95 `16→7`、比值 `3.50×` |
| P2-06 联合盲测与接缝诊断 第 4 组 | [mosaic_p206_blind_2026-08-31g4_summary.json](mosaic_p206_blind_2026-08-31g4_summary.json) | `ac2b244` 无头采集 1,340 帧，GUI 实测不存在；巡检域内零漏拍，锚点 P95 `1.787 mm`，接缝跳变 P95 `16→7`、比值 `3.50×` |
| P2-06 门限独立检验 第 5 组 | [mosaic_p206_blind_2026-08-31g5_summary.json](mosaic_p206_blind_2026-08-31g5_summary.json) | 门限冻结（`1b1a0c7`）之后采集，不参与推导；七项全过：域内零漏拍、锚点 P95 `1.130 mm`、残差 P95 `0.143 mm`、尺度 `16.2 ppm`、航向 `0.0002°`、接缝比 `3.50`；用户快速浏览基本全组未见明显问题，逐张原尺寸细节复核仍待完成 |

完整的验收要求、证据路径和缺口请看 [验收矩阵](../docs/ACCEPTANCE.md)。

## 当前有效范围

- 正式覆盖与控制结论仅适用于表中的仿真工况；它们不是实机验收结果。
- P2-06 的覆盖已闭合：**五组**重复采集，巡检域内均零漏拍，绝对锚点 P95 `1.670 / 0.630 /
  1.360 / 1.787 / 1.130 mm`。全局 hard-cut 接缝的真值扣除跳变 P95 五组同为 pose-only `16`、
  optimized `7` 灰度/像素；同图 off-seam 基线均为 `2`，即接缝额外不连续从 `8×` 降至 `3.5×`。
  此前结构边位移代理不具判别力，已撤回。**门限由带渲染记录的第 2、3、4 组推导，并由第 5 组
  独立检验通过**（第 5 组在门限冻结提交之后采集，不参与推导）；第 1 组渲染工况未记录，只作
  旁证。门限值与推导见[拼接计划](../docs/MOSAIC_PLAN.md)。人工 100% 全分辨率复核已于
  2026-09-01 在第 1 组的 `170` 个原尺寸 tile 上完成，未发现错位类缺陷；可见的亮度接缝是
  光度问题，根因是旧标定靶与被摄墙的着色路径不匹配；当前修复与 A/B 见
  [当前状态](../docs/STATUS.md)。第 5 组 tile 已逐个哈希核验，可运行
  `python3 tools/prepare_native_tile_review.py <第5组 diagnostic_inspection_summary.json>`
  在同目录生成本机查看页。页面一次显示一张，三列直接标明真值、仅位姿和优化后拼接；
  可按特征跳转、切换概览／原尺寸，并记录“无明显问题／需复查”。标记只是本机查看记录，
  不能自动产生人工通过结论。
- 尺度误差含已知流水线系统偏置 `+21 ppm`（10 m 上 `0.21 mm`），各组复现到 `±5 ppm`，
  来源已定位到位姿图上游的相机模型与重采样链，不是采集工况效应；门限 `≤ 50 ppm` 包住它。
- 巡检域外各组有 `43 万` 量级未覆盖像素，不作门禁要求；其中一部分落在当前计划 SCAN 足迹内，
  冻结 `motion_region` 安全位姿相机包络外恒为 `0`（属构造必然，不作独立证据）。逐组数值见
  各摘要的 `inspection.visible_feature_coverage`。
- P2.7d 及更早的绝对毫米数（三锚点 P95 19.48 → 10.40 mm）受一处墙面几何缺陷影响：
  贴图块曾高出碰撞面 1.25 mm，使拼接带上 +4566 ppm 尺度误差。这些值只能用于当时的
  相对比较，不能与 `3f422f5` 之后的数据混用。
- P2-06 新任务的采集前预检把 feature 裁到巡检域再判断，与采集后检查器的口径不同；
  它不读取新采图像，不能替代 `coverage_count.tif`。
- 2026-08-18b 之前的法向载荷口径为 220 N；与旧 400 N 基线不能直接混用，更不能外推到当前新质量负压模型。
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
