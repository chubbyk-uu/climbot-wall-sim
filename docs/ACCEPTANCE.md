# 第一阶段最终验收矩阵

更新日期：2026-08-21。

本文把 [PROJECT_GUIDE.md](../PROJECT_GUIDE.md) §14 的每一项要求映射到可复查证据。
它不另立指标；规范仍是要求的唯一来源，[results/README.md](../results/README.md)
仍是正式实验批次与口径的唯一索引。

状态含义：

- **通过**：已有自动化测试或可追溯的 Gazebo 实验直接覆盖；
- **仿真通过／实机待冻结**：达到 §14.3 的建议初值，但最终实机阈值尚未确定；
- **条件不适用**：规范中的条件分支当前未触发；
- **待实机**：软件仿真不能给出最终安全或产品结论。

## 结论摘要

| 范围 | 结论 |
| --- | --- |
| A～F 仿真实现 | 通过；§14.1、14.2、14.4、14.5 和 14.6 的仿真内要求均有对应证据 |
| §14.3 建议阈值 | 当前正式八工况全部通过；这些数值仍是“建议初值”，不是已冻结的真机指标 |
| 软件停车 | Gazebo 与节点级故障注入通过；ROS hold 不是硬件急停，实机必须使用默认失效关闭的驱动使能／急停回路 |
| Phase F | 仿真阶段完成；不把实机阈值冻结、硬件急停验证或后续 `climbot_inspection` 当成已完成 |

正式运动基线采用 `2026-08-20`（默认 time 控制）和 `2026-08-20d`
（distance 对照），各 8 个工况、共 16 个工况均 `passed=true`。它们产自提交
`84c28fc` 的干净工作树；控制器拆分前后的指标没有系统性偏移。定位采用
[localization_2026-08-19_summary.json](../results/localization_2026-08-19_summary.json)，
侧滑专项采用 `slip_compensation_*_2026-08-17` 配对结果。

## 14.1 功能验收

| ID | 要求摘要 | 状态 | 证据 |
| --- | --- | --- | --- |
| F-01 | 一条 launch 启动完整仿真 | 通过 | `src/climbot_bringup/launch/coverage_sim.launch.py`；README 启动流程；launch 测试 |
| F-02 | 墙、机器人、重力、吸附与接触同时生效 | 通过 | `test_wall_world_matches_description.py`、`test_model_defaults.py`、[normal_loads_400N.csv](../results/normal_loads_400N.csv) |
| F-03 | 两主动轮、中部略靠前、后球随动轮 | 通过 | `robot.yaml`、URDF/Xacro、`test_robot_description_defaults.py` |
| F-04 | 吸附投影在三点支撑内且不与主动轮轴重合 | 通过 | `robot.yaml` 几何；`test_normal_load_statistics.py`；PROJECT_GUIDE §6.3 |
| F-05 | 全工况三点正载荷，后轮不低于 `0.15 F_s` | 通过 | [normal_loads_400N.csv](../results/normal_loads_400N.csv)：零载荷采样 0，最低后轮 `95.886 N > 60 N` |
| F-06 | 前进、后退、停止、原地旋转 | 通过 | §15.5～15.7 结果、`turn_map.csv`、`test_turn_profile.py` |
| F-07 | 静止不持续下滑或脱墙 | 通过 | `normal_loads_400N.csv` 静止接触率 100%；侧滑标定静止阶段 |
| F-08 | 关闭补偿可测得水平下滑 | 通过 | `slip_compensation_off_2026-08-17_*`：平均下降 `121.43 mm` |
| F-09 | 开启补偿形成向上偏航并减小漂移 | 通过 | `slip_compensation_on_2026-08-17_*`：平均偏航 `+6.13°`，漂移降低 `94.82%` |
| F-10 | 发布真值、IMU、轮里程计、全站仪与融合位姿 | 通过 | `climbot_wall.launch.py`、`ekf_wall.yaml`、`test_ekf_startup.py` |
| F-11 | 合法矩形／等腰梯形，横／纵弓字规划 | 通过 | `test_coverage_geometry.cpp`、`test_coverage_planner_node.py`；八工况两种区域和方向 |
| F-12 | 原地转向＋直线段执行完整任务 | 通过 | `test_coverage_executor.py`；16 个正式完整覆盖工况 |
| F-13 | 任务中不重置全局里程计 | 通过 | 定位配置与执行器无 reset 接口；`test_command_routing.py` |
| F-14 | 定位丢失或严重故障停车 | 通过（软件边界） | `test_safe_stop.py`、`test_cmd_vel_watchdog_node.py`、管理器 executor-loss/late-callback/forced-recovery 测试 |
| F-15 | 完成后停车并输出结果 | 通过 | 16 个正式摘要 `passed=true`；`test_coverage_executor.py`、`test_results_are_machine_readable.py` |

## 14.2 路径几何验收

| ID | 要求摘要 | 状态 | 证据 |
| --- | --- | --- | --- |
| G-01 | 名义点和动态参考均在 `motion_region` | 通过 | `test_coverage_geometry.cpp`、`test_safe_stop.py`、`test_coverage_arc_entry.py` |
| G-02 | 扫描线平行且与指定方向一致 | 通过 | `test_coverage_geometry.cpp`；八工况轨迹摘要 |
| G-03 | 相邻扫描线间距满足配置误差 | 通过 | 正式基线最坏 `5.17 mm ≤ 20 mm`；规划几何单测 |
| G-04 | 相邻扫描方向交替 | 通过 | `test_coverage_geometry.cpp`、`test_coverage_planner_node.py` |
| G-05 | 无圆弧拐角或跨路点捷径 | 通过 | 规划 Path 全为直线段；`test_coverage_schedule_pose.py`；动态小弧线仅是偏差过大时的执行入轨，不改名义路径 |
| G-06 | 覆盖有效矩形或等腰梯形 | 通过 | 正式八工况覆盖率 `98.68%～100%`；规划名义覆盖率测试 ≥98% |
| G-07 | 斜向换道与顶部收边仍由直线组成 | 通过 | `test_coverage_top_edge_scan.py`、`test_coverage_geometry.cpp` |
| G-08 | 同输入生成完全一致路径 | 通过 | 规划器确定性单测与节点级回归 |

## 14.3 轨迹跟踪验收

### 指标是否产出

| ID | 指标／行为 | 状态 | 证据 |
| --- | --- | --- | --- |
| T-01 | 单段横轨 RMS 和最大绝对误差 | 通过 | 每份 `coverage_*_summary.json` 的逐段指标；`test_execution_metrics.py` |
| T-02 | 终点位置误差 | 通过 | 同上，真值口径动态终点误差 |
| T-03 | 转向结束航向误差 | 通过 | 同上；由真值与控制目标重构，不取控制器自报误差 |
| T-04 | 实际／规划长度比 | 通过 | 正式基线 `1.016～1.038`（distance）并在摘要记录 |
| T-05 | 水平段高度漂移 | 通过 | 摘要逐段高度指标与 §14.4 配对实验 |
| T-06 | 最大补偿角与角速度 | 通过 | 摘要记录；正式基线最大补偿角约 `7.03°` |
| T-07 | 切角／明显蛇形判据 | 通过 | `execution_metrics` 统一 `20 mm` 可见往复判据；正式八工况全部为 0 |
| T-08 | 覆盖率按实际二维足迹，且只累计 SCAN | 通过 | `test_coverage_metrics.py`；摘要同时使用 `detection_width` 与 `detection_length` |
| T-09 | 竖向任务不逐列倒车 | 通过 | 规划几何测试与正式竖向轨迹；动态换道前进执行 |
| T-10 | 低覆盖时可加顶部收边 | 通过 | `top_edge_scan=auto/always/never` 与 `test_coverage_top_edge_scan.py` |

### 建议初值对照

| ID | §14.3 建议初值 | 仿真最坏值 | 状态 |
| --- | ---: | ---: | --- |
| TT-01 | 单段横轨 RMS ≤ `20 mm` | time 正式八工况最大 `3.96 mm` | 仿真通过／实机待冻结 |
| TT-02 | 最大绝对横轨误差 ≤ `50 mm` | time 正式八工况最大 `5.99 mm` | 仿真通过／实机待冻结 |
| TT-03 | 路点终点误差 ≤ `30 mm` | `4.19 mm` | 仿真通过／实机待冻结 |
| TT-04 | 转向航向误差 ≤ `2°` | `1.05°` | 仿真通过／实机待冻结 |
| TT-05 | 水平段净高度误差 ≤ `30 mm` | time 正式八工况最大 `1.67 mm` | 仿真通过／实机待冻结 |
| TT-06 | 扫描线间距误差 ≤ `20 mm` | `5.17 mm` | 仿真通过／实机待冻结 |
| TT-07 | 漏扫面积 ≤ `5%` | `1.32%` | 仿真通过／实机待冻结 |
| TT-08 | 定位严重超时停车 ≤ `0.5 s` | Gazebo 回归 `246 ms` | 仿真通过／实机待冻结 |

这里的“实机待冻结”不是仿真失败。它明确保留 PROJECT_GUIDE §14.3 已写下的产品决策：
只有拿到真实机器人尺寸、检测载荷足迹、全站仪数据和作业质量要求后，才能决定是否
收紧或放宽这些阈值。

## 14.4 侧滑补偿专项验收

| ID | 要求 | 结果 | 状态 |
| --- | --- | --- | --- |
| S-01 | 关闭时稳定、可重复向下漂移 | 三次全部向下，平均 `121.43 mm` | 通过 |
| S-02 | 下降／前进比 CV ≤5% | `0.41%` | 通过 |
| S-03 | 开启后净高度误差降低 ≥70% | `94.82%` | 通过 |
| S-04 | 允许小幅向上航向偏角 | 平均 `+6.13°` | 通过 |
| S-05 | 不产生明显蛇形 | `20 mm` 门限可见往复 0 次 | 通过 |
| S-06 | 位置比机体朝向更接近水平线 | 轨迹倾角 `0.03～0.69°`，机体约 `6.13°` | 通过 |

完整方法、三次重复与重生成命令见 [results/README.md](../results/README.md) 的
“§14.4 侧滑补偿专项验收”。

## 14.5 定位验收

| ID | 要求摘要 | 状态 | 证据 |
| --- | --- | --- | --- |
| L-01 | 所有传感器时间戳有效 | 通过 | 适配器实现与 `test_ekf_startup.py` |
| L-02 | TF 连续且无环 | 通过 | `test_ekf_startup.py`、`test_wall_frame.py`；单一 TF 发布职责 |
| L-03 | 正常条件下融合定位不中断、不跳变 | 通过 | 四方向 [定位摘要](../results/localization_2026-08-19_summary.json)，最大误差 `2.636 mm` |
| L-04 | 轮里程计体现不可观测侧滑误差 | 通过 | 同一定位实验轮式航位推算最终误差 `511.553 mm` |
| L-05 | 全站仪频率、噪声、延迟可配置 | 通过 | 默认 `12 Hz / 1 mm / 50 ms`；`nstd*`、`nrate*`、`ndrop*` 扫描 |
| L-06 | 融合长期误差显著小于轮里程计 | 通过 | 最大误差比 `194×`，EKF 不增长而轮式误差累积 |
| L-07 | 评价不得以估计值冒充真值 | 通过 | Gazebo `ground_truth` 独立输入评价器；`test_trajectory_io.py` |
| L-08 | 使用 `robot_localization/ekf_node` 融合三类输入 | 通过 | `ekf_wall.yaml`、`climbot_wall.launch.py`、`test_ekf_startup.py` |
| L-09 | 状态选择、协方差、频率与 TF 配置可追溯 | 通过 | 版本控制内 `ekf_wall.yaml` 与定位摘要 provenance |
| L-10 | 专用 EKF 需与现成模块定量对比 | 条件不适用 | 当前没有自研 EKF，继续使用 `robot_localization` |

## 14.6 工程质量验收

| ID | 要求摘要 | 状态 | 证据 |
| --- | --- | --- | --- |
| Q-01 | 构建无错误 | 通过 | 本矩阵提交前全量 `colcon build` 门禁 |
| Q-02 | 自动化测试通过 | 通过 | 本矩阵提交前全量 `colcon test` 与 `colcon test-result` 门禁 |
| Q-03 | 启停无残留关键进程 | 通过 | 回归脚本进程组 TERM→KILL；管理器/看门狗 launch 测试；正式批次清理检查 |
| Q-04 | 参数集中配置且标单位 | 通过 | description/gazebo/control/coverage YAML；配置测试 |
| Q-05 | 节点、话题、服务、Action、TF 有文档 | 通过 | [ARCHITECTURE.md](ARCHITECTURE.md)、[OPERATION.md](OPERATION.md) |
| Q-06 | 默认矩形可复现实验 | 通过 | `coverage_horizontal_demo.yaml` 与 `run_coverage_regression.sh` |
| Q-07 | 结果含参数、路径、融合轨迹、真值与摘要 | 通过 | 正式 `*_summary.json` + `*_trajectory.csv.gz`；严格 JSON 门禁 |
| Q-08 | 无特定开发机绝对路径 | 通过 | ament 包定位、相对配置与 launch 参数；源码搜索门禁 |
| Q-09 | C++ 核心，Python 仅辅助 | 通过 | 规划、跟踪、状态机均 C++；仿真适配与评价脚本 Python |
| Q-10 | 依赖、Gazebo、接口、入口明确 | 通过 | 各 `package.xml`、README、ARCHITECTURE、OPERATION |

## 尚不能由 Phase F 仿真关闭的事项

1. **冻结实机阈值。** 需要真实整机尺寸、检测载荷有效足迹、不同壁面摩擦、全站仪
   实测误差与作业漏检成本；届时逐项替换 TT-01～TT-08 的“建议初值”。
2. **硬件安全边界。** 仿真已证明软件停车路径，但不能证明 ROS 进程、IPC 或主机掉电时
   仍停车。实机必须验收默认失效关闭的驱动使能／急停回路，并单列响应时间。
3. **WheelSlip 保真度。** Gazebo 柔度按标称载荷缩放，不随三个接触点瞬时法向载荷变化；
   该限制已接受用于第一阶段，不能直接把参数解释成真实轮墙材料参数。
4. **视觉巡检扩展。** 墙面贴图及其渲染质量验证是 `climbot_inspection` 的准备工作；
   面阵相机、位置触发拍照和图像—融合位姿关联属于下一阶段，不反向阻塞 Phase F。

## Phase G1 面阵相机基础验收（待实施）

本节是 G1 的进入／退出门，不表示当前已经通过。G1 只验收安装几何、真实镜头模型、
标准图像接口和人工单拍；自动位置触发及图像—任务—位姿绑定属于 G2。

| ID | G1 要求 | 验收方法／门限 | 当前状态 |
| --- | --- | --- | --- |
| C-01 | 分辨率与轴向正确 | 原图严格为 `1920×1080`；标定靶证明 1920 方向垂直机器人前进方向、1080 方向平行，机器人前进在图像中向上 | 待实施 |
| C-02 | 前置安装且无遮挡 | 镜头中心外参为 `[0.300,0,0.275] m`；解析净空不小于 `20 mm`，完整有效 ROI 的 Gazebo 图像中不得出现机体、支架或线缆 | 待实施 |
| C-03 | 原始／有效视野满足覆盖 | 标称原始墙面视野不小于 `0.550×0.309375 m`；去畸变有效区域实测覆盖不小于 `0.500×0.28125 m` | 待实施 |
| C-04 | TF 外参唯一且一致 | URDF、Gazebo 渲染位姿和共享 YAML 来自同一数值；平移差 ≤`1 mm`、旋转差 ≤`0.1°`，TF 树无环 | 待实施 |
| C-05 | 内参发布正确 | `CameraInfo` 的尺寸、`K/P`、主点、焦距和 `frame_id` 与配置逐项一致；图像与标定时间戳完全相同 | 待实施 |
| C-06 | 非理想镜头畸变生效 | 原图 `D` 非零且为 `plumb_bob`；畸变网格观测与 Brown–Conrady 投影的 RMS ≤`1 px`，校正后直线残差 RMS ≤`1 px` | 待实施 |
| C-07 | 单次触发语义可靠 | 每个成功服务调用恰好产生一帧；空闲 10 s 零帧；并发明确拒绝；超时／迟到帧不串单 | 待实施 |
| C-08 | 投影中心考虑前置外参 | 用真值仅作评价时，外参投影中心误差 ≤`2 mm`；用 EKF 计算的中心误差不超过现有定位验收门限，业务节点不订阅真值 | 待实施 |
| C-09 | 相机载荷不破坏吸附 | 按相机及支架 `2.0 kg` 代理质量重跑三点法向载荷、静止吸附和代表性横／竖任务，继续满足 §14.3～14.4，且无持续接触丢失 | 待实施 |
| C-10 | 参数与标定可追溯 | 摘要记录内外参、畸变模型、有效 ROI、相机质量、配置来源、提交和工作树状态；真机标定不得沿用仿真 `D` 冒充测量值 | 待实施 |

G1 通过前不把 `coverage` 配置中的临时 `detection_length=0.01 m` 直接改成相机长度。
修改覆盖足迹时必须同时实现 `+0.300 m` 前置投影偏移的规划／评价语义，并作为 G2
接口改动单独验收。
