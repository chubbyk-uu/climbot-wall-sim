# 验收矩阵

更新日期：2026-10-22。

本文把项目规范的每一项要求映射到可复查证据。它不另立指标；
[PROJECT_GUIDE.md](../PROJECT_GUIDE.md) 仍是要求的唯一来源，
[results/README.md](../results/README.md) 仍是正式实验批次与口径的唯一索引。

本文中的 `§N` 沿用带编号的完整规范
[PROJECT_GUIDE_2026-08-27.md](archive/specifications/PROJECT_GUIDE_2026-08-27.md)：
当前的 `PROJECT_GUIDE.md` 已收敛为不分节的约束清单，编号只在归档件中可查。
保留编号是为了不改写既有证据的引用口径，不表示归档件仍是规范来源。

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
| Phase F | 仿真阶段完成；不把实机阈值冻结或硬件急停验证当成已完成 |
| 三风机负压 M1／M2 | M1 的 S0～S7 完成并合并 main；新负压底盘为默认，S6 全链＋S7 双 GUI 中等采集／拼接通过；M2 T0～T3 完成，当前默认起伏墙＋柔顺轮；μ=0.8 的短工况范围另列 |

当前 M2 的依据为 V-07～V-10；S6／S7 覆盖 M1 冻结工况。以下旧结构数字保留为控制算法历史基线。
历史运动基线采用 `2026-08-20`（默认 time 控制）和 `2026-08-20d`
（distance 对照），各 8 个工况、共 16 个工况均 `passed=true`。它们产自提交
`84c28fc` 的干净工作树；控制器拆分前后的指标没有系统性偏移。定位采用
[localization_2026-08-19_summary.json](../results/localization_2026-08-19_summary.json)，
侧滑专项采用 `slip_compensation_*_2026-08-17` 配对结果。

## 14.1 功能验收

F-02～F-05 的历史数字来自冻结旧结构，不能解释为当前默认的轮载结果。
新默认内置球轮不沿用 `0.15 F_s`，按 V 系列和 S6 逐轮判据验收。

| ID | 要求摘要 | 状态 | 证据 |
| --- | --- | --- | --- |
| F-01 | 一条 launch 启动完整仿真 | 通过 | `src/climbot_bringup/launch/coverage_sim.launch.py`；README 启动流程；launch 测试 |
| F-02 | 墙、机器人、重力、吸附与接触同时生效 | 通过 | `test_wall_world_matches_description.py`、`test_model_defaults.py`、[normal_loads_400N.csv](../results/normal_loads_400N.csv) |
| F-03 | 两主动轮、中部略靠前、后球随动轮 | 通过 | `robot.yaml`、URDF/Xacro、`test_robot_description_defaults.py` |
| F-04 | 吸附投影在三点支撑内且不与主动轮轴重合 | 通过 | `robot.yaml` 几何；`test_normal_load_statistics.py`；归档规范 §6.3 |
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

### 负压专项新增验收

要求来源为 [项目规范](../PROJECT_GUIDE.md) 与 [S0 实验合同](SUCTION_CONTRACT.md)，阶段和
完整证据要求见 [吸附计划](SUCTION_PLAN.md)。下表不改变既有运动／图像门限。

| ID | 要求摘要 | 状态 | 证据／入口 |
| --- | --- | --- | --- |
| V-01 | 新结构、质量／惯量、圈外驱动轮、圈内球轮与同源压力面 | S1 设计、S3 原生集成通过 | [S1 净空与预览](../results/suction_s1_rearward_2026-10-17/REPORT.md)、[S3 原生轮载](../results/suction_s3_2026-10-17/REPORT.md)；球轮 x=-0.300 m |
| V-02 | 各驱动轮摩擦圆裕量至少 1.5，球轮不少于 10 N，重新求吸力 | M1 名义静止与注册运动通过；M2 使用独立冻结判据，真机待验 | [S3 两种横向分配包络及原生轮载](../results/suction_s3_2026-10-17/REPORT.md)；静止最低裕量 2.72；[S5](../results/suction_s5_2026-10-18/REPORT.md) 运动最低 1.625，原生经验下限 7500 Pa／984.70 N，逆动力学约 756 N 不能覆盖换向内力 |
| V-03 | 三风机单腔、裙边压缩／唇口反力、连续漏气及唯一施力路径 | S2 隔离、S3 平墙静止集成通过 | [S2 子模型](../results/suction_s2_2026-10-17/REPORT.md)、[S3 同插件 B/C 及互斥负对照](../results/suction_s3_2026-10-17/REPORT.md)；动态漏气及停机边界见 [S5](../results/suction_s5_2026-10-18/REPORT.md) |
| V-04 | 压力测量与真值分离，启动就绪释放支撑、暂停／reset 正确 | S3/S4 通过 | [S3 启动、超时、暂停/reset](../results/suction_s3_2026-10-17/REPORT.md)、[S4 原生传感器与真实桥接](../results/suction_s4_2026-10-18/REPORT.md)；误差统计、延迟、故障隔离及 reset 重放通过，参数仍为 provisional |
| V-05 | 正常漏气范围、故障倒灌扫描、数值收敛与边界 | 子模型与注册整车边界通过；失效点单列 | [S2 留出、240 个故障点与数值收敛](../results/suction_s2_2026-10-17/REPORT.md)；不利参数端点失败单列。风机曲线选型是等效标定；[S5 整车边界](../results/suction_s5_2026-10-18/REPORT.md) 已完成，全停 1 s 不能自行重新密封 |
| V-06 | 新候选完整验收与默认切换验证 | S6／S7 通过，已合并 main | [S6 最终报告](../results/suction_s6_2026-10-18/REPORT.md)：9 组双 GUI 控制、2680 张真实纹理照片、CUDA／人工复核及最终 1727 项零失败，总门禁 40 项通过。[S7](../results/suction_s7_2026-10-18/REPORT.md)：134 项契约及 37 项相关回归、默认双 GUI reset、70/70 张及 CUDA 拼接；RTF 0.988452。复用 S6，不重跑完整套件或最大横纵任务；范围见[计划](archive/suction/M1_DELIVERY_2026-10-19.md#s7-验证范围用户于-2026-10-18-最终调整) |
| V-07 | 同源表面、接触与连续贴合过渡 | M2 注册工况完成 | [T2 收口](../results/m2_t2_closure_2026-10-19/REPORT.md)、[M2 近似与边界](m2/MODEL.md)；驱动轮仅起伏高度场，不宣称真实陷槽验证 |
| V-08 | 柔顺轮、动态 TF 与整车耦合 | T1～T3 完成，默认开启 | [T1](../results/m2_wheel_compliance_2026-10-19/REPORT.md)、[T3](../results/m2_t3_2026-10-20/REPORT.md)；[450 N 上界](../results/m2_friction_2026-10-22/REPORT.md)保持 k/c |
| V-09 | 带噪声定位标定与 M2 图像对照 | 标定默认启用，拼接尺度候选不切默认 | [标定](../results/m2_localization_calibration_2026-10-22/REPORT.md)、[新整墙](../results/m2_calibrated_full_vertical_2026-10-22/REPORT.md)；缺测和残余接缝边界保留 |
| V-10 | 新默认 μ=0.8 | 横纵短任务及回归通过 | [摩擦报告](../results/m2_friction_2026-10-22/REPORT.md)；不扩大为新摩擦的整墙／全部故障验收 |

## 分册

- [路径、控制、侧滑、定位与工程质量](acceptance/CONTROL.md)。
- [相机 G1～G4、暂停恢复、预处理与拼接](acceptance/IMAGING.md)。
- [M2 当前判据与适用范围](m2/VALIDATION.md)。

旧结构、M1 和 M2 的数字分别按冻结配置解释，当前总体状态见[状态页](STATUS.md)。
