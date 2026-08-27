# ROS 2 与离线数据接口合同

更新：2026-08-27。本文只描述当前公共接口、关键语义和配置归属；完整历史参数推导见
[接口归档](archive/interfaces/INTERFACES_2026-08-27.md)。主线操作步骤见
[根 README](../README.md#快速启动)，实验变体与故障处置见 [OPERATION](OPERATION.md)，
架构职责见 [ARCHITECTURE](ARCHITECTURE.md)。

## 启动入口

| 入口 | 合同 |
| --- | --- |
| `climbot_wall.launch.py` | Gazebo、桥接、TF、传感器适配、EKF、速度看门狗 |
| `coverage_planner.launch.py` | 独立规划与可选 RViz，仅预览 |
| `coverage_sim.launch.py` | 仿真与规划联合预览，不启动执行器 |
| `coverage_mission.launch.py` | 完整任务：仿真、规划、执行、RViz 和可选巡检 |
| `coverage_executor.launch.py` | 多段 Action 执行器与跟踪器 |
| `inspection.launch.py` | G1/G2/G4 相机、触发、归档与可选平场预览 |

带归档的 launch 从 `CLIMBOT_DATA_ROOT` 取得 `inspection_output_root`；调用方也可传入绝对路径。
未配置时使用记录器主机当前用户的 `$HOME/climbot_data`，绝不把具体用户目录写入仓库。

## 坐标、控制与定位

| 名称 | 类型/方向 | 语义 |
| --- | --- | --- |
| `world → odom → base_link` | TF | `odom` 为墙面工作系；world 真值不能进入业务控制 |
| `/control/cmd_vel → /cmd_vel` | `Twist` | 控制层唯一入口；看门狗以 50 Hz 转发并负责停车 |
| `/odometry/filtered` | `Odometry` | EKF 输出；控制和巡检位姿绑定的唯一业务定位源 |
| `/wheel_odom`、`/imu_wall`、`/total_station/pose` | 传感器输入 | 分别提供轮速/偏航、墙面姿态、绝对位置 |
| `/model/climbot/ground_truth` | `Odometry` | 仅仿真传感器与独立评价使用 |
| `/control/hold_active` | `Bool` | transient-local 的执行保持状态 |

`localization_profile` 为 `precision` 或 `realistic`。棱镜残差与时间戳残差可以独立开关；它们的
实际值必须由评价器从节点参数服务写入 provenance。

## 规划与执行

| 名称 | 类型 | 合同 |
| --- | --- | --- |
| `/coverage/task` | `CoverageTask` | 已验证的任务几何、扫描线和版本 |
| `/coverage/configure` | service | 仅无运行任务时修改区域形状/扫描方向 |
| `/coverage/start`、`/coverage/cancel` | service | 受控启动/取消；归档预检失败不得发送运动 Goal |
| `/coverage/manager_status` | `CoverageStatus` | 管理器权威状态、任务版本、段进度、错误和归档摘要 |
| `/coverage/status` | `String` | 规划器的点选与规划结果说明，仅供界面显示，不是任务状态 |
| `ExecuteCoverage` | Action | 执行冻结的多段路径；反馈不等于重新规划 |
| `/control/execution_reference` | `ExecutionReference` | 当前实际直线与采集许可；不是名义预览路径 |

矩形点两次、等腰梯形点三次，均在 `odom` 中解释。运行时区域/方向与构造时参数不可混用；
精确字段、状态迁移和参数表由消息定义、launch 与 YAML 共同维护。

## 巡检与归档

| 名称 | 类型 | 合同 |
| --- | --- | --- |
| `/inspection/camera/image_raw`、`camera_info` | 图像/标定 | 原始 `mono8`；补偿图不能替代归档原图 |
| `/inspection/capture_metadata` | `InspectionCapture` | 成功图的任务、触发、冻结参考和曝光 EKF 相机位姿 |
| `/inspection/capture_gate` | `InspectionCaptureGate` | Reliable + transient-local 健康 heartbeat；失联受控停车 |
| `/inspection/archive/prepare`、`finalize` | service | 原子创建/封存 run；失败保留已提交证据且报告失败 |
| `/inspection/archive/status` | `InspectionArchiveStatus` | 归档权威状态、计数、目录和错误 |

G4 run 必须有 manifest、原图 SHA-256、每图标签和相机快照，且 `expected_images == saved_images`。
P1 仅接收完整 run，输出新的 processed-run；处理顺序固定为暗场、平场、可选去噪、去畸变。

## 离线拼接

`climbot_mosaic` 依次提供 `validate_mosaic_inputs`、`build_initial_projection`、
`build_overlap_candidates`、`build_local_matches`、`build_pose_graph` 和 `build_wall_mosaic`。
所有输入/输出目录必须为绝对路径；输入只读，输出不存在且原子发布。跨 run 帧键为
`(source_run_id, frame_index)`。

正式 mosaic 包含 pose-only/optimized 无损母版、差分、覆盖次数、不确定度、预览和严格 JSON
manifest。`evaluate_diagnostic_mosaic` 与 `inspect_diagnostic_mosaic` 是后验接口，不参与拼接决策。
精确 CLI、目录树、编码与失败码见 [拼接计划](MOSAIC_PLAN.md) 和各包 `--help`。

## 配置归属

| 范围 | 位置 |
| --- | --- |
| 机器人、墙面、相机几何 | `climbot_description/config/` |
| Gazebo、传感器与定位 profile | `climbot_gazebo/config/` 和 launch 参数 |
| 规划与控制 | `climbot_coverage/config/`、`climbot_control/config/` |
| 采集和归档 | `climbot_inspection/config/inspection.yaml` |

公共参数不得依赖用户目录；外部数据路径经 launch 参数或 `CLIMBOT_DATA_ROOT` 注入。
