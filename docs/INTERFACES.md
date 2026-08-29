# ROS 2 与离线数据接口合同

更新：2026-08-28。本文只描述当前公共接口、关键语义和配置归属；完整历史参数推导见
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
| `/coverage/pause`、`/coverage/resume` | service | 就地暂停/继续同一任务；不结束任务、不封存归档 |
| `/coverage/executor_pause` | service | 管理器到执行器的 `SetBool`；面板不直接调用 |
| `/coverage/manager_status` | `CoverageStatus` | 管理器权威状态、任务版本、段进度、错误和归档摘要 |
| `/coverage/status` | `String` | 规划器的点选与规划结果说明，仅供界面显示，不是任务状态 |
| `ExecuteCoverage` | Action | 执行冻结的多段路径；反馈不等于重新规划 |
| `/control/execution_reference` | `ExecutionReference` | 当前实际直线与采集许可；不是名义预览路径 |

暂停是任务内的一次停车，不是任务的结束：`task_id`、`revision`、段序号和归档 run 全程不变，
段超时与调度时基在暂停期间冻结，恢复后从当前位姿继续同一段。执行器在 `PAUSED` 期间发布的
`ExecutionReference` 带 `inspection_enabled=false`，采集门因此关闭；这与 transition 段是同一
种形状，采集节点不需要知道“暂停”这件事。关门的时点是机器人真正停住，不是暂停请求被接受：
`PAUSING` 期间机器人还在走刹车距离，那段路仍是这条扫描线，参考照发 `true`，否则落在其中的触发
目标会被跳过、恢复时在停住的位置补拍，从而违反归档的纵向重叠合同。Stop 的语义不变，暂停中依然
是取消本次任务。

`CoverageStatus.archive_default_root` 是管理器解析完显式 launch 参数、`CLIMBOT_DATA_ROOT` 和
`$HOME/climbot_data` 回退后的权威默认数据根。RViz 始终显示这个非空值；用户在面板中修改目录只会
覆盖该面板发出的 Start 请求，不修改进程环境变量，Default 恢复管理器发布的值。

矩形点两次、等腰梯形点三次，均在 `odom` 中解释。运行时区域/方向与构造时参数不可混用；
精确字段、状态迁移和参数表由消息定义、launch 与 YAML 共同维护。

## 巡检与归档

| 名称 | 类型 | 合同 |
| --- | --- | --- |
| `/inspection/camera/image_raw` | `sensor_msgs/Image` | 每次成功曝光的原始 `mono8`；补偿图不能替代归档原图 |
| `/inspection/camera/camera_info` | `sensor_msgs/CameraInfo` | reliable + transient-local 的会话级不可变标定；源相机侧仍逐次与图像同时间戳配对 |
| `/inspection/capture_receipt` | `std_msgs/Header` | G1 在正式图像与标定交给 DDS 后发布的轻量完成回执，供离线评价器独立清点曝光；G2 触发链**不消费**它 |
| `/inspection/capture_metadata` | `InspectionCapture` | 成功图的任务、触发、冻结参考和曝光 EKF 相机位姿 |
| `/inspection/capture_gate` | `InspectionCaptureGate` | Reliable + transient-local 健康 heartbeat；失联受控停车 |
| `/inspection/archive/prepare`、`finalize` | service | 原子创建/封存 run；失败保留已提交证据且报告失败 |
| `/inspection/archive/status` | `InspectionArchiveStatus` | 归档权威状态、计数、目录和错误 |

`automatic_capture_node` 对执行参考和里程计的回调间隔做定量观测：超过阈值的告警按 5 s 限流，
避免系统恶化时日志写入本身加剧问题；被限流掉的数字由 `AUTOMATIC_CAPTURE_TIMING summary` 按
`gap_summary_period_s` 补出，含样本数、最大值和 50/100/200/250 ms 阈值计数（阈值与
`climbot_control` 的 `kTimingThresholdNs` 一致，便于两侧对照）。统计是固定空间的，不保留逐条
记录；某路没有新间隔时不输出。

`/inspection/capture_once` 的响应携带成功曝光的 `Header`。它与调用方的 future 天然一一对应，
因此不存在把上一次被放弃的请求的回执记到下一次 pending 上的可能；成功但 `stamp` 为零的响应视为
故障并停掉该 SCAN。图像是否真的送达归档由 G4 的图像/metadata 配对和最终计数负责，不由触发侧推断。

`/control/execution_reference` 是采集侧的唯一活性信号：几何、状态或段号一变立即可靠发布，其余
时间按 `execution_reference_heartbeat_hz`（5 Hz）保活。它同时驱动两个监督计时器 ——
`automatic_capture_node.reference_timeout_s` 决定何时停止触发，`line_tracker` 的采集门在
`capture_gate_timeout_s` 判陈旧、再过同样时长判超时并停车。采集门由参考派生，送到跟踪器必然
晚于原参考送到采集节点，因此前者必须至少覆盖后者的两倍再加一个参考心跳周期；否则参考断流时
会出现"已停止拍照、机器人仍在走"的窗口，恢复后的第一张可能越过目标位置被归档拒绝。采集完成
不额外刷新采集门，使两个监督始终由同一条参考驱动。该合同由
`climbot_gazebo/test/test_inspection_contract.py` 跨包断言。

G4 按时间戳配对每张原图和 `InspectionCapture`，同时使用 transient-local 的最新会话标定；
标定内容在 run 内发生变化必须失败。run 必须有 manifest、原图 SHA-256、每图标签和相机快照，
且 `expected_images == saved_images`。
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
