# ROS 2 与离线数据接口合同

更新：2026-10-22。本文只描述当前公共接口、关键语义和配置归属；完整历史参数推导见
[接口归档](archive/interfaces/INTERFACES_2026-08-27.md)。主线操作步骤见
[快速启动](QUICKSTART.md#快速启动)，实验变体与故障处置见 [OPERATION](OPERATION.md)，
架构职责见 [ARCHITECTURE](ARCHITECTURE.md)。

## 启动入口

| 入口 | 合同 |
| --- | --- |
| `climbot_wall.launch.py` | 默认负压底盘、起伏墙、柔顺轮、Gazebo、桥接、TF、传感器适配、EKF、速度看门狗 |
| `climbot_vacuum_stack.launch.py` | 与 climbot_wall 共用实现的兼容入口 |
| `coverage_planner.launch.py` | 独立规划与可选 RViz，仅预览 |
| `coverage_sim.launch.py` | 仿真与规划联合预览，不启动执行器 |
| `coverage_mission.launch.py` | 完整任务：仿真、规划、执行、RViz 和可选巡检 |
| `coverage_executor.launch.py` | 多段 Action 执行器与跟踪器 |
| `inspection.launch.py` | G1/G2/G4 相机、触发、归档与可选平场预览 |
| `climbot_vacuum.launch.py` | 隔离物理诊断入口；正常任务使用完整默认 stack |

带归档的 launch 从 `CLIMBOT_DATA_ROOT` 取得 `inspection_output_root`；调用方也可传入绝对路径。
未配置时使用记录器主机当前用户的 `$HOME/climbot_data`，绝不把具体用户目录写入仓库。

当前模型选项：`wall_surface=undulating`、`wheel_compliance=true`；`traction_mu` 留空时读取
`simulation.yaml`，有效驱动摩擦取墙面与轮胎较小值，当前 0.8。历史 M1 几何对照不会自动恢复
旧摩擦。当前参数来源与接触近似见[M2 模型](m2/MODEL.md)。

### `/clock` 发布率

`clock_publish_hz` 默认 `500`。设为 `0` 时 bridge 直接把 Gazebo 每个物理步长发到 `/clock`；取正值时
bridge 改发 `/clock_raw`，由 `clock_throttle_node` 抽稀后发布 `/clock`；**物理步长不受影响**，
仿真本身一步不差。

只有能整除输入步长的速率是可达的。1 ms 步长对应的梯子是 1000、500、333.3、250、200 Hz……
落在两档之间的请求无法实现，会降到下一档（例如请求 400 Hz 实得 333.3 Hz）。节点在启动后
测量并打印 `CLOCK_THROTTLE measured input=… delivered=… requested=…`，请求与实得不符时告警——
日志报告的是实测值，不是请求值。

两条硬约束：

- 节点必须运行在墙上时间。它产生所有其他节点等待的时钟，若自己也用仿真时间就会等待自己的
  输出。`use_sim_time` 为真时节点拒绝启动。
- 节点退出即整体 `Shutdown()`。所有停滞检测器都按仿真时间老化，时钟一断它们随之冻结，
  系统会安静地停住而没有任何节点报错。

## 吸附与压力接口

默认 `robot_geometry=vacuum_chassis`，当前只接受这一几何；`adhesion_mode=vacuum` 为三机 C，
`fixed_pressure` 为同插件 B，`auto` 等价于 vacuum。legacy／legacy_constant 已移除且明确拒绝。
物理配置来自 vacuum_physics.yaml，启动／支撑来自 vacuum_runtime.yaml。

压力与故障配置来源为 `climbot_gazebo/config/vacuum_io.yaml`，启动参数 `io_config:=<YAML>` 可覆盖其中
`vacuum_io` 下的字段；传感器字段按键覆盖，其余字段整体覆盖，未知键、非法参数和未对齐物理步长
的采样周期／固定延迟均拒绝启动。默认仍为三台固定档位风机、同一个连通腔体，无压力闭环。

| ROS topic | ROS 类型 / Gazebo 类型 | 方向与语义 |
| --- | --- | --- |
| `/suction/pressure` | `sensor_msgs/FluidPressure` / `gz.msgs.FluidPressure` | GZ→ROS；腔内**绝压 Pa**，header 为采样时刻，frame 为 `chassis_link`，variance 单位 Pa² |
| `/suction/ambient_pressure` | 同上 | GZ→ROS；外界绝压，当前固定取 `vacuum_physics.yaml` 的 `gas.ambient_pa=101325`，带仿真时间戳，variance=0 |
| `/suction/fans/fan_{0,1,2}/enabled` | `std_msgs/Bool` / `gz.msgs.Boolean` | ROS→GZ；true 为固定档位启用，false 为关闭；不接收连续调速 |
| `/suction/fans/fan_{0,1,2}/failed` | 同上 | ROS→GZ；true 注入驱动失效，false 恢复；有效目标为 `enabled && !failed`，故恢复不覆盖操作者的关闭命令 |
| `/suction/sensor/fault` | `std_msgs/Int32` / `gz.msgs.Int32` | ROS→GZ；0 正常、1 丢报、2 卡值；其他值计入拒绝数且不改变状态 |
| `/suction/ground_truth/state` | `std_msgs/String` / `gz.msgs.StringMsg` | GZ→ROS；严格 JSON 评价快照，含 `time_s`、`generation`、腔压／环境压／压差、力矩、风机命令与实际转速、传感器故障及采样／送达状态 |

以上桥接采用 reliable、volatile、keep-last 10；评价客户端可增大本地队列。订阅者须发现实际
发布端再开始计数，volatile 不补发历史。单向桥接防止回环；真值 topic 仅供评价，未接入定位、
规划、控制或拼接输入。业务读取压差应使用 `Δp = p_a - fluid_pressure`，不能将绝压当吸力。

默认传感器 100 Hz、白噪声标准差 10 Pa、固定偏置 +25 Pa、固定延迟 20 ms、量程
20000～120000 Pa、种子 4817。这是 provisional 仿真配置，尚无真机参数依据。
采样在绝对仿真时间网格上进行：当前 1 ms 物理步下首次非暂停更新为 1 ms，首个采样为 10 ms，
首个送达为 30 ms；不会伪造 t=0 的测量。header 始终保留采样时间，送达时间另见真值快照。
噪声／偏置后按量程截断，`sensor_saturated` 标记饱和；variance 是配置的截断前噪声方差，
不包含偏置或饱和的误差。种子可复现性限定同一标准库／构建环境。

丢报模式继续推进采样计数与随机源但不发布；卡值冻结最近一次**采集**的值，仍发布新采样时间戳，
variance=0 仅描述注入的恒定读数，不表示真实压力无误差。切换故障时清空待送达队列，恢复后
重新经历固定延迟。所有测量误差和传感器故障均不修改物理腔压或施力。

风机关闭／失效采用同一转速一阶衰减，不模拟叶片机械卡死；停转口仍按原曲线倒灌，未增加止回阀。
B 的理想固定压力源保持设定压差，即使风机关闭；风机故障对腔压的验证必须使用 C。
Gazebo 传输回调只入队，下一个非暂停 PreUpdate 才应用；命令无源时间戳，生效时间以快照
`time_s` 与递增 `command_revision` 确认。入队最多 1024 条，溢出／非法故障计入
`rejected_commands`。不把重复发布当作同步确认。

同源配置的 `events` 支持可重复注入：`{time_s, kind, index, value}`，按 time_s 非降序；
kind 0=enabled、1=failed、2=sensor fault，风机 index=0..2，传感器 index=0。
到达事件时间的首个非暂停步执行，同步时刻按配置顺序执行，随后执行已排队的实时命令；
两入口共用应用函数。暂停不积分、不采样、不发布新状态、不应用队列。reset 恢复环境腔压、
零转速、三机启用／无失效、传感器正常、随机源、采样计数和延迟队列，并清除已入队命令，
事件表从头重放，`generation` 增一。无时间戳的实时命令若在 reset 后才到达，仍视为新命令。

独立评价 JSON 字段名来自 `vacuum_state.hpp`；`sensor_stamp_s=-1` 表示本代尚无送达测量。
物理域异常时 `runtime_failed=1` 锁存至 world reset，不恢复支撑或恒力。真值仍按原频率
发布递增 `time_s`，风机命令及一阶转速继续更新；`last_valid_time_s` 保留最后成功求解时间。
此时腔压、间隙及流量等求解字段仅为失效前快照，必须结合故障标志判无效；实际施加的
压力／裙边／支撑／滚阻力矩字段为零。压力传感器停止采样和送达，不把旧腔压加新时间戳
伪装为有效测量；保留的 `sensor_stamp_s` 不再推进。暂停规则不变，reset 清除失效状态。
`sensor_sequence` 是已采样数，`sensor_delivery_s` 是最近实际送达的仿真时刻；卡值、丢报和
真值变化均须保留原始记录。truth 默认 100 Hz，首步额外发布安装状态，随后按时间网格发布。
这些输出不新增速度控制通道或运行低压门禁，启动支撑仍只读取物理真值。

S5 原生试验补充：`vacuum_io.opening_schedule` 是独立试验漏口面积时间表，每行
`{time_s, area_m2}`，非空时从 t=0 开始、时间严格递增、面积在 0～0.02 m² 内；段间线性
插值，末值保持。它叠加到基础漏气和实际裙边开口上，暂停不推进、reset 重放；不表示墙面
新增碰撞沟槽。`vacuum_runtime.fixture_bench_hold=true` 显式保持安装支撑，仅供受约束压力／
流量台架；默认 false，不能作为自由整车保持证据。两项实际值均进入评价快照。
评价快照另记录施力实际读取的线／角速度（`force_input_*`）、漏气／风机正向／倒灌质量流量
及实际转速下最高关断压差。评估摩擦功率须使用同一施力输入状态，不能混用物理步后速度。


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

`line_tracker` 的融合里程计活性同时检查消息到达时间和 `Odometry.header.stamp` 的源时间：
源时间为零、重复或倒退、超过 `odometry_timeout_s`（默认 `0.25 s`）未更新，或超前当前 ROS
时钟多于 `odometry_future_tolerance_s`（默认 `0.05 s`）时均不能刷新有效位姿。这样持续重发
旧 EKF 位姿不能误当成定位仍新鲜；仿真时钟重置后旧 epoch 位姿也会被清除。

G4 按时间戳配对每张原图和 `InspectionCapture`，同时使用 transient-local 的最新会话标定；
标定内容在 run 内发生变化必须失败。run 必须有 manifest、原图 SHA-256、每图标签和相机快照，
且 `expected_images == saved_images`。
P1 仅接收完整 run，输出新的 processed-run；处理顺序固定为暗场、平场、可选去噪、去畸变。

归档内容记录 `summarize_archive_content` 是独立诊断，不在正式五阶段链内：它读一个或多个
归档 run 的原图，发布带 `stage_provenance.json` 的目录，内含 `archive_content.npz`（逐帧
16×16 块降采样、32 段行/列均值剖面、mean/std/laplacian variance）和 `archive_content_summary.json`
（分布、对参考 run 的比值与偏差、判定）。闸门有三层：run 级标量中位比值、run 级行列剖面偏差，
以及**按同序帧对齐的逐帧偏差**（单帧 max 与 p95 两道，容差由 5340 帧实测定出）。逐帧这一层是
必需的——run 级统计看不见单张坏帧（一张全黑帧只把 680 帧的中位数挪动可忽略的量）也看不见帧序
互换。块降采样只记录不判，因为 34 mm 的块对两次采集之间毫米级的位姿差敏感。帧数与参考不一致
直接拒绝，不截断对齐。记录格式版本为 `2`，输入里的归档哈希带 run 名，便于按身份而非列表顺序
绑定。**`build_mosaic_evidence_summary.py` 强制要求这些记录**：校验其 provenance、traceable、
输出回读哈希和"全部匹配"判定，并要求每个冻结任务的归档 manifest 哈希出现在某份记录里——
作为通过闸门的输入，或作为该记录自身的参考基准。它存在的理由是：两组数据"内容一致"这句话
必须是可复核的产物，而不是文档里的结论——曾有两整组数据在计数守卫全绿的情况下整幅 z-fighting。

归档 manifest 的 `archive_format_version` 现为 `2`，新增 `render_environment`：`declared`
是 launch 传入的 `headless` / `gpu_backend` / `gui_gpu_backend`，`observed_environment` 是
记录器进程实测的 `GALLIUM_DRIVER`、`MESA_D3D12_DEFAULT_ADAPTER_NAME` 和 `DISPLAY`
（`gpu_backend:=auto` 在仿真 launch 内解析，所以环境才是权威值），外加 `observed_gazebo_gui_process`：
写每次 manifest 时扫本机是否存在活着的 `gz sim -g` 客户端。它是**观测**，不是声明——`headless:=false`
只说明请求启动 GUI，而 `DISPLAY` 在无头采集下同样存在，两者都不能证明 GUI 真的活着。渲染条件在图像里看不出来，
却决定两次采集是否可比，冻结门限必须能查到它。脱离仿真 launch 单独起记录器时 `declared`
为 `unknown`。P1 同时接受 `1` 和 `2`：已冻结进 P2-06 链的归档是 `1`，必须保持可处理。

## 离线拼接

`climbot_mosaic` 依次提供 `validate_mosaic_inputs`、`build_initial_projection`、
`build_overlap_candidates`、`build_local_matches`、`build_pose_graph` 和 `build_wall_mosaic`。
所有输入/输出目录必须为绝对路径；输入只读，输出不存在且原子发布。跨 run 帧键为
`(source_run_id, frame_index)`。

正式 mosaic 包含 pose-only/optimized 无损母版、差分、覆盖次数、不确定度、预览和严格 JSON
manifest。`evaluate_diagnostic_mosaic` 与 `inspect_diagnostic_mosaic` 是后验接口，不参与拼接决策。
精确 CLI、目录树、编码与失败码见 [拼接计划](MOSAIC_PLAN.md) 和各包 `--help`。

`build_wall_mosaic --backend cpu|cuda|auto` 只选择第 4 阶段执行设备，不改变输入、网格、候选、
hard-cut 平局规则或输出 schema。默认 `cpu`；`cuda` 要求已构建自定义 CUDA 扩展；`auto` 只对
`cuda_unavailable` 与 `cuda_runtime_failure` 从头回退 CPU，`input_contract` 和
`backend_independent_failure` 必须原样失败。CUDA OpenCV 不是该接口的运行依赖。

## 配置归属

| 范围 | 位置 |
| --- | --- |
| 机器人、墙面、相机几何 | `climbot_description/config/` |
| Gazebo、传感器与定位 profile | `climbot_gazebo/config/` 和 launch 参数 |
| 规划与控制 | `climbot_coverage/config/`、`climbot_control/config/` |
| 采集和归档 | `climbot_inspection/config/inspection.yaml` |

公共参数不得依赖用户目录；外部数据路径经 launch 参数或 `CLIMBOT_DATA_ROOT` 注入。
