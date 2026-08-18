# ROS 2 接口与配置索引

本文档记录当前已实现接口，以及已经冻结但尚未实现的阶段 E 第一版任务与控制接口。

## 启动入口

| 命令 | 用途 |
| --- | --- |
| `ros2 launch climbot_gazebo climbot_wall.launch.py` | Gazebo、桥接、TF、传感器适配和 EKF |
| `ros2 launch climbot_coverage coverage_planner.launch.py` | 独立覆盖规划器和可选 RViz |
| `ros2 launch climbot_bringup coverage_sim.launch.py` | 联合启动仿真、规划器和 RViz；**不含执行器**，只能预览 |
| `ros2 launch climbot_bringup coverage_mission.launch.py` | 完整任务入口：仿真、规划器、RViz、跟踪器和管理器 |
| `ros2 launch climbot_control line_tracker.launch.py` | 单段直线跟踪器；从共享描述注入轮距和轮缘硬限值 |
| `ros2 launch climbot_control coverage_executor.launch.py` | 多段覆盖 Action 执行器；不接入 Nav2 |

已提供两个参数式完整任务演示配置：

| 配置 | 区域 | 路径 |
| --- | ---: | --- |
| `coverage_vertical_demo.yaml` | `3.30 × 4.50 m` | 8 条竖向扫描线、15 段 |
| `coverage_horizontal_demo.yaml` | `4.30 × 1.70 m` | 4 条横向扫描线、7 段 |

联合启动时通过 `config_file:=<配置绝对路径>` 选择演示；同时启动
`coverage_executor.launch.py use_sim_time:=true`，再由 Action 客户端发送规划器发布的
`/coverage/task`。

`coverage_mission.launch.py` 把上述两条命令合成一条，默认 `input_mode:=rviz`，
用于操作员在 RViz 中点选区域后手动启动执行：

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `headless` | `false` | 不启动 Gazebo GUI；RViz 仍然启动 |
| `gpu_backend` | `auto` | 透传给 `climbot_wall.launch.py` |
| `rviz` | `true` | 启动 RViz；点选工具由它提供 |
| `input_mode` | `rviz` | `rviz` 点选或 `parameters` 配置区域 |
| `planner_config_file` | `coverage_rectangle.yaml` | 规划器参数；点选模式下忽略其中的角点 |
| `control_config_file` | `control.yaml` | 跟踪器参数 |
| `region_type` | `rectangle` | `rectangle` 需两点，`trapezoid` 需三点 |
| `sweep_direction` | `horizontal` | 扫描方向 |

launch 参数只给这两项定初值。运行中改用 `/coverage/configure` 或面板上的
Region / Sweep 下拉框，不必重启，见下面的"运行时构型"。`ros2 param set` 仍然不
生效——参数只在构造时读一次。

两个参数文件名字不同且各自显式传递。被包含的 launch 会继承父作用域的同名参数，
且 `DeclareLaunchArgument` 的默认值对父作用域已设定的名字不生效，因此一个共用的
`config_file` 会同时落到规划器和跟踪器上，使跟踪器静默退回内置默认值。

点选模式下的操作顺序是：用 RViz 的 `Publish Point` 工具依次点击区域角点
（矩形为左下、右上；梯形再加右下），确认 `Coverage Task` 面板的 State 变为
`Ready`，再点面板上的 `Start`（等价于调用 `/coverage/start`）。`/coverage/status`
会回显每次点击被接受时的坐标，可据此发现相机视角造成的镜像或旋转误选。
角点点错时用面板的 `Clear points` 或 `/coverage/clear_points` 清空重来。
RViz 的固定坐标系是 `odom`，即墙面平面，所以点选工具给出的 `x`、`y` 就是墙面
坐标；规划器忽略 `z`。

`climbot_wall.launch.py` 的主要 launch 参数：

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `use_sim_time` | `true` | 使用 Gazebo 时钟 |
| `headless` | `false` | 无 GUI 启动 Gazebo server |
| `gpu_backend` | `auto` | `auto`、`wsl_d3d12` 或 `native` |
| `total_station_rate_hz` | `12.0` | 模拟全站仪频率 |
| `total_station_stddev_m` | `0.001` | 全站仪位置一倍标准差 |
| `total_station_delay_s` | `0.05` | 固定传输延迟 |
| `wheel_forward_velocity_stddev_mps` | `0.03` | 轮式前向速度标准差 |
| `wheel_yaw_rate_stddev_rps` | `0.05` | 轮式角速度标准差 |
| `imu_orientation_stddev_rad` | `0.00174532925` | IMU 姿态标准差（0.1°） |

## 仿真与定位话题

| 话题 | 类型 | 生产者 | 消费者/用途 |
| --- | --- | --- | --- |
| `/control/cmd_vel` | `geometry_msgs/msg/Twist` | E3 直线控制器、未来任务状态机 | E4 速度看门狗；控制层唯一入口 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | E4 速度看门狗 | Gazebo DiffDrive；每 `20 ms` 重发 |
| `/model/climbot/ground_truth` | `nav_msgs/msg/Odometry` | Gazebo | `header.frame_id=world`；仅模拟传感器和评价 |
| `/model/climbot/odometry` | `nav_msgs/msg/Odometry` | Gazebo DiffDrive | 诊断、轮式协方差适配 |
| `/wheel_odom` | `nav_msgs/msg/Odometry` | `wall_wheel_odom_adapter` | EKF 的前向速度和偏航角速度 |
| `/imu` | `sensor_msgs/msg/Imu` | Gazebo | 诊断、IMU 适配 |
| `/imu_wall` | `sensor_msgs/msg/Imu` | `wall_imu_adapter` | EKF 姿态和角速度 |
| `/total_station/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | `total_station_sim` | EKF 绝对位置 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | `robot_localization/ekf_node` | 融合位姿和未来控制反馈 |
| `/joint_states` | `sensor_msgs/msg/JointState` | Gazebo bridge | `robot_state_publisher` |
| `/contact/left_wheel` | `ros_gz_interfaces/msg/Contacts` | Gazebo bridge | 法向载荷评价 |
| `/contact/right_wheel` | `ros_gz_interfaces/msg/Contacts` | Gazebo bridge | 法向载荷评价 |
| `/contact/caster` | `ros_gz_interfaces/msg/Contacts` | Gazebo bridge | 法向载荷评价 |

`/model/climbot/ground_truth` 是 Gazebo world 坐标真值，不得成为轨迹控制器输入。
模拟全站仪和所有评价工具使用前必须将它转换到墙面 `odom` 工作坐标系；模拟全站仪
还必须经过频率、噪声和延迟模型。

`climbot_wall.launch.py` 始终启动 `cmd_vel_watchdog_node`，它是唯一的执行器控制
输出者：以 `50 Hz` 从
`/control/cmd_vel` 转发到 Gazebo `/cmd_vel`，在未收到首条指令、仿真时钟倒退，或
距最后一条指令超过 `command_timeout_s`（默认 `0.40 s`）时发布零速。键盘、四个实验
脚本和自动控制器均发布 `/control/cmd_vel`。当前采用单一上游来源规则；需要遥控抢占
自动任务时再增加带明确优先级的仲裁器。

### 控制环时钟

`line_tracker_node`、`cmd_vel_watchdog_node` 和 `coverage_manager_node` 的定时器
和**所有时长测量**都不走节点默认时钟，而是走 `climbot_control/control_clock.hpp`
里的 `controlClock()`：

| `use_sim_time` | 用哪个时钟 | 理由 |
| --- | --- | --- |
| `true` | 节点时钟（`RCL_ROS_TIME`，跟 `/clock`） | 仿真时间是被控对象自己的时间轴，控制环必须跟着它走 |
| `false` | `RCL_STEADY_TIME` | 唯一不会被设置、不会倒退的时钟 |

节点默认时钟是 `RCL_ROS_TIME`，**在仿真时间未激活时会退化为系统时钟**。系统时钟
可以被设置、可以往回跳（WSL2 每约 30 s 对宿主机重同步一次，实测回跳
`1.2～1.7 s`）。建在它上面的定时器在回跳期间**根本不触发**——跟踪器整段时间一条
指令都不发，而机器人还按最后一条指令在走；同时所有 `now()` 差值都会**变小**，
把"数据过期"这类判断压下去而不是报出来。

消息时间戳（`/control/reference_path`、`/coverage/manager_status` 的 `header.stamp`）
仍用 ROS 时间，保证与 TF 和 bag 对齐。

`use_sim_time=true` 时 `controlClock()` 返回的就是节点时钟本身，仿真行为与改动前
逐位一致（实测：`/clock` 按 0.5× 真实时间推进时，控制环仍为 `49.7 Hz` 仿真时间 /
`25.0 Hz` 墙钟）。

当前 EKF 以 `50 Hz` 发布 `/odometry/filtered`，阶段 E 控制器默认也以 `50 Hz`
运行。全站仪 `12 Hz` 只表示绝对位置更新频率，控制器不得将每个 50 Hz EKF
输出误认为新的独立绝对测量。

`evaluate_coverage_execution.py` 的 `execution_timeout_s` 默认是 `120 s`，这是评价工具
等待整个 Action 的墙钟超时，只适合紧凑回归用例，不是控制器的运动安全限制。上述
大矩形演示应显式设置 `execution_timeout_s:=600.0`；若评价工具超时，它会主动取消
Action。控制器自身仍按每段 `segment_timeout_s` 独立执行安全停车判定。

评价器按 Gazebo 真值位置与航向栅格化实际
`detection_width × detection_length` 矩形检测足迹，只累计正式 `SCAN` 直线，不把
转向、换道或小弧线入轨误算成覆盖。`minimum_actual_coverage_ratio` 默认 `0.95`，
`coverage_grid_resolution_m` 默认 `0.01 m`；低于门限时进程返回失败。该门限于
2026-08-14 由 `0.98` 放宽到 `0.95`，见 §14.3。
`trajectory_csv` 和 `summary_json` 默认为空，显式配置后分别保存完整真值/融合轨迹和
机器可读的 Action、逐段误差、覆盖/漏扫面积摘要。两者都在 `try/finally` 中写出：
超时或异常时同样落盘，摘要里 `completed=false`、`passed=false` 并记录
`failure_reason`，因此最需要现场数据的失败运行不会丢数据。摘要还带 `provenance`
段，记录代码提交、分支、`src` 树是否有未提交改动、评价器全部参数和执行任务的
名义几何，满足 §12 与 §14.6 对可追溯性的要求。

轨迹 CSV 由 `climbot_gazebo.trajectory_io` 写出，**文件名以 `.gz` 结尾就自动 gzip
压缩**，数值一律取到 `1e-6`（米/弧度/秒下的微米、微弧度、微秒）。§14.3 的验收
阈值都以毫米和度表述，原先写满 17 位有效数字只是把浮点往返噪声也存了下来；两项
合计使归档缩小约 `10×`。读取端用同一模块的 `read_trajectory()`，`.csv` 和
`.csv.gz` 都能直接读，不需要先解压。评价器同时按冻结后的动态直线参考
检查终点位置和转向结束航向，默认门限分别为 `maximum_endpoint_error_m=0.030`、
`maximum_turn_end_heading_error_deg=2.0`；水平 `SCAN` 的首末真值高度差默认不得超过
`maximum_horizontal_height_drift_m=0.030`。

摘要还输出 `scan_line_spacing`：每条正式 `SCAN` 的真值位置投影到统一法向轴后，
相对名义扫描线的偏移、相邻扫描线的间距误差，以及两者的最大值。相邻扫描线方向
相反，各自的法向会翻转，因此必须投影到同一条轴上才能比较。该项由
`maximum_scan_line_spacing_error_m`（默认 `0.020`，即 §14.3 阈值）判定；扫描线
少于两条时该指标无定义，不参与判定。横轨误差是相对冻结后的执行参考算的，无论
那条线被平移到哪里都很小，因此它无法替代这项检查。

转向结束航向误差按 §14.5 用真值度量：先由该段首个跟踪采样点的
`filtered_yaw_rad + heading_error_rad` 还原控制器当时瞄准的重力补偿后目标航向，
再用 `truth_yaw_rad` 与之比较。控制器估计只用于定义目标，不充当被测量，因此
EKF 航向漂移会体现在该指标中，而不是被它掩盖。摘要还记录实际/名义线段总长之比、最大
机体航向补偿角和正式直线期间最大指令角速度，当前只报告后面三项，不设臆测门限。

起点进入是**一条**直线,直接开到首个路点。它在原地对准稳定后把起点重设为机器人的
实际位置：直线是在对准之前捕获的,这次对准的下坠否则会成为该段的初始横轨误差,而
进入段很短,横轨修正会在 `max_heading_correction` 上饱和仍追不回来。

进入段的**终点按首条扫描线上那次转向的预计下坠抬高**,用的就是换道段那套
`reservedTurnDrop()` 不动点,所以机器人转完正好落在扫描线上。抬高后的点越出
`motion_region` 时逐档缩短,取仍在区域内的最大抬升。

这取代了原先的"跑道点"——首条扫描线后方 `0.40 m` 的同向进入点。跑道点让转向发生在
扫描线**外面**,确实有效,但它是为同一个问题准备的第二套机制,而且当机器人本来就在
首个路点的下游时,会强迫它先开过头再掉头回来(实测约 `0.8 m` 加一次 `180°` 掉头的
纯绕路)。预留不需要绕路,而且覆盖了跑道点**放不下**的场景——那种情况以前只能拒绝。

接受目标阶段仍校验
`start_approach_tolerance_m + 未被预留的下坠的法向分量 ≤ maximum_scan_offset_m`,
不满足即以 `TRACKING_FAILED` 拒绝并报出具体数值。只计法向分量：转向下坠沿重力方向,
对水平扫描线是整段平移,对竖向扫描线则是沿轨位移,不构成间距误差。

单段调试节点发布 `/control/segment_complete`（`std_msgs/msg/Bool`，reliable、
transient local、depth 1）。启动时为 `false`，同时满足终点位置、补偿后航向和停车
速度判据后锁存为 `true`。这是接入 Action 前的单段诊断接口；完整任务以
`/coverage/execute` 的 Result/Feedback 为权威。

### 控制安全参数

| 节点 | 参数 | 默认值 | 行为 |
| --- | --- | ---: | --- |
| `line_tracker` | `odometry_timeout_s` | `0.25 s` | 超过该时间未收到有效融合位姿时持续发布零速，并清空上一控制指令 |
| `line_tracker` | `standalone_mode` | `true` | `true` 执行参数单段；Action launch 将其设为 `false` |
| `line_tracker` | `segment_timeout_s` | `120 s` | 单段超过该时间则停车并以 `CONTROL_TIMEOUT` 终止任务 |
| `line_tracker` | `motion_region_tolerance_m` | `0.02 m` | 融合位置越出安全运动区域的数值容差 |
| `line_tracker` | `turn_slip_per_degree_m` | `0.0005 m/°` | 横向换道终点为第二次转向预留的标定下滑系数；**壁面相关**，随摩擦与 WheelSlip 变化，换墙必须用 `measure_turn_slip.py` 重标定 |
| `line_tracker` | `parallel_scan_offset_m` | `0.045 m` | 首条或转后偏差不超过此值时直接冻结实测位置对应的平行扫描线 |
| `line_tracker` | `maximum_scan_offset_m` | `0.12 m` | 转后可尝试单次前进小弧线入轨的最大法向偏差 |
| `line_tracker` | `arc_entry_finish_offset_m` | `0.012 m` | 小弧线结束、重新对齐并冻结平行扫描线的法向偏差门限；直接决定该扫描线冻结在何处，因此必须落在 §14.3 的 `20 mm` 间距预算内 |
| `line_tracker` | `arc_entry_speed_mps` | `0.08 m/s` | 带重力侧滑前馈的小弧线恒定前进速度；采集保持关闭 |
| `line_tracker` | `arc_entry_lookahead_m` | `0.20 m` | 小弧线航向引导前视距离 |
| `line_tracker` | `arc_entry_max_heading_deg` | `20°` | 小弧线相对名义扫描方向的最大航向修正 |
| `line_tracker` | `arc_entry_max_angular_speed` | `0.25 rad/s` | 小弧线最大角速度 |
| `line_tracker` | `arc_entry_timeout_s` | `15 s` | 小弧线未收敛时的停车失败门限 |
| `line_tracker` | `cruise_speed` | `0.20 m/s` | 扫描和换道期望巡航速度 |
| `line_tracker` | `max_linear_speed` | `0.25 m/s` | 控制器线速度上限，高于巡航值以容纳上爬打滑 |
| `line_tracker` | `visible_oscillation_amplitude_m` | `0.03 m` | 仅记录肉眼可见幅度的横轨往复；小误差反复过零不算故障 |
| `line_tracker` | `control_frequency_hz` | `50 Hz` | 直线跟踪控制频率 |
| `line_tracker` | `cross_gain` | `1.0 rad/m` | 横轨比例反馈增益 |
| `line_tracker` | `cross_integral_gain` | `0.30 rad/(m·s)` | 横轨积分反馈增益 |
| `line_tracker` | `cross_integral_limit_m_s` | `0.10 m·s` | 横轨积分状态绝对限值 |
| `line_tracker` | `cross_slowdown_start_m` | `0.03 m` | 超过此横轨误差后开始连续降速 |
| `line_tracker` | `cross_slowdown_full_m` | `0.08 m` | 到此误差后保持最小缩放速度，不完全停车 |
| `line_tracker` | `cross_slowdown_min_scale` | `0.25` | 大横轨误差下的最小线速度比例 |
| `line_tracker` | `max_gravity_feedforward_deg` | `8°` | 重力前馈独立限幅 |
| `line_tracker` | `max_cross_feedback_deg` | `8°` | 横轨 PI 反馈独立限幅 |
| `line_tracker` | `max_heading_correction_deg` | `12°` | 前馈与反馈相加后的总航向硬限幅 |
| `line_tracker` | `alignment_tolerance_deg` | `2°` | 进入直线跟踪前的航向稳定误差 |
| `line_tracker` | `alignment_settle_duration_s` | `0.50 s` | 航向和角速度连续稳定时间 |
| `line_tracker` | `alignment_threshold_deg` | `10°` | 直线前进许可门限 |
| `line_tracker` | `alignment_reentry_threshold_deg` | `12°` | 超过时停车并重新进入 `ALIGN` |
| `line_tracker` | `max_turn_angular_speed` | `0.60 rad/s` | 原地转向曲线峰值角速度 |
| `line_tracker` | `max_turn_angular_acceleration` | `1.00 rad/s²` | 原地转向曲线角加速度 |
| `line_tracker` | `turn_heading_gain` | `2.0` | 转向参考航向闭环增益 |
| `line_tracker` | `final_approach_distance_m` | `0.05 m` | 进入终点低速收敛的剩余沿轨距离；只是护栏，停车精度由制动剖面负责 |
| `line_tracker` | `final_approach_speed_mps` | `0.08 m/s` | 终点收敛线速度上限，在横轨积分更新之后施加 |
| `line_tracker` | `goal_position_tolerance_m` | `0.03 m` | 完成时二维终点距离严格门限 |
| `line_tracker` | `goal_position_exit_tolerance_m` | `0.04 m` | 终点候选状态退出门限 |
| `line_tracker` | `start_approach_tolerance_m` | `0.05 m` | 采集关闭的起点进入位置门限 |
| `line_tracker` | `start_approach_exit_tolerance_m` | `0.06 m` | 起点进入候选状态退出门限 |
| `line_tracker` | `goal_heading_exit_tolerance_deg` | `3°` | 航向候选状态退出门限；严格门限沿用 `2°` |
| `line_tracker` | `stopped_linear_speed_mps` | `0.01 m/s` | 完成时融合线速度上限 |
| `line_tracker` | `stopped_angular_speed_rps` | `0.02 rad/s` | 完成时融合角速度上限 |
| `line_tracker` | `goal_settle_duration_s` | `0.30 s` | 位置、航向和停车状态稳定时间 |
| `line_tracker` | `max_linear_deceleration` | `0.25 m/s²` | 速率限幅器的减速权限 |
| `line_tracker` | `braking_profile_deceleration` | `0.12 m/s²` | 距离-停车剖面使用的减速度，必须小于 `max_linear_deceleration`：两者相等时指令永远滞后剖面一步，每个段终点都会冲过去 |
| `cmd_vel_watchdog` | `command_timeout_s` | `0.40 s` | 上游速度指令超时后持续发布零速 |
| `cmd_vel_watchdog` | `publish_rate_hz` | `50 Hz` | 向执行器重发安全速度的频率 |

两个节点都拒绝非有限输入。直线跟踪器在启动时校验线段、增益、速度、加速度、
轮距、重力方向和坐标系参数；非法配置直接启动失败，不进入周期回调。轮距、轮缘
速度硬限值和轮缘加速度硬限值不存放在 `control.yaml`，由标准 launch 从
`climbot_description/config/robot.yaml` 注入。

横轨积分只在直线前进时更新；定位超时会清零。反馈或总航向修正达到限幅时采用
条件积分抗饱和，但重力前馈达到自身独立限幅不会阻止 PI 修正剩余误差。

`ALIGN` 先制动到线速度为零，再跟踪自动选择的三角形/梯形角速度曲线；曲线结束后
在 `2°` 航向容差内稳定 `0.50 s` 才允许直行。`10°/12°` 的退出/重入门限提供迟滞。
终点低速段只在空间剩余距离触发；完成判据同时使用二维位置、补偿后航向和
`/odometry/filtered` 的融合速度，不使用预定运行时间。完成后状态和零速输出锁存。

## 覆盖规划接口

### 话题

| 话题 | 类型 | QoS | 含义 |
| --- | --- | --- | --- |
| `/clicked_point` | `geometry_msgs/msg/PointStamped` | depth 10 | RViz `Publish Point` 输入 |
| `/coverage/path` | `nav_msgs/msg/Path` | transient local, depth 1 | 墙面覆盖路径；失败或清空时发布空路径 |
| `/coverage/markers` | `visualization_msgs/msg/MarkerArray` | transient local, depth 1 | 墙面、原始区域、有效区域、路径和方向 |
| `/coverage/status` | `std_msgs/msg/String` | transient local, depth 1 | 等待、成功和失败原因 |

所有覆盖接口默认使用 `odom`。RViz 点选消息的 `frame_id` 不匹配时会被拒绝。

### 服务

| 服务 | 类型 | 行为 |
| --- | --- | --- |
| `/coverage/clear_points` | `std_srvs/srv/Trigger` | 清除点选，并发布空 Path/Marker |
| `/coverage/replan` | `std_srvs/srv/Trigger` | 使用当前区域重新规划 |

任务管理器已实现以下任务操作接口：

| 服务 | 类型 | 行为 |
| --- | --- | --- |
| `/coverage/start` | `std_srvs/srv/Trigger` | 锁定管理器当前显示的有效 `task_id + revision`，校验后发送 `/coverage/execute` Goal |
| `/coverage/cancel` | `std_srvs/srv/Trigger` | 请求取消当前 Goal；控制器确认停车后返回 |

### 管理器状态话题

| 话题 | 类型 | QoS | 内容 |
| --- | --- | --- | --- |
| `/coverage/manager_status` | `climbot_interfaces/msg/CoverageStatus` | reliable、transient local、depth 1 | 操作界面所需的全部信息 |

界面所需的一切都在这一个话题上，界面因此不持有任何自己的状态，也不可能和管理器
对当前在跑什么产生分歧。这是 §11.1「面板只负责人机交互」的落地方式：
`climbot_rviz_plugins/Coverage` 面板就只订阅这一个话题并调用下面四个服务。

| 字段 | 含义 |
| --- | --- |
| `state` | 管理器状态：`IDLE` / `INVALID` / `READY` / `STARTING` / `EXECUTING` / `FINISHED` |
| `task_id`、`revision` | 已缓存或正在执行的任务标识；`task_id` 为空且 `revision` 为 `0` 表示从未收到任务 |
| `current_segment` | 执行器上报的当前段；接近首点期间为 `-1`，仅在 `EXECUTING` 有意义 |
| `total_segments` | 来自缓存任务，从 `READY` 起即可用 |
| `progress` | 按预计耗时加权的完成比例，`0`～`1`；起点进入期间固定为 `0`，因为进入段不是任务段 |
| `executor_state` | 执行器运动状态，取值与 `ExecuteCoverage.action` 反馈一致；仅在 `EXECUTING` 有意义 |
| `result_code` | 上一次执行的结果码，取值与 Action 结果一致；仅在 `FINISHED` 有意义 |
| `can_start`、`can_cancel` | 管理器当前是否接受该请求，由它自己服务的前置条件计算；界面直接渲染，不从 `state` 反推 |
| `message` | 与管理器日志同一行文本 |

`can_start` 和 `can_cancel` 是提示不是保证：请求仍可能被拒绝（例如执行器 Action
服务未运行），拒绝原因在服务响应里。界面不得据此认为请求一定成功。

### 操作在各状态下的行为

| 状态 | Start | Cancel | Replan / Clear points |
| --- | --- | --- | --- |
| `IDLE` | 拒绝，无有效任务 | 拒绝，无执行中任务 | 允许；`Replan` 在点选模式下需先选够点 |
| `INVALID` | 拒绝，无有效任务 | 拒绝 | 同上 |
| `READY` | **接受**，发送 Goal | 拒绝 | 允许，重新生成预览 |
| `STARTING` | 拒绝，已有任务在启动 | 拒绝，Goal 仍在接受中 | 允许；只改预览，不影响在途 Goal |
| `EXECUTING` | 拒绝 | **接受**，请求取消 | 允许；只改预览，运行中的任务继续 |
| `FINISHED` | **接受**（若仍有缓存任务），重跑该任务 | 拒绝 | 允许 |

执行中收到新预览（重新规划、清除点选，或在 RViz 里重新点选）只更新缓存，不改变
`state`，也不改变 `task_id`/`revision`——它们始终标识**当前 `state` 所描述的那个
任务**。否则运行中的机器人会被报成 `Ready` 或 `Idle`，取消按钮随之消失。执行器
在接受 Goal 时已复制任务，运行中的任务本身不受预览影响。

规划失败与"未选择区域"在管理器看来都是空任务，它无法区分，因此都报
`IDLE: no coverage region selected.`。真正的原因只在规划器的 `/coverage/status`
上，面板因此单独显示该话题。

已接受的 Goal 只由结果回调结束，而执行器崩溃时结果永远不会到达。管理器以
`executor_timeout_s`（默认 `5.0 s`）监视 Action 服务的存在：持续消失超过该时间即
释放该 Goal，报 `FINISHED` 与 `CONTROL_TIMEOUT`，操作员无需重启管理器即可重新开始。
实测从 `SIGKILL` 到释放约 `25 s`，其中约 `20 s` 是 DDS 摘除已死参与者所需的时间，
正常退出会快得多。这期间机器人已由速度看门狗停住——指令一停它就发零速。

`message` 与日志共用一份措辞，命令行观察等价于原来的 `std_msgs/String`：

```bash
ros2 topic echo /coverage/manager_status --field message
```

状态转换由管理器发布，执行器反馈按 `feedback_publish_period_s`（默认 `0.2 s`）
限频转发。反馈本身以控制环频率到达，任何显示都不需要那个速率，日志也不应承载它。

`progress` 按每段的**预计耗时**加权，而不是按段数或路程：

```text
progress = (已完成各段预计耗时 + 当前段已完成部分) / 全任务预计耗时
```

每段预计耗时 = 原地转向时间（复用控制器自己的梯形角速度曲线 `planTurn`）+ 稳定
时间 + 直线行驶时间（加速、巡航、减速三段）。这些量全部来自已有控制参数，没有
另行标定的常数。段内插值同样分两段：转向部分按对准曲线的实际进度推进，直线部分
按沿轨距离推进。

按段数等权会让 `0.44 m` 的换道段和 `4.5 m` 的扫描段各占 `1/N`，实测进度条在换道
段的推进速率是扫描段的 `3.2` 倍。按路程加权则反过来——换道段的时间大半花在原地
转向上，路程几乎为零，进度条会停住。改为耗时加权后实测两者速率比为 `1.16`。

`/coverage/start` 不会使规划器一发布任务就自动运动。管理器拒绝空任务、已有任务
执行中或 Action 服务不可用的开始请求，并在响应和 `/coverage/manager_status` 中给出
锁定的任务版本。管理器复制该任务作为不可变 Goal，因此预览更新不会改写执行中的
任务；定位可用性和边界安全性仍由执行器在实际动作前持续检查。后续 RViz 面板调用
同一组接口，不绕过管理器直接控制机器人。

发出 Goal 后管理器进入"等待执行器应答"状态并拒绝新的开始请求。若执行器在应答前
崩溃或被挂起，该应答永不到达，因此这个状态带 `start_response_timeout_s`（默认
`5.0 s`）超时：超时后管理器在 `/coverage/manager_status` 报告一次并接受新的开始
请求，不需要重启管理器。超时只释放"等待应答"，已被接受的 Goal 仍只能由
`/coverage/cancel` 停止。

### 运行时构型

`region_type` 和 `sweep_direction` 可以在运行中改,不必重启 launch。二者由
`/coverage/configure`（`climbot_interfaces/srv/ConfigureCoverage`）**一次原子设定**,
现行值发布在 latched 的 `/coverage/config`（`CoverageConfig`）上。

刻意不做成"先 `ros2 param set` 两个参数、再调 `/coverage/replan`":那样在两步之间
存在半配置状态,调用方中途死掉会把规划器留在那里,第二个客户端也能插进来。

请求里**留空的字段表示不改动**,所以只改扫描方向的客户端不必重述它以为的形状,
也就不会覆盖别人刚做的修改。响应带回**实际生效**的构型（接受与否都带）,调用方
据此立即同步显示,不必等话题。

| `CoverageConfig` 字段 | 含义 |
| --- | --- |
| `region_type` / `sweep_direction` | 现行构型 |
| `input_mode` | `rviz` 或 `parameters`;后者下点数无意义 |
| `required_points` / `selected_points` | 该形状需要的点数与当前已选点数 |
| `can_plan` | 现在重新规划会不会被接受 |
| `message` | `can_plan` 为假时的原因 |

**换形状不丢点。** A、B 两点在矩形和梯形里是同一个角,所以矩形选好 2 点后切到梯形
只是等第 3 点,切回来立刻恢复可规划;梯形 3 点切回矩形则用前 2 点并说明多余的那个
被忽略。丢点会让下拉框上的一次误点变成不可逆操作。

`can_plan` 是**提示不是保证**:请求仍可能被拒,原因在服务响应里。判定只写在规划器的
`planBlockedReason()` 一处,由 Replan 守卫、`configure` 响应和面板置灰共用,三者不可能
互相矛盾。

### 参数

| 参数 | 可选值/单位 | 来源或作用 |
| --- | --- | --- |
| `frame_id` | frame 名称 | 默认 `odom` |
| `input_mode` | `parameters` / `rviz` | 区域输入方式 |
| `region_type` | `rectangle` / `trapezoid` | 矩形或等腰梯形 |
| `lower_left` | `[x, y] m` | A：左下角 |
| `upper_right` | `[x, y] m` | B：右上角 |
| `lower_right` | `[x, y] m` | C：梯形右下角 |
| `sweep_direction` | `horizontal` / `vertical` | 弓字扫描方向 |
| `start_corner` | `lower_left` / `lower_right` / `upper_left` / `upper_right` | 起始角点 |
| `detection_width` | m | 检测有效宽度 |
| `detection_length` | m | 沿行进方向的检测有效长度；当前默认 `0.01`，待载荷标定 |
| `overlap_ratio` | `[0, 1)` | 相邻扫描带重叠率 |
| `robot_length`、`robot_width` | m | launch 从 `robot.yaml` 注入 |
| `edge_clearance` | m | launch 从 `robot.yaml` 注入 |
| `wall_width`、`wall_height` | m | launch 从 `wall.yaml` 注入 |
| `path_height` | m | RViz 路径离墙显示高度 |
| `bottom_warning_tolerance` | m | 梯形底边点高度修正提示阈值 |
| `minimum_nominal_coverage_ratio` | `(0, 1]` | 规划期覆盖率门限，默认 `0.965` |
| `top_edge_scan` | `auto` / `always` / `never` | 顶部收边扫描，默认 `auto` |

规划器内部计算：

```text
row_spacing = detection_width × (1 - overlap_ratio)
safety_margin = 0.5 × hypot(robot_length, robot_width) + edge_clearance
```

矩形点选顺序为 A（左下）、B（右上）；等腰梯形为 A（左下）、B（右上）、
C（右下）。A、C 的高度取平均值修正为水平底边。

完整任务接口区分检测覆盖区域 `coverage_region` 和机器人中心安全运动区域
`motion_region`。以下参数和输出在阶段 E/F 实现，不属于当前规划器已完成接口：

| 项目 | 含义 |
| --- | --- |
| `detection_length` | 沿行进方向的检测有效长度；与 `detection_width` 共同形成二维检测足迹 |
| `turn_clearance` | 竖向底部转向相对 `motion_region` 下边界的安全余量，初值 `0.06 m` |
| `/control/reference_path` | 转向后根据 EKF 实际位置生成的动态直线执行参考，仅供显示和评价 |

竖向为主时，控制器以第一次转向后的实际位置为斜直线 `TRANSITION` 起点，第二次
转向后按统一入轨判据冻结与名义线平行的 `SCAN`，不得逐列倒车返回名义起点。覆盖率
必须按二维检测足迹重新计算；低于 95% 时增加一条顶部水平收边扫描。

### 换道段的转向下坠预留

每个 `TRANSITION` 的**终点**都会沿反重力方向抬高一个预留量,使机器人在终点转向、
下坠之后正好落在下一条线的名义起点上。以前只有横向扫描做这件事,竖向按"下坠沿
扫描线方向、属于沿轨误差"豁免——**这个理由是错的**:线**起点**处的沿轨偏移不会被
跟踪器消掉,它把线截短了。实测 `3.30 × 4.50 m` 竖向矩形,每条**向下**的扫描列都
在距区域顶边 `46 mm` 处停止(正好一次 `90°` 转向的下坠),而向上的列全部越过顶边
——那里同样的下坠把起点往后推,只是多扫一点底部。

预留量按机器人**实际**会转过的角度算,不是名义几何:

| 项 | 取值 |
| --- | --- |
| 转向终态航向 | 下一条线航向 **+ 该线的重力前馈** |
| 转向初态航向 | **实走**换道段航向 **+ 该段的重力前馈** |
| 实走换道段 | 实际起点(已掉落)→ **抬高后**的终点 |
| 预留量 | `turn_slip_per_degree_m × 转角`,夹在 `[0, k × 180°]` |

抬高终点会让实走换道段更斜,从而改变终点转角,所以这是个不动点,用迭代解。收缩
因子约 `k × (180/π) / 换道段长`,`0.39 m` 时为 `0.074`,四次迭代到微米级。换道段
短于 `0.05 m` 时航向主要是位置噪声、迭代不再收缩,此时回退名义航向。

三处"实际"缺一不可。以矩形竖向的 `0.40 m` 换道段为例:名义转角 `90°`(预留
`45 mm`)→ 计入起点掉落和终点抬升后 `103.6°`(`51.8 mm`)→ 再计入换道段自身
`5.9°` 前馈后 `109.5°`(`54.7 mm`)。

**不再用实测下坠兜底。** 原先取 `max(模型值, 上一次转向的实测下坠)`,而"上一次
转向"和"要预留的这次转向"是两次不同的转向。梯形右斜边上机器人刚转过 `166°`
(掉约 `83 mm`),接着只需转 `14°`(该留 `7 mm`),这个下限会把预留抬到 `83 mm`,
多抬 `76 mm`。现在完全信任已标定的系数;`line_tracker` 改为每次对准都拿实测下坠
与模型比对,偏差超过 `50%` 时告警提示重跑 `measure_turn_slip.py`——标定过期会被
**报出来**,而不是被运行时悄悄补偿掉。

### 顶部收边扫描

由规划器在**发送 Goal 之前**追加，作为一条普通的 `SEGMENT_SCAN`；执行器不在运行
中热追加线段（§10.7、§11 执行规则第 7 条）。`top_edge_scan` 决定何时追加：

| 取值 | 行为 |
| --- | --- |
| `auto`（默认） | 仅当**预计**覆盖率低于 `minimum_nominal_coverage_ratio` 时追加 |
| `always` | 只要能放进 `motion_region` 就追加 |
| `never` | 从不追加 |

收边线位于 `区域顶边 - 0.5 × detection_width`，即其扫过的条带上沿正好落在区域
顶边；区域高度不足一个 `detection_width` 时取区域中线。进入方向取距离上一条扫描
线终点较近的一端，避免为了上线横穿整个区域。两个端点必须都在 `motion_region`
内，否则不追加并在 `/coverage/status` 说明原因——宁可少扫，也不生成一个执行器会在
接受 Goal 阶段拒绝的任务。

**只对竖向扫描生效。** 横向扫描的最高一条扫描线本来就在
`区域顶边 - 0.5 × detection_width`，收边线会与它完全重合。`always` 配横向扫描时
不追加，并在状态里说明。

**`auto` 用的是预计覆盖率，看不到执行损失。** 实测的顶部漏扫来自扫描柱末端的足迹
姿态，名义几何里并不存在（当前两个竖向工况的预计覆盖率都是 `100%`）。因此 §10.7
所说的"实测覆盖率低于 95%"这一半是操作流程：验收发现不足时，把
`top_edge_scan` 设为 `always` 重新规划，而不是期待 `auto` 自己触发。

## 阶段 E 冻结接口

### 公共接口包

新增 `climbot_interfaces`，只安装消息和 Action，不包含运行节点。规划器和控制器
都依赖该包，控制器不得依赖 `climbot_coverage` 的实现库。

### `CoverageTask.msg`

第一版字段冻结为：

```text
uint8 SWEEP_HORIZONTAL=1
uint8 SWEEP_VERTICAL=2

uint8 SEGMENT_SCAN=1
uint8 SEGMENT_TRANSITION=2
uint8 SEGMENT_RETURN=3

std_msgs/Header header
string task_id
uint32 revision
uint8 sweep_direction

geometry_msgs/Pose[] waypoints
uint8[] segment_types

geometry_msgs/Polygon coverage_region
geometry_msgs/Polygon motion_region

float64 detection_width
float64 detection_length
```

`header.frame_id` 是所有路点和两个 Polygon 的共同坐标系，默认 `odom`；
`header.stamp` 是该规划版本的生成时间。`N` 个 `waypoints` 必须对应 `N-1` 个
`segment_types`，其中 `segment_types[i]` 描述 `waypoints[i] → waypoints[i+1]`。
路点姿态指向下一段，最后一个姿态沿用到达航向。

接收方必须拒绝以下任务：

- `task_id` 为空、`revision` 为零或扫描方向非法；
- 路点少于两个，或线段类型数量不是路点数量减一；
- 坐标、四元数、检测尺寸包含非有限值，或检测尺寸不为正；
- 存在零长度线段、未知线段类型或非法四元数；
- Polygon 非法，或任一名义路点位于 `motion_region` 外；
- `SCAN` 无法对 `coverage_region` 达到规划器声明的覆盖要求。

`coverage_region`、`motion_region` 和 `waypoints` 不带各自 Header，统一继承任务
Header，避免同一任务内部出现多个坐标系或时间戳。

### 任务话题

| 话题 | 类型 | QoS | 权责 |
| --- | --- | --- | --- |
| `/coverage/task` | `climbot_interfaces/msg/CoverageTask` | reliable、transient local、depth 1 | 已实现；最新完整任务预览，不直接热更新正在执行的任务 |
| `/coverage/path` | `nav_msgs/msg/Path` | reliable、transient local、depth 1 | 从任务路点派生，仅用于 RViz 和通用工具 |
| `/control/reference_path` | `nav_msgs/msg/Path` | reliable、transient local、depth 1 | E3 已实现单段直线参考预览；后续状态机在转向后以 EKF 实测起点更新动态参考，仅用于显示和评价 |

规划失败、清空或开始新区块时，规划器同时发布空 `/coverage/path` 和空路点的
`/coverage/task`，并增加 revision，使旧预览失效。空任务永远不能作为执行 Goal。
当前规划器以原始用户区域填充 `coverage_region`，以整面墙按机器人安全边距内缩的
多边形填充 `motion_region`。E2 已按 `detection_width × detection_length` 的矩形足迹
布置标称 `SCAN`：中心线覆盖目标区域，端点沿行进方向按半个检测长度外延；每个中心
路点必须位于 `motion_region`，否则拒绝规划。标称覆盖率低于
`minimum_nominal_coverage_ratio`（默认 `0.965`）也会拒绝规划；该值高于 §14.3 的
`95%` 验收门限，留出实测 `0.37～1.09` 个百分点的执行损失。
竖向执行中的转向下滑属于实际轨迹问题，E9 将用同一足迹评估实际轨迹，并在需要时追加
一条顶部水平收边扫描。

### `ExecuteCoverage.action`

Action 名称为 `/coverage/execute`，E7 已实现。使用 ROS 2 Action 不表示接入 Nav2。

`evaluate_coverage_execution.py` 仍可订阅规划任务并充当测试 Action Client；正式操作
流程由 `coverage_manager_node` 在收到显式 `/coverage/start` 后发送 Goal。执行器已先
执行采集关闭的 `GO_TO_START/APPROACH_START`：校验当前点和首个路点均在凸
`motion_region` 内（因此其直线连接也在区域内），原地对准、直线到达、停车、再对准
首条扫描线。该进入段不计入覆盖采集或 `completed_segments`。

Feedback 已追加 `APPROACH_START=6`；进入阶段 `current_segment=-1`、
`segment_type=0`，用于与正式任务线段区分，不能误标为 `TRACK_LINE`。

Goal：

```text
climbot_interfaces/CoverageTask task
```

Result：

```text
uint16 SUCCESS=0
uint16 CANCELED=1
uint16 INVALID_TASK=2
uint16 LOCALIZATION_TIMEOUT=3
uint16 CONTROL_TIMEOUT=4
uint16 OUT_OF_BOUNDS=5
uint16 TRACKING_FAILED=6

uint16 result_code
string message
uint32 completed_segments
float64 elapsed_time_s
```

Feedback：

```text
uint8 WAITING=0
uint8 ALIGN=1
uint8 TURN_SETTLE=2
uint8 TRACK_LINE=3
uint8 FINAL_APPROACH=4
uint8 STOPPED=5
uint8 APPROACH_START=6

uint8 state
int32 current_segment
uint8 segment_type
float64 along_track_error
float64 cross_track_error
float64 heading_error
float64 remaining_distance
float32 progress
```

`current_segment = -1` 表示尚未进入任何线段。误差单位分别为米、米、弧度和米；
`progress` 范围为 `[0, 1]`，只用于显示，不作为任务完成判据。
当前实现中 `along_track_error` 表示机器人相对本段起点的有符号沿轨坐标；
`remaining_distance` 才是到本段终点的沿轨剩余量。

执行器完整复制并校验 Goal，一次只执行一个任务。每段都经过
`制动 → 原地 ALIGN → 航向稳定 → 直线跟踪 → 空间到达并停车`，不会用运行时间替代
到达判据；取消、定位超时、单段超时或越界均先发布零速。执行时 `CoverageTask`
保持不变，但转向稳定后更新动态执行参考：横向换道采用实测起点，并在换道终点上方
预留第二次转向的预计下滑量；竖向换道采用实测起点到名义终点的斜直线，不倒车返回
名义点。首条扫描和第二次转向后的扫描使用同一入轨规则：法向偏差不超过 `0.045 m`
时直接冻结实测位置对应的平行扫描线；偏差在 `0.045～0.12 m` 时先执行一次带重力
前馈的前进小弧线，收敛到 `0.03 m` 后重新对齐并冻结平行线；偏差更大、超时、剩余距离不足或动态端点
越界时停车并终止任务。正式扫描线一旦冻结便保持为直线，不在跟踪中继续移动。
横向换道的预计下滑量取 `turn_slip_per_degree_m × 下一转角` 与第一次转向实测下坠
量的较大值；短上底梯形会因此自动适应斜向换道两侧不同的接触扰动。

“不出现明显蛇形”的判断不使用横轨误差过零次数作为单独故障条件：默认只对横向
幅度超过 `0.03 m` 且沿轨持续出现的往复做诊断告警。最终接受与否还要同时检查真值
横轨包络、航向摆幅和整段曲率；厘米级以内的小误差反复过零允许存在。

### 执行与版本规则

1. `task_id` 标识一次用户任务；对同一任务重新规划时 revision 严格递增。revision
   是单调发布版本号，规划失败、清空点选和空任务发布也会占用版本号，因此允许跳号，
   接收方只能比较新旧，不得假设连续编号。`task_id` 来自规划器参数而非区域内容，
   若希望结果能按 `task_id` 区分，必须为每个区域配置各自的值；六个演示配置已分别
   设定。同一语义也写在 `CoverageTask.msg` 中。
2. Action Server 接受 Goal 时完整复制并再次校验任务，此后该执行版本不可变。
3. `/coverage/task` 后续出现新 revision 只更新预览，不得热切换当前 Action。
4. 同一时刻只允许一个执行 Goal；切换任务必须先取消当前 Goal并停车。
5. Action 取消、异常终止或定位超时都必须先输出零速，再返回最终状态。
6. 转向后的斜线 `TRANSITION` 只进入控制器内部执行参考和
   `/control/reference_path`，不得反写冻结的 `CoverageTask`。
7. 顶部收边扫描若按预计覆盖率需要，必须在发送 Goal 前作为 `SCAN` 写入任务；
   第一版不在执行中热追加线段。实测覆盖率不足视为验收失败并调整下一版任务。

## 侧滑标定参数

`calibrate_wall_slip.py` 的主要参数：

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `repetitions` | `3` | 水平、上行和下行重复次数 |
| `static_duration_s` | `30.0` | 静止稳定性记录时长 |
| `drive_duration_s` | `8.0` | 每个运动段的仿真时长 |
| `linear_speed_mps` | `0.15` | 标定线速度指令 |
| `heading_hold_gain` | `1.5` | 融合航向保持比例增益 |
| `horizontal_repeatability_max_cv` | `0.05` | 水平下降比总体变异系数上限 |
| `trajectory_csv` | 空 | 非空时保存真值相对轨迹和真值/融合航向 |

水平段只做航向保持，不做横轨位置纠偏。正式运行必须使用全新启动的仿真世界。

## 原地转向下滑标定参数

`measure_turn_slip.py` 的主要参数：

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `angles_deg` | `[30, 45, 90, 135, 180]` | 测试转角，交替方向 |
| `max_rates_rps` | `[0.3, 0.6]` | 角速度上限档位 |
| `repetitions` | `2` | 每组重复次数，第二次反向 |
| `maximum_reference_offset_m` | `0.05` | 参考点偏移自检上限 |
| `output_csv` | `results/turn_slip.csv` | 逐次转向的原始位移 |

结束时由 `climbot_gazebo.turn_slip_model` **联合拟合**两个量:上报位姿相对旋转
中心的偏移,以及下滑系数 `turn_slip_per_degree_m`,后者可直接填进 `control.yaml`。

**两者必须联合拟合,不能先解偏移再算系数**:先解偏移会把一部分真实下滑吸收进
偏移里。实测代价是系数从 `0.00050` 被压到 `0.00022`,少一半还多。

偏移是自检项:上报位姿不在旋转中心时,原地转向会让它绕中心划弧,这段**运动学
摆动**混进 `vertical_mm`,使 CSV 里逐角度、逐方向的数值不再是真实滑移(2026-08-13
那份数据偏移 `79 mm`,逆时针大角度甚至呈现"净上升")。超过
`maximum_reference_offset_m` 即报错。**聚合系数不受影响**——标定扫描两个方向对称,
摆动在总体斜率里抵消,所以那份坏数据拟合出的系数仍是 `0.00049`。

## 侧滑补偿专项验收参数（§14.4）

`evaluate_slip_compensation.py` 在同一仿真、同一段墙面上跑两个阶段，比较横轨闭环
关闭与开启的水平直线。`mode` 决定跑哪个阶段：

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `mode` | `open_loop` | `open_loop` 为补偿关闭，`compensated` 为补偿开启 |
| `repetitions` | `3` | 每个阶段的重复次数，§14.4 要求关闭态至少三次 |
| `line_length_m` | `1.20` | 被测水平直线的名义长度 |
| `linear_speed_mps` | `0.15` | 与侧滑标定一致的线速度 |
| `heading_hold_gain` | `1.5` | 仅 `open_loop` 使用的航向保持增益 |
| `entry_lead_m` | `0.40` | 仅 `compensated` 使用的引入换道段长度 |
| `minimum_measured_fraction` | `0.95` | 实测直线长度相对名义长度的下限 |
| `minimum_height_error_reduction` | `0.70` | §14.4 的净高度误差降幅门限 |
| `maximum_open_loop_cv` | `0.05` | §14.4 的关闭态下降比变异系数上限 |
| `visible_excursion_m` | `0.020` | 计入可见往复的横轨幅值 |
| `maximum_visible_reversals` | `0` | 补偿态允许的可见蛇形往复次数 |
| `reference_summary_json` | 空 | `compensated` 模式下读取的关闭态摘要，用于计算降幅 |
| `trajectory_csv` / `summary_json` | 空 | 非空时落盘逐采样轨迹与判定摘要 |

两个阶段的差异只有补偿本身：**`open_loop` 阶段必须在没有 `line_tracker` 的情况下
运行**。空闲的跟踪器同样以 `50 Hz` 在 `/control/cmd_vel` 上发布零速，会与开环指令
互相覆盖。速度看门狗由 `climbot_wall.launch.py` 启动，因此 `/cmd_vel` 的安全门与
正常任务完全一致。

`compensated` 阶段是**一个**任务：一段 `TRANSITION` 引入段，加 `repetitions + 1`
条同向 `SCAN`。掉头对准会带来约 `85 mm` 的下坠，而 §10.7 的换道段是把参考线平移
到实际位置、不爬回名义线，因此第一条扫描线的起始偏差必然超过
`parallel_scan_offset_m`（`45 mm`），跟踪器会用一次前进小弧线入轨而不是直接平移
扫描线，吃掉约 `0.33 m`。**第一条扫描线因此只执行、不计入**：它承担入轨，其后的
扫描线之间没有转向，是与开环段可比的稳态直线。加长引入段无效——实测 `0.5 m` 和
`1.2 m` 的引入段都仍把 `81～87 mm` 的偏差交给扫描段。

`minimum_measured_fraction` 是这条逻辑的护栏：任一被测扫描线的实测长度低于名义
长度的该比例即失败，不会把半条线的结果混进平均值。它就是发现上述入轨消耗的原因。

## TF

| 变换 | 发布者 | 说明 |
| --- | --- | --- |
| `world → wall` | static transform publisher | 来自 `wall.yaml` |
| `wall → odom` | static transform publisher | 当前为单位变换 |
| `odom → base_link` | `robot_localization` | 连续融合位姿 |
| `base_link → links` | `robot_state_publisher` | 来自共享 URDF 和 `/joint_states` |

## 配置文件

| 文件 | 内容 |
| --- | --- |
| `climbot_description/config/robot.yaml` | 机器人共享物理属性、限幅和 footprint |
| `climbot_description/config/wall.yaml` | 工作坐标系、墙面宽高 |
| `climbot_gazebo/config/simulation.yaml` | Gazebo 专有物理和传感器参数 |
| `climbot_gazebo/config/ekf_wall.yaml` | EKF 状态选择、频率和协方差输入 |
| `climbot_coverage/config/coverage_rectangle.yaml` | 默认矩形任务 |
| `climbot_coverage/config/coverage_trapezoid.yaml` | 默认等腰梯形任务 |
