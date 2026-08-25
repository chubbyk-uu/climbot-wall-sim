# ROS 2 接口与配置索引

本文档记录当前已实现接口。阶段 E 的任务与控制接口原先在这里以"已冻结但尚未实现"
列出，现已全部实现并有归档证据，见 `docs/STATUS.md` 的阶段状态表。

## 启动入口

| 命令 | 用途 |
| --- | --- |
| `ros2 launch climbot_gazebo climbot_wall.launch.py` | Gazebo、桥接、TF、传感器适配和 EKF |
| `ros2 launch climbot_coverage coverage_planner.launch.py` | 独立覆盖规划器和可选 RViz |
| `ros2 launch climbot_bringup coverage_sim.launch.py` | 联合启动仿真、规划器和 RViz；**不含执行器**，只能预览 |
| `ros2 launch climbot_bringup coverage_mission.launch.py` | 完整任务入口：仿真、规划器、RViz、跟踪器和管理器 |
| `ros2 launch climbot_control line_tracker.launch.py` | 单段直线跟踪器；从共享描述注入轮距和轮缘硬限值 |
| `ros2 launch climbot_control coverage_executor.launch.py` | 多段覆盖 Action 执行器；不接入 Nav2 |
| `tools/run_coverage_regression.sh -j 4 -t <tag> [-m time]` | 八个验收工况并行回归，每条 lane 独占 `ROS_DOMAIN_ID` 与 `GZ_PARTITION` |

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
| `planner_config_file` | `coverage_interactive.yaml` | 规划器参数；点选模式下忽略其中的角点。该文件的 `task_id` 是 `rviz-selection`，不含形状——形状由面板在运行时决定，写进标识就会出现「梯形下面写着 Executing rectangle」 |
| `control_config_file` | `control.yaml` | 跟踪器参数 |
| `region_type` | `rectangle` | `rectangle` 需两点，`trapezoid` 需三点 |
| `sweep_direction` | `horizontal` | 扫描方向 |
| `tracking_mode` | `time` | 直线段控制律：`time` 或 `distance` |
| `wall_grid_spacing` | `1.0` | **只管 Gazebo 墙面上**画的参考网格线，间距（m），`0` 不画。默认值取自 `climbot_description/config/wall.yaml`。RViz 叠加层不受它控制——那一套在 Displays 里勾掉 |
| `wall_texture` | 空 | 透传给 `climbot_wall.launch.py`，见下 |
| `flat_field_file` | 空 | 可选平场 `.npz`；仅发布补偿预览并作为归档标定引用，绝不改写 `images/raw/` |

launch 参数只给这三项定初值。`region_type` 和 `sweep_direction` 运行中改用
`/coverage/configure` 或面板上的下拉框，见下面的"运行时构型"；规划器的其余参数
只在构造时读一次，`ros2 param set` 不生效。

`tracking_mode` 是例外：它**是**运行时可写的参数，用
`ros2 param set /line_tracker tracking_mode time` 或面板上的 Algorithm 下拉框，
执行器只在没有任务运行时接受。见"直线段控制律与时间参数化"。

两个参数文件名字不同且各自显式传递。被包含的 launch 会继承父作用域的同名参数，
且 `DeclareLaunchArgument` 的默认值对父作用域已设定的名字不生效，因此一个共用的
`config_file` 会同时落到规划器和跟踪器上，使跟踪器静默退回内置默认值。

点选模式下的操作顺序是：用 RViz 的 `Publish Point` 工具依次点击区域角点
（矩形为左下、右上；梯形再加右下），确认 `Coverage Task` 面板的 State 变为
`Ready`，再点面板上的 `Start`（等价于调用 `/coverage/start`）。`/coverage/status`
会回显每次点击被接受时的坐标，可据此发现相机视角造成的镜像或旋转误选。
角点点错时用面板的 `Clear points` 或 `/coverage/clear_points` 清空重来。
RViz 的固定坐标系是 `odom`，即墙面平面，所以点选工具给出的 `x`、`y` 就是墙面
坐标；规划器忽略 `z`。该坐标系的原点在墙面左下角，因此点出来的坐标全为非负，
`(0, 0)` 就是墙面左下角本身。

`climbot_wall.launch.py` 的主要 launch 参数：

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `use_sim_time` | `true` | 使用 Gazebo 时钟 |
| `headless` | `false` | 无 GUI 启动 Gazebo server |
| `gpu_backend` | `auto` | `auto`、`wsl_d3d12` 或 `native` |
| `total_station_rate_hz` | `12.0` | 模拟全站仪频率 |
| `total_station_stddev_m` | `0.001` | 全站仪位置一倍标准差 |
| `total_station_delay_s` | `0.01` | 固定传输延迟（10 ms）；位姿延迟误差保留给标签和后处理，不以巡检限速掩盖 |
| `wheel_forward_velocity_stddev_mps` | `0.03` | 轮式前向速度标准差 |
| `wheel_yaw_rate_stddev_rps` | `0.05` | 轮式角速度标准差 |
| `imu_orientation_stddev_rad` | `0.00174532925` | IMU 姿态标准差（0.1°） |
| `wall_grid_spacing` | `1.0` | 墙面参考网格线间距（m），`0` 不画。默认值取自 `climbot_description/config/wall.yaml`。只影响 Gazebo 渲染的墙面 |
| `wall_texture` | 空 | `tools/bake_wall_texture.py` 产出的清单路径；空则用 `simulation.yaml` 的 `wall.texture_manifest`。路径不存在时报错退出，不退回平色墙 |

## 仿真与定位话题

| 话题 | 类型 | 生产者 | 消费者/用途 |
| --- | --- | --- | --- |
| `/control/cmd_vel` | `geometry_msgs/msg/Twist` | E3 直线控制器、未来任务状态机 | E4 速度看门狗；控制层唯一入口 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | E4 速度看门狗 | Gazebo DiffDrive；每 `20 ms` 重发 |
| `/control/hold_active` | `std_msgs/msg/Bool` | E4 速度看门狗 | transient local；速度保持是否生效 |
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
转向、换道或小弧线入轨误算成覆盖。`minimum_actual_coverage_ratio` 默认 `0`，
`coverage_grid_resolution_m` 默认 `0.01 m`；正常任务仅记录覆盖比例，不因可走区未完整
拍到而失败。需要合同覆盖验收时才显式设置正数门限；低于该门限时进程返回失败。
`trajectory_csv` 和 `summary_json` 默认为空，显式配置后分别保存完整真值/融合轨迹和
机器可读的 Action、逐段误差、覆盖/漏扫面积摘要。两者都在 `try/finally` 中写出：
超时或异常时同样落盘，摘要里 `completed=false`、`passed=false` 并记录
`failure_reason`，因此最需要现场数据的失败运行不会丢数据。摘要还带 `provenance`
段，记录代码提交、分支、`src` 树是否有未提交改动、评价器全部参数和执行任务的
名义几何，满足 §12 与 §14.6 对可追溯性的要求。

`provenance` 有四个子段，共用 `climbot_gazebo.provenance`：

| 子段 | 内容 | 来源 |
| --- | --- | --- |
| `git` | `commit`、`branch`、`source_modified`、`traceable` | 工作树 |
| `noise_sources` | `total_station_sim` 的种子、`stddev`、频率、延迟、丢包率；`wall_imu_adapter` 的种子和 `stddev` | **向节点的参数服务问回来** |
| `control_parameters` | `line_tracker` 的 `tracking_mode`、`cruise_speed`、`turn_slip_per_degree_m` 和三个扫描偏移门限 | **向节点的参数服务问回来** |
| `evaluator_parameters` | 评价器自己的全部参数 | 自身 |

后两段是问回来的而不是复述配置文件，因为**传给一个没起来的节点的参数，看起来和用过
的一模一样**——和 `source_modified` 曾经被写下却没人读是同一类错误。问不到时写
`null`，例如定位对照实验不起跟踪器，它的 `control_parameters.line_tracker` 就是
`null`，而不是抄一份配置值冒充。

摘要**必须是合法 JSON**：写出时用 `allow_nan=False`。Python 的 `json.dump` 默认会写出
裸 `NaN` / `Infinity` 记号，Python 自己读得回来，Ruby、严格 Java/Go、schema 校验和多数
数据仓库读不了——只有生产者自己能读的证据不算机器可读证据。不适用的指标写 `null`，
并另带 `applicable: false` 和 `not_applicable_reason`：单独一个 `null` 分不清
"不适用"和"没测到"，而这是关于一次运行的两件不同的事。`test_results_are_machine_readable.py`
对 `results/` 下全部摘要做严格解析，并要求每个 `null` 都给出理由。

`turn_slip_per_degree_m` 在 `control_parameters` 里，是因为 `reservedTurnDrop()` 用它
抬高起点进入的终点，它直接决定第一条扫描线的初始横轨误差；三个偏移门限是那个误差
随后被判定的梯子。实测见 `results/README.md`「G-1 不同初始横轨误差」。

评价器的 `case` 决定它执行什么任务：

| `case` | 任务来源 | 用途 |
| --- | --- | --- |
| `planned_task` | 订阅规划器发布的 `/coverage/task` | 八工况覆盖回归 |
| `vertical_rectangle` | 评价器自建 | 紧凑三段竖向用例 |
| `short_top_trapezoid` | 评价器自建 | 紧凑五段梯形用例 |
| `straight_line` | 评价器自建 | 单段直线与起点进入（§15.7、阶段 E 第 8 项、G-1） |

`straight_line` 由四个参数确定，全部相对机器人**当前**位姿：

| 参数 | 默认值 | 行为 |
| --- | ---: | --- |
| `straight_line_bearing_deg` | `0.0` | 线本身的走向，墙面系。`0` 沿墙，`90` 朝上 |
| `straight_line_length_m` | `2.0` | 线长 |
| `straight_line_start_offset_m` | `0.6` | 线首点离机器人多远 |
| `straight_line_approach_bearing_deg` | `NaN` | 首点在机器人的哪个方位；`NaN` 表示沿线方向。写进 `provenance` 时记为 `null`——`NaN` 不是合法 JSON |

**方位与走向是两个角，不能合并。** 偏置沿线方向取时机器人一开始就朝着首点，起点进入
等于没被考验——这是一个角度唯一能表达的情形。

**偏置也不能为零。** 线首点正好压在机器人身上时，起点进入无处可去，入线弧只能从线
本身里扣：`180°` 转向后实测扣掉 `383 mm`，即 `4 m` 线的 `8.6%` 永远盖不到。真实任务里
规划器把进入放在第一条扫描线之前，这个偏置就是把那个结构还原。见
`results/README.md`「§15.7 单段直线」的 `line1`。

覆盖区取扫掠带，两端各内缩半个足迹——足迹中心最远只到端点，那半个足迹是任何正确执行
都盖不到的。

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
| `line_tracker` | `turn_slip_per_degree_m` | `0.00041 m/°` | 横向换道终点为第二次转向预留的标定下滑系数；**壁面相关**，随摩擦与 WheelSlip 变化，换墙必须用 `measure_turn_slip.py` 重标定 |
| `line_tracker` | `parallel_scan_offset_m` | `0.045 m` | 首条或转后偏差不超过此值时直接冻结实测位置对应的平行扫描线 |
| `line_tracker` | `maximum_scan_offset_m` | `0.12 m` | 转后可尝试单次前进小弧线入轨的最大法向偏差 |
| `line_tracker` | `arc_entry_finish_offset_m` | `0.012 m` | 小弧线结束、重新对齐并冻结平行扫描线的法向偏差门限；直接决定该扫描线冻结在何处，因此必须落在 §14.3 的 `20 mm` 间距预算内 |
| `line_tracker` | `arc_entry_speed_mps` | `0.08 m/s` | 带重力侧滑前馈的小弧线恒定前进速度；采集保持关闭 |
| `line_tracker` | `arc_entry_lookahead_m` | `0.20 m` | 小弧线航向引导前视距离 |
| `line_tracker` | `arc_entry_max_heading_deg` | `20°` | 小弧线相对名义扫描方向的最大航向修正 |
| `line_tracker` | `arc_entry_max_angular_speed` | `0.25 rad/s` | 小弧线最大角速度 |
| `line_tracker` | `arc_entry_timeout_s` | `15 s` | 小弧线未收敛时的停车失败门限 |
| `line_tracker` | `cruise_speed` | `0.20 m/s` | 扫描和换道期望巡航速度 |
| `line_tracker` | `max_linear_speed` | `0.25 m/s` | 位置控制模式的线速度上限，高于巡航值以容纳上爬打滑 |
| `line_tracker` | `visible_oscillation_amplitude_m` | `0.03 m` | 仅记录肉眼可见幅度的横轨往复；小误差反复过零不算故障 |
| `line_tracker` | `control_frequency_hz` | `50 Hz` | 直线跟踪控制频率 |
| `line_tracker` | `cross_gain` | `1.0 rad/m` | 横轨比例反馈增益 |
| `line_tracker` | `cross_integral_gain` | `0.30 rad/(m·s)` | 横轨积分反馈增益 |
| `line_tracker` | `cross_integral_limit_m_s` | `0.10 m·s` | 横轨积分状态绝对限值 |
| `line_tracker` | `cross_slowdown_start_m` | `0.03 m` | 超过此横轨误差后开始连续降速 |
| `line_tracker` | `cross_slowdown_full_m` | `0.08 m` | 到此误差后保持最小缩放速度，不完全停车 |
| `line_tracker` | `cross_slowdown_min_scale` | `0.25` | 大横轨误差下的最小线速度比例 |
| `line_tracker` | `heading_gain` | `2.0` | 直线跟踪的航向内环增益 |
| `line_tracker` | `arc_entry_heading_gain` | `2.0` | 小弧线入轨的航向增益 |
| `line_tracker` | `max_angular_speed` | `0.35 rad/s` | 直线跟踪角速度上限（原地转向另有 `max_turn_angular_speed`） |
| `line_tracker` | `max_linear_acceleration` | `0.20 m/s²` | 速率限幅器的加速权限，位置控制模式 |
| `line_tracker` | `max_angular_acceleration` | `0.80 rad/s²` | 直线跟踪的角加速度权限 |
| `line_tracker` | `gravity_slip_ratio` | `0.1042` | 单位前进距离的下滑比，决定重力前馈角 `atan(ratio·投影)`；**壁面相关**，换墙须用 `calibrate_wall_slip.py` 重标定 |
| `line_tracker` | `gravity_down_x` / `gravity_down_y` | `0.0` / `-1.0` | 作业平面内的重力方向单位向量 |
| `line_tracker` | `visible_oscillation_minimum_travel_m` | `0.10 m` | 判定横轨振荡前必须走过的最短距离 |
| `line_tracker` | `visible_oscillation_reversals` | `4` | 判定为可见振荡所需的往复次数 |
| `line_tracker` | `max_gravity_feedforward_deg` | `8°` | 重力前馈独立限幅 |
| `line_tracker` | `max_cross_feedback_deg` | `8°` | 横轨 PI 反馈独立限幅 |
| `line_tracker` | `max_heading_correction_deg` | `12°` | 前馈与反馈相加后的总航向硬限幅 |
| `line_tracker` | `alignment_tolerance_deg` | `1°` | 进入直线跟踪前的航向稳定误差；这是死区不是收敛目标，必须明显小于 §14.3 接受的 `2.0°` 转向落点误差 |
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
| `line_tracker` | `goal_heading_exit_tolerance_deg` | `3°` | 航向候选状态退出门限；严格门限是 `alignment_tolerance_deg`，`control.yaml` 里为 `1°` |
| `line_tracker` | `stopped_linear_speed_mps` | `0.01 m/s` | 完成时融合线速度上限 |
| `line_tracker` | `stopped_angular_speed_rps` | `0.02 rad/s` | 完成时融合角速度上限 |
| `line_tracker` | `goal_settle_duration_s` | `0.30 s` | 位置、航向和停车状态稳定时间 |
| `line_tracker` | `max_linear_deceleration` | `0.25 m/s²` | 速率限幅器的减速权限 |
| `line_tracker` | `braking_profile_deceleration` | `0.12 m/s²` | 距离-停车剖面使用的减速度，必须小于 `max_linear_deceleration`：两者相等时指令永远滞后剖面一步，每个段终点都会冲过去 |
| `cmd_vel_watchdog` | `command_timeout_s` | `0.40 s` | 上游速度指令超时后持续发布零速 |
| `cmd_vel_watchdog` | `publish_rate_hz` | `50 Hz` | 向执行器重发安全速度的频率 |

速度看门狗另外提供一条**不经过执行器**的停止通路：

| 服务 | 类型 | 行为 |
| --- | --- | --- |
| `/control/hold` | `std_srvs/srv/SetBool` | `true` 使 `/cmd_vel` 恒为零速，与 `/control/cmd_vel` 上是否还有指令无关；`false` 解除 |

系统里其他所有停止都是**请求正在驱动的那一方停下来**：取消 Goal，控制器自己减速
停车。执行器还在应答时这是对的做法。这条不是——它管的是执行器还活着、`/control/cmd_vel`
仍在刷新、却没有任何请求能送达的情况。此时指令并不陈旧，只是没人要，看门狗的超时
永远不会触发。保持位于轮子前的最后一跳，因此与图上其余部分处于什么状态无关。

保持状态在 `/control/hold_active` 上以 transient local 发布：施加保持的一方可能在
保持生效期间崩溃或被重启，届时这个状态就只存在于看门狗内部，而一台"无缘无故不动"
的机器人是最难诊断的故障。反过来，看门狗重启会丢失这个进程内状态，因此管理器在
`STOPPING` 期间持续核对并重试保持；`hold_active=true` 只证明当前输出被钳为零，不能
证明上游任务已经终止。它也不是实机安全完整性保证：实机必须另有默认失效关闭、独立
于 ROS 进程的硬件急停或驱动使能回路。

#### 直线段控制律与时间参数化

`tracking_mode` 决定直线段线速度**怎么算出来**，其余控制链路两种模式完全一致：
角速度双环、横轨 PI、重力前馈、以及"航向超阈值则线速度归零"都不随模式变化。
原地转向段本来就是时间参数化的（`planTurn`），也不随模式变化。

`tracking_mode` 是唯一可以在**运行时**改的控制参数，因为它是操作面板上的一个下拉
框而不是 launch 里的一行。执行器**只在没有任务运行时接受**这个写入，运行中拒绝并
在 `SetParametersResult.reason` 里说明——控制律换了而时间表还是按旧律排的，那是
危险的。

| 节点 | 参数 | 默认值 | 行为 |
| --- | --- | ---: | --- |
| `line_tracker` | `tracking_mode` | `time` | `time` 按时间参数化速度曲线行驶；`distance` 按剩余距离制动。非法值拒绝。**不在 `control.yaml` 里**：它是 launch 参数兼运行时可写参数，两处都写会静默互相覆盖 |
| `line_tracker` | `time_profile_acceleration` | `0.20 m/s²` | 时间曲线的加速段斜率，与额定线加速度相同 |
| `line_tracker` | `time_profile_deceleration` | `0.20 m/s²` | 时间曲线的减速段斜率，对称 |
| `line_tracker` | `time_speed_lag_s` | `0.08 s` | 执行机构速度环滞后，作为加速度前馈的系数：`v_cmd += τ·v̇`。实测 `0.0799 s`（sd `0.0193`，124 段） |
| `line_tracker` | `time_along_gain` | `2.5` | 沿轨滞后的比例增益。`0` 时竖向段开环超时失败，`6` 相对 `2.5` 无收益而指令抖动翻倍 |
| `line_tracker` | `time_along_integral_gain` | `0.0` | 沿轨积分增益。巡航段无稳态速度误差，默认不启用 |
| `line_tracker` | `time_along_integral_limit_m_s` | `0.05 m/s` | 积分项对线速度的贡献上限 |
| `line_tracker` | `catch_up_max_linear_speed` | `0.35 m/s` | 时间模式的线速度上限，即修正量可用的余量 |
| `line_tracker` | `catch_up_max_linear_acceleration` | `0.35 m/s²` | 时间模式速率限幅器的加减速权限 |
| `line_tracker` | `time_mode_final_approach_enabled` | `true` | 终点前 `final_approach_distance_m` 交回距离制动曲线 |
| `line_tracker` | `time_axis_stretch_enabled` | `false` | 沿轨滞后超阈值时冻结参考时钟推进 |
| `line_tracker` | `time_axis_stretch_lag_m` | `0.05 m` | 上一项的触发阈值 |

曲线始终按**额定工作点**规划（`cruise_speed` 与两个 `time_profile_*`），
`catch_up_*` 只是修正量的活动余量。这条区分是必需的：曲线要求的必须小于限幅器能
给的，否则指令一路贴着限幅器走，修正一点也送不进去——`braking_profile_deceleration`
低于 `max_linear_deceleration` 是同一个道理。

轮级饱和会按相同比例缩放左右轮，所以线速度顶到轮速上限时会**连带压掉角速度修正**。
必须满足 `v_max ≤ 轮速上限 − ω_max·轮距/2`；轮速上限来自
`climbot_description/config/robot.yaml`，见"配置文件"。这条**在启动时校验**，不满足
直接拒绝启动并报出各项数值：缩放不会报任何错，时间模式下的表现只是一个永远追不回来
的滞后，靠看日志发现不了。`v_max` 取两种模式里更快的那个——`distance` 模式受
`max_linear_speed` 约束，`time` 模式钳到 `catch_up_max_linear_speed` 而**不经过**
`max_linear_speed`，且模式可在运行时切换，所以两者都要满足。

`time_mode_final_approach_enabled` 默认打开，因为时间参考按计划时刻停车，会把残余
滞后原样变成落点误差（实测 `10.9~15.4 mm`），而距离曲线是位置的函数，位置不到就
不停（实测恢复到 `2.50~3.42 mm`）。

时长模型的每段固定开销，实测标定：

| 节点 | 参数 | 默认值 | 行为 |
| --- | --- | ---: | --- |
| `line_tracker` | `schedule_align_converge_s` | `1.24 s` | `ALIGN_SETTLE` 收敛到航向死区的每段耗时 |
| `line_tracker` | `schedule_handshake_s` | `0.43 s` | 转向开始前等待上一条指令衰减的每段耗时 |
| `line_tracker` | `schedule_goal_stop_s` | `0.13 s` | 终点限速与减速到静止的每段额外耗时 |

这三项由八条基线轨迹按控制状态拆分实测得到，只进入进度权重和 `planned_total_s`，
**不参与任何控制指令**。全为 `0` 时预测比实际短 `10.6%`；代入后 `act/plan` 落在
`0.981~1.015`（`timeE` 八工况实测，见 `results/README.md`）。

两个节点都拒绝非有限输入。直线跟踪器在启动时校验线段、增益、速度、加速度、
轮距、重力方向和坐标系参数；非法配置直接启动失败，不进入周期回调。轮距、轮缘
速度硬限值和轮缘加速度硬限值不存放在 `control.yaml`，由标准 launch 从
`climbot_description/config/robot.yaml` 注入。

横轨积分只在直线前进时更新；定位超时会清零。反馈或总航向修正达到限幅时采用
条件积分抗饱和，但重力前馈达到自身独立限幅不会阻止 PI 修正剩余误差。

`ALIGN` 先制动到线速度为零，再跟踪自动选择的三角形/梯形角速度曲线；曲线结束后
在 `1°` 航向容差内稳定 `0.50 s` 才允许直行。`10°/12°` 的退出/重入门限提供迟滞。
终点低速段只在空间剩余距离触发；完成判据同时使用二维位置、补偿后航向和
`/odometry/filtered` 的融合速度，不使用预定运行时间。完成后状态和零速输出锁存。

## 覆盖规划接口

### 话题

| 话题 | 类型 | QoS | 含义 |
| --- | --- | --- | --- |
| `/clicked_point` | `geometry_msgs/msg/PointStamped` | depth 10 | RViz `Publish Point` 输入 |
| `/coverage/path` | `nav_msgs/msg/Path` | transient local, depth 1 | 墙面覆盖路径；失败或清空时发布空路径 |
| `/coverage/markers` | `visualization_msgs/msg/MarkerArray` | transient local, depth 1 | 墙面、绿色虚线绝对安全区、橙色任务可走区、蓝色机器人路径、黄色相机覆盖带和方向 |
| `/coverage/wall_grid` | `visualization_msgs/msg/MarkerArray` | transient local, depth 1 | 参考网格线，启动时发一次。单独一个话题是为了在 RViz 里能单独勾掉；线位于工作系原点整数倍处、只画内部线，与 Gazebo 墙面上那套规则一致。间距只从 `wall.yaml` 取，不受 `wall_grid_spacing` 影响 |
| `/coverage/status` | `std_msgs/msg/String` | transient local, depth 1 | 等待、成功和失败原因 |

所有覆盖接口默认使用 `odom`。RViz 点选消息的 `frame_id` 不匹配时会被拒绝。

`effective` 命名空间里的**绿色虚线绝对安全区**是墙面按 `safety_margin` 内缩的结果，
与点选角点无关，因此从节点启动起一直发布。`original` 的橙色实线是用户点选的机器人
任务可走区，`coverage_path` 的蓝色实线是 `base_link` 名义路径；所有蓝色路点必须位于
橙区内。`camera_coverage` 只有规划成功后才发布，用黄色半透明 `TRIANGLE_LIST` 逐条画出
相机矩形扫掠带；它是可重叠的不规则并集，不冒充一个矩形或梯形 Polygon。

### 服务

| 服务 | 类型 | 行为 |
| --- | --- | --- |
| `/coverage/clear_points` | `std_srvs/srv/Trigger` | 清除点选，并发布空 Path/Marker |
| `/coverage/replan` | `std_srvs/srv/Trigger` | 使用当前区域重新规划 |

任务管理器已实现以下任务操作接口：

| 服务 | 类型 | 行为 |
| --- | --- | --- |
| `/coverage/start` | `std_srvs/srv/Trigger` | 锁定管理器当前显示的有效 `task_id + revision`，校验后发送 `/coverage/execute` Goal |
| `/coverage/cancel` | `std_srvs/srv/Trigger` | 请求取消当前 Goal；若仍在等待 hold 释放则丢弃未发送 Goal、重施 hold 并进入恢复锁；最终状态看 `/coverage/manager_status` |
| `/coverage/force_abandon` | `std_srvs/srv/Trigger` | 仅在 Start 应答未知的 `STOPPING` 中放弃等待；进入 `RECOVERY_LOCKED`，不代表任务已停止 |
| `/coverage/rearm` | `std_srvs/srv/Trigger` | 操作员确认硬件停车或执行器终止后解除恢复锁；hold 留到下一次 Start 才释放 |

### 管理器状态话题

| 话题 | 类型 | QoS | 内容 |
| --- | --- | --- | --- |
| `/coverage/manager_status` | `climbot_interfaces/msg/CoverageStatus` | reliable、transient local、depth 1 | 操作界面所需的全部信息 |

界面所需的一切都在这一个话题上，界面因此不持有任何自己的状态，也不可能和管理器
对当前在跑什么产生分歧。这是 §11.1「面板只负责人机交互」的落地方式：
`climbot_rviz_plugins/Coverage` 面板就只订阅这一个话题并调用上述管理器服务。

| 字段 | 含义 |
| --- | --- |
| `state` | 管理器状态：`IDLE` / `INVALID` / `READY` / `STARTING` / `EXECUTING` / `STOPPING` / `RECOVERY_LOCKED` / `FINISHED` |
| `task_id`、`revision` | 已缓存或正在执行的任务标识；`task_id` 为空且 `revision` 为 `0` 表示从未收到任务 |
| `current_segment` | 执行器上报的当前段；接近首点期间为 `-1`，仅在 `EXECUTING` 有意义 |
| `total_segments` | 来自缓存任务，从 `READY` 起即可用 |
| `progress` | 按预计耗时加权的完成比例，`0`～`1`；起点进入期间固定为 `0`，因为进入段不是任务段 |
| `executor_state` | 执行器运动状态，取值与 `ExecuteCoverage.action` 反馈一致；仅在 `EXECUTING` 有意义 |
| `result_code` | 上一次执行的结果码，取值与 Action 结果一致；仅在 `FINISHED` 有意义 |
| `can_start`、`can_cancel` | 管理器当前是否接受该请求，由它自己服务的前置条件计算；界面直接渲染，不从 `state` 反推 |
| `can_force_abandon`、`can_rearm` | 是否允许放弃未知应答、是否允许在完成外部停车确认后解除恢复锁 |
| `message` | 与管理器日志同一行文本 |

`can_start` 和 `can_cancel` 是提示不是保证：请求仍可能被拒绝（例如执行器 Action
服务未运行），拒绝原因在服务响应里。界面不得据此认为请求一定成功。

### 操作在各状态下的行为

| 状态 | Start | Cancel | Replan / Clear points / Region / Sweep / Algorithm |
| --- | --- | --- | --- |
| `IDLE` | 拒绝，无有效任务 | 拒绝，无执行中任务 | 允许；`Replan` 在点选模式下需先选够点 |
| `INVALID` | 拒绝，无有效任务 | 拒绝 | 同上 |
| `READY` | **接受**，发送 Goal | 拒绝 | 允许，重新生成预览 |
| `STARTING` | 拒绝，已有任务在启动 | 等待 hold 释放时**接受**，丢弃未发送 Goal 后进入恢复锁；已发送 Goal 的正常应答期限内拒绝 | **面板置灰**；服务层仍会受理，只改预览 |
| `EXECUTING` | 拒绝 | **接受**，请求取消 | **面板置灰**；`tracking_mode` 由执行器直接拒绝 |
| `STOPPING` | 拒绝，仍在停机 | **接受**，重试速度保持与取消 | **面板置灰** |
| `RECOVERY_LOCKED` | 拒绝，外部停车尚未确认 | 拒绝，无受监督 Goal handle | **面板置灰** |
| `FINISHED` | **接受**（若仍有缓存任务），重跑该任务 | 拒绝 | 允许 |

任务运行期间面板冻结全部五个规划控件，只留 Cancel。它们发出的请求确实只改预览、
不动运行中的 Goal，但**预览就是画在机器人身上的那条轨迹**，运行中改它看起来就像
任务被换掉了；换形状还会直接把它撤掉。置灰取自管理器自己发布的 `can_cancel`，
面板不另立一套"是否在运行"的判断。

`tracking_mode` 是唯一一个在**服务层**也拒绝的：执行器只在没有任务运行时接受它。

执行中若仍有客户端绕过面板送来新预览，只更新缓存，不改变 `state`，也不改变
`task_id`/`revision`——它们始终标识**当前 `state` 所描述的那个任务**。否则运行中的
机器人会被报成 `Ready` 或 `Idle`，取消按钮随之消失。执行器在接受 Goal 时已复制
任务，运行中的任务本身不受预览影响。

规划失败与"未选择区域"在管理器看来都是空任务，它无法区分，因此都报
`IDLE: no coverage region selected.`。真正的原因只在规划器的 `/coverage/status`
上，面板因此单独显示该话题。

已接受的 Goal 只由结果回调结束，而执行器崩溃时结果永远不会到达。管理器以
`executor_timeout_s`（默认 `5.0 s`）监视 Action 服务的存在：持续消失超过该时间即
进入 `STOPPING`。

`STOPPING` 存在的原因是「Action 服务不可发现」和「机器人已经停下」是两件不同的事。
执行器真的死了会同时给出这两件；而 DDS 发现抖动、Action 通道故障或取消请求没送达
只给出第一件——执行器还活着、`/control/cmd_vel` 仍在刷新、速度看门狗没有超时可
触发、机器人继续在走。此前管理器在这里直接报 `FINISHED` 并释放 Goal handle，而
`can_cancel` 由该 handle 决定，于是操作员的停止入口恰好在最需要它的时刻消失。

进入 `STOPPING` 后管理器做三件事：调用 `/control/hold` 施加速度保持、在 Goal handle
可用时发取消、保持 `can_cancel=true`（此时按停止会重试保持和取消）。保持请求带响应
期限；无应答会重发，`hold_active=false`（包括看门狗重启）也会重新施加。
离开 `STOPPING` 只接受**关于任务和运动**的证据，二者取一：

1. `/control/cmd_vel` 上连续 `command_quiet_s`（默认 `1.0 s`）没有非零指令；
2. 执行器最终应答了，此时用它返回的真实结果而不是 `EXECUTOR_LOST`。

`hold_active=true` 本身不再结束 `STOPPING`：上游仍可能持续发运动指令，看门狗一旦
重启就会恢复运动。两个退出条件都不成立就停在 `STOPPING` 不动——这正是这个状态
存在的目的。`EXECUTOR_LOST`
只由管理器发出，执行器自己永远不会返回它；先前这里复用 `CONTROL_TIMEOUT`，报的是
一个并未发生的原因。实测从 `SIGKILL` 到释放约 `25 s`，其中约 `20 s` 是 DDS 摘除已死
参与者所需的时间，正常退出会快得多。

Start 应答未知时，当前静默不能证明请求以后不会被接受，因此这条 `STOPPING` 默认没有
自动超时逃生口。`/coverage/force_abandon` 是明确的人工风险边界，只在这一分支可用：
调用后退休未知请求的代次并进入 `RECOVERY_LOCKED`，持续请求 `/control/hold=true`，
`can_start=false`。它不发布 `FINISHED`，也不声称任务已停止。操作员确认硬件急停、驱动
失能或执行器进程确已终止后调用 `/coverage/rearm`，管理器才恢复 `READY`；hold 本身仍
保持到下一次 Start 显式请求并确认释放。若被放弃的请求后来竟返回“已接受”，管理器会
立即重新进入 `RECOVERY_LOCKED`、施加 hold，并取消旧 Goal 和当时任何新 Goal。

RViz 的 Force abandon 必须在 `5 s` 内点击两次；第一次只显示上述风险说明。Rearm 是
独立按钮，按钮文字要求完成外部确认。命令行可以直接调用服务，因此使用者承担同一物理
确认责任，不能把脚本调用当成自动故障恢复。

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
`5.0 s`）超时：超时后管理器进入 `STOPPING`、施加速度保持，并且继续等待该请求的
最终应答，不接受新的开始。应答若为拒绝即可安全回到可启动状态；若为迟到接受，管理器
保存其 handle、发取消，并持续监督到结果或运动指令静默。操作员在 handle 尚未返回时
按停止也会成功进入同一流程，而不是因为"还不能发 Action cancel"而丢失停止入口。

超时**并不撤回已经发出的请求**，因此每次开始都带一个单调代次，三个回调
（应答、反馈、结果）都校验自己所属的代次。代次只在请求被拒绝、任务取得最终结果或
明确完成停机后退休；不能仅因应答超时就作废，否则迟到接受会成为无人监督的任务。
新的 Start 在旧代次退休前一律拒绝。即便未来替换成允许并发 Goal 的 Action 服务，旧
回调也不能改写新任务状态或撤掉新任务的停止入口。

只要速度看门狗服务存在，每次新 Start 都先进入 `STARTING` 并显式请求解除保持；只有
`/control/hold` 成功应答或 `/control/hold_active=false` 确认后才真正发送 Action Goal，
避免旧锁存状态或消息到达竞态让 Goal 已开始而轮子仍被 hold 锁住。

这段“已排队、尚未发送 Goal”的 `STARTING` 同样有 `start_response_timeout_s` 上限。解除
hold 超时、执行器在解除后消失，或操作员按 Cancel 时，管理器丢弃队列 Goal、以新代次
重施 `hold=true`，并进入 `RECOVERY_LOCKED`。因为解除请求可能已生效但应答丢失，它不能
直接报 READY；操作员确认停车后才可 Rearm。此时 Force abandon 不适用：它只处理已经发出、
但 Action 接受结果未知的 Goal。

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

`CoverageStatus` 同步转发执行器反馈里的 `planned_total_s`、`schedule_lag_s` 和
`estimated_remaining_s`，定义以 `ExecuteCoverage.action` 为准。

**换形状会撤掉当前预览。** 任务被清空、标记线被删除,要按 `Replan` 用现有点重建,
或清点重选。下拉框是在问"这些点是什么形状",不是在下令规划——梯形 3 点切回矩形时
它曾经悄悄按前 2 点画出一条谁也没要求过的新轨迹。换扫描方向不撤预览:它不改变
区域本身,只是换个方向画同一块地。

**换形状不丢点。** A、B 两点在矩形和梯形里是同一个角,所以矩形选好 2 点后切到梯形
只是等第 3 点。丢点会让下拉框上的一次误点变成不可逆操作。

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
| `detection_length` | m | 沿行进方向的检测有效长度；G2 标称 `0.28125`，旧回归配置保留 `0.01` |
| `detection_forward_offset` | m | 检测中心相对 `base_link` 的前向偏移；巡检相机为 `0.340` |
| `detection_edge_overlap` | m | 兼容保留参数；可走区优先语义下不改变蓝线或真实检测足迹，当前不参与覆盖面积计算 |
| `overlap_ratio` | `[0, 1)` | 相邻扫描带的**横向**重叠率；默认 `0.20`，独立于拍照触发重叠 |
| `robot_length`、`robot_width` | m | launch 从 `robot.yaml` 注入 |
| `edge_clearance` | m | launch 从 `robot.yaml` 注入 |
| `wall_width`、`wall_height` | m | launch 从 `wall.yaml` 注入 |
| `path_height` | m | RViz 路径离墙显示高度 |
| `bottom_warning_tolerance` | m | 梯形底边点高度修正提示阈值 |
| `minimum_nominal_coverage_ratio` | `[0, 1]` | 规划期可选覆盖率门限；默认 `0` 只报告、不因覆盖不足拒绝安全路径 |
| `top_edge_scan` | `auto` / `always` / `never` | 顶部收边扫描，默认 `never`；`auto` 需配置正覆盖率门限 |

规划器内部计算：

```text
row_spacing = detection_width × (1 - overlap_ratio)
safety_margin = 0.5 × hypot(robot_length, robot_width) + edge_clearance
```

`inspection_geometry_profile` 是 `coverage_planner.launch.py` 的 launch 参数，不写入
`CoverageTask`：`calibrated`（默认）从共享相机描述注入物理足迹和前向偏移；`configured`
严格采用 planner YAML，用于复现冻结的历史回归。无论 profile 如何，`overlap_ratio` 都只
来自任务 YAML；纵向照片重叠由巡检节点的 `image_overlap_ratio` 独立决定。

矩形点选顺序为 A（左下）、B（右上）；等腰梯形为 A（左下）、B（右上）、
C（右下）。A、C 的高度取平均值修正为水平底边。

由于消息字段需要兼容现有 Action 和归档，`coverage_region` 现承载用户选择的机器人任务
可走区；`motion_region` 承载整面墙的绝对安全运动区。相机实际覆盖不是单个 Polygon，
而是从 `SCAN` 与检测足迹推导的扫掠带并集。以下参数和输出已经进入规划／执行接口：

| 项目 | 含义 |
| --- | --- |
| `detection_length` | 沿行进方向的检测有效长度；与 `detection_width` 共同形成二维检测足迹 |
| `turn_clearance` | 竖向底部转向相对绝对安全边界的安全余量，初值 `0.06 m` |
| `/control/reference_path` | 转向后根据 EKF 实际位置生成的动态直线执行参考，仅供显示和评价 |

竖向为主时，控制器以第一次转向后的实际位置为斜直线 `TRANSITION` 起点，第二次
转向后按统一入轨判据冻结与名义线平行的 `SCAN`，不得逐列倒车返回名义起点。覆盖率
必须按二维检测足迹重新计算并报告；只有合同门限或操作员显式要求时才增加顶部水平收边扫描。

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
| `auto` | 仅当**预计**覆盖率低于显式正的 `minimum_nominal_coverage_ratio` 时追加 |
| `always` | 只要蓝色收边线能放进橙色任务可走区就追加 |
| `never` | 从不追加 |

收边蓝线位于橙色任务可走区顶边，相机足迹自然越过该边缘。进入方向取距离上一条扫描
线终点较近的一端，避免为了上线横穿整个区域。两个端点必须都在橙色任务可走区
内，否则不追加并在 `/coverage/status` 说明原因——宁可少扫，也不生成一个执行器会在
接受 Goal 阶段拒绝的任务。

**只对竖向扫描生效。** 横向扫描的最高一条扫描线已经按检测宽度布置，另加收边会
重复同一覆盖用途。`always` 配横向扫描时
不追加，并在状态里说明。

**`auto` 用的是预计覆盖率，看不到执行损失。** 默认模式为 `never`；部署显式配置
`auto` 和正数门限时，它才会在预计比例不足时尝试追加。实测覆盖不足时，
操作员可用 `always` 增加一条仍位于橙色可走区内的顶边扫描，或扩大可走区后重新规划，
而不是让机器人为追求规则覆盖边界越过限制。

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
float64 detection_forward_offset
```

`header.frame_id` 是所有路点和两个 Polygon 的共同坐标系，默认 `odom`；
`header.stamp` 是该规划版本的生成时间。`N` 个 `waypoints` 必须对应 `N-1` 个
`segment_types`，其中 `segment_types[i]` 描述 `waypoints[i] → waypoints[i+1]`。
路点姿态指向下一段，最后一个姿态沿用到达航向。

`detection_forward_offset` 是检测中心沿机器人前向相对 `base_link` 的偏移，普通工具可
为 `0`，G2 前置相机任务为 `+0.340 m`。

接收方必须拒绝以下任务：

- `task_id` 为空、`revision` 为零或扫描方向非法；
- 路点少于两个，或线段类型数量不是路点数量减一；
- 坐标、四元数、检测尺寸／偏移包含非有限值，检测尺寸不为正或偏移为负；
- 存在零长度线段、未知线段类型或非法四元数；
- 线段类型为 `SEGMENT_RETURN`。该常量保留在消息里，只是为了编号不会挪动、不会让
  已录制的 bag 换个含义；但没有任何执行器为它定义过行为，收下它等于按默认分支去走一段
  谁都没定义、谁都没测过的轨迹。在任务边界上拒绝，意味着真要用它的人在启动时就知道，
  而不是从跑出来的形状里猜；
- Polygon 非法、`coverage_region` 不在 `motion_region` 内，或任一名义路点位于用户
  `coverage_region` 外；
- 显式配置了正数覆盖率门限，且 `SCAN` 对用户可走区达不到该门限。默认门限为零，只报告。

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
当前规划器以原始用户区域填充 `coverage_region`（任务可走区），以整面墙按机器人安全
边距内缩的多边形填充 `motion_region`（绝对安全区）。扫描方向上的蓝色端点直接取橙色
边界，不再为相机偏移向外延长；横向布线仍按检测宽度和重叠率均匀分配。黄色相机覆盖
由蓝线叠加 `detection_forward_offset` 和真实二维足迹得到。规划器报告橙区被预计拍到的
比例；`minimum_nominal_coverage_ratio=0` 为默认的“尽量覆盖”模式，只有显式配置正数
门限时才因覆盖不足拒绝。
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
uint16 LOCALIZATION_TIMEOUT=3
uint16 CONTROL_TIMEOUT=4
uint16 OUT_OF_BOUNDS=5
uint16 TRACKING_FAILED=6
uint16 EXECUTOR_LOST=7
uint16 ARCHIVE_FAILED=8

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
float64 planned_total_s
float64 schedule_lag_s
float64 estimated_remaining_s
```

时间三项与 `progress` 并列而不是合并进去，因为它们回答的是两个不同问题：
`progress` 说**做完了多少工作量**，机器人卡住时它正确地停住不动；后三项说
**跟不跟得上计划**，一个由 elapsed 驱动的进度条会在机器人卡死时照样走到 100%。

| 字段 | 含义 |
| --- | --- |
| `planned_total_s` | 任务开始时算定，全程不变，含到第一个路点的接近段 |
| `schedule_lag_s` | 正数为落后计划。`distance` 模式恒为 `0`——没有时间表就谈不上落后 |
| `estimated_remaining_s` | 每周期更新并带上当前累计滞后，所以落后时它变长而不是匀速递减 |

接近段在 `planned_total_s` 里计入，但**不进入 `progress` 的分母**——进度条数的是任务
的段，接近段不是其中之一。该段的转向和直线各自按自己的完成度递减，否则倒计时会
在整条腿上纹丝不动、到达时再一次性跳掉一整段。

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

## 三点法向载荷测量参数

`measure_normal_loads.py` 的主要参数：

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `static_headings_deg` | `0°～345°`，间隔 `15°` | 逐一停稳并测量三轮负载的静态航向；空、重复或非有限值拒绝 |
| `static_settle_duration_s` | `1.0` | 每个静态航向到达后的停稳时间 |
| `static_duration_s` | `10.0` | 每个静态航向的负载记录时长 |
| `drive_duration_s` | `8.0` | 每个正交方向的直行记录时长 |
| `brake_duration_s` | `2.0` | 每个正交方向直行后的制动记录时长 |
| `turn_angle_deg` | `360.0` | 顺、逆时针分别记录的累计原地转角 |
| `linear_speed_mps` | `0.15` | 直行速度 |
| `angular_speed_rps` | `0.6` | 原地转向速度 |
| `contact_timeout_s` | `0.15` | 接触消息超过该时长未更新即按零载荷处理 |
| `turn_timeout_s` | `25.0` | 单次对准或完整转向的超时 |
| `output_csv` | `results/normal_loads.csv` | 逐工况、逐接触点结果 |

默认依次记录 24 个静态航向、右／左／上／下直行及各自制动、顺／逆时针完整原地
转向。CSV 写出工况类别和航向，并对每个工况和接触点写出均值、最小值、最大值、
样本数、零载荷样本数和接触率。接触传感器不发布的时刻按零载荷处理，不沿用失联前
的最后一个正值。

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

`measure_turn_band.py` 用固定小转角检查这个标定在全航向是否仍然平坦，G1 正式运行
默认 `0°～345°` 每 `15°`、正反各 `30°`，共 48 条：

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `headings_deg` | 24 个航向 | 起始航向，每 `15°` 一档 |
| `angles_deg` | `[30,-30]` | 每个起始航向的正反转角 |
| `output_csv` | `results/turn_map.csv` | 逐次转向原始记录 |
| `summary_json` | 空 | 非空时写严格 JSON 判定与 Git 溯源 |
| `maximum_mm_per_deg` | `0.55` | 任一航向侧滑绝对上限 |
| `maximum_range_mm_per_deg` | `0.10` | 48 条中最大值减最小值上限，用于排除局部滑移带 |
| `maximum_turn_error_deg` | `2.0` | 实际转角相对命令转角的最大误差 |

记录数或三个门限任一项不满足时，脚本保留 CSV/JSON 后以非零状态退出。改变质量、
质心、吸附、摩擦或 WheelSlip 后必须从全新仿真重跑。

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

## 定位对照参数（§14.5）

`evaluate_localization.py` 驱动一次四方向闭环行驶，逐段用 Gazebo 真值同时测融合位姿
误差和轮式航位推算误差。它自己发 `/control/cmd_vel`，**不需要跟踪器**。

| 参数 | 默认值 | 行为 |
| --- | ---: | --- |
| `segment_duration_s` | `8.0 s` | 每个方向的行驶时长 |
| `linear_speed_mps` | `0.15 m/s` | 行驶速度 |
| `turn_tolerance_deg` | `1.0°` | 方向切换的对准容差 |
| `turn_timeout_s` | `25.0 s` | 单次对准超时 |
| `settle_duration_s` | `1.0 s` | 段末静止等待，让 `12 Hz` 的延迟观测追上 |
| `heading_hold_gain` | `1.5` | 行驶中的航向保持增益 |
| `summary_json` | 空 | 非空时落盘逐段记录与判定摘要 |

摘要里两个误差的**口径不同,不能互换**：

- `ekf_position_error_m` 是**绝对位置**误差。融合位姿估计的就是墙面位姿本身；
- `wheel_dead_reckoning_error_m` 是**位移**误差——航位推算认为自己走了多远，对上真值
  实际走了多远。

轮式里程计的 `odom` 锚在出生点，首采样是 `(0, 0, 0)` 而真值在 `(0, 2.0)`，按绝对位置
比会在它还没动的时候先记上 `2 m` 的出生点偏置。摘要保留 `start` 一条记录作为坐标系
自检：**融合从真值上开始，轮式从零开始**，两者一眼可辨。

`passed` 为真只要求最大轮式位移误差大于最大融合误差；`wheel_to_ekf_maximum_ratio`
给出倍数，实测 `194 倍`，见 `results/README.md`「§14.5 融合定位相对轮式里程计的
优势」。

## TF

| 变换 | 发布者 | 说明 |
| --- | --- | --- |
| `world → wall` | static transform publisher | 来自 `wall.yaml` |
| `wall → odom` | static transform publisher | 当前为单位变换 |
| `odom → base_link` | `robot_localization` | 连续融合位姿 |
| `base_link → links` | `robot_state_publisher` | 来自共享 URDF 和 `/joint_states` |
| `base_link → inspection_camera_link` | `robot_state_publisher` | 相机机械安装位置，来自共享描述 |
| `inspection_camera_link → inspection_camera_optical_frame` | `robot_state_publisher` | ROS 光学轴约定；合成外参见 PROJECT_GUIDE §18.1 |

## `climbot_inspection` G1 相机接口

G1 使用标准 ROS 图像接口，命名不带 `gz`，使仿真相机和真机驱动可直接替换：

| 名称 | 类型 | QoS／行为 |
| --- | --- | --- |
| `/inspection/camera/image_raw` | `sensor_msgs/msg/Image` | 畸变原图，`1920 × 1080`；每次成功触发发布一帧 |
| `/inspection/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 与原图同时间戳、同 `frame_id`，Transient Local 不用于替代逐帧匹配 |
| `/inspection/capture_once` | `climbot_interfaces/srv/CaptureOnce` | G1 人工单拍；成功返回前对应图像必须已发布。`reason` 是稳定枚举：`OK`、`WARMING`、`BUSY`、`DRAINING`、`TIMEOUT`，调用者不得解析英文 `message`。 |
| `/inspection/capture_state` | `std_msgs/msg/UInt8` | 单拍节点的 transient-local 当前状态，编码与 `CaptureOnce.reason` 对应：`OK`=可接受请求、`WARMING`=预热、`BUSY`=当前曝光、`DRAINING`=排空可能迟到帧。 |
| `/inspection/capture_reset` | `std_srvs/srv/Trigger` | 人工恢复入口；`WARMING` 或正在曝光时明确拒绝，其他临时状态重新开始 `warmup_quiet_s` 的排空期。不能取消一个正在等待的曝光。 |

`image_raw.header.frame_id` 和 `camera_info.header.frame_id` 均为
`inspection_camera_optical_frame`。两条消息必须具有相同时间戳；消费者以时间戳配对，
不得用“最近一条标定消息”掩盖分辨率或标定版本切换。服务并发请求串行化；已有请求
未完成时新请求明确拒绝，不合并、不悄悄多拍。超时返回失败后节点进入短暂 `DRAINING`：
排空已有图像／标定消息，并以触发时刻作为下一帧的因果时间下界，随后自动重新预热。
因此迟到帧不得被记到下一次请求，但一次可恢复的丢帧也不要求操作员重启节点。G1 不
提供自动连拍服务。

`CameraInfo` 使用 `plumb_bob`。`D` 的顺序为 `[k1,k2,p1,p2,k3]`，标称
`K/D/P`、有效区域和 `base_link → optical` 外参见 PROJECT_GUIDE §18.1～18.2。
Gazebo SDF 中畸变标签的书写顺序不等于 ROS 数组顺序，桥接或适配节点必须按字段名
映射，不能直接按位置复制。原图必须保留畸变；如提供
`/inspection/camera/image_rect`，其 `CameraInfo` 必须描述实际校正后的 `P/ROI`，不能
沿用非零畸变原图的语义。

G1 中唯一允许使用 Gazebo 真值的是独立验收程序。`capture_once`、相机驱动和未来 G2
触发节点均不得订阅 `/model/climbot/ground_truth`。

## `climbot_inspection` G3 平场补偿接口

`calibrate_flat_field` 默认依次调用 `/inspection/capture_once` 30 次，且相邻请求至少
间隔 `0.10 s`。每帧必须具有唯一时间戳和唯一 SHA-256；所有像素的平均时间标准差须
不低于 `0.10 DN`。输出 NPZ 至少保存 `gain`、`mean_image`、帧数、唯一时间戳数、唯一
哈希数和实测噪声，运行节点载入时再次拒绝样本数不足或含重复帧的文件。
仿真默认 `target_mean_dn=180`，用于让较低反射率的墙面保持可读亮度；该值写入 NPZ。

| 名称 | 类型 | 语义 |
| --- | --- | --- |
| `/inspection/camera/image_raw` | `sensor_msgs/msg/Image` | 保留的原始畸变 `mono8` 图 |
| `/inspection/camera/image_compensated` | `sensor_msgs/msg/Image` | 固定平场增益后的 `mono8` 图；header 与原图相同 |
| `inspection_flat_field_target` | launch 参数 | 仅仿真标定时在当前视场放置纯灰低反光板；正常任务必须为 false |
| `flat_field_file` | launch 参数 | 非空时启动补偿节点并加载 NPZ；空值保持原图链路不变 |

仿真最终输出端白噪声标准差为 `0.004 × 255 ≈ 1.02 DN`，随机种子固定用于回归，但
每次曝光继续推进随机序列。SDF 渲染器内部的噪声配置不能作为独立帧保证。

`image_compensated` 是可选预览接口，不属于正式归档输入。正式数据记录器只能消费
`image_raw`；未提供 `flat_field_file` 时补偿节点不启动。

## G4 任务归档接口

| 名称 | 类型 | 方向与权威语义 |
| --- | --- | --- |
| `/inspection/archive/prepare` | `climbot_interfaces/srv/PrepareInspectionArchive` | 管理器 → 记录器。提交冻结 `CoverageTask` 与输出根目录；成功响应给出 `run_id`、绝对目录、名义容量预估。此响应只代表归档准备完成，随后才允许发送运动 Goal。 |
| `/inspection/archive/finalize` | `climbot_interfaces/srv/FinalizeInspectionArchive` | 管理器 → 记录器。按 `run_id` 封存为 `COMPLETED`／`CANCELED`／`FAILED`；完成时严格核对冻结参考计划、实际归档张数以及每段相邻实际曝光位置的最大间距。 |
| `/inspection/archive/status` | `climbot_interfaces/msg/InspectionArchiveStatus` | 记录器 → 管理器／RViz，Reliable + transient-local。它是本次归档状态、实际预计张数、保存／失败计数和最终目录的权威来源。 |
| `/simulation/inspection_camera/trigger` | `std_msgs/msg/Bool` | 仅 Gazebo 内部：`capture_once_node` → 相机传感器的单次触发 topic。真机不实现该接口。 |
| `/simulation/inspection_camera/ideal_image`、`ideal_camera_info` | `sensor_msgs/msg/Image`、`CameraInfo` | 仅 Gazebo 内部的无镜头畸变渲染输出。 |
| `/simulation/inspection_camera/image_raw`、`camera_info` | `sensor_msgs/msg/Image`、`CameraInfo` | 仅 Gazebo 内部的畸变适配器输出；桥接到正式 `/inspection/camera/*` 输入前必须时间戳成对。 |

G4 的 `climbot_inspection` 任务级记录器输入 `image_raw`、`CameraInfo`、
`InspectionCapture`、`ExecutionReference` 和冻结任务快照，输出到受配置约束的任务目录。操作员选择根目录，
节点生成并校验任务子目录；不得直接拼接未经校验的任务 ID 形成路径。第一版使用
`<output_root>/<safe-task-id>/r<revision>_<UTC-time>_<run-id>/`，同一 revision 重跑也必须
产生新目录，绝不覆盖旧数据。

建议的第一版目录契约：

```text
output_root/
└── safe-task-id/
    └── r000123_20260825T103015Z_run-id/
        ├── manifest.json
        ├── calibration/
        │   ├── camera_info.yaml
        │   ├── camera_extrinsics.yaml
        │   └── flat_field_reference.json
        ├── images/raw/000000.png
        └── metadata/000000.json
```

单张标签以 `(task_id, revision, segment_index, trigger_index)` 为业务主键，以图像 header
为曝光配对键。PNG 必须是原始畸变 `mono8` 像素的无损编码，并记录文件 SHA-256；标签
保存墙面相机位置、航向、完整协方差、目标／实际沿轨位置和所有标定哈希。`manifest`
保存任务区域、规划、软件提交、名义容量预估、冻结扫描参考、最终计数和失败清单。名义
计数只能用于开始前预留空间；每个首次生效的正式冻结 SCAN 参考按
`ceil(L / (detection_length × (1 − overlap)))` 累加最终预期数，并且必须与已归档照片数
严格相等；任务完成时也必须已经见到任务全部 SCAN 段的冻结参考。触发点不要求落在终点
精确位姿，最后一个点保留在终点前一个间隔内，以适配控制器的终点容差。只有图片和标签
均经临时文件写入、校验并原子改名后才增加成功计数；任务结束时必须验证主键、文件名和
时间戳一一对应。

不完整的 `image_raw`／`CameraInfo`／`InspectionCapture` 三元组在 `pair_timeout_s` 后仅计入
`failed_images` 与 manifest 的失败清单，记录器保持 `RECORDING`，使上游有机会补拍；是否可
以 `COMPLETED` 封存仍由冻结段、成功张数和实际几何三项最终裁决。每段相邻标签的
`actual_along_track_m` 间距不得超过 `detection_length × (1 − overlap)` 加
`actual_spacing_tolerance_m`，且单帧相对 `target_along_track_m` 的后滞不得超过
`maximum_target_lag_m`。这些是归档时的运行时质量门，不只是离线 G2 评估指标。
自动采集器的位置闸门把尚未完成的目标限制在更小的 `capture_gate_max_lag_m` 内；记录器
的门仍是最终独立裁决，不能以控制器是否停车代替。

`/coverage/manager_status` 为这两个口径分别提供
`archive_preflight_expected_images` 和 `archive_expected_images`：前者是开始前固定的名义
全任务容量预估，后者是目前已冻结 SCAN 参考的累计实际计划。RViz 必须同时标示两者，
不得把尚未冻结全部扫描线时的后者显示成任务总预计数。

### G4 任务启动与 RViz 接口

现有 `/coverage/start` 不携带任务选项。G4 第一版已在 `climbot_interfaces` 增加一个
由管理器提供的带选项启动服务 `/coverage/start_configured`
（`climbot_interfaces/srv/StartCoverage`），请求至少携带 `inspection_enabled` 和
`output_root`。响应只确认“管理器已经接受这次准备请求”：归档准备是异步的，不能在 ROS
服务回调中阻塞等待另一个服务；成功后的 `run_id`、记录器解析出的绝对任务目录和错误
说明以 `/coverage/manager_status` 为权威。RViz 面板只调用这个管理器接口，管理器内部
完成“准备归档 → 成功后发送执行 Goal”的顺序，不允许 UI 自己拼接两次服务调用。旧
`/coverage/start` 保留给命令行和兼容测试，使用管理器参数中的
`inspection_default_enabled` 与 `inspection_output_root`：裸 `coverage_manager_node`
默认 motion-only，完整 `coverage_mission.launch.py` 显式传入 `inspection:=true` 和默认
根目录 `~/climbot_data`。具体字段可扩展，但这套异步与权威状态语义不得改变。

管理器状态将聚合记录器的权威状态，供面板显示：是否启用、`PREPARING`／`READY`／
`RECORDING`／`FINALIZING`／`COMPLETED`／`CANCELED`／`FAILED`、预计／成功／失败照片数、
最终目录和最后错误。采集启用时，归档准备失败必须使 Start 失败且不发送运动 Goal；
执行中发生不可恢复的归档错误默认请求受控取消。取消或运动故障只改变 manifest 结果，
不得删除已经提交的照片。

归档封存 RPC 使用 `archive_finalize_timeout_s`（默认 `5.0 s`）这一稳态时间期限。记录器在
收到请求后消失、因而永远不回响应时，管理器将归档记为 `FAILED`、保留已经写入的部分目录，
并重新允许操作员启动下一项任务；迟到响应以代次隔离，不能覆盖这一失败结论。准备阶段在
创建 run 目录后若标定快照或首份 manifest 写入失败，则删除尚未发布的半成品 run 目录。

RViz 仍使用一个 `Coverage Task` dock，布局为：顶部公共状态和采集摘要；中间三个页签
`任务规划`、`巡检采集`、`详情`；底部固定 Start、Cancel / Stop。恢复操作只在管理器进入
对应异常状态时临时显示，平时不占用任务面板高度。采集页提供
任务级开关、根目录编辑／浏览／恢复默认、预计照片数量与空间、保存计数、最终目录和
错误。Start 后这些设置冻结，直到本次任务封存。路径由记录器所在主机解释；分机部署时
本地文件选择器只是一项便利，记录器返回的绝对路径才是权威结果。

离线 `climbot_image_processing` 先在畸变原图坐标中应用暗场／平场，再去畸变；
`climbot_mosaic` 以 EKF 位姿和协方差为绝对先验，以重叠特征匹配为相对约束做鲁棒位姿
图优化或束调整。优化目标是加权残差而不是直接压低协方差；输出必须附残差、内点率、
连通性和后验不确定度。

## `climbot_inspection` G2 自动采集接口

| 名称 | 类型 | 语义 |
| --- | --- | --- |
| `/control/execution_reference` | `climbot_interfaces/msg/ExecutionReference` | 执行器当前冻结的有向直线、任务版本、段号、段类型和采集许可；不是规划器的名义预览 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 触发位置和曝光时间位姿插值的唯一业务定位源 |
| `/inspection/capture_metadata` | `climbot_interfaces/msg/InspectionCapture` | 一张成功原图的任务、触发点、冻结参考和曝光时刻 EKF 相机位姿 |
| `/inspection/capture_gate` | `climbot_interfaces/msg/InspectionCaptureGate` | 自动采集器 → 跟踪器，Reliable + transient-local。启用采集的 `SCAN` 必须持续收到同一任务／版本／段的 gate；`active=true` 时相机中心不得越过 `maximum_camera_along_track`，成功配对、禁用或离开该段后发布 `active=false`。释放消息一律携带当前参考的任务／版本／段号，不得回退到上一段的身份。`reason` 是给操作员看的自由文本，调用方不得据其内容分支。 |

`ExecutionReference.inspection_enabled` 只在正式 `SCAN` 的 `TRACK_LINE`／
`FINAL_APPROACH` 为真；起点进入、对准、转向稳定、动态过渡和小弧线入轨均为假。
`detection_forward_offset` 来自不可变任务，G2 相机任务必须为 `0.340 m` 并与共享安装
外参一致。

自动采集的参考失效、等图像和等 EKF 插值括号期限全部用稳态时间测量，即使 Gazebo
暂停、`use_sim_time` 的 ROS 时钟不前进，`image_wait_timeout_s` 仍会按墙钟重试同一空间
触发点。重试不是让机器人带着未完成曝光继续前进：采集器在目标位置加
`capture_gate_max_lag_m`（默认 `15 mm`）处发布位置闸门，跟踪器据此限速并可停车；只有
相机帧已和 EKF 位姿完成绑定后才放行下一目标。该值必须小于记录器
`maximum_target_lag_m`（默认 `25 mm`），且闸门只接受冻结参考的任务 ID、版本和段号
都相同的消息。可选平场节点在启动时即校验 NPZ gain 与共享相机的 `1920×1080` 分辨率
相符、有限且严格为正；不兼容标定不等待第一帧才暴露。运行中遇到异常图像只丢弃补偿
预览，不影响正式 `image_raw` 归档。

gate 是存活监督而不只是位置上限：跟踪器从进入启用采集的 `SCAN` 起，使用本机稳态时钟
在一次性的 `capture_gate_start_timeout_s`（默认 `2.0 s`）建立窗口内等待第一条匹配 gate，
收到后要求每条消息在 `capture_gate_timeout_s`（默认 `0.50 s`）内刷新。首次或后续心跳超时
均发布零速度并以 `TRACKING_FAILED` 中止 Action，不能把过期 gate 当作 inactive 后继续运动。
中止与等待消息都会附上最后一条 gate 的 `reason` 与其段号（匹配与否都附），因为一条来自
别的段的 gate 本身就是诊断。`active=false` 的含义是“无需等待，继续行驶”，因此当采集器
判定本段根本无法拍摄——例如任务的 `detection_forward_offset` 与相机安装不符——它不发布
任何 gate（`active` 与否都不发），让心跳超时按上述路径快速停段，而不是放行机器人开完
一条注定没有曝光的扫描线、到归档收尾时才发现为空。
`header.stamp` 只用于消息可追溯性，不能用于时效判断，因为
`use_sim_time` 暂停时它不会前进。完整 inspection launch 同时强制
`capture_gate_max_lag_m < maximum_target_lag_m`、采集器与记录器的纵向重叠一致；v1 gate
接口明确要求 `camera_mount_y_m == 0`，非零横向安装须先扩展接口。

`InspectionCapture.header` 必须逐字段等于对应 `image_raw.header`。`camera_pose` 是光学
中心在 `header.frame_id` 下的 EKF 插值位姿，协方差包含前置杠杆对航向不确定度的传播；
`wall_heading_rad` 是机器人前进方向在墙面内的航向，亦为标称安装下图像向上的方向。
消费者以 `(task_id, revision, segment_index, trigger_index)` 作为业务主键，以图像时间戳
核对数据配对，不按消息到达顺序猜测。

仿真内部接口不属于真机公共 API：Gazebo 在
`/simulation/inspection_camera/trigger` 收到触发后发布较宽视场的
`ideal_image/ideal_camera_info`，仿真畸变适配器再输出
`/simulation/inspection_camera/image_raw` 和 `camera_info`。1920×1080 RGB 图像采用
Reliable、depth 1，避免大帧在同机桥接和 Python 适配之间被 Best Effort 静默丢弃；
`climbot_inspection` 只消费适配后的成对消息。

`climbot_inspection/config/inspection.yaml` 定义源／输出话题、服务名、期望尺寸与
`frame_id`，以及 `capture_timeout_s`、传输发现稳定时间和预热重试／静默窗口。启动
预热帧只用于建立桥接与渲染传输，全部丢弃；预热输出静默后服务才进入 READY。标准
`coverage_sim.launch.py` 和 `coverage_mission.launch.py` 默认启动该节点，可用
`inspection:=false` 关闭。

## 配置文件

| 文件 | 内容 |
| --- | --- |
| `climbot_description/config/robot.yaml` | 机器人共享物理属性、限幅和 footprint |
| `climbot_description/config/inspection_camera.yaml`（G1 新增） | 相机内参、畸变、有效足迹和标称安装外参 |
| `climbot_description/config/wall.yaml` | 工作坐标系、墙面宽高、参考网格线间距 |
| `climbot_gazebo/config/simulation.yaml` | Gazebo 专有物理和传感器参数 |
| `climbot_gazebo/config/ekf_wall.yaml` | EKF 状态选择、频率和协方差输入 |
| `climbot_coverage/config/coverage_interactive.yaml` | RViz 点选工作流，`coverage_mission.launch.py` 的默认 |
| `climbot_coverage/config/coverage_rectangle.yaml` | 参数模式的矩形任务 |
| `climbot_coverage/config/coverage_trapezoid.yaml` | 默认等腰梯形任务 |
