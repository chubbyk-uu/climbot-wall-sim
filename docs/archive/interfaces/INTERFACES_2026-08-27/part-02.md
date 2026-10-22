# 历史分册 2

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
| `noise_sources` | `total_station_sim` 的位置种子、`stddev`、频率、延迟、丢包率、profile、棱镜残差开关／向量、时间戳开关／偏差／抖动／种子；`wall_imu_adapter` 的种子和 `stddev` | **向节点的参数服务问回来** |
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

