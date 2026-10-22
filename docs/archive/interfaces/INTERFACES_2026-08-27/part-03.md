# 历史分册 3

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

