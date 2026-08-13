# ROS 2 接口与配置索引

本文档记录当前已实现接口，以及已经冻结但尚未实现的阶段 E 第一版任务与控制接口。

## 启动入口

| 命令 | 用途 |
| --- | --- |
| `ros2 launch climbot_gazebo climbot_wall.launch.py` | Gazebo、桥接、TF、传感器适配和 EKF |
| `ros2 launch climbot_coverage coverage_planner.launch.py` | 独立覆盖规划器和可选 RViz |
| `ros2 launch climbot_coverage coverage_sim.launch.py` | 当前阶段联合启动仿真、规划器和 RViz |
| `ros2 launch climbot_control line_tracker.launch.py` | 单段直线跟踪器；从共享描述注入轮距和轮缘硬限值 |

`climbot_wall.launch.py` 的主要 launch 参数：

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `use_sim_time` | `true` | 使用 Gazebo 时钟 |
| `headless` | `false` | 无 GUI 启动 Gazebo server |
| `gpu_backend` | `auto` | `auto`、`wsl_d3d12` 或 `native` |
| `total_station_rate_hz` | `12.0` | 模拟全站仪频率 |
| `total_station_stddev_m` | `0.005` | 全站仪位置一倍标准差 |
| `total_station_delay_s` | `0.05` | 固定传输延迟 |
| `wheel_forward_velocity_stddev_mps` | `0.03` | 轮式前向速度标准差 |
| `wheel_yaw_rate_stddev_rps` | `0.05` | 轮式角速度标准差 |
| `imu_orientation_stddev_rad` | `0.00872664626` | IMU 姿态标准差（0.5°） |

## 仿真与定位话题

| 话题 | 类型 | 生产者 | 消费者/用途 |
| --- | --- | --- | --- |
| `/control/cmd_vel` | `geometry_msgs/msg/Twist` | E3 直线控制器、未来任务状态机 | E4 速度看门狗；控制层唯一入口 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | E4 速度看门狗 | Gazebo DiffDrive；每 `20 ms` 重发 |
| `/model/climbot/ground_truth` | `nav_msgs/msg/Odometry` | Gazebo | 仅模拟传感器和评价 |
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

`/model/climbot/ground_truth` 不得成为轨迹控制器输入。模拟全站仪可以由它派生，
但必须经过频率、噪声、延迟和坐标变换模型。

`climbot_wall.launch.py` 始终启动 `cmd_vel_watchdog_node`，它是唯一的执行器控制
输出者：以 `50 Hz` 从
`/control/cmd_vel` 转发到 Gazebo `/cmd_vel`，在未收到首条指令、仿真时钟倒退，或
距最后一条指令超过 `command_timeout_s`（默认 `0.40 s`）时发布零速。键盘、四个实验
脚本和自动控制器均发布 `/control/cmd_vel`。当前采用单一上游来源规则；需要遥控抢占
自动任务时再增加带明确优先级的仲裁器。

当前 EKF 以 `50 Hz` 发布 `/odometry/filtered`，阶段 E 控制器默认也以 `50 Hz`
运行。全站仪 `12 Hz` 只表示绝对位置更新频率，控制器不得将每个 50 Hz EKF
输出误认为新的独立绝对测量。

### 控制安全参数

| 节点 | 参数 | 默认值 | 行为 |
| --- | --- | ---: | --- |
| `line_tracker` | `odometry_timeout_s` | `0.25 s` | 超过该时间未收到有效融合位姿时持续发布零速，并清空上一控制指令 |
| `line_tracker` | `control_frequency_hz` | `50 Hz` | 直线跟踪控制频率 |
| `cmd_vel_watchdog` | `command_timeout_s` | `0.40 s` | 上游速度指令超时后持续发布零速 |
| `cmd_vel_watchdog` | `publish_rate_hz` | `50 Hz` | 向执行器重发安全速度的频率 |

两个节点都拒绝非有限输入。直线跟踪器在启动时校验线段、增益、速度、加速度、
轮距、重力方向和坐标系参数；非法配置直接启动失败，不进入周期回调。轮距、轮缘
速度硬限值和轮缘加速度硬限值不存放在 `control.yaml`，由标准 launch 从
`climbot_description/config/robot.yaml` 注入。

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
转向后直接从实际位置开始下一条竖直 `SCAN`，不得逐列倒车返回名义起点。覆盖率
必须按二维检测足迹重新计算；低于 98% 时增加一条顶部水平收边扫描。

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
路点必须位于 `motion_region`，否则拒绝规划。标称覆盖率低于 98% 也会拒绝规划。
竖向执行中的转向下滑属于实际轨迹问题，E9 将用同一足迹评估实际轨迹，并在需要时追加
一条顶部水平收边扫描。

### `ExecuteCoverage.action`

Action 名称冻结为 `/coverage/execute`，将在 E7 实现。使用 ROS 2 Action 不表示接入 Nav2。

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

### 执行与版本规则

1. `task_id` 标识一次用户任务；对同一任务重新规划时 revision 严格递增。
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
