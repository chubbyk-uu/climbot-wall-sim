# ROS 2 接口与配置索引

本文档记录当前已实现接口。未来控制器接口尚未冻结，实施时必须同步更新。

## 启动入口

| 命令 | 用途 |
| --- | --- |
| `ros2 launch climbot_gazebo climbot_wall.launch.py` | Gazebo、桥接、TF、传感器适配和 EKF |
| `ros2 launch climbot_coverage coverage_planner.launch.py` | 独立覆盖规划器和可选 RViz |
| `ros2 launch climbot_coverage coverage_sim.launch.py` | 当前阶段联合启动仿真、规划器和 RViz |

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
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 键盘、实验脚本、未来控制器 | Gazebo DiffDrive |
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

同一时刻只能有一个任务级 `/cmd_vel` 发布者。阶段 E 必须加入独立速度看门狗，
在指令超时后发布零速。

当前 EKF 以 `50 Hz` 发布 `/odometry/filtered`，阶段 E 控制器默认也以 `50 Hz`
运行。全站仪 `12 Hz` 只表示绝对位置更新频率，控制器不得将每个 50 Hz EKF
输出误认为新的独立绝对测量。

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

阶段 E 接入前需要补充与相邻 Path 位姿段一一对应的 `SCAN`、`TRANSITION`、
`RETURN` 类型元数据。该接口尚未冻结；控制器不得从 Marker 的颜色或命名推断
线段类型。

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
