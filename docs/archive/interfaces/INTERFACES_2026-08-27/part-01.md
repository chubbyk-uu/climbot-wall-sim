# 历史分册 1

# ROS 2 接口与配置索引

本文是当前 ROS 2、离线目录和配置合同的唯一字段级来源。使用步骤见
[操作手册](../../../OPERATION.md)，系统职责见 [架构](../../../ARCHITECTURE.md)，拼接设计与验收门禁见
[拼接计划](../../../MOSAIC_PLAN.md)。历史接口演变不在本文维护。

| 查找内容 | 位置 |
| --- | --- |
| launch、仿真、定位与控制话题/安全参数 | [启动入口](#启动入口)、[仿真与定位话题](#仿真与定位话题) |
| 覆盖区域、运行时构型、服务和任务状态 | [覆盖规划接口](part-03.md#覆盖规划接口) |
| `CoverageTask`、`ExecuteCoverage` 与执行版本规则 | [公共接口包](part-05.md#公共接口包) |
| 法向载荷、侧滑、转向与定位对照参数 | [物理与评价参数](part-06.md#三点法向载荷测量参数) |
| 相机、平场、G4 归档、预处理与 G2 自动采集 | [巡检采集接口](part-06.md#climbot_inspection-g1-相机接口) |
| 位姿图、hard-cut、真值评价和原尺寸巡检 CLI | [离线拼接接口](part-07.md#climbot_mosaic-离线输入预检) |
| 各 YAML/JSON 配置文件的归属 | [配置文件](part-08.md#配置文件) |

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
| `localization_profile` | `precision` | 全站仪测量模型：`precision` 保持原 12 Hz / 1 mm / 10 ms 基线；`realistic` 同时启用以下两项系统残差 |
| `prism_extrinsic_error_mode` | `auto` | `auto` 随 profile，或独立指定 `enabled` / `disabled`；实际棱镜相对 EKF 假定机器人参考点的固定残差 |
| `prism_extrinsic_error_robot_m` | `[0.020, -0.010, 0.0]` | 棱镜残差 `[前向, 左向, 墙法向]`（m）；由真值墙面 yaw 旋转后加到位置测量 |
| `measurement_timestamp_error_mode` | `auto` | `auto` 随 profile，或独立指定 `enabled` / `disabled`；仅改变观测 header 时间戳 |
| `measurement_timestamp_bias_s` | `0.020` | header 时间戳固定时钟偏差（s），不改变位置、协方差或送达时刻 |
| `measurement_timestamp_jitter_stddev_s` | `0.002` | header 时间戳零均值抖动一倍标准差（s） |
| `measurement_timestamp_jitter_seed` | `20260827` | 时间戳抖动独立随机种子；不影响位置噪声或丢包序列 |
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
还必须经过频率、噪声和延迟模型。`realistic` 中的棱镜残差由仿真传感器读取真值
yaw 后投影到墙面，因此它是可解释的传感器真值模型，控制、归档标签仍只读同一份 EKF 输出。
时间戳偏差／抖动只改 `/total_station/pose` 的 header stamp；固定 `10 ms` 送达队列仍以
原始真值采样时刻排程，二者不能互相替代。

`climbot_wall.launch.py` 始终启动 `cmd_vel_watchdog_node`，它是唯一的执行器控制
输出者：以 `50 Hz` 从
`/control/cmd_vel` 转发到 Gazebo `/cmd_vel`，在未收到首条指令、仿真时钟倒退，或
距最后一条指令超过 `command_timeout_s`（默认 `0.40 s`）时发布零速。键盘、四个实验
脚本和自动控制器均发布 `/control/cmd_vel`。当前采用单一上游来源规则；需要遥控抢占
自动任务时再增加带明确优先级的仲裁器。

