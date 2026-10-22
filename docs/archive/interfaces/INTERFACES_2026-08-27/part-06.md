# 历史分册 6

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

