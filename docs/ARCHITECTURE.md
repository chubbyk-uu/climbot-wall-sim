# 系统架构与目录职责

本文档描述当前代码结构和依赖边界。项目目标与验收标准以
[PROJECT_GUIDE.md](../PROJECT_GUIDE.md) 为准，运行接口见
[INTERFACES.md](INTERFACES.md)，实施状态见 [STATUS.md](STATUS.md)。

## 包依赖方向

```text
                      climbot_bringup
                     /       │        \
                    v        v         v
climbot_rviz_plugins   climbot_coverage  climbot_control   climbot_gazebo
                    \        │        /                    /
                     v       v       v                    v
                    climbot_interfaces        climbot_description
                             ^                       ^
                             └── climbot_inspection ─┘
```

自上而下读：`climbot_bringup` 只有组合 launch，在运行时点名下游三个包；
`climbot_interfaces` 和 `climbot_description` 是两个共享上游。

`climbot_rviz_plugins` 只依赖 `climbot_interfaces`、`std_srvs` 和 RViz/Qt，不依赖
规划或控制实现。`climbot_coverage` 运行时依赖它，是因为 `coverage.rviz` 载入该面板。

`climbot_interfaces` 是无业务实现的公共 ROS 接口包，不依赖其他项目包。
`climbot_description` 是共享物理描述的唯一上游。规划器和控制器不得依赖
`climbot_gazebo`，也不得读取 Gazebo 真值或仿真专有参数。

组合入口 `coverage_sim.launch.py` 和 `coverage_mission.launch.py` 集中在
`climbot_bringup`。它们在运行时查找 `climbot_gazebo`、`climbot_coverage` 和
`climbot_control`，这是启动编排，不是算法依赖；把它们放在 `climbot_coverage` 里
会让规划器包的依赖表读起来像规划算法依赖 Gazebo 和控制器，因此单独成包，且没有
任何包依赖 `climbot_bringup`。`climbot_gazebo` 为启动速度看门狗而运行时依赖
`climbot_control`，同样只属于仿真编排；控制包不反向依赖 Gazebo。

## 包职责

### `climbot_interfaces`（阶段 E 新建）

只包含跨包通信定义：

- `msg/CoverageTask.msg`：不可分割的名义覆盖任务；
- `msg/CoverageStatus.msg`：面向操作界面的管理器状态汇总；
- `action/ExecuteCoverage.action`：任务执行、取消、反馈和结果。

该包不得读取 YAML，不包含几何规划、控制算法、Gazebo 代码或节点实现。

### `climbot_description`

共享给仿真、规划、控制和未来实机部署的描述包：

- `config/robot.yaml`：机器人几何、质量、轮系、驱动限幅、规划轮廓，以及 G1 相机本体
  与支架的物理质量／质心／惯量；
- `config/wall.yaml`：墙面工作坐标系、可作业表面尺寸及参考网格线间距；
- `urdf/climbot.urdf.xacro`：`robot_state_publisher` 使用的机器人描述；
- `climbot_description/`：墙面坐标变换和通用几何函数。

该包不得包含摩擦、WheelSlip、吸附施力、出生位姿、仿真传感器噪声或
Gazebo 真值接口。

### `climbot_gazebo`

仿真环境、定位链路和实验工具：

- `config/simulation.yaml`：Gazebo 专有的接触、摩擦、滑移、吸附和传感器参数；
- `config/ekf_wall.yaml`：当前阶段的 `robot_localization` 配置；
- `worlds/`、`models/`：Gazebo 世界与 SDF 模型；
- `launch/climbot_wall.launch.py`：墙面仿真、桥接、TF、传感器适配和 EKF；
- `scripts/`：全站仪模拟、协方差适配、标定和评估工具；
- `climbot_gazebo/provenance.py`：所有评估工具共用的溯源采集——git 状态，以及**向
  运行中的节点问回来**的噪声源与跟踪器参数。问回来而不是复述配置，是因为传给一个
  没起来的节点的参数在摘要里看起来和用过的一模一样。

Gazebo 真值只能用于模拟传感器生成、记录和独立评价，不得反馈给规划器或
未来控制闭环。

### `climbot_inspection`（阶段 G 新建）

视觉巡检采集和图像关联：

- G1：订阅／桥接面阵相机原图和 `CameraInfo`，提供单次人工触发，并检查图像、标定
  信息和 TF 的一致性；
- G2：按冻结后的动态执行参考和 EKF 沿轨进度触发拍照，将图像与任务版本、扫描线、
  触发编号和插值融合位姿绑定；
- G3：由 30 次独立纯灰曝光计算固定 LED 平场矩阵，原图和补偿图并行发布；
- G4：把扫描任务的原始畸变图和曝光标签原子写入指定任务目录；
- 后续检测算法只消费已绑定的数据，不反向进入底盘控制闭环。

本包依赖公共接口和共享描述，不依赖 Gazebo API。Gazebo 相机传感器、渲染噪声和触发
适配留在 `climbot_gazebo`，所以同一个 `climbot_inspection` 节点可替换为真实相机输入。
正常触发逻辑不得订阅 Gazebo 真值。

当前 Gazebo 8 的 Ogre2 后端不支持 SDF `lens/distortion` 对图像像素生效。因此仿真包
用较宽视场的 triggered camera 生成理想图，再由 `camera_distortion_adapter` 按共享
`inspection_camera.yaml` 对像素施加 Brown–Conrady 畸变并生成匹配的
`CameraInfo`。这层仅用于补足渲染后端能力，不进入 `climbot_inspection`，也不改变其
真机可替换边界。

Gazebo 触发相机可能为静态场景返回像素完全相同的缓存渲染；仿真适配器因此在最终
`mono8` 输出端加入可复现的逐帧高斯读出噪声。平场标定器仍以时间戳、内容哈希和
时间标准差三重门禁确认 30 张确为独立样本，不能靠复制一张图满足样本数。

正式归档永远订阅 `image_raw`；`image_compensated` 是默认关闭的在线调试预览，不是
数据产品。离线处理仍放在本仓库，但使用计划新增的独立包保持进程、依赖和数据边界：

```text
climbot_inspection ──原图+标签目录──> climbot_image_processing ──校正图──> climbot_mosaic
       ↑                                                                  │
  在线任务与相机                                               位姿图优化、拼接与质量报告
```

`climbot_image_processing` 和 `climbot_mosaic` 只读取已封存任务目录，不向在线规划、
控制或拍照触发发布反馈。当前平场计算核心以后下沉／复用到图像处理包；现有补偿话题仅
保留为同算法的可视化验证入口。

### `climbot_coverage`

C++ 覆盖规划器和 RViz 可视化：

- `coverage_geometry`：矩形/等腰梯形修正、内缩和弓字路径几何；
- `coverage_planner_node`：参数或 RViz 点选输入、路径与状态发布、重规划服务；
- `config/`：默认矩形和等腰梯形任务；
- `launch/coverage_planner.launch.py`：独立规划入口；
- `rviz/`：墙面、区域和路径显示配置。

规划器读取 `climbot_description` 的墙面尺寸和机器人轮廓，不读取任何
Gazebo 接触参数。

### `climbot_rviz_plugins`

C++ RViz 操作面板：

- `CoveragePanel`：区域形状/扫描方向/直线控制算法三个下拉框，重新规划/清除点选/
  开始/取消·停车四个按钮，以及状态、任务版本、段进度、时间表和最近一次请求结果的
  显示；G4 在同一个 dock 中使用 `任务规划`／`巡检采集`／`详情` 页签，不新增第二个
  面板，也不增大默认 dock 宽度。

面板只订阅 `/coverage/manager_status`、调用管理器与规划器的服务、并读写执行器的
`tracking_mode` 参数，自身不保存任务状态，因此任务锁定、版本检查和安全状态转换
不会被分叉到界面里。控件的置灰一律取自被调用方发布的许可位（`can_start`、
`can_cancel`、`can_plan`），面板不另立一套状态判断，否则两边会得出不同结论。
Qt 与 pluginlib 依赖集中在本包，控制包保持无界面依赖。

G4 后面板仍只提交意图、不接触文件系统：采集开关和根目录通过管理器的带选项 Start
一次性锁定；管理器先协调 `climbot_inspection` 完成目录、空间和标定预检，再启动运动，
并把记录器状态聚合回界面。记录器独立于 RViz 生命周期，关闭或重启 RViz 不得中断
正在进行的任务归档。公共状态区与固定的停车／恢复按钮不随页签滚动或隐藏。

`tracking_mode` 直接走参数接口而不是 `/coverage/configure`：后者是规划器的服务，
把控制器的配置塞进去会让两个包的职责混在一条请求里。

### `climbot_control`

C++ 轨迹控制和速度安全：

- `line_tracker`：任意二维直线的沿轨、横轨和航向闭环及联合轮速限幅；
- `turn_profile` / `travel_profile`：原地转向与直线段的梯形/三角形时间参数化曲线，
  纯函数，两者对称；`segment_duration` 由它们推导每段耗时，用于进度权重和时间表；
- `line_tracker_node`：融合位姿输入、定位超时停车和单段参考显示；
- `cmd_vel_watchdog_node`：`/control/cmd_vel` 到 `/cmd_vel` 的唯一安全出口，
  同时提供 `/control/hold`——唯一一条不经过执行器的停止通路。其余所有停止都是
  「请求正在驱动的一方停下来」，只在它还应答时有效；保持位于轮子前的最后一跳，
  与图上其余部分处于什么状态无关。该保持是看门狗进程内的易失状态，不是硬件急停，
  实机最后边界必须由默认失效关闭的硬件级停机回路承担；
- `include/climbot_control/control_clock.hpp`：控制环和安全兜底该用哪个时钟。
  节点默认时钟在非仿真时间下退化为**可被设置、可倒退**的系统时钟，定时器建在
  它上面会在时钟回跳期间停止触发。仿真时间激活时跟节点时钟，否则用单调时钟；
  消息时间戳仍用 ROS 时间。详见 [INTERFACES.md](INTERFACES.md) 的"控制环时钟"；
- `config/control.yaml`：正常作业限幅、控制增益、超时，以及两种直线控制律的参数；
- `launch/line_tracker.launch.py`：从共享机器人描述注入轮距和轮缘硬限值。

控制包不得读取 Gazebo 真值、WheelSlip 或吸附参数。

### `climbot_bringup`

整系统启动入口，只有 launch 编排：

- `launch/coverage_sim.launch.py`：仿真加规划器加 RViz 的预览入口；
- `launch/coverage_mission.launch.py`：再加跟踪器和管理器的完整任务入口。

本包不含节点、算法和参数文件，只把各阶段包自己的单包入口组合起来；单包入口
（`climbot_wall.launch.py`、`coverage_planner.launch.py`、
`coverage_executor.launch.py`）留在各自包内，不在此重复封装。没有任何包依赖
`climbot_bringup`，因此这里点名下游包不会污染算法包的依赖表。

## 配置归属

| 配置 | 所有者 | 消费者 | 说明 |
| --- | --- | --- | --- |
| `robot.yaml` | description | Gazebo、coverage、未来 control | 真实物理属性、相机／支架惯性和保守规划轮廓 |
| `inspection_camera.yaml`（G1 新增） | description | Gazebo、inspection、未来实机 | 分辨率、内参、畸变、有效 ROI 和标称相机外参 |
| `wall.yaml` | description | Gazebo、coverage、定位、未来实机 | `world → wall` 基准、作业面尺寸和参考网格线间距 |
| `simulation.yaml` | gazebo | 仅 Gazebo | 吸附、摩擦、WheelSlip、出生位姿、仿真噪声 |
| `ekf_wall.yaml` | gazebo | `robot_localization` | 定位链路配置，随喂给它的仿真传感器留在 gazebo；不属于编排，未随 bringup 外移 |
| `coverage_*.yaml` | coverage | 覆盖规划器 | 区域输入和扫描参数 |
| `control.yaml` | control | 直线跟踪器、速度看门狗 | 作业速度、控制增益、软件限幅和超时；不复制机器人硬件属性 |

同一物理量只能有一个权威来源。launch 可以把共享 YAML 展开为节点参数或
xacro 映射，但不得在第二个文件中复制同一数值作为长期配置。

## 运行时数据流

```text
Gazebo physics
  ├─ ground truth ───────────────> 评估/模拟全站仪（禁止进入控制）
  ├─ wheel odometry -> covariance adapter ─┐
  └─ IMU -> attitude adapter ──────────────┼─> robot_localization EKF
模拟全站仪 position-only ──────────────────┘           │
                                                        v
                                                /odometry/filtered
                                                        │
Gazebo triggered camera -> image_raw + CameraInfo ──────┼─> climbot_inspection
                                                        │       ├─> G1 单次采集
冻结执行参考 + coverage status ─────────────────────────┘       └─> G2 位置触发/数据绑定

区域参数或 RViz 点选 -> coverage planner -> /coverage/task（权威任务预览）
                                              ├─> /coverage/path（派生显示）
                                              └─> coverage manager（显式开始并锁定版本）
                                                            │
                                                            └─> ExecuteCoverage Action goal
                                                            │
                                                            v
                                                     climbot_control
                                                            ├─> /control/reference_path
                                                            └─> /control/cmd_vel ─┐
键盘或单个实验脚本 ────────────────────────────────────────> /control/cmd_vel ─┤
                                                                                v
                                                                    cmd_vel_watchdog
                                                                                │
                                                                                v
                                                                            /cmd_vel
```

## 坐标系

```text
world
└── wall
    └── odom
        └── base_link
            ├── imu_link
            ├── inspection_camera_link
            │   └── inspection_camera_optical_frame
            ├── left_wheel_link
            ├── right_wheel_link
            └── caster_ball_link
```

`base_link` 位于两个主动轮轴中点在墙面接触平面上的投影。覆盖路径、EKF、
里程计和未来控制器必须使用同一参考点。

`inspection_camera_optical_frame` 的标称平移为 `[0.340, 0.000, 0.275] m`，光学
`+z` 指向墙面，光学 `+x` 对应机器人横向，光学 `+y` 对应机器人反向。该前置偏移
意味着相机投影中心不是 `base_link`；巡检覆盖和端点计算必须显式使用 TF。

## 后续包边界

`climbot_interfaces` 和 `climbot_control` 已建立。控制包最终负责 50 Hz
C++ 通用直线段跟踪、任务状态机、
线段类型执行、转向下坠补偿、左右轮联合限幅和速度看门狗。横向为主与竖向为主
的覆盖路径共用同一控制器，只由规划结果和段类型驱动。控制器保留名义覆盖路径，
并在转向后根据 EKF 实际位置冻结单独的平行直线执行参考：小偏差直接接受为平行
扫描线，较大但可恢复的偏差先执行一次前进小弧线再冻结直线。横向保留第二次转向
下滑预补偿，竖向不预补偿且不逐列倒车。

`coverage_manager_node` 已订阅 `/coverage/task` 并缓存最新有效预览，只有收到操作员
明确的 `/coverage/start` 后才复制并锁定 `task_id + revision`、发送 Action Goal。
它还提供 `/coverage/cancel`，并在 `/coverage/manager_status` 上以
`climbot_interfaces/msg/CoverageStatus` 汇总状态、任务标识、段进度和上次结果，
使界面无需自行拼装状态；失去执行器时它先进入 `STOPPING` 停机而不是直接报完成，
速度保持只作为持续施加的保护而不作为任务已经终止的证据；离开该状态只认无人继续
下达运动指令或执行器最终应答，期间停止入口一直有效。应答永远不到时，操作员可二次
确认强制放弃监督，但只会进入持续 hold、禁止启动的 `RECOVERY_LOCKED`；物理确认后另行
Rearm 才恢复任务入口；规划器不直接调用控制器，
RViz 面板也不直接实现安全状态机。执行器在首条扫描前完成采集关闭的起点进入：一条直线开到首个路点，
终点按该处转向的预计下坠抬高，与换道段共用同一套预留；首条扫描随后复用统一的动态
入轨判据。首点之外的直线不计入覆盖段。管理器留在 `climbot_control`：它是任务状态机和 Action 客户端，
不是启动编排，而 `climbot_bringup` 只放 launch。

面阵相机及位置触发采集归属于独立的 `climbot_inspection`。它消费冻结后的动态
执行参考、任务状态、EKF 位姿和相机图像，生成触发事件及带位姿的检测数据；不参与
底盘闭环，也不得使用 Gazebo 真值决定拍照。墙面纹理和仿真相机传感器属于
`climbot_gazebo`，真实/共享相机几何安装关系属于 `climbot_description`。
