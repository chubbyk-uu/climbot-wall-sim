# 包职责

## 包职责

### `climbot_common`

无 ROS 运行时依赖的跨包工具：统一 Git provenance、SHA-256 和原子 JSON 写入，并提供大图
发布者共用的 Fast DDS profile。它不包含业务算法、ROS 接口或仿真配置；Gazebo、inspection
和 mosaic 只能向它单向依赖。

### `climbot_interfaces`

只包含跨包通信定义：

| 定义 | 用途 |
| --- | --- |
| `msg/CoverageTask.msg` | 不可分割的名义覆盖任务 |
| `msg/CoverageConfig.msg` | 面板可写的区域形状与扫描方向 |
| `msg/CoverageStatus.msg` | 面向操作界面的管理器状态汇总 |
| `msg/ExecutionReference.msg` | 转向后冻结的实际直线与采集许可 |
| `msg/InspectionCapture.msg` | 单张成功图的任务、触发与曝光位姿绑定 |
| `msg/InspectionCaptureGate.msg` | 采集侧的存活 heartbeat；v1 要求 `active=false` |
| `msg/InspectionArchiveStatus.msg` | 归档权威状态、计数、目录与错误 |
| `srv/ConfigureCoverage.srv` | 无运行任务时改区域/方向 |
| `srv/StartCoverage.srv` | 带采集选项的受控启动 |
| `srv/CaptureOnce.srv` | G1 单次人工触发及其拒绝码 |
| `srv/PrepareInspectionArchive.srv` | 原子创建 run 与预检 |
| `srv/FinalizeInspectionArchive.srv` | 封存 run 并复核计数 |
| `action/ExecuteCoverage.action` | 任务执行、取消、反馈和结果 |

该包不得读取 YAML，不包含几何规划、控制算法、Gazebo 代码或节点实现。

### `climbot_description`

共享给仿真、规划、控制和未来实机部署的描述包：

- `config/robot.yaml`：机器人几何、质量、轮系、驱动限幅、规划轮廓，以及 G1 相机本体
  与支架的物理质量／质心／惯量；
- `config/wall.yaml`：墙面工作坐标系、可作业表面尺寸及参考网格线间距；
- `config/vacuum_chassis.yaml` 与 `robot_geometry.load_robot()`：同源新底盘几何、质量／惯量与规划轮廓；
- `urdf/sensor_frames.urdf.xacro`：共享 IMU 帧；完整 URDF 由同一底盘生成器输出给 robot_state_publisher；
- `include/climbot_description/` 与 `src/`：墙面坐标变换和通用几何函数的**唯一实现**（C++）；
- `climbot_description/`：上述实现的 pybind11 绑定与薄封装。Python 侧 API 与下游调用点
  保持不变，但两种语言共用同一份实现——热路径上的 C++ 节点需要这个变换，而一份约定分裂成
  两份实现迟早会各自漂移。

该包不得包含摩擦、WheelSlip、吸附施力、出生位姿、仿真传感器噪声或
Gazebo 真值接口。

### `climbot_gazebo`

仿真环境、定位链路和实验工具：

- `config/simulation.yaml`：Gazebo 专有的接触、摩擦、滑移、灯光、IMU 安装和传感器参数；
- `config/ekf_wall.yaml`：当前阶段的 `robot_localization` 配置；
- `worlds/` 与 `models/climbot/components.sdf.xacro`：墙面及共享传感器／灯光／驱动组件；
- `vacuum_assets.py`：生成新底盘 SDF／URDF，接入唯一 VacuumSystem 施力路径；
- `launch_entry.py`／`simulation_stack.py`：默认压力模型和共享 ROS／Mesa 编排；
- `vacuum_{physics,runtime,io}.yaml`：风机／裙边／漏气、启动支撑和压力观测／故障输入；
- `launch/climbot_wall.launch.py`：墙面仿真、桥接、TF、传感器适配和 EKF；
- `scripts/`：全站仪模拟、协方差适配、标定和评估工具；
- `climbot_gazebo/provenance.py`：所有评估工具共用的溯源采集——git 状态，以及**向
  运行中的节点问回来**的噪声源与跟踪器参数。问回来而不是复述配置，是因为传给一个
  没起来的节点的参数在摘要里看起来和用过的一模一样。

Gazebo 真值只能用于模拟传感器生成、记录和独立评价，不得反馈给规划器或
未来控制闭环。

### `climbot_inspection`

视觉巡检采集和图像关联：

- G1：订阅／桥接面阵相机原图和 `CameraInfo`，提供单次人工触发，并检查图像、标定
  信息和 TF 的一致性；
- G2：按冻结后的动态执行参考和 EKF 沿轨进度触发拍照，将图像与任务版本、扫描线、
  触发编号和插值融合位姿绑定；
- G3：由 30 次独立均匀靶曝光计算固定 LED 平场矩阵；标定靶必须与被摄面具有相同光度
  响应，原图和补偿图并行发布；
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
数据产品。离线处理仍放在本仓库，但由独立包承担，以保持进程、依赖和数据边界：

```text
climbot_inspection ──原图+标签目录──> climbot_image_processing ──校正图──> climbot_mosaic
       ↑                                                                  │
  在线任务与相机                                               位姿图优化、拼接与质量报告
```

`climbot_image_processing` 和 `climbot_mosaic` 只读取已封存任务目录，不向在线规划、
控制或拍照触发发布反馈；两者都没有 ROS 图依赖。`climbot_mosaic` 仅依赖无 ROS 运行时的
`climbot_common`，`climbot_image_processing` 不依赖其他项目包。
在线补偿话题保留为同算法的可视化验证入口，不是数据产品。

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

- `CoveragePanel`：区域形状/扫描方向/直线控制算法三个下拉框，重新规划/清除点选及
  Start/Pause/Resume/Stop 控制，以及状态、任务版本、段进度、时间表和最近一次请求结果的
  显示；G4 在同一个 dock 中使用 `任务规划`／`巡检采集`／`详情` 页签，不新增第二个
  面板，也不增大默认 dock 宽度。

面板只订阅 `/coverage/manager_status`、调用管理器与规划器的服务、并读写执行器的
`tracking_mode` 参数，自身不保存任务状态，因此任务锁定、版本检查和安全状态转换
不会被分叉到界面里。任务按钮的置灰取自管理器发布的 `can_start/can_pause/can_resume/
can_cancel/can_force_abandon/can_rearm`，规划按钮另取规划器的 `can_plan`，面板不另立一套
业务许可判断，否则两边会得出不同结论。
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
  “请求正在驱动的一方停下来”，只在它还应答时有效；保持位于轮子前的最后一跳，
  与图上其余部分处于什么状态无关。该保持是看门狗进程内的易失状态，不是硬件急停，
  实机最后边界必须由默认失效关闭的硬件级停机回路承担；
- `include/climbot_control/control_clock.hpp`：控制环和安全兜底该用哪个时钟。
  节点默认时钟在非仿真时间下退化为**可被设置、可倒退**的系统时钟，定时器建在
  它上面会在时钟回跳期间停止触发。仿真时间激活时跟节点时钟，否则用单调时钟；
  消息时间戳仍用 ROS 时间。控制接口边界见 [INTERFACES.md](../INTERFACES.md)；
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

