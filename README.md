# Climbot Sim

基于 ROS 2 Jazzy 和 Gazebo Harmonic 的垂直壁面爬壁机器人仿真项目。机器人
保留真实重力，通过持续法向吸附力贴墙，以前置差分主动轮和后球形随动轮运动，
并模拟运动时受重力影响的侧滑。

当前已经完成物理仿真、定位融合、覆盖路径规划、多段覆盖 Action、自定义直线跟踪、
转向下坠处理和速度安全链，不使用 Nav2。横向与竖向弓字任务共用控制器：转后小
偏差冻结为平行扫描线，较大偏差先执行一次前进小弧线，正式扫描段始终保持直线。

## 文档导航

| 文档 | 用途 |
| --- | --- |
| [PROJECT_GUIDE.md](PROJECT_GUIDE.md) | 项目目标、设计约束、实施阶段和最终验收标准 |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | 包职责、依赖方向、配置归属和运行时数据流 |
| [docs/INTERFACES.md](docs/INTERFACES.md) | launch、话题、服务、参数和 TF |
| [docs/STATUS.md](docs/STATUS.md) | 阶段完成度、review 闭环和下一步 |
| [results/README.md](results/README.md) | 实验结果用途、有效性和重新生成方法 |

## 软件基线

- ROS 2 Jazzy；
- Gazebo Harmonic / gz-sim 8；
- `robot_localization`；
- C++17 用于核心规划和控制；
- Python 用于 launch、传感器适配、实验和数据分析。

## 工作区结构

```text
climbot_sim/
├── PROJECT_GUIDE.md             总指导规范
├── README.md                    项目入口与快速启动
├── docs/                        架构、接口和状态
├── results/                     可追溯实验输出
└── src/
    ├── climbot_description/     共享机器人与墙面描述
    ├── climbot_gazebo/          仿真、定位与评估工具
    ├── climbot_interfaces/      覆盖任务消息与执行 Action
    ├── climbot_coverage/        C++ 覆盖规划与 RViz
    ├── climbot_rviz_plugins/    RViz 操作面板
    └── climbot_control/         C++ 轨迹跟踪和速度安全
```

依赖方向为：

```text
climbot_interfaces
      ^        ^
      │        │
climbot_coverage  climbot_control（多段覆盖 Action 与直线跟踪）
      │        │
      └──> climbot_description <── climbot_gazebo
```

规划器和控制器都不读取 Gazebo 真值或仿真专有参数；Gazebo 包仅因仿真组合 launch
依赖控制包。

## 环境安装

本仓库当前在以下环境上验证通过：

| 组件 | 版本 |
| --- | --- |
| 操作系统 | Ubuntu 24.04.4 LTS（WSL2，内核 `6.18` microsoft-standard） |
| ROS 2 | Jazzy Jalisco |
| Gazebo | Harmonic / gz-sim `8.11.0` |
| Python | `3.12.3` |
| 构建工具 | `colcon`（`python3-colcon-common-extensions`） |

### 1. 添加 ROS 2 apt 源并安装 Jazzy

apt 源的添加方式以
[官方安装文档](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
为准（本机使用的是 `ros2-apt-source` deb 方式）：

```bash
sudo apt update && sudo apt install -y curl
export ROS_APT_SOURCE_VERSION=$(curl -s \
  https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo apt install -y /tmp/ros2-apt-source.deb
sudo apt update
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep
```

需要 `ros-jazzy-desktop` 而不是 `ros-base`，因为点选和预览依赖 RViz2。

### 2. 安装本仓库声明的依赖

**不需要单独添加 `packages.osrfoundation.org` 源。** Gazebo Harmonic 由
`ros-jazzy-gz-sim-vendor` 和 `ros-jazzy-gz-tools-vendor` 随
`ros-jazzy-ros-gz` 一并从 packages.ros.org 装入，`gz` 命令在 source 之后可用。

```bash
sudo rosdep init      # 只需一次，已初始化过会报错，可忽略
rosdep update

cd ~/robot_ws/climbot_sim
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
```

该命令会按各包 `package.xml` 装齐 `ros_gz`、`robot_localization`、
`teleop_twist_keyboard`、`rviz2`、`xacro`、`robot_state_publisher`、
`python3-yaml` 和 `python3-matplotlib`。想先看会装什么，把 `-y` 换成
`--simulate`。

### 3. 验证

```bash
source /opt/ros/jazzy/setup.bash
gz sim --versions        # 应输出 8.x
ros2 doctor --report | head -20
```

## 构建与测试

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

运行全部测试：

```bash
colcon test
colcon test-result --verbose
```

## 快速启动

启动墙面仿真、传感器和 EKF：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch climbot_gazebo climbot_wall.launch.py
```

WSL2 默认自动使用 Mesa D3D12 GPU 后端。如果自动检测不符合当前环境，可指定：

```bash
ros2 launch climbot_gazebo climbot_wall.launch.py gpu_backend:=wsl_d3d12
```

键盘控制在另一个终端运行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -r /cmd_vel:=/control/cmd_vel \
  -p speed:=0.15 -p turn:=0.8
```

机器人初始朝墙面水平方向。水平行驶时应在重力作用下逐渐下降，停车后由静摩擦
基本保持高度。

启动仿真、覆盖规划器和 RViz（只能预览，不能执行）：

```bash
ros2 launch climbot_coverage coverage_sim.launch.py
```

使用独立规划器或等腰梯形的命令见
[climbot_coverage/README.md](src/climbot_coverage/README.md)。

### 在 RViz 中点选区域并执行

一条命令启动仿真、规划器、RViz、跟踪器和任务管理器：

```bash
ros2 launch climbot_coverage coverage_mission.launch.py
```

在 RViz 工具栏选择 `Publish Point`，按下表顺序点击区域角点。每次点击
`/coverage/status` 都会回显它认成了哪个角和坐标，用来确认没有因为相机视角
而选反方向：

| 区域 | 点击顺序 |
| --- | --- |
| `rectangle`（默认） | A 左下 → B 右上 |
| `trapezoid` | A 左下 → **B 右上** → C 右下 |

梯形第二下是右上、第三下才是右下，容易顺手点成逆时针。手点两个底角不可能等高，
规划器会取均值，差超过 `bottom_warning_tolerance`（默认 `50 mm`）时状态里会多一句
`Bottom clicks differed by ... and were corrected to their mean height.`，这是提示
不是错误。

最后一次点击后 RViz 中出现规划路径，左侧 **Coverage Task** 面板的 State 变为
`Ready` 并显示任务名与 revision。确认路径正确后直接点面板上的按钮：

| 按钮 | 作用 |
| --- | --- |
| **Start** | 开始执行，State 转为 `Executing`，进度条和 Segment 随之更新 |
| **Cancel / Stop** | 中途停车 |
| **Clear points** | 点错角点时清空重选 |
| **Replan** | 用当前角点重新规划 |

Start 和 Cancel 的置灰由管理器发布的 `can_start` / `can_cancel` 决定，不是面板
自己推断的；Replan 和 Clear points 始终可用，它们只改预览，不影响正在执行的任务。
无论面板显示什么，非法请求都由管理器拒绝，原因显示在 Last request 一行。

几个容易踩的点：

- 取消或跑完之后 **Start 仍然可用**，再按一次就重跑同一个任务；
- 执行中重新规划或清除点选**不会影响正在跑的任务**，只更新下一次要跑的预览，
  面板的 State 和任务号仍然指向正在执行的那个；
- 点选模式下**没选够点时 Replan 会被拒绝**，因为配置文件里的角点仍在，否则会规划出
  一块没人选过的区域；
- Planner 一行显示规划器自己的状态——规划失败和"没选区域"在管理器看来都是空任务，
  只有这一行能区分。

面板由 `climbot_rviz_plugins` 提供，已写入 `coverage.rviz`，随 launch 自动出现；
若被关掉，用 RViz 菜单 `Panels → Add New Panel → climbot_rviz_plugins/Coverage`
恢复。面板可以随 dock 拖窄到约 240 px：长消息整行换行，任务号在下划线处折行，
正文放不下时滚动，按钮固定在底部不参与滚动。更窄 RViz 会拒绝，因为再窄按钮上的
字就要被裁掉了。排版细节见
[`src/climbot_rviz_plugins/README.md`](src/climbot_rviz_plugins/README.md)。

同样的操作也可以走命令行，两者等价：

```bash
ros2 service call /coverage/start std_srvs/srv/Trigger
ros2 service call /coverage/cancel std_srvs/srv/Trigger
ros2 service call /coverage/clear_points std_srvs/srv/Trigger

# 只看人类可读的一行，等价于原来的 std_msgs/String
ros2 topic echo /coverage/manager_status --field message

# 看完整状态：state、task_id、revision、current_segment/total_segments、progress
ros2 topic echo /coverage/manager_status
```

#### 切换区域形状与扫描方向

`region_type` 和 `sweep_direction` 只在规划器启动时读取一次，`ros2 param set`
改了不生效，换构型必须重启 launch：

```bash
# 矩形 + 竖向扫描（点 2 下）
ros2 launch climbot_coverage coverage_mission.launch.py sweep_direction:=vertical

# 梯形 + 横向扫描（点 3 下）
ros2 launch climbot_coverage coverage_mission.launch.py region_type:=trapezoid

# 梯形 + 竖向扫描（点 3 下）
ros2 launch climbot_coverage coverage_mission.launch.py \
  region_type:=trapezoid sweep_direction:=vertical
```

想和 `results/` 中的基线对照，可直接点选基线几何：矩形取
`(0.005, 1.75)`–`(4.305, 3.45)`；梯形取 A `(-0.6, 1.4)`、B `(2.7, 4.2)`、
C `(3.4, 1.4)`，即底边 `4.00 m`、上底 `2.60 m`、高 `2.80 m`。梯形横向约
`232 s`、13 段，梯形竖向约 `284 s`、19 段，竖向工况接近五分钟，不是卡住。

跳过点选、直接用配置里的角点启动同一条链：

```bash
ros2 launch climbot_coverage coverage_mission.launch.py \
  input_mode:=parameters region_type:=trapezoid sweep_direction:=vertical \
  planner_config_file:="$(pwd)/src/climbot_coverage/config/coverage_trapezoid_vertical_demo.yaml"
```

该 launch 的规划器与控制器参数文件分别叫 `planner_config_file` 和
`control_config_file`，不能都写成 `config_file`：被包含的 launch 会继承父作用域的
同名参数，一个 `config_file` 会同时落到两个节点上，使跟踪器退回内置默认值。

### 完整覆盖任务演示

仓库提供矩形和等腰梯形参数式演示：

- `coverage_vertical_demo.yaml`：`3.30 × 4.50 m`，8 条竖向扫描线；
- `coverage_horizontal_demo.yaml`：`4.30 × 1.70 m`，4 条横向扫描线。
- `coverage_trapezoid_horizontal_demo.yaml`：底边 `4.00 m`、上底 `2.60 m`、高 `2.80 m`，横向扫描；
- `coverage_trapezoid_vertical_demo.yaml`：同一梯形，竖向扫描。

以下示例运行横向长扁矩形。终端 1 启动仿真、规划器和 RViz：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch climbot_coverage coverage_sim.launch.py \
  config_file:="$(pwd)/src/climbot_coverage/config/coverage_horizontal_demo.yaml" \
  input_mode:=parameters region_type:=rectangle sweep_direction:=horizontal
```

终端 2 启动覆盖执行器：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch climbot_control coverage_executor.launch.py use_sim_time:=true
```

终端 3 将规划器发布的任务发送给 Action，并用 Gazebo 真值评价轨迹：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run climbot_gazebo evaluate_coverage_execution.py --ros-args \
  -p use_sim_time:=true -p case:=planned_task \
  -p startup_timeout_s:=20.0 -p execution_timeout_s:=600.0 \
  -p trajectory_csv:=results/coverage_trajectory.csv.gz \
  -p summary_json:=results/coverage_summary.json
```

竖向演示只需在终端 1 改用 `coverage_vertical_demo.yaml`，并把
`sweep_direction` 改为 `vertical`。评价工具默认的 `120 s` 是整任务等待时间，
大区域演示必须显式提高；它与控制器每段的安全超时不是同一个参数。

## 当前状态

- 阶段 A：基础物理与机器人模型——完成；
- 阶段 B：运动侧滑——完成；
- 阶段 C：传感器与定位融合——完成；
- 阶段 D：覆盖路径规划——完成；
- 阶段 E：自定义轨迹跟踪——进行中（多段 Action、动态换道、转后平行扫描和小弧线入轨已完成，最终系统评价待完成）；
- 阶段 F：系统测试与数据评价——进行中（横/竖向大型矩形与横/竖向等腰梯形四份覆盖基线、§14.4 侧滑补偿专项验收已完成，均在同一提交的干净工作树上产出；单段直线、定位噪声和固定种子重复性三项测试场景待补）。

详细证据和待办见 [docs/STATUS.md](docs/STATUS.md)。

## 安全提示

Gazebo DiffDrive 会持续执行最后收到的 `/cmd_vel`，因此仿真 launch 始终启动速度
看门狗，并由它作为 `/cmd_vel` 的唯一发布者。键盘、实验脚本和自动控制统一发布到
`/control/cmd_vel`；当前一次只能运行一个上游控制源，不要同时启动键盘和自动任务。

控制环和看门狗的定时器**不使用节点默认时钟**。节点默认时钟在非仿真时间下就是
系统时钟，可以被设置、可以往回跳（WSL2 每约 30 s 回跳 1～2 s），建在它上面的
定时器在回跳期间不触发——控制器整段不发指令，而机器人还在按最后一条指令走。
仿真时间激活时跟 `/clock`，否则用单调时钟，见
[docs/INTERFACES.md](docs/INTERFACES.md) 的"控制环时钟"。
