# Climbot Sim

基于 ROS 2 Jazzy 和 Gazebo Harmonic 的垂直壁面爬壁机器人仿真项目。机器人
保留真实重力，通过持续法向吸附力贴墙，以前置差分主动轮和后球形随动轮运动，
并模拟运动时受重力影响的侧滑。

当前已经完成物理仿真、定位融合和覆盖路径规划；自定义直线轨迹跟踪尚未开始，
不使用 Nav2。

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
- C++17 用于核心规划和未来控制；
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
    └── climbot_coverage/        C++ 覆盖规划与 RViz
```

依赖方向为：

```text
climbot_description <── climbot_gazebo
        ^
        └──────────── climbot_coverage
```

规划器不读取 Gazebo 真值或仿真专有参数。

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
  -p speed:=0.15 -p turn:=0.8
```

机器人初始朝墙面水平方向。水平行驶时应在重力作用下逐渐下降，停车后由静摩擦
基本保持高度。

启动仿真、覆盖规划器和 RViz：

```bash
ros2 launch climbot_coverage coverage_sim.launch.py
```

使用独立规划器或等腰梯形/RViz 点选的命令见
[climbot_coverage/README.md](src/climbot_coverage/README.md)。

## 当前状态

- 阶段 A：基础物理与机器人模型——完成；
- 阶段 B：运动侧滑——功能完成，重复性验收待闭环；
- 阶段 C：传感器与定位融合——完成；
- 阶段 D：覆盖路径规划——完成；
- 阶段 E：自定义轨迹跟踪——未开始；
- 阶段 F：系统测试与数据评价——未开始。

详细证据和待办见 [docs/STATUS.md](docs/STATUS.md)。

## 安全提示

Gazebo DiffDrive 会持续执行最后收到的 `/cmd_vel`。在阶段 E 的系统级速度看门狗
完成前，结束键盘或实验脚本时必须确认机器人已收到零速；不要同时运行多个
任务级 `/cmd_vel` 发布者。
