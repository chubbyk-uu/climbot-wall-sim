# Climbot Sim

基于 ROS 2 Jazzy 和 Gazebo Harmonic 的垂直壁面爬壁机器人仿真项目。

项目目标、设计约束和验收标准见 [PROJECT_GUIDE.md](PROJECT_GUIDE.md)。

## 软件基线

- ROS 2 Jazzy
- Gazebo Harmonic / gz-sim 8
- C++ 作为核心规划与控制语言
- Python 用于 launch、测试和数据分析

## 构建

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 阶段 1～3：墙面运动与侧滑

当前版本包含：

- 标准重力 `0 0 -9.81 m/s²`；
- 垂直静态墙面；
- 扁平三轮机器人：两个中部略靠前的主动轮和一个后球形轮；
- `220 N` 持续法向吸附力；
- Gazebo DiffDrive；
- 速度相关的横向 WheelSlip；
- ROS 2 `cmd_vel` 和里程计桥接。

本项目运行于 WSL2，launch 会设置 `GALLIUM_DRIVER=d3d12` 并选择 NVIDIA 适配器，使 Gazebo OGRE2 通过 Mesa D3D12 使用 GPU 渲染。

启动仿真：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch climbot_gazebo climbot_wall.launch.py
```

在第二个终端启动键盘控制：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p speed:=0.15 -p turn:=0.8
```

键位采用 `teleop_twist_keyboard` 默认布局：

```text
u  i  o
j  k  l
m  ,  .
```

- `i`：向机器人前方行驶；
- `,`：后退；
- `j` / `l`：原地转向；
- `k`：停车；
- `q` / `z`：同时增大/减小线速度和角速度；
- `Ctrl+C`：退出并发送停车指令。

机器人初始前向为墙面水平向右。持续按 `i` 水平行驶时，可以观察到机器人在真实重力作用下缓慢向下侧滑；按 `k` 停车后，静摩擦应使高度基本保持不变。

查看轮式里程计：

```bash
ros2 topic echo /model/climbot/odometry
```

当前标定条件下，机器人以 `0.15 m/s` 水平行驶时，约每行驶 `0.30 m` 下降 `0.05 m`。该侧滑由真实重力驱动，WheelSlip 只描述运动轮胎的横向蠕滑，不施加额外向下力。

当机器人转为竖直方向并保持相同的 `0.15 m/s` 指令时，重力和纵向 WheelSlip 会导致上下行实际速度不对称。分别持续行驶 `15 s`，并对中间 `9.979 s` 的 Gazebo 世界位姿做线性拟合后，上行速度为 `0.14721 m/s`，下行速度为 `0.15279 m/s`。
