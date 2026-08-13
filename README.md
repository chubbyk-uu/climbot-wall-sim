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

## 阶段 4：传感器与定位融合

启动仿真后，ROS 2 可获得以下定位链路：

- `/model/climbot/ground_truth`：Gazebo 物理真值，仅用于记录与评估；
- `/model/climbot/odometry`：原始轮式里程计，仅用于诊断；
- `/wheel_odom`：协方差适配后的爬壁轮式里程计，EKF 仅融合其前向速度和绕墙面法向的角速度。默认一倍标准差为 0.03 m/s、0.05 rad/s，显式表达轮墙滑移的不确定性；
- `/imu`：100 Hz 原始 Gazebo IMU，仅用于诊断；
- `/imu_wall`：EKF 使用的 IMU 姿态观测，默认附加 0.5° 一倍标准差姿态噪声，并填充对应协方差；
- `/total_station/pose`：从真值派生的模拟全站仪位置，默认 **12 Hz**、5 mm 一倍标准差噪声和 50 ms 固定延迟；
- `/odometry/filtered`：`robot_localization/ekf_node` 的融合输出和 `odom -> base_link` TF。

融合坐标系 `odom` 固定在墙面上：`+X` 为初始前进方向，`+Y` 向上，`+Z` 为离墙法向。全站仪适配节点会将 Gazebo 世界坐标转换到该坐标系；Gazebo 真值话题仍保持原始世界坐标，便于独立评估。

全站仪参数均可在启动时修改，例如：

```bash
ros2 launch climbot_gazebo climbot_wall.launch.py \
  total_station_rate_hz:=12.0 total_station_stddev_m:=0.005 \
  total_station_delay_s:=0.05
```

全站仪只发布绝对位置，不伪造航向或速度；其原始 Gazebo 真值不会送入 EKF 或控制器。

轮式里程计的不确定度可独立调整，例如：

```bash
ros2 launch climbot_gazebo climbot_wall.launch.py \
  wheel_forward_velocity_stddev_mps:=0.03 wheel_yaw_rate_stddev_rps:=0.05
```

IMU 姿态标准差也可配置，默认 `0.00872664626 rad`（0.5°）：

```bash
ros2 launch climbot_gazebo climbot_wall.launch.py imu_orientation_stddev_rad:=0.00872664626
```

可在仿真运行时执行以下可重复的四方向定位评估。脚本使用 EKF 融合航向闭环转到 `0°`、`90°`、`180°`、`-90°`，每段默认前进 8 秒；它会同时输出 Gazebo 实际航向、EKF 融合航向、轮式里程计航向和位置误差：

```bash
ros2 run climbot_gazebo evaluate_localization.py
```

墙面滑移标定工具会以 Gazebo 真值独立评估静止、无纠偏水平行驶，以及融合航向保持下的上/下行；真值不会进入控制回路。默认执行一次 30 秒静止测试和三组 8 秒运动测试：

```bash
ros2 run climbot_gazebo calibrate_wall_slip.py
```

可缩短试运行或增加重复次数，例如：

```bash
ros2 run climbot_gazebo calibrate_wall_slip.py --ros-args \
  -p repetitions:=3 -p static_duration_s:=30.0 -p drive_duration_s:=8.0
```

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

当前 WheelSlip 参数应以 `calibrate_wall_slip.py` 的重复实验结果为准。该侧滑由真实重力驱动，WheelSlip 只描述运动轮胎的运动蠕滑，不施加额外向下力。

当前参数（横向 `0.12`、纵向 `0.04`）的三重复标定基线为：静止 30 秒无可见下滑；水平下降/前进比均值 `8.18%`；上行 `0.14077 m/s`，下行 `0.15177 m/s`，下行快 `7.81%`。该数值用于后续回归对比，改变质量、吸附力或 WheelSlip 参数后必须重新标定。
