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

墙体尺寸位姿和机器人几何分别由 `climbot_description/config/wall.yaml` 和 `climbot_description/config/robot.yaml` 单一描述，供仿真、规划、控制和实机共同使用。Gazebo 专有的摩擦、WheelSlip、吸附、出生位姿及仿真传感器参数位于 `climbot_gazebo/config/simulation.yaml`。

渲染后端由 `gpu_backend` 参数选择，默认 `auto`：检测到 WSL 时设置 `GALLIUM_DRIVER=d3d12` 并选择 NVIDIA 适配器，让 Gazebo OGRE2 通过 Mesa D3D12 使用 GPU；原生 Linux 上不设置这些变量。可用 `gpu_backend:=native` 或 `wsl_d3d12` 强制指定。

## 阶段 4：传感器与定位融合

启动仿真后，ROS 2 可获得以下定位链路：

- `/model/climbot/ground_truth`：Gazebo 物理真值，仅用于记录与评估；
- `/model/climbot/odometry`：原始轮式里程计，仅用于诊断；
- `/wheel_odom`：协方差适配后的爬壁轮式里程计，EKF 仅融合其前向速度和绕墙面法向的角速度。默认一倍标准差为 0.03 m/s、0.05 rad/s，显式表达轮墙滑移的不确定性；
- `/imu`：100 Hz 原始 Gazebo IMU，仅用于诊断；
- `/imu_wall`：EKF 使用的 IMU 姿态观测，默认附加 0.5° 一倍标准差姿态噪声，并填充对应协方差；
- `/total_station/pose`：从真值派生的模拟全站仪位置，默认 **12 Hz**、5 mm 一倍标准差噪声和 50 ms 固定延迟；
- `/odometry/filtered`：`robot_localization/ekf_node` 的融合输出和 `odom -> base_link` TF。

融合坐标系 `odom` 固定在墙面上：`+X` 沿墙水平，`+Y` 向上，`+Z` 为离墙法向，由 `climbot_description/config/wall.yaml` 定义。全站仪适配节点按该描述把 Gazebo 世界坐标转换过来；Gazebo 真值话题仍保持原始世界坐标，便于独立评估。

TF 树为 `world → wall → odom → base_link → {imu_link, 两个主动轮, 后球轮}`。**`base_link` 的原点是两个主动轮轴的中点**（见 guide §4.3）：差分驱动绕该点旋转，原地转向不会在运动学上移动它，轮式里程计推算的也正是该点。

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

当前参数（横向 `0.12`、纵向 `0.04`、质心 `base_link` z `0.085`）的三重复标定基线：

| 指标 | 数值 |
| --- | ---: |
| 静止 30 秒位移 | `0.0000 m` |
| 水平下降/前进比（3 次） | `7.03%` / `9.30%` / `9.71%`，均值 `8.68%` |
| 上行速度 | `0.14087 m/s` |
| 下行速度 | `0.15198 m/s` |
| 下行快于上行 | `7.89%` |

改变质量、吸附力或 WheelSlip 参数后必须重新标定。轨迹数据 `results/wall_slip_trajectory.csv`，可视化 `results/wall_slip.png`，由以下命令生成：

```bash
ros2 run climbot_gazebo calibrate_wall_slip.py --ros-args \
  -p trajectory_csv:=results/wall_slip_trajectory.csv
ros2 run climbot_gazebo plot_wall_slip.py results/wall_slip_trajectory.csv
```

**关于上下行速度差的机理。** 质心高度由 `0.15` 降至 `0.085` 使主动轮法向载荷大幅变化（上行 `43.9 → 55.5 N`，下行 `102.7 → 91.2 N`），但上下行速度差几乎不变（`7.81% → 7.89%`）。因此该速度差**不是**由法向载荷转移造成的。原因是 WheelSlip 按**配置的** `wheel_normal_force`（`77 N`）而非实际接触载荷来缩放滑移柔度，纵向蠕滑只随切向力变化，而上下行所需切向力等于重力沿行进方向的分量，与质心高度无关。

这意味着 `77 N` 与实际载荷（全工况 `40.3 ~ 105.0 N`）的偏差是**模型保真度**问题而非当前数值的误差：仿真的滑移不会随法向载荷变化，而真实爬壁机器人在轮子卸载时滑移会加剧。

**重复性待改进。** 三次水平试验的下降比跨度 `7.03% ~ 9.71%`（相对均值 ±19%），第 1 次系统性偏低。guide §12 要求测试可重复，当前离散度使 `8.68%` 作为回归基线偏弱。

## 阶段 5：覆盖路径规划与 RViz

一条命令启动墙面仿真、覆盖规划器和 RViz2：

```bash
ros2 launch climbot_coverage coverage_sim.launch.py
```

RViz 显示半透明墙面、原始区域（橙色）、内缩有效区域（绿色）、弓字路径与方向箭头（蓝色）。默认矩形通过 YAML 中的左下角 A 和右上角 B 定义。等腰梯形增加右下角 C，左上角 D 关于底边中心自动镜像；A、C 高度存在偏差时自动取平均值作为修正底边。

使用等腰梯形和纵向扫描：

```bash
ros2 launch climbot_coverage coverage_sim.launch.py \
  config_file:=$(ros2 pkg prefix climbot_coverage)/share/climbot_coverage/config/coverage_trapezoid.yaml \
  region_type:=trapezoid sweep_direction:=vertical
```

使用 RViz 鼠标点选时，先选择工具栏的 `Publish Point`：矩形依次点击 A（左下）、B（右上）；等腰梯形依次点击 A（左下）、B（右上）、C（右下）。每收齐一组点会自动重新规划；重新点击会开始下一组。也可以手动清空：

```bash
ros2 launch climbot_coverage coverage_sim.launch.py \
  input_mode:=rviz region_type:=trapezoid sweep_direction:=horizontal
```

另一个终端中可手动清空已点击的点：

```bash
ros2 service call /coverage/clear_points std_srvs/srv/Trigger '{}'
```

规划器发布 `/coverage/path`、`/coverage/markers` 和 `/coverage/status`，并提供 `/coverage/replan` 服务。道间距由 `detection_width × (1 - overlap_ratio)` 得到，扫描线位于均匀覆盖带中心；机器人轮廓和墙面尺寸从 `climbot_description` 读取。所有路径段均为直线，不生成圆角。

每个 Path 位姿的航向指向下一段直线，最后一个位姿沿用到达航向。规划失败、清除点选或开始输入新区块时都会发布空 Path，防止 transient-local 订阅者继续执行旧路径。

## 测量与评估工具

全部按仿真时间计时，被 `SIGINT`/`SIGTERM` 终止时会先发布停车指令。真值只用于记录和评估，不进入控制回路。

| 脚本 | 用途 | 输出 |
| --- | --- | --- |
| `measure_normal_loads.py` | 七工况三点轮墙法向载荷（guide §15 测试 2） | `results/normal_loads_*.csv` |
| `measure_turn_slip.py` | 原地转向下滑，多转角 × 多转速（guide §15 测试 6） | `results/turn_slip.csv` |
| `calibrate_wall_slip.py` | 静止、水平侧滑、上下行速度标定 | `results/wall_slip_trajectory.csv` |
| `plot_wall_slip.py` | 离线绘图，等比例墙面轨迹 | `results/wall_slip.png` |
| `evaluate_localization.py` | 右/上/左/下四方向定位精度对比 | 日志 |

自动化运行时用 `headless:=true` 关闭 Gazebo GUI。

### 法向载荷分配

质心 `base_link` z = `0.085` 时的实测值（`results/normal_loads_after_base_link_move.csv`）：

| 工况 | 左轮 | 右轮 | 后球轮 |
| --- | ---: | ---: | ---: |
| 静止 | 45.8 N | 100.9 N | 73.3 N |
| 水平行驶 | 45.6 / 100.7 N | 100.7 / 45.6 N | 73.7 N |
| 上行 | 55.5 N | 55.3 N | 109.2 N |
| 下行 | 91.2 N | 91.2 N | 37.6 N |
| 下行制动 | 91.4 N | 91.4 N | 37.2 N（最低 35.9） |
| 原地转向 | 66.1 N | 61.2 N | 92.8 N |

最恶劣工况（下行制动）后球轮为 `16.3% F_s`，满足 guide §6.3 放宽后的 `≥ 0.15 F_s`。单个主动轮的载荷跨度是 `40.3 ~ 105.0 N`，而 WheelSlip 用的是单一标称值 `77 N`。

后球轮接触已按与物理步一致的 `1000 Hz` 重新核查。Gazebo 原生消息在覆盖四次交替方向 90° 转向的窗口内有 `29944 / 29944` 个有效接触步，在覆盖同方向连续旋转 12 秒的窗口内有 `17941 / 17941` 个有效接触步；事件式测量节点的 8 秒原地转向也得到 `7997 / 7997`、最低 `37.8 N`，接触率均为 `100%`。此前把 ROS 侧接收频率下降误判为物理脱离；Gazebo 原生接触消息证实球轮没有离墙。`measure_normal_loads.py` 现按接触消息时间戳逐条记录，并显式输出零载荷步数和接触率，避免轮询重复值再次掩盖接触状态。

### 定位精度

四方向测试（每段 8 秒、0.15 m/s）中 EKF 相对 Gazebo 真值的误差：起点 3.0 mm，右行 10.8 mm，上行 7.8 mm，左行 11.8 mm，下行 9.8 mm。同一时刻轮式里程计的航向误差最大 9.7°，体现了 guide §8.4 要求的侧滑不可观测性。

## 已知问题与未决事项

- **侧滑标定重复性偏弱。** 三次水平试验的下降比为 `7.03% / 9.30% / 9.71%`，相对均值 ±19%，且第一次系统性偏低。guide §12 要求测试可重复，当前离散度使均值作为回归基线偏弱。
- **速度指令看门狗尚未实现。** 见 guide §11。目前只有各脚本自身的终止时停车，系统级看门狗要在阶段 E 随状态机一并实现。
- **WheelSlip 对法向载荷不敏感。** 滑移柔度按配置的 `wheel_normal_force` 缩放而非实际接触载荷，属于模型保真度限制，真实爬壁机器人在轮子卸载时滑移会加剧。
