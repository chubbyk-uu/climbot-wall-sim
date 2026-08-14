# climbot_gazebo

垂直墙面 Gazebo 仿真、ROS/Gazebo 桥接、定位链路和实验评估工具。

## 启动

```bash
ros2 launch climbot_gazebo climbot_wall.launch.py
```

自动化测试使用：

```bash
ros2 launch climbot_gazebo climbot_wall.launch.py headless:=true
```

WSL2 默认自动选择 Mesa D3D12；也可显式设置 `gpu_backend:=wsl_d3d12`。

## 内容

- `config/simulation.yaml`：吸附、摩擦、WheelSlip、出生位姿和仿真传感器；
- `config/ekf_wall.yaml`：当前 `robot_localization` 配置；
- `models/`、`worlds/`：运行时由共享 YAML 渲染的 SDF/Xacro；
- `launch/climbot_wall.launch.py`：Gazebo、桥接、TF、适配器和 EKF；
- `scripts/total_station_sim.py`：12 Hz 带噪声和延迟的绝对位置；
- `scripts/wall_*_adapter.py`：轮式里程计和 IMU 协方差适配；
- `scripts/calibrate_wall_slip.py`：静止、水平和上下行侧滑标定；
- `scripts/measure_normal_loads.py`：三点法向载荷与接触率；
- `scripts/measure_turn_slip.py`：多角度原地转向下滑；
- `scripts/evaluate_coverage_execution.py`：执行内置紧凑任务，或用 `case:=planned_task` 执行规划器发布的完整任务，按动态直线参考统计 Gazebo 真值横轨、终点、转向结束航向、水平高度漂移、路径长度和补偿量，并计算实际二维检测足迹覆盖率；
- `scripts/evaluate_localization.py`：四方向定位误差评价。

## 侧滑标定

`calibrate_wall_slip.py` 在水平段使用 `/odometry/filtered` 保持目标航向，但不做
横轨位置纠偏；Gazebo 真值仅用于记录和评价。默认执行三次重复，并要求水平
下降/前进比的总体变异系数不超过 `horizontal_repeatability_max_cv=0.05`。
超过阈值时脚本会在保存诊断 CSV 后返回失败。正式标定必须从全新启动的仿真
开始，完整命令和当前结果见 [results/README.md](../../results/README.md)。

完整大区域覆盖评价应将 `execution_timeout_s` 从默认 `120 s` 显式提高，例如
`600 s`。该参数是评价工具等待整个 Action 的墙钟超时；超时后工具会主动取消任务，
它不是控制器的单段安全超时。

通过 `trajectory_csv` 和 `summary_json` 可选参数可分别保存完整真值/融合轨迹与评价
摘要。覆盖率默认使用 `10 mm` 栅格，只累计正式扫描直线，并要求不低于 `95%`；终点、
转向结束航向和水平扫描净高度漂移默认分别不超过 `30 mm`、`2°` 和 `30 mm`。

## 边界

机器人和墙面共享属性来自 `climbot_description`。Gazebo 真值只能用于模拟
测量和评价，不得直接进入控制器。

话题、launch 参数和 TF 见 [接口文档](../../docs/INTERFACES.md)，结果有效性见
[results/README.md](../../results/README.md)。
