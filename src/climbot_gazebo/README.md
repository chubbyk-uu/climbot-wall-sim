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
- `scripts/evaluate_localization.py`：四方向定位误差评价。

## 边界

机器人和墙面共享属性来自 `climbot_description`。Gazebo 真值只能用于模拟
测量和评价，不得直接进入未来控制器。

话题、launch 参数和 TF 见 [接口文档](../../docs/INTERFACES.md)，结果有效性见
[results/README.md](../../results/README.md)。
