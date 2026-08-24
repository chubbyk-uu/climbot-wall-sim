# climbot_description

机器人和墙面工作系的共享描述包，供仿真、规划、控制器和实机部署使用。

## 内容

- `config/robot.yaml`：机器人几何、质量、轮系、驱动限幅、footprint，以及相机本体和
  支架的物理代理模型；
- `config/inspection_camera.yaml`：相机光心外参、分辨率、内参、畸变和有效检测足迹；
- `config/wall.yaml`：`world → wall` 位姿及墙面宽高。**工作系原点在墙面左下角**，
  所以作业面是 `x ∈ [0, 宽]`、`y ∈ [0, 高]`，不出现负坐标；`origin_xyz` 是该原点
  在世界系中的位置，仿真里墙仍居中于世界 `Y = 0`，因此它是 `-宽/2`。
  还有 `reference_grid.spacing_m`：参考网格线间距，`0` 表示不画。真实墙面上没有
  网格，它是绘图辅助；放在这里是因为 Gazebo 墙面和 RViz 叠加层两个视图都要画它，
  各存一份默认值迟早会变成两套不同的网格；
- `urdf/climbot.urdf.xacro`：`robot_state_publisher` 使用的 URDF；
- `climbot_description/geometry.py`：四元数和角度通用函数；
- `climbot_description/wall_frame.py`：世界坐标与墙面工作坐标转换，以及
  `reference_grid_spacing()`——四个 launch 文件读同一个网格间距的唯一入口。

## 边界

该包只保存仿真与实机都成立的描述，不包含吸附施力、摩擦、WheelSlip、出生
位姿或仿真传感器噪声。共享物理量不得在下游包中复制一份长期配置。

## 测试

```bash
colcon test --packages-select climbot_description
colcon test-result --verbose
```

整体依赖和配置归属见 [系统架构](../../docs/ARCHITECTURE.md)。
