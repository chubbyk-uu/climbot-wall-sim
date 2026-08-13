# climbot_description

机器人和墙面工作系的共享描述包，供仿真、规划、未来控制器和实机部署使用。

## 内容

- `config/robot.yaml`：机器人几何、质量、轮系、驱动限幅和 footprint；
- `config/wall.yaml`：`world → wall` 位姿及墙面宽高；
- `urdf/climbot.urdf.xacro`：`robot_state_publisher` 使用的 URDF；
- `climbot_description/geometry.py`：四元数和角度通用函数；
- `climbot_description/wall_frame.py`：世界坐标与墙面工作坐标转换。

## 边界

该包只保存仿真与实机都成立的描述，不包含吸附施力、摩擦、WheelSlip、出生
位姿或仿真传感器噪声。共享物理量不得在下游包中复制一份长期配置。

## 测试

```bash
colcon test --packages-select climbot_description
colcon test-result --verbose
```

整体依赖和配置归属见 [系统架构](../../docs/ARCHITECTURE.md)。
