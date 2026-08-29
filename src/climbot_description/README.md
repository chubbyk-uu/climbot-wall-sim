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
- `include/climbot_description/geometry.hpp`、`src/geometry.cpp`：四元数和角度通用函数；
- `include/climbot_description/wall_frame.hpp`、`src/wall_frame.cpp`：世界坐标与墙面工作
  坐标转换。**这是唯一实现**，C++ 节点直接链接 `climbot_description_core`；
- `src/bindings.cpp`：上述实现的 pybind11 绑定，导出为 `_climbot_description`；
- `climbot_description/geometry.py`、`climbot_description/wall_frame.py`：绑定的薄封装。
  Python 侧签名不变——四元数仍是 `(x, y, z, w)` 元组、向量仍是 `(x, y, z)` 元组、返回值
  仍是元组，所以下游调用点无需改动。`quaternion_tuple()` 留在 Python，因为它适配的是 ROS
  消息；`wall_description_path()` 和 `reference_grid_spacing()` 也留在 Python，它们是启动期
  的 ament 索引查找，不在热路径上。

## 为什么下沉到 C++

定位与控制热路径上的 C++ 节点需要这个变换。此前它只有 Python 实现，C++ 侧要用就得再写一份
——而同一约定的两份实现最终一定会漂移。`test/test_wall_frame.py` 是当初为 Python 实现写的，
迁移时**一字未改**：它现在验证的是绑定，通过即等价性证据。

## 边界

该包只保存仿真与实机都成立的描述，不包含吸附施力、摩擦、WheelSlip、出生
位姿或仿真传感器噪声。共享物理量不得在下游包中复制一份长期配置。

## 测试

```bash
colcon test --packages-select climbot_description
colcon test-result --verbose
```

整体依赖和配置归属见 [系统架构](../../docs/ARCHITECTURE.md)。
