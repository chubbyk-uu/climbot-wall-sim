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

阶段 1～3 的启动和操作说明将在仿真模型完成后补充。

