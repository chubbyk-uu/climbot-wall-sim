# climbot_coverage

C++ 墙面覆盖路径规划器，支持矩形、等腰梯形、横向/纵向弓字扫描和 RViz
点选输入，不依赖 Nav2。

## 启动

独立规划与 RViz：

```bash
ros2 launch climbot_coverage coverage_planner.launch.py
```

联合墙面仿真：

```bash
ros2 launch climbot_coverage coverage_sim.launch.py
```

大区域参数式演示配置：

- `config/coverage_vertical_demo.yaml`：`3.30 × 4.50 m` 竖向任务；
- `config/coverage_horizontal_demo.yaml`：`4.30 × 1.70 m` 横向任务。

通过 `coverage_sim.launch.py config_file:=<配置绝对路径>` 选择配置。完整自动执行还需
启动 `climbot_control/coverage_executor.launch.py`，再向 `/coverage/execute` 发送
规划器发布的任务；完整命令见仓库根目录 [README](../../README.md)。

等腰梯形点选顺序为 A（左下）、B（右上）、C（右下）；矩形只使用 A、B。

## 输出

- `/coverage/task`：权威 `CoverageTask` 预览，包含路径、线段类型、区域和检测足迹；
- `/coverage/path`：从任务派生、带逐段目标航向的 `nav_msgs/Path`，用于通用显示；
- `/coverage/markers`：墙面、原始/有效区域、路径和方向；
- `/coverage/status`：规划状态和错误原因；
- `/coverage/clear_points`：清空 RViz 点选；
- `/coverage/replan`：使用当前输入重新规划。

规划失败、清空点选或开始输入新区域时会同时发布空 Task 和空 Path，避免下游执行
旧任务。每次发布的 Task revision 会递增；正在执行的任务由控制 Action 冻结，
不会被此预览话题热切换。

## 几何约束

```text
row_spacing = detection_width × (1 - overlap_ratio)
safety_margin = 0.5 × hypot(robot_length, robot_width) + edge_clearance
```

`detection_length` 是沿行进方向的检测有效长度，当前默认 `0.01 m` 为保守临时值，
后续必须按实际检测载荷标定。

机器人轮廓和墙面尺寸由 `climbot_description` 注入。规划器只生成直线段和
路点处原地转向，不生成圆角或切弯。控制器可能在转后偏差较大时执行一次采集关闭的
前进小弧线，但正式 `SCAN` 仍为冻结的直线。

## 测试

```bash
colcon test --packages-select climbot_coverage
colcon test-result --verbose
```

测试覆盖矩形/梯形、横纵扫描、≥98% 覆盖率、确定性、路径航向和失败空路径。
完整参数与服务见 [接口文档](../../docs/INTERFACES.md)。
