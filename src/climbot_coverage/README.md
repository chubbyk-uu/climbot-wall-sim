# climbot_coverage

C++ 墙面覆盖路径规划器，支持矩形、等腰梯形、横向/纵向弓字扫描和 RViz
点选输入，不依赖 Nav2。

## 启动

独立规划与 RViz：

```bash
ros2 launch climbot_coverage coverage_planner.launch.py
```

联合墙面仿真，只预览不执行：

```bash
ros2 launch climbot_coverage coverage_sim.launch.py
```

仿真、规划器、RViz、跟踪器和任务管理器一并启动，默认在 RViz 中点选区域：

```bash
ros2 launch climbot_coverage coverage_mission.launch.py
```

大区域参数式演示配置：

- `config/coverage_vertical_demo.yaml`：`3.30 × 4.50 m` 竖向任务；
- `config/coverage_horizontal_demo.yaml`：`4.30 × 1.70 m` 横向任务。
- `config/coverage_trapezoid_horizontal_demo.yaml`：大型等腰梯形横向任务；
- `config/coverage_trapezoid_vertical_demo.yaml`：同一梯形竖向任务。

通过 `config_file:=<配置绝对路径>` 选择配置，两个联合 launch 都接受该参数。
用 `coverage_sim.launch.py` 时还需另行启动
`climbot_control/coverage_executor.launch.py` 才能执行；`coverage_mission.launch.py`
已经包含它。完整命令见仓库根目录 [README](../../README.md)。

等腰梯形点选顺序为 A（左下）、B（右上）、C（右下）；矩形只使用 A、B。点选工具
发布的点必须在规划器的 `frame_id`（默认 `odom`，即墙面平面）中，其他坐标系的点
会被拒绝并在 `/coverage/status` 中说明原因。规划器只使用点的 `x`、`y`，忽略 `z`。

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

## 顶部收边扫描

竖向弓字任务在扫描柱末端会留下一条很薄的顶部漏扫。`top_edge_scan` 决定是否在
路径末尾追加一条水平收边 `SCAN`：

```bash
# 强制追加（验收发现顶部漏扫时用这个重新规划）
ros2 launch climbot_coverage coverage_mission.launch.py \
  sweep_direction:=vertical \
  planner_config_file:=<带 top_edge_scan: always 的配置>
```

`auto`（默认）只看**预计**覆盖率，而顶部漏扫是执行损失、名义几何里不存在，所以
`auto` 在当前两个竖向工况下不会触发——它们的预计覆盖率都是 `100%`。需要收边时请
显式设 `always`。横向扫描恒不追加：它最高一条扫描线已经压在区域顶边上。

## 测试

```bash
colcon test --packages-select climbot_coverage
colcon test-result --verbose
```

测试覆盖矩形/梯形、横纵扫描、名义覆盖率、确定性、路径航向和失败空路径。
几何单测仍按 `98%` 断言名义覆盖率：那是对规划几何质量的回归保护，
与 §14.3 的 `95%` 验收门限是两回事。
完整参数与服务见 [接口文档](../../docs/INTERFACES.md)。
