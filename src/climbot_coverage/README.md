# climbot_coverage

C++ 墙面覆盖路径规划器，支持矩形、等腰梯形、横向/纵向弓字扫描和 RViz
点选输入，不依赖 Nav2。

## 启动

独立规划与 RViz：

```bash
ros2 launch climbot_coverage coverage_planner.launch.py
```

联合墙面仿真，只预览不执行（组合 launch 属于 `climbot_bringup`）：

```bash
ros2 launch climbot_bringup coverage_sim.launch.py
```

仿真、规划器、RViz、跟踪器和任务管理器一并启动，默认在 RViz 中点选区域：

```bash
ros2 launch climbot_bringup coverage_mission.launch.py
```

大区域参数式演示配置：

- `config/coverage_vertical_demo.yaml`：`3.30 × 4.50 m` 竖向任务；
- `config/coverage_horizontal_demo.yaml`：`4.30 × 1.70 m` 横向任务。
- `config/coverage_trapezoid_horizontal_demo.yaml`：大型等腰梯形横向任务；
- `config/coverage_trapezoid_vertical_demo.yaml`：同一梯形竖向任务。
- `config/coverage_p206_diagnostic_full_{horizontal,vertical}.yaml`：P2-06 诊断墙的互补
  横／竖任务；二者都使用绿框内 `0.55…9.45 × 0.55…7.45 m` 的矩形，必须成对采集。

通过 `config_file:=<配置绝对路径>` 选择配置（`coverage_mission.launch.py` 用
`planner_config_file`）。用 `coverage_sim.launch.py` 时还需另行启动
`climbot_control/coverage_executor.launch.py` 才能执行；`coverage_mission.launch.py`
已经包含它。主线命令见 [根 README 的快速启动](../../README.md#快速启动)，
批处理与实验变体见 [docs/OPERATION.md](../../docs/OPERATION.md)。

等腰梯形点选顺序为 A（左下）、B（右上）、C（右下）；矩形只使用 A、B。点选工具
发布的点必须在规划器的 `frame_id`（默认 `odom`，即墙面平面）中，其他坐标系的点
会被拒绝并在 `/coverage/status` 中说明原因。规划器只使用点的 `x`、`y`，忽略 `z`。

## 输出

- `/coverage/task`：权威 `CoverageTask` 预览，包含路径、线段类型、区域和检测足迹；
- `/coverage/path`：从任务派生、带逐段目标航向的 `nav_msgs/Path`，用于通用显示；
- `/coverage/markers`：墙面、全局安全区域（绿色虚线，启动起常驻）、用户任务可走区
  （橙色实线）、机器人路径（蓝色实线）、相机预计覆盖（黄色半透明带）和方向；
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

`detection_width`、`detection_length` 和前向偏移的标定权威来源是
`climbot_description/config/inspection_camera.yaml`；当前值为 `0.500 m`、`0.28125 m`、
`0.340 m`。横向 `overlap_ratio` 是规划任务策略，纵向 `image_overlap_ratio` 是采图策略；
两者独立配置，当前默认均为 `20%`。标准 launch 默认 `inspection_geometry_profile:=calibrated`
并注入上述物理相机几何；历史回归显式使用 `configured`，严格复现其 YAML 冻结几何。
相机外参和检测长度只用于推导黄色覆盖及覆盖率，不得把蓝色机器人端点推出橙色任务可走区。
真机仍须按实际检测载荷标定。

机器人轮廓和墙面尺寸由 `climbot_description` 注入。规划器只生成直线段和
路点处原地转向，不生成圆角或切弯。控制器可能在转后偏差较大时执行一次采集关闭的
前进小弧线，但正式 `SCAN` 仍为冻结的直线。

### 贴近硬边界的机动裕量

`motion_region` 是 base_link 的绝对硬边界。用户选区贴近它时，规划器会先按
`maneuver_boundary_margin_m` 和 `maneuver_drift_direction` 构造机动包络，再只将触碰
该包络的路径向内调整；这同时覆盖扫描末端的下滑和下一段动态补偿的上抬。它不是各向
同性缩边：当前墙面重力方向为 `[0.0, -1.0]`，因此横向轨道的左右端点不会被无谓缩短。

默认裕量为 `0.10 m`，覆盖当前标定的最大转向位移、运行时边界容差和测量余量。规划状态会
明确报告是否发生了自动内收，并始终按原始用户选区重新计算相机预计覆盖率。验收任务应设置
`minimum_nominal_coverage_ratio`，使日后的几何或标定变化导致覆盖不足时在规划阶段明确失败；
P2-06 两个全墙任务的门限为 `95%`。

## 顶部收边扫描

竖向弓字任务在扫描柱末端会留下一条很薄的顶部漏扫。`top_edge_scan` 决定是否在
路径末尾追加一条水平收边 `SCAN`：

```bash
# 强制追加（验收发现顶部漏扫时用这个重新规划）
ros2 launch climbot_bringup coverage_mission.launch.py \
  sweep_direction:=vertical \
  planner_config_file:=<带 top_edge_scan: always 的配置>
```

默认 `never`，因为顶部漏扫是执行损失、名义几何里通常不可见。`auto` 仅在同时显式
设置正 `minimum_nominal_coverage_ratio` 且预计覆盖不足时追加；需要确定补边时请设
`always`。横向扫描恒不追加：它最高一条扫描线已经压在区域顶边上。

## 测试

```bash
colcon test --packages-select climbot_coverage
colcon test-result --verbose
```

测试覆盖矩形/梯形、横纵扫描、名义覆盖率、确定性、路径航向和失败空路径。
几何单测仍按 `98%` 断言名义覆盖率：那是对规划几何质量的回归保护，
与[验收矩阵](../../docs/ACCEPTANCE.md) TT-07 的 `95%` 门限是两回事。
完整参数与服务见 [接口文档](../../docs/INTERFACES.md)。
