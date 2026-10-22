# 历史分册 2

### 任务级原始图像归档

默认启动的完整 `coverage_mission` 开启采集。规划完成后在 `Capture` 页确认
**Capture raw images** 和记录器端根目录（默认读取 `CLIMBOT_DATA_ROOT`），再按 Start。该服务
只表示“归档准备请求已受理”：管理器先让记录器检查目录、标定和可写性，成功后才向
执行器发送运动 Goal。运行目录、计数和故障以公共区／Capture 页中的任务状态为准。

Start 后开关和根目录冻结，直到任务完成、取消或归档失败后封存。取消准备阶段不会让
机器人运动；记录器若迟到返回，管理器仍会将那次独立归档取消，不能把任务重新变为
可执行。运行中归档失败会请求受控停车。归档只保存畸变 `mono8` 原图、同曝光标签和
标定／任务 manifest；补偿图只用于预览。目录结构和离线处理边界见
[接口规范](../../../INTERFACES.md)。

若 State 显示 `Stopping` 且管理器一直等待未知的 Start 应答，默认保持这个状态最安全。
只有已经从机器人外部确认停车条件时才使用 Force abandon：第一次点击只显示风险，
5 秒内第二次点击才进入 `Recovery locked`。这一步仍不开放 Start；完成硬件/执行器确认
后再点击 Rearm。不要把这两个按钮连成自动脚本，ROS hold 不能替代实机硬件急停。

几个容易踩的点：

- 取消或跑完之后 **Start 仍然可用**，再按一次就重跑同一个任务；
- 点选模式下**没选够点时 Replan 会被拒绝**，因为配置文件里的角点仍在，否则会规划
  出一块没人选过的区域；
- Planner 一行显示规划器自己的状态——规划失败和"没选区域"在管理器看来都是空任务，
  只有这一行能区分。

面板由 `climbot_rviz_plugins` 提供，已写入 `coverage.rviz`，随 launch 自动出现；
若被关掉，用 RViz 菜单 `Panels → Add New Panel → climbot_rviz_plugins/Coverage`
恢复。排版细节见
[`src/climbot_rviz_plugins/README.md`](../../../../src/climbot_rviz_plugins/README.md)。

同样的操作也可以走命令行，两者等价：

```bash
ros2 service call /coverage/start std_srvs/srv/Trigger
ros2 service call /coverage/cancel std_srvs/srv/Trigger
ros2 service call /coverage/force_abandon std_srvs/srv/Trigger
# 完成外部停车确认后才执行：
ros2 service call /coverage/rearm std_srvs/srv/Trigger
ros2 service call /coverage/clear_points std_srvs/srv/Trigger

# 只看人类可读的一行
ros2 topic echo /coverage/manager_status --field message

# 完整状态：state、task_id、revision、current_segment/total_segments、progress、
# 以及 planned_total_s / schedule_lag_s / estimated_remaining_s
ros2 topic echo /coverage/manager_status
```

### Schedule 一行怎么读

```text
Position only      total 6:30  ·  left ~4:12  ·  estimate only
Timed trajectory   total 6:30  ·  left ~4:12  ·  +0.04 s
```

- **total** 是任务开始时算定的预计总时长，全程不变，含到第一个路点的接近段；
- **left** 每周期更新，并带上当前累计滞后，所以落后时它会变长而不是匀速递减；
- 末尾一项区分两种模式：
  - `Timed trajectory` 显示**相对时间表的滞后**，正数为落后、负数为超前。跑直线时
    实时变化，转向和对准阶段停在 `0.00`（那时没有直线时间表在跑），每段开始重新
    起算。正常运行在几个百分之一秒量级。
  - `Position only` 显示 `estimate only`。这不是说这个数不准——时长模型的每段开销
    常数本来就是从位置控制的运行数据标定的——而是说**没有任何东西执行或监测它**：
    机器人打滑超出模型假设时不会有人纠正，也不会上报，只会悄悄迟到。

进度条和 Schedule 回答的是两个不同问题：进度条说**做完了多少工作量**，机器人卡住
时它就不动；Schedule 说**跟不跟得上计划**。所以两者刻意分开，没有把时间折进
进度条。

### 切换区域形状与扫描方向

面板上的 **Region** 和 **Sweep** 两个下拉框直接切换，不用重启 launch。

- **切换形状会撤掉当前预览。** 轨迹消失，需要按 **Replan** 用现有点重建，或
  **Clear points** 重新选。下拉框是在问"这些点是什么形状"，不是在下令规划——
  梯形 3 点切回矩形时它曾经悄悄按前 2 点画出一条谁也没要求过的新轨迹。
- **切换形状不丢点。** A、B 在两种形状里是同一个角，所以矩形选好 2 点切到梯形只
  等第 3 点。丢点会让下拉框上的一次误点变成不可逆操作。
- **切换扫描方向不撤预览**，它不改变区域本身，只是换个方向画同一块地。
- **没选点就切构型**只改构型，不会规划出一块没人选过的区域。
- **请求被拒时下拉框弹回**规划器实际生效的值，不会停在一个规划器从未同意的显示上。

命令行等价写法：

```bash
ros2 service call /coverage/configure climbot_interfaces/srv/ConfigureCoverage \
  "{region_type: trapezoid, sweep_direction: vertical}"

# 只改一项：留空的字段保持不变
ros2 service call /coverage/configure climbot_interfaces/srv/ConfigureCoverage \
  "{sweep_direction: horizontal}"

# 当前构型（latched，后启动的客户端也能拿到）
ros2 topic echo /coverage/config
```

启动时仍可用 launch 参数指定初值：

```bash
# 矩形 + 竖向扫描（点 2 下）
ros2 launch climbot_bringup coverage_mission.launch.py sweep_direction:=vertical

# 梯形 + 横向扫描（点 3 下）
ros2 launch climbot_bringup coverage_mission.launch.py region_type:=trapezoid
```

### 切换直线控制算法

面板上的 **Algorithm** 下拉框在两种直线段控制律之间切换：

| 选项 | `tracking_mode` | 直线段怎么走 |
| --- | --- | --- |
| **Timed trajectory**（默认） | `time` | 按时间参数化的梯形/三角形速度曲线行驶：前馈该时刻的曲线速度与加速度，反馈该时刻应到位置与实际位置之差 |
| **Position only** | `distance` | 恒定巡航速度前进，终点靠 `sqrt(2·a·剩余距离)` 距离制动收尾。控制器不知道"现在应该走到哪儿" |

两者在落点、道间距、转向落点航向和覆盖率上**没有系统性差异**——2026-08-19 在同一
提交的同一棵干净工作树上各跑了八工况，只差这一个参数，单工况互有胜负，差值都落在
运行间散布内。分得开的只有任务时长预测：

| | `act/plan` 区间 | sd | 最大偏离 | 滞后是否被测量 |
| --- | --- | ---: | ---: | --- |
| `time` | `0.988~1.020` | 0.92% | 1.96% | 是，`0.03~0.05 s` |
| `distance` | `0.968~1.016` | 1.43% | 3.18% | 否 |

`time` 是默认值，理由是后一列而不是前几列：它逐拍上报自己与计划的偏差，落后了会
自己说出来；`distance` 只能等任务结束后对账。原地转向段两种模式相同——它本来就是
时间参数化的。实测数据见 [`results/README.md`](../../../../results/README.md)「两种算法的
对照」和 [PLAN_2026-08-18_time_control.md](../../plans/PLAN_2026-08-18_time_control.md) 第 8 节。

**任务运行中不能切换**，执行器会拒绝并说明原因：控制律换了而时间表还是按旧律排
的，那是危险的。面板在运行期间已经把这个框置灰。

启动时指定（默认已是 `time`，这行是切回位置控制用的）：

```bash
ros2 launch climbot_bringup coverage_mission.launch.py tracking_mode:=distance
```

命令行等价（同样只在没有任务运行时接受）：

```bash
ros2 param set /line_tracker tracking_mode distance
ros2 param get /line_tracker tracking_mode
```

## 对照基线几何

想和 `results/` 中的基线对照，可直接点选基线几何：矩形取
`(0.005, 1.75)`–`(4.305, 3.45)`；梯形取 A `(-0.6, 1.4)`、B `(2.7, 4.2)`、
C `(3.4, 1.4)`，即底边 `4.00 m`、上底 `2.60 m`、高 `2.80 m`。梯形横向约
`232 s`、13 段，梯形竖向约 `284 s`、19 段，竖向工况接近五分钟，不是卡住。

跳过点选、直接用配置里的角点启动同一条链：

```bash
ros2 launch climbot_bringup coverage_mission.launch.py \
  input_mode:=parameters region_type:=trapezoid sweep_direction:=vertical \
  planner_config_file:="$(pwd)/src/climbot_coverage/config/coverage_trapezoid_vertical_demo.yaml"
```

该 launch 的规划器与控制器参数文件分别叫 `planner_config_file` 和
`control_config_file`，不能都写成 `config_file`：被包含的 launch 会继承父作用域的
同名参数，一个 `config_file` 会同时落到两个节点上，使跟踪器退回内置默认值。

## 参数式完整演示

仓库提供矩形和等腰梯形参数式演示：

- `coverage_vertical_demo.yaml`：`3.30 × 4.50 m`，8 条竖向扫描线；
- `coverage_horizontal_demo.yaml`：`4.30 × 1.70 m`，4 条横向扫描线；
- `coverage_trapezoid_horizontal_demo.yaml`：底边 `4.00 m`、上底 `2.60 m`、高 `2.80 m`，横向扫描；
- `coverage_trapezoid_vertical_demo.yaml`：同一梯形，竖向扫描。

以下示例运行横向长扁矩形。终端 1 启动仿真、规划器和 RViz：

```bash
ros2 launch climbot_bringup coverage_sim.launch.py \
  config_file:="$(pwd)/src/climbot_coverage/config/coverage_horizontal_demo.yaml" \
  input_mode:=parameters region_type:=rectangle sweep_direction:=horizontal
```

终端 2 启动覆盖执行器：

```bash
ros2 launch climbot_control coverage_executor.launch.py use_sim_time:=true
```

终端 3 将规划器发布的任务发送给 Action，并用 Gazebo 真值评价轨迹：

```bash
ros2 run climbot_gazebo evaluate_coverage_execution.py --ros-args \
  -p use_sim_time:=true -p case:=planned_task \
  -p startup_timeout_s:=20.0 -p execution_timeout_s:=600.0 \
  -p trajectory_csv:=results/coverage_trajectory.csv.gz \
  -p summary_json:=results/coverage_summary.json
```

竖向演示只需在终端 1 改用 `coverage_vertical_demo.yaml`，并把
`sweep_direction` 改为 `vertical`。评价工具默认的 `120 s` 是整任务等待时间，
大区域演示必须显式提高；它与控制器每段的安全超时不是同一个参数。

## 批量回归

八个验收工况并行跑一轮，每条 lane 用独立的 `ROS_DOMAIN_ID` 和 `GZ_PARTITION`
隔离：

```bash
tools/run_coverage_regression.sh -j 4 -t <tag>              # 默认时间点控制
tools/run_coverage_regression.sh -j 4 -t <tag> -m distance  # 位置控制
```

结果写进 `results/coverage_<case>_<tag>_summary.json` 和同名轨迹 CSV，汇总表附在
命令输出末尾。用途和有效性见 [results/README.md](../../../../results/README.md)。

**跑正式基线前先把 `src/` 提交干净。** 脚本开跑前检查工作树，非干净时自动给标签加
`-dirty` 后缀并打印醒目告警。带 `-dirty` 的结果只能当过程记录。

### 工况有两张表

`CASES` 是八个覆盖工况，经规划器分解区域。`LINE_CASES` 是单段直线和起点进入工况，
**不起规划器**——评价器自己发布两路点任务，被测的是跟踪器在一条线上的表现，而不是
区域分解：

```bash
tools/run_coverage_regression.sh -t <tag> -j 5 \
    line_horizontal line_horizontal_back line_vertical \
    line_diagonal line_diagonal_back                       # §15.7 单段直线
tools/run_coverage_regression.sh -t <tag> -j 4 \
    entry_near entry_mid entry_far entry_side entry_behind \
    entry_vertical_side entry_diagonal                     # 阶段 E 第 8 项起点进入
tools/run_coverage_regression.sh -t <tag> -j 1 \
    -o "turn_slip_per_degree_m=0.0" g1_cross               # G-1 初始横轨误差
```

