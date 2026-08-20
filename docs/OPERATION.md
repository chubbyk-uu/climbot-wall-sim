# 操作手册

从零启动一次覆盖任务需要知道的全部内容。包边界和数据流见
[ARCHITECTURE.md](ARCHITECTURE.md)，话题、服务和参数的完整清单见
[INTERFACES.md](INTERFACES.md)，安装与构建见
[README](../README.md)。

以下所有命令都假定已经 source 过环境：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 只看仿真

启动墙面、机器人、传感器和 EKF：

```bash
ros2 launch climbot_gazebo climbot_wall.launch.py
```

WSL2 默认自动使用 Mesa D3D12 GPU 后端。自动检测不符合当前环境时可以指定：

```bash
ros2 launch climbot_gazebo climbot_wall.launch.py gpu_backend:=wsl_d3d12
```

键盘控制在另一个终端运行：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -r /cmd_vel:=/control/cmd_vel \
  -p speed:=0.15 -p turn:=0.8
```

机器人初始朝墙面水平方向。水平行驶时应在重力作用下逐渐下降，停车后由静摩擦
基本保持高度。

只要预览规划、不需要执行：

```bash
ros2 launch climbot_bringup coverage_sim.launch.py
```

使用独立规划器或等腰梯形的命令见
[climbot_coverage/README.md](../src/climbot_coverage/README.md)。

## 完整任务：点选区域并执行

一条命令启动仿真、规划器、RViz、跟踪器和任务管理器：

```bash
ros2 launch climbot_bringup coverage_mission.launch.py
```

### 点选区域

RViz 里从启动起就画着一个**绿色边框**，那是墙面按机器人安全边距内缩后的可达
区域。点必须落在它里面——落在外面时足迹覆盖不到要求的比例，规划会失败。

工作系的原点在墙面左下角，所以坐标全是非负的：`10 × 8 m` 的墙面配上
`safety_margin = 0.5 × hypot(0.76, 0.475) + 0.10 = 0.548 m`，绿框就是
`x ∈ [0.548, 9.452]`、`y ∈ [0.548, 7.452]`，机器人出生在 `(5.0, 2.0)`。

墙面上和 RViz 里都画着 `1 m` 的**参考网格线**。线落在工作坐标的整数倍上、只画
内部线，两个视图用的是同一条规则和同一个间距（`climbot_description/config/wall.yaml`
的 `reference_grid.spacing_m`），所以从任一视图读出来的坐标一致。

- **临时藏起来**：在 RViz 左侧 Displays 里取消勾选 `Wall Reference Grid`，立即生效，
  不影响 Gazebo 里墙面上那套。
- **整轮都不画**：加 `wall_grid_spacing:=0`，Gazebo 墙面和 RViz 叠加层同时不画。
  拍摄墙面的运行必须这样跑——网格线是周期性高对比特征，还浮在墙面前 `3 mm`，
  在 `0.50 × 0.28 m` 的拍摄视场下会进入 `67%` 的照片。

```bash
ros2 launch climbot_bringup coverage_mission.launch.py wall_grid_spacing:=0
```

在工具栏选择 `Publish Point`，按下表顺序点击角点。每次点击 `/coverage/status`
都会回显它认成了哪个角和坐标，用来确认没有因为相机视角而选反方向：

| 区域 | 点击顺序 |
| --- | --- |
| `rectangle`（默认） | A 左下 → B 右上 |
| `trapezoid` | A 左下 → **B 右上** → C 右下 |

梯形第二下是右上、第三下才是右下，容易顺手点成逆时针。手点两个底角不可能等高，
规划器会取均值，差超过 `bottom_warning_tolerance`（默认 `50 mm`）时状态里会多一句
`Bottom clicks differed by ... and were corrected to their mean height.`，这是提示
不是错误。

最后一次点击后 RViz 中出现规划路径，左侧 **Coverage Task** 面板的 State 变为
`Ready` 并显示任务名与 revision。

### 面板

![Coverage Task 面板](images/rviz_coverage_task.png)

| 行 | 内容 |
| --- | --- |
| **Region** / **Sweep** | 区域形状与扫描方向，见下节 |
| **Algorithm** | 直线段的控制律：`Position only` 或 `Timed trajectory`，见下节 |
| **Points** | 点选模式下已选/需要的点数 |
| **State** | 管理器状态 |
| **Segment** | 当前段 / 总段数 |
| **Progress** | 按各段**预计耗时**加权的完成比例 |
| **Schedule** | 任务总时长、预计剩余、以及相对时间表的滞后 |

| 按钮 | 作用 |
| --- | --- |
| **Start** | 开始执行，State 转为 `Executing` |
| **Cancel / Stop** | 中途停车 |
| **Clear points** | 点错角点时清空重选 |
| **Replan** | 用当前角点重新规划 |

**任务运行期间，Region、Sweep、Algorithm、Replan、Clear points 五个控件全部
置灰。** 运行中唯一能做的事是 Cancel。这些控件发出的请求确实只改预览、不动
正在执行的 Goal，但预览就是画在机器人身上的那条轨迹，运行中改它看起来就像任务
被换掉了。

Start 和 Cancel 的置灰由管理器发布的 `can_start` / `can_cancel` 决定，五个规划
控件的置灰同样取自 `can_cancel`，不是面板另外判断一遍状态。无论面板显示什么，
非法请求都由管理器拒绝，原因显示在 Last request 一行。

几个容易踩的点：

- 取消或跑完之后 **Start 仍然可用**，再按一次就重跑同一个任务；
- 点选模式下**没选够点时 Replan 会被拒绝**，因为配置文件里的角点仍在，否则会规划
  出一块没人选过的区域；
- Planner 一行显示规划器自己的状态——规划失败和"没选区域"在管理器看来都是空任务，
  只有这一行能区分。

面板由 `climbot_rviz_plugins` 提供，已写入 `coverage.rviz`，随 launch 自动出现；
若被关掉，用 RViz 菜单 `Panels → Add New Panel → climbot_rviz_plugins/Coverage`
恢复。排版细节见
[`src/climbot_rviz_plugins/README.md`](../src/climbot_rviz_plugins/README.md)。

同样的操作也可以走命令行，两者等价：

```bash
ros2 service call /coverage/start std_srvs/srv/Trigger
ros2 service call /coverage/cancel std_srvs/srv/Trigger
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
时间参数化的。实测数据见 [`results/README.md`](../results/README.md)「两种算法的
对照」和 [PLAN_2026-08-18_time_control.md](PLAN_2026-08-18_time_control.md) 第 8 节。

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
命令输出末尾。用途和有效性见 [results/README.md](../results/README.md)。

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

### 扫描用的开关

| 开关 | 含义 | 默认 |
| --- | --- | --- |
| `-s` / `-i` | 全站仪 / IMU 噪声种子 | `42` / `17` |
| `-n` | 全站仪位置噪声 `stddev`，米 | `0.001` |
| `-r` | 全站仪发布频率，Hz | `12.0` |
| `-d` | 全站仪丢包率 | `0.0` |
| `-o` | 覆盖一个 `line_tracker` 参数，`name=value`，可重复 | 无 |

默认值全部照抄 launch 文件的，所以不带开关跑出来就是普通配置，每个扫描点与基线只差
一个数。`-o` 是拷一份 `control.yaml` 打补丁放进运行目录，不动工作树。

**实际生效的值不靠转述**：评价器启动后向 `total_station_sim`、`wall_imu_adapter` 和
`line_tracker` 的参数服务问回来，写进摘要的 `provenance.noise_sources` 和
`provenance.control_parameters`。传给一个没起来的节点的参数，在摘要里看得出来。

### §14.5 定位对照

不走回归脚本，单独跑一次四方向闭环，逐段用真值同时测融合位姿误差和轮式航位推算
误差：

```bash
ros2 launch climbot_gazebo climbot_wall.launch.py use_sim_time:=true headless:=true
ros2 run climbot_gazebo evaluate_localization.py --ros-args -p use_sim_time:=true \
    -p summary_json:=results/localization_<tag>_summary.json
```

## 安全提示

Gazebo DiffDrive 会持续执行最后收到的 `/cmd_vel`，因此仿真 launch 始终启动速度
看门狗，并由它作为 `/cmd_vel` 的唯一发布者。键盘、实验脚本和自动控制统一发布到
`/control/cmd_vel`；当前一次只能运行一个上游控制源，不要同时启动键盘和自动任务。

控制环和看门狗的定时器**不使用节点默认时钟**。节点默认时钟在非仿真时间下就是
系统时钟，可以被设置、可以往回跳（WSL2 每约 30 s 回跳 1～2 s），建在它上面的
定时器在回跳期间不触发——控制器整段不发指令，而机器人还在按最后一条指令走。
仿真时间激活时跟 `/clock`，否则用单调时钟，见
[INTERFACES.md](INTERFACES.md) 的"控制环时钟"。
