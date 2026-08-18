# 实验结果说明

本目录保存可追溯的实验输出，不是所有文件都代表当前正式验收基线。正式实验
应同时记录代码提交、配置、随机种子、仿真时长和生成命令。

> ## 吸附力分界：220 N / 400 N
>
> 2026-08-18 把 `suction.force_n` 从 `220.0` 提到 `400.0`，同时改了
> `nominal_normal_force_n`（77 → 133.3）和两个 WheelSlip 柔度（0.12/0.04 →
> 0.208/0.069）。原因是 220 N 下主动轮摩擦饱和，见 `docs/STATUS.md`。
>
> **凡是文件名日期早于 `2026-08-18b` 的，都产生于 220 N 配置，不能与之后的结果
> 直接比较，也不能用来标定当前配置。**
>
> 覆盖归档一共三批，配置各不相同：
>
> | 标签 | 吸附力 | `alignment_tolerance_deg` | 地位 |
> | --- | ---: | ---: | --- |
> | `2026-08-18b` | 400 N | 2.0 | 历史对照 |
> | `2026-08-18c` | 400 N | 0.5 | 历史对照（收窄死区的中间验证，5 个工况） |
> | **`2026-08-18d`** | **400 N** | **1.0** | **当前正式基线（8 个工况）** |
>
> 标定基线是 `turn_slip.csv` 和 `turn_map.csv`，两者都在 400 N 下重测。

> **四份覆盖基线已于 2026-08-17 在提交 `6227a3c` 的干净工作树上重跑。** 每份摘要
> 的 `provenance.git` 都是 `commit=6227a3c`、`source_modified=false`，此后不会再
> 出现"不知道哪一版代码产生"的情况。这次重跑做了两件事：补上一直缺失的**横向
> 矩形**工况（§13 阶段 F 第 2 条要求横向和竖向矩形各一份），并把 8 月 14 日那三份
> 带未提交改动的结果换成可追溯的版本。

| 文件 | 状态 | 说明 |
| --- | --- | --- |
| `normal_loads_before_p2.csv` | 历史对照 | P2 调整前的法向载荷 |
| `normal_loads_after_p2.csv` | 历史对照 | P2 调整后的法向载荷 |
| `normal_loads_after_base_link_move.csv` | 当前几何参考 | `base_link` 移至主动轮轴中点后的七工况载荷 |
| `turn_slip.csv` | 当前正式基线（400 N） | 20 次原地转向拟合 `turn_slip_per_degree_m=0.00041`，残差 RMS `0.5 mm` |
| `turn_map.csv` | 当前正式基线（400 N） | 全圆 48 次转向，`0.392~0.441 mm/度`，带已消失 |
| `turn_band.csv` | 历史对照（220 N），**不可用于标定** | 起步滑移带的加密扫描，峰值 `2.55 mm/度` |
| `band_variants/` | 220 N 成因实验 | 18 个单参数变体，证明带是主动轮摩擦饱和；`suction_210/215` 里的位移是脱落后的自由落体，不是量测量 |
| `normal_loads_400N.csv` | 当前几何参考（400 N） | 三接触点七工况载荷；`nominal_normal_force_n` 由此定为 133.3 |
| `turn_slip_2026-08-13_uncorrected.csv` | 历史对照，**不可用于标定** | 采集时参考点在主动轮轴后约 `79 mm`，逐角度、逐方向的数值混着运动学摆动 |
| `wall_slip_trajectory.csv.gz` | 当前正式基线 | 10209 行；真值时间戳去重；包含真值与融合航向 |
| `wall_slip.png` | 当前正式基线 | 由当前侧滑轨迹 CSV 生成 |
| `coverage_*_2026-08-18d_*` | **当前正式基线** | 400 N + 1.0° 死区，八工况全部 `passed=true`，见下表 |
| `coverage_*_2026-08-18c_*` | 历史对照 | 400 N + 0.5° 死区，收窄死区的中间验证，5 个工况 |
| `coverage_*_2026-08-18b_*` | 历史对照 | 400 N + 2.0° 死区，八工况 |
| `coverage_*_2026-08-17_*` | 历史对照（220 N） | 原始四工况的 220 N 版本 |
| `coverage_big*_2026-08-18_*` | 历史对照（220 N + `turnLeadOut()`） | 放大四工况；`turnLeadOut()` 已删除，这批不可复现 |
| `slip_compensation_off_2026-08-17_*` | §14.4 正式结果 | 横轨闭环关闭的三次水平直线 |
| `slip_compensation_on_2026-08-17_*` | §14.4 正式结果 | 同一仿真、同一墙面上横轨闭环开启的三次水平直线 |
| `coverage_*_2026-08-14_summary.json` | 历史对照 | 同三个工况的上一版指标，`source_modified=true`，仅供对比趋势 |

每组 `*` 包含一个 `_trajectory.csv.gz`（逐采样真值、融合位姿、动态参考、状态和
横轨误差）和一个 `_summary.json`（Action 结果、逐段误差、覆盖率和 `provenance`）。

## 归档格式

轨迹以 **gzip 压缩的 CSV**（`.csv.gz`）保存，数值保留到 **6 位小数**。两项合计
把归档压到原来的十分之一（`24.6 MB → 2.4 MB`）：

| 处理 | 效果 |
| --- | ---: |
| 数值取到 `1e-6` | `1.93×` |
| gzip | 再 `4.1×` |
| 合计 | `10.1×` |

`1e-6` 是米、弧度、秒下的微米/微弧度/微秒，而 §14.3 的验收阈值全部以毫米和度
表述——原先写的 17 位有效数字里，第 6 位小数之后是浮点往返表示的噪声，不是测量。
用压缩后的 `wall_slip_trajectory.csv.gz` 重新出图，PNG 与原图**逐字节相同**
（`md5 17be4409…`）。

`climbot_gazebo.trajectory_io` 按文件名后缀决定是否压缩，读写两端都走它，因此
`plot_wall_slip.py` 之类的工具直接传 `.csv.gz` 即可，不需要先解压。

2026-08-14 那三份的轨迹 CSV 已删除，只保留摘要 JSON：它们已被同工况、同参数、
提交更干净的 08-17 版本取代，而摘要里就有全部指标和 `provenance`，对照趋势够用。
**注意删除不会缩小 `.git`**——那些文件仍在历史里，回收需要改写历史，本仓库已推送
故不做。这次调整的意义是止损：以后每次正式实验不再往仓库里加约 7 MB。

## 覆盖基线（八工况）

2026-08-18 在 **400 N 吸附力 + 1.0° 对齐死区**下重跑，四路并行、每路独立世界。原始
四个工况为矩形 `4.30 × 1.70 m` 横向、`3.30 × 4.50 m` 竖向，以及底边 `4.00 m`、上底
`2.60 m`、高 `2.80 m` 的等腰梯形横向和竖向；放大四个为矩形 `5.70 × 3.60 m` 和斜边
`58°` 的梯形。全部用 `0.50 × 0.01 m` 检测足迹和 `10 mm` 覆盖栅格，只累计正式
`SCAN` 直线。

| 指标 | 矩形横 | 矩形竖 | 梯形横 | 梯形竖 | 大矩横 | 大矩竖 | 大梯横 | 大梯竖 | §14.3 阈值 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 覆盖率 % | 99.94 | 100.00 | 99.75 | 98.63 | 99.94 | 100.00 | 99.33 | 98.97 | ≥ 95 |
| 最大动态终点误差 mm | 3.54 | 3.67 | 3.03 | 3.48 | 2.91 | 3.90 | 3.05 | 4.65 | ≤ 30 |
| 最大转向结束航向误差 ° | 0.723 | 0.913 | 0.793 | 0.851 | 0.934 | 0.876 | 0.868 | 0.873 | ≤ 2.0 |
| 最大相邻扫描线间距误差 mm | 3.59 | 2.20 | 3.00 | 4.07 | 3.47 | 4.82 | 3.25 | 5.47 | ≤ 20 |
| 最大扫描线偏离名义 mm | 3.79 | 4.13 | 2.56 | 4.18 | 4.35 | 5.11 | 3.31 | 4.48 | — |
| 最大机体航向补偿角 ° | 6.84 | 6.89 | 6.57 | 6.93 | 6.54 | 6.93 | 6.53 | 6.96 | — |
| 实际/名义线段总长比 | 1.0246 | 1.0180 | 1.0374 | 1.0276 | 1.0214 | 1.0165 | 1.0273 | 1.0376 | — |
| 执行时间 s | 138.5 | 301.2 | 228.0 | 292.4 | 289.4 | 409.1 | 247.3 | 398.3 | — |
| 可见反向往复 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

**八个工况全部通过评价器的每一项门限**（`passed=true`）。

### 相对 220 N 的改善

| 工况 | 220 N | 400 N | 400 N + 1.0° 死区 |
| --- | ---: | ---: | ---: |
| 大梯形横向 间距误差 | **43.09 mm**（超标） | 2.28 mm | **3.25 mm** |
| 大梯形竖向 间距误差 | **26.87 mm**（超标） | 6.63 mm | **5.47 mm** |
| 大矩形竖向 转向结束航向 | 2.028°（超标） | 1.991° | **0.876°** |

前两行靠提高吸附力消除主动轮摩擦饱和解决；第三行与摩擦无关，靠把
`alignment_tolerance_deg` 从 2.0 收到 1.0 解决——它原本与 §14.3 的验收阈值取同一个
数，余量结构性为零。八个工况的航向误差现在是 `0.723~0.934°`，余量 53%~64%。

`ALIGN_SETTLE` 是死区不是收敛目标：误差一进死区就把角速度指令置零、不再纠正，所以
残差是穿过边界那一刻的余量。收窄死区没有引发振荡——八个工况共 **182 次整定，单次
角速度符号翻转从未超过 1 次**（1 次是冲过头后的正常单次回调，振荡需要 ≥2 次）。
代价是整定均值从 0.80 s 升到约 1.15 s。

## 重新生成覆盖基线

```bash
tools/run_coverage_regression.sh                    # 全八工况，四路并行，标签用今天日期
tools/run_coverage_regression.sh -j 6               # 六路
tools/run_coverage_regression.sh -t 2026-08-18d     # 指定标签
tools/run_coverage_regression.sh bigV bigTH         # 只跑指定工况
tools/run_coverage_regression.sh -k                 # 保留未压缩轨迹
```

脚本从新启动的无界面世界执行每个工况，写出 `results/coverage_<工况>_<标签>_*`，
末尾打印验收汇总表；**任一工况不达标返回退出码 1** 并列出工况名，可直接进 CI。
八个工况四路并行约 11 分钟，串行约 70 分钟。

### 并行需要两层隔离，不是一层

`ROS_DOMAIN_ID` 只隔离 DDS。Gazebo 走的是 **gz-transport**，一套独立的发现机制，
两个实例的话题路径（`/world/climbot_wall/model/climbot/...`）完全相同，不设
`GZ_PARTITION` 就会让桥接订阅到另一个世界。**这个故障不会崩溃**——数据看起来正常，
只是来自别人的仿真，比直接报错危险得多。脚本给每路配一组
`ROS_DOMAIN_ID` + `GZ_PARTITION`，收尾时读每个进程自己的 `/proc/<pid>/environ`
匹配分区来定位，一条路杀不到另一条路。

隔离验证：同一工况并行与串行的**覆盖率逐位相同**（`99.93%` / `99.74%`），这是整条
轨迹在 10 mm 栅格上的积分，任何串台都会立刻体现。

并行不降速：瓶颈是 1 ms 物理步长的单线程串行，四个实例各占约 70% 的一个核，实时
因子 `0.85~0.94`，与单独运行的 `0.84~0.94` 一致。24 核机器上还有余量再加路。

### 逐工况手动运行

需要单独调试某个工况时，四个终端分别启动：

```bash
# 终端 1：仿真、TF、传感器、EKF 和速度看门狗
ros2 launch climbot_gazebo climbot_wall.launch.py use_sim_time:=true headless:=true

# 终端 2：规划器（工况之间只有这一行的配置和 sweep_direction 不同）
ros2 launch climbot_coverage coverage_planner.launch.py \
  use_sim_time:=true rviz:=false \
  config_file:="$(pwd)/src/climbot_coverage/config/coverage_horizontal_demo.yaml" \
  input_mode:=parameters region_type:=rectangle sweep_direction:=horizontal

# 终端 3：跟踪器和管理器
ros2 launch climbot_control coverage_executor.launch.py use_sim_time:=true

# 终端 4：发送 Action 并按真值评价
ros2 run climbot_gazebo evaluate_coverage_execution.py --ros-args \
  -p use_sim_time:=true -p case:=planned_task \
  -p startup_timeout_s:=30.0 -p execution_timeout_s:=600.0 \
  -p trajectory_csv:=results/coverage_horizontal_manual_trajectory.csv \
  -p summary_json:=results/coverage_horizontal_manual_summary.json
```

八个工况的配置与扫描方向：

| 工况 | `config_file` | `region_type` | `sweep_direction` |
| --- | --- | --- | --- |
| 矩形横向 | `coverage_horizontal_demo.yaml` | `rectangle` | `horizontal` |
| 矩形竖向 | `coverage_vertical_demo.yaml` | `rectangle` | `vertical` |
| 梯形横向 | `coverage_trapezoid_horizontal_demo.yaml` | `trapezoid` | `horizontal` |
| 梯形竖向 | `coverage_trapezoid_vertical_demo.yaml` | `trapezoid` | `vertical` |
| 大矩形横向 | `coverage_horizontal_large.yaml` | `rectangle` | `horizontal` |
| 大矩形竖向 | `coverage_vertical_large.yaml` | `rectangle` | `vertical` |
| 大梯形横向 | `coverage_trapezoid_horizontal_large.yaml` | `trapezoid` | `horizontal` |
| 大梯形竖向 | `coverage_trapezoid_vertical_large.yaml` | `trapezoid` | `vertical` |

必须用 `planner_config_file` / `control_config_file` 这类互不冲突的名字，或者像上面
这样分别启动：被包含的 launch 会继承父作用域的同名参数，一个 `config_file` 会同时
落到规划器和跟踪器上，使后者静默退回内置默认值（包括侧滑补偿为 `0`）。

评价器在任一验收项不达标时返回失败，但无论成功、超时还是异常都会先写出 CSV 和
摘要；失败运行的摘要里 `completed=false` 并记录 `failure_reason`，不要把这类文件
当作基线。

## §14.4 侧滑补偿专项验收

2026-08-17 在**同一个**无界面仿真、同一段墙面上依次执行两个阶段，提交
`515e5e3`、工作树干净。两个阶段的速度（`0.15 m/s`）、名义直线长度（`1.20 m`）、
吸附力和 WheelSlip 参数完全相同，差别只有横轨闭环的开关。

| 指标 | 补偿关闭 | 补偿开启 | §14.4 要求 |
| --- | ---: | ---: | --- |
| 三次净高度误差 | `-122.1 / -121.2 / -121.0 mm` | `-14.5 / +3.8 / -0.6 mm` | — |
| 三次实测直线长度 | `1.169 / 1.169 / 1.169 m` | `1.201 / 1.201 / 1.199 m` | — |
| 平均净高度误差绝对值 | `121.43 mm` | `6.29 mm` | 降低 ≥ 70% → **94.82%** |
| 平均下降/前进比 | `10.387%` | `0.313%` | 归一化降幅 **96.99%** |
| 三次下降比 | `10.45 / 10.36 / 10.35%` | — | 变异系数 ≤ 5% → **0.41%** |
| 漂移方向 | 三次全部向下 | — | 稳定、可重复的向下漂移 ✔ |
| 平均机体航向偏角 | `+0.45°` | `+6.13°`（向上） | 允许小幅向上偏角 ✔ |
| 位置轨迹相对水平线的倾角 | — | `0.69 / 0.18 / 0.03°` | 位置比机体朝向更贴近规划直线 ✔ |
| 可见反向往复（`20 mm` 门限） | — | `0` | 无明显蛇形 ✔ |

六条要求全部满足。最后两行是这一节的关键证据：补偿开启后机体持续向上偏
`6.13°`，而**实际走出来的轨迹**相对水平线只倾斜 `0.03~0.69°`——机器人是"斜着身
子走直线"，正是 §10.4 期望的形态，而不是把整条线走歪。

补偿关闭态与 2026-08-13 的独立标定基线互相印证：下降比 `10.39%` vs `10.56%`，
变异系数 `0.41%` vs `0.64%`。两者用不同脚本、不同仿真实例测得。本次实验中该阶段
还被完整重复了三遍（三次独立仿真给出 `10.39 / 10.39 / 10.38%`，变异系数
`0.41 / 0.34 / 0.41%`）。

**第一条扫描线不计入**：掉头对准会带来约 `85 mm` 下坠，§10.7 把参考线平移到实际
位置而不是爬回名义线，因此第一条扫描线的起始偏差必然超过
`parallel_scan_offset_m`（`45 mm`），跟踪器改用一次前进小弧线入轨，吃掉约
`0.33 m`。它照常执行但不参与统计；其后三条扫描线之间没有转向，是与开环段可比的
稳态直线。加长引入段无效，实测 `0.5 m` 和 `1.2 m` 都仍把 `81~87 mm` 交给扫描段。

## 重新生成 §14.4 配对实验

两个阶段必须在**同一个**仿真里依次运行，而且 `open_loop` 阶段不能有
`line_tracker`：空闲的跟踪器同样以 `50 Hz` 在 `/control/cmd_vel` 上发布零速，会
覆盖开环指令。速度看门狗由 `climbot_wall.launch.py` 启动，`/cmd_vel` 的安全门与
正常任务一致。

```bash
# 终端 1：仿真、传感器、EKF 和速度看门狗，不含跟踪器
ros2 launch climbot_gazebo climbot_wall.launch.py \
  use_sim_time:=true headless:=true

# 终端 2，阶段一：补偿关闭
ros2 run climbot_gazebo evaluate_slip_compensation.py --ros-args \
  -p use_sim_time:=true -p mode:=open_loop \
  -p trajectory_csv:=results/slip_compensation_off_2026-08-17_trajectory.csv.gz \
  -p summary_json:=results/slip_compensation_off_2026-08-17_summary.json

# 阶段一结束后再启动跟踪器和管理器
ros2 launch climbot_control coverage_executor.launch.py use_sim_time:=true

# 终端 2，阶段二：补偿开启，并按关闭态摘要计算降幅
ros2 run climbot_gazebo evaluate_slip_compensation.py --ros-args \
  -p use_sim_time:=true -p mode:=compensated \
  -p reference_summary_json:=results/slip_compensation_off_2026-08-17_summary.json \
  -p trajectory_csv:=results/slip_compensation_on_2026-08-17_trajectory.csv.gz \
  -p summary_json:=results/slip_compensation_on_2026-08-17_summary.json
```

任一验收项不达标时进程返回失败，摘要仍会写出并带 `failure_reason`。

## 原地转向下滑基线

2026-08-17 在全新无界面仿真中重测：5 个角度（`30/45/90/135/180°`）× 2 个角速度
（`0.3/0.6 rad/s`）× 2 次重复，每次交替转向，共 20 次。

| 指标 | 结果 |
| --- | ---: |
| 拟合 `turn_slip_per_degree_m` | **`0.00050`**（即 `0.50 mm/度`） |
| 拟合出的参考点偏移（自检） | **`0.2 mm`** |
| 模型残差 RMS | `2.0 mm` |
| 与 `control.yaml` 现值之差 | `0.4%` |

**下滑量与角速度无关**：`0.3` 和 `0.6 rad/s` 两组在同一角度上几乎相同
（`90°`：`-43.5` vs `-43.9 mm`；`180°`：`-90.3` vs `-90.8 mm`），说明这是准静态
滑移，只取决于转过的角度。因此单个 `mm/度` 常数是合适的模型形式，不需要按转向
方向或角速度拆成多个参数。

### 为什么旧数据不能直接用

`turn_slip_2026-08-13_uncorrected.csv` 采于把 `base_link` 移到主动轮轴中点
（提交 `89feb21`，13:56）**之前** 15 分钟（提交 `0598525`，13:41）。参考点当时在
旋转中心后方约 `79 mm`，原地转向会让它绕中心划一段弧——这段**运动学摆动**和真实
下滑一起进了 `vertical_mm`。后果是逐方向的数值毫无物理意义：

| 转角 | 旧数据 CW | 旧数据 CCW | 新数据 CW | 新数据 CCW |
| ---: | ---: | ---: | ---: | ---: |
| `90°` | `-123.8 mm` | **`+34.2 mm`** | `-42.2 mm` | `-45.2 mm` |
| `180°` | `-243.6 mm` | **`+68.3 mm`** | `-90.5 mm` | `-90.6 mm` |

旧数据里逆时针大角度净位移**向上**——重力滑移不可能这样。新数据两个方向一致到
`0.0 mm/度`。

**但聚合系数一直是对的**：标定扫描两个方向对称，摆动在总体斜率里互相抵消，所以
两份数据拟合出的系数都是 `0.00049~0.00050`。`control.yaml` 里那个手工填的
`0.0005` 因此并没有错，只是当时无法证明。

## 重新标定转向下滑

```bash
ros2 launch climbot_gazebo climbot_wall.launch.py \
  use_sim_time:=true headless:=true

ros2 run climbot_gazebo measure_turn_slip.py --ros-args \
  -p use_sim_time:=true -p output_csv:=results/turn_slip.csv
```

脚本结束时联合拟合参考点偏移和下滑系数，直接打印可填进 `control.yaml` 的
`turn_slip_per_degree_m`，并把偏移作为自检报出：超过
`maximum_reference_offset_m`（默认 `0.05 m`）即报错，说明上报位姿不在旋转中心，
CSV 里的逐角度数值不能当作真实滑移读。换墙面、改摩擦或 WheelSlip 参数后都必须
重标定。

## 起步滑移带

`measure_turn_slip.py` 从机器人当时朝向起转，因此漏掉了一个只取决于**起始朝向**的
效应。`measure_turn_band.py` 把起始朝向固定成自变量，扫「起始朝向 × 转角 × 转向
方向」：

```bash
ros2 launch climbot_gazebo climbot_wall.launch.py \
  use_sim_time:=true headless:=true

# 全圆映射（results/turn_map.csv）：每 15° 一个朝向，正反各转 30°
ros2 run climbot_gazebo measure_turn_band.py --ros-args \
  -p use_sim_time:=true -p output_csv:=results/turn_map.csv

# 带形细扫（results/turn_band.csv）
ros2 run climbot_gazebo measure_turn_band.py --ros-args \
  -p use_sim_time:=true -p output_csv:=results/turn_band.csv \
  -p headings_deg:='[100.0, 105.0, 110.0, 112.0, 114.0, 116.0, 120.0, 125.0]' \
  -p angles_deg:='[30.0]'
```

**这条带只存在于 220 N 配置**：起步做原地转向时，若朝向偏离竖直 12°~40° 且转向
方向使车头继续下压，起步瞬间沿车身轴滑约 **68 mm**，与转角无关；反向转、或已扫过
该带，都正常。峰值在偏离竖直 24°（即 114°/66°/246°/294°）。

**400 N 下带已消失**，`turn_map.csv` 全圆 48 次转向读数 `0.392~0.441 mm/度`，六个
原危险航向与其余航向无异。曾按这两份 CSV 标定的 `turnLeadOut()` 已随之删除。改
墙面、摩擦、吸附力、机器人质量或 WheelSlip 参数后都必须重扫这张全圆图，确认它仍
是平的。

### 带的成因：主动轮摩擦饱和

`results/band_variants/` 是把吸附力和摩擦系数当自变量的对照扫描，每个变体在
`100/105/110/114/120/125°` 起转 `+30°`。复现方式是改
`src/climbot_gazebo/config/simulation.yaml`（`install/` 是符号链接，不必重编），
每个变体重启一次仿真后：

```bash
ros2 run climbot_gazebo measure_turn_band.py --ros-args \
  -p use_sim_time:=true \
  -p output_csv:=results/band_variants/<变体名>.csv \
  -p headings_deg:='[100.0, 105.0, 110.0, 114.0, 120.0, 125.0]' \
  -p angles_deg:='[30.0]'
```

| 变体 | 峰值 mm/度 |
| --- | ---: |
| `baseline`（220 N，`wheel_mu` 1.1） | 2.536 |
| `lateral_0`（`slip_lateral` → 0） | 2.637 |
| `longitudinal_0`（`slip_longitudinal` → 0） | 2.530 |
| `caster_mu_1p1`（0.35 → 1.1） | 2.536 |
| `wheel_mu_2.2`（墙面与主动轮同时 1.1 → 2.2） | 0.416 |
| `wheel_mu_0.7` | 整机脱落 |
| `suction_210` / `suction_215` | 整机脱落 |
| `suction_222` | 1.270 |
| `suction_225` / `230` / `240` / `260` | 0.531 / 0.525 / 0.519 / 0.508 |
| `suction_440` / `880` | 0.416 / 0.338 |

`wheel_mu` 加倍与吸附力加倍给出**逐点相同**的曲线，所以支配量是乘积 `μN`，带是
主动轮的库仑摩擦饱和。万向轮 μ 无关——它是滚动的球，只抢载荷不出切向力。

`suction_210` 与 `suction_215` 的 CSV 里 `slide_mm` 是几百米到几千公里、
`wall_height_m` 一路变负，那是脱落后的自由落体，不是量测量，只用来定脱落门槛。

详细核算见 `docs/STATUS.md` 的「220 N 贴在脱落悬崖边」。

## 当前侧滑基线

2026-08-13 在全新启动的无界面仿真中完成一次正式实验。参数为横向 WheelSlip
`0.12`、纵向 WheelSlip `0.04`、速度 `0.15 m/s`、静止 `30 s`、各运动段
`8 s`、三次重复。水平段只保持融合航向为 `0°`，不使用横轨位置纠偏。

| 指标 | 结果 |
| --- | ---: |
| 静止高度变化 | < 0.1 mm |
| 三次水平下降/前进比 | 10.47% / 10.64% / 10.57% |
| 水平下降比均值 | 10.56% |
| 水平下降比总体变异系数 | 0.64% |
| 平均上行速度 | 0.14086 m/s |
| 平均下行速度 | 0.15197 m/s |
| 下行比上行快 | 7.89% |
| 重复 `phase + time_s` | 0 |

三重复总体变异系数低于 `5%` 验收阈值。正式 CSV 中水平段真值平均航向为
`0.28°～0.41°`，因此该下降率主要反映重力侧滑，而不是未约束的航向漂移。

## 重新生成侧滑基线

每次正式标定必须从全新启动的仿真开始；不能在同一世界状态中连续运行两次。
在工作区根目录分别运行：

```bash
ros2 launch climbot_gazebo climbot_wall.launch.py headless:=true
```

然后在另一终端运行：

```bash
ros2 run climbot_gazebo calibrate_wall_slip.py --ros-args \
  -p use_sim_time:=true \
  -p repetitions:=3 \
  -p static_duration_s:=30.0 \
  -p drive_duration_s:=8.0 \
  -p horizontal_repeatability_max_cv:=0.05 \
  -p trajectory_csv:=results/wall_slip_trajectory.csv.gz

ros2 run climbot_gazebo plot_wall_slip.py results/wall_slip_trajectory.csv.gz
```

命令在水平下降比总体变异系数超过阈值时返回失败，但仍会先保存 CSV 供诊断。
替换基线前还应确认同一 `phase + time_s` 无重复，并记录当前配置和代码提交。
