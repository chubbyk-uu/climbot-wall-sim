# 实验结果说明

本目录保存可追溯的实验输出，不是所有文件都代表当前正式验收基线。正式实验
应同时记录代码提交、配置、随机种子、仿真时长和生成命令。

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
| `turn_slip.csv` | 当前正式基线 | 2026-08-17 重测，多角度、多角速度原地转向下滑；参考点已在旋转中心 |
| `turn_slip_2026-08-13_uncorrected.csv` | 历史对照，**不可用于标定** | 采集时参考点在主动轮轴后约 `79 mm`，逐角度、逐方向的数值混着运动学摆动 |
| `wall_slip_trajectory.csv.gz` | 当前正式基线 | 10209 行；真值时间戳去重；包含真值与融合航向 |
| `wall_slip.png` | 当前正式基线 | 由当前侧滑轨迹 CSV 生成 |
| `coverage_horizontal_2026-08-17_*` | 阶段 F 正式结果 | 大型横向矩形，4 条扫描线、7 段 |
| `coverage_vertical_2026-08-17_*` | 阶段 F 正式结果 | 大型竖向矩形，8 条扫描线、15 段 |
| `coverage_trapezoid_horizontal_2026-08-17_*` | 阶段 F 正式结果 | 大型等腰梯形横向扫描，13 段 |
| `coverage_trapezoid_vertical_2026-08-17_*` | 阶段 F 正式结果 | 同一等腰梯形竖向扫描，19 段 |
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

## 覆盖基线（四工况）

2026-08-17 在四个全新无界面仿真中各执行一次，每次都从新启动的世界开始。矩形
两个工况为 `4.30 × 1.70 m` 横向和 `3.30 × 4.50 m` 竖向；梯形两个工况为底边
`4.00 m`、上底 `2.60 m`、高 `2.80 m` 的同一等腰梯形分别横向和竖向扫描。四次都用
`0.50 × 0.01 m` 检测足迹和 `10 mm` 覆盖栅格，只累计正式 `SCAN` 直线，不把转向、
换道和入轨运动算成覆盖。

| 指标 | 矩形横向 | 矩形竖向 | 梯形横向 | 梯形竖向 | §14.3 阈值 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Action 结果 | 7/7 段 | 15/15 段 | 13/13 段 | 19/19 段 | 成功 |
| 扫描线条数 | 4 | 8 | 7 | 10 | — |
| 执行时间 | 134.3 s | 288.0 s | 231.0 s | 283.1 s | — |
| 实际覆盖率 | 99.964% | 99.638% | 99.798% | 98.070% | ≥ 95% |
| 最大单段横轨 RMS | 1.08 mm | 1.64 mm | 1.90 mm | 2.23 mm | ≤ 20 mm |
| 最大绝对横轨误差 | 2.53 mm | 2.65 mm | 8.78 mm | 3.40 mm | ≤ 50 mm |
| 最大动态终点误差 | 3.15 mm | 3.65 mm | 3.42 mm | 3.78 mm | ≤ 30 mm |
| 最大转向结束航向误差（真值口径） | 1.64° | 1.82° | 1.73° | 1.78° | ≤ 2° |
| 最大扫描线偏离名义 | 3.20 mm | 9.30 mm | 11.22 mm | 6.04 mm | — |
| 最大相邻扫描线间距误差 | 5.08 mm | 10.83 mm | 16.26 mm | 6.27 mm | ≤ 20 mm |
| 最大水平扫描净高度误差 | 1.34 mm | 不适用 | 2.18 mm | 不适用 | ≤ 30 mm |
| 实际/名义线段总长比 | 1.0303 | 1.0178 | 1.0453 | 1.0232 | — |
| 最大机体航向补偿角 | 7.34° | 7.02° | 6.71° | 7.01° | — |
| 正式直线最大指令角速度 | 0.061 rad/s | 0.069 rad/s | 0.078 rad/s | 0.064 rad/s | — |
| 可见反向往复 | 0 | 0 | 0 | 0 | 0 |

四个工况全部通过评价器的每一项门限。相对 §14.3 于 2026-08-14 放宽后的 `95%`
覆盖率门限，余量分别为 `4.96 / 4.64 / 4.80 / 3.07` 个百分点，因此本版仍不增加
顶部收边扫描。**"不需要收边"不是永久结论**：修改检测足迹、行距、重叠率、转向
下滑或入轨参数后必须重新评价，而梯形竖向 `3.07` 个百分点是四者中最薄的一份。

新增的横向矩形是四者中最好的一份（覆盖率 `99.964%`、最大横轨 RMS `1.08 mm`、
间距误差 `5.08 mm`）。它扫描线最少、转向最少，符合预期。

竖向矩形与两个梯形相对 2026-08-14 那一版的差异都在测量噪声量级内
（覆盖率 `99.632→99.638%`、`99.798→99.798%`、`98.122→98.070%`），说明 8 月 15
至 17 日的三次修改（进度加权、面板状态、控制环时钟）确实没有改变运动行为——
这三处要么只影响状态上报，要么在 `use_sim_time:=true` 下逐位不变。

## 重新生成覆盖基线

每次正式实验都从全新启动的无界面仿真开始。`climbot_wall.launch.py` 不启动规划器
和执行器，需分别启动；三个工况只有 `--params-file` 与输出文件名不同：

```bash
# 终端 1：仿真、TF、传感器、EKF 和速度看门狗
ros2 launch climbot_gazebo climbot_wall.launch.py \
  use_sim_time:=true headless:=true

# 终端 2：规划器（四个工况只有这一行的配置和 sweep_direction 不同）
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
  -p trajectory_csv:=results/coverage_horizontal_2026-08-17_trajectory.csv.gz \
  -p summary_json:=results/coverage_horizontal_2026-08-17_summary.json
```

四个工况的配置与扫描方向：

| 工况 | `config_file` | `region_type` | `sweep_direction` |
| --- | --- | --- | --- |
| 矩形横向 | `coverage_horizontal_demo.yaml` | `rectangle` | `horizontal` |
| 矩形竖向 | `coverage_vertical_demo.yaml` | `rectangle` | `vertical` |
| 梯形横向 | `coverage_trapezoid_horizontal_demo.yaml` | `trapezoid` | `horizontal` |
| 梯形竖向 | `coverage_trapezoid_vertical_demo.yaml` | `trapezoid` | `vertical` |

必须用 `planner_config_file` / `control_config_file` 这类互不冲突的名字，或者像
上面这样分别启动：被包含的 launch 会继承父作用域的同名参数，一个 `config_file`
会同时落到规划器和跟踪器上，使后者静默退回内置默认值（包括侧滑补偿为 `0`）。

评价器在任一验收项不达标时返回失败，但无论成功、超时还是异常都会先写出 CSV 和
摘要；失败运行的摘要里 `completed=false` 并记录 `failure_reason`，不要把这类文件
当作基线。摘要的 `provenance.git.source_modified` 为 `true` 表示该结果产生时
`src` 有未提交改动，此时记录的提交号只是父提交。

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
