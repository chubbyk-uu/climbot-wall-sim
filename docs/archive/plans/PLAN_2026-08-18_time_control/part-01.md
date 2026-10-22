# 历史分册 1

# 基于时间点的轨迹控制：设计与实施计划

制定日期：2026-08-18。分支 `feature/time-parameterized-control`。
**执行结果见文末第 8 节；第 3 节的设计结论已全部实测验证，第 7 节的风险已闭合。**

本文档记录把项目 Word 文档 §5.3《基于两点直线运动的位置和航向闭环控制》所述
的时间参数化控制引入 `climbot_control` 的设计结论和实施步骤。目标、算法约束和
验收阈值仍以 [PROJECT_GUIDE.md](../../../../PROJECT_GUIDE.md) 为准；本文档只覆盖本次改动。

## 1. 目标

现有直线跟踪是**空间控制**：恒定 `cruise_speed` 前进，终点靠
`sqrt(2·a·remaining)` 的距离制动收尾，控制器不知道"现在应该走到哪儿"。
Word 文档描述的是**时间参数化控制**：离线算出梯形/三角形速度曲线，按控制周期
离散化得到 `(t_k, v_k, d_k)`，运行时以 `v_k` 为前馈、`d_k − d_实际` 为反馈修正。

引入它的两个动机：

1. 位置偏差在时间轴上可观测，落后时可以主动追赶，而不是被动等它走完；
2. 每段的执行时长从"估计"变成"下发的时间表"，任务总时长和 ETA 因此可以对外
   承诺，而不只是内部用来给进度条加权。

## 2. 现状对照

| Word 文档内容 | 现状 |
| --- | --- |
| 图27/28 原地旋转：梯形/三角形 `ω(t)`、`θ(t)`，按 `t_k` 取 `ω_k`，PID 修正 `(θ_k − θ_f)` | **已实现**。`turn_profile.hpp` 的 `planTurn/sampleTurn` 即该组公式；`line_tracker_node.cpp` 的 `ALIGN_PROFILE` 即图28 框图，PID 退化为 P（`turn_heading_gain`） |
| 图29/30 直线：梯形/三角形 `v(t)`、`d(t)`，按 `t_k` 取 `v_k`，PID 修正 `(y_k − y_f)` | **未实现**，本次要做 |
| 图31 角速度双环：`x'` 误差 → `dθ` → 叠加到 `θ_t` → 与 `θ_f` 做差 → `ω` | **结构已有**。`cross_gain/cross_integral_gain → heading_correction → target_yaw → heading_gain → ω` 就是这个双环，外环 PI、内环 P |

所以本次真正新写的只有直线段的时间参考和纵向速度环，其余是接线和参数。

## 3. 设计决策

### 3.1 PID 按需选择，不强制三项齐全

文档中的 PID 是示意。转向段现纯 P 即达标（落点 0.8°），直线纵向环先用纯 P，
只有稳态滞后消不掉才加 I。横轨外环维持现有 PI 不动。

### 3.2 抬高执行机构上限，额定工作点不变

速度曲线仍按今天的额定值生成（`vm=0.20`、`av=0.20`、`ωm=0.60`、`aω=1.00`），
多出来的余量**只**留给追赶修正。改 `climbot_description/config/robot.yaml` 的
`drive` 段一处，Gazebo DiffDrive 与控制器的 `wheel_speed_limit /
wheel_acceleration_limit` 同时生效。

电机侧已核对：轮速提到 `0.45 m/s` 只有 `5.6 rad/s`，远低于 `joint_velocity_rps=30`，
`joint_effort_nm=80` 亦充裕。

### 3.3 轮级饱和必须给角速度留预算

`rateLimit()` 末尾是等比缩放左右轮：线速度顶到 `wheel_speed_limit` 时再叠加角速度
修正，两者会被一起削——恰好是最需要转向修正的时刻。约束是

```
v_max ≤ wheel_speed_limit − ω_max · wheel_separation / 2
```

今天最坏组合 `0.20 + 0.35×0.215 = 0.275 < 0.30`，尚未饱和但余量已经很薄；
`max_linear_speed=0.25` 配 `ω_max=0.35` 其实已越界，只是 `ω` 未跑满没暴露。
抬高上限后该约束自动满足。

### 3.4 profile 用对称加减速，`planTravel` 仍保留独立参数

现有的 `0.20 / 0.12` 不对称**不是物理不对称，是"曲线要求"与"限幅器能力"之间
的余量**：

- `max_linear_acceleration: 0.20` —— 速率限幅器的加速上限
- `max_linear_deceleration: 0.25` —— 速率限幅器的减速上限
- `braking_profile_deceleration: 0.12` —— 距离制动曲线**要求**的减速度

距离制动 `sqrt(2a·remaining)` 是位置的函数、没有闭环：指令一慢，机器人仍在前进，
曲线就要求更大的减速度，形成正反馈直至冲过终点。只能靠"曲线要求远小于限幅器
能力"（2× 余量）来堵。加速侧之所以是 `0.20`，是因为加速侧**根本没有曲线**——
`trackLine()` 直接命令 `cruise_speed`（阶跃），实际斜坡完全由限幅器塑造。

time 模式下 profile 是**时间**的函数，且外挂了对 lag 的显式闭环，失效模式不同，
不需要那么保守的余量。因此：

- **profile 用对称 `av = 0.20`**（即文档模型），限幅器提到 catch-up 值（`0.35`），
  两侧同为 1.75× 余量；
- **`planTravel` 仍支持独立加减速参数**。留这个能力是给 `estimateTravelDuration`
  用的：distance 模式的进度条必须继续按 `0.20 / 0.12` 计算，才能与现状逐位一致，
  否则 A/B 会被进度口径变化污染。

同一个"曲线必须低于限幅器"的道理原样适用于 time 模式的**加速侧**：若 profile 的
`av` 等于限幅器上限，加速斜坡顶在边缘，追赶修正一点也送不进去。3.2 的"提上限、
不动额定"正好保证了这一点。

实测影响：横向矩形用例的直线段总时长由 `101.53 s` 降至 `99.20 s`，仅省 `2.33 s`
（1.7%），因为 4.31 m 长段中制动只占很短一截。两种模式的名义时长差异可忽略。

### 3.5 每段时间轴独立重置，不做全任务统一时间表

到达路点、原地转向、进入下一段时 `t=0` 重新起算，profile 按**当时的剩余距离**
现算。全局时间表会让误差跨段累积（前段欠的账后段一直在追），且中途暂停或重规划
后整张表作废。文档本身也是按点到点逐段分析的。

推论：`climbot_interfaces` 不需要增加路点时间戳，**规划器一行都不用改**。

### 3.6 A/B 可比性优先：只覆盖一个信号

time 模式下 `trackLine()` 照常调用，角速度双环、横轨 PI、重力前馈、
"航向超阈值则线速度归零"全部原样保留，**只替换 `desired.linear`**：

```
elapsed = current_time − travel_start_
sample  = sampleTravel(travel_profile_, elapsed)
lag     = sample.distance − (desired.along − travel_start_along_)
linear  = clamp(sample.speed + time_along_gain·lag + I项, 0, catch_up_max_linear_speed)
```

两条曲线的差异因此能唯一归因到线速度这一路。

### 3.7 两个兜底实现但默认关闭

先看裸算法的失效模式，不行再开：

- `time_axis_stretch_enabled`（默认 `false`）：沿轨滞后超阈值时冻结参考时钟推进。
  不开的话，严重打滑会让修正持续饱和、误差不收敛。
- `time_mode_final_approach_enabled`（默认 `false`）：`remaining ≤
  final_approach_distance_m` 时切回距离制动。对称 `av=0.20` 使制动距离由
  `0.167 m` 缩到 `0.10 m`，刹得更晚更狠，落点精度风险上升，此项是对应的兜底。

段完成判据（`updateGoalCompletion`：位置 + 航向 + 静止 + settle）**本次不动**。
"时间到但位置没到"先表现为多花时间收敛，把它量出来再决定要不要改判据。

### 3.8 ARC_ENTRY 保持空间控制，但必须测兼容

`preparePostTurnScan()` 按转向后实际横偏三分支：

| 横偏 | 走向 |
| --- | --- |
| ≤ `parallel_scan_offset_m` = 45 mm | `lockParallelScanLine()`：冻结平行扫描线，直接直线跟踪 |
| 45 ~ 120 mm（`maximum_scan_offset_m`） | **ARC_ENTRY** 弧线入轨 |
| > 120 mm | `TRACKING_FAILED` |

现转向落点误差 2.7~2.9 mm，正常任务全落第一档，**ARC_ENTRY 是恢复路径而非常规
路径**。它走 `followArcEntry()` 独立分支、线速度固定为 `arc_entry_speed_mps`、
不经过 `trackLine()`，因此 time 模式天然不影响它；也不关闭它——关掉会改变换道
几何，污染 A/B 归因。

两个必须处理的接缝：

1. 常规路径 `lockParallelScanLine()` 会**替换 `start_`/`end_`**。`travel_profile_`
   必须在其后、按冻结后的线段规划（时序上落在 `prepareDynamicReference()` 之后，
   实现时验证）。
2. **弧线结束切回 `TRACK_LINE` 的那一刻必须重新 `planTravel()`**，不能沿用进段时
   算的 profile。

### 3.9 进度与总时间：进度条口径不变，另加时间信息（方案 C）

现有 `taskProgress()` 是"按预估时长加权的空间进度"：权重是各段预估时长，段内
推进量看 `command.along / length`（几何比例）。特点是只跟位置有关——卡住就不动，
永远单调不回退。

**进度条口径保持不变。** 改成 `elapsed / duration` 会让机器人卡住时进度条照样
爬到 100%，对操作者是误导。

改为新增三个字段（`ExecuteCoverage.action` feedback 与 `CoverageStatus.msg`）：

```
float64 planned_total_s        # 任务开始时算定，全程不变
float64 schedule_lag_s         # 正 = 落后计划
float64 estimated_remaining_s  # ETA，每周期更新
```

面板在进度条下增加一行。**实现后与此处的设想有两处出入，以实现为准**：
`planned_total_s` 和 `estimated_remaining_s` 在 distance 模式下同样有效——时长模型
预测的是两种模式，且它的每段开销常数本来就是从 distance 运行数据标定的——所以那一
行照常显示，只是末尾写 `estimate only` 而不是滞后，说明没有任何东西执行或监测它。
真正只在 time 模式有意义的是 `schedule_lag_s`。

`planned_total_s` 现在其实已经在算（`total_duration_estimate_`），只是纯估计、
无人保证执行按它走，所以从未对外发布。time 模式使它成为**下发的时间表**，
从"估计"升级为"承诺"，这时公布才有意义。

### 3.10 总时长估算：三层模型

用横向矩形用例（7 段，3×4.31 m 扫描线 + 3×0.4 m 换道）实算：

```
纯 profile（转向 19.31 + 直线 101.53）  = 120.84 s
+ settle 常数 0.80 × 7 段                = 126.44 s   ← 现在的 total_duration_estimate_
实测 elapsed_time_s                      = 134.26 s
未建模额外时间                           =   7.82 s  (6.2%，平均每段 1.1 s)
```

额外时间的来源与处理：

| 来源 | 处理 |
| --- | --- |
| 起始接近腿（approach start） | 现在漏算，补进 `planned_total_s`；估计占 7.82 s 的大头，实现时单独量 |
| `ALIGN_SETTLE` 收敛（P 环收到 1°） | 实测标定 `align_converge_s` |
| `ALIGN_SETTLE` 保持 0.50 s | 常数，已含在 0.80 内 |
| `FINAL_APPROACH` 限速（最后 0.05 m 压到 0.08 m/s，0.625 s vs profile 的 ~0.35 s） | 由参数**精确算出**，约 +0.28 s/段 |
| 终点静止判据（0.08 → `stopped_linear_speed=0.01`） | 由参数**精确算出**，约 +0.28 s/段 |
| `goal_settle` 保持 0.30 s | 常数，已含在 0.80 内 |
| 段间握手往返 | 实测标定 `handshake_s` |
| `cross_slowdown` 触发（横轨 >3 cm 降速至 0.25×） | **异常路径，不计入点估计** |
| 中途重对准（航向 ≥12° 回 `ALIGN_BRAKE`） | **异常路径，不计入点估计** |
| ARC_ENTRY（上限 15 s） | **异常路径，不计入点估计** |

因此把 `DurationModel::settle_duration{0.80}` 这个黑盒常数拆开：

```cpp
double align_settle_s{0.50};    // 保持，常数
double align_converge_s{0.30};  // 收敛，实测标定
double final_approach_s{0.28};  // 由 final_approach_speed 精确算出
double goal_stop_s{0.58};       // 减速到静止 + goal_settle 保持
double handshake_s{0.10};       // 段间往返，实测标定
```

并**按段类型分别标定**：水平扫描线维持约 6° 重力前馈抬头、竖直线前馈为 0，
横轨收敛行为不同，不应共用一个常数。

异常路径不计入点估计的理由：正常任务横轨最大 3.6 mm、转向落点 0.8°，三者均不
触发；计入只会让预估系统性偏长。它们通过 `schedule_lag_s` 实时暴露，一旦触发
滞后量当场跳变，ETA 自动后推。

**澄清一处**：曾担心"横向调整航向角"是额外等待时间——实际不是。
`trackLine()` 返回的 `heading_error` 已含 `gravity_feedforward + cross_feedback`，
而 `ALIGN_PROFILE` 的转角就是拿这个 `heading_error` 去 `planTurn()` 的，
**对准阶段瞄的就是含重力前馈的航向**，不存在"转完 90° 再单独转 6°"。

ETA 的定义因此是动态的：

```
planned_total_s       = Σ(turn_profile + travel_profile + 固定开销) + 起始接近腿   # 任务开始时算定
estimated_remaining_s = Σ剩余段同上 + 当前段剩余 + 当前累计 lag                    # 每周期更新
schedule_lag_s        = 参考位置 − 实际位置，按当前参考速度换算成秒
```

标定目标：把 6.2% 的缺口压到 2% 以内。压不下去也不算失败——只要偏差稳定且方向
已知，`schedule_lag_s` 能实时补上，偏差的离散度即 ETA 的置信区间。

## 4. 实施阶段

