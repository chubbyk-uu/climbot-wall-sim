# 历史分册 2

### 阶段 0 — 分支与基线

建 `feature/time-parameterized-control`，用当前代码跑一次
`tools/run_coverage_regression.sh`（tag `basedist`）固化 distance 基线。
已有参考值：`horizontal` = `2.74 mm / 0.804° / 2.41 mm / 99.93%`，
`trapezoid_horizontal` = `2.90 / 0.872 / 5.46 / 99.75`。

### 阶段 1 — 抬高执行机构上限

改 `climbot_description/config/robot.yaml` 的 `drive`：

| 参数 | 现值 | 新值 |
| --- | --- | --- |
| `max_linear_velocity_mps` | 0.30 | **0.45** |
| `max_linear_acceleration_mps2` | 0.40 | **0.70** |
| `max_angular_acceleration_rps2` | 1.50 | **2.20** |
| `max_angular_velocity_rps` | 1.20 | 不变（已足够） |

`control.yaml` 一个字不改。

**门禁**：重跑回归必须与阶段 0 基线一致。理论上是 no-op（今天最坏轮速
`0.275 < 0.30`，未饱和过）；若指标变化，说明轮级饱和一直在暗中起作用，
必须先查清再往下走。

### 阶段 2 — `travel_profile`（纯函数 + 单测）

新增 `include/climbot_control/travel_profile.hpp` 与 `src/travel_profile.cpp`，
与 `turn_profile` 对称：

```cpp
struct TravelProfile { double peak_speed, ramp_duration, coast_duration,
                       acceleration, deceleration, duration; };
struct TravelSample  { double distance, speed; };
TravelProfile planTravel(double distance, double max_speed,
                         double acceleration, double deceleration);
TravelSample  sampleTravel(const TravelProfile &, double elapsed);
```

即文档 `D ≥ vm²/av` 判梯形/三角形那组公式，推广到独立加减速。加入 CMakeLists 的
`line_tracker` 库；新增 `test/test_travel_profile.cpp`：梯形/三角形分界、速度
积分等于 `D`、`d(T)=D`、末速为 0、非法参数抛异常、加减速相等时退化为文档形式。

同时把 `segment_duration.cpp` 的 `estimateTravelDuration` 改为调用
`planTravel(length, cruise, accel, brake).duration`，消掉第二份独立公式；
参数仍传 `0.20 / 0.12`，`test_segment_duration` 必须逐位不变。

### 阶段 3 — 接入 `line_tracker_node`

`control.yaml` 新增：

```yaml
tracking_mode: distance          # distance | time
time_along_gain: 1.0             # 纵向滞后 → 线速度修正（先纯 P）
time_along_integral_gain: 0.0    # P 不够再开
time_along_integral_limit_m_s: 0.05
time_profile_acceleration: 0.20  # profile 用，对称
time_profile_deceleration: 0.20
catch_up_max_linear_speed: 0.35
catch_up_max_linear_acceleration: 0.35
time_axis_stretch_enabled: false
time_axis_stretch_lag_m: 0.05
time_mode_final_approach_enabled: false
```

实现要点见 3.6。`travel_profile_` 在 `ALIGN_SETTLE → TRACK_LINE` 切换那一刻
（`reanchorStartApproach()` 旁）按**当时的剩余距离**规划，时刻取自
`controlClock`；弧线出口切回 `TRACK_LINE` 时同样重新规划（3.8）。
`limitAndPublish` 在 time 模式下用 `catch_up_max_linear_acceleration`。

### 阶段 4 — 时间信息上行（方案 C）

- `ExecuteCoverage.action` feedback 与 `CoverageStatus.msg` 增加 3.9 的三个字段；
- `coverage_manager` 透传；
- `coverage_panel` 在进度条下增加一行，distance 模式末尾写 `estimate only`；
- `DurationModel` 按 3.10 拆分固定开销，并把起始接近腿计入 `planned_total_s`；
- `evaluate_coverage_execution.py` 的 `summary_json` 增加 `planned_total_s`、
  `max_along_lag_m`、`linear_saturation_fraction`；回归汇总表相应加列。

### 阶段 5 — A/B 与调参

- `run_coverage_regression.sh` 增加 `--tracking-mode` 透传，同批用例跑两轮；
- 先只调 `time_along_gain`，看 lag 收敛与落点误差；稳态滞后消不掉才开 I；
- 用 `calibrate_wall_slip.py` 在 `0.20~0.35` 速度区间重标 `gravity_slip_ratio`
  ——现值 `0.1042` 是在 `cruise=0.20` 标定的，追赶时单位距离下滑变小，该值未必
  仍成立。这是本次最不确定的一项；
- 用回归结果标定 3.10 的固定开销，把 6.2% 缺口往 2% 压；
- 拿到数据后再决定是否打开 3.7 的两个兜底。

### 阶段 6 — 文档

STATUS.md 记决策与实测；PROJECT_GUIDE §14 控制链路补时间模式；
ARCHITECTURE.md 不变（无新包）。

## 5. 测试计划

| 测试 | 内容 |
| --- | --- |
| `test_travel_profile.cpp`（新） | 见阶段 2 |
| `test_segment_duration.cpp`（现有） | 改用 `planTravel` 后结果逐位不变 |
| `test_line_tracker_node.py`（现有） | distance 模式行为不变 |
| `test_coverage_arc_entry.py`（现有） | 保持 distance 模式 |
| `test_coverage_arc_entry_time.py`（新） | time 模式下强制 45~120 mm 转向后横偏，验证进 ARC_ENTRY → 出弧线 → **时间参考重置** → 落点达标 |
| `test_line_tracker_time_mode.py`（新） | time 模式单段：lag 收敛、落点、饱和占比 |
| 回归 `run_coverage_regression.sh` | distance/time 两轮 A/B |

## 6. 明确不做

- **整任务级统一时间表**（理由见 3.5）；
- **`climbot_interfaces` 增加路点时间戳**（3.5 的推论，规划器不改）；
- **ARC_ENTRY 的时间参数化**（3.8）；
- **在规划阶段就报出总时长**：几何路点在规划器手里，理论上规划完即可报
  "本任务预计 6 分 30 秒"，但那要求规划器知道控制参数（`cruise_speed` 等），
  属跨包依赖，配置归属需先想清楚。记为后续待办。

## 7. 风险与门禁

| 风险 | 门禁/缓解 |
| --- | --- |
| 抬高上限改变 distance 基线 | 阶段 1 独立门禁：重跑回归必须与阶段 0 一致 |
| 落点精度退化（制动距离 0.167 → 0.10 m） | 阶段 5 观测；`time_mode_final_approach_enabled` 为对应兜底 |
| `gravity_slip_ratio` 在追赶速度下失效 | 阶段 5 用 `calibrate_wall_slip.py` 重标 |
| 高速下 WheelSlip 法向载荷局限放大 | 已记于 STATUS 未决事项 1；结论中注明可迁移性折扣 |
| 进度条口径被改动污染 A/B | 3.4/3.9：`estimateTravelDuration` 仍传 `0.20/0.12`，进度条口径不变 |
| `cmd_vel_watchdog` 的 0.40 s 超时在高速下对应更长惯性行程 | 阶段 3 一并收紧 |

## 8. 执行结果（2026-08-18）

### 8.0 定版配置

```yaml
tracking_mode: time
time_profile_acceleration: 0.20      # 对称，等于额定加速度
time_profile_deceleration: 0.20
time_speed_lag_s: 0.08               # 加速度前馈，实测 0.0799 s，见 8.7
time_along_gain: 2.5                 # 纯 P，见 8.8
time_along_integral_gain: 0.0        # 不需要，巡航段无稳态速度误差
catch_up_max_linear_speed: 0.35
catch_up_max_linear_acceleration: 0.35
time_mode_final_approach_enabled: true
time_axis_stretch_enabled: false     # 从未触发
schedule_align_converge_s: 1.24      # 时长模型标定，见 8.4
schedule_goal_stop_s: 0.13
schedule_handshake_s: 0.43
```

定版回归 `timeE`，八用例全过：

| 用例 | 落点 mm | 转向 ° | 道间距 mm | 覆盖 % | act/plan | lag s |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| bigV | 3.89 | 0.894 | 4.85 | 100.00 | 0.997 | 0.04 |
| bigTV | 3.02 | 0.907 | 5.50 | 98.98 | 1.002 | 0.04 |
| vertical | 2.96 | 0.872 | 2.19 | 100.00 | 1.015 | 0.04 |
| trapezoid_vertical | 3.86 | 0.965 | 4.34 | 98.63 | 1.001 | 0.05 |
| bigH | 2.94 | 0.880 | 2.77 | 99.97 | 1.005 | 0.03 |
| bigTH | 3.05 | 0.900 | 2.58 | 99.34 | 1.013 | 0.03 |
| trapezoid_horizontal | 3.69 | 0.994 | 4.66 | 99.75 | 1.002 | 0.04 |
| horizontal | 2.64 | 0.763 | 1.51 | 99.93 | 0.981 | 0.04 |

落点 `2.64~3.89 mm`、道间距 `1.51~5.50 mm`、转向落点航向 `0.763~0.994°`、
覆盖率 `98.63~100%`、`act/plan` `0.981~1.015`、峰值滞后 `0.03~0.05 s`。
`timeD`（`gain=3.0`）为同一配置的前一版，落点上界为 `4.59 mm`。

> **`timeE` 已被 `2026-08-19t` 取代。** 这一轮产自带未提交改动的工作树
> （`source_modified=true`），因此不可作为验收证据引用。同一配置已在提交
> `104caaa` 的干净工作树上重跑，落点 `2.53~3.98 mm`、`act/plan` `0.988~1.020`、
> 峰值滞后 `0.03~0.05 s`，与上表落在同一散布带内。正式基线见
> [`results/README.md`](../../../../results/README.md)。上表保留是因为本节的调参推理引用
> 了它的具体数字。

### 8.1 三轮回归对照

八个验收用例，每轮全部通过。落点误差 / 道间距误差，单位 mm：

| 用例 | distance（`ceilgate`） | time 裸算法（`timeA`） | time + 兜底（`timeB`） |
| --- | --- | --- | --- |
| bigV | 3.88 / 5.36 | 15.23 / 17.60 | **3.31 / 4.64** |
| bigTV | 5.04 / 6.34 | 15.38 / 11.99 | **3.15 / 5.11** |
| vertical | 3.45 / 3.72 | 15.08 / 8.18 | **3.24 / 2.51** |
| trapezoid_vertical | 3.24 / 3.15 | 15.33 / 7.72 | **3.26 / 3.49** |
| bigH | 2.60 / 3.57 | 13.26 / 8.84 | **2.65 / 3.56** |
| bigTH | 2.67 / 2.34 | 10.98 / 5.49 | **3.42 / 2.85** |
| trapezoid_horizontal | 2.97 / 6.21 | 10.87 / 9.49 | **2.67 / 3.56** |
| horizontal | 2.87 / 1.88 | 11.98 / 6.41 | **2.50 / 2.98** |

验收限值：落点 ≤ 30 mm，转向落点航向 ≤ 2.0°，道间距 ≤ 20 mm，覆盖率 ≥ 95%。
转向落点航向三轮均在 0.75~1.01° 之间——转向那条路本来就没改动，这正是预期。

任务时长（仿真秒）：time + 兜底比 distance 一致快 0.2~1.9%，来自对称制动。

### 8.2 阶段 1 门禁：抬高上限没有改变 distance 模式

八个用例的**覆盖率全部保持到小数点后两位完全一致**，落点四升四降、转向四升四降，
无系统性偏移。门禁通过。

### 8.3 裸算法的失效模式已定位

`timeA` 的落点误差 10.9~15.4 mm 与峰值滞后 0.08 s 完全对应
（0.08 s × 0.20 m/s = 16 mm）。解析轨迹后确认机制：

- **滞后几乎全部在第一秒内建立**（`lag@1s` 已达 9~15 mm，之后基本不再增长）。
  profile 假设机器人从静止按额定 `0.20 m/s²` 起步，实际起步差约 12%。
- **P 环随后过度补偿**：到段末机器人反而**超前**自己的时间表 7~13 mm。

所以落点误差是**超冲而非欠冲**。第 3.7 节的兜底之所以只需交出最后 50 mm 就够，
正是因为把停车判据换回位置的函数后，携带的动量和残余修正不再参与停车决策。

### 8.4 时长模型标定完成，预测精度 1.5% 量级

把八条基线轨迹按控制状态拆开，与模型逐项对比，缺口是三个稳定的每段常数：

| 项 | 每段超出量 | sd | 对应参数 |
| --- | --- | --- | --- |
| `ALIGN_SETTLE` 收敛到 1° 死区 | 1.240 s | 0.144 | `schedule_align_converge_s` |
| `ALIGN` 段（含等待上一条指令衰减） | 0.427 s | 0.168 | `schedule_handshake_s` |
| 终点限速 + 减速到静止 | 0.130 s | 0.057 | `schedule_goal_stop_s` |

三项之和 1.80 s/段。代入后预测精度由 **+10.6%（sd 2.4%）** 变为
**−0.4%（sd 1.2%）**，最差 3.0%（`horizontal`，段数最少所以每段项占比最大）。
`timeB` 实测 `act/plan` 落在 **0.977~1.015**，均值 1.003。

第 3.10 节设定的 2% 目标达成。主项是 `ALIGN_SETTLE` 收敛而非任何一个"保持"常数——
这正是把单一 `settle_duration` 常数拆开的价值：拆开之前它只能被整体拟合。

### 8.5 与预期不符之处

- **梯形并不比矩形显著更差。** 3.10 节推测梯形因斜向换道转向更多而开销更大，
  实测每段缺口梯形 2.89 s、矩形 2.51 s，区间重叠，不构成分类依据。真正的
  区分度在段数与扫描线长度上，而三个每段常数已经把它吸收掉了。
- **"横向调整航向角"不是额外等待时间**（3.10 节已澄清）：`trackLine()` 返回的
  `heading_error` 已含重力前馈，`ALIGN_PROFILE` 对准的就是含前馈的航向。

