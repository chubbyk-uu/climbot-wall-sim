# climbot_control

C++ 轨迹控制、任务状态机和速度安全出口。覆盖任务的所有运动都出自本包。

## 节点

| 节点 | 职责 |
| --- | --- |
| `line_tracker_node` | 任意二维直线的沿轨、横轨和航向闭环，及左右轮联合限幅 |
| `coverage_manager_node` | 任务状态机与 `ExecuteCoverage` Action 客户端 |
| `cmd_vel_watchdog_node` | `/control/cmd_vel` → `/cmd_vel` 的唯一安全出口 |

## 启动

跟踪器加管理器，`coverage_mission.launch.py` 已经包含这一条：

```bash
ros2 launch climbot_control coverage_executor.launch.py
```

只启动跟踪器：

```bash
ros2 launch climbot_control line_tracker.launch.py
```

`line_tracker.launch.py` 从 `climbot_description` 注入轮距和轮缘硬限值，不在本包
复制机器人硬件属性。

## 纯函数库

- `turn_profile` / `travel_profile`：原地转向与直线段的梯形／三角形时间参数化曲线，
  两者对称；
- `segment_duration`：由上面两者推导每段耗时，用于进度权重和时间表；
- `schedule_estimate`：把逐段耗时汇总成剩余时间与 `schedule_lag_s`；
- `scan_entry`：点到直线的投影，给出沿轨距离与带符号的横轨偏差；
- `segment_arrival`：到点判据，用紧进／松出双阈值，避免一段被反复报完成又未完成；
- `coverage_execution`：任务合法性校验、平行扫描段与动态换道段的推导，以及横轨振荡监控；
- `command_watchdog`：看门狗的超时判定，与节点分离以便单测。

## 两条直线控制律

`tracking_mode` 参数在 `distance` 和 `time` 之间二选一，运行中也可以从 RViz 面板的
Algorithm 下拉框切换。`distance` 按剩余距离定速；`time` 按时间参数化曲线跟踪，因而能报告
`schedule_lag_s` 并在落后时以 `catch_up_max_linear_speed` 追赶。默认为 `time`，
正式基线 `2026-08-20` 用它，`2026-08-20d` 是同口径的 `distance` 对照。

## 时钟纪律

`include/climbot_control/control_clock.hpp` 决定控制环和安全兜底该用哪个时钟。
不启用仿真时间时，节点默认时钟退化成系统时钟——那是一个**可以被人改、也可以往回跳**的
时钟，定时器建在它上面，会在时钟回跳的那段时间里干脆不触发。所以：仿真时间激活时跟随
节点时钟，否则一律用单调时钟；消息时间戳则仍然用 ROS 时间。安全期限更进一步，一律用
`steady_clock` 配 `create_wall_timer`，不受任何时间源切换影响。

## 停止通路

`/control/hold` 是唯一一条不经过执行器的停止通路，由看门狗自己实现。它守在轮子前面的
最后一跳，ROS 图上其余部分是什么状态都不影响它。其余所有停止方式本质上都是“请正在
驱动的那一方停下来”，只有在对方还应答时才有效。

但 `/control/hold` 只是看门狗**进程内的易失状态，不是硬件急停**。实机的最后一道边界必须由
默认失效关闭的硬件级停机回路承担，仿真结论替代不了它。

## 暂停与恢复

`/coverage/pause` 与 `/coverage/resume` 让一次任务就地停住再继续，它们不是 Stop 的别名：
`task_id`、`revision`、当前段和归档 run 全程不变，Stop 依旧是取消本次任务。

管理器只是转发和回显。它把请求变成一次到执行器的 `/coverage/executor_pause`（`SetBool`），
状态则跟着执行器反馈走：`PAUSING`→`PAUSED`→`EXECUTING`。中间那层
`pause_response_timeout_s`（`2.0 s`）管的是“请求发出去了但没人应答”——那种情况下执行器可能
正在减速、也可能还在全速，管理器无从判断，因此按失联处理：请求 hold 并取消任务。执行器根本
没提供该服务则相反，那是一个明确的答复，任务原样继续，只把请求拒掉。

执行器这一侧，`PAUSED` 的判据是**命令和实测速度同时归零**，不是“发过停车指令了”。在此之前
状态是 `PAUSING`，`pause_stop_timeout_s`（`5.0 s`）只约束这一段减速；一旦进入 `PAUSED`，
想停多久停多久。恢复时，任务、段、转向和时间轨迹的计时基准一起平移暂停时长，所以暂停既不会
触发段超时，也不会凭空制造调度滞后。被平移的只有真正静止的那一段时间——减速阶段机器人还在
本任务下行走，那部分时间照算。

暂停期间除定位以外的监督全部冻结，这是有意的：段超时、调度和采集门衡量的都是“本来就不打算
取得的进展”。定位单独区别对待——减速途中判定是否停稳要读实测速度，读到的如果是过期数据就
什么也证明不了，所以那一段仍然照常做陈旧检查；停稳之后每个周期发的都是硬零，没有什么需要
它保护，而恢复前会重新检查一次，位姿过期时 `/coverage/resume` 直接拒绝，不会先答应再中止
任务。

## 采集存活监督

巡检 SCAN 期间，跟踪器要求 `/inspection/capture_gate` 持续到达。第一条心跳有一个
一次性的建立窗口 `capture_gate_start_timeout_s`（`2.0 s`）；之后每一条的期限都是
`capture_gate_timeout_s`（`0.50 s`）。一旦超期，跟踪器立即零速，并赶在
`segment_timeout_s` 触发之前中止这一段——而不是等整条线开完、封存时才发现归档是空的。

协议还会拒绝 `active=true` 的消息。那种消息意味着采集侧想给出一个位置栅栏来限住
机器人能开到哪里，而正常采图不允许调制扫描速度，所以这一版的 gate 只能当心跳用。

采集门跟着**机器人停没停**关，不跟着"暂停请求发出了没有"关。`PAUSING` 期间机器人还在走完整个
刹车距离（`cruise_speed²/(2·max_linear_deceleration)`，按当前配置是 `0.08 m`），这段路仍属于这
条扫描线，参考照发 `inspection_enabled=true`，落在这 8 厘米里的触发目标照常在正确位置曝光——
而且此时速度更低，运动模糊只会更小。只有进入 `PAUSED`（真正静止）参考才转 `false`，采集侧随之
停发心跳，采集门关闭。

反过来做是错的，而且是实机上跑出来的：在请求瞬间就撤掉参考，刹车距离内的目标全被跳过，恢复时
在停住的位置补开一枪，实测落在目标后方 `26.1 mm`，超过归档的 `25 mm` 纵向重叠容差，整个 run 被
判失败。跟踪器在暂停分支里提前返回，因此不会拿暂停这段时间去判采集门超时；恢复时这条等待重新
计时，采集节点在下一个控制周期就能把心跳补上。

## 边界

本包不得读取 Gazebo 真值、WheelSlip 或吸附参数，也不依赖 `climbot_gazebo`。
`config/control.yaml` 只放作业速度、控制增益、软件限幅和超时。

## 测试

```bash
colcon test --packages-select climbot_control
colcon test-result --verbose
```

接口字段见 [接口合同](../../docs/INTERFACES.md)，职责边界见
[系统架构](../../docs/ARCHITECTURE.md)。
