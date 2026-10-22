# 历史分册 4

### 操作在各状态下的行为

| 状态 | Start | Cancel | Replan / Clear points / Region / Sweep / Algorithm |
| --- | --- | --- | --- |
| `IDLE` | 拒绝，无有效任务 | 拒绝，无执行中任务 | 允许；`Replan` 在点选模式下需先选够点 |
| `INVALID` | 拒绝，无有效任务 | 拒绝 | 同上 |
| `READY` | **接受**，发送 Goal | 拒绝 | 允许，重新生成预览 |
| `STARTING` | 拒绝，已有任务在启动 | 等待 hold 释放时**接受**，丢弃未发送 Goal 后进入恢复锁；已发送 Goal 的正常应答期限内拒绝 | **面板置灰**；服务层仍会受理，只改预览 |
| `EXECUTING` | 拒绝 | **接受**，请求取消 | **面板置灰**；`tracking_mode` 由执行器直接拒绝 |
| `STOPPING` | 拒绝，仍在停机 | **接受**，重试速度保持与取消 | **面板置灰** |
| `RECOVERY_LOCKED` | 拒绝，外部停车尚未确认 | 拒绝，无受监督 Goal handle | **面板置灰** |
| `FINISHED` | **接受**（若仍有缓存任务），重跑该任务 | 拒绝 | 允许 |

任务运行期间面板冻结全部五个规划控件，只留 Cancel。它们发出的请求确实只改预览、
不动运行中的 Goal，但**预览就是画在机器人身上的那条轨迹**，运行中改它看起来就像
任务被换掉了；换形状还会直接把它撤掉。置灰取自管理器自己发布的 `can_cancel`，
面板不另立一套"是否在运行"的判断。

`tracking_mode` 是唯一一个在**服务层**也拒绝的：执行器只在没有任务运行时接受它。

执行中若仍有客户端绕过面板送来新预览，只更新缓存，不改变 `state`，也不改变
`task_id`/`revision`——它们始终标识**当前 `state` 所描述的那个任务**。否则运行中的
机器人会被报成 `Ready` 或 `Idle`，取消按钮随之消失。执行器在接受 Goal 时已复制
任务，运行中的任务本身不受预览影响。

规划失败与"未选择区域"在管理器看来都是空任务，它无法区分，因此都报
`IDLE: no coverage region selected.`。真正的原因只在规划器的 `/coverage/status`
上，面板因此单独显示该话题。

已接受的 Goal 只由结果回调结束，而执行器崩溃时结果永远不会到达。管理器以
`executor_timeout_s`（默认 `5.0 s`）监视 Action 服务的存在：持续消失超过该时间即
进入 `STOPPING`。

`STOPPING` 存在的原因是「Action 服务不可发现」和「机器人已经停下」是两件不同的事。
执行器真的死了会同时给出这两件；而 DDS 发现抖动、Action 通道故障或取消请求没送达
只给出第一件——执行器还活着、`/control/cmd_vel` 仍在刷新、速度看门狗没有超时可
触发、机器人继续在走。此前管理器在这里直接报 `FINISHED` 并释放 Goal handle，而
`can_cancel` 由该 handle 决定，于是操作员的停止入口恰好在最需要它的时刻消失。

进入 `STOPPING` 后管理器做三件事：调用 `/control/hold` 施加速度保持、在 Goal handle
可用时发取消、保持 `can_cancel=true`（此时按停止会重试保持和取消）。保持请求带响应
期限；无应答会重发，`hold_active=false`（包括看门狗重启）也会重新施加。
离开 `STOPPING` 只接受**关于任务和运动**的证据，二者取一：

1. `/control/cmd_vel` 上连续 `command_quiet_s`（默认 `1.0 s`）没有非零指令；
2. 执行器最终应答了，此时用它返回的真实结果而不是 `EXECUTOR_LOST`。

`hold_active=true` 本身不再结束 `STOPPING`：上游仍可能持续发运动指令，看门狗一旦
重启就会恢复运动。两个退出条件都不成立就停在 `STOPPING` 不动——这正是这个状态
存在的目的。`EXECUTOR_LOST`
只由管理器发出，执行器自己永远不会返回它；先前这里复用 `CONTROL_TIMEOUT`，报的是
一个并未发生的原因。实测从 `SIGKILL` 到释放约 `25 s`，其中约 `20 s` 是 DDS 摘除已死
参与者所需的时间，正常退出会快得多。

Start 应答未知时，当前静默不能证明请求以后不会被接受，因此这条 `STOPPING` 默认没有
自动超时逃生口。`/coverage/force_abandon` 是明确的人工风险边界，只在这一分支可用：
调用后退休未知请求的代次并进入 `RECOVERY_LOCKED`，持续请求 `/control/hold=true`，
`can_start=false`。它不发布 `FINISHED`，也不声称任务已停止。操作员确认硬件急停、驱动
失能或执行器进程确已终止后调用 `/coverage/rearm`，管理器才恢复 `READY`；hold 本身仍
保持到下一次 Start 显式请求并确认释放。若被放弃的请求后来竟返回“已接受”，管理器会
立即重新进入 `RECOVERY_LOCKED`、施加 hold，并取消旧 Goal 和当时任何新 Goal。

RViz 的 Force abandon 必须在 `5 s` 内点击两次；第一次只显示上述风险说明。Rearm 是
独立按钮，按钮文字要求完成外部确认。命令行可以直接调用服务，因此使用者承担同一物理
确认责任，不能把脚本调用当成自动故障恢复。

`message` 与日志共用一份措辞，命令行观察等价于原来的 `std_msgs/String`：

```bash
ros2 topic echo /coverage/manager_status --field message
```

状态转换由管理器发布，执行器反馈按 `feedback_publish_period_s`（默认 `0.2 s`）
限频转发。反馈本身以控制环频率到达，任何显示都不需要那个速率，日志也不应承载它。

`progress` 按每段的**预计耗时**加权，而不是按段数或路程：

```text
progress = (已完成各段预计耗时 + 当前段已完成部分) / 全任务预计耗时
```

每段预计耗时 = 原地转向时间（复用控制器自己的梯形角速度曲线 `planTurn`）+ 稳定
时间 + 直线行驶时间（加速、巡航、减速三段）。这些量全部来自已有控制参数，没有
另行标定的常数。段内插值同样分两段：转向部分按对准曲线的实际进度推进，直线部分
按沿轨距离推进。

按段数等权会让 `0.44 m` 的换道段和 `4.5 m` 的扫描段各占 `1/N`，实测进度条在换道
段的推进速率是扫描段的 `3.2` 倍。按路程加权则反过来——换道段的时间大半花在原地
转向上，路程几乎为零，进度条会停住。改为耗时加权后实测两者速率比为 `1.16`。

`/coverage/start` 不会使规划器一发布任务就自动运动。管理器拒绝空任务、已有任务
执行中或 Action 服务不可用的开始请求，并在响应和 `/coverage/manager_status` 中给出
锁定的任务版本。管理器复制该任务作为不可变 Goal，因此预览更新不会改写执行中的
任务；定位可用性和边界安全性仍由执行器在实际动作前持续检查。后续 RViz 面板调用
同一组接口，不绕过管理器直接控制机器人。

发出 Goal 后管理器进入"等待执行器应答"状态并拒绝新的开始请求。若执行器在应答前
崩溃或被挂起，该应答永不到达，因此这个状态带 `start_response_timeout_s`（默认
`5.0 s`）超时：超时后管理器进入 `STOPPING`、施加速度保持，并且继续等待该请求的
最终应答，不接受新的开始。应答若为拒绝即可安全回到可启动状态；若为迟到接受，管理器
保存其 handle、发取消，并持续监督到结果或运动指令静默。操作员在 handle 尚未返回时
按停止也会成功进入同一流程，而不是因为"还不能发 Action cancel"而丢失停止入口。

超时**并不撤回已经发出的请求**，因此每次开始都带一个单调代次，三个回调
（应答、反馈、结果）都校验自己所属的代次。代次只在请求被拒绝、任务取得最终结果或
明确完成停机后退休；不能仅因应答超时就作废，否则迟到接受会成为无人监督的任务。
新的 Start 在旧代次退休前一律拒绝。即便未来替换成允许并发 Goal 的 Action 服务，旧
回调也不能改写新任务状态或撤掉新任务的停止入口。

只要速度看门狗服务存在，每次新 Start 都先进入 `STARTING` 并显式请求解除保持；只有
`/control/hold` 成功应答或 `/control/hold_active=false` 确认后才真正发送 Action Goal，
避免旧锁存状态或消息到达竞态让 Goal 已开始而轮子仍被 hold 锁住。

这段“已排队、尚未发送 Goal”的 `STARTING` 同样有 `start_response_timeout_s` 上限。解除
hold 超时、执行器在解除后消失，或操作员按 Cancel 时，管理器丢弃队列 Goal、以新代次
重施 `hold=true`，并进入 `RECOVERY_LOCKED`。因为解除请求可能已生效但应答丢失，它不能
直接报 READY；操作员确认停车后才可 Rearm。此时 Force abandon 不适用：它只处理已经发出、
但 Action 接受结果未知的 Goal。

### 运行时构型

`region_type` 和 `sweep_direction` 可以在运行中改,不必重启 launch。二者由
`/coverage/configure`（`climbot_interfaces/srv/ConfigureCoverage`）**一次原子设定**,
现行值发布在 latched 的 `/coverage/config`（`CoverageConfig`）上。

刻意不做成"先 `ros2 param set` 两个参数、再调 `/coverage/replan`":那样在两步之间
存在半配置状态,调用方中途死掉会把规划器留在那里,第二个客户端也能插进来。

请求里**留空的字段表示不改动**,所以只改扫描方向的客户端不必重述它以为的形状,
也就不会覆盖别人刚做的修改。响应带回**实际生效**的构型（接受与否都带）,调用方
据此立即同步显示,不必等话题。

| `CoverageConfig` 字段 | 含义 |
| --- | --- |
| `region_type` / `sweep_direction` | 现行构型 |
| `input_mode` | `rviz` 或 `parameters`;后者下点数无意义 |
| `required_points` / `selected_points` | 该形状需要的点数与当前已选点数 |
| `can_plan` | 现在重新规划会不会被接受 |
| `message` | `can_plan` 为假时的原因 |

`CoverageStatus` 同步转发执行器反馈里的 `planned_total_s`、`schedule_lag_s` 和
`estimated_remaining_s`，定义以 `ExecuteCoverage.action` 为准。

**换形状会撤掉当前预览。** 任务被清空、标记线被删除,要按 `Replan` 用现有点重建,
或清点重选。下拉框是在问"这些点是什么形状",不是在下令规划——梯形 3 点切回矩形时
它曾经悄悄按前 2 点画出一条谁也没要求过的新轨迹。换扫描方向不撤预览:它不改变
区域本身,只是换个方向画同一块地。

**换形状不丢点。** A、B 两点在矩形和梯形里是同一个角,所以矩形选好 2 点后切到梯形
只是等第 3 点。丢点会让下拉框上的一次误点变成不可逆操作。

`can_plan` 是**提示不是保证**:请求仍可能被拒,原因在服务响应里。判定只写在规划器的
`planBlockedReason()` 一处,由 Replan 守卫、`configure` 响应和面板置灰共用,三者不可能
互相矛盾。

### 参数

| 参数 | 可选值/单位 | 来源或作用 |
| --- | --- | --- |
| `frame_id` | frame 名称 | 默认 `odom` |
| `input_mode` | `parameters` / `rviz` | 区域输入方式 |
| `region_type` | `rectangle` / `trapezoid` | 矩形或等腰梯形 |
| `lower_left` | `[x, y] m` | A：左下角 |
| `upper_right` | `[x, y] m` | B：右上角 |
| `lower_right` | `[x, y] m` | C：梯形右下角 |
| `sweep_direction` | `horizontal` / `vertical` | 弓字扫描方向 |
| `start_corner` | `lower_left` / `lower_right` / `upper_left` / `upper_right` | 起始角点 |
| `detection_width` | m | 检测有效宽度 |
| `detection_length` | m | 沿行进方向的检测有效长度；G2 标称 `0.28125`，旧回归配置保留 `0.01` |
| `detection_forward_offset` | m | 检测中心相对 `base_link` 的前向偏移；巡检相机为 `0.340` |
| `detection_edge_overlap` | m | 兼容保留参数；可走区优先语义下不改变蓝线或真实检测足迹，当前不参与覆盖面积计算 |
| `overlap_ratio` | `[0, 1)` | 相邻扫描带的**横向**重叠率；默认 `0.20`，独立于拍照触发重叠 |
| `robot_length`、`robot_width` | m | launch 从 `robot.yaml` 注入 |
| `edge_clearance` | m | launch 从 `robot.yaml` 注入 |
| `wall_width`、`wall_height` | m | launch 从 `wall.yaml` 注入 |
| `path_height` | m | RViz 路径离墙显示高度 |
| `bottom_warning_tolerance` | m | 梯形底边点高度修正提示阈值 |
| `minimum_nominal_coverage_ratio` | `[0, 1]` | 规划期可选覆盖率门限；默认 `0` 只报告、不因覆盖不足拒绝安全路径 |
| `top_edge_scan` | `auto` / `always` / `never` | 顶部收边扫描，默认 `never`；`auto` 需配置正覆盖率门限 |

规划器内部计算：

```text
row_spacing = detection_width × (1 - overlap_ratio)
safety_margin = 0.5 × hypot(robot_length, robot_width) + edge_clearance
```

`inspection_geometry_profile` 是 `coverage_planner.launch.py` 的 launch 参数，不写入
`CoverageTask`：`calibrated`（默认）从共享相机描述注入物理足迹和前向偏移；`configured`
严格采用 planner YAML，用于复现冻结的历史回归。无论 profile 如何，`overlap_ratio` 都只
来自任务 YAML；纵向照片重叠由巡检节点的 `image_overlap_ratio` 独立决定。

矩形点选顺序为 A（左下）、B（右上）；等腰梯形为 A（左下）、B（右上）、
C（右下）。A、C 的高度取平均值修正为水平底边。

由于消息字段需要兼容现有 Action 和归档，`coverage_region` 现承载用户选择的机器人任务
可走区；`motion_region` 承载整面墙的绝对安全运动区。相机实际覆盖不是单个 Polygon，
而是从 `SCAN` 与检测足迹推导的扫掠带并集。以下参数和输出已经进入规划／执行接口：

| 项目 | 含义 |
| --- | --- |
| `detection_length` | 沿行进方向的检测有效长度；与 `detection_width` 共同形成二维检测足迹 |
| `turn_clearance` | 竖向底部转向相对绝对安全边界的安全余量，初值 `0.06 m` |
| `/control/reference_path` | 转向后根据 EKF 实际位置生成的动态直线执行参考，仅供显示和评价 |

竖向为主时，控制器以第一次转向后的实际位置为斜直线 `TRANSITION` 起点，第二次
转向后按统一入轨判据冻结与名义线平行的 `SCAN`，不得逐列倒车返回名义起点。覆盖率
必须按二维检测足迹重新计算并报告；只有合同门限或操作员显式要求时才增加顶部水平收边扫描。

