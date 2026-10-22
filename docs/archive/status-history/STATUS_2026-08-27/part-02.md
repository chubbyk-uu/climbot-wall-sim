# 历史分册 2

## 2026-08-15 结构化管理器状态（阶段 2）

§11.1 要求操作面板显示任务版本、执行进度和错误，但管理器的 `SendGoalOptions`
只注册了 `goal_response_callback` 和 `result_callback`，**没有 `feedback_callback`**，
执行进度根本没有数据源；`/coverage/manager_status` 又是无结构的
`std_msgs/String`，界面只能靠解析日志文本还原状态。

| 项目 | 状态 | 结果 |
| --- | --- | --- |
| `CoverageStatus.msg` | 已完成 | 新增消息：`state` 六态枚举（`IDLE`/`INVALID`/`READY`/`STARTING`/`EXECUTING`/`FINISHED`）、`task_id`、`revision`、`current_segment`、`total_segments`、`progress`、`executor_state`、`result_code`、`message`。`executor_state` 与 `result_code` 的取值以 `ExecuteCoverage.action` 为权威定义，消息里只做镜像说明，不复制成第二份真相 |
| 话题类型替换而非并存 | 已完成 | `/coverage/manager_status` 直接换成 `CoverageStatus`，没有保留并行的 String 话题。两个话题携带同一信息迟早会互相矛盾；人类可读那一行作为 `message` 字段保留，`ros2 topic echo --field message` 与原来等价 |
| 执行进度 | 已完成 | 管理器注册 `feedback_callback`，把执行器上报的 `current_segment`、`progress`、`state` 并入状态发布。反馈以控制环频率到达，按 `feedback_publish_period_s`（默认 `0.2 s`）限频转发，且不写日志；状态跃迁始终立即发布 |
| `STARTING` 与应答的竞态 | 已完成 | `STARTING` 在 `async_send_goal` **之前**发布。反过来写的话，单线程执行器虽然不会真的乱序，但一旦换成多线程执行器，已被接受的 Goal 就可能被这一行覆盖成"仍在启动"。顺序调整后该问题不依赖执行器类型 |
| 发布器与订阅器创建顺序 | 已完成 | 状态发布器移到任务订阅之前创建，任何预览都不可能在有地方汇报之前被处理 |
| 进度回归护栏 | 已完成 | 新增 `test_publishes_executor_progress`：断言执行中出现非 `WAITING` 的 `executor_state` 且 `progress` 在 `[0, 1]`。移除 `feedback_callback` 即失败（`The executor never reported a motion state.`）。另外两个用例改为断言结构化字段，并按 `message` 文本匹配，使三个用例不依赖执行顺序 |

面板由此只需订阅一个话题：任务锁定、版本检查和安全状态转换全部留在管理器，
符合 §11.1 对界面插件的限制。

## 2026-08-15 RViz 操作面板（阶段 3）

新建 `climbot_rviz_plugins`，实现 §11.1 要求的面板。放在独立包而不是
`climbot_control`：面板引入 Qt5 与 pluginlib，控制包应保持无界面依赖。同类需求
的既有做法一致——Nav2 的 `nav2_rviz_plugins` 提供四个 `rviz_common::Panel`
（含带启动/暂停/取消按钮的 `Nav2Panel`），MoveIt 的 MotionPlanning 面板同理；
与三维场景无关的工具才走 rqt。

| 项目 | 状态 | 结果 |
| --- | --- | --- |
| `CoveragePanel` | 已完成 | `rviz_common::Panel` 子类，四个按钮（Replan / Clear points / Start / Cancel·Stop）加 State、Task+revision、Segment、进度条、管理器消息和最近一次请求结果 |
| 面板不持有状态 | 已完成 | 只订阅 `/coverage/manager_status` 并调用四个 Trigger 服务。按钮置灰仅依据已发布的 `state`，是提示不是校验；非法请求仍由管理器拒绝，原因显示在 Last request 一行。任务锁定、版本检查和安全状态转换全部不在插件里 |
| 线程安全 | 已完成 | RViz 在执行器线程派发订阅与服务回调，Qt 控件只能在 GUI 线程访问。回调只写入互斥量保护的成员，控件由 `100 ms` 的 `QTimer` 在 GUI 线程统一刷新 |
| 随 launch 自动出现 | 已完成 | 写入 `climbot_coverage/rviz/coverage.rviz`，`coverage_mission.launch.py` 启动即带面板；`climbot_coverage` 相应补上 `climbot_rviz_plugins` 依赖 |
| 插件加载回归护栏 | 已完成 | 构建成功不能说明面板可用：插件加载失败时 RViz 只警告一次然后照常启动，面板是消失而不是报错。`test_coverage_panel_plugin` 经 ament 索引按 RViz 的方式加载该类；把 `plugins_description.xml` 的库名改错即失败 |
| 配置与声明交叉检查 | 已完成 | `climbot_coverage` 新增 `test_rviz_config`：断言 `coverage.rviz` 里配置的 `climbot_*` 面板类名确实由 `climbot_rviz_plugins` 声明，防止改名后静默失效 |

实机验证：在 WSLg 显示下启动 RViz 加载该配置，日志无插件加载警告；
`ros2 topic info /coverage/manager_status -v` 显示 `rviz` 节点已按正确类型订阅，
`ros2 node info /rviz` 列出四个 Trigger 服务客户端，即 `onInitialize()` 已执行且
接线正确。**面板的实际视觉布局和按钮点击行为未经自动化验证**，需人工确认。
（布局部分已由 2026-08-17 一节闭合，点击行为已于 2026-08-18 人工确认。）

全量 261 项测试通过，19 项按环境跳过。

## 2026-08-15 起点进入被计入任务进度

面板上线后发现：任务刚开始、机器人还在驶向首点时进度条就开始上涨，真正进入
第一段时又跌回 `0`。

根因在 `publishFeedback()`。进度按 `(completed_segments + 当前段已走比例) / 总段数`
计算，而 `command.along` 是相对**当前执行直线**的。起点进入段有自己的直线，
`completed_segments_` 仍为 `0`，于是进入段走完一整条线就把进度推到 `1/N`；第一段
配置自己的直线后 `along` 归零，进度随之跌回 `0`。起点进入分跑道段和最后一段两条
直线，因此这个上涨-归零会出现两次。

| 项目 | 状态 | 结果 |
| --- | --- | --- |
| 进入段进度 | 已完成 | 进入首段之前 `progress` 固定为 `0`。进入段不是任务段，走完它并不代表任务完成了任何比例；`current_segment` 本来就已经是 `-1`，面板据此显示 `approach of N` |
| 回归护栏 | 已完成 | 新增 `test_coverage_progress.py`：首点设在 `1.0 m` 外触发起点进入，用合成里程计沿进入直线推进，断言 `current_segment < 0` 的状态里 `progress` 恒为 `0`。修复前实测该处最高报出 `0.912`，即 `91.2%` |

## 2026-08-15 进度按预计耗时加权

进度条在扫描段涨得慢、换道段涨得快。原因是 `progress` 按**段数**等权：
`(已完成段数 + 当前段已走比例) / 总段数`，于是 `0.44 m` 的换道段和 `4.5 m` 的
扫描段各占 `1/N`。用已归档基线量化：换道段耗时 `9.4 s`、扫描段 `19～24 s`，
进度条在换道段的推进速率是扫描段的 `3.2` 倍。

按路程加权并不能解决问题，反而更糟：换道段的时间大半花在原地转向上，路程几乎
为零，进度条会在换道段停住（估算速率比约 `0.27`）。

| 项目 | 状态 | 结果 |
| --- | --- | --- |
| 耗时模型 | 已完成 | 新增 `climbot_control::estimateSegmentDuration`：原地转向时间复用控制器自己的梯形角速度曲线 `planTurn`，加稳定时间，加加速/巡航/减速三段直线行驶时间。全部输入都是已有控制参数，没有另行标定的常数。对两份基线的六个采样点，预测与实测偏差在 `20%` 以内（多数 `5~8%`） |
| 加权与段内插值 | 已完成 | 进度改为 `(已完成各段预计耗时 + 当前段已完成部分) / 全任务预计耗时`。段内同样分两段插值：转向部分按对准曲线的实际进度推进，直线部分按沿轨距离推进，因此转向期间进度条也在走而不是停住再跳 |
| 实测效果 | 已完成 | 同一横向矩形任务重跑：扫描段推进速率 `1.02～1.03`、换道段 `0.89`（`1.00` 为完全匀速），最差比值由 `3.16` 降到 `1.16` |
| 单元测试 | 已完成 | `test_segment_duration.cpp` 六个用例：对基线实测值的拟合精度、换道段与扫描段的权重必须拉开、零长度段仍计转向时间、过短段不得按巡航速度估算、单调性、非有限输入不污染进度 |

## 2026-08-15 面板操作在各状态下的行为

按钮做出来之后逐一核对"每个按钮在每个状态下按会怎样"，发现四个问题，其中两个
会在机器人运动时移除停车入口。

| 项目 | 状态 | 结果 |
| --- | --- | --- |
| 取消或跑完后无法重新开始 | 已完成 | 取消不清除缓存任务，管理器实际接受重新开始，但面板按 `state == READY` 置灰，`FINISHED` 下按钮是灰的，只能走命令行 |
| 执行中收到新预览覆盖执行状态 | 已完成 | 任务订阅回调无条件把状态改成 `READY`/`IDLE`，从不检查 `active_goal_`。执行中重新规划会把面板变成 `Ready` 并显示**新任务的 revision**，清除点选会变成 `Idle`——两种情况下 **Cancel 按钮消失，机器人还在走**。执行本身安全（执行器已复制任务），坏的是显示和操作入口。现改为执行中只更新缓存，保持 `state` 与运行中任务的标识不变 |
| 执行器消失后永久锁死 | 已完成 | 已接受的 Goal 只由结果回调结束，执行器被杀后结果永不到达，管理器永远停在 `EXECUTING`，`can_start` 永远为假，只能重启管理器。实测确认。新增 `executor_timeout_s`（默认 `5.0 s`）监视 Action 服务存在性并释放 Goal，报 `CONTROL_TIMEOUT`。`SIGKILL` 到释放实测约 `25 s`，其中约 `20 s` 是 DDS 摘除已死参与者的时间。机器人此时已由速度看门狗停住 |
| 点选模式下未选点即 Replan | 已完成 | `lower_left` 等在点选模式下仍保留配置文件的默认值，且 `clear_points` 不重置它们。未点选或清除后按 Replan 会规划出一块**没人选过的区域**并变为可启动。规划器现在在点选模式下要求先选够点，否则拒绝并说明还差几个点 |
| `can_start` / `can_cancel` | 已完成 | `CoverageStatus` 增加两个布尔，由管理器按自己服务的前置条件计算，面板直接渲染。面板从 `state` 反推正是第一条 bug 的成因。二者是提示不是保证：请求仍可能被拒绝（如执行器未运行），原因在服务响应里 |
| 面板显示规划器状态 | 已完成 | 规划失败与"未选择区域"在管理器看来都是空任务，无法区分，都报 `Idle`。原因只在规划器的 `/coverage/status` 上，面板因此增加一行单独显示该话题，同时也能看到每次点击被接受的坐标 |
| Replan / Clear points 不再置灰 | 已完成，2026-08-18 已修正 | 当时改为始终可用，理由是它们只改预览、不动运行中的 Goal。该理由对**消息**成立，对**操作者看到的东西**不成立：预览就是画在机器人身上的那条轨迹。见下面 2026-08-18 一节 |
| `test_coverage_executor` 偶发失败 | 已完成 | 全量并行跑时 `test_refuses_a_first_scan_entry_that_cannot_be_recovered` 失败过一次，单独跑通过，重跑全量两次均通过。定位为负载下 `3 s` 的 Goal 应答等待过紧。该类等待纯属 ROS 往返，与被测行为无关，统一放宽到 `12～15 s`；测量运动的 `40/45 s` 上限不动 |

新增两个 launch 测试：`test_coverage_manager_states.py` 覆盖上述状态矩阵五条路径，
`test_coverage_manager_executor_loss.py` 用一个只接受不完成的替身 Action 服务复现
执行器消失。规划器侧新增点选模式下 Replan 前置条件的用例。全量 288 项通过，
22 项按环境跳过。

### 已知限制，本轮不修

- **执行中重启管理器**：新管理器没有该 Goal 的句柄，Action 无法认领已有目标，
  因此会显示 `IDLE` 且无法取消，而执行器仍在跑。需要停车时用
  `ros2 action send_goal` 之外的手段，或直接停执行器（速度看门狗会零速停车）。
  这是 ROS Action 的固有限制，改需要另一套设计。

## 2026-08-17 面板在 dock 宽度下被裁字

操作者反馈"有些提示文字显示不全，可能是窗口太小"。WSLg 下截不到 RViz 窗口
（XGetImage 只返回黑图），改为把面板本身离屏渲染成 PNG 逐宽度检查，问题一次可见。

| 项目 | 状态 | 结果 |
| --- | --- | --- |
| 面板最小宽度 566 px | 已完成 | 两列网格把每条消息的整行宽度当成硬性要求交给 dock。渲染窗口在 1200 px 配置里只剩 760 px，dock 给不起，RViz 直接裁字。长文本改为整行、字段名在上方，标签水平策略改 `QSizePolicy::Ignored`（接受任何宽度，用 `heightForWidth` 申报高度）。最小宽度降到 240 px，实测渲染窗口从 `760` 增至 `820` px |
| 任务号被拦腰截断 | 已完成 | `coverage_20260817_143512_rectangle` 里没有空格，换行算法无处可断，只能截断且无任何提示。`wrappableText()` 在 `_` `/` `-` 后插入零宽空格（U+200B）作为断点；宽度够时不会被使用。管理器消息里同样引用任务号，走同一条路径 |
| dock 变矮时吞掉下面几行 | 已完成 | 正文放进 `QScrollArea`；按钮留在滚动区外，运动中找 Cancel 不该需要先滚动 |
| State 值列被挤到放不下 `Executing` | 已完成 | 网格把值列压到 63 px。值列下限改为按主题字体量出的最宽状态词，而不是写死像素 |
| 显式 `setMinimumWidth` 反而制造裁字 | 已完成 | 曾设 `180 px`，它覆盖布局算出的最小值，等于允许 dock 把面板压回裁字状态。改为不设，由布局自己申报；按钮不能换行也不能滚动，是宽度下限的真正来源 |
| 布局回归护栏 | 已完成 | 新增 `test_coverage_panel_layout`，在 240/300/420/560 px 下真正排版并检查无裁切，同时约束 `minimumSizeHint ≤ 240 px`、`sizeHint ≤ 340 px`。测试经 `renderStatus()` 走真实渲染路径而不是复制一份文案。`CLIMBOT_PANEL_SHOT` 可让同一用例导出各宽度 PNG 供肉眼确认 |

上一轮遗留的"面板实际视觉布局未经自动化验证"由此闭合，按钮点击行为仍靠
`test_coverage_manager_states.py` 从服务侧覆盖。

### dock 高度分配

配置里原本只有 `Width` / `Height`，没有 `QMainWindow State`，Qt 就把左列平分：
Tool Properties 只有两行工具设置，却和操作面板一样高。现在 `coverage.rviz` 带上
该段（Qt `saveState()` 的十六进制，按 dock 的 `objectName` 恢复，RViz 把它设成面板
名），左列约 `300 / 90 / 374 px` 分给 Displays / Tool Properties / Coverage Task。

生成器 `climbot_coverage/scripts/make_rviz_window_state.py` 用同名 dock 重建同样的
布局再 `saveState()`，只改配置里的那一行，不像在 RViz 里存盘那样重写整个文件
（会丢掉所有注释）。写死的宽度是 `342` 而不是 `364`：该脚本的窗口没有菜单栏和
状态栏，RViz 的有，恢复出来会宽约 `22 px`，实测 `342` 正好落回原来的 `364`，
即这次改动只动纵向切分。宽度低于面板自身最小值会被 Qt 静默夹住——`250` 被夹回
`364`，是 Displays 属性树的下限。

`test_rviz_config` 增加一条：blob 里必须出现 `Panels` 中每个面板名（Qt 以 UTF-16BE
编码存 `objectName`）。改名后忘记重新生成时 Qt 只是退回平分，不报任何错。
已用把 `Coverage Task` 改成 `Coverage Panel` 验证该用例会失败。

RViz 确实读取该段：把宽度写成 `500` 后渲染窗口从 `820x764@364` 变为
`662x764@522`。**dock 的实际像素高度未直接观测**——WSLg 下截不到 RViz 窗口，
证据是"RViz 应用该 blob"加"该 blob 离屏恢复出 322/97/402"两条。

## 2026-08-17 控制环被系统时钟回跳冻住

`test_coverage_executor.py` 长期偶发失败（整包 ctest 实测 2/4，单文件 2/9）。逐层
剥下来根因与覆盖控制逻辑无关：**控制环的定时器建在可以往回跳的时钟上**。

### 定位过程

| 步 | 实验 | 结果 |
| --- | --- | --- |
| 1 | 录制整个闭环轨迹 | 失败那次指令**冻结**在 `v=0.1878` 约 `1.3 s`，机器人冲到 `x=-0.5201` |
| 2 | 测试内订阅 vs **进程外**独立订阅 | 两边看到完全相同的间隙（`1.923/1.733/1.143 s`）→ 是发布端停了 |
| 3 | 空跑节点：不发目标、不发里程计、无测试 | `90 s` 内仍停 3 次（`1.68/1.38/1.30 s`）→ 与控制逻辑无关 |
| 4 | 同时记录 cmd_vel 间隙与 `CLOCK_REALTIME` 跳变 | 间隙 `t=18.40/50.87/83.13`，跳变 `t=18.42/50.89/83.14`；幅度 `1.331/1.340/1.237` vs `1.310/1.320/1.209` |
| 5 | **同一进程内** A/B 两个 50 Hz 定时器 | ROS 时钟定时器在跳变处出现 `1.238/1.674 s` 间隙；**STEADY 时钟定时器 0 个间隙** |

WSL2 每约 `30 s` 对宿主机重同步一次，把 `CLOCK_REALTIME` 往回步进 `1.2～1.7 s`。
节点默认时钟是 `RCL_ROS_TIME`，实测在 `use_sim_time=false` 时与 `CLOCK_REALTIME`
相差 `<0.1 ms`——即退化为系统时钟。定时器在回跳期间不触发，控制环整段不发指令。

**为什么表现为这个测试偶发失败**：起跑点在 `x=-0.40`（首航点前 `0.40 m` 引道），
motion region 边界在 `x=-0.50`，只有 `100 mm` 余量。停顿落在减速进点那一刻时，
测试的 plant 按最后收到的指令继续积分，`1.3 s × 0.19 m/s ≈ 0.25 m` 无制动前冲，
冲出边界报 `Fused robot position left the task motion region`；落在原地转向或停车时
则毫无影响。跳变每 `30 s` 一次、一次跑 `20～40 s`，所以是抛硬币。

**Gazebo 全系统跑不受影响**：那边 `use_sim_time:=true`，跟的是 `/clock`，单调。
暴露的是所有节点级 launch 测试（都没设 `use_sim_time`）和真机（任何 NTP 阶跃）。

### 修改

| 项目 | 状态 | 结果 |
| --- | --- | --- |
| `control_clock.hpp` | 已完成 | `controlClock(node)`：`use_sim_time=true` 返回节点时钟（仿真行为逐位不变），否则返回 `RCL_STEADY_TIME` |
| `line_tracker_node` | 已完成 | 定时器与**全部**时长测量（`segment_timeout_s`、settle 窗口、`dt`、里程计新鲜度、`elapsed_time_s`、stale 警告节流）改用控制时钟；只有 `reference_path` 的 `header.stamp` 留在 ROS 时间 |
| `cmd_vel_watchdog_node` | 已完成 | 定时器与 `commandAt()/accept()` 的取时改用控制时钟。这是最后一道兜底，之前与控制器**共享同一失效模式**：回跳时它的定时器也冻住，而且 `now() - received_time_s_` 反而变小，`timedOut()` 更不容易成立 |
| `coverage_manager_node` | 已完成 | 反馈节流与 `start_response_timeout_s` 的计时同样改用控制时钟（G/H 之外顺手补齐的同类实例，影响面只到状态发布） |
| 测试 plant 忠实化 | 已完成 | `test_coverage_executor.py` 的 `_current_command()` 在指令超过 `0.40 s`（= `command_timeout_s`）未更新时返回零速，与真实看门狗一致。此前它永远按最后一条指令积分，等于把控制环的任何停顿记成静默超冲，而不是真机会做的停车 |
| 回归护栏 | 已完成 | `test_control_clock`：非仿真时间下必须是 `RCL_STEADY_TIME`、仿真时间下必须就是节点时钟、`RCL_STEADY_TIME` 确实不是系统时钟的别名。把实现改回 `get_clock()` 即失败（已验证） |

### 验证

- 空跑节点 `100 s`：2 次时钟回跳（`-1.648/-1.232 s`），**cmd_vel 0 个间隙**，
  `5001` 条 / `99.9 s` = `50.06 Hz`（修前 `47.7 Hz` 带 3 个间隙）。
- 看门狗同样 `100 s`：4 次回跳，**0 个间隙**，`5001` 条。
- 仿真时间语义未变：`/clock` 按 `0.5×` 推进时控制环 `49.7 Hz` 仿真时间 /
  `25.0 Hz` 墙钟。
- `climbot_control` 整包 ctest **连续 15 次运动测试全过**（修前 2/4 失败）。
- 全量 `308` 项通过，`25` 项按环境跳过。

## 2026-08-17 阶段 F 验收数据补齐

逐条核对 §13 阶段 F、§14 和 §15 后，发现缺的不是代码而是**数据**：横向矩形工况
从未归档过，§14.4 整节没有任何配对数据，三份既有基线的 `provenance` 都带
`source_modified=true`。

### 四份覆盖基线

| 项目 | 状态 | 结果 |
| --- | --- | --- |
| 横向矩形基线缺失 | 已完成 | §13 阶段 F 第 2 条要求横向和竖向矩形各一份，`coverage_horizontal_demo.yaml`（`4.30 × 1.70 m`）的配置一直在，实验从未落盘。8 月 13 日那次 `132.5 s` 的演示在停车精度修复之前，也没有存文件。现已归档，且是四者中最好的一份：覆盖率 `99.964%`、最大单段横轨 RMS `1.08 mm`、间距误差 `5.08 mm`、`0` 次可见往复 |
| 三份旧基线过期 | 已完成 | 摘要记的提交是 `2949323` 且 `source_modified=true`，此后隔了 12 个提交。在 `6227a3c` 的干净树上重跑，四份摘要现在统一为 `commit=6227a3c`、`source_modified=false` |
| 8 月 15～17 日改动未影响运动 | 已确认 | 重跑数值与 8 月 14 日那版差异都在测量噪声量级内（覆盖率 `99.632→99.638%`、`99.798→99.798%`、`98.122→98.070%`）。这与代码分析一致：进度加权和面板状态只影响状态上报，控制环时钟在 `use_sim_time:=true` 下逐位不变 |

四个工况全部通过评价器的每一项门限，逐项数值见
[../results/README.md](../../../../results/README.md)。相对 `95%` 覆盖率门限的余量为
`4.96 / 4.64 / 4.80 / 3.07` 个百分点，因此仍不需要顶部收边扫描（B-4）；梯形竖向
的 `3.07` 个百分点是四者中最薄的。

