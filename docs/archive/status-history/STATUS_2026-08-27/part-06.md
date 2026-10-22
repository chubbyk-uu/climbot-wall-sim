# 历史分册 6

## 2026-08-20 README 时长口径与 RViz 截图（review M6 + L1）

**M6：`README.md` 对时间点控制的入口结论过度概括。** 原文写"额外给出误差约 `2%` 的
任务时长预测"——这只对当前八个长任务基线成立。同一仓库的 `docs/STATUS.md` 和
`results/README.md` 已经证明单段短任务系统性高估 `8%~17%`，离轴起点进入还会改变误差
符号。不影响直线跟踪验收，但会误导用 `planned_total_s` 排工期的读者。入口改为两段
口径，并指回 `STATUS.md`。

**L1：`docs/images/rviz_coverage_task.png` 已重拍。** 旧图是墙面工作系原点还在底边
中心时拍的：坐标轴画在绿框底边中央、绿框跨正负 X，而 `Displays` 里还是已经删掉的
`rviz_default_plugins/Grid`。新图是当前状态：坐标轴在墙面左下角，网格由
`Wall Reference Grid`（MarkerArray）画。

重拍时发现 `coverage.rviz` 的视角有个实际缺陷：`Distance: 9` 的注释写着"框住整面
`10 × 8 m` 墙"，实际框不住。渲染区宽大于高，所以约束在高度上，而 RViz `45°` 垂直
视场在 `9 m` 处只看得到约 `7.5 m`——**安全内缩绿框的上下两条边落在视野外**，而那个框
正是点选必须落在里面的东西。改成 `Distance: 12`，窗口 `1280 × 1000`。

截图本身是脚本拍的，不是手点的：三个角点用 `/clicked_point` 发布（规划器分不出这和
鼠标点选的区别），构型走 `/coverage/configure`，启动走 `/coverage/start`；机器上没装
截图工具，用 `ctypes` 直接调 `libX11` 的 `XGetImage` 抓窗口。

## 2026-08-20 面板的迟到应答（review M3）

上一轮 M5 修好了"永久置灰"，但留下了和 H2 同一个代次问题：3 秒超时只把
`configure_pending` / `tracking_pending` 置回 `false`，**并没有撤回请求**。服务随后
仍可能应答，而它带回来的是被问的那一刻的内容——当时在用的构型、当时的 `tracking_mode`。
操作员此时早已改做别的（超时正是通知他们可以继续的那件事），第一个请求的迟到回调会
清掉第二个请求的 pending 标志，并把旧 config、旧模式和旧提示写回界面，看起来像面板
撤销了操作员刚做的动作。

改法与 H1/H2 同构：新增 `RequestGate`（`coverage_panel.hpp`，与 `requestHasExpired()`
并列，同样是把可测的判断从 Qt/ROS 线程编排里拿出来）。每个请求发出时 `begin()` 取一个
代次，回调带着它回来交给 `isCurrent()` 判断；超时改调 `abandon()`——代次前进但不算发出
新请求，于是在飞的那个请求的应答从此不匹配。三条路径全部走这个门：`onConfigurationChosen`、
`onAlgorithmChosen`、`readTrackingMode`。

`test_request_gate.cpp` 五个用例，其中一个就是面板实际会产生的时序：发出 → 超时 →
再发 → 第一个服务这时才应答，断言迟到那个既不被应用、也不能把第二个请求判为已完成。

## 2026-08-20 摘要改成合法 JSON（review M1）

`results/` 里 172 份摘要有 **27 份不是合法 JSON**：Python 的 `json.dump()` 默认写出裸
`NaN` 记号，RFC 8259 只允许有限数字。Python 自己读得回来，所以仓库里没人发现；Ruby、
严格 Java/Go、schema 校验和多数数据仓库读不了。只有生产者自己能读的证据，不算机器可读
验收证据。

复核后比 review 的描述更值得注意的一点：**27 份里有 26 份是干净树产物**，也就是正式
可引用证据，和 M4 删不删脏树轨迹无关。

`NaN` 出现在四处，全部改掉：

| 位置 | 含义 | 现在写什么 |
| --- | --- | --- |
| `scan_line_spacing` 少于两条平行扫描线 | 不适用 | `null` + `applicable: false` + `not_applicable_reason` |
| `execution_quality` 四个 `max(..., default=nan)` | 没有可测的段 | `null`（与旁边一直是 `None` 的 `maximum_horizontal_height_drift_m` 一致） |
| `straight_line_approach_bearing_deg` 的哨兵值 | "沿线方向" | 参数本身仍是 `NaN`，写进 `provenance` 时转 `null` |
| `_write_json` | — | `allow_nan=False`，非有限值直接报错而不是写出去 |

`applicable` 标志是必要的：单独一个 `null` 分不清"不适用"和"没测到"，而这是关于一次
运行的两件不同的事。原代码靠 `NaN` 的比较语义来编码这个区别——间距用 `isnan` 判"不适用
就跳过"，端点误差用 `nan <= limit` 为假来判"没证据就失败"。两者都对，但**读起来像是
巧合**，现在各自写明。

判定逻辑相应改为显式：端点误差为 `null` 是失败（这次运行没产生任何证据），转向末端
航向误差为 `null` 是不适用（单段直线任务本来就没有转向），二者不再共用一个记号。

已有 27 份摘要就地重写为 `null`，只改这些字段，其余数值逐字不动。新增
`test_results_are_machine_readable.py` 对 `results/` 下全部摘要做严格解析，并要求每个
`null` 间距都带上不适用的理由；未强制要求已有的 145 份合法文件补 `applicable` 字段——
它们带的是真实数值，本来就没有歧义，为此重写它们只会制造无谓的 diff。

实测：`line_vertical`（单条扫描线，正是不适用那条路径）跑通，`exit=0`，回归表格的
间距列显示 `n/a` 而不是 `nan`。

## 2026-08-20 回归脚本的进程清理（review M2）

`tools/run_coverage_regression.sh` 用 `setsid ... &` 起仿真、规划器和执行器，却没有
任何 `trap`。Ctrl-C、关终端或脚本异常时，独立会话里的子进程收不到前台进程组信号，
`lane_teardown()` 也未必执行；而它扫全 `/proc`、按 `GZ_PARTITION` 直接 `kill -9`，
既不限于本脚本创建的进程，也不给 Gazebo 落盘的机会。这与本项目反复出现的
"后台还有东西在跑"完全对得上——本轮开始前刚清掉 19 个几小时前的残留节点。

改为：

- 进程组记账。每个 `setsid` 起的东西创建时把进程组 ID（从内核读回，不假设 `setsid`
  一定 exec）记进 `$RUN_DIR/laneN.pgids`；teardown 先对这些组发 `TERM`、限时等待
  （`LANE_TERM_GRACE_S`，默认 `20 s`），仍活着才 `KILL` 并报告。
- **评价器也进独立会话**。它原先在前台跑，位于脚本自己的进程组里——唯一能打到它的
  信号是发给整个组的，而那正是 lane 绝不能发的。实测中断时它每次都活下来，只能靠
  分区兜底扫掉。退出码经 `wait` 照常回来。
- 顶层 `trap cleanup_all EXIT` 与 `trap interrupted INT TERM`，退出时先 `TERM` 各
  lane 子 shell，再逐 lane teardown，并打印 teardown 摘要。
- 分区扫描保留为兜底，但先 `TERM` 后 `KILL`，且**报告它抓到了什么**——兜底抓到东西
  意味着这一 lane 创建了没记账的进程，下一个逃掉的可能根本不带分区。`ros2 daemon`
  单列：它由 `ros2 topic list` 拉起、自己 daemonize 到独立会话、并继承 lane 的环境，
  脚本无从记账，属于预期内，不计为逃逸。
- 清理逻辑拆到 `tools/lane_processes.sh`，因为它只在出错时才有意义，正常跑一遍永远
  测不到。`tools/test_lane_processes.sh` 直接测它：正常组被 `TERM` 收走且不产生报告、
  忽略 `TERM` 的组被 `KILL` 并被报告、组外进程被兜底抓到并被报告、一个 lane 不碰另一个
  lane 的进程。

实测：起一个 lane 到 Gazebo 就绪（15 个进程），向脚本发 `SIGINT`，清理后存活进程
为 `0`，teardown 摘要为空（无逃逸）。随后完整跑一遍 `line_vertical`：`exit=0`，
用例通过，结束后同样无残留。

## 2026-08-20 失联后的停机权限（review H1 + H2）

2026-08-20 复核的两条高严重度项已修完。两者根子是同一个：管理器只有
「当前有没有 Goal」这一个状态，没有代次，也没有区分「Action 服务不可发现」和
「机器人已经停下」。

**H1：失联后在取消被确认前就宣布 FINISHED 并撤掉停止入口。**

原实现在 Action 服务连续消失超过 `executor_timeout_s` 后，发一次 `async_cancel_goal`
（不保存 future、不等确认）、立刻 `active_goal_.reset()`、报 `FINISHED/EXECUTOR_LOST`。
注释称"机器人此时已由速度看门狗停住"——这只在执行器真的死了、`/control/cmd_vel`
停止刷新时成立。DDS 发现抖动、Action 通道故障或取消没送达时，执行器还活着、指令还在
刷新，看门狗没有超时可触发，机器人继续在走；而 `can_cancel` 由 Goal handle 决定，
于是操作员的停止入口恰好在最需要它的时刻消失。

现在改为：

1. 新增管理器状态 `STOPPING`（`CoverageStatus.msg`）。失联先进这个状态，不报完成、
   不释放 Goal handle、`can_cancel` 保持为真（此时按停止会重试保持和取消）。
2. 新增一条不经过执行器的停止通路：`cmd_vel_watchdog_node` 的 `/control/hold`
   （`std_srvs/SetBool`）把 `/cmd_vel` 强制为零，与 `/control/cmd_vel` 上是否还有指令
   无关，状态在 `/control/hold_active` 上 transient local 发布。系统里其余所有停止都是
   「请求正在驱动的一方停下来」，执行器不应答时它们全部失效；保持在轮子前的最后一跳，
   因此与图上其余部分处于什么状态无关。
3. 离开 `STOPPING` 只认**关于机器人**的证据，三者取一：保持已生效；`/control/cmd_vel`
   上连续 `command_quiet_s`（默认 `1.0 s`）没有非零指令；执行器最终应答了（此时用它
   返回的真实结果而非 `EXECUTOR_LOST`）。三者都不成立就停在 `STOPPING`——这正是这个
   状态存在的目的。

**H2：超时只清 pending，没有使旧异步回调失效。**

三个 Goal 回调只捕获 `this / task_id / revision`，没有代次也没有 handle 校验，而
`expireStalePending()` 只清 `start_pending_since_`，原 `async_send_goal` 仍在飞。迟到的
应答会把一个没人在看的 Goal 重新报成 `EXECUTING`；迟到的反馈会把旧任务进度写进新任务；
迟到的结果会无条件 `reset` 掉正在运行的 Goal handle 并报 `FINISHED`——连带撤掉停止入口。

现在每次 `start()` 分配单调代次，三个回调都只在自己那一代仍是当前代时才说话；超时、
失联和新的开始各自使旧代次作废。迟到的应答若带回一个已被接受的 Goal，管理器直接对它
发取消：没人打算运行它，放着不管它会把整个任务走完。

**测试。** 原 `test_coverage_manager_executor_loss.py` 只覆盖"测试桩被真正销毁"这一
安全情况，全绿并不能证明失联路径安全。现在：

- `test_an_unreachable_but_still_driving_executor_is_not_called_finished`：Action server
  销毁、但 `/control/cmd_vel` 持续下发非零指令。断言管理器停在 `STOPPING`、`can_cancel`
  为真、**从不**发布 `FINISHED`；随后出现速度保持服务，才允许释放。这是原实现会失败的
  那条路径。
- `test_the_operator_stop_still_does_something_while_stopping`：失联期间按停止必须真的
  打到速度保持上，不能是空操作。
- `test_coverage_manager_late_callbacks.py`：桩执行器第一次 Goal 拖到超时之后才应答、
  且拒绝被取消，于是应答、反馈、结果三个迟到回调依次到达一个已经在跑第二个任务的
  管理器。断言状态仍是 `EXECUTING`、版本仍是新任务、进度没有被写脏、停止入口还在。
- `test_cmd_vel_watchdog_node.py` 增加保持测试：指令持续到达时保持仍把输出压零，
  解除后恢复——保持不是一个繁忙发布者能不断重置的超时。

宿主全量：**455 tests，0 error，0 failure，29 skipped**（原 449，新增 6 项）。

## 2026-08-20 墙面贴图贴上墙

烘焙工具链进仓库，整墙贴图本地生成并渲染验证通过。参数是 08-19/08-20 定案的那一套，
测量与理由在 `results/README.md`「墙面贴图：从崩溃到定案」。

| | |
| --- | --- |
| 仓库里有的 | `tools/fetch_wall_texture.sh`（源素材 `sha256`）、`tools/bc1.py`、`tools/bake_wall_texture.py`、`climbot_gazebo/wall_texture.py` |
| 仓库里没有的 | 源素材 `540 MB`、烘焙产物 `204 MB`（`textures/` 已 gitignore） |
| 复现靠 | 校验和 + 种子 `20260820` + 清单里记录的全部参数 |

`--source-size-m` 没有默认值，必须显式给。ambientCG 没有记录 `Concrete044D` 的真实
尺寸，`2.5 m` 是**本项目的声明**，而烘焙里每一个长度都从它推出来——让它出现在命令行
和清单里，比藏在一个默认值里诚实。

实测与预测逐项相符：`20000 × 16000` 纹素、`8 × 10 = 80` 块、`203.5 MiB`（预测 `203`）、
边缘块 `1568 × 1664`（4 的倍数）。quilting `26 s`，BC1 编码约 `3 min`。

渲染验证的结论和那个只有渲染能查出的坐标 bug，见上面「未决事项」的下一步第 5 条第 2 小项。

## 2026-08-20 参考网格线做成开关

墙面参考网格线现在可以关掉，两个视图都能关，且能在 RViz 里当场勾掉。

起因是拍摄墙面的准备：网格线是周期性高对比特征，还浮在墙面前 `3 mm`，在
`0.50 × 0.28 m` 的拍摄视场下会进入 `67%` 的照片（实测见下面的贴图一节）。但它
对人工看规划有用，所以不能直接删。

两套网格各有各的开关，互不影响：

| 开关 | 作用范围 | 生效方式 |
| --- | --- | --- |
| `wall_grid_spacing:=0` | **只**关 Gazebo 墙面上那套 | 启动时 |
| RViz 左侧 `Wall Reference Grid` 复选框 | **只**关 RViz 叠加层 | 当场生效，不重启 |

一开始把两者绑在同一个参数上（"一个词管两边"）。这是错的：会进照片的只有墙面那套，
而拍照运行里操作员照样要看规划，关掉墙面的同时把他的网格也拿走没有道理。现在
`wall_grid_spacing` 只到 Gazebo，RViz 叠加层的间距直接取 `wall.yaml`，永远发布。

三处顺带修掉的问题：

- **两个视图原本画的不是同一套网格。** Gazebo 那套按"距每条边一个间距"排线，
  是原点还在墙面中心时定的规则；RViz 这边最初按工作系原点整数倍画，两者在
  `10 × 8 m`、`1 m` 间距下恰好重合，换个间距就分家。统一成"工作系原点的整数倍、
  只画内部线"。默认构型下渲染结果逐项不变（列 `1..9`、行 `1..7`，中线加粗在
  `x = 5`、`y = 4`），所以 `docs/images/gazebo_wall.png` 仍然有效；
- **`coverage.rviz` 里那个 `Wall Grid` 显示项在原点移动后就错了。** 它是
  `rviz_default_plugins/Grid`，画的是以固定坐标系为中心的正方形——原点在墙面中心时
  正确，移到左下角之后变成一个挂在墙角外的 `10 × 10 m` 方块。已换成订阅
  `/coverage/wall_grid` 的 `MarkerArray`，线是照墙面裁的；
- **间距值原本只存在 `simulation.yaml` 里，RViz 读不到。** 移到
  `climbot_description/config/wall.yaml` 的 `reference_grid.spacing_m`，由
  `wall_frame.reference_grid_spacing()` 统一读取，四个 launch 文件共用。

新增两个测试：`test_coverage_wall_grid_off.py`（间距 `0` 时只发 `DELETEALL`、
不发任何可画的东西，而规划本身照常）和 `test_coverage_planner_node.py` 里的
网格线位置断言（按规则逐条比对线的工作坐标，不只比数量）。后者已用"把起始
下标从 `1` 改回 `0`"验证过确实会失败。`test_rviz_config.py` 再加一条：显示项订阅的
话题必须是规划器真的发布的那个——话题名写错时 RViz 只会安静地什么都不画。

**`docs/images/rviz_coverage_task.png` 仍需重拍**，除了旧工作系的坐标和绿框，
现在网格显示项也换了。（已于 2026-08-20 重拍。）

