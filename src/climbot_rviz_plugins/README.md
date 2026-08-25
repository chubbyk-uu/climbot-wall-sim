# climbot_rviz_plugins

覆盖任务的 RViz 操作面板。按 PROJECT_GUIDE §11.1，面板只负责人机交互：
它渲染管理器发布的状态、调用管理器的服务，任务锁定、版本检查和安全状态转换
全部留在管理器里。

## 面板

`climbot_rviz_plugins/Coverage`（`rviz_common::Panel`）。已写入
`climbot_coverage/rviz/coverage.rviz`，随 `coverage_mission.launch.py` 自动出现；
手动添加则用 RViz 菜单 `Panels → Add New Panel`。

显示：

| 字段 | 来源 |
| --- | --- |
| Region / Sweep | `CoverageConfig`，可写，调 `/coverage/configure` |
| Algorithm | `/line_tracker` 的 `tracking_mode` 参数，可写 |
| Points | `selected_points` / `required_points`，点选模式下才有意义 |
| State | `CoverageStatus.state` |
| Segment | `current_segment + 1` / `total_segments`；接近首点期间显示 `approach of N` |
| Progress | `progress`，百分比进度条 |
| Schedule | `planned_total_s` / `estimated_remaining_s` / `schedule_lag_s` |
| Task | `task_id` 与 `revision` |
| Manager | `message`，与管理器日志同一行 |
| Planner | 规划器的 `/coverage/status`：点击被接受的坐标、规划失败原因、清空确认 |
| Last request | 最近一次按钮调用的服务响应 |

Progress 和 Schedule 分开是有意的。Progress 说**做完了多少工作量**，机器人卡住时
它正确地停住不动；Schedule 说**跟不跟得上计划**。把时间折进进度条会让一台卡死的
机器人把条走到 100%。位置控制模式下 Schedule 末尾写 `estimate only`——不是说这个
数不准，而是说没有任何东西执行或监测它；时间点控制模式下写的是实时滞后。

Algorithm 走参数接口而不是 `/coverage/configure`，因为后者是规划器的服务，
`tracking_mode` 是执行器的配置。

规划失败与"未选择区域"在管理器看来都是空任务，无法区分，都会报 `Idle`。
真正的原因只在 Planner 一行。

按钮：

| 按钮 | 服务 |
| --- | --- |
| Replan | `/coverage/replan` |
| Clear points | `/coverage/clear_points` |
| Start | `/coverage/start` |
| Cancel / Stop | `/coverage/cancel` |
| Force abandon（5 秒内二次点击） | `/coverage/force_abandon` |
| Rearm after verification | `/coverage/rearm` |

四个任务操作按钮的可用性直接取自 `CoverageStatus` 的 `can_start`、`can_cancel`、
`can_force_abandon`、`can_rearm`——由管理器按自己服务的前置条件计算，面板不做推断。面板自行从
`state` 推断正是"取消后无法重新开始"那个 bug 的成因。

Force abandon 只处理 Start 应答永久未知的恢复死锁。第一次点击只显示“这不证明任务
停止”的风险说明，5 秒内第二次点击才调用服务；管理器随后进入 `RECOVERY_LOCKED` 并
持续请求 hold，Start 仍不可用。完成硬件急停、驱动失能或执行器终止确认后，操作员才
使用单独的 Rearm。两个恢复按钮平时隐藏，只有管理器明确允许其中一个时才显示；面板只
实现二次确认交互，恢复锁及许可判断仍全部属于管理器。

**任务运行期间，Region、Sweep、Algorithm、Replan、Clear points 五个控件全部置灰**，
只留 Cancel。它们发出的请求确实只改预览、不动运行中的 Goal，但预览就是画在机器人
身上的那条轨迹，运行中改它看起来就像任务被换掉了；换形状还会直接把它撤掉。置灰
同样取自 `can_cancel`，面板不另立一套"是否在运行"的判断。

这些都是提示而非校验：无论面板显示什么，非法请求都由管理器或规划器拒绝，并把
原因显示在 Last request 一行。

## 布局

面板停靠在渲染窗口旁边的 dock 里，宽度由操作者拖动决定，而管理器发布的是整句话
而不是词。最初的两列网格把每条消息的整行宽度当成硬性要求交给 dock，最窄要
566 px，dock 给不起，RViz 就直接把文字裁掉——按钮上剩下"Cancel / Sto"。现在：

- 长文本（Task / Manager / Planner / Last request）各占整行，字段名在上方；
  只有 State / Segment / Progress 还保留"名字在左、值在右"的两列。
- 这些标签的水平尺寸策略是 `QSizePolicy::Ignored`，即接受 dock 给的任何宽度，
  再用 `heightForWidth` 自己申报高度，不会反过来把 dock 撑宽。
- 任务号 `coverage_20260817_143512_rectangle` 里没有空格，换行算法无处可断，
  只能拦腰截断。`wrappableText()` 在 `_` `/` `-` 后插入零宽空格（U+200B），
  给换行算法一个断点；宽度够时它不会被使用，任务号仍然显示成一整条。
- 正文放在 `QScrollArea` 里，dock 太矮时滚动而不是把下面几行吞掉；按钮留在滚动
  区外面，正在运动时找 Cancel 不该需要先滚动。
- State 一行只放状态名。未收到管理器状态时它显示 `Not connected`，解释那句话放在
  整行宽的 Manager 一行——放在 State 的窄列里会被折成四行。
- dock 的高度分配不归面板管，归 `coverage.rviz` 里的 `QMainWindow State`（Qt 的
  `saveState()` 十六进制，按 dock 的 `objectName` 恢复，RViz 把它设成面板名）。
  没有这一段时 Qt 把左列平分，Tool Properties 只有两行工具设置却和操作面板一样高。
  默认不加载 Tool Properties，把左列主要高度留给 Displays 和 Coverage Task；需要调整
  RViz 工具属性时可从 `Panels` 菜单手动添加。
  改面板名或加面板后要用 `climbot_coverage/scripts/make_rviz_window_state.py`
  重新生成，否则 Qt 静默退回平分。
- 面板不设显式 `setMinimumWidth`：显式最小宽度会覆盖布局算出来的那个，一旦写小了
  就等于允许 dock 把面板压回裁字的状态。按钮不能换行也不能滚动，它们才是
  dock 宽度下限的真正来源（约 240 px）。

G4 不会把巡检控件继续堆到现有正文底部，也不会新增另一个 dock。计划把本面板改为：

- 顶部固定公共区：任务、状态、段进度，以及 `OFF`／`Ready 0/N`／`Recording M/N`／
  `Failed`／`Completed N/N` 形式的一行采集摘要；
- 中部三个可滚动页签：`任务规划` 放现有规划配置和 Replan／Clear points，`巡检采集`
  放任务级采集开关、根目录、浏览／恢复默认、预计数量／空间、归档计数／目录／错误，
  `详情` 放 Manager／Planner／Last request 等长文本；
- 底部固定安全区：Start、Cancel / Stop，不随页签隐藏；Force abandon 或 Rearm 仅在
  对应异常恢复状态临时出现。

默认宽度和渲染窗口占用保持不变。路径编辑在窄面板中独占一行，浏览按钮另起一行；
机器相关默认路径来自 launch／YAML，不保存进公共 `.rviz`。实现后继续使用 240、300、
420、560 px 离屏布局测试，并在实际 RViz dock 中检查标签、滚动和按钮可达性；视觉效果
不合适时允许调整页内排列，但不能牺牲固定停车入口或采集故障摘要。

## 线程

RViz 在执行器线程上派发订阅与服务回调，Qt 控件只能在 GUI 线程访问。因此回调
只把数据写进互斥量保护的成员，控件全部由 `QTimer`（`100 ms`）在 GUI 线程刷新。

## 测试

```bash
colcon test --packages-select climbot_rviz_plugins climbot_coverage
```

- `test_coverage_panel_plugin`：按 RViz 的方式经 ament 索引加载该 Panel 类。
  构建成功并不能说明面板可用——插件加载失败时 RViz 只警告一次然后照常启动，
  面板是消失而不是报错。该用例覆盖库名写错、缺少导出宏和符号未解析。
- `test_coverage_panel_layout`：在 240 / 300 / 420 / 560 px 四种 dock 宽度下真正
  排版一次面板，检查没有控件被裁：标签的 `heightForWidth` 不超过分到的高度、
  没有不可断的长串宽于所在标签、按钮宽度不小于其 `sizeHint`。同时约束
  `minimumSizeHint`（≤240 px，否则 dock 给不起）与 `sizeHint`（≤340 px，否则
  新开的 dock 会把渲染窗口挤没）。
- `climbot_coverage` 的 `test_rviz_config`：交叉检查 `coverage.rviz` 里配置的
  `climbot_*` 面板类名确实由本包声明，防止改名后面板静默消失。

排版靠肉眼确认时，用 `CLIMBOT_PANEL_SHOT` 让同一个用例把各宽度渲染成 PNG：

```bash
CLIMBOT_PANEL_SHOT=/tmp ./build/climbot_rviz_plugins/test_coverage_panel_layout
```

（离屏渲染，不需要显示器；WSLg 下截不了 RViz 窗口，这是唯一能看清面板的办法。）
