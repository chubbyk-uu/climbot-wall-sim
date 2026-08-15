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
| State | `CoverageStatus.state` |
| Task | `task_id` 与 `revision` |
| Segment | `current_segment + 1` / `total_segments`；接近首点期间显示 `approach of N` |
| Progress | `progress`，百分比进度条 |
| Manager | `message`，与管理器日志同一行 |
| Planner | 规划器的 `/coverage/status`：点击被接受的坐标、规划失败原因、清空确认 |
| Last request | 最近一次按钮调用的服务响应 |

规划失败与"未选择区域"在管理器看来都是空任务，无法区分，都会报 `Idle`。
真正的原因只在 Planner 一行。

按钮：

| 按钮 | 服务 |
| --- | --- |
| Replan | `/coverage/replan` |
| Clear points | `/coverage/clear_points` |
| Start | `/coverage/start` |
| Cancel / Stop | `/coverage/cancel` |

`Start` 与 `Cancel / Stop` 的可用性直接取自 `CoverageStatus` 的 `can_start` /
`can_cancel`——由管理器按自己服务的前置条件计算，面板不做推断。面板自行从
`state` 推断正是"取消后无法重新开始"那个 bug 的成因。

`Replan` 与 `Clear points` 始终可用：它们是规划器的服务，只改预览，不影响正在
执行的任务，管理器无权替其决定，面板更无权。

这些都是提示而非校验：无论面板显示什么，非法请求都由管理器或规划器拒绝，并把
原因显示在 Last request 一行。

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
- `climbot_coverage` 的 `test_rviz_config`：交叉检查 `coverage.rviz` 里配置的
  `climbot_*` 面板类名确实由本包声明，防止改名后面板静默消失。
