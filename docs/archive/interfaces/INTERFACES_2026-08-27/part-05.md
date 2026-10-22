# 历史分册 5

### 换道段的转向下坠预留

每个 `TRANSITION` 的**终点**都会沿反重力方向抬高一个预留量,使机器人在终点转向、
下坠之后正好落在下一条线的名义起点上。以前只有横向扫描做这件事,竖向按"下坠沿
扫描线方向、属于沿轨误差"豁免——**这个理由是错的**:线**起点**处的沿轨偏移不会被
跟踪器消掉,它把线截短了。实测 `3.30 × 4.50 m` 竖向矩形,每条**向下**的扫描列都
在距区域顶边 `46 mm` 处停止(正好一次 `90°` 转向的下坠),而向上的列全部越过顶边
——那里同样的下坠把起点往后推,只是多扫一点底部。

预留量按机器人**实际**会转过的角度算,不是名义几何:

| 项 | 取值 |
| --- | --- |
| 转向终态航向 | 下一条线航向 **+ 该线的重力前馈** |
| 转向初态航向 | **实走**换道段航向 **+ 该段的重力前馈** |
| 实走换道段 | 实际起点(已掉落)→ **抬高后**的终点 |
| 预留量 | `turn_slip_per_degree_m × 转角`,夹在 `[0, k × 180°]` |

抬高终点会让实走换道段更斜,从而改变终点转角,所以这是个不动点,用迭代解。收缩
因子约 `k × (180/π) / 换道段长`,`0.39 m` 时为 `0.074`,四次迭代到微米级。换道段
短于 `0.05 m` 时航向主要是位置噪声、迭代不再收缩,此时回退名义航向。

三处"实际"缺一不可。以矩形竖向的 `0.40 m` 换道段为例:名义转角 `90°`(预留
`45 mm`)→ 计入起点掉落和终点抬升后 `103.6°`(`51.8 mm`)→ 再计入换道段自身
`5.9°` 前馈后 `109.5°`(`54.7 mm`)。

**不再用实测下坠兜底。** 原先取 `max(模型值, 上一次转向的实测下坠)`,而"上一次
转向"和"要预留的这次转向"是两次不同的转向。梯形右斜边上机器人刚转过 `166°`
(掉约 `83 mm`),接着只需转 `14°`(该留 `7 mm`),这个下限会把预留抬到 `83 mm`,
多抬 `76 mm`。现在完全信任已标定的系数;`line_tracker` 改为每次对准都拿实测下坠
与模型比对,偏差超过 `50%` 时告警提示重跑 `measure_turn_slip.py`——标定过期会被
**报出来**,而不是被运行时悄悄补偿掉。

### 顶部收边扫描

由规划器在**发送 Goal 之前**追加，作为一条普通的 `SEGMENT_SCAN`；执行器不在运行
中热追加线段（§10.7、§11 执行规则第 7 条）。`top_edge_scan` 决定何时追加：

| 取值 | 行为 |
| --- | --- |
| `auto` | 仅当**预计**覆盖率低于显式正的 `minimum_nominal_coverage_ratio` 时追加 |
| `always` | 只要蓝色收边线能放进橙色任务可走区就追加 |
| `never` | 从不追加 |

收边蓝线位于橙色任务可走区顶边，相机足迹自然越过该边缘。进入方向取距离上一条扫描
线终点较近的一端，避免为了上线横穿整个区域。两个端点必须都在橙色任务可走区
内，否则不追加并在 `/coverage/status` 说明原因——宁可少扫，也不生成一个执行器会在
接受 Goal 阶段拒绝的任务。

**只对竖向扫描生效。** 横向扫描的最高一条扫描线已经按检测宽度布置，另加收边会
重复同一覆盖用途。`always` 配横向扫描时
不追加，并在状态里说明。

**`auto` 用的是预计覆盖率，看不到执行损失。** 默认模式为 `never`；部署显式配置
`auto` 和正数门限时，它才会在预计比例不足时尝试追加。实测覆盖不足时，
操作员可用 `always` 增加一条仍位于橙色可走区内的顶边扫描，或扩大可走区后重新规划，
而不是让机器人为追求规则覆盖边界越过限制。

## 阶段 E 冻结接口

### 公共接口包

新增 `climbot_interfaces`，只安装消息和 Action，不包含运行节点。规划器和控制器
都依赖该包，控制器不得依赖 `climbot_coverage` 的实现库。

### `CoverageTask.msg`

第一版字段冻结为：

```text
uint8 SWEEP_HORIZONTAL=1
uint8 SWEEP_VERTICAL=2

uint8 SEGMENT_SCAN=1
uint8 SEGMENT_TRANSITION=2
uint8 SEGMENT_RETURN=3

std_msgs/Header header
string task_id
uint32 revision
uint8 sweep_direction

geometry_msgs/Pose[] waypoints
uint8[] segment_types

geometry_msgs/Polygon coverage_region
geometry_msgs/Polygon motion_region

float64 detection_width
float64 detection_length
float64 detection_forward_offset
```

`header.frame_id` 是所有路点和两个 Polygon 的共同坐标系，默认 `odom`；
`header.stamp` 是该规划版本的生成时间。`N` 个 `waypoints` 必须对应 `N-1` 个
`segment_types`，其中 `segment_types[i]` 描述 `waypoints[i] → waypoints[i+1]`。
路点姿态指向下一段，最后一个姿态沿用到达航向。

`detection_forward_offset` 是检测中心沿机器人前向相对 `base_link` 的偏移，普通工具可
为 `0`，G2 前置相机任务为 `+0.340 m`。

接收方必须拒绝以下任务：

- `task_id` 为空、`revision` 为零或扫描方向非法；
- 路点少于两个，或线段类型数量不是路点数量减一；
- 坐标、四元数、检测尺寸／偏移包含非有限值，检测尺寸不为正或偏移为负；
- 存在零长度线段、未知线段类型或非法四元数；
- 线段类型为 `SEGMENT_RETURN`。该常量保留在消息里，只是为了编号不会挪动、不会让
  已录制的 bag 换个含义；但没有任何执行器为它定义过行为，收下它等于按默认分支去走一段
  谁都没定义、谁都没测过的轨迹。在任务边界上拒绝，意味着真要用它的人在启动时就知道，
  而不是从跑出来的形状里猜；
- Polygon 非法、`coverage_region` 不在 `motion_region` 内，或任一名义路点位于用户
  `coverage_region` 外；
- 显式配置了正数覆盖率门限，且 `SCAN` 对用户可走区达不到该门限。默认门限为零，只报告。

`coverage_region`、`motion_region` 和 `waypoints` 不带各自 Header，统一继承任务
Header，避免同一任务内部出现多个坐标系或时间戳。

### 任务话题

| 话题 | 类型 | QoS | 权责 |
| --- | --- | --- | --- |
| `/coverage/task` | `climbot_interfaces/msg/CoverageTask` | reliable、transient local、depth 1 | 已实现；最新完整任务预览，不直接热更新正在执行的任务 |
| `/coverage/path` | `nav_msgs/msg/Path` | reliable、transient local、depth 1 | 从任务路点派生，仅用于 RViz 和通用工具 |
| `/control/reference_path` | `nav_msgs/msg/Path` | reliable、transient local、depth 1 | E3 已实现单段直线参考预览；后续状态机在转向后以 EKF 实测起点更新动态参考，仅用于显示和评价 |

规划失败、清空或开始新区块时，规划器同时发布空 `/coverage/path` 和空路点的
`/coverage/task`，并增加 revision，使旧预览失效。空任务永远不能作为执行 Goal。
当前规划器以原始用户区域填充 `coverage_region`（任务可走区），以整面墙按机器人安全
边距内缩的多边形填充 `motion_region`（绝对安全区）。扫描方向上的蓝色端点直接取橙色
边界，不再为相机偏移向外延长；横向布线仍按检测宽度和重叠率均匀分配。黄色相机覆盖
由蓝线叠加 `detection_forward_offset` 和真实二维足迹得到。规划器报告橙区被预计拍到的
比例；`minimum_nominal_coverage_ratio=0` 为默认的“尽量覆盖”模式，只有显式配置正数
门限时才因覆盖不足拒绝。
竖向执行中的转向下滑属于实际轨迹问题，E9 将用同一足迹评估实际轨迹，并在需要时追加
一条顶部水平收边扫描。

### `ExecuteCoverage.action`

Action 名称为 `/coverage/execute`，E7 已实现。使用 ROS 2 Action 不表示接入 Nav2。

`evaluate_coverage_execution.py` 仍可订阅规划任务并充当测试 Action Client；正式操作
流程由 `coverage_manager_node` 在收到显式 `/coverage/start` 后发送 Goal。执行器已先
执行采集关闭的 `GO_TO_START/APPROACH_START`：校验当前点和首个路点均在凸
`motion_region` 内（因此其直线连接也在区域内），原地对准、直线到达、停车、再对准
首条扫描线。该进入段不计入覆盖采集或 `completed_segments`。

Feedback 已追加 `APPROACH_START=6`；进入阶段 `current_segment=-1`、
`segment_type=0`，用于与正式任务线段区分，不能误标为 `TRACK_LINE`。

Goal：

```text
climbot_interfaces/CoverageTask task
```

Result：

```text
uint16 SUCCESS=0
uint16 CANCELED=1
uint16 LOCALIZATION_TIMEOUT=3
uint16 CONTROL_TIMEOUT=4
uint16 OUT_OF_BOUNDS=5
uint16 TRACKING_FAILED=6
uint16 EXECUTOR_LOST=7
uint16 ARCHIVE_FAILED=8

uint16 result_code
string message
uint32 completed_segments
float64 elapsed_time_s
```

Feedback：

```text
uint8 WAITING=0
uint8 ALIGN=1
uint8 TURN_SETTLE=2
uint8 TRACK_LINE=3
uint8 FINAL_APPROACH=4
uint8 STOPPED=5
uint8 APPROACH_START=6

uint8 state
int32 current_segment
uint8 segment_type
float64 along_track_error
float64 cross_track_error
float64 heading_error
float64 remaining_distance
float32 progress
float64 planned_total_s
float64 schedule_lag_s
float64 estimated_remaining_s
```

时间三项与 `progress` 并列而不是合并进去，因为它们回答的是两个不同问题：
`progress` 说**做完了多少工作量**，机器人卡住时它正确地停住不动；后三项说
**跟不跟得上计划**，一个由 elapsed 驱动的进度条会在机器人卡死时照样走到 100%。

| 字段 | 含义 |
| --- | --- |
| `planned_total_s` | 任务开始时算定，全程不变，含到第一个路点的接近段 |
| `schedule_lag_s` | 正数为落后计划。`distance` 模式恒为 `0`——没有时间表就谈不上落后 |
| `estimated_remaining_s` | 每周期更新并带上当前累计滞后，所以落后时它变长而不是匀速递减 |

接近段在 `planned_total_s` 里计入，但**不进入 `progress` 的分母**——进度条数的是任务
的段，接近段不是其中之一。该段的转向和直线各自按自己的完成度递减，否则倒计时会
在整条腿上纹丝不动、到达时再一次性跳掉一整段。

`current_segment = -1` 表示尚未进入任何线段。误差单位分别为米、米、弧度和米；
`progress` 范围为 `[0, 1]`，只用于显示，不作为任务完成判据。
当前实现中 `along_track_error` 表示机器人相对本段起点的有符号沿轨坐标；
`remaining_distance` 才是到本段终点的沿轨剩余量。

执行器完整复制并校验 Goal，一次只执行一个任务。每段都经过
`制动 → 原地 ALIGN → 航向稳定 → 直线跟踪 → 空间到达并停车`，不会用运行时间替代
到达判据；取消、定位超时、单段超时或越界均先发布零速。执行时 `CoverageTask`
保持不变，但转向稳定后更新动态执行参考：横向换道采用实测起点，并在换道终点上方
预留第二次转向的预计下滑量；竖向换道采用实测起点到名义终点的斜直线，不倒车返回
名义点。首条扫描和第二次转向后的扫描使用同一入轨规则：法向偏差不超过 `0.045 m`
时直接冻结实测位置对应的平行扫描线；偏差在 `0.045～0.12 m` 时先执行一次带重力
前馈的前进小弧线，收敛到 `0.03 m` 后重新对齐并冻结平行线；偏差更大、超时、剩余距离不足或动态端点
越界时停车并终止任务。正式扫描线一旦冻结便保持为直线，不在跟踪中继续移动。
横向换道的预计下滑量取 `turn_slip_per_degree_m × 下一转角` 与第一次转向实测下坠
量的较大值；短上底梯形会因此自动适应斜向换道两侧不同的接触扰动。

“不出现明显蛇形”的判断不使用横轨误差过零次数作为单独故障条件：默认只对横向
幅度超过 `0.03 m` 且沿轨持续出现的往复做诊断告警。最终接受与否还要同时检查真值
横轨包络、航向摆幅和整段曲率；厘米级以内的小误差反复过零允许存在。

