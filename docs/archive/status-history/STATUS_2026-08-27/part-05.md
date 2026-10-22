# 历史分册 5

### 2. `test_coverage_manager.py` 对机器负载敏感（已修）

机器同时跑着 Gazebo GUI 时，该 launch 测试会间歇失败：一处是
`start_client.wait_for_service(timeout_sec=3.0)` 超时，另一处是紧随其后的用例读到
上一个用例的缓存任务。单独重跑必过，全量重跑也过。原因是 3 秒的服务发现窗口在
负载下不够，而这些用例共用一个已启动的管理器，所以第一处失败会把状态留给第二处。

2026-08-19 修复并双向验证：24 路满负载下旧版 17 次跑挂 3 次，两种失败模式都复现出来
且与上面的判断一致（`wait_for_service` 返回 `False`；缓存任务没被清掉导致
`assertFalse(no_task.success)` 反过来成立）；新版同样负载下 13 次全过。改了三处：
发现窗口与服务调用窗口放宽到 15 s / 10 s；每个用例自己把管理器推到它需要的状态
（用空预览清缓存、取消后等到 `FINISHED` 再返回，因为 `busy()` 期间管理器根本不发
`READY`）；每个用例用自己的 `revision`，且状态匹配只看本用例发起之后收到的消息——
状态话题是 transient local，否则订阅那一刻收到的旧状态就能直接把断言喂饱。

### 3. ~~§14 最终验收清单尚未逐条走查~~ 已完成

六个小节约六十条，绝大多数已有归档证据。2026-08-19 补掉了走查前发现的唯一实质空缺：
§14.5 的「融合位置误差应显著小于仅使用轮式里程计时的长期误差」原先只有脚本没有数据，
现已归档为 `localization_2026-08-19`（融合 `1.6~2.6 mm` 不增长，轮式航位推算累积到
`511.6 mm`，比值 `194 倍`）。2026-08-21 已在
[ACCEPTANCE.md](../../../ACCEPTANCE.md) 为六个小节逐条分配 ID、状态和证据。

其中有一件不是工程决定：§14.3 那张阈值表写明是"建议初值……待机器人实际尺寸和真实
数据确定后再冻结"。实测值离阈值都还有大余量（落点 `2.5 mm` 对 `30 mm`），按实测收紧
还是维持建议值，需要产品侧决定。

### 4. ~~README 的 RViz 截图早于"运行中冻结规划控件"~~ 已解决

**2026-08-19 已重截替换。** 旧图摄于 `21:27`，冻结那次提交在 `21:30`，所以图里
`State: Executing` 的同时 Region、Sweep、Replan、Clear points 仍是可点的，只有
Algorithm 是灰的，与 `OPERATION.md` 里"运行期间五个控件全部置灰"矛盾。

新图是执行中的梯形横向任务（`Timed trajectory`、`Segment 3 of 15`、进度 `21%`、
`Schedule` 滞后 `+0.01 s`），五个控件连同 Start 全部置灰，只有 Cancel / Stop 可点，
与文档一致。

### 5. 15 个旧归档标签不可追溯，且无法补救

`results/README.md` 的标签索引列出全部标签的可追溯性。2026-08-18 及以前的 18 个
标签里只有 `2026-08-17`、`timeA`、`ceilgate2` 三个产自干净工作树；当时被指定为
"当前正式基线"的 `2026-08-18d` 与"定版回归"的 `timeE` 都不在其中。

**已闭合的部分**：产出机制修好（回归脚本脏树时给标签加 `-dirty` 并告警，评价器写
`traceable` 并在每个工况结束时打印），且已在提交 `104caaa` 的干净工作树上重跑出
`2026-08-19`（distance）与 `2026-08-19t`（time）两组八工况基线取而代之，两者
`traceable=true`、commit 相同。

**无法补救的部分**：那 15 个旧标签事后没有办法知道当时的工作树是什么样，只能标注。
不过重跑同时也是一次检验——新旧数字落在同一散布带内，覆盖率多数工况逐位对齐，
说明当时那些未提交改动对结果没有实质影响。旧结果不可**引用**，作为参考仍可信。

### 6. ~~墙面工作系原点在底边中心，应移到左下角~~ 已完成（2026-08-20）

原点已移到墙面左下角。工作系现在是 `x ∈ [0, 10]`、`y ∈ [0, 8]`，**任何区域、结果或
日志里都不再出现负坐标**。可达绿框 `x ∈ [0.548, 9.452]`、`y ∈ [0.548, 7.452]`，
机器人出生在工作系 `(5.0, 2.0)`。

墙在世界系里没有动，尺寸也仍是 `10 × 8 m`：移动的只是工作系原点，由 `wall.yaml` 的
`origin_xyz` 从 `[0, 0, 0]` 改为 `[0, -5, 0]` 吸收。**这是一次纯粹的坐标平移，几何
没有任何变化**，所以十一个覆盖配置只把 `x` 加了 `5.0`，形状和尺寸逐项不变，
`§15.7` 与阶段 E 的直线工况完全未动，**归档基线不需要重跑**（详见
`results/README.md` 开头的坐标分界说明）。

过程中一度把墙面同时改成 `8 × 6 m` 并按新墙重画了全部区域，随后按要求改回
`10 × 8 m`。那一版的教训留在下面，因为它们与尺寸无关。

改动前的清单有三处是错的：

| 清单原文 | 实际 |
| --- | --- |
| `simulation.yaml` 的 `spawn.lateral_m` 要改成 `W/2` | **不需要改。** 它在 xacro 里是机器人出生位姿的**世界 Y**，不是工作系坐标；墙仍居中于世界 `Y=0`，`0.0` 依然是墙面横向中心 |
| `test_coverage_geometry.cpp` 要改硬编码坐标 | **不受影响。** 它用显式角点直接调 `makeRectangle` / `insetConvexPolygon`，从不经过 `motionRegion()` |
| 评价器自建工况不受影响 | **只对了一半。** 路点确实是机器人相对坐标，但 `evaluate_coverage_execution.py` 的 `_motion_region()` 写死了 `(-4.6, 0.4)..(4.6, 7.6)` |

清单完全没列到、实际必须改的：

| 位置 | 内容 |
| --- | --- |
| `climbot_coverage/rviz/coverage.rviz` | `Focal Point` 与 `Distance` 是按居中原点定的 |
| `coverage_planner_node.cpp:618` | RViz 墙面 marker 只设了 `pose.position.y`，原点移动后还要设 `position.x` |
| `simulation.yaml` 的 `wall.centre_xyz` | `z` 就是 `H/2`，墙底贴地全靠它（缩墙那一版才会触发） |
| `climbot_wall.sdf.xacro` 的默认值 | 没有任何测试保护，会静默过期 |
| `tools/run_coverage_regression.sh` | 缩墙那一版有 4 个直线工况会跑出墙外；改回 `10 × 8 m` 后原长度全部合用，最紧的一个余 `0.35 m` |
| 覆盖配置 | 是**十一个**不是十个 |

顺带修掉的两处既有缺陷：`climbot_wall.sdf.xacro` 的 `suction_force` 默认值仍是
`220.0`（权威值早已是 `400.0`）；评价器的 motion region 内缩 `0.4` 而规划器是
`0.548`，两者一直对不上，现已改为按 `wall.yaml` 与 `robot.yaml` 推导。

新增 `climbot_gazebo/test/test_wall_world_matches_description.py` 钉住这一类失配：
世界 xacro 的每个默认值必须等于 launch 注入的值，`simulation.yaml` 的墙体中心必须
放置 `wall.yaml` 描述的那个面，墙底必须在 `y = 0`，原点必须是左下角。

**一个被这次改动掀出来的既有缺陷（已修）。** 启动后 RViz 里机器人会先出现在墙面左下角
原点，十几秒后才跳到真实位置。实测原因不是全站仪慢——它在 `sim t = 0.05 s` 就给出了
正确的坐标——而是 EKF 在**拒绝**它：`initial_estimate_covariance` 用的是
robot_localization 的默认值（对角 `1e-9`），等于宣称"我确信自己在原点"，于是
`pose0_rejection_threshold: 5.0` 的马氏距离检验把第一帧观测当野值丢掉，直到滤波器自己的
`x` 方差涨到能接受为止。

| | 收敛到观测 `0.10 m` 以内 |
| --- | ---: |
| 修改前 | `12.86 s` |
| 修改后 | **`0.10 s`** |

**这个缺陷一直存在，只是被掩盖着**：旧工作系的原点恰好在出生点正下方，`x` 残差为零。
修法不是放宽野值门限，而是把位置的初始方差改为 `100`——开机时滤波器**确实**不知道自己
在哪，第一帧观测一到方差就塌到 `0.002`。`test_ekf_startup.py` 按"墙面对角线那么大的残差
也必须被接受"来钉住它，与出生点无关，所以将来再动原点不会重犯。

顺带修掉一处解析隐患：矩阵里的 `1e-09` 不带小数点，PyYAML 按**字符串**解析而 ROS 的
`rcl_yaml_param_parser` 按浮点，同一文件两个解析器读出不同类型。已统一写成 `1.0e-09`。

**`docs/images/rviz_coverage_task.png` 已过期**：图中的坐标和绿框属于旧工作系。
`gazebo_wall.png` 不受影响，墙面在世界系里没有变。（已于 2026-08-20 重拍，见
「README 时长口径与 RViz 截图」。）

## 2026-08-20 拆出控制器的三块边界（review M5）

`line_tracker_node.cpp` 上一轮 review 时 1741 行，本轮涨到 1788，单类同时负责参数、
Action、8 态运动控制、调度估计、动态参考、反馈和安全终止。上一轮决定"下次动这个文件时
顺带拆"，本轮已多次触及却没执行。

按 review 建议的三条边界抽出三个可单测对象，**只搬不改算法**：

| 新对象 | 管什么 | 原来在哪 |
| --- | --- | --- |
| `ScheduleEstimate` | 任务该花多久、花掉多少：进度条、倒计时、`planned_total_s` | `planSegmentDurations` / `taskProgress` / `startApproachRemaining` / `estimatedRemaining` 及五个成员 |
| `ScanEntry` | 扫描线顶端的三个判断：机器人相对线在哪、转完后的偏移怎么办、首条扫描线进不进得去 | `preparePostTurnScan` / `firstScanEntryFits` / `warnIfTurnSlipLooksStale` 里的几何部分 |
| `SegmentArrival` | 一段什么时候算到了：双阈值迟滞 + 静止判据 + 稳定时间 | `updateGoalCompletion` 与 `goal_settle_start_` |

这三块的共同点是：**它们决定了每一次运行的形状，而唯一能验证它们的方式是跑一整个任务**。
现在各有一组直接提问的单元测试，共 25 项——`ScheduleEstimate` 8 项、`ScanEntry` 10 项、
`SegmentArrival` 7 项。几个此前完全没有覆盖的点：进度按耗时而非段数加权、起点进入是
逐渐倒计时而非到达时整块消失、沿重力方向的扫描线把转向下坠吃在自己长度里所以不占偏移
预算、双阈值之间的漂移保留稳定计时但不能凭此判完成、以及以速度穿过终点不算到达。

搬迁过程中发现并保留了一处原有语义：`updateGoalCompletion` 在"进了松阈值、出了紧阈值"
时返回 `false` 而**不**清计时器。第一版重写把它写成了"计时够了就完成"，是行为改变；已
按原样恢复，并在 `SegmentArrival` 里把这条写明——两个阈值band 是为了让停在紧阈值边缘的
机器人既不会永远重新计时、也不会从松阈值外面判完成。

`line_tracker_node.cpp` 1788 → 1745 行。行数不是重点，**编排能被直接提问了**才是。

宿主全量：**643 tests，0 error，0 failure，39 skipped**（原 588，新增 55）。

**八工况基线两种控制律各复测一遍**（`commit=84c28fc` 干净树，标签 `2026-08-20` /
`2026-08-20d`），16 个工况全部 `passed=true`，与上一版干净树基线逐项对比**没有系统性
偏移**：变化都在既往逐次波动范围内且方向不一致，覆盖率逐工况相同到 `0.02` 个百分点
以内，时间点控制的实际/计划比仍在 `0.99~1.03`。逐项对比表见 `results/README.md`
「控制器拆分后的复测」，两组基线已在该文件里替换为当前正式基线。

第一次对比时我拿 time 模式的新结果去比了 `2026-08-19`（distance），数字碰巧也接近，
但那不是同一件事的对比；已按同模式对同模式重做。

复测顺带验证了 M2 的两件事。一是中断清理：第一次启动方式不对，脚本连同 trap 一起被
外部杀掉，清理跑到一半停了，留下两个孤儿节点——这恰好说明 trap 本身是生效的。二是
`lane_remember` 那条"不记录脚本自己的进程组"的保护，本来是照着"万一 `setsid` fork 了"
写的，结果在 distance 那轮真的触发了一次（`[lane3] pid 138354 did not get a session of
its own; not recorded`）。没有它，那一刻的 teardown 会对脚本自己的进程组发 `TERM`，
把四条 lane 一起带走。

## 2026-08-20 Apache-2.0 文件头（review L7）

七个包的 `package.xml` 都写着 `<license>Apache-2.0</license>`，根目录有完整的
`LICENSE`，但**没有一个源文件带许可头**，而且七个 `CMakeLists.txt` 全都写着
`set(ament_cmake_copyright_FOUND TRUE)` 把版权检查显式关掉了。公开仓库里这是声明与
事实不一致。

给 `src/` 下全部 100 个源文件（`.py` / `.cpp` / `.hpp`）和 `tools/` 下 7 个脚本加上
标准 Apache-2.0 头，`#!` 行保持在最前；删掉七处 `ament_cmake_copyright_FOUND`，让
`ament_copyright` 重新参与每次 `colcon test`。测试数从 481 升到 **588**，多出来的
107 项就是重新打开的版权检查。

顺带修掉一处被这次改动暴露的测试污染：`test_cmd_vel_watchdog_node.py` 里我新加的保持
测试和原有的超时测试共用一个看门狗节点，前者的驱动线程停下后没等系统静下来，残留的
指令会被后者当成自己的。现在保持测试结束前先解除保持、再等输出回落到零速。连跑三次
确认。

## 2026-08-20 低严重度小项打包（review L2~L6、L8）

- **L2 规划器只比大小，不查有限性。** `validatePhysicalParameters()` 的每条检查都是
  比较，而 NaN 是**全部比较都为假**而不是其中之一为假，`detection_width <= 0.0` 对它
  是假，于是坏参数直接通过。实测 `detection_width:=.nan`：节点照常启动、什么都规划不出、
  发空任务、报覆盖率 `0%`。失败方向是对的（fail closed），但它表现成规划故障，会把人
  引去看区域几何，而不是看自己敲的那个数。新增 `requireFinite()`，覆盖九个物理参数、
  网格间距和两个点参数；`test_planner_rejects_bad_numbers.py` 用七个参数逐个验证：
  进程非零退出，且消息里点名是哪个参数。
- **L3 三个传感器脚本同样只查正负。** `position_stddev_m`、IMU 姿态噪声、轮速协方差
  的 NaN 能绕过校验进入消息和协方差矩阵。抽出 `climbot_gazebo/parameter_checks.py` 的
  `require_finite()`，三处统一调用。
- **L4 贴图 manifest 只检查文件存在。** 不校验 scale、尺寸、块是否落在墙内，路径还直接
  拼进 XML。现在 `load_manifest()` 在发出任何 visual 之前把这些全查一遍——因为另一种
  发现方式就是看渲染图，而那正是工作系原点 bug 的发现方式：块整体偏了半面墙、一半墙面
  没贴图、加载过程一声不吭。路径改为 XML 转义：带 `&` 或尖括号的目录（谁给目录起的名字、
  或 WSL 下看到的 Windows 共享）会让 `minidom.parseString` 在整个世界文件上失败，而报错
  指向的是烘焙写出的那一行。两个新测试。
- **L5 包元数据不自洽。** `climbot_rviz_plugins` 用了 `std_msgs`/`rcl_interfaces` 却
  没声明；`climbot_control` 和 `climbot_gazebo` 也直接用 `rcl_interfaces`。当前靠传递
  依赖能构建，但那不是依赖关系。已补齐 `package.xml` 与 `CMakeLists.txt`。
- **L6 `standalone_mode` 下运行中也能切控制律。** 那条拒绝的注释写的是"只能在任务之间
  切换"，但判据只有 `active_goal_`；`standalone_mode` 从来没有 Goal，却从拿到位姿那刻起
  就在走配置好的那一段。改为 `isDrivingASegment()`，两种情况一起管。
- **L8 `SEGMENT_RETURN` 验证器收、执行器没语义。** 常量保留在 `CoverageTask.msg`（编号
  一挪，已录制的 bag 就换了含义），但 `validateCoverageTask()` 现在明确拒绝它并说明原因：
  收下它等于按默认分支走一段谁都没定义、谁都没测过的轨迹。

宿主全量：**481 tests，0 error，0 failure，30 skipped**。

## 2026-08-20 清掉不可追溯的过程轨迹（review M4）

`results/` 87 MiB / 373 文件里，172 份摘要中 **92 份 `source_modified=true`**，对应
`52.8 MiB` 压缩轨迹。上一轮补的标签索引和 gzip 压缩解决了"看不懂"和体积增速，没解决
`AGENTS.md` 自己的规则：本目录只保存正式、可复现的验收证据。

按"删轨迹、留摘要"处理：删掉 89 个脏树 `_trajectory.csv.gz`，`results/` 降到
**34 MiB / 284 文件**。摘要全部保留——索引表和文档里的数字都指着它们，删掉摘要等于把
`gainP*`、`timeA~D`、`ceilgate` 这些过程结论变回"不知道这数字哪来的"。干净树的 80 份
摘要和 79 份轨迹一份没动。

删之前逐个核对过：89 个文件没有任何一个被仓库里的其他文件按名字引用。

