# 故障案例：TESTING

入口与新增规则见[故障索引](../INCIDENTS.md)。

### 1. ROS domain 冲突把另一用例的数据当成自己的

- **现象**：规划器测试偶发读到只有两个路点的任务，像是路径算法退化；相关用例单独运行正常。
- **隐蔽性**：两个 launch test 只有并行且生命周期重叠时才相互发现；错误数据本身完全合法。
- **误导线索**：失败断言落在路径长度和几何内容上，不会直接报告 domain 冲突。
- **根因与修复**：`test_coverage_manager_pause_loss` 使用了属于另一个包的 `ROS_DOMAIN_ID=61`。
  重新划定各包 ID 区间，并由工作区一致性测试静态检查全仓唯一性。
- **回归**：`src/climbot_bringup/test/test_workspace_consistency.py`；相关提交 `748d386`。
- **经验**：跨进程测试收到“结构正确但内容不属于本用例”的消息时，先查隔离，不先改算法。

### 2. 固定睡眠不是 DDS discovery 完成条件

- **现象**：自动采集测试偶发 `capture_calls=0`；归档管理器第一次 Start 偶发报告 Action server
  不可用。
- **隐蔽性**：机器空闲时 `sleep(0.3)` 常常够，负载或 `-j8` 下 discovery 尾部才越过它。
- **误导线索**：看起来像节点没触发、服务预算错误或业务状态机拒绝，实际消息根本没送到。
- **根因与修复**：fixture 用固定睡眠猜测发现完成。发布者改为等待匹配订阅者，ActionClient
  显式等待 server；超时仍保留，以便真实连线错误明确失败。
- **回归**：`test_automatic_capture_node.py`、`test_coverage_manager_archive.py`；提交
  `ff41a4c`、`fbeef21`。
- **经验**：测试前置条件必须等待可观察状态，不能用“通常足够久”的睡眠代替。

### 3. 把循环次数当秒数，既会过早失败也会无限拖长

- **现象**：同一次 `-j4` 中两个 case 报缺结果，第三个挂到 ctest 90 秒上限；正常只需约 3.6 秒。
- **隐蔽性**：`spin_once(timeout=0.1)` 有消息时立即返回、负载高时又可能超过 0.1 秒，因此
  `deadline -= 0.1` 与真实经过时间没有固定关系。
- **误导线索**：早退像发布太慢，超时像进程卡死，两者看似相反，实为同一计时错误。
- **根因与修复**：用 `time.monotonic()` 的绝对截止时刻替代迭代计数，所有同型等待一起修正。
- **回归**：`test_coverage_top_edge_scan.py`、`test_coverage_configure.py`；提交 `c48255c`。
- **经验**：任何跨线程、DDS 或 I/O 的等待都只能用真实时钟度量，不能假设一次循环等于 timeout。

### 4. `-j8` 失败不能归类为“时间噪声”

- **现象**：降低为四并发后看似稳定，恢复八并发又出现不同用例的低概率失败。
- **根因**：后来逐一找到 domain 冲突、Goal handle 析构、discovery、错误 deadline 和归档迟到
  状态等真实缺陷；没有一项只是“机器慢”。
- **处理**：默认恢复 `--parallel-workers 8`；低频失败必须有根因、确定性回归或明确运行判据，
  不能用重跑通过关闭。提交 `f77624f` 汇总了这次口径修正。

## 二、Action、服务与迟到消息

### 17. 复用节点却重置测试时钟，失败断言还会留下活跃任务

- **现象**：吸附 S0 审计的既有执行器 launch 测试中，某次目标刚接受便报里程计过期，
  巡航采样为零；另一次前一用例失败后，下一用例被拒绝，提示仍有活动任务。
- **误导线索**：把零样本归为主机负载、把下一用例的拒绝归为 Action 接收竞态，或者以单跑
  通过关闭问题；这些不能解释日志中的旧里程计时间戳和新时钟代次交错。
- **根因**：整份 launch 测试共用一个 tracker 进程，但每个 Python 测试客户端将 `/clock`
  从零开始；五次发布也不等于新发布者与订阅者已完成 discovery。与此同时，巡航速度断言
  在取消 Goal 之前执行，断言抛出后只销毁客户端，未等待活动任务终止。
- **修复**：提交 `c09e6be`。夹具跨客户端生命周期延续仿真时间，等待实际订阅匹配；时间戳
  统一通过整数纳秒拆分，避免浮点近整数时产生 `nanosec=1e9`。巡航用例在 `finally` 中
  取消并等待终态。不修改控制器、仿真物理或运动门限。
- **回归**：`test_clock_epoch_survives_recreating_the_test_client` 真正销毁并重建 ROS 客户端，
  验证同一 tracker 的时钟连续；`test_cruise_assertion_failure_still_retires_the_goal` 故意
  触发断言，验证服务器已返回取消终态。仅回退时钟继承或异常清理的独立负对照必须失败。
- **边界**：初次审计还出现过 `0.210464 m/s > 0.205 m/s` 的速度断言；其直接原因尚未独立
  证明，不能仅凭上述修复后的连续通过就声称这一观测已定位。完整运行记录与该未关闭项见
  [S0 报告](../../results/suction_s0_2026-10-17/REPORT.md)。
- **后续专项**：现已受控复现并修复“当前计划对比旧时间戳位姿”导致的同类超速，见第 21 项。
  原次观测未保存逐包时序，仍不能将其唯一归因于该机制；这属于历史证据缺口，不影响新回归
  对已确认缺陷的关闭。旧报告不改写。

### 18. 测试客户端发现 Action server，不代表管理器内部也已发现

- **现象**：`-j8` 的强制恢复用例偶发首次 `/coverage/start` 返回 `success=false`，但任务预览
  已为 READY，测试客户端也已通过 `wait_for_server()`。管理器与执行器都没有崩溃。
- **根因**：两个独立的 Action 客户端异步完成 DDS discovery。READY／`can_start` 描述任务与
  忙闲状态，不是管理器内部 Action 客户端的发现屏障。源码的其他拒绝分支（无任务、忙碌、
  巡检归档未就绪）在该运动专用测试的首次 Start 中均不成立。
- **修复**：提交 `3429e17`。先在完全没有执行器时明确验证“Action server unavailable”拒绝，
  再启动执行器，仅对同一种**尚未发送 Goal 的拒绝**做有期限等待／重试，直到管理器接受。
  其他失败立即报错，不重发已经接受的 Start，不用固定睡眠猜测发现完成。
- **回归**：`test_operator_can_recover_without_calling_an_unknown_goal_finished` 包含上述完整
  未就绪→就绪流程，然后仍验证请求未答复窗口的 SIGKILL、强制放弃与人工重新使能。
  产品服务逻辑和超时门限均未修改。

- **S3 归档夹具补查**：另一份 `test_coverage_manager_archive.py` 首个 Start 在 -j8 下
  返回 false。原断言没记录 response.message，不能再区分 Action 与 archive 两个未就绪
  端点；单跑连续通过也不构成根因证明。代码核对确认其仅等待测试客户端发现 executor，
  仍缺管理器自身依赖的发现屏障。夹具现只对 `requestStart` 在任何状态变更／预检／Goal
  之前返回的两种明确 unavailable 拒绝做有期限等待，其他拒绝立即失败，接受后绝不重发。
  `TestStartDependencyDiscovery` 用确定性响应序列验证两端点、接受后停止、其他失败和
  到期不派发；不把原观测写成某个端点已被实测定位。该测试及最终 -j8 复验另见 S3 记录。
- **拒绝诊断补齐**：提交 `c44b318` 为两种 Start 服务入口记录完整拒绝原因，未改变服务行为。
  强制恢复测试在无执行器时明确断言生产日志中的 unavailable 原因；原次未记录的端点仍不可
  追溯。新证据见[时戳专项](../../results/cruise_timestamp_2026-10-17/REPORT.md)。

- **S5 同根因补查**：首次完整 -j8 的 pause 夹具首个 Start 拒绝，生产日志明确给出
  `Coverage executor Action server is unavailable.`，而不是业务状态拒绝。该夹具仍缺少管理器
  自身发现屏障。提交 `9f3da0f` 添加仅对这一派发前明确拒绝有界等待的
  `_start_after_executor_discovery`；其他拒绝立即失败，接受后不重发。
  `TestPauseExecutorDiscovery` 的三项确定性测试覆盖这三种边界，完整 1598 项 -j8 零失败。
  首次失败日志名及最终日志名见 [S5 regression](../../results/suction_s5_2026-10-18/regression.json)。

### 19. 原生 Gazebo 测量夹具在 reset 后不再执行 PreUpdate

- **现象**：新吸附模型的首次运行和暂停正常，reset 后整车动量观测报非主连杆速度缺失。
  给观测组件补一次初始化仍复现，不能据此判定球轮、压力核心或 Mesa 失效。
- **根因**：gz-sim 8.15.0 的 TestFixture helper 是内存系统、未实现 `ISystemReset`。
  SystemManager reset 会移除这类无法重建的系统；旧并行 PostUpdate worker 还可能调用
  已有 observer，而 PreUpdate 集合已没有它。故后续反馈申请不再执行，测量链先失效。
- **修复**：S3 原生测量器使用自行持有且实现 `ISystemReset` 的观测系统，reset 时重新
  申请反馈，另存初始压力／转速／夹具代次快照，继续完整逐步观测。源代码与官方
  [SystemManager::Reset](https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8_8.15.0/src/SystemManager.cc)
  的明确拒绝分支一致，不通过增加等待或编造零速度绕过。
- **回归**：`test_vacuum_native.py::test_native_observer_and_vacuum_survive_reset` 分别实际
  运行 B/C，验证暂停、reset、重新抽气、支撑撤除及 4000 个有效物理样本。
  修复提交 `9a51f50`；失败短测及重启后的正式证据另列在 S3 报告。

### 20. 等待首个采集心跳时反复发布参考，污染后续“无心跳”断言

- **现象**：S3 的 -j8 诊断第 6 轮中，`test_noninspection_reference_does_not_heartbeat_the_next_scan`
  期望空列表，却收到 6 个 segment=13 的 `inspection capture in flight`，下一段为 14。
  后续两轮通过仍不能据此忽略该失败。
- **根因**：夹具每 10 ms 重发有效段 13，直至异步收到第一个 gate；清空 Python 列表
  不等于排空 DDS 中已发布的参考／gate。生产节点只在有效 onReference 中发心跳，
  无效段 14 清空 reference 后立即返回；迟到 13 来自此前队列，不能当作它给 14 续命。
- **修复**：两项严格“随后不再有心跳”的测试，在已确认可靠 reader 匹配后只发布一次
  有效参考，等待其唯一对应心跳形成观测屏障，再开始无效参考阶段。保留 `gates == []`
  及原时间窗口，不屏蔽旧段消息、不延长固定睡眠、不修改产品心跳／监督期限。
- **回归**：`test_camera_mount_mismatch_withholds_the_gate_heartbeat` 与上述
  `test_noninspection_reference_does_not_heartbeat_the_next_scan` 的实际节点测试，以及
  修复后的完整 -j8 套件。修复提交 `fe3cc38`，运行记录见 S3 报告。

### 22. APPEND_ENV 拼接 ROS_DOMAIN_ID，破坏测试域隔离

- **现象**：S4 新增真实桥接测试在单跑时通过，完整 -j8 下却在 `rclpy.init()` 报
  `ROS_DOMAIN_ID is not an integral number`，尚未进入压力／风机验证。
- **误导线索**：归为 DDS discovery、Gazebo／Mesa 并发失败，或用再次单跑通过关闭；
  实际错误发生在 ROS 上下文初始化之前。
- **根因**：新 CMake 使用 `APPEND_ENV ROS_DOMAIN_ID=96`。ament 按路径列表规则拼接：
  继承空值时产生 `:96` 并明确失败；继承 17 时产生 `17:96`，本机 RCL 取成 17，
  即使初始化成功也丢失预期的域隔离。该参数必须覆盖，不能追加。
- **修复**：提交 `c2ef15c` 改为 `ENV ROS_DOMAIN_ID=96`，新增失败诊断日志保存到独立
  临时文件；仓库域检查禁止这种 APPEND_ENV 用法，并继续核对测试域唯一性。
- **回归**：`test_launch_test_domain_ids_are_unique_across_the_workspace` 和 B/C
  `test_pressure_bridge_and_all_live_fault_inputs`；显式继承 `ROS_DOMAIN_ID=17` 的
  完整 -j8 为 1590 tests、0 errors、0 failures、79 skipped。实际 ament wrapper 的
  两种负对照／覆盖正对照见 [S4 证据](../../results/suction_s4_2026-10-18/domain_environment.json)。
  第一次失败未保留充分诊断，补强日志后完整复现相同初始化错误；不靠猜测消息队列归因。

### 24. 状态观察窗口没有维持合成里程计，触发真实过期保护

- **现象**：S6 完整 -j8 中，“运行中清空选点”得到 FINISHED 而非 EXECUTING。
  日志明确记录 `Filtered odometry is unavailable or stale`，不是清空选点使管理器回到 IDLE。
- **误导线索**：归为负载下偶发抖动、放宽状态断言，或增加产品的里程计超时。
- **根因**：夹具仅在轮询服务／状态时发送里程计；两个固定睡眠观察窗口中传感器断流。
  原失败中最后有效位姿到停机约 2 s，恰为夹具配置的保护期限；调度使观察窗口超过期限。
- **修复**：本条所属提交只修夹具：观察状态时持续发布带新时间戳的里程计，并将观察区间
  设为 2.2 s，主动覆盖原 2 s 期限。产品保护、状态／按钮许可断言均不变。
- **回归**：受控将原两个无输入窗口延长至 2.2 s，稳定复现 2 failed；修复后同样
  区间两个状态断言通过。见 `test_coverage_manager_states.py` 及
  [红绿记录](../../results/suction_s6_2026-10-18/state_fixture_regression/summary.json)。

### 28. 状态夹具在新传感器端点匹配前凭缓存位姿启动

- **现象**：完整 -j8 的新预览状态断言得到 FINISHED，日志显示新用例没有新的里程计回调，
  上一用例回调约 2 s 后触发保护；观察期间夹具实际仍在发布，不能套用案例 24 的固定睡眠原因。
- **误导线索**：单跑绿就忽略、放宽两秒看门狗，或将服务发现等同于传感器就绪。
- **确认的夹具缺口**：每个用例重建发布端，却没有检查输入订阅匹配／送达。受控保持输入端点
  3 s 不匹配时，上一用例的有效位姿能让动作接受，随后因无输入过期；旧夹具稳定复现相同断言。
  原次没有端点时序采样，底层 DDS 触发原因仍未唯一定位，不把受控复现写成原次逐时证明。
- **修复**：本条所属提交在夹具 `_reach_executing` 中先确认输入匹配与可靠消息送达，再启动；
  产品控制保护、任务状态断言和 2.2 s 持续输入观察窗口均不变。
- **回归**：`test_a_restart_waits_for_the_new_sensor_endpoint`，同一真实执行器／管理器下
  移除屏障为 7 tests／1 failure，加入后 7 tests／0 failures；随后执行完整 -j8 重复回归。
  [红绿记录](../../results/suction_s6_2026-10-18/appearance_checkpoint/sensor_endpoint_regression.json)
  及[完整检查](../../results/suction_s6_2026-10-18/appearance_checkpoint/commit_checks.json)。

## 37. 退休模型的哈希守卫测试误读当前 main

- **现象**：墙面专项第一次完整 `-j8` 中，历史吸附输入守卫的正样本报四个“未批准变更”；
  涉及文件在本专项开始前已是 main 的新默认，当前专项没有修改其物理参数。
- **误导线索**：认为必须放宽旧输入哈希，或把新机器人恢复成旧配置才能使测试通过。
- **根因**：`copy_frozen_contract` 把当前 main 复制为历史正样本；S7 迁移默认后，这一前提
  不再成立。夹具还依赖未发布的本机 S0 附件，新克隆不具备相同条件。
- **修复**：本条所属提交将两份确切的历史源文本及已审阅 GUI/UDP 编辑保存为测试文本；
  使用前核对原始/批准 SHA256，其余身份路径使用相同的合成字节。生产守卫及批准哈希不变，
  不恢复旧模型入口；这不是对新模型物理的再次验收。
- **回归**：`tools/test_suction_analysis.py` 的精确批准正样本及三条追加变更拒绝用例；
  初次失败、修复后定向 46 项通过及最终完整八 worker 结果见
  [墙面专项报告](../../results/wall_texture_randomization_2026-10-18/REPORT.md)。

## 41. 分叉试片重复量规互撞，接触误记为墙面误差

- **现象**：真实裂缝试片在主裂缝中点出现约 0.196 mm 的接触高度偏差；扩大近似试片量规
  后出现更多异常。初看像窄网格或 ODE 接触故障。
- **根因**：主裂缝中点与分叉起点是同一位置，生成器重复放置独立球形量规；旧评估器
  将量规间接触也与墙面源场比较。日志中的双方碰撞名称确认稳态有 202 条双向互撞记录。
- **修复**：当前 M2 工作树的试片生成器去重位置、拒绝相交量规；评估器要求所有接触的
  另一方都是表面。近似试片采用分散位置，不将密集窄缝量规直接放大成互相穿透的球。
- **回归**：`test_proxy_fixture_deduplicates_branch_start_and_preserves_role` 与
  `test_evaluator_rejects_contacts_between_gauges`；去重后裂缝高度误差约 0.00086 mm，
  该旧高度异常归因撤回。真实网格的尖角法向问题仍未通过，不被本修复掩盖。
  证据见[裂缝定位报告](../../results/m2_t0_crack_bridge_2026-10-19/REPORT.md)。
### 43. 无响应 hold 的测试没有先建立故障条件

- **现象**：完整 `-j8` 中 `test_coverage_manager_hold_release` 偶发直接进入执行，随后等待 STARTING 超时。
- **隐蔽性**：测试先发布 transient-local hold 状态就调用 Start，DDS 接收顺序不能证明管理器已消费状态。
- **误导线索**：表象像取消或 deadline 失效，实际故障注入尚未建立，管理器没有进入待释放状态。
- **根因与修复**：用明确已存在但不响应的 `/control/hold` 服务构造未完成 release，单独 callback group
  隔离阻塞，teardown 主动解除服务等待。不修改生产控制器或延长验收门槛。
- **回归**：原用例继续验证 STARTING、取消／deadline 后恢复锁定、rearm，以及从未发送运动 Goal。


### 53. 系统时钟每约 32 秒被校一次，控制节点测试偶发提前结束

- **现象**：`test_coverage_manager_states` 在完整 `-j8` 中偶发失败：任务开始后不到 1 s 管理器
  报告 FINISHED，跟踪节点日志为“Filtered odometry is unavailable or stale”和“ROS time moved
  behind the last odometry stamp”；单独重跑三次全部通过。
- **隐蔽性**：日志时间戳在失败处倒退约 0.8 s；`journalctl` 当天有 245 次“Clock change
  detected”，间隔约 32 s，失败时刻与其中一次同秒。WSL 的 Hyper-V 宿主校时与
  `systemd-timesyncd` 同时工作、互相拉扯；只有恰好跨过一次回拨的用例才受影响。
- **误导线索**：先怀疑同批次的起点接近修复或提速；该用例起点即在首个航点上，不经过接近段。
- **根因**：10 个启动跟踪节点的测试使用系统时间；跟踪节点比较里程计时间戳与 ROS 时间，
  时间倒退时按设计判定位姿无效并停止。管理节点的计时已经使用 steady clock，不受影响。
- **修复**：这 10 个测试经 `fixtures/steady_sim_clock.py` 以单调时钟发布 `/clock`，被测节点与
  测试节点均用仿真时间；测试本身仍按真实时间节奏等待。未关闭 NTP，也未放宽跟踪节点对
  时间倒退的保护（真车遇到校时回跳时停车同样是安全行为）。同另外两个项目的规则：
  定时与看门狗不用可回拨的系统时钟。
- **回归**：改后连续两次完整 `-j8`（1951 项）零失败。

