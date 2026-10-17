# 隐蔽故障与排查经验

更新：2026-10-17。本文记录开发中已经确认、但不容易稳定复现或很容易被误判的问题。它不是
操作手册，也不重复当前接口；正常使用见[根 README](../README.md)，当前状态见
[STATUS](STATUS.md)，长任务运行判据见[时序稳定性](LOCALIZATION_GAP_PLAN.md)。

每条记录固定回答六件事：看到什么、为什么难复现、哪些线索会误导、根因是什么、怎样修、
什么防止它回来。提交号用于追溯完整差异；测试名是当前最接近根因的回归入口。

## 快速索引

| 类别 | 典型表象 | 首先检查 |
| --- | --- | --- |
| 并行测试 | 单跑通过，`-j8` 偶发少消息、错误任务或超时 | ROS domain、DDS discovery、真实墙钟 deadline、迟到回调代次 |
| Action/服务 | 退出时 SIGABRT，或超时后旧请求改写新状态 | Goal handle 生命周期、Future 与 topic 是否一起退休 |
| 控制时序 | 约 30 秒一次停发、恢复瞬间冲速、采集先停而机器人仍走 | steady/ROS time、暂停的“期限”与“运动曲线”、跨包超时不变量 |
| 图像采集 | 单帧偶发等待数秒、错配下一次曝光、最终少图 | RTPS 分片、请求关联身份、归档最终计数 |
| 数据证据 | 摘要通过但实际什么都没量，或 provenance 自相矛盾 | 非真空守卫、冻结输入哈希链、逐阶段代码状态 |
| 拼接光学 | 图像彼此对齐但整墙尺度错，或校正后仍有亮度碗形 | 被拍平面、材质/着色路径，而不只是内参与标定日期 |

## 一、并行测试与 ROS 发现

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

### 5. 活跃 Goal handle 在析构中发布结果并终止进程

- **现象**：`line_tracker_node` 大约三次中一次在退出时 SIGABRT，日志含
  `Asked to publish result for goal that does not exist`。
- **隐蔽性**：只在 shutdown 与 Action server 退休顺序撞上时发生；出错点在析构而非业务调用栈。
- **根因**：若活跃 `ServerGoalHandle` 未进入终态就被释放，其析构会尝试取消并发布；server 已
  退休时异常逃出析构，触发 `std::terminate`。
- **修复**：所有释放统一经过 `retireActiveGoal()`，先转换终态，再捕获发布失败，最后清 handle。
- **回归**：`test_goal_handle_retirement.py` 静态守住“唯一释放点且先终结”的不变量；提交
  `e41aac8`。单纯重复运行未复现不能证明修复。

### 6. 服务 Future 退休了，迟到 topic 仍能改写终态

- **现象**：归档 finalization 已超时并显示 FAILED，稍后却可能被迟到的 CANCELED/完成状态覆盖；
  在完整 `-j8` 第 8 轮复现。
- **隐蔽性**：Future 本身已有 generation 防护，看起来超时处理完整；同一操作还会经独立 DDS
  status topic 发布终态，后者不受 Future generation 约束。
- **误导线索**：提高反馈节流后，错误内部状态可能暂时不发布，使测试偶尔看不见污染。
- **根因与修复**：本地 finalization 超时后同时退休该 run 的 archive status；正常响应仍允许
  topic 与服务交叉到达，以保留权威最终计数。
- **回归**：`test_coverage_manager_archive.py` 确定性地在超时后释放服务并重发迟到状态；提交
  `1266f22`。
- **经验**：一次逻辑操作若有 Future、feedback 和 topic 多条回路，代次退休必须覆盖全部回路。

## 三、时间源、暂停与跨包监督

### 7. WSL 系统时间回跳同时冻结控制器和看门狗

- **现象**：约每 30 秒出现 1.2–1.7 秒命令空窗，机器人却继续执行最后速度；同一集成测试
  四次可失败两次，落在转向时则不明显。
- **隐蔽性**：WSL 对宿主时间校准会向后跳；ROS 节点在未启用仿真时间时默认使用可回拨的系统
  时钟，timer 要等时间重新追上才触发。看门狗也用同一时钟，因而同时失效。
- **修复**：仿真时间激活时跟随 ROS time，否则控制周期和安全期限统一使用 steady clock；只有
  消息 header 保留 ROS time。测试 plant 同时实现真实看门狗的命令过期归零。
- **回归**：`test_control_clock.cpp`、`test_coverage_executor.py`；提交 `6227a3c`。
- **经验**：安全计时器不能与它要防护的控制器共享一个会被冻结或回拨的时间源。

### 8. 暂停恢复不能只把原运动曲线的时钟向后平移

- **现象**：恢复瞬间命令达到 `0.350 m/s`，暂停前只有 `0.203 m/s`；转向子状态曾未被测试真正
  命中。
- **根因**：时间参数化曲线假定从静止起步。机器人停住后从曲线中段继续，调度器把零速读成
  落后并按追赶上限冲速；刹车期间曲线还继续前进，进一步制造虚假欠账。
- **修复**：期限只平移真正静止的时段；运动曲线从当前位姿重新规划，清空 lag、积分和 stretch。
  转向恢复重新进入制动/规划流程。
- **回归**：`test_coverage_executor_pause.py` 明确命中每个内部运动状态并比较恢复前后峰值；提交
  `55147ad`。
- **经验**：暂停后的 deadline、状态身份和运动 profile 是三种不同对象，不能用一个“续时钟”
  同时处理。

### 9. 参考心跳与采集门超时单边调参会造成“相机停、机器人仍走”

- **现象**：参考停顿后自动采集先关门，机器人仍沿 SCAN 行驶；恢复后的第一帧落在目标后方，
  被归档 25 mm 合同拒绝。
- **隐蔽性**：两个超时分属 control 与 inspection 包；各自在本包看都合理，组合顺序却反了。
- **根因与修复**：采集停止期限必须晚于 tracker 的两级 gate 监督，并额外留一个 5 Hz 心跳周期：
  `reference_timeout_s ≥ 2 × capture_gate_timeout_s + heartbeat_period`，当前为 `1.25 s`。
  gate 只由 reference 驱动，避免一次在飞曝光重新续命。
- **回归**：`src/climbot_gazebo/test/test_inspection_contract.py` 跨包读取安装配置并断言不变量；提交
  `4ddbcd8`、`d4decb5`。
- **经验**：跨包安全关系必须由跨包测试表达，不能只在两份 YAML 各写一句注释。

## 四、图像关联、归档与中间件

### 10. 无 request ID 的 receipt 会串到下一次曝光

- **现象**：某段服务成功后 receipt 投递超过等待期限，迟到 receipt 被下一段第一个 pending
  请求接收，最终因标签几何不匹配白丢一帧。
- **隐蔽性**：超时失败路径有 DRAINING，会丢迟到帧；真正可达的是“服务已成功、topic 仍迟到”
  这条较窄路径。扩大 QoS 队列反而让旧 receipt 更容易存活。
- **修复**：`CaptureOnce` 响应直接返回曝光 Header；Future 天生与请求关联，不再维护第二套
  topic 身份协议。归档的图像/metadata 配对和最终计数仍是交付权威。
- **回归**：automatic capture 的成功、零时间戳和迟到响应测试；提交 `87fa907`。

### 11. 耐久批处理优化扩大了完整性盲区，`syncfs` 计时又被采样周期错开

- **现象**：逐图 `fsync` 导致磁盘 I/O 尖峰；改为每 32 张提交后，写后回读校验被一并删掉，
  manifest 的 SHA-256 只证明内存字节。与此同时每 50 张 INFO 与每 32 张 commit 在 660 帧任务
  中从不重合，21–23 ms 的 commit 实际完全不可见。
- **根因与修复**：图像/标签保持原子写，逐图原位回读校验摘要；每 32 张 `syncfs` 推进耐久计数，
  并单独记录每次批大小、耗时和 run 最大值。注意 `syncfs` 刷整个文件系统，批大小只限制频率，
  不限制单次停顿。
- **回归**：archive core/recorder 的损坏、批次与 manifest 指标测试；提交 `85679bb`、`c396d0a`、
  `1eb3197`。
- **经验**：性能优化删除校验时，必须重新写出数据合同；周期采样不能替代事件本身的计时。

### 12. Fast DDS 共享内存段不足造成数秒级可靠重传等待

- **现象**：长采集中单帧偶发等待 `0.6–4.0 s`，不一定每次任务出现；一旦高频压测进入劣化，
  后续 237 次中 64 次停顿。机器人、渲染和小话题仍正常。
- **误判过程**：先后被归因为 Gazebo 渲染、GUI/D3D12、主机负载和共享 bridge 队头阻塞；这些
  相关现象都不能解释 trigger→Gazebo 正常而 gz→ROS 接近 3 秒。
- **根因**：一帧 RGB 图为 6,220,800 bytes，默认 SHM 段约 512 KiB。持续分片写入耗尽段缓冲、
  丢一个片段后，可靠读端要等默认 3 秒周期心跳发现缺口。
- **修复**：三个实际大图发布者使用 64 MiB `fastdds_inspection_image.xml`；不能只保护第一跳，
  mono8 适配器和 `capture_once_node` 仍是大图 writer。操作者自带 profile 时明确由其覆盖。
- **回归**：`test_large_image_transport.py` 静态检查所有 writer、profile 大小和 launch 注入；
  动态证据为 46,300 次曝光零慢捕获及完整纵向 `660/660`。提交 `9ff0e7a`、`8b8181f`。
- **经验**：新增超过 512 KiB 的可靠话题时，检查写端 SHM，不要只看 DDS reader 或平均 CPU。

### 13. `dxg` 总数和启动噪声不能证明 GPU 驱动故障

- **现象**：一次 GUI 长任务中 WSL 整体挂死并出现 87 条运行期 `dxg` 错误；健康 GUI 启动同样
  会出现约 122 条，headless 离屏渲染也会增加日志。
- **误导线索**：只比较总数会把每次健康启动都判成故障；把 headless 当成“完全不用 GPU”也错。
- **判据**：只有运动期成簇增量与所有传感器接收断档同时间发生，才支持 Windows/WSLg 驱动
  挂起；还要排除 WSL 重启和 OOM。该主机问题没有仓库内确定性回归。
- **处理**：D3D12 保持默认，软件渲染只做 A/B；运行判据记录在
  [LOCALIZATION_GAP_PLAN](LOCALIZATION_GAP_PLAN.md)。结论修订提交 `e8462ee`。

## 五、拼接几何、证据与光度

### 14. 图像彼此对齐，不代表绝对比例正确

- **现象**：optimized 局部对齐很好，绝对锚点却偏约 20.6 mm，整墙尺度 `+4566 ppm`；优化
  无法消除。
- **根因**：诊断贴图块外表面比轮子行驶的碰撞面高 1.25 mm，相机拍的是悬空薄膜，拼接却按
  碰撞面 `z=0` 投影。在 0.275 m 物距上形成各向同性比例错误。
- **修复**：贴图外表面、碰撞面和拼接投影面统一；只在加载贴图块时内缩基础 visual，避免把
  无贴图墙反向放到碰撞面后。
- **回归**：墙面 xacro/纹理几何测试和 P2-06 绝对真值；提交 `3f422f5`、`7f8b875`。
- **经验**：相对残差只能证明照片互相一致；绝对比例必须用独立墙面真值和真实被拍平面检查。

### 15. 验收可以在“什么都没量”时真空通过，手填 provenance 也可以自相矛盾

- **现象**：NaN、域外矩形或零 feature 区域都可能得到“域内零漏拍”；正式摘要还曾声明一个
  不可能产生其字段的旧提交且 `source_modified=false`。
- **根因**：验收域由命令行自由输入，空 mask 上 `uncovered=0`；整条跨提交链只有一个手填
  provenance 块，没有程序核对每阶段到底读了什么、写了什么。
- **修复**：沿 mosaic→processed→archive→冻结任务哈希链取得多边形巡检域；非有限、无交集、
  零 feature 都报错。每阶段在原子发布内写自己的代码状态、参数、输入/输出哈希，摘要固定校验
  五阶段和五条链接。
- **回归**：`test_diagnostic_inspection.py`、`test_stage_provenance.py`、
  `test_evidence_chain.py`、`test_evidence_summary.py`；提交 `7f8b875`、`d9e82e1`。
- **经验**：`all([])` 的 true 不是验收通过；正式证据不允许人工填写机器可以计算的溯源字段。

### 16. 平场流程正确，仍可能因标定靶材质不同而欠校正

- **现象**：旧平场后角/中心仍为 `0.907`，约 9% 碗形残差；重新拍同一灰板与旧标定逐点重合，
  证明不是漂移或样本失效。
- **误导线索**：月光、灰板尺寸、标定日期和标定算法都曾被怀疑；它们无法解释残差固定在相机
  画幅且相邻反向扫描线几乎相同。
- **根因**：灰板走 classic material，诊断墙走 PBR metal + albedo map；同一 LED 下两条
  着色路径的空间响应不同。
- **修复**：标定靶改用与被摄墙相同的 PBR 字段、表面平面和均匀中位 albedo；新标定使角/中心
  到 `1.011`，optimized 接缝 P95 `7→6 DN`，几何指标不退化。
- **回归**：`test_wall_world_matches_description.py`、平场单测与同输入 A/B；提交 `73e6e8f`、
  `630db54`。
- **经验**：平场标定匹配的是完整光度传递路径，不只是相机、曝光和一块“看起来均匀”的表面。

## 六、测试夹具的时钟与清理

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
  [S0 报告](../results/suction_s0_2026-10-17/REPORT.md)。
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
  追溯。新证据见[时戳专项](../results/cruise_timestamp_2026-10-17/REPORT.md)。

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

### 21. 时间控制将新计划与旧时间戳位姿比较，产生虚假追赶

- **现象**：保持时钟与运动积分正常，只在巡航中暂缓 6 次里程计发布，旧控制器即从约
  `0.205 m/s` 升到 `0.218992 m/s`，触发原 `≤0.205 m/s` 断言；新增回归在修复前也以
  `0.218799 m/s` 失败。
- **误导线索**：重跑通过、放宽速度门槛或直接限制所有追赶速度，都不能区分输入延迟和真实
  行驶落后，也不能解释同一输入时间戳下的虚假误差。
- **根因**：时间控制将当前计划距离与最近一次测量位置相减，遗漏位姿源时间与控制时刻的差。
  新鲜度检查仍允许的输入延迟因此被当作真实落后，触发追赶项。
- **修复**：提交 `c44b318`。反馈误差改用位姿源时间对应的计划距离；前馈仍用当前计划。
  旧新鲜度检查、`0.205 m/s` 测试门槛及真实落后的追赶能力均保留。
- **回归**：`test_delayed_odometry_does_not_create_false_cruise_catchup` 在原巡航用例中
  注入 6 次暂缓；正常及延迟输入修复后峰值均约 `0.200000 m/s`。既有
  `test_falls_behind_under_slip_and_then_catches_up` 同时通过，完整 -j8 连续两轮各为
  1555 tests、0 errors、0 failures、74 skipped。
- **证据边界**：[原始时序与红绿对照](../results/cruise_timestamp_2026-10-17/REPORT.md)
  确认这一缺陷机制；S0 最初 `0.210464 m/s` 的逐包时序未留存，不能倒推其唯一原因。
  本项为 ROS 控制测试，实际 Gazebo 运动及新吸附配置仍须 S5/S6 验收。

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
  两种负对照／覆盖正对照见 [S4 证据](../results/suction_s4_2026-10-18/domain_environment.json)。
  第一次失败未保留充分诊断，补强日志后完整复现相同初始化错误；不靠猜测消息队列归因。

## 新增案例的规则

只有根因已经被代码、受控 A/B 或不可歧义的运行证据确认，才把案例写成“已解决”。尚未定位的
现象留在对应专项计划，不在这里猜原因。新增条目必须包含：误导线索、修复提交和回归入口；若
客观上无法写确定性测试（例如宿主 GPU 驱动挂起），必须明确写出可操作的判据和证据边界。
