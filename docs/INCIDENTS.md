# 隐蔽故障与排查经验

更新：2026-10-19。本文记录开发中已经确认、但不容易稳定复现或很容易被误判的问题。它不是
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
| 密封积分 | 临界转角漏气低估，微小位姿舍入导致面积突变 | 积分是否明确保留两个端点，不能只比较粗细采样 |
| 资产缓存 | 改代码后 GUI 不出现，重试仍失败 | 缓存是否原子发布，冷构建是否超出启动等待 |
| 仿真性能 | RTF 低，看似没用 GPU | 主线程 perf：静态 model 数量和粗碰撞几何体数量 |

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

- **S5 同根因补查**：首次完整 -j8 的 pause 夹具首个 Start 拒绝，生产日志明确给出
  `Coverage executor Action server is unavailable.`，而不是业务状态拒绝。该夹具仍缺少管理器
  自身发现屏障。提交 `9f3da0f` 添加仅对这一派发前明确拒绝有界等待的
  `_start_after_executor_discovery`；其他拒绝立即失败，接受后不重发。
  `TestPauseExecutorDiscovery` 的三项确定性测试覆盖这三种边界，完整 1598 项 -j8 零失败。
  首次失败日志名及最终日志名见 [S5 regression](../results/suction_s5_2026-10-18/regression.json)。

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

### 23. GUI 观察器在正常收尾期间误报窗口退出

- **现象**：受控信号时序中，在 `spin_once` 返回期间收到已请求的 TERM，GUI 同时退出，
  观察器仍进入存活检查并报错。正式控制 16 组均未触发；这不是实际仿真崩溃证据。
- **误导线索**：将正常进程组收尾当作 GUI 故障，或完全移除存活检查以避免误报。
- **根因**：循环入口只检查一次 stop；阻塞调用期间信号改变 stop，剩余循环体仍执行。
- **修复**：本条所属提交在 spin 后立即检查 stop，再决定是否检查 GUI；意外退出仍失败。
- **回归**：`test_gui_observer_shutdown.py` 同时覆盖请求收尾和意外退出。旧版本为
  1 failed / 1 passed，修复后两项均通过；红绿日志与源码哈希见
  [S6 记录](../results/suction_s6_2026-10-18/observer_shutdown_regression.json)。
  另以三个纯时钟回归确保测量不受接收抖动影响，且不隐藏 reset／暂停。

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
  [红绿记录](../results/suction_s6_2026-10-18/state_fixture_regression/summary.json)。

### 25. 自发光标靶叠加反射，且亮度检查遗漏既有栅格边缘 allowance

- **现象**：S6 实际 GUI 中，新旧模型的完整可见彩色标记均被 G1 误报为遮挡；两者未知
  像素约 6.3 万，RGB 标记次通道约 73/255，超过原颜色分类的 0.25 比例。
- **误导线索**：据此判定新底盘挡住镜头，或提高颜色／亮度容差并删除失败记录。
- **根因与第一步修复**：专用 emissive 参考同时带 ambient/diffuse 反射。`d88fb30` 只在
  候选生成的可选 G1 标靶移除反射，保留 emission／几何／灯光／相机；次通道降至 39，
  未知像素为 0。正常墙面不含该标靶，冻结旧世界源不改写。
- **进一步发现**：蓝／黄边界的一个真实双线性混色像素为 `[147,147,141]`；颜色检查按既有
  一像素 allowance 接受，raw 最大通道却低于 150。两套边缘处理不一致导致剩余误报。
  `eaa71b7` 将同一个固定 3×3 allowance 用于亮度，保留 raw 最小值，并另记处理后最小值。
  150、零未知像素、ROI、允许边宽不变；这是判据实现修正，raw=147 不追认为 raw≥150。
- **回归**：实际帧重放及显式合成黑／低亮黄色遮挡；后两项仍失败。单元红绿为
  1 failed / 4 passed → 5 passed；实际 GUI G1 全部检查通过，完整 -j8 为
  1645 tests、0 errors、0 failures、79 skipped。全部失败与边界说明见
  [S6 标靶证据](../results/suction_s6_2026-10-18/g1_fixture_regression/green_live.json)。

### 26. 大纹理冷启动中服务先于时钟／定位就绪，任务提前接受

- **现象**：S6 首次实际 GUI 长采集已接受任务，约 39 s 场景加载后立即被原位置越界保护
  停止，0 张图像；早期窗口截图仍为空白。原始越界位姿没有保存，不追认唯一具体数值。
- **误导线索**：把服务 discovery 当作仿真／定位就绪，以固定睡眠猜加载完成，或据短窗口
  RTF 0.87757 判断稳态性能；增大墙面边界或控制器超时也不能修复这一前置条件。
- **修复**：`3ac39f2` 在 G2 工具中增加只读屏障：非零推进时钟、新鲜传感流、TF，以及
  truth／filtered 均在原规划安全域内，连续满足 2 个仿真秒后才启动观察器和任务。
  原控制保护、模型、旧恒力、20 墙钟秒预热和拍照门槛不变。
- **回归与边界**：8 项纯就绪测试及完整 -j8 的 1656 项零失败；独立 r2 四组实际 GUI
  横纵任务全部采集通过，2680 张图像、无残留清理。新 trace 明确记录 filtered 初始在原点，
  随后绝对位置更新再满足屏障；它支持前置条件缺陷，不补造首次失败的具体位姿。
  见 [S6 首轮报告](../results/suction_s6_2026-10-18/CAPTURE_R2_REPORT.md)。新模型拍照
  实时率仍较低，单列为未通过项，不因启动修复而判完整 S6 通过。

### 27. 密封膨胀的预测上端点因乘除舍入被误判为无括区

- **现象**：S6 求解器离线对照的固定输入11704中，全停、无漏口、体积膨胀且仍为正表压，
  新旧初稿均抛出 `implicit positive-pressure bracket failed`，但恒质量解析压力合法。
- **误导线索**：把整个随机输入组都当作已比较，或因两版都抛错就把有效输入归为非法。
- **根因**：`p_old*V_old/V_new` 的乘除舍入使上端点质量残差略负；原检查要求上端点非负，
  因而拒绝了真实位于该边界的零流量解。
- **修复**：本条所属提交将完整上端点向外移动一个浮点ULP，同时为性能加入局部符号括区、
  有界扩大和原完整区间回退。残差方程、1e-7Pa终止精度、压力子步和物理参数不变。
- **回归**：`ImplicitSealedExpansionAcceptsRoundedUpperEndpoint`稳定复现，20项中1失败
  →20项全通过；解析密封质量和大漏口／压缩排气亦覆盖。11999个新旧可比输入最大差
  9.48e-8Pa，另1个修复输入单独核对解析解，不隐藏异常不匹配。见
  [源码、红绿及数值／性能证据](../results/suction_s6_2026-10-18/SOLVER_OPTIMIZATION.md)。
  离线提速不代表完整S6或实际GUI实时率已经通过，后者仍须独立重验。

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
  [红绿记录](../results/suction_s6_2026-10-18/appearance_checkpoint/sensor_endpoint_regression.json)
  及[完整检查](../results/suction_s6_2026-10-18/appearance_checkpoint/commit_checks.json)。

### 29. 吸附模型异常后真值停报，外部只剩健康旧快照

- **现象**：实际双 GUI 三风机全停，机器人下滑到地面；裙边压缩超限后日志报错，
  `/suction/ground_truth/state` 停止更新，最后一条仍为 `runtime_failed=0`。
- **误导线索**：用最后一条健康真值证明全程无错误，或删除几何限值／恢复恒力使跌落看似通过。
- **根因**：异常分支只更新 ECM 的 failure 字段，没有向 IO 发布；其后 PreUpdate 直接返回。
  ECM 观察器能看到失效，传输话题订阅者看不到，两条评价路径不一致。
- **修复**：本条所属提交在锁存故障后持续发布时间和故障，保留最后有效求解时间；命令与
  一阶转速继续更新，所有吸附／裙边／支撑／滚阻施力为零，停止无效腔压的传感器采样。
  完整步骤校验在施力前完成，reset 清除故障；没有扩展翻转／撞地的吸附模型有效域。
- **回归**：`test_all_fans_off_failure_remains_observable_and_reset_recovers`，真实全停下滑
  后显式注入远离墙面的不支持方向，两个 reset 代次验证实际传输及原生状态。相同当前
  IO／核心／物理配置，仅切换 VacuumSystem：旧实现因状态时间冻结失败，新实现通过，
  新实现的实际话题收到 260 条故障快照。源、时序与限制见
  [红绿记录](../results/suction_s6_2026-10-18/failure_observability/REPORT.md)。

### 30. 首帧前 D3D12 上下文创建失败不能当成控制或实时率通过

- **现象**：新质量双 GUI 控制的第五次启动，GUI 先报 `D3D12: Removing Device.`，
  再报 `Failed to create OpenGL context` 并退出 250。server 继续推进，原生 RTF 接近 1，
  但没有 GUI／RViz 截图，控制 evaluator 也没有取得执行服务；本组明确失败。
- **误导线索**：只读取 RTF 数值、将 progress 中存在条目等同于全部 checks 为真，
  或直接重跑并丢弃失败。GUI 启动失败不代表运动控制已通过。
- **已确认的恢复缺口**：已有 server→GUI→RViz 串行启动，仍没有处理 GUI 首帧前的
  特定上下文创建失败。宿主驱动为什么移除设备尚未定位，不能宣称已修复 GPU 根因。
- **有界恢复**：本条所属提交只对 D3D12 GUI、两条精确错误均出现、子进程非零退出、
  单次启动不超过 30 s 且尚未观察到 `/gui/camera/pose` 的情况，最多重试两次。
  每次明确输出原因，使用同一后端、同一环境；server 不重启，不回退软件渲染。
  首帧后崩溃、其他错误、关闭信号、超出预算均不恢复，正式任务仍须完整通过双 GUI 检查。
- **回归与边界**：`test_gui_recovery_is_bounded_and_only_before_first_frame` 使用真实失败
  子进程验证一次恢复、三次上限、首帧后及无关错误拒绝；另测迟到失败和关闭。
  相同受控子进程旧实现退出 250／一次，新实现退出 0／两次。该 fixture 验证恢复合同，
  不注入真实 GPU 故障；实际九组双 GUI 回归须在新源码冻结后重验，不能追认原失败。
  见[失败和恢复证据](../results/suction_s6_2026-10-18/gui_startup_recovery/scope.json)。

### 31. fork 后立即读取进程组，偶发漏记尚未完成 setsid 的子进程

- **现象**：源 `76b0cbd` 的旧结构横向双 GUI 回归，控制与全部运行检查通过，收尾却报告
  `pid ... did not get a session of its own; not recorded`，随后分区兜底扫描清理整个仿真组。
  本组按原清理门槛判失败，没有把“最终没有残留”替代“正常清理通过”。
- **误导线索**：直接归因于 Gazebo supervisor 派生孤儿，或者凭控制结果正常忽略 teardown。
- **根因**：`lane_remember` 在父进程恢复执行后只读取一次 PGID；子进程可能仍未调用
  `setsid`。同组判据本来用于防止误杀整条回归，却把合法的短暂过渡误认作调用错误。
- **修复**：本条所属提交有界轮询真实 PGID（50 次、100 ms 间隔），直到子进程离开当前组；
  自己的 PID、读取失败及永远不换组仍拒绝。分区隔离、TERM/KILL 与兜底告警不变。
- **回归**：`tools/test_lane_processes.sh` 的新 fixture 先返回真实旧 PGID，再放行子进程
  执行 `setsid`，稳定重现旧实现漏记；修复后记录实际新组且不告警。测试自身也用唯一分区
  前缀，避免并行 fixture 彼此清理；正式红绿按串行执行，初次共用分区的并行尝试不作证据。
  见[实际失败记录](../results/suction_s6_2026-10-18/lane_setsid_regression/scope.json)。

### 32. TERM 取消 ROS launch，双 GUI 收尾留下仍存活的 RViz

- **现象**：`f9089a3` 的新结构横向控制、TF、压力和双界面检查通过，收尾两个组被
  KILL；不能把无残留的最终状态追认为正常收尾成功。
- **误导线索**：因为 `kill -0` 对僵尸也返回成功，先假定只是僵尸并忽略告警。
  实际退出采样发现 RViz 是 `Sl` 活进程，26 s 后仍存活。
- **根因**：清理首先向组发 TERM。已安装 ROS launch 的 `_on_sigterm` 取消任务并明确
  警告可能留下孤儿；`_on_sigint` 才进入有序 shutdown。新 GUI lane 更容易暴露这一差异。
- **修复**：本条所属提交在控制与 G2 清理中先向已登记组发 INT，最多等待 8 s，再按
  原 TERM／KILL 路径兜底。Gazebo supervisor 把自身 INT 转为对子 GZ 的 TERM，仍避免
  原生 GZ 被直接中断；强杀、兜底扫描与残留判据不放宽。
- **回归与边界**：`tools/test_lane_processes.sh` 新增真实子进程仅 INT 正常退出的 fixture，
  旧 helper 稳定失败两项，新 helper 全通过。相同私有 Mesa、相同新结构和真实 GUI／RViz
  启动命令的退出对照中，TERM 在 26 s 仍留 RViz，INT 的已知组约 0.318 s 全部退出；
  该诊断不作为轨迹或 RTF 验收。见[实际对照](../results/suction_s6_2026-10-18/exit_group_investigation/actual_gui_comparison.json)。

### 33. SHM 发现端口检查持锁，单设 UDP 环境变量仍保留 SHM

- **现象**：约 340 ms 的局部定位／控制断档，平均 RTF 仍接近 1；重启后仍可出现。
- **误导线索**：把 WSL uptime、历史 I/O 或 Mesa 泄漏直接套用；认为
  `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` 已证明实际停用 SHM。最小 publisher 实验未复现，
  早期 env-only 协议对照无效，均不作根因确认依据。
- **捕获机制**：333 ms nanosleep 条件断点记录 SHM find_port／发现匹配调用栈；
  同时 SHM 接收线程等待 PDP liveliness 锁。RMW 8.4.4 LOCALHOST 分支会追加 SHM。
  原 bigH 没有栈；本条确认的是实际捕获的机制，不能追认原失败为唯一同因。
- **修复**：本条所属提交对子节点配置显式回环 UDP、SYSTEM_DEFAULT、关闭 builtins，
  仅处理显式本机 Fast DDS；操作员覆盖和网络发现范围保留，大图保持原 SHM profile。
- **回归**：`climbot_common/test/test_dds.py`；
  `climbot_gazebo/test/test_small_message_transport.py` 实际 participant 资源检查，
  不允许环境变量正确但仍映射 SHM／绑定非回环。双 GUI 10 个晚加入默认订阅者诊断
  最大控制／odom 间隔 24.449／23.916 ms；正式 S6 单独验收。
  证据和边界见[本次调查](../results/suction_s6_2026-10-18/localization_gap_investigation/HISTORY_REVIEW.md)。

### 34. 组级 INT 与 launch 转发向子进程重复发送退出信号

- **现象**：R5 新结构横向控制和双 GUI RTF 通过，收尾压力桥未结束并被 KILL；
  相机适配器异常退出，不能把运行期通过当作整组通过。
- **误导线索**：把它当作僵尸或忽略 teardown；短 parents→group→parents 三次根进程均退出且无残留，
  但 Gazebo server 仍有原生卸载期段错误，不能写成所有内部进程正常，
  也不能凭一次短对照认定原生挂住已稳定复现。
- **确认缺陷**：组级 INT 先通知孩子，launch 随后再转发；原日志有两次信号，真实
  子进程 fixture 的旧 helper 得到 2 次 INT／退出 4，新 helper 为 1 次／退出 0。
- **修复**：本条所属提交共用根进程 INT、整组 TERM/KILL；原等待预算、强杀失败与
  分区兜底不变。该修复消除重复信号，但不冒称唯一重建原生桥退出内部死锁。
- **回归**：`tools/test_lane_processes.sh` 的 launch-style forwarding fixture；
  [红绿及实际对照](../results/suction_s6_2026-10-18/shutdown_signal_investigation/REPORT.md)。

### 35. Bash 后台继承 SIG_IGN，使 launch 根进程收不到有序退出 INT

- **现象**：改成 root INT 后仍有 planner/RViz 强杀；直接 Python Popen 的短诊断根进程能退出且无残留。
- **误导线索**：只检查发送了 kill，或用显式安装 signal handler 的假 launch 证明真实
  CLI 必定收到 INT；两者都漏掉了 Bash 后台继承的 SIG_IGN。
- **根因**：实际 ROS launch 最小场景中，Bash 后台 root 的 `/proc/status` SigIgn
  包含 INT，root-only INT 无效且不转发；重置后同一个场景约 32 ms 有序退出。
- **修复**：本条所属提交的 `tools/start_session.sh` 在 exec 前重置 INT，保持原会话
  PID；控制和 G2 统一使用，root INT／整组 TERM/KILL 预算及强杀失败判据不变。
- **回归**：`tools/test_lane_processes.sh` 新增实际后台根默认 handler fixture；
  红测 1 项失败，绿测全部通过。实际 ROS launch 三种启动方式对照见
  [信号继承证据](../results/suction_s6_2026-10-18/signal_inheritance_regression/REPORT.md)。

## 36. world reset 后 RViz 墙面、网格和路径消失

- **现象**：S7 首轮 10 张采集数值通过，实际 RViz 在 reset 后只剩机器人；用户和真实截图确认。
- **误导线索**：transient-local topic 仍有 publisher、任务仍执行、照片仍保存，不代表 RViz 缓存恢复。
- **根因**：时钟回退使 RViz 清空 Marker/Path 显示；现有订阅不断开，规划器的一次性历史发布不会自动重送。
- **修复**：`1846f6a` 对缓存的网格、区域和 Path 做 1 Hz 显示刷新，只更新可视化时间戳，不重规划、
  不修改或重发执行任务；同样覆盖 RViz 手动 Reset。
- **回归**：`test_coverage_visual_refresh.py` 在 `/clock` 从 100 s 回退至 1 s 后验证三类显示恢复、
  几何不变且零新任务，先红后绿；随后 70 张双 GUI 任务真实窗口复核通过。
  [正式证据](../results/suction_s7_2026-10-18/REPORT.md)。

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
  [墙面专项报告](../results/wall_texture_randomization_2026-10-18/REPORT.md)。

## 38. 稀疏覆盖物的重压缩及局部多边形裁剪造成 gutter 差异

- **现象**：新整墙纯混凝土 DDS 的 3624 次 mip 0～5 共区比较全部一致；增加裂缝、
  板缝和涂鸦后，932 次不一致。退出正常或 PNG 看着连续不能替代压缩后的共区检查。
- **误导线索**：覆盖物按全局坐标绘制且局部两块测试一致，就认为整墙 gutter 已通过；
  原检查没有覆盖“修改块旁边是未修改硬链接块”的边界。
- **根因**：修改块从 BC1 解码后整块重编码，即使没涂画的部分也发生有损变化；
  相邻未修改块继续链接原 BC1，两侧 mip 的相同物理位置因此不一致。
- **修复**：本条所属提交逐级传播实际涂画像素的影响范围，未受影响的 BC1 block
  保留原文件对应字节；覆盖物之外不增加第二次压缩。此修复后仅余 2 次 mip 共区差异，
  来自修补多边形在不同局部画布边缘的 PIL 斜线裁剪，实际相差一个像素；修补轮廓改为
  在完整全局像素包围框内绘制后合成裁入各块。新资产独立生成，旧资产不改写。
- **回归**：`test_diagnostic_bc1_preserves_shared_gutters_beside_unpainted_tiles` 覆盖
  不跨 gutter 和跨 gutter 两种涂画；另有
  `test_repair_polygon_clipping_preserves_the_first_pixel_of_shared_gutter` 复现真实修补块边缘。
  两个受控旧行为对照都重现失败，修复后与相关测试共 52 项通过；完整八并发 1779 项零错误／失败。
  最终整墙 3624 次 mip 共区比较零差异；双 GUI 代表采集及 CUDA 拼接尚未启动，
  已按用户要求留到下一次。证据见[当前墙面报告](../results/wall_texture_local_2026-10-18/REPORT.md)。

## 39. ODE 哈希空间跨零回绕导致碰撞候选漏检

- **现象**：M2 水平起伏探针出现约 0.740 mm 穿透和间歇断触，同源垂直样本通过。
- **误导线索**：猜测多体竞争、探针尺寸、轮胎刚度或物理步长。单探针、自由探针及半步长
  仍失败；将同一数据仅换为完整配对后恢复正常，但不能把全配对当作低成本运行方案。
- **根因**：系统 ODE 0.16.2 的 `dxHashSpace::collide` 在插入／查询单元时先取模再递增桶号，
  丢掉完整地址的无符号回绕。64 位表长 31 的跨零示例中，同一 `z=0` 单元分别落到桶 16／0。
  两个确实相交的基本几何体即能复现，候选数 0，窄相根本没有被调用。
- **修复**：当前 `feature/m2-wall-compliance` 工作树（尚未提交；提交后以本条所属提交为准）
  的局部 ODE 补丁在插入和查询两处都从完整地址计算桶号，保持哈希筛选；构建配方固定源包与
  SHA256，校验库及原生回归，观察器核对实际加载路径。不替换系统库或修改其他项目，
  当前仅由 M2 探针显式选择，默认应用配置未切换。
- **回归**：`tools/ode_hash_probe.cpp` 的五种平移 × 哈希／完整配对；同编译参数的上游／修复库
  受控 A/B；修复库下平面、水平／垂直起伏、半步长和首条真实板缝五组原生检查通过。
  源包、补丁、库与试片指标见[专项证据](../results/m2_t0_hashspace_2026-10-19/REPORT.md)，
  复现命令见 [M2 操作入口](OPERATION.md#m2-原生接触与局部-ode-修复)。不等同于 T0/M2 整体验收。

## 40. 原生高度场碰撞范围比声明尺寸小

- **现象**：内部高度／法向探针正常、远处边界外探针正常，但预期边界内 0.15 mm 的量规
  在高度场上失去支撑；同一位置的有限平面盒控制正常。
- **根因**：gz-physics/DART 按 SDF 长宽除以 N 确定点间距，N 点只有 N−1 个间隔，
  不是 ODE 哈希补丁失效。0.8 m、257 点试片每边缩进约 1.56 mm，129 点约 3.10 mm。
  本机私有参考 Subway 的 `docs/design/MECHANICS.md` 已记录相同采样约定。
- **修复**：当前 M2 工作树的 `tools/probe_m2_wall_surface.py` 将 SDF 长宽乘 N/(N−1)，
  源图、解析坐标、预期边界及误差门槛保持不变；默认整车场景未切换。
- **回归**：`--boundary-only` 的平面盒控制、129／257 点垂直高度场和 257 点水平高度场，
  每组四边共 24 个独立内外探针。未补偿时三组高度场均失败，补偿后四组均通过；
  单元回归检查不同密度、四边布点和非法采样拒绝。见[边界报告](../results/m2_t0_extent_2026-10-19/REPORT.md)。

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
  证据见[裂缝定位报告](../results/m2_t0_crack_bridge_2026-10-19/REPORT.md)。

## 新增案例的规则

只有根因已经被代码、受控 A/B 或不可歧义的运行证据确认，才把案例写成“已解决”。尚未定位的
现象留在对应专项计划，不在这里猜原因。新增条目必须包含：误导线索、修复提交和回归入口；若
客观上无法写确定性测试（例如宿主 GPU 驱动挂起），必须明确写出可操作的判据和证据边界。

### 42. 私有 ODE 链接 libccd 仍不等于发行版配置

- **现象**：打开系统 libccd 后，平墙直行出现驱动轮短暂失载；no-libccd 的补丁 A/B 不能证明发行版等价。
- **隐蔽性**：ODE 的 CMake 与 Autoconf 对“启用全部 libccd 碰撞器”的默认含义不同。
- **误导线索**：仅检查 `ldd` 中存在 libccd 或 `dGetConfiguration` 一致，会漏掉具体碰撞器分派。
- **根因与修复**：Ubuntu 的 `--enable-libccd` 排除 box-cylinder，而 CMake 默认包含。
  显式 `ODE_WITH_LIBCCD_BOX_CYL=OFF`，其余 CCD 分派开启，并保留发行版内部断言。
  另外修正上游 CMake 的目标大小写、custom 头路径和 Ubuntu 导出目标名，不修改碰撞算法。
- **回归**：`tools/validate_m2_review.py` 在真实系统／私有加载下对照三组 M1 工况，检查 libccd 映射、
  逐轮判据及数值差异；加载守卫拒绝旧配置。原 no-libccd 报告只保留补丁因果证据。

### 43. 无响应 hold 的测试没有先建立故障条件

- **现象**：完整 `-j8` 中 `test_coverage_manager_hold_release` 偶发直接进入执行，随后等待 STARTING 超时。
- **隐蔽性**：测试先发布 transient-local hold 状态就调用 Start，DDS 接收顺序不能证明管理器已消费状态。
- **误导线索**：表象像取消或 deadline 失效，实际故障注入尚未建立，管理器没有进入待释放状态。
- **根因与修复**：用明确已存在但不响应的 `/control/hold` 服务构造未完成 release，单独 callback group
  隔离阻塞，teardown 主动解除服务等待。不修改生产控制器或延长验收门槛。
- **回归**：原用例继续验证 STARTING、取消／deadline 后恢复锁定、rearm，以及从未发送运动 Goal。


### 44. 起伏三角网下轮速正常、车体却明显降速

- **现象／误导线索**：板缝整车试验，柔性轮行程比刚性轮更短，初看像弹簧或裙边拖曳异常。
- **确认范围**：去掉板缝仍降速；同源起伏、软轮、裙边、压差和指令保持，仅切换驱动轮接触
  为原生高度场，行程 0.120→0.609 m，实测轮速均 4.210526 rad/s。
  三角网左／右平均约 10 个接触点，高度场约 1～1.4 个；裙边法向反力约 145 N，基本不变。
  接触表示的数值性滑移是本次主因；未定位到 ODE 的具体源码行，不声称仅接触数量决定结果。
- **处理状态**：`1079039` 后的 M2 接触修正随本节证据一并提交，未切默认。
  无板缝控制恢复运动后，采用仅起伏高度场承载驱动轮，球轮代理、视觉及裙边保留板缝；
  一次带缝源整车短试验通过（0.613485 m），不以加大风机或放宽运动门槛掩盖。
  驱动轮省略5 mm缝的几何下沉有明确近似边界；整墙性能及T2载荷／漏气仍待验收。
- **回归入口**：`tools/probe_m2_robot_patch.py --soft --smooth-control` 与追加
  `--native-heightmap` 的配对；前者应失败、后者应满足同一运动门槛。
  正向回归为 `--soft --native-heightmap`（不带 `--smooth-control`），保留真实缝源。
  见[证据及限制](../results/m2_robot_patch_2026-10-19/REPORT.md)。

### 45. 密封边积分终点因舍入被丢弃，造成局部漏气低估

- **现象**：T2 保存姿态重放时，极少数转角的漏气面积对浮点舍入敏感，粗细积分也不一致。
- **误导线索**：看似四元数变换／时间对齐或板缝分辨率不够；仅加密有时也未恢复完整面积。
- **根因**：`length*base/base` 偶尔比 `length` 大一舍入单位，`add()` 的范围检查丢弃终点；
  后面一小段未被积分，若通道恰在段尾，就显著低估。
- **修复**：明确加入 `0` 和 `length`，只在内部节点计算乘除；不增加实时采样数、不改风机。
- **回归**：`SealChannels.FinalEndpointCannotRoundOutsideAndOmitAnEdgeTail` 在无槽平面上
  对照完整周长常量开口积分；旧源码确定性失败，新源码 1／2／4／8 倍全部通过。
  临界有槽姿态另与每边 80,000 点中点积分比较，默认面积误差约 0.003%。
- **证据**：[T2 收口报告](../results/m2_t2_closure_2026-10-19/REPORT.md)。

### 46. 验收轮载记录器在 GUI reset 时被重建

- **现象**：T3 双向转向检查完成后，reset 使服务器报 `load trace already exists` 并退出；尚未派发采集。
- **根因**：观察插件未实现 `ISystemReset`，Gazebo 重建插件并再次执行拒绝覆盖文件的 Configure。
- **修复**：实现 reset 接口，刷新并保留同一记录流；时钟回退与 generation 均保留，不覆盖既有记录。
- **验证**：同一整墙模型、双 GUI 下真实 reset，服务接受、generation 递增、临时支撑重施后撤除、
  风机重新升速、负压恢复及无物理失败九项检查全部通过；新一代记录继续写入。

### 47. 起伏墙 OBJ 的贴图纵坐标重复翻转，暴露纹理块边界

- **现象**：整墙 GUI 出现规则小块明暗／纹理分界，除真实板缝外还有大量矩形接缝；
  同一套 v6 DDS 在旧平墙无此问题。用户目视发现，短程照片保留用于复查。
- **根因**：新网格写入 top-origin V，但 gz-common5 `graphics/src/OBJLoader.cc`
  以 `AddTexCoord(uv.X(), 1.0-uv.Y())` 加载 OBJ，再翻一次，导致每块单独上下倒置，无法拼回全局图像。
- **修复**：写标准 bottom-origin OBJ V，由加载器统一转到图片坐标；同步明确混凝土材质
  `metalness=0`，与旧平墙一致。原始 DDS、gutter、像素分辨率及物理几何均不变。
- **回归**：裁块测试按加载器翻转后的全局图片行坐标检查，三个整墙资产测试通过；
  整墙缓存按源码哈希独立重建，GUI 前后截图另存数据根。大面积采集仍暂停。

### 48. 整墙冷缓存超出启动等待，留下半成品目录阻塞以后所有启动

- **现象**：改动整墙资产代码后首次双 GUI 启动，采集脚本报 simulator did not become ready；
  再次启动则在生成资产时报目录已存在，GUI 始终不出现。
- **误导线索**：像 GUI／GPU 或孤儿 Gazebo 服务端问题；同时确有一个无 GUI 的遗留服务端在跑，
  停掉后仍不能启动。
- **根因**：显示缓存键包含实现源码哈希，代码一改就要重建约 130 s，超过启动等待时限；
  中断后缓存目录只写了一半、没有 manifest，而 `prepare()` 用 `mkdir(exist_ok=False)` 拒绝复用。
- **修复**：先写入同级临时目录，完成并写 manifest 后一次改名发布；中断只留下可忽略的
  `.partial-*` 目录。代码改动后先离线预建缓存，再启动 GUI。
- **回归**：`test_interrupted_build_does_not_block_the_next_launch`，旧代码确定性
  FileExistsError，新代码通过。

### 49. 整墙双 GUI 下 RTF 约 0.6，看似 GPU 未启用

- **现象**：起伏整墙接入后 GUI 显示 RTF 0.55～0.64，用户怀疑没有用 GPU。
- **误导线索**：GUI 进程 CPU 偏高、视觉网格约 547 万三角形；但服务端、GUI、RViz 均已加载
  D3D12（NVIDIA），渲染线程只占约 10%。
- **根因**：服务端主线程占满一核，约八成时间在 DART／ODE 的逐物体簿记。每个贴图块、每个球轮
  碰撞块各是一个静态 model，共 400 多个 DART 骨架，每步刷新位姿；80 个碰撞几何体又让
  ODE 哈希空间粗碰撞单项占约 11%。
- **修复**：视觉块合为一个 link 的 320 个 visual（同 M1 平墙），并关闭墙面投影；80 个碰撞块
  离线合并为一个网格，三角形不变。双 GUI 竖向短任务整任务 RTF 0.968。
- **回归**：`test_wall_tiles_share_one_static_link_per_role` 检查只有 3 个墙面 model、
  合并网格逐顶点等于原块加偏移。排查方法：`perf record -t <服务端主线程>`，按库统计。
