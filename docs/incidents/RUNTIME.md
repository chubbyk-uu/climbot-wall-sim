# 故障案例：RUNTIME

入口与新增规则见[故障索引](../INCIDENTS.md)。

### 13. `dxg` 总数和启动噪声不能证明 GPU 驱动故障

- **现象**：一次 GUI 长任务中 WSL 整体挂死并出现 87 条运行期 `dxg` 错误；健康 GUI 启动同样
  会出现约 122 条，headless 离屏渲染也会增加日志。
- **误导线索**：只比较总数会把每次健康启动都判成故障；把 headless 当成“完全不用 GPU”也错。
- **判据**：只有运动期成簇增量与所有传感器接收断档同时间发生，才支持 Windows/WSLg 驱动
  挂起；还要排除 WSL 重启和 OOM。该主机问题没有仓库内确定性回归。
- **处理**：D3D12 保持默认，软件渲染只做 A/B；运行判据记录在
  [LOCALIZATION_GAP_PLAN](../LOCALIZATION_GAP_PLAN.md)。结论修订提交 `e8462ee`。

## 五、拼接几何、证据与光度

### 23. GUI 观察器在正常收尾期间误报窗口退出

- **现象**：受控信号时序中，在 `spin_once` 返回期间收到已请求的 TERM，GUI 同时退出，
  观察器仍进入存活检查并报错。正式控制 16 组均未触发；这不是实际仿真崩溃证据。
- **误导线索**：将正常进程组收尾当作 GUI 故障，或完全移除存活检查以避免误报。
- **根因**：循环入口只检查一次 stop；阻塞调用期间信号改变 stop，剩余循环体仍执行。
- **修复**：本条所属提交在 spin 后立即检查 stop，再决定是否检查 GUI；意外退出仍失败。
- **回归**：`test_gui_observer_shutdown.py` 同时覆盖请求收尾和意外退出。旧版本为
  1 failed / 1 passed，修复后两项均通过；红绿日志与源码哈希见
  [S6 记录](../../results/suction_s6_2026-10-18/observer_shutdown_regression.json)。
  另以三个纯时钟回归确保测量不受接收抖动影响，且不隐藏 reset／暂停。

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
  见 [S6 首轮报告](../../results/suction_s6_2026-10-18/CAPTURE_R2_REPORT.md)。新模型拍照
  实时率仍较低，单列为未通过项，不因启动修复而判完整 S6 通过。

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
  见[失败和恢复证据](../../results/suction_s6_2026-10-18/gui_startup_recovery/scope.json)。

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
  见[实际失败记录](../../results/suction_s6_2026-10-18/lane_setsid_regression/scope.json)。

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
  该诊断不作为轨迹或 RTF 验收。见[实际对照](../../results/suction_s6_2026-10-18/exit_group_investigation/actual_gui_comparison.json)。

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
  证据和边界见[本次调查](../../results/suction_s6_2026-10-18/localization_gap_investigation/HISTORY_REVIEW.md)。

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
  [红绿及实际对照](../../results/suction_s6_2026-10-18/shutdown_signal_investigation/REPORT.md)。

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
  [信号继承证据](../../results/suction_s6_2026-10-18/signal_inheritance_regression/REPORT.md)。

## 36. world reset 后 RViz 墙面、网格和路径消失

- **现象**：S7 首轮 10 张采集数值通过，实际 RViz 在 reset 后只剩机器人；用户和真实截图确认。
- **误导线索**：transient-local topic 仍有 publisher、任务仍执行、照片仍保存，不代表 RViz 缓存恢复。
- **根因**：时钟回退使 RViz 清空 Marker/Path 显示；现有订阅不断开，规划器的一次性历史发布不会自动重送。
- **修复**：`1846f6a` 对缓存的网格、区域和 Path 做 1 Hz 显示刷新，只更新可视化时间戳，不重规划、
  不修改或重发执行任务；同样覆盖 RViz 手动 Reset。
- **回归**：`test_coverage_visual_refresh.py` 在 `/clock` 从 100 s 回退至 1 s 后验证三类显示恢复、
  几何不变且零新任务，先红后绿；随后 70 张双 GUI 任务真实窗口复核通过。
  [正式证据](../../results/suction_s7_2026-10-18/REPORT.md)。

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

### 50. 采集脚本给 ros2 launch 传空参数，默认 G2 验收无法启动

- **现象**：未设置 `G2_RESOLVED_ASSETS` 时报 simulator did not become ready。
- **根因**：脚本无条件传 `resolved_assets_directory:="$G2_RESOLVED_ASSETS"`，空值成为
  `resolved_assets_directory:=`，ros2 launch 拒绝；`m2_load_trace` 同理。仅在 M2 运行时总被设置，未暴露。
- **修复**：改为 `${VAR:+name:="$VAR"}`，与 `clock_publish_hz` 一致；平墙刚性轮对照已按默认参数通过。

### 54. 每个物理步发布接触与关节状态，M2 实时率被同步开销吃掉

- **现象**：M2（起伏墙＋柔顺轮）双 GUI 实时率约 0.94～0.955，贴着 0.95；GUI 显示的瞬时
  实时率波动几十个百分点。无 GUI、无渲染的纯物理基准却有 1.7～2.0 倍实时余量。
- **隐蔽性**：全机只用约 5.6 个核，不像算力不足；服务端主线程只忙约 74%，每秒约 1900 次
  主动让出，即每步阻塞两次。gz-sim8 每个带 PostUpdate 的系统各占一个工作线程并每步汇合。
- **误导线索**：以为 WSL 运行久了泄漏（重启前后纯物理基准相同、重启后 GUI 实时率不变）；
  以为是提速（0.20／0.30 交替对照差别在噪声内）；以为接触传感器 `update_rate` 能降频
  （gz-sim8 的 Contact 系统无视该值，实测仍约 900 Hz）。
- **根因**：三个接触传感器和关节状态每步序列化并经 bridge 转换，只有诊断工具订阅；
  拍照时整墙渲染另造成短时下跌（曝光附近 0.1 s 窗口平均约 0.90）。
- **修复**：关节状态 100 Hz（1814d8c）；默认不生成接触传感器与 Contact 系统（0b2128d），
  吸附系统与轮载记录器自建并读取同一份逐步接触数据，`contact_topics:=true` 恢复诊断话题。
  带轮载记录的同任务实时率 0.946→0.978，v2 仍通过。
- **回归**：`test_contact_observers_can_be_restored_for_the_load_baseline`、
  `test_joint_states_are_published_below_the_physics_step`；6000 步无 GUI A/B 轮载逐位一致，
  原生台架在无传感器时全部通过。



