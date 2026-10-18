# S6 定位断档：三个本机项目的问题记录核对

状态：重启后捕获 SHM 发现锁等待机制，已实现本机关键节点的显式回环 UDP；正式 S6 重验待完成。原失败没有线程栈，不能唯一追认历史实例。

## 历史上相似表象的不同原因

| 来源 | 已记录的原因与证据 | 本次适用性 |
| --- | --- | --- |
| 本项目 INCIDENTS §7、提交 6227a3c | WSL 系统时间回拨 1.2–1.7 s，系统时钟定时器与看门狗一起冻结 | 必须核对。当前控制启动显式 use_sim_time=true，诊断 2 的实际运行参数查询也返回 true；controlClock 使用 ROS 仿真时钟；非仿真路径使用 steady。直接系统时间定时的旧缺陷已有修复，但不能据此排除中间件／宿主的时序影响。 |
| 本项目 INCIDENTS §11 | 逐图 fsync 造成 I/O 尖峰；改为批量 syncfs，但后者仍刷新整个文件系统，不能保证单次延迟上界 | 当前失败没有拍照归档，不能直接归因采集同步。仍须测诊断写盘及系统回写压力。 |
| 本项目 INCIDENTS §12 | 大图超过默认 Fast DDS SHM，可靠重传等待约 3 s；三个 writer 改为 64 MiB profile | 当前失败在灰墙控制工况、没有图像捕获服务，量级约 0.34 s；匹配度较低。后续拍照仍须核对实际 writer profile。 |
| 4WIDS_agv/docs/issues/CONTROL_LOOP_STALL.md | WSL 脏页回写引起约 0.35 s I/O 停顿，同步日志／遥测发布将其传进控制线程；控制停顿被误报定位失效。已有分段墙钟／线程 CPU／等待点证据；移出线程后 66,845 拍无停顿，同期仍有系统 I/O 压力 | 本次 timer 与 odom 回调同时约 340 ms，量级接近，应重点核对。但尚无本次 I/O、syscall 或分段证据，不能宣布同根因。 |
| 4WIDS_agv/docs/issues/SCAN_STABILITY.md | 面板与控制共享单线程，静态显示重复可靠发布；测得回调约 21 ms，未唯一复现历史 259 ms。之后隔离进程，并冻结面板验证控制不受影响 | 本项目 tracker 已独立进程；仍有同步 DDS publish 和日志调用。不能把 BEST_EFFORT 当作 publish 不阻塞的保证。 |
| 本项目 INCIDENTS §13 | GPU／WSLg 运行期挂起，必须有运动期新 dxg 簇和全流断档同窗 | 原失败没有对应 raw wheel／IMU 断档记录，暂不符合充分判据；不能靠启动 dxg 总数认定。 |
| Subway_scan_bot_sim/docs/DEVELOPMENT_RULES.md 与 history/REPAIR_PLAN_2026-10-02.md | 仿真或 steady 计时；有界异步写盘，阶段哈希，避免高频小文件／全局同步；该文引用 climbot 历史教训 | 是可借鉴的处理原则，不是本次已证实的根因。 |

以上两个外部项目均为本机私有参考，外部读者可能无法访问。

## 原失败能够证明和不能证明的内容

源 6dd84af，原日志 /tmp/coverage_regression_aN5SsW/bigH/。

- 任务开始后 1.969 仿真秒安全停止；control_timer 间隔 341.316 ms，odom 回调间隔 340.139 ms。
- 独立 monitor 的 filtered 接收间隔 320.175 ms、header 间隔 160 ms；total_station 接收 292.329 ms、header 296.667 ms。
- 最后 stale 日志距 odom 回调入口仅 1.398 ms，且没有 stamp 拒绝日志。这说明“回调刚来过”不等于源时间新鲜；也可能涉及 clock 追赶或回调内部延迟。旧日志没有判定当刻的控制时间／源时间快照，不能区分。
- 没有原失败的系统／单调双时钟、I/O PSI 或线程等待点记录。不能事后补证或唯一归因。
- native 平均 RTF 0.999788 只证明吞吐，不排除短时停顿或追赶。

## 独立低扰动诊断 1

输入与执行脚本：/tmp/climbot_s6_gap_diagnostic_20261018.py；记录目录：
/tmp/climbot_s6_gap_diagnostic_1792305851505596393/bigH_0。
不改变运行源码和任何控制门限；附加 C++ 内存 trace，退出后写 CSV，另有 10 Hz /proc 采样。

- 完整 bigH 成功，29,682 轨迹样本。仅为诊断，本次没有复现原任务期故障。
- ROS clock／filtered／wheel／IMU 在初始化阶段仍有约 0.33–0.46 s 接收断档，源时间仅前进约 2–20 ms；这是仿真／接收停顿，不能当作系统时钟回拨。
- filtered 在整个 trace 的最大 ROS 源年龄 19 ms；没有证据表明本次发生原故障。
- 334.1 s 采样中，系统时间相对单调时间的观测漂移约 -0.83 至 +7.26 ms；没有观测到历史秒级回拨。采样未夹逼计时，包含读取调度误差，不能用作高精度时钟漂移测量。
- dmesg 前后逐行差集为零，当前诊断不支持新的 GPU 驱动错误。

诊断 2 增加夹逼双时钟、I/O／内存 PSI、脏页／回写、各进程 I/O、主线程 CPU／调度等待／syscall／wchan。仍须在同一断档窗口做相关性核对；成功重跑不关闭原故障。

## 原失败的额外边界核对

- 原生 WorldStatistics 在评分期没有超过 200 ms 的接收间隔；插件真值在失败附近继续按 10 ms 源时间步进（其较大源缺口只出现在 0.42–0.49、0.58–0.81、11.82–12.08 s）。
- 因而本次证据更偏向 ROS 定时／执行／传输链路局部停顿，尚不足以认定 Gazebo 物理整体冻结或全虚拟机停止。I/O 也可能只阻塞执行了系统调用的线程，不能仅凭其他流继续而排除。
- 这些检查没有记录原失败时 CLOCK_REALTIME 与 CLOCK_MONOTONIC 的差值，仍不能事后排除时钟回跳。

## 独立诊断 2 的初步结果

目录：/tmp/climbot_s6_gap_diagnostic_v2_1792306288615899596/bigH_0。
完整 bigH 成功，29,276 轨迹样本；只计诊断。

- 在任务接收前约 0.69 s 出现相似的局部停顿：filtered 接收间隔 345.490 ms，源时间 17.56→17.58 s，但接收端 ROS 时间 17.562→17.906 s，首个迟到样本源年龄 326 ms。tracker 同时明确拒绝该过期源时间，之后恢复。
- 此次事件窗口 I/O PSI some/full 增量均为零；CPU some 累计约 10.915 ms，内存压力零。整个任务系统时间相对夹逼单调时间漂移约 -0.00512 至 +0.017664 ms，trace 未观察到 ROS clock 或源时间回退。这一次事件不支持系统时钟回拨或 guest I/O 压力解释。
- 整个任务 I/O PSI some/full 分别增加约 72.515/57.715 ms；不能把任务总数当作断档时刻的证据。
- 线程采样有诊断缺陷：当前权限不能读取 /proc/<pid>/syscall，/proc/<pid>/io 读取返回不存在；旧采样器将可选字段失败与整条进程记录一起丢弃，导致本轮没有有效线程记录。已修正为显式 unavailable，保留其余调度／wchan 数据。sudo 无免密权限，未更改全局 ptrace 配置。
- 第二次重复已主动 SIGINT 收尾（exit 130），因为该版本无法取得所需线程证据；不计验收或成功重复。

诊断 3 改用短直线工况，并增加只在诊断子进程中启用的临时 RCL 调用探针。探针在内存保留 ≥20 ms 的 publish/take/wait/log/timer/clock 调用及线程 CPU 时长，退出后统一写盘；不修改项目源码或系统设置。独立假 publisher 睡眠 40 ms 的夹具验证，探针记录 40.092 ms 墙钟、约 0.020 ms 线程 CPU 及正确 topic。实际分层结果仍待运行完成。该探针不能作为正式性能验收配置。

## 暂停与结论边界

用户要求暂停以重启 WSL。最小发布／晚加入订阅实验默认→UDP→默认三组均没有复现慢 publish，故 333 ms 健康检查源码只能作候选；不能据此归因。临时函数拦截探针 v3 那轮出现相机进程异常，已停止，该轮无效；后续 v4 的应用进程保持原样。
UDP 两次短测之后，默认反向两次短测 filtered 最大源年龄同样只有 2 ms，说明该 A/B 不能唯一识别传输根因。
另两个项目长期稳定这一事实要求优先核对本项目链路／配置差异，WSL uptime 本身没有因果证据。当前保持暂停，不改变保护阈值、传输默认值或验收结论。详见 RESTART_CHECKPOINT.md。

## 重启后：捕获的机制及有效修复

新 boot ID `959746e5-c38a-4399-888a-69dff7f192d0`；问题在重启后仍可出现，
不支持把运行时长或 Mesa 内存泄漏当作原因。私有 GDB 仅在请求 333 ms nanosleep 时截栈，
捕获 `SharedMemTransport::find_port → push_discard → send → ReaderLocator →
StatefulWriter::matched_reader_add → EDP`；同一快照的 SHM 接收线程等待
`PDP::assert_remote_participant_liveliness` 的锁，主执行器等待 DDS WaitSet。
该发现锁内端口健康检查可阻塞接收，与约 0.34 s 局部断档吻合。
捕获发生在任务开始前；调试器会干扰计时，不作为 RTF 验收或原 bigH 实例的唯一复原。

必须更正早期“UDP”对照的含义：只设置 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` 时，
本机 RMW LOCALHOST 分支仍额外追加 SHM，实际 maps 中仍有 fastrtps 共享段。
`udp.json`、`clean_comparison.json` 以及重启前 env-only 对照均不能作为有效协议 A/B；
原数字保留，成功重跑及零断点命中均不能证明去掉了等待。原最小 publisher 实验也是阴性结果。

[rmw_fastrtps 8.4.4 原始源码](https://github.com/ros2/rmw_fastrtps/blob/8.4.4/rmw_fastrtps_shared_cpp/src/participant.cpp)
的 LOCALHOST / SYSTEM_DEFAULT 分支解释了这个差异。有效配置使用仅回环的显式 XML，
关闭 builtins，并对子节点使用 LOCALHOST_ONLY=0、DISCOVERY_RANGE=SYSTEM_DEFAULT。
实际 /proc 资源检查证明关键节点无 SHM transport mappings、UDP 仅绑定 127.0.0.1；
图像发布链路保留原 64 MiB profile。

无调试器、10 个默认传输订阅者中途加入的完整双 GUI 直线诊断：
13,815 轨迹样本、控制最大间隔 24.449 ms、odom 23.916 ms、filtered 最大源年龄 8 ms、
原生平均 RTF 0.9997947703，正常收尾。见 `gdb_discovery/effective_local_udp.json`
与 `effective_runtime.json`。这属于诊断，正式九组和后续采集需要冻结新源码后重新完成。

回归不只断言环境变量：实际启动 Fast DDS participant 检查 SHM maps 和 UDP socket 地址；
另验证 launch 传播、操作员配置优先、网络范围保留以及缺失 profile 拒绝回退。
