# 巡航时间戳错配复现、修正与启动拒绝诊断

日期：2026-10-17。当前修复代码 `c44b318`；本批是 ROS 节点与平面运动夹具，未运行 Gazebo，
不替代 S5/S6 的真实接触运动及完整任务。旧 S0/S3 记录保持原样。

## 结论与适用范围

时间模式曾将当前计划位置与旧时间戳的里程计位置比较。反馈只是晚到，也会产生约
`v × age` 的虚假进度落后，再经比例增益触发追赶。修正只把反馈误差对齐到里程计时间，
前馈仍取当前计划速度；真实滑移造成的落后仍可追赶。未调整 0.205 m/s 测试门槛、0.20 m/s
巡航、0.35 m/s 追赶上限、增益或新旧几何。

原始 0.210464 m/s 那一刻没有逐周期数据，无法重建唯一的消息顺序。本批以受控同类复现、
修改前必失败的节点回归、修改后通过和真实滑移正对照，关闭已证明的时间错配缺陷，
不将“后续绿色”当作历史某个样本的唯一根因证明。历史信息缺失作为溯源限制保留。

## 受控实验

复用 `test_coverage_executor.py` 的原始启动参数及 4 m SCAN：仿真步 10 ms、10 倍时间，
持续正常 capture heartbeat、相同平面运动夹具。只在 x=0.8～1.0 m 内暂缓 6 个里程计样本，
其间仿真时钟与实际夹具运动继续，之后恢复。输入仍在原 2 s 新鲜度限制内。

| 对照 | 原实现峰值 m/s | 修正后峰值 m/s | 原 0.205 门槛 |
| --- | ---: | ---: | --- |
| 正常里程计 | 0.204998824 | 0.1999999997 | 两者通过 |
| 暂缓 6 个样本 | 0.218992268 | 0.1999999999 | 修改前失败，修改后通过 |

独立新增节点回归 `test_delayed_odometry_does_not_create_false_cruise_catchup`：在修改产品
代码之前实际得到 `0.218798528 > 0.205`，其余用例通过；[红色结果](red_regression.json)
保留失败栈。修改后沿用同一断言通过。原有
`test_line_tracker_time_mode.py::test_falls_behind_under_slip_and_then_catches_up`
仍通过，其中要求真实滑移后峰值超过巡航的 1.05 倍；不是用取消追赶换来绿色。

记录见 `before/`、`normal.json`、`delayed_odometry.json`。逐步输入、命令和 Action 反馈均
保留；全段 schedule_lag 包含加速／取消阶段，不能将其全段峰值与某个巡航命令硬配成同周期。
旧二进制未留 SHA-256，清单明确记为 null；旧源码版本、当前源码与二进制摘要另列，
不事后伪造旧二进制身份。正常和延迟实验共享同一 tracker，使用连续仿真时钟，每段退出前
取消并等待 Goal 终态，最后进程正常退出 0。

## 归档启动拒绝

此前已补齐管理器依赖发现等待。本次在两个启动服务统一记录完整拒绝理由：
`Coverage start refused: <reason>`，不记录输出根目录，也不改变拒绝／接受逻辑。
原有 forced-recovery 测试在确实没有 executor 时，既验证 service 的 unavailable 响应，
又等待相同原因出现在进程日志，然后才启动 executor 继续原恢复流程。

原历史 Start=false 缺失的具体端点无法补回；当前防护、响应和日志链已经补齐，这不再作为
待开发缺口。测试仅重试尚未派发预检或 Goal 的两类 unavailable，其他拒绝仍立即失败。

## 验证与复现

最终连续两轮 `colcon test --executor parallel --parallel-workers 8`：每轮 **1555 tests，
0 errors，0 failures，74 skipped**，跳过来自原 cppcheck 性能检查。包含新的延迟回归、
真实滑移追赶、暂停恢复、失去里程计、归档拒绝日志及原生吸附测试。
详见 [回归记录](regression_validation.json) 和 [检查结果](validation.json)。

完整日志留在临时目录：`climbot_cruise_timestamp_final_regression_{0,1}_20261017.log`；
原修改前后诊断日志与无测试匹配的第一次筛选不作为通过证据，真正的红色节点回归单列。
正式 collector 的实际控制器日志名见 `execution.json`。

在加载 ROS 与已构建工作区后，以全新目录复现修正后两组输入：

```bash
ROS_DOMAIN_ID=223 python3 tools/probe_cruise_timestamps.py --output results/全新目录
```

源版本与哈希见 [source_manifest.json](source_manifest.json)。本次没有改变吸附核心或覆盖
S3 原始文件；实际 Gazebo 八工况、长采集及最终默认仍须按 S6/S7 复验，不外推节点级结果。
