# 大区域横向的定位过期：原始失败快照

以下保留排查开始时的记录。后续机制捕获、有效 UDP 配置、正式回归及最终 S6 结论见
[调查](HISTORY_REVIEW.md)和[最终报告](../REPORT.md)；原失败不追认为通过。

源 6dd84af，final_controls_r4 前五组完整通过。bigH 在任务约 1.969 s 因
Filtered odometry is unavailable or stale 安全停止，0 个段完成，因此该组未通过。
本组 GUI／RViz、TF、腔压、原生时钟和无强杀收尾均通过；整段 RTF 约 0.999788
不能替代控制结果。记录约 340 ms 的控制／odom 回调间隔，filtered 接收间隔约
320 ms，另有 total_station 接收／header 间隔。根因尚未唯一确定，未修改任何保护门槛。
原日志 /tmp/coverage_regression_aN5SsW/bigH/，后续补独立时序采样。
