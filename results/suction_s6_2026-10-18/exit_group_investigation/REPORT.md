# 双 GUI 退出：TERM 与 INT 的实际对照

`final_controls_r3` 的旧横向完整通过，新横向运动／压力／双 GUI 均通过，
但新横向收尾两个进程组在 20 s 后被强杀；失败保留，不追认为通过。
实际退出探针证明是活进程：相同真实 Gazebo／RViz 启动下，发 TERM 后 RViz
在 26 s 仍为 Sl，之后需要强制清理。ROS launch 的 `_on_sigterm` 明确取消并警告孤儿；
`_on_sigint` 才正常 shutdown。不能通过忽略 kill -0 或延长等待掩盖它。

改为 INT（8 s）→TERM（原 20 s）→KILL，控制及 G2 脚本同样处理；
GZ supervisor 继续使用对子 GZ 的安全 TERM 转发。无强杀／无兜底／无残留要求不变。
真实 INT 探针的已知进程组约 0.317794 s 全部退出，补查同分区也无残留。
该时间只指采样的已知组，非图形加载时间或运动／RTF 结论。

新 shell fixture 的实际子进程只接受 INT 正常退出，旧实现有两项断言失败，新实现全通过；
原 TERM 拒绝、KILL、孤儿分区扫描、跨 lane 隔离及延迟 setsid 检查仍通过。
正式控制将在修复源码冻结后从头重跑九组，之后继续传感和带纹理照片。
完整日志和套件结果见 checks.json；进程时序保留在 term_trace.json / int_trace.json。
