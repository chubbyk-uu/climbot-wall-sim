# Bash 后台进程继承忽略 INT，阻断 launch 有序关闭

R6 梯形横向运动、控制、压力及实时率通过（0.9999494301），但 planner/RViz 组
最终需要 KILL。此前短 GUI 退出对照直接由 Python Popen 启动，不能覆盖 Bash 后台的信号继承。

实际 ROS launch 最小场景，不加载 Gazebo、DDS 图像或模型：

| 启动方式 | 根进程忽略 INT | 子进程收到 INT | 根进程按 INT 结束 |
|---|---|---|---|
| Bash 后台 setsid | 是 | 否 | 否，2 s 后需 TERM |
| 启动前重置 INT | 否 | 是 | 是，32.8 ms |
| 项目 start_session.sh | 否 | 是 | 是，32.1 ms |

Bash 后台继承 SIG_IGN；仅发送 root INT 并不能唤醒这种 ROS launch。
新入口在执行 Python 前以 `env --default-signal=INT` 恢复默认，再保持同一 PID 进入新会话。
控制回归和 G2 的所有会话启动共用该入口；根 INT 与整组 TERM/KILL 兜底不变。

`tools/test_lane_processes.sh` 新增真实后台根进程、不手工安装 INT handler 的回归：
旧入口丢失 INT（红测 1 项失败），新入口全部通过。此前的假 launch fixture 显式安装
handler，遮蔽了继承问题；保留它用于检查重复 INT，新增 fixture 补上真实启动边界。
日志 `/tmp/climbot_s6_inherited_int_{red,green}_20261018.log`。

本次不改变任何 src 字节、物理／控制／图像参数。保留 R6 前三组满足原门槛的完整任务
和各自源码提交；失败的梯形不复用。后六组在新入口下执行，以显式哈希审计合并九组，
不把混合提交写成单次同提交全量运行。最终完整套件在收口时执行。
