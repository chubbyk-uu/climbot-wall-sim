# 有序退出中的重复 INT

正式 R5 的新结构横向控制、定位、TF、压力和双 GUI 运行检查均通过，原生平均 RTF
0.9998164421，但压力桥未在退出预算内结束，最终被 KILL；因此整组失败，不追认为通过。
原日志 `/tmp/coverage_regression_rcQmQQ/horizontal/sim.log` 记录桥在 06:47:31.535 和
.665 两次处理 INT；相机适配器退出 -11，server 也打印退出期栈。原失败没有等待栈，
不能声称已经唯一解释这个偶发挂住／崩溃。

组级 INT 同时打到 launch 和它的孩子；launch 的有序 shutdown／required 子进程退出
处理又转发 INT。确定性真实子进程 fixture 中，旧 helper 让子进程收到 2 次 INT、
退出 4；根进程统一转发后收到 1 次、退出 0。红测 2 项失败，绿测全部通过。
这是确认的重复信号缺陷及修复，不是对原生中间件内部挂住的唯一复原。

控制 lane 和 G2 共用 `lane_signal_group`：先只通知已登记组的根进程，等待原 8 s；
之后仍按整组 TERM、20 s、KILL 和分区残留检查兜底，任何强杀仍判失败。
独立短启动对照 parents→group→parents 三次根进程均返回 0 且无残留（约 .411/.315/.410 s），
没有稳定重现原挂住，不能把中间 group 的结束记录丢掉。进一步检查三次 server 日志均有退出期段错误，group 还出现 ROS 子进程信号退出；不能把根返回 0 写成全部底层正常。
签名与本机私有参考 `4WIDS_agv/docs/issues/GZ_SHUTDOWN_CRASH_DUMPS.md` 的库卸载后线程访问相同，具体库仍未识别，本轮不声称修复它。见 actual_gui_comparison.json。

红测：`/tmp/climbot_s6_root_int_red_20261018.log`（exit 1）；
绿测：`/tmp/climbot_s6_root_int_green_20261018.log`（exit 0）。
正式九组、传感与拍照在新冻结源码下重验，保留 R5 原失败。
