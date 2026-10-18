# 重启后诊断复现入口

这些是当次实际运行的诊断脚本快照，保留当时的 /tmp 工具路径；不是正式 RTF 配置。
原始长记录保存在 `${HOME}/climbot_data/s6_localization_diagnostics/`，精选数值与栈在
`../gdb_discovery/`；C++ trace 源见 `../pre_restart_records/diagnostic_sources/`。
若需重放，先按脚本路径重建 C++ trace 和私有 GDB，再设置各自唯一的 domain / partition。
GDB 仅解包到临时目录，不安装系统组件，不改变全局 ptrace 设置。

`gdb_udp`、`late_join_udp` 和 `clean_compare` 仅设置 builtin transport 环境变量，
RMW LOCALHOST 仍追加 SHM，属于无效协议 A/B，保留它们是为了可审计，不可据此认定 UDP 效果。
真正有效的是 `effective_udp`，其 XML 明确仅回环 UDP，并使用 SYSTEM_DEFAULT 避开 RMW
追加描述符。正式实现、可移植资源回归入口和作用范围见项目 common DDS helper 与两个 test 文件。
