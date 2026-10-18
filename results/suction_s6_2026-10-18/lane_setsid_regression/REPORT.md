# 进程组登记竞态

源 `76b0cbd` 的旧结构横向实际控制、GUI／RViz、RTF 和 TF 等检查通过，
但收尾发现仿真组未登记，兜底扫描才将其清理，因此本次完整验收失败。
原始记录见 observed_control_checks.json、observed_teardown.txt；没有残留进程。

`setsid ... &` 返回子 PID 时，子进程可能还没完成 setsid；旧 helper 只读一次 PGID，
恰好看到继承的当前进程组就拒绝登记。修复轮询该过渡，最多 5 s，仍拒绝登记自身进程组，
没有放宽 teardown 检查或屏蔽兜底告警。

新增确定性 fixture：第一次 ps 返回真实旧组后才放行子进程完成 setsid。
正式旧／新 helper 红绿串行执行，旧实现只在新增的两项断言失败，新实现全通过。
初次并行诊断曾共用默认 lane1，产生 fixture 相互清理，已排除；测试现使用每次独立前缀。
完整检查与日志见 checks.json。后续 final_controls_r3 从同一新提交重跑九组；
本失败不因功能检查通过而追认为完整通过。
