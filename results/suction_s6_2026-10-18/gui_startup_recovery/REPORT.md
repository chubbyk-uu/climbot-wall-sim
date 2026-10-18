# 双界面启动恢复：范围与失败记录

在源 `413cd89` 的九组 GUI 控制批次中，前四组通过，第五组梯形纵向失败。
实际日志先出现 D3D12 设备移除，再出现 OpenGL 上下文创建失败，GUI 退出 250；
没有首帧／GUI 截图，RViz 等待 GUI，控制 evaluator 因服务不可用退出。
该组 server 的原生 RTF 约 0.999976，不能据此宣布 GUI 或控制通过。
[原批检查](pre_recovery_controls.json)、错误摘录（本地附件：`results/suction_s6_2026-10-18/gui_startup_recovery/failure_extract.txt`）和
[观察器失败](failed_observer.json)保留，完整日志位于 `/tmp/coverage_regression_JVXyjZ/`。

修复限定为首帧前、30 s 内、两条精确错误同时出现的非零退出，最多重试两次，
明确输出恢复记录；仍用私有 Mesa / D3D12 和原渲染配置，不改变物理、相机或照片纹理。
首帧后的异常、无关退出、超时和关闭不重试。宿主 GPU 移除设备的根因仍未确定。
恢复能力通过真实子进程 fixture 红绿对照（旧 250／一次，新 0／两次）验证，
七项新增回归还覆盖次数上限、首帧后失败、无关错误、迟到失败和关闭。
该受控 fixture 不冒充真实设备故障注入；实际九组控制必须在冻结修复源码后全部重跑。

首轮完整并行套件的行为回归通过，lint 对新增测试的多行引号报一项错误
（在 CTest 和 xunit 各计一次，摘要为 1696 tests / 2 failures）；改为规定的双引号后重验。
最终检查结果另见 checks.json。本记录不代表完整 S6 已通过。


本页以反引号标注的本地验收附件不随 Git 发布，见[发布范围说明](../../PUBLICATION.md)。
