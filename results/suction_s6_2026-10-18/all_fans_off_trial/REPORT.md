# 三台风扇全停的 GUI 跌落试验

2026-10-18，用户要求现场验证。持续 Goal 保持暂停，仅做本项演示。

12 秒 Gazebo 实录（本地附件：`results/suction_s6_2026-10-18/all_fans_off_trial/all_fans_off.mp4`）：约第 3 秒同时关闭三台风扇，机器人从约 2 m
高处下滑、翻倒到地面。关机前（本地附件：`results/suction_s6_2026-10-18/all_fans_off_trial/before.png`）、落地后（本地附件：`results/suction_s6_2026-10-18/all_fans_off_trial/after.png`）。Gazebo GUI 与 RViz
同时打开，使用当前候选的 fan_ode 模式、私有 Mesa 和原 DDS 墙面，没有修改物理参数。

以首次三台 enabled 均为 0 的真值样本（仿真时间 457.93 s）为参考：

| 观察 | 结果 |
| --- | --- |
| 关机前压差／基座高度 | 8404.44 Pa／1.99998 m |
| 高度下降超过 10 mm | 约 0.19 s 后 |
| 压差降到 1 Pa 以下 | 约 0.66 s 后 |
| 基座降到 0.4 m 以下 | 约 0.73 s 后 |
| 最后成功发布的吸附状态 | 约 0.90 s 后，压差 0.190 Pa |
| 最终基座位置 | x=0.313 m，y=-0.069 m，z=0.225 m |
| 启动支撑 | 关机前已撤除；停机后没有重新启用 |

**本次发现未修复的模型边界缺陷，不能判为完整跌落物理验收通过。** 接近落地时插件记录
`skirt compression hard limit: invalid prescribed pose`，进入 runtime failure，随后不再
更新／发布腔体状态，也不恢复恒力或临时支撑。Gazebo 刚体仍继续运动到地面。最后发布的
`runtime_failed=0` 是异常前的旧样本，不能用它声称全程没有模型故障；显示动画也会因
真值消息断流停在最后一帧。开始下滑和大部分下降段的负压模型仍在正常运行。

后续应明确机器人失去密封、翻转和撞地时模型的有效域／停用行为，并使故障状态可被外部
及时观察；不能简单删掉压缩限值或静默添加吸力来让试验通过。本次只记录问题，不改模型。
此前 S5 的全停失去接触结论保留，这次实际 GUI 试验不追加完整落地有效性的声明。

试验后执行 world reset，generation=1；已验证三个风扇重新启用、压差恢复至 8.38 kPa
以上、启动支撑再次撤除、runtime_failed=0，保留正常吸附预览。

[摘要与哈希](summary.json)、错误摘录（本地附件：`results/suction_s6_2026-10-18/all_fans_off_trial/error_excerpt.log`）、压缩原始真值／位姿记录和
本次脚本（本地附件：`results/suction_s6_2026-10-18/all_fans_off_trial/reproduce.py`）同目录保存。脚本要求已运行的同一预览分区，并在 `/tmp` 新建
独立输出目录；它会关闭三个风扇并在录像结束暂停现场。恢复操作由本次会话另行执行。
完整模拟器日志仍在 `/tmp/climbot_fan_animation_v2_preview_20261018/simulator.log`。


本页以反引号标注的本地验收附件不随 Git 发布，见[发布范围说明](../../PUBLICATION.md)。
