# 实验结果索引

更新：2026-10-22。Git 只保留精简报告／摘要，完整附件在本地数据根，见[发布范围](PUBLICATION.md)。
以下证据按冻结配置分别有效；旧报告中的“未开始／默认关闭”按当时阶段解释。
参数探索、失败方案和旧基线统一从[分阶段索引](HISTORY.md)与[早期结果归档](archive/RESULTS_2026-08-27.md)查找。

## 当前引用入口

| 范围 | 报告／摘要 | 证据边界 |
| --- | --- | --- |
| M2 默认摩擦 0.8 | [报告](m2_friction_2026-10-22/REPORT.md)、[摘要](m2_friction_2026-10-22/summary.json) | 0.7 未过原门槛后采用 0.8；横纵 22 张、固定 k/c 的 450 N 上界、完整回归通过；未重跑整墙／全部故障 |
| M2 定位标定 | [报告](m2_localization_calibration_2026-10-22/REPORT.md)、[摘要](m2_localization_calibration_2026-10-22/summary.json) | 带噪声数据标定 Q／延迟历史；传感器 R 不变；缺测相对退化边界保留 |
| 标定后竖向整墙 | [报告](m2_calibrated_full_vertical_2026-10-22/REPORT.md)、[摘要](m2_calibrated_full_vertical_2026-10-22/summary.json) | 660 张、最大落点误差 4.740 mm；尺度候选改善几何，但覆盖／原尺寸接缝未闭合 |
| 接缝有界验证 | [报告](m2_seam_bounded_2026-10-22/REPORT.md)、[摘要](m2_seam_bounded_2026-10-22/summary.json) | 新增整墙仿射拒绝，本轮停止；保留 SE(2) 默认与显式尺度选项 |
| M2 T3 默认切换 | [报告](m2_t3_2026-10-20/REPORT.md)、[摘要](m2_t3_2026-10-20/summary.json) | μ=1.1、双 GUI 660 张、RTF 0.979；不是 μ=0.8 的整墙性能证明 |
| M2 T2 耦合 | [报告](m2_t2_closure_2026-10-19/REPORT.md)、[摘要](m2_t2_closure_2026-10-19/summary.json) | 注册代表区域、留出与收敛；失败球座不启用 |
| M1 S6 全链 | [最终报告](suction_s6_2026-10-18/REPORT.md)、[总门禁](suction_s6_2026-10-18/FINAL_GATE.json) | 新质量下物理、控制、采集与拼接冻结证据；不外推为 M2 |
| M1 S7 交付 | [报告](suction_s7_2026-10-18/REPORT.md) | 当时默认切换、70 张双 GUI 中等任务；现行几何默认后来由 M2 取代 |
| 墙面纹理 | [局部配置](wall_texture_local_2026-10-18/REPORT.md)、[采集](wall_texture_capture_2026-10-19/REPORT.md)、[审核](wall_texture_review_2026-10-19/REPORT.md) | 专项关闭；边缘漏拍单列，不追认为完整矩形零漏拍 |
| P2-06 平墙 | [验收域与冻结数值](../docs/MOSAIC_PLAN.md#p2-06-正式结果)、[分阶段证据](HISTORY.md) | 冻结墙面／任务数值独立检验通过；第 5 组逐张原尺寸人工复核未完成 |
| 新机器复现 | [报告](fresh_clone_2026-10-18/REPORT.md) | 当时独立克隆、资产与标定检查；当前部署按[安装](../docs/INSTALLATION.md) |

当日首轮尺度、完整姿态与缺口诊断是过程证据，见[分阶段索引](HISTORY.md)。
摘要路径使用环境变量占位；临时日志名不保证对应文件永久存在，不改变原始 producer 哈希。
报告的历史数字不随当前默认改变，JSON／原图 provenance 不补写。

## 已撤销

| 证据 | 撤销原因 |
| --- | --- |
| `mosaic_p206_blind_2026-08-30_summary.json` | **provenance 不成立，不可引用。** 它声明 `commit=ec62a54`、`source_modified=false`，但产生它自身覆盖字段的代码直到 `139a21e` 才存在——那个 provenance 块是手填的，没有任何程序算过。文件保留在原处以便对照，测量本身没有问题（同一批数据在 `20260830c` 上逐位复现），可引用的版本是 `mosaic_p206_blind_2026-08-30b_summary.json`。 |
| `mosaic_p206_blind_2026-08-31_summary.json` | 已由 `2026-08-31c` 取代：旧摘要缺少 off-seam 分母，并包含在本墙面数据上不具判别力的结构边位移代理；其余测量并未被否定。 |

## 数据可用性

10-22 两批已授权精简后，部分 processed PNG、检查 tile、特征缓存和差异 TIFF 需重建；
优化母版、原图、录包、测量与人工裁片保留。具体路径与回执只在[数据保留](../docs/DATA_RETENTION.md)维护。
