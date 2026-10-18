# S7 默认负压底盘收口

2026-10-18。运行验收通过；默认切换提交 `1b6e390`，RViz 修复及正式采集源码 `1846f6a`。
已通过合并提交 `ea7de8f` 进入本地 main；合并后已更新 README、主要文档及包说明，
限定临时数据清理已执行，见 [最终门禁](FINAL_GATE.json) 和 [清理清单](CLEANUP.json)。
持续 Goal 保持暂停，不启动 M2、墙面贴图去重或 0.30 m/s 提速专项。

## 范围和结果

用户先约定小采集且不拼接，执行中改为一次约 3×2 m 中等采集并拼接。本轮只有一组正式
水平任务；不重复最大的横／纵两组、八工况、标定、91 组物理扫描或完整测试套件。
完整物理、控制、传感和绝对拼接质量继续引用 [S6](../suction_s6_2026-10-18/REPORT.md)。

| 检查 | 实际结果 |
| --- | --- |
| 默认结构与 S6 候选等价 | 7,626 个 XML 节点；20 个网格字节相同；消费参数和物理解析输入完全相同 |
| 受影响契约 | 134 项通过；可视化／规划／编排补充回归 37 项通过，分组存在重合，不作为新增独立总数 |
| 默认双 GUI 启动／reset | 未传 robot_geometry 或 adhesion_mode；实际 GZ 与 RViz 同开；压力启动、撤支撑、reset 后再升速／撤支撑通过 |
| 中等任务 | 工作系 `[3.5,2.5]–[6.5,4.5] m`，5 条扫描线／9 个运动段；0.20 m/s |
| 原始归档 | 完成 70/70 张；failed=staged=0；所有 PNG、元数据及图像哈希逐项核验 |
| 精度 | 原墙 DDS 0.25 mm/像素、1920×1080 mono8 不变；相机位置误差 P95 2.049 mm、最大 2.349 mm |
| 双 GUI 实时率 | 原生时钟完整任务平均 **0.98845193**；最小 10 s 窗口 0.97602744；固定 1 ms、目标实时率 1，去掉起始 20 s 预热 |
| 负压 | 最低 8404.4426 Pa，持续高于新质量解析预算 6100 Pa；三机健康、无理想源／台架支撑回退；16,594 个状态样本 |
| 插件 CPU | 均值 30.203 μs、P95 49.578 μs |
| 退出 | 无 KILL、无残留进程组；正式中等组日志无 Gazebo 段错误，不能据此宣称历史环境退出边界已修复 |
| 后处理 | 预处理、输入校验、匹配、位姿图、显式 CUDA 拼接全部 exit 0；约 13.44 s，不重采集 |
| 拼接 | 14,631×8,675，0.25 mm/像素；`execution.effective=cuda`、fallback=false；70 帧／174 条边，连通分量 `[70]` |
| 局部指标 | 位姿图边残差 P95 3.186→0.156 mm；重叠区灰度标准差 P95 5.480→3.114。它们不是对诊断墙真值的绝对误差 |

代码删除旧底盘 SDF/URDF 和恒力 ApplyLinkWrench 路径；共享传感器、灯光和驱动模板独立为
`components.sdf.xacro`。公共 `climbot_wall.launch.py` 与兼容名称 `climbot_vacuum_stack.launch.py`
共用一个入口。规划、控制和评价默认新结构，拒绝 legacy／legacy_constant，保留同插件
fixed_pressure B 诊断。旧模型需从冻结 Git 版本（例如 `cb3fbda`）复现。
IMU 原实际安装位置单列到 simulation.yaml，避免删旧模板时改变传感器外参。

## 本轮发现并修复的问题

第一次 1×0.9 m 小任务虽然拍完 10 张，但 reset 后 RViz 没有墙面、网格和路径，用户和
原始截图共同确认，因此不作为 S7 显示通过。规划器仅发布一次 transient-local 可视化，
RViz 在时钟回退时清空缓存且订阅不断开，不会重新接收历史消息。

`1846f6a` 增加 1 Hz 缓存可视化刷新，只更新显示时间戳，不重新规划、不发布／修改执行任务。
独立时钟回退回归先红后绿；正式 70 张任务截图确认墙、网格、安全框、路线、机器人和实时
相机均存在。见 [首轮失败记录](initial_small_display_failure/assessment.json)、
[针对性检查](targeted_checks.json)及实际采集中截图（本地附件：`results/suction_s7_2026-10-18/live_capture/rviz.png`）。

记录器审计脚本首次把 images/ 当成 PNG 直接目录，实际格式为 images/raw/，导致计数检查失败；
70 张图本身均已正确逐项验证。修正递归目录核对后通过，不重跑采集。
原始失败摘要保留。重跑审计的目录存在性修正也仅影响工具，不改变原始证据。

## 图像复核与边界

已看优化母版概览、三处 512×512 原像素对照（施工缝及两处纹理）、一张原分辨率原图，
以及启动和采集中的实际 GZ/RViz 截图。优化后施工缝与纹理对齐改善，样本未见明显重复结构；
仍有轻微 hard-cut 亮度阶梯和光学柔化，不声称无接缝。

母版外沿阶梯／黑色缺口来自交替方向相机足迹的不同端点；本次不要求整面墙或整个矩形外包框
满覆盖。没有逐像素审阅全部母版，也没有重新执行完整墙的绝对真值验收。
真机风机曲线、裙边和漏气仍为暂定仿真参数；全部不利参数组合、掉墙后重密封及撞地物理
不因 S7 通过而扩大结论。0.30 m/s 和 M2 不在本次范围。

## 数据与复现

- [运行门禁](RUNTIME_GATE.json)、[展开等价审计](s6_equivalence.json)、[采集逐项审计](capture/audit.json)。
- [后处理命令、时长和数据相对路径](postprocess/execution.json)、[原生 CUDA manifest](postprocess/mosaic_manifest.json)、[人工复核](postprocess/visual_review.json)。
- 原图位于 `$CLIMBOT_DATA_ROOT/suction_s7_medium_20261018/`；processed、matches、pose-graph、mosaic
  目录由后处理记录列出；原始照片、曝光信息、标定、母版和覆盖图均保留。
- 压力及完整生成场景、S6 前后展开原件在 `$CLIMBOT_DATA_ROOT/acceptance_evidence/suction_s7_20261018/`，
  全文件哈希见[保留清单](retained_data_index.json)。
- `replay/medium_task.yaml` 冻结任务；`replay/medium_capture.sh horizontal` 重建独立 lane。
  该脚本从仓库根目录运行，需为新采集指定新的 `INSPECTION_OUTPUT_ROOT`，原精度纹理、双 GUI
  和 Mesa 设置见 [source_manifest](source_manifest.json)。后处理／审计支持 `CLIMBOT_S7_LANE`
  指向新 `/tmp/.../horizontal`；重跑还需给派生输出新的目录名，不能覆盖正式数据。
- 完整临时输出在 `/tmp/climbot_s7_*.log`，未提交；模拟器、规划、控制、评价等各自日志位于
  本次 `/tmp/climbot_g2_p5hodY/horizontal/`。读取成功结果只取摘要，未反复加载完整运行日志。

## 最终收口

本地 main 的合并提交为 `ea7de8f40645e75938ab0b1259f16268f2790dbd`，包含全部 S6／S7
运行源代码与证据；随后在 main 更新 README、项目指南、架构、接口、操作、状态、验收、
吸附计划、数据保留、结果索引和三个相关包 README。新 README 使用本次实际新模型截图。
未推送远端；功能分支保留供追溯。

清理实际删除 25,451,917 逻辑字节，包含首轮替代小任务 10 张 PNG、重建缓存及已校验归档的
工作副本。首轮元数据与图像哈希保留，已删除的 PNG 不能从哈希恢复。
70 张正式原图、全部曝光信息、标定、处理结果、全分辨率母版、压力和生成场景证据完整保留。
本次正式任务和离线步骤均完成，无后台验收进程继续运行；不自动进入后续专项。


本页以反引号标注的本地验收附件不随 Git 发布，见[发布范围说明](../PUBLICATION.md)。
