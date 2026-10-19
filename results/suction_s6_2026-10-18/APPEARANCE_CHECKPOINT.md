# 外观与双界面启动阶段记录

2026-10-18。用户暂定本次外观并要求提交；持续 Goal 保持暂停，S6 尚未验收完成。

## 当前效果与模型边界

保留前部倾斜圆润主臂，恢复双圆管提手：中心距 80 mm，降低 20 mm，尾端合为加粗支架，
斜向连接车尾，避免竖直下落。补齐可辨认的三风机圆形出风口、静止叶片／护圈、外翻裙边、
灯、轮轴与轮毂，并使用与旧结构相近的橙／灰／黑／蓝配色。

斜视（本地附件：`results/suction_s6_2026-10-18/appearance_checkpoint/oblique.png`）、侧视（本地附件：`results/suction_s6_2026-10-18/appearance_checkpoint/side_profile.png`）、
俯视（本地附件：`results/suction_s6_2026-10-18/appearance_checkpoint/top.png`） 均为最终双管版本的实际 Gazebo 窗口截图。
[外观不变量检查](appearance_checkpoint/appearance_invariance.json)确认：双管间距、降低量和
斜向尾座符合设定；原主臂网格不变，SDF／URDF 去除 visual 后与前版一致。

这不是新的物理最终候选：当前质量仍为 14.869704049819479 kg，提手等外观网格新增质量、
惯量和风机出风遮挡尚未建模。外翻裙边也是显示代理，不能从其网格面积推导有效受压面积。
外观阶段没有修改压力插件、轮胎碰撞、控制、相机光学或物理质量配置。

## 启动与实时率修改

- 本机 WSL／私有 Mesa 的大 DDS 上传中，CPU 采样显示 REP MOVSB 占约 96.91%。
  在该运行时为 glibc 禁用 ERMS／FSRM，保留其他 tunable；`CLIMBOT_MEMCPY_MODE=default`
  可显式关闭。离开私有 Mesa 时恢复原环境，不替换共享 Mesa。
- 未改变原 320 张 DDS、约 1.07 GB 资产及 0.25 mm 采样精度。相同静态相机姿态的
  1920×1080 原生 RGB 在优化前后逐字节相同。该直接 CLI 对比的两个进程都曾在退出时
  返回 -11，所以它只证明像素／加载时间，不能作为生命周期通过证据。
- 优化前完整场景首个 100 物理步约 43.44 s；删除机器人 visual 后仍为 43.15 s，
  新机器人配平面墙约 2.76 s，表明主要瓶颈在墙面纹理上传。优化后的独立同场景诊断
  约 4.69 s。后续双界面串行启动三次，server 约 5.11～5.19 s，GUI 约 8.59～8.80 s。
  数字属于本机诊断窗口，不是跨机器性能保证。
- 顺序为 server 物理推进到 100 步，再开 Gazebo GUI；收到实际 GUI 相机姿态后再开 RViz。
  等待有明确超时并失败，不静默回退。RViz 也使用本地 Mesa，避免落入高 CPU 的 llvmpipe。
- Gazebo 右下角使用 132×32 的原生瞬时 RTF 显示，暂停时显示 Paused，不将数值钳成 100%。
  窗口显示值不能代替完整任务平均实时率。

[三次串行启动诊断](appearance_checkpoint/serial_startup_diagnostic.json)与
[最终双 GUI 启动检查](appearance_checkpoint/dual_gui_startup_smoke.json)保留原始摘要。
最终短静态检查两个启动进程均正常退出；[观察摘要](appearance_checkpoint/dual_gui_static_observation.json)
中约 1.0000 的 ROS 时钟比仅是静态诊断，不能宣称正常运动／拍照时的完整任务 RTF 已通过。
本次未降低拍照精度。先前右下角黑色纹理块和一次 Qt 上下文创建失败的底层唯一原因
未被证明；串行初始化后的重复检查未再出现，仍需正式长任务验证长期稳定性。

## 求解器复验证据交接

源码 `1cd16ba2d98725cd3ff8b9778728ca6f154ed5ad` 的静态复验见
[231 项检查](solver_native_static/validation.json)及[独立审计](solver_native_static/raw_audit.json)；
运动见 91 组摘要（本地附件：`results/suction_s6_2026-10-18/solver_native_motion/summary.json`）及
[独立审计](solver_native_motion/independent_audit.json)，698 项／139 项均通过。
这些结果属于记录中的旧物理配置，不自动转用于未来质量更新。

`capture_solver_retest/` 的横向采集因用户暂停而中断（exit 130），纵向未执行，不能作为
正式采集通过。根目录及 `single_gui_epoch_interrupted/` 的 workflow JSON 和 replay 中旧
驱动脚本是历史快照，其中 PID／waiting／in_progress 不表示当前仍有任务运行；不要执行旧
PID 等待脚本来恢复。后续使用新输出目录及当前源码入口重新启动。

270 份新增原始压力／接触／运动压缩数据，共 1,062,741,774 字节，已校验后完整归档至
`${CLIMBOT_DATA_ROOT:-$CLIMBOT_DATA_ROOT}/acceptance_evidence/suction_s6_20261018/`。
data_artifact_index.json（本地附件：`results/suction_s6_2026-10-18/data_artifact_index.json`）记录路径、大小和 SHA-256，共 276 份索引；
原冻结索引、历史 tracked 数据均未改写。工作输入仍保留，本次没有提前执行验收后清理。
可用 `python3 results/suction_s6_2026-10-18/replay/restore_pressure_records.py` 校验归档，
加 `--restore` 恢复缺失输入。历史 manifest 中原始压缩文件哈希仍适用。

## 提交前回归与一次新发现

构建、完整 `colcon test --executor parallel --parallel-workers 8`、逐轮结果和源码哈希见
[提交检查记录](appearance_checkpoint/commit_checks.json)。第一次完整回归未通过：新增
测试 docstring 的 D403 已修正；已有状态测试在新用例中持续发里程计，却没有新回调，
上一用例最后回调约 2 s 后触发真实保护。原次缺少端点发现的逐时采样，不能唯一追认 DDS
底层原因，也不能把它记成负载下的计时噪声。

夹具确有可独立复现的缺口：服务已发现不表示新传感器发布端已匹配；缓存的旧位姿可以让
任务先启动，再因输入未送达过期。受控让输入端点 3 s 不匹配（超过原 2 s 看门狗），
旧夹具稳定得到 FINISHED 而非 EXECUTING，修复后通过。现在每次启动前确认订阅匹配，
并等待可靠消息确认送达。产品看门狗、保护和状态断言不变，2.2 s 观察窗口也保留。
[红绿证据](appearance_checkpoint/sensor_endpoint_regression.json)记录该边界及源码哈希。

## 下一阶段顺序

1. 确认并补齐提手／支架等新增部件的等效质量、重心、惯量及必要碰撞代理；重算垂直墙
   全航向三轮载荷和最低吸力／压差，再审查风机／密封圈配置，不能倒推 400 N。
2. 冻结候选及启动参数，在 Gazebo GUI 与 RViz 同时打开时重跑旧／新各八个控制工况、
   定位／侧滑／相机受影响项和旧／新横纵长采集。完整正常任务原生平均 RTF ≥ 0.95，
   保持目标实时率 1、物理步 1 ms；记录界面、加载时间和黑块是否复现。
3. 对完整采集显式执行 CUDA 拼接（`--backend cuda`），确认 manifest 的 effective backend，
   按既有图像／覆盖／拼接门限验收，归档正式结果后按授权清理临时数据。
4. S6 真正关闭后再单独讨论 S7 默认切换、M2 起伏墙／柔性轮和墙面纹理随机化；本次不进入。


本页以反引号标注的本地验收附件不随 Git 发布，见[发布范围说明](../PUBLICATION.md)。
