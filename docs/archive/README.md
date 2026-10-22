# 文档归档

这里保存仍可追溯、但不再是当前操作或规范依据的材料：已关闭 review、被替代的设计计划、
历史状态记录和外部参考资料。

归档规则：

- 保留原日期和文件名；
- 当前文档只链接到归档结论，不复制完整历史；
- 未跟踪的本地 review 和 PDF 也可迁入对应子目录，但迁移前先确认其是否应提交；
- 归档不等于删除。删除只适用于有明确可再生来源、且不再被证据或文档引用的文件。

## 当前归档索引

- [STATUS_2026-08-27.md](status-history/STATUS_2026-08-27.md)：此前的完整实施过程、review
  闭环和历史待办；当前状态以 [`../STATUS.md`](../STATUS.md) 为准。
- [PLAN_2026-08-18_time_control.md](plans/PLAN_2026-08-18_time_control.md)：基于时间点的
  轨迹控制实施记录；当前控制入口与参数以 [`../INTERFACES.md`](../INTERFACES.md) 为准。
- [MOSAIC_PLAN_2026-08-27.md](plans/MOSAIC_PLAN_2026-08-27.md)：P2.1～P2.7e 的完整设计、
  阶段流水和当时的基线分析；当前设计与验收边界以 [`../MOSAIC_PLAN.md`](../MOSAIC_PLAN.md) 为准。
- [PROJECT_GUIDE_2026-08-27.md](specifications/PROJECT_GUIDE_2026-08-27.md)：此前完整项目规范；
  当前目标与硬约束以 [`../../PROJECT_GUIDE.md`](../../PROJECT_GUIDE.md) 为准。
- [INTERFACES_2026-08-27.md](interfaces/INTERFACES_2026-08-27.md)：此前完整参数与接口字典；
  当前合同以 [`../INTERFACES.md`](../INTERFACES.md) 为准。
- [OPERATION_2026-08-27.md](operations/OPERATION_2026-08-27.md)：此前完整操作与实验记录；
  当前流程以 [`../OPERATION.md`](../OPERATION.md) 为准。

`reviews/REVIEW_*.md` 由 `.gitignore` 忽略，是尚未纳入 Git 的本地 review 工作稿。它们不属于
当前文档体系；如需正式保留，应先逐份确认内容与提交必要性，再解除忽略。

## 新增主题分册（2026-10-22）

- [M1 历史设计](suction/README.md)、[M2 阶段记录](m2/README.md)：原长计划按设计、阶段与后续试验拆分。
- [CUDA 原合同](gpu/DESIGN_2026-09-01.md)、[否决路线与结果](gpu/RESULTS_2026-09-01.md)。
- [墙面原计划](plans/WALL_TEXTURE_PLAN_2026-10-19.md)、[旧协议 v1](protocols/WALL_TEXTURE_PROTOCOL_V1_2026-10-18.md)、[旧协议 v2](protocols/WALL_TEXTURE_PROTOCOL_V2_2026-10-18.md)。
- [10-22 状态快照](status-history/STATUS_2026-10-22.md)：整理前混合记录，当前状态以现行入口为准。
- [历次数据保留与清理](data-retention/README.md)：保留授权、执行结果与回执路径。

旧的超长归档文件名保留为分册目录页；从目录页进入对应章节，不丢弃历史内容。

- [本地参考原件](references/README.md)：早期需求 DOCX 与参考专利 PDF，原件不随 Git 分发。
