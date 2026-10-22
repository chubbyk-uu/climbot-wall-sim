# 2026-10-22 文档盘点与整理

本轮范围：整理前 Git 跟踪的 114 份 Markdown、包级说明，以及本地参考原件的存放位置。
构建目录、测试夹具文本和忽略的 review 工作稿不作为现行项目文档改写。
完整逐文件分类、行数与 SHA256 清单留在数据根 `documentation_inventory_20261022.json`。

## 当前入口

| 内容 | 整理后的唯一入口 |
| --- | --- |
| 安装、依赖、资产和测试 | [INSTALLATION](INSTALLATION.md) |
| 首次运行全链路 | [QUICKSTART](QUICKSTART.md) |
| 当前默认与停止边界 | [STATUS](STATUS.md) |
| 日常实验和排障 | [OPERATION](OPERATION.md)，专项放 `operations/` |
| 规范、架构和接口 | [项目规范](../PROJECT_GUIDE.md)、[架构](ARCHITECTURE.md)、[接口](INTERFACES.md) |
| 验收与报告 | [验收矩阵](ACCEPTANCE.md)、[结果入口](../results/README.md)；详细旧批次放[分阶段索引](../results/HISTORY.md) |
| M1／M2 | [M1](SUCTION_PLAN.md)、[M2](SUCTION_M2_PLAN.md)，模型与合同在 `suction/`、`m2/` |
| 故障经验 | [55 个案例索引](INCIDENTS.md)，按机制分为六册 |
| 数据保留 | [现行清单](DATA_RETENTION.md)，历次授权／回执在[历史分册](archive/data-retention/README.md) |

## 篇幅与归档

| 入口 | 整理前行数 | 整理后行数 |
| --- | ---: | ---: |
| 根 README | 558 | 81 |
| STATUS | 221 | 58 |
| M1 计划 | 679 | 38 |
| M2 计划 | 949 | 64 |
| INCIDENTS | 805 | 76 |
| DATA_RETENTION | 336 | 66 |
| ARCHITECTURE | 334 | 55 |
| ACCEPTANCE | 288 | 85 |

- M1／M2 开发设计、门槛修订、失败方案和阶段流水移入主题历史分册。
- 已完成 CUDA 计划和否决路线归档，当前页仅保留现用后端合同。
- 墙面旧计划和 v1／v2 协议归档，当前协议保留；旧失败不按新口径追认通过。
- 七份原有超长归档进一步拆分，旧文件名保留分册索引；按顺序拼回后原文一致，仅链接地址变化。
- 本地需求 DOCX 和专利 PDF 移至[参考原件目录](archive/references/README.md)，SHA256 不变，仍不进 Git。
- S0 与 S1～S6 方法保留原路径，因为工具将其计入溯源；页首标明冻结时点，避免当作现行默认。

整理后公共 Markdown 各篇不超过 300 行；行数用于浏览，不作为必须拆碎短文的固定规则。

## 过时与重复内容

更新了规范、验收矩阵和包 README 中“M2 未开始”“平墙刚性轮默认”“巡航 0.20 m/s”等旧说明。
现行文档统一指向 M2、μ=0.8、已标定 EKF、尺度候选不切默认以及本轮算法停止决定。

状态页不再重复各轮大表和试验流水；安装页不重复纹理性能结论；完整主线命令从根 README
移到安装／快速启动；包文档和操作页通过链接复用它。旧结果行移到分阶段索引，保留原结论。
数据页按两批清理后的实际状态维护，不再同时把已删派生图写作仍完整保留。

## 验证与边界

- 全部公共 Markdown 的本地链接与章节锚点检查，代码围栏闭合检查通过。
- 55 个故障编号保留；七份大归档的合并一致性检查通过。
- 所有正式 `REPORT.md` 正文与数值保持，仅修复迁移后的链接地址；本轮没有修改任何结果 JSON。
- 完整 `colcon test --executor parallel --parallel-workers 8` 与结果检查均退出 0：
  **1996 tests，0 errors，0 failures，91 skipped**。
- 本次日志：`climbot_docs_full_j8_20261022_194103.log`，摘要同名 `_summary.log`，位于 `/tmp`。

本轮不重新验收图像、摩擦或频闪。此前未提交的整墙默认 GUI 视角改动保留；本轮只整理文档与
本地参考资料的忽略规则，没有修改模型、EKF、物理或拼接算法。
