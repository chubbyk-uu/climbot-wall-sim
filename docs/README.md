# 文档导航与维护边界

更新：2026-10-22。当前说明与历史过程分开维护；历史报告的数字、门槛和 producer provenance
不因当前默认改变而改写。文档整理记录见[本轮盘点](DOCUMENTATION_AUDIT_2026-10-22.md)。

## 按任务查找

| 需要做什么 | 入口 |
| --- | --- |
| 了解项目与默认行为 | [根 README](../README.md)、[当前状态](STATUS.md) |
| 安装、构建、生成资产、测试 | [安装](INSTALLATION.md)，WSL 图形库详见[Mesa](MESA_SETUP.md) |
| 首次跑通仿真、采集与拼接 | [快速启动](QUICKSTART.md) |
| 变更参数、批处理或排查故障 | [操作入口](OPERATION.md)、[故障索引](INCIDENTS.md) |
| 产品目标与硬约束 | [项目规范](../PROJECT_GUIDE.md) |
| 包职责、数据流与配置归属 | [系统架构](ARCHITECTURE.md) |
| ROS 话题、服务、Action 与文件格式 | [接口合同](INTERFACES.md)及各包 README |
| 验收状态、指标与证据 | [验收矩阵](ACCEPTANCE.md)、[结果入口](../results/README.md) |
| 负压、起伏与轮胎模型 | [M1](SUCTION_PLAN.md)、[M2](SUCTION_M2_PLAN.md) |
| 离线拼接、CUDA 与墙面资产 | [拼接](MOSAIC_PLAN.md)、[CUDA](GPU_ACCELERATION_PLAN.md)、[墙面资产](WALL_TEXTURE_PLAN.md) |
| 数据能否清理、已删什么 | [当前保留规则](DATA_RETENTION.md)、[历史回执索引](archive/data-retention/README.md) |
| 查旧设计、失败路线和阶段流水 | [归档入口](archive/README.md)，不以归档默认替代当前配置 |

## 文件职责

- 根 README：项目概览与导航，不重复安装和完整运行命令。
- INSTALLATION／QUICKSTART：分别负责准备环境与首次跑通；OPERATION 只写额外实验／排查。
- PROJECT_GUIDE：目标和硬约束；INTERFACES：接口语义；ARCHITECTURE：依赖与配置归属。
- STATUS：当前状态和停止边界，不按日期堆叠日志；ACCEPTANCE：逐项状态与证据。
- M1／M2、拼接、CUDA、墙面入口：现行模型／合同及证据入口；已完成计划放 archive。
- 包 README：包内职责、参数与 API，不再复制整套主线流程。
- results：正式批次报告／小型摘要。当前入口与分阶段历史索引分开，报告不因默认变化重写结论。
- DATA_RETENTION：现行保留与最新清理结果；每次授权和旧目录状态放清理历史。

## 分册结构

| 目录 | 职责 |
| --- | --- |
| `architecture/` | 包职责与运行数据流 |
| `acceptance/` | 控制／定位与成像／归档验收分册 |
| `operations/` | 采集、图像链与 M2 物理专项命令 |
| `m2/`、`suction/` | 当前模型与验收边界 |
| `incidents/` | 保留案例编号的故障分册 |
| `archive/` | 按主题／日期组织的阶段历史、旧协议与清理回执索引 |

## 更新规则

1. 同一规范性事实只维护一份；状态页和 README 用简短结论链接它，不复制大表或完整过程。
2. 当前配置与冻结试验分清；“默认关闭／待验收”若来自历史报告，须标明记录时点。
3. 归档／拆分保留内容与日期，同时修复相对链接和章节锚点。长归档继续拆分，旧文件名保留索引。
4. 代码或 YAML 是运行参数的来源；文档解释作用与边界，不创建第二份长期配置。
5. 清理数据只按用户明确授权的精确范围，保留哈希、保留项核对和回执；规则本身不授权删除。
6. 本地 `archive/reviews/REVIEW_*.md` 为忽略的工作稿，不在公开文档分册迁移范围。
