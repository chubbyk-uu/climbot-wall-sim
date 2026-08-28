# 文档导航与治理

本目录只保留当前有效的工程说明、接口合同、操作手册和验收结论。历史设计讨论、review
记录、过期运行日志和参考资料应保留可追溯性，但不应干扰日常查找。

`README.md`（仓库根目录）是所有使用者的入口。它必须能够独立回答：运行环境是什么、如何
安装和构建，以及怎样从零跑完仿真、规划、控制、采集、图像预处理和墙面拼接这一整条主线。
它不承载批量回归、参数变体和历史决策。

README 和 `OPERATION.md` **按读者分工，不按详略分工**：README 面向第一次跑通的人，
OPERATION 面向已经跑通、要做实验或者出了问题的人。判据只有一条——**同一条命令只出现在
一处**：它属不属于“第一次跑通”这条主线，就决定它写在哪边。所以 README 里的步骤可以写得
比 OPERATION 详细，这不是倒置。

## 当前文档的职责

下表按“唯一归属”列出，`README.md`、`PROJECT_GUIDE.md` 和 `results/README.md`
不在 `docs/` 内，但同样受这套边界约束。

| 文档 | 只应包含 | 不应包含 |
| --- | --- | --- |
| [`../README.md`](../README.md) | 环境需求、安装部署、构建测试、**一条跑通全链路的主线**、文档入口 | 批量回归、参数变体、实验过程、历史决策 |
| [`PROJECT_GUIDE.md`](../PROJECT_GUIDE.md) | 产品目标、范围、不可违背的设计约束、规范性验收要求 | 具体命令、已完成的开发日志 |
| [`ARCHITECTURE.md`](ARCHITECTURE.md) | 包职责、依赖、配置归属、运行时数据流 | 命令教程、接口字段的逐项字典 |
| [`INTERFACES.md`](INTERFACES.md) | 当前话题、服务、Action、参数、文件格式与兼容性合同 | 已废弃接口的演变过程 |
| [`OPERATION.md`](OPERATION.md) | **主线之外**的批量回归、评价工具、参数变体、诊断与故障处置 | 主线命令的第二份拷贝、架构解释、结果结论 |
| [`ACCEPTANCE.md`](ACCEPTANCE.md) | 当前验收项、状态、正式证据和未关闭门禁 | 完整实验流水账 |
| [`MOSAIC_PLAN.md`](MOSAIC_PLAN.md) | 离线拼接的当前设计、风险、阶段门禁和待办 | 已废弃试验的逐轮细节 |
| [`LOCALIZATION_GAP_PLAN.md`](LOCALIZATION_GAP_PLAN.md) | 定位链路偶发断档的当前证据、根因调查、优化顺序与验收标准 | 未证实的根因结论、通用操作教程 |
| [`STATUS.md`](STATUS.md) | 一页式当前阶段、风险、近期下一步 | 按日期追加的开发日记 |
| [`../results/README.md`](../results/README.md) | 当前有效基线和归档结果的索引、重生成入口 | 长篇技术分析 |
| [`DATA_RETENTION.md`](DATA_RETENTION.md) | 工作区外大数据的保留等级、清理前置条件和目录清单 | 删除命令或临时日志 |

## 当前结构

```text
docs/
  README.md                本页：文档导航与写作边界
  ARCHITECTURE.md          当前架构
  INTERFACES.md            当前接口合同
  OPERATION.md             实验与故障处置手册
  ACCEPTANCE.md            当前验收矩阵
  MOSAIC_PLAN.md           当前离线拼接设计
  LOCALIZATION_GAP_PLAN.md 定位断档调查与优化计划
  STATUS.md                当前状态快照
  DATA_RETENTION.md        数据保留与清理清单
  images/                  当前文档引用的截图
  archive/
    README.md              归档索引与迁移规则
    interfaces/            已替代的详细接口说明
    operations/            已替代的操作与实验流水
    plans/                 已完成或被替代的设计计划
    reviews/               已提交的 review（当前为空，见归档规则）
    specifications/        已替代的完整规范
    status-history/        状态与开发过程记录
```

`docs/REVIEW_*.md` 是未纳入版本控制的本地 review 工作稿，按 `.gitignore` 忽略；
它们停留在 `docs/` 根目录，只有逐份确认需要正式保留时才迁入 `archive/reviews/`。
新增归档子目录前先确认确有内容要放，空目录不进仓库。

`results/` 分为“当前正式基线”和“历史对照”两层；文件本身可以保留，根目录索引只指向
当前仍有效的证据和归档入口。

## 迁移规则

1. 当前合同优先于历史叙事；同一事实只能有一个规范性来源。
2. 历史文档迁移到 `archive/` 时保留日期和原文件名，并在当前文档留下简短链接。
   迁移会改变相对深度，必须同时把文中的相对链接改到新位置——归档件失效的链接
   和当前文档失效的链接一样算错误。
3. 结果摘要、输入哈希和提交号不改写；只调整索引和目录位置。
4. 新功能先更新其唯一归属文档，再更新根 README 的快速入口；不要把同一段说明复制到多处。
5. 数据清理必须先更新 `DATA_RETENTION.md`，经确认后才执行删除。
