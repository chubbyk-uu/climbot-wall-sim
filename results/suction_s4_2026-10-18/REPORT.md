# S4 压力测量、三风机与故障入口验收

日期：2026-10-18。实现及实验源版本 **`c2ef15c`**，开始正式原生实验时 tracked worktree
干净；源码／配置及实际二进制 SHA-256 见 [source_manifest.json](source_manifest.json)。
本批只关闭 S4 的观测和故障入口，不关闭 S5 的真实行走、漏气／失压边界或 S6 全链路任务。
S0～S3 原始证据保持不变。

## 实现与冻结接口

实验入口仍为 `climbot_vacuum.launch.py`。压力物理和启动支撑继续用原物理真值，单一施力
路径不变；新增测量模型没有向腔压或力反馈。三台风机保持固定档位，分别控制启用和驱动失效，
实际目标为 `enabled && !failed`，转速按原 0.15 s 一阶模型变化，原端口倒灌模型保留。
未加入止回阀、连续调速、压力闭环或运行低压运动门禁。

正式 topic、类型、QoS、时间戳、量程与故障／reset 语义见
[INTERFACES](../../docs/INTERFACES.md#吸附与压力接口)。主要接口：

- `/suction/pressure`：FluidPressure，腔内**绝压 Pa**，header 为采样时刻；
- `/suction/ambient_pressure`：带时间戳的外界绝压，当前固定 101325 Pa；
- `/suction/fans/fan_{0,1,2}/{enabled,failed}`：Bool 命令，下一非暂停步生效；
- `/suction/sensor/fault`：Int32，0 正常、1 丢报、2 卡值，非法值拒绝并计数；
- `/suction/ground_truth/state`：仅评价用严格 JSON，含仿真时间、代次、独立物理状态、风机
  命令／实际转速、命令版本及最近测量的采样与送达时刻。未供给定位、规划、控制或拼接。

默认测量参数来自 `vacuum_io.yaml`：100 Hz、偏置 +25 Pa、噪声标准差 10 Pa、延迟 20 ms、
量程 20000～120000 Pa、种子 4817。首个非暂停物理步为 1 ms，首个实际采样为 10 ms，
首次送达为 30 ms。方差字段为截断前噪声方差，不包括偏置或饱和误差；卡值 variance=0
仅描述恒定读数，不能解读为真实压力无误差。所有参数均为 **provisional 仿真假设**。

## 实验设计与原始证据

九组均为平墙、刚性轮、无相机渲染，使用真实 Gazebo 接触和 S3 独立观测器；每组共推进
4 s，默认物理步 1 ms，half_step 为 0.5 ms，合计 40000 个非暂停物理样本。每组保留完整
配置、生成 SDF／URDF、resolved、请求、原始逐步 frames 和 contacts（gzip，无降采样），
及生命周期、实际动态库与测量统计记录。请求中的 world 路径相对本组目录。

| 组 | 改变的输入 | 送达测量数 | 检验目的 |
| --- | --- | ---: | --- |
| normal | 默认 C、三机启用 | 398 | 正常统计、时间戳与延迟 |
| repeat | 完全同配置重跑 | 398 | 固定种子与物理重复性 |
| zero_error | noise=0、bias=0 | 398 | 与采样时刻真实腔压逐点相等 |
| sensor_faults | 0.3 s 丢报、0.6 s 恢复、0.9 s 卡值、1.2 s 恢复 | 362 | 故障、队列清理与物理隔离 |
| fans | 依次使风机 0 失效、1 关闭、2 失效，各 150 ms 后恢复 | 398 | 各风机独立降速及腔压响应 |
| fixed_fans | 同一风机序列、B 理想固定压力 | 398 | B 保持理想压差这一诊断合同 |
| lifecycle | 同风机序列；1.35 s 丢报、1.7 s 恢复；0.3 s 暂停、1.5 s reset | 343 | 带活动故障 reset、随机源／队列清理和事件重放 |
| saturation | 量程收窄到 95000～100000 Pa | 398 | 截断与饱和标记 |
| half_step | 0.5 ms 物理步，其他同 normal | 398 | 采样网格、延迟与随机误差不随物理步改变 |

lifecycle 的 4 s 为 reset 前 1.5 s 加 reset 后 2.5 s，暂停迭代另记；不是 reset 后仍只
观察到 4 s 的同一时间轴。暂停的首次后端姿态缓存刷新按 S3 规则单列，之后不再推进。

## 定量结果

[summary.json](summary.json) 中 **63 项原生检查全部通过**：

- 默认 398 个送达样本的误差均值 **25.741059 Pa**，方差 **109.167322 Pa²**；相对配置
  +25 Pa／100 Pa²，满足冻结的均值 ±2 Pa、方差 ±15% 软件统计检查。它是有限样本的
  合成噪声验证，不是实机传感器标定或独立硬件留出验证；核心回归另检查 10000 个噪声样本。
- 零误差组与对应采样时刻的实际腔压逐点一致，误差均值与方差均为 **0**。
- 全部样本落在 10 ms 网格；送达减采样时刻均为 20 ms，浮点最大残差
  **4.27×10⁻¹⁶ s**。这是离散时钟验证精度，不是物理传感器精度。
- repeat 的测量逐样本相同。repeat、zero_error、sensor_faults、saturation 四组与 normal
  的机械／腔压记录逐项完全相同；仅排除 CPU 计时、传感器字段和命令版本。
- 丢报切换清除尚未送达队列；恢复后重新等待延迟。卡值期间读数保持常数，采样时间持续递增。
  含卡值／饱和的全段误差统计不用于反推正常噪声方差。
- 三机分别降速、其他风机继续启动，恢复后末端归一化转速均 >0.999；C 腔压与正常组的最大
  差异 >100 Pa，证明故障入口实际改变物理求解。B 仍为固定压差，不能用 B 检验失压边界。
- pause 不变、reset 恢复支撑／环境腔压／零转速，清除活动传感器故障与队列；随机误差序列
  重放一致。reset 前已执行 7 条事件、reset 后执行 8 条，均与各自观察时长一致。
- 0.5 ms 与 1 ms 的实际采样时刻和逐样本随机误差一致；本批不重命名为压力动力学收敛验收。

真实腔压、延迟测量、三机独立降速及传感器故障（本地附件：`results/suction_s4_2026-10-18/sensor_and_fans.png`）

## 真实 ROS bridge 验证

另从标准 launch 分别启动 B/C，在线发布全部六个风机输入和三种传感器模式，拒绝非法模式 9。
不是替换订阅回调或直接设置 ECM。两组使用零噪声／零偏置覆盖配置，以独立真值在采样时刻
核对经桥接的 header 与测量值；C 的瞬变压力使时间错位不能仅靠相等常数掩盖。

| 模式 | 真值记录 | 压力消息 | 环境压力消息 | 检查 |
| --- | ---: | ---: | ---: | --- |
| B | 107 | 77 | 107 | 31/31 |
| C | 107 | 76 | 106 | 31/31 |

消息数差异来自 volatile 发现／观测终点及异步回调，不作全历史无丢失承诺；原始到达记录
`bridge_{B,C}/ros.jsonl` 和命令确认时刻均保留。验证可靠 QoS、采样时间与值、20 ms 延迟、
ROS 丢报／卡值、非法命令拒绝、风机确认与实际降速。两次 launch 均退出 0，进程组清理通过。

九组原生 Gazebo 进程均记录实际加载映射，确认私有 Mesa 25.2.8 的四个库完整映射、无系统
Gallium 混入；Gallium SHA-256 为
`f2c80fbfb37a73aab1e4b94cf95bc862f271cd30055573c6196b9073cc4d6d66`。
bridge launch 使用相同私有 Mesa 入口，本批不额外声称已保存其进程映射或 GUI／相机验收。

## 开发中暴露的问题与最终回归

1. 初版假定首个采样一定在 t=0，实际 Gazebo 首步为 dt，原生 B/C 因网格不一致明确失败。
   改为首个非暂停更新之后的绝对网格采样，未伪造 t=0 数据；原有 S3 启动／reset 测试保留。
2. 初拟 `/fans/0/` 不符合 ROS topic token 不能以数字开头的规则，统一改为 `/fans/fan_0/`；
   正式 B/C 真实桥接覆盖全部最终路径。
3. 新增代码格式／导入检查曾失败，按仓库格式修正后全套通过，未绕过检查。
4. 新桥接测试最初将 ROS_DOMAIN_ID 配为 APPEND_ENV，继承空值时形成 `:96`，ROS 初始化失败；
   继承 17 时形成 `17:96`，本机 RCL 实际取成 17，导致隔离失效而非必然报错。
   改为 ENV 覆盖 96，增加仓库契约检查。[domain_environment.json](domain_environment.json)
   保留实际 ament wrapper 的上述两种负对照和正对照。首次失败临时日志未充分保留，补强后
   完整复现了同一初始化失败；不依靠重跑绿色解释。详见 INCIDENTS 第 22 项。

冻结前完整回归显式带继承 `ROS_DOMAIN_ID=17`，使用
`colcon test --executor parallel --parallel-workers 8`：**1590 tests，0 errors，0 failures，
79 skipped**。跳过均来自 cppcheck 性能检查，功能／原生／桥接测试未跳过；见
[regression.json](regression.json)。完整日志留在临时目录，正式记录只保存 basename；
最终日志 `climbot_s4_domain_full_20261018.log`，结果日志
`climbot_s4_domain_full_result_20261018.log`。8 个修改／新增 Python 文件单独 lint 通过。

## 复现与边界

在源版本 c2ef15c 构建并加载 ROS／工作区后，输出目录必须不存在：

```bash
python3 tools/validate_vacuum_io.py --output results/新的S4目录
ROS_DOMAIN_ID=223 python3 tools/validate_vacuum_bridge.py --mode fixed_pressure --output results/新的B目录
ROS_DOMAIN_ID=223 python3 tools/validate_vacuum_bridge.py --mode fan_ode --output results/新的C目录
```

日常接口实验可运行 `ros2 launch climbot_gazebo climbot_vacuum.launch.py headless:=true`，
通过 `io_config:=<YAML>` 使用同源事件表。传感器参数和风机／裙边曲线仍待实测，未改为真机结论。
风机故障安排在启动阶段，本批不验证运行时安全保持、深沟槽／最不利漏气、倒灌参数失效边界，
也不承诺停一台就必然失压；这些必须在 S5 结合实际运动和独立开口扫描验收。


本页以反引号标注的本地验收附件不随 Git 发布，见[发布范围说明](../PUBLICATION.md)。
