> 历史记录，按记录时点解释；当前默认与结论见 [当前状态](../../STATUS.md)。

# M1 历史：IMPLEMENTATION

## 5. 软件与接口安排

| 归属 | 计划修改 |
| --- | --- |
| `climbot_description` | 共享几何、质量／惯量、内置球轮、裙边轮廓、测量 frame 和一致的 URDF |
| `climbot_gazebo` | 平墙、吸附参数、无 ROS 依赖的 C++ 物理核心、原生 System、SDF 与测试 |
| Gazebo launch | 选择几何／吸附模式，传递配置、加载插件、压力与关节状态桥接；沿用现有 bridge 组织方式 |
| `climbot_bringup` | 透传启动参数，保证两套覆盖任务入口配置一致 |
| `climbot_interfaces` | 仅在现有标准消息不足时增加有明确单位和风机身份的状态／命令合同 |

下列记录开发期与最终配置合同。S7 已实现最终默认；保留 robot_geometry 名称作兼容输入，
但只允许 vacuum_chassis，已无新旧切换能力。旧组合仅用于理解冻结历史，不是当前有效接口。

| 参数 | 候选值 | 开发期默认（S0–S6） | S7 后默认 | 用途 |
| --- | --- | --- | --- | --- |
| `robot_geometry` | `legacy`／`vacuum_chassis` | `legacy` | 仅接受 vacuum_chassis，无切换能力 | 开发期分离旧结构基线与新底盘 |
| `suction_model` | `legacy_constant`／`fixed_pressure`／`vacuum` | `legacy_constant` | `vacuum`，删除 `legacy_constant` | A/B0／B／C；最终保留 B 作诊断 |
| `wall_surface` | `flat`／`undulating` | `flat` | `flat` | 起伏属于 M2，默认关闭 |
| `wheel_compliance` | `false`／`true` | `false` | `false` | 法向柔顺属于 M2，默认关闭 |

M1 的 B/C 保持平墙＋刚性轮。M2 计划接口见独立文档，不属于当前已实现接口。
无效组合、缺失资产或参数应明确报错，不自动替换机械配置。
`legacy_constant` 只用于 A/B0；B/C 的后两种模式由同一插件独占压力及裙边施力，必须移除旧
persistent wrench。`vacuum` 还显式选择准静态或容积动态求解并记录 provenance。
插件缺失、参数错误或数值异常时明确失败，不能静默回退或偷偷切换求解器。

冻结期新旧底盘并存，`robot_geometry` 必须让所有读取方选到同一份描述变体：`legacy` 继续使用
现有 `robot.yaml` 且不修改，新底盘放入独立变体文件。当前直接读取 `robot.yaml` 的位置包括
URDF、Gazebo SDF、`climbot_wall.launch.py`、`coverage_planner.launch.py`、`line_tracker.launch.py`、
`evaluate_coverage_execution.py` 和 `evaluate_g1_camera.py`；全部改为按所选变体解析，两套覆盖
任务入口统一透传。增加契约测试，核对规划足迹、控制限幅、URDF/SDF 几何和评价器解析出的是
同一变体，任何一处仍读另一变体即失败。合法组合只有：`legacy`＋`legacy_constant`（A）；
`vacuum_chassis`＋`legacy_constant`（仅 B0）／`fixed_pressure`（B）／`vacuum`（C）；其余组合
启动即拒绝。

旧底盘生命周期：

1. **冻结期（S0–S6）**：`legacy` 的描述、SDF、参数和恒力配置不再修改，只作默认回归基线和
   A 组来源。`robot_geometry` 开关只为分支开发期新旧并存而设，不作为长期双模型合同。
2. **切换与删除（S7）**：S0–S6 的新底盘 C 配置通过完整回归并冻结提交与参数后，删除
   `legacy` 几何、A/B0 使用的 `legacy_constant`、`ApplyLinkWrench` 配置及其测试分支。
   新底盘成为唯一几何，`vacuum` 成为默认吸附模式；保留同插件 `fixed_pressure` 作 B 组诊断，
   平墙和刚性轮仍为默认。移除 `robot_geometry` 的旧结构分支，仅保留单值兼容参数；新结构标量
   更新到唯一的 `robot.yaml`，详细几何仍从 vacuum_chassis.yaml 同源派生；
   变体契约测试收回为单一描述的一致性检查。
   A/B0 通过固定 Git 提交／标签、实验配置、资产哈希和结果记录保持可复现，不长期维护第二套
   活跃几何；删除前后均不改写旧证据。
3. **合并**：复用 S6 已通过的完整验收证据；删除旧底盘后，只验证默认切换／删旧影响的
   契约及一次 3×2 m 中等采集与拼接，通过后合并到 `main`，不重复完整套件或最大横纵两组。
   M2 不是合并前置条件，未完成部分保持开关默认关闭或另行交付。

最终默认启动按“临时安装支撑→从环境压力抽气→压力就绪→解除支撑→正常作业”执行，三个
风机按固定档位启动。S0 按启动姿态冻结就绪压差、持续时间和超时条件，明确压力判定的数据
来源；超时保持支撑并报告未就绪，不能按固定延时无条件撤除。记录支撑实际解除时刻；reset
恢复初始支撑、环境腔压及风机／就绪状态，再走完整启动序列。该顺序仅用于仿真安装与初始化，
不引入风机闭环调速，也不扩展为运行中的低压运动门禁；预充压力仍是单独标记的试验配置。

现有控制八工况、WheelSlip 侧滑标定（约 `0.46 mm/°`）、EKF 及完整套件在冻结期继续以原有
默认配置作回归基线，不能为新模型改宽其门槛。新底盘的这些专项已在 S6 通过，S7 在实际
几何、物理、控制、传感参数保持冻结时直接复用，只补默认切换／删旧相关检查和一次中等采集／拼接；
门限沿用现有合同，确需修改时说明物理原因并单独冻结，不能因换底盘静默放宽；不冻结历史测试
数量。M1 新几何／轮径的标定适用性单列，
M2 柔顺工况另行检查有效轮径、侧滑及里程计／EKF 影响，旧标定不直接继承为新模型证据。

柔顺关节的发布、桥接及动态 TF 合同见 [M2 软件与状态链](../m2/DESIGN_2026-10-19.md#5-软件开关与状态链)。


压力传感器首版建议使用 `/suction/pressure`，类型为 `sensor_msgs/msg/FluidPressure`，单位 Pa，
其 `fluid_pressure` 表示腔内绝对压力。同时提供外界压力 `p_a` 的配置／话题和单位；若运行中
变化须有对应时间戳，使下游可重建 `Δp=p_a-p_c`，不混用绝压和表压。
传感器包含可配置的噪声、偏置和延迟，仿真真值单独放在评价通道。传感器故障不应改变物理腔压，
真实状态也不能因方便而直接交给未来控制器。正式 topic、QoS 和故障注入合同在 S4 写入
[INTERFACES](../../INTERFACES.md) 后冻结。

物理更新使用 Gazebo 仿真时间与物理步长；暂停时停止积分，reset 时重置腔压、风机状态、随机源
和传感器延迟队列。Gazebo 真值只用于物理／传感器仿真和独立评价，不进入定位、规划或拼接算法。
轮速控制仍走现有 `/control/cmd_vel` 与看门狗，压力观测不另建速度指令通道。

