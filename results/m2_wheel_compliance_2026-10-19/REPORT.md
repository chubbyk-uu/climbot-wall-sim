# M2 T1：名义柔顺轮标定与状态链

2026-10-19。**首版 T1 通过**：单轮实际压缩量标定、独立接触测量、加载／卸载、
整车平墙短动作、真实 ROS joint_states→动态 TF、reset 和正常退出。
开发期默认仍为刚性轮；起伏墙、T2 耦合载荷／漏气、扩大载荷包络及 T3 双 GUI 性能尚未验收。

## 固定协议与参考

复用本机私有参考 `Subway_scan_bot_sim` 的 `ssb_tools/wheel_stiffness.py` 和
`docs/design/MECHANICS.md` 的经验：DART 1 ms 下实际挠度不等于 N/k，应标定实际关节位移。
未改参考项目、系统 ODE 或 Mesa。

运行前记录的协议见 [protocol.json](protocol.json)：

- 真实轮半径 47.5 mm、宽 47 mm、总质量 0.6 kg，来自共享机械模型。
  从其中拆出 0.02 kg 轴座，0.58 kg 轮体；同心拆分保持总质量及零位装配惯量。
- 法向自由的 7.1592405 kg 代表车体、弹簧移动轴座和自由旋转圆柱轮；墙面竖直，法向为 X。
  零重力仅隔离法向标定，不把重力切向载荷混入本试验；位移没有速度或位置强制命令。
- 法向载荷循环：0→50→150→300→150→0 N，100 ms 斜坡，每次模拟 4.6 s。
  接触力来自原生碰撞 wrench，独立于弹簧位移；没有用 kδ 反推“实测轮压”。
- 目标 150 N 下实际压缩 1 mm，容差 ±5%；稳态波动≤0.01 mm，卸载残余≤0.02 mm，
  独立接触载荷误差≤1 N，加载／卸载 150 N 重复差≤0.02 mm，线性偏差≤10%。
- 行程 0～4 mm，无负行程拉伸；最终候选不得触及上限。阻尼计算采用名义 ζ=0.3、
  上述代表车体质量；步长固定 1 ms，不降低拍照精度。

## 标定过程与结果

保留全部四个候选，没有改最终门槛，也没有重跑已完成候选：

| 候选 | 刚度 N/m | 150 N 实际压缩 | 结果 |
| --- | ---: | ---: | --- |
| 名义 N/δ 初值 | 150000 | 2.286285 mm | 过软，300 N 时碰上限 |
| 比例校正 | 342942.719 | 1.372743 mm | 仍过软 |
| 倒刚度割线校正 | 721726.300 | 0.930266 mm | 比目标小约 7%，未通过 |
| 离散响应预测＋确认 | **622067.9745** | **1.000000 mm** | **全部门槛通过** |

前面三个点的实际响应都与 `δ=F/k·(1+c·dt/m_unsprung)+F·dt²/m_unsprung` 相符，
差值不超过 2.8e−11 m。离线反解后只补一个确认点。复现工具已采用该关系预测，
相同条件新跑通常只需名义初值和确认两次；不把预测公式本身算成通过证据。
最终原生模拟测量如下：

| 窗口 | 实际压缩 | 独立接触轮压 |
| --- | ---: | ---: |
| 加载 50 N | 0.333333 mm | 50.000000 N |
| 加载 150 N | 1.000000 mm | 150.000000 N |
| 加载 300 N | 2.000000 mm | 300.000000 N |
| 回落 150 N | 1.000000 mm | 150.000000 N |
| 卸载 0 N | 约 0 mm | 0 N |

最终阻尼 **1266.2039235 N·s/m**，全循环最大压缩 **2.046460 mm**，未碰上限。
写入 `config/m2_wheel_compliance.yaml` 的 622 kN/m 是该步长和表示下的求解器输入，
不应解释成真机胎面的实测材料刚度；名义实际等效约为 150 kN/m。
隔离试验核对了实际加载的私有 hashfix ODE 路径，采用与发行版一致的 CCD 配置和私有 Mesa 环境。

## 整车接口和 ROS／TF 检查

`build_assets(..., wheel_compliance=True)` 及压力专项入口的
`wheel_compliance:=true` 显式启用；默认 false 直接生成原刚性链，不残留自由移动关节。
整车总质量仍为 15.518481 kg，零位装配惯量和外观保持；新增轴座质量从各自已有轮体中拆分。
原 DiffDrive、WheelSlip、控制限幅和相机配置没有改变。

状态链为 `base_link→{left,right}_wheel_axle_link→{left,right}_wheel_link`，
移动关节名为 `{left,right}_tyre_joint`，旋转关节仍为原名。Gazebo 的 JointStatePublisher
显式发布四个运动关节，经现有 bridge 到 `/joint_states`，由 robot_state_publisher 产生动态 TF。
URDF 与 SDF 的弹簧轴均沿车体法向；URDF 只负责状态／TF，物理弹簧在 SDF 中执行。

真实无 GUI ROS 检查只跑一次：同一短任务含 0.2 m/s 直行、0.35 rad/s 原地转向、停车和 reset。
**18 项检查通过**：

- 运动时左右各 127 个 TF 与源关节状态同时间戳配对，reset 后左右各 37 个；位移误差为 0。
- 两轮实际压缩范围约 0.196～1.748 mm，未被静态零值掩盖；存在实际平移和转动。
- 没有吸附运行错误或裙边止挡反力，reset 后恢复。
- launch 退出码 0，SIGINT 正常收尾，进程组已清理，没有强杀。

ROS 状态检查走压力入口的原有运行库选择，没有另采库映射，不把它列作私有 ODE 加载证据。
该次只保存紧凑检查摘要、配对数和误差，未归档逐条 ROS 消息；可用下列入口复现。
这不是 T2 的逐轮摩擦／最低压差或 T3 的 GUI、采集性能验收，也不代表旧 WheelSlip／EKF 标定已适用于起伏墙。

## 验证与复现

受影响生成器 **28 项测试通过**，包含默认刚性结构、开关组合、质量／惯量、移动关节父子关系、
发布名称和 TF 链；包内 flake8、38 项 uncrustify 及两个新增工具的 ament_flake8 通过。
构建通过，未重跑 S6 或全套 ROS 回归。详细小摘要见 [summary.json](summary.json)、[verification.json](verification.json)。

```bash
source install/setup.bash
python3 tools/calibrate_m2_wheel_compliance.py \
  --ode-prefix "$HOME/opt/climbot-ode-0.16.2-hashfix-ccd/install" \
  --output "${CLIMBOT_DATA_ROOT:-$HOME/climbot_data}/m2-tyre-$(date +%Y%m%d-%H%M%S)"
ROS_DOMAIN_ID=97 python3 tools/validate_m2_wheel_state.py \
  --output "${CLIMBOT_DATA_ROOT:-$HOME/climbot_data}/m2-tyre-state-$(date +%Y%m%d-%H%M%S)"
```

输出须为新目录；标定 `--resume` 仅在协议、原生二进制及 ODE 哈希一致时复用失败候选。
原始 SDF、请求、接触／位移 CSV、测量摘要和源码快照留在本机数据根
`m2_wheel_compliance_20261019/`，CSV 已无损 gzip 并删除未压缩副本；没有采集照片。
Git 仅保存报告／协议／小摘要。来源为脏工作树，摘要保留相应标记及源码／二进制哈希，
测试时源码快照留在本地；后续只有工具格式整理和将 TF 尺寸从字面量改为共享配置读取。

下一步接入同源起伏墙及球轮代理面，再做 T2 的耦合轮载／裙边／漏气检查。
如果耦合预测超出本次 0～300 N 采样范围，只补必要上界点，不展开敏感性矩阵。
