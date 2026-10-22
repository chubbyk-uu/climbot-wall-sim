# M2 摩擦、接触与 ODE 诊断

前置环境见[操作入口](../OPERATION.md#运行前准备)。

## M2 原生接触与局部 ODE 修复

默认起伏墙（2026-10-20 起）的必需依赖：启动时校验构建记录和库摘要，只给 Gazebo 子进程设置库路径；
`wall_surface:=flat` 的 M1 平墙仍用系统库。下列 T0 探针同样使用它。
2026-10-22 起默认轮胎—墙面有效摩擦为 **0.8**，见[复核报告](../../results/m2_friction_2026-10-22/REPORT.md)。
`traction_mu:=0.7`／`traction_mu:=1.1` 是显式试验，空值使用配置；平墙几何也使用当前摩擦配置。
`tools/run_g2_inspection_acceptance.sh` 用 `TRACTION_MU` 传入覆盖值。与既有标定比较时用
`LOCALIZATION_PROFILE=precision`：保留全站仪／IMU 随机噪声，不另注入外参和时戳失标。
`realistic` 另有故障注入，不可将两档误差直接归因于摩擦或 EKF。

新载荷记录自动保存 `recorded_physics_mu`，评测命令为：

```bash
python3 tools/evaluate_m2_load_trace.py --input <loads.bin> --output <新摘要.json>
```

无摩擦元数据的旧记录必须加 `--mu 1.1`（或明确的反事实系数），不能按当前默认猜测历史物理。
`tools/validate_m2_friction.py --mu <系数> --ode-prefix <目录> --output <新目录>` 是额外小试片
边界诊断；不是默认整墙短任务验收，不能把夹具自身在 1.1 下也失败的结果全部归因于降摩擦。

从工作区根目录、已加载 ROS／install 环境执行。源包及补丁均有 SHA256 校验；需要 CMake、
C++ 编译器和 `patch`（缺少时安装 `build-essential cmake patch libccd-dev`）。不替换系统库。

第一次在新机器上构建局部 ODE；已有同路径且构建记录／库摘要匹配时直接复用，不覆盖旧目录：

```bash
python3 tools/build_private_ode.py \
  --prefix "$HOME/opt/climbot-ode-0.16.2-hashfix-ccd" --jobs 4
```

构建配方使用固定的 ODE 0.16.2 源包、哈希回绕补丁、双精度及 Ubuntu 兼容 SONAME，
链接系统 libccd，保留内部断言；按发行版关闭 libccd 的 box-cylinder 路径，其它 CCD 路径开启。
上游 CMake 的目标名和头文件路径问题由配方显式修正，均记入构建清单。旧的 no-libccd
构建仅保留历史诊断，当前加载守卫拒绝它；不要覆盖旧前缀。
完整输出为前缀下 `build.log`。成功时 `build-result.json` 记录源包／补丁／库摘要，
并要求 `hash_regression.json` 的原生两物体／多平移回归通过。
`--variant upstream` 仅供同编译参数的未修复 A/B，必须使用另一新前缀；其回归失败会如实记录，
不能交给修复库加载入口。

```bash
M2_CASE_ROOT="${CLIMBOT_DATA_ROOT:-$HOME/climbot_data}/m2-t0-$(date +%Y%m%d-%H%M%S)"
python3 tools/probe_m2_wall_surface.py \
  --ode-prefix "$HOME/opt/climbot-ode-0.16.2-hashfix-ccd/install" \
  --joint-mesh --output "$M2_CASE_ROOT"
```

此入口将库路径只传给子进程，并用实际进程映射核对加载库；不用全局修改 `LD_LIBRARY_PATH`。
失败保持非零退出码和原始结果。省略 `--ode-prefix` 是显式系统库对照，在受影响 ODE 上可以失败。
`--joint-mesh` 只增加一条有真实深度的水平板缝，不代表已测试斜缝／裂缝、轮胎或密封耦合。
`--boundary-only` 与 `--joint-mesh` 互斥，单独运行平面控制及 129／257 点的水平／垂直
高度场边界检查；同样需要新的 `--output` 目录。每条边在预期边界内外各 0.15 mm 布点，
量规半径 0.05 mm，内侧须持续有支撑、外侧须无接触并符合自由运动参考，约束边界误差
不超过 0.2 mm。SDF 长宽按 N/(N−1) 补偿引擎采样跨度，不能移动源场边界迎合引擎。
`--features-only` 是交汇板缝／真实裂缝三角网格的诊断入口，目前法向检查失败，不得当作
通过证据。它与上述选项互斥；原有量规互撞已修复，任何非墙面接触都会使检查失败。
`--bridged-cracks-only` 显式检查窄裂缝的轮接触近似：完整源配置保留裂缝，另保存
`contact_config.contact_policy` 和 `omitted_cracks`；当前只用于裂缝隔离试片，不是整车默认。
静态支撑核算用 `python3 tools/check_m2_crack_bridge.py --output <新JSON路径>`，读取当前
轮径与裂缝参数，记录 0.1／0.05 mm 两档采样结果。它不替代动态越缝与逐轮载荷验收。
实际轮径的隔离扫描入口为 `tools/probe_m2_joint_sweep.py --ode-prefix <局部ODE安装目录>
--output <新目录>`；附加 `--rounded-rims` 测试 0.1 mm 圆角，`--caster-only` 仅测球轮。
`--native-heightmaps` 与 `--collision-detector fcl` 是失败定位对照，不能组合使用；均不
改变正式应用后端。当前只有主动轮短扫描通过，不能拿这些诊断当作球轮或整车验收。
球径／缝口四组复现：`python3 tools/compare_m2_caster_lips.py --ode-prefix <局部安装目录>
--output <新目录>`。固定 25／40 mm 球径、0／0.25 mm 口沿 45° 倒角及交汇路径；
读取 `comparison.json` 的 `all_physics_checks_passed`，工具退出 0 只表示四组完整完成。
单组参数为 `--caster-diameter-mm`、`--top-chamfer-mm`、`--paths cross --skip-static`，
均为隔离试片选项，不写回整车配置。原槽是陡斜壁梯形槽，不要将其结果称作纯直角槽测试。
所有测试均为无渲染物理探针，没有加载 Gazebo GUI／RViz，因此不能用其耗时作双 GUI RTF 证据；
没有启动 Ogre2 时无需加载 Mesa。后续 GUI／相机验收仍必须使用现有本地 Mesa 修复环境。

### M2 review 的小范围复验

不重跑完整 S6；三个 M1 运动工况分别在系统库和显式私有库下执行，包含起步静止阶段：

```bash
python3 tools/validate_m2_review.py \
  --ode-prefix "$HOME/opt/climbot-ode-0.16.2-hashfix-ccd/install" \
  --output "${CLIMBOT_DATA_ROOT:-$HOME/climbot_data}/m2-review-$(date +%Y%m%d-%H%M%S)"
```

每组检查真实加载库和系统 libccd、原有逐轮判据，并比较最小轮载、压力及摩擦裕量。
`--summarize-existing` 只重新核对已保存工况，不重跑仿真。结果不能扩大成整个 M1/M2 验收。
表面几何必须先构建工作区，Python 调用本地 C++ 扩展；不提供第二份 Python 公式回退。
