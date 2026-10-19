# 实验与故障处置手册

更新：2026-10-19。第一次跑通全链路请看 [README 的快速启动](../README.md#快速启动)——
仿真、规划预览、点选任务、离线处理和墙面拼接的主线命令都在那里，本页不重复。

本页只写主线之外的内容：批量回归、评价工具、参数变体和故障处置。接口字段见
[INTERFACES](INTERFACES.md)，历史命令和参数探索见[操作归档](archive/operations/OPERATION_2026-08-27.md)。

## 运行前准备

下面所有命令都假定已经执行过：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# 可选：未设置时保留系统默认的 $HOME/climbot_data；已有值会被保留。
export CLIMBOT_DATA_ROOT="${CLIMBOT_DATA_ROOT:-$HOME/climbot_data}"
```

`CLIMBOT_DATA_ROOT` 用于覆盖默认数据根，必须是记录器主机可写的绝对路径。未设置时，launch 使用
当前用户的 `$HOME/climbot_data`，并在首次归档时创建；不要将真实路径写入参数文件、说明、日志或提交的结果。

## 启动变体

主线之外常用的两种跑法：

```bash
# 无 GUI 批处理：任务来自配置文件而不是点选
ros2 launch climbot_bringup coverage_mission.launch.py \
  headless:=true rviz:=false input_mode:=parameters \
  planner_config_file:=<配置绝对路径> \
  region_type:=rectangle sweep_direction:=horizontal \
  inspection_output_root:="$CLIMBOT_DATA_ROOT"

# 新版诊断墙与 realistic 定位；默认诊断资产须先按 README 生成
ros2 launch climbot_bringup coverage_mission.launch.py \
  wall_grid_spacing:=0 localization_profile:=realistic \
  inspection_output_root:="$CLIMBOT_DATA_ROOT"
```

`input_mode` 默认是 `rviz`，也就是等着有人用 `Publish Point` 点选区域。**一旦关掉 RViz，
就必须同时把它改成 `parameters`**，否则规划器会一直等一个永远不会到来的点击。改成
`parameters` 之后，区域角点来自 `planner_config_file`，而 `region_type` 和
`sweep_direction` 覆盖配置里的对应项；可用的演示配置见
[climbot_coverage README](../src/climbot_coverage/README.md)。

### 固定压差与压力故障诊断

正常入口已默认新底盘＋vacuum，见根 README。需要同插件 B 对照时才显式传入：

```bash
ros2 launch climbot_bringup coverage_mission.launch.py \
  adhesion_mode:=fixed_pressure input_mode:=parameters
```

vacuum 与 fixed_pressure 共享几何、裙边、传感器和施力点，仅压力源不同。
`auto` 等价于 vacuum；旧 `legacy` 几何和 `legacy_constant` 模式已删除，不会静默回退。
旧 A/B0 对照从冻结 Git 版本（如 `cb3fbda`）复现，不向当前运行链重新引入旧底盘。

压力观测和三机／传感器故障 topic 见 [INTERFACES](INTERFACES.md#吸附与压力接口)。
C 从环境压启动，压力就绪后撤安装支撑；失败后不会自动转为理想恒力。
默认已通过 [S6](../results/suction_s6_2026-10-18/REPORT.md) 与
[S7](../results/suction_s7_2026-10-18/REPORT.md)。Gazebo 和 RViz 双开、正常采集完整任务
平均 RTF ≥0.95；右下角显示的是瞬时 RTF，不能用单次截图替代完整任务统计。

控制回归默认已选新底盘，GUI 使用 `COVERAGE_HEADLESS=false`、`COVERAGE_RVIZ=true` 和 `-j 1`；
采集工具使用 `G2_HEADLESS=false G2_RVIZ=true`。完整输出写 /tmp，各组独立记录原生时钟、
窗口、Mesa 映射和 TF。world reset 或 RViz Reset 后，可视化缓存约 1 s 内自动恢复，
不触发重规划或重新发布执行任务；正在执行时不进行 world reset。

### 历史 P2-06 近绿框诊断采集

本节是旧贴图 1340 张历史基线的复现命令，显式保留旧资产路径。新任务使用 README 的
新版默认墙；旧基线的零漏拍结论不转移给新版资产。

先从仓库根目录运行一次采集前预检；它按真实离散触发合同检查横、竖任务的**联合**覆盖，
不启动 Gazebo，也不替代采集后的实际覆盖图：

```bash
ros2 run climbot_mosaic preflight_diagnostic_coverage \
  --task-config src/climbot_coverage/config/coverage_p206_diagnostic_full_horizontal.yaml \
  --task-config src/climbot_coverage/config/coverage_p206_diagnostic_full_vertical.yaml \
  --wall-manifest textures/wall_diagnostic_025/wall_texture.json \
  --camera-config src/climbot_description/config/inspection_camera.yaml \
  --robot-config src/climbot_description/config/robot.yaml \
  --wall-config src/climbot_description/config/wall.yaml \
  --output "$PWD/results/p206_diagnostic_coverage_preflight_2026-08-28.json"
```

然后分别启动横向和纵向任务。`sweep_direction` 是 launch 覆盖项，必须显式写出，不能只依赖
YAML 内同名字段。每次在 RViz 确认蓝线和绿色安全框后按 **Start**；任务完成才启动下一次。

```bash
# horizontal: 17 SCAN / nominal 680 exposures
ros2 launch climbot_bringup coverage_mission.launch.py \
  input_mode:=parameters region_type:=rectangle sweep_direction:=horizontal \
  planner_config_file:="$PWD/src/climbot_coverage/config/coverage_p206_diagnostic_full_horizontal.yaml" \
  wall_texture:="$PWD/textures/wall_diagnostic_025/wall_texture.json" \
  wall_grid_spacing:=0 localization_profile:=realistic

# vertical: 22 SCAN / nominal 660 exposures after maneuver-envelope reservation
ros2 launch climbot_bringup coverage_mission.launch.py \
  input_mode:=parameters region_type:=rectangle sweep_direction:=vertical \
  planner_config_file:="$PWD/src/climbot_coverage/config/coverage_p206_diagnostic_full_vertical.yaml" \
  wall_texture:="$PWD/textures/wall_diagnostic_025/wall_texture.json" \
  wall_grid_spacing:=0 localization_profile:=realistic
```

两次 G4 archive 必须保留并共同送进后续预处理和 hard-cut 流程。不要把横向单独作为“全覆盖”
数据：预检已证明它在三个边缘 decal 上留有离散曝光缺口。

当前预检结果是横向 17 条／680 张、纵向 22 条／660 张，联合 1340 张。纵向少于最初未预留
边界机动空间时的 682 张，是因为上下端点各向内保留了 100 mm 转向／补偿裕量；预检工具与
在线规划器使用同一安全矩形，不能手工把旧计数当成归档期望值。

WSL 默认用 D3D12 GPU 渲染。只有排查渲染后端时才加 `gpu_backend:=software` 做对照；软件路径
会让 Gazebo、RViz 和传感器统一走 llvmpipe，通常更慢、CPU 和内存也更高。正常运行不要设置
该参数；`auto` 会回到 D3D12。

WSL D3D12 默认使用共享 Mesa 修复库，原生 Linux 自动走 native，不加载它。构建脚本和补丁
已随本仓库提供；安装、共享前缀、模式切换及实际选库验证统一见
[MESA_SETUP](MESA_SETUP.md)，本页不维护第二套安装命令。headless 相机采集同样需要渲染库。

`clock_publish_hz` 默认 `500`。它只抽稀 ROS 侧的时钟广播，**物理步长不变，仿真一步不差**
（实测 `gz sim` 自身占用 98.9% 对 98.8%，无差别）。设为 `0` 可退回按每个物理步长直发。

可达速率只有 1000 的整除档（1000/500/333.3/250/200……），请求落在两档之间会降到下一档，
因此启动约 5 s 后节点会打印 `CLOCK_THROTTLE measured input=… delivered=… requested=…`，
**以 delivered 为准**，不符时另有 WARN。之后每 10 s 打印一次收/发两侧的墙上时间间隔统计：
两侧最大值相近说明断档来自上游，出侧显著大于入侧才说明节流节点自己是瓶颈。

实测（P2-06 横向全任务，headless，运动期 600 s 采样）：直连 295.0% 对 500 Hz 234.7%，
**净省 0.60 核，已扣除节流节点自身的 12.4%**；去掉评价器后生产侧净省 0.48 核。同一批任务
长任务中位姿 p95 差异在 0.15 mm 以内；500 Hz 的 7 次运行中 6 次无运动期 ≥100 ms 事件、
1 次在 54 s 内聚集 5 次，直连 1000 Hz 的 4 次均为 0。该单次聚集按主机扰动记录，不能写成
“两档从未出现”；完整口径见[长任务时序稳定性](LOCALIZATION_GAP_PLAN.md)的调查归档。

G4 对高帧数任务采用每 32 张一次的耐久提交，避免逐图 `fsync` 造成宿主磁盘长时间满载；图像和
标签本身仍是一对一原子可见。运行中 manifest 的 `staged_images` 是尚未耐久提交的尾批，任务通过
完成、取消或失败的 finalization 结束时会强制提交。只把 `outcome=completed`、`staged_images=0`
的 run 交给正式离线处理；如需调整批量，修改
`archive_recorder_node.durable_commit_batch_images`（正整数）。

`syncfs(2)` 刷新的是整个文件系统，批大小只决定它多久调用一次，不能限制单次停顿多长。因此每
次耐久提交都会输出一条 INFO，记录本批张数、本次 syncfs 毫秒数和本 run 的累计最大值；同样的
数值也写入 manifest 的 `durable_commits`、`durable_commit_last_ms`、`durable_commit_last_images`、
`durable_commit_max_ms` 和 `durable_commit_max_images`，长跑证据不依赖临时终端日志。判读时注意
它只计 syncfs 本身，不含随后那次耐久 manifest 写入。

带贴图运行时一律设 `wall_grid_spacing:=0`，避免参考网格线进入巡检图像。

## 运行纪律

- 想让机器人停一下再接着跑，用 **Pause**，恢复用 **Resume**。暂停不结束任务：任务号、
  段序号和归档 run 都不变，恢复后从当前位姿继续同一段，段超时和调度时基在暂停期间不走。
- 真要结束这次任务才按 **Stop**，它的含义没有因为 Pause 的加入而改变。
- Pause 按下后状态先是 `Pausing`，机器人减速；命令和实测速度都归零之后才变 `Paused`。
  没到 `Paused` 之前 Resume 会被拒绝，这是正常的，等一下再点。
- 暂停中定位数据过期时 Resume 会被拒绝并说明原因；任务仍然停在原地，等定位恢复再点一次，
  不需要重新开始。
- 归档失败、capture gate 丢失、Action 失败或看门狗超时都要**等受控停车走完**，不要在它
  结束之前再发别的命令。
- 任何时刻只允许一个上游控制源；不要同时启动键盘遥控和自动控制。
- 绿色框是机器人安全工作区；黄色相机足迹只是预测覆盖，既不是安全边界，也不是实际覆盖结论。

## 覆盖回归与定位对照

### 负压专项的旧模型基线与静力诊断

在已经按根 README 加载 ROS 和工作区环境的终端中执行。先提交 `src/` 与 `tools/` 的修改，
选择未被其他仿真实例使用的 ROS 域号；输出目录必须不存在，不覆盖历史批次：

```bash
bash tools/run_suction_baseline.sh results/suction_A_<tag> 187
python3 tools/screen_suction_layouts.py \
  --baseline results/suction_A_<tag>/baseline_summary.json \
  --observations results/suction_A_<tag>/a_loads_summary.json \
  --output-dir results/suction_A_<tag>/static_screening
```

第一条冻结旧输入并与 `c2acf17` 和安装区比较，自动创建独立 Gazebo partition／进程组，
复测 34 个 A 组轮压工况，清理后写 `run_summary.json`。返回非零时保留失败批次、查 `/tmp`
下分离的模拟器／测量日志；`cleanup_passed`、实际测量退出码和原有测量器 `passed` 分别核对。
日志路径与命令摘要在结果中留存，不把完整调试日志提交到仓库。

第二条仅按冻结的**旧质量／重心**扫描压力中心和球位，输出 1° 全航向及极值附近 0.1°
采样。包含静态偏航，分别报告按法向比例分配的需求和摩擦圆可行域下限，不计裙边或动态。
不能将其输出命名为新结构 `F_design`，也不能把可行域下限当作真实接触分配已验证。
旧几何删除后需从保存该基线工具及 legacy 描述的 Git 版本复现，不重新引入第二套活动模型。

合同见 [SUCTION_CONTRACT](SUCTION_CONTRACT.md)，正式批次入口见
[结果索引](../results/README.md)。原有轮压测量器只保存事件统计，未留逐步原始接触时序；
这批结果不能替代新模型对缺失采样、启动撤架和短时失载的时序验收。

### 既有覆盖与定位入口

```bash
tools/run_coverage_regression.sh -t <tag> -j 4

ros2 launch climbot_gazebo climbot_wall.launch.py use_sim_time:=true headless:=true
ros2 run climbot_gazebo evaluate_localization.py --ros-args \
  -p use_sim_time:=true -p summary_json:=results/localization_<tag>_summary.json
```

回归脚本的摘要是仿真证据，不是实机验收。需要改变噪声、控制或定位 profile 时，使用显式参数
并保留输出 provenance；不要改写现有正式结果。长时间作业按 [AGENTS.md](../AGENTS.md) 后台运行
并把完整日志留在 `/tmp`。

## 采集内容核对

计数守卫（`expected == saved`、`failed=0`、`staged=0`）不看像素。把新采集与一次已知良好的
同计划采集比对，产出可引用的记录：

```bash
ros2 run climbot_mosaic summarize_archive_content \
  --input-run "$CLIMBOT_DATA_ROOT/<新采集 run 目录>" \
  --reference-run "$CLIMBOT_DATA_ROOT/<基准 run 目录>" \
  --output-dir "$CLIMBOT_DATA_ROOT/archive-content-<id>"
```

横向只能对横向、纵向只能对纵向；比对按同序帧对齐，所以帧数必须与参考一致，不一致直接拒绝。
判定不通过时退出码为 `3`，不要把它当成可以忽略的告警——两整组 z-fighting 数据就是在计数全绿的
情况下走到拼接阶段的。这些记录是正式摘要的强制输入：`build_mosaic_evidence_summary.py` 要求
每个冻结任务的归档都被某份记录覆盖并通过，缺一份就拒绝出摘要。

## 预处理与拼接的前置检查

逐步命令见 [README 的离线图像处理](../README.md#4-离线图像处理)和
[离线墙面拼接](../README.md#5-离线墙面拼接)。开始之前先确认三件事：

1. G4 manifest 的 `outcome` 为 `completed`；半个 run 的计数本来就不可信，
   `--allow-incomplete` 只用于明确的取证运行。
2. 每一级的输出目录都是**新的、不存在的绝对路径**，且不在输入 run 内部。
3. 横向和竖向两次采集要一起拼时，把多个 `--input-run` 传给同一条命令，而不是各拼各的
   再去合并两张成品。

原始归档、processed-run 和正式 mosaic 一经发布不可覆盖；`--work-dir` 下的缓存可以随时删除重建。
输入输出契约见 [接口合同](INTERFACES.md)，设计与门禁见 [拼接计划](MOSAIC_PLAN.md)，
各命令的完整参数以 [`climbot_image_processing`](../src/climbot_image_processing/README.md) 和
[`climbot_mosaic`](../src/climbot_mosaic/README.md) 的 README 及 `--help` 为准。

本项目 CUDA 开发机上的完整拼接、性能和验收运行，`build_wall_mosaic` 必须显式传
`--backend cuda`，结束后核对 `mosaic_manifest.json` 的 `execution.effective` 为 `cuda` 且 `execution.fallback=false`。
不要用 `auto` 生成 GPU 性能或验收证据；它可能按设计回退 CPU。CPU 只用于明确的基线或 A/B。

## 诊断墙后验检查

只有诊断墙数据才能做这一步，且必须在 mosaic 渲染完成之后运行——评价器是独立的，不参与
候选生成、匹配、优化或渲染决策。

`MOSAIC_ROOT` 就是上一步 `build_wall_mosaic` 的 `--output-dir`。下面采用新任务的默认墙；
复查旧数据时必须改为该采集归档记录的原始清单，不能用新墙评价旧照片：

```bash
MOSAIC_ROOT="$CLIMBOT_DATA_ROOT/mosaic-<id>-hardcut"

ros2 run climbot_mosaic evaluate_diagnostic_mosaic \
  --mosaic-dir "$MOSAIC_ROOT" \
  --wall-manifest "$CLIMBOT_DATA_ROOT/assets/wall_local_diagnostic_v6/wall_texture.json" \
  --output-dir "$MOSAIC_ROOT-truth-<new-id>"

ros2 run climbot_mosaic inspect_diagnostic_mosaic \
  --mosaic-dir "$MOSAIC_ROOT" \
  --wall-manifest "$CLIMBOT_DATA_ROOT/assets/wall_local_diagnostic_v6/wall_texture.json" \
  --output-dir "$MOSAIC_ROOT-inspection-<new-id>" \
  --input-run "$CLIMBOT_DATA_ROOT/processed-<horizontal-id>" \
  --input-run "$CLIMBOT_DATA_ROOT/processed-<vertical-id>"
```

**巡检域不再由命令行给出。** `--input-run` 指出这幅拼接是由哪些 processed-run 建的，
检查器沿

```
mosaic manifest → processing_manifest.json → 归档 manifest.json → 冻结任务
```

一路回溯，每一环都用 SHA-256 校验，然后从归档里的 `task.coverage_region` 取巡检域、
从 `task.detection_*` 取相机足迹。指错 run 会因为摘要里的 digest 对不上而被拒绝，所以
运行检查器的人不能再选择这次运行被拿什么来判。`--archive-root` 默认取
`$CLIMBOT_DATA_ROOT`。

摘要因此输出 `all_inspection_region_feature_pixels_covered`（门禁）和逐 feature 的域内／域外
缺口。域外再按两套冻结几何拆分：任务实际连续 SCAN 足迹，以及 `motion_region` 内正反扫描
方向的安全位姿相机包络。前者回答计划是否本应扫到，后者回答是否存在安全中心位姿可拍；
不能再从保守上包络的成员关系推断位姿存在。域内 feature 像素数为零时检查器直接报错，
不会输出一个什么都没量到的 `true`。

重点看真值摘要里的共同锚点、局部残差、覆盖计数和未覆盖 feature。原尺寸 tile 只能证明
**已经导出**，不能证明相机拍到了它的每一个像素。当前门禁状态以 [STATUS](STATUS.md) 为准。

## 墙面贴图

素材来源、校验、普通墙／诊断墙的完整生成命令及资源预算统一见
[README：从源码生成资产](../README.md#4-从源码生成资产)。墙面贴图由 `.gitignore` 排除，
不进仓库；实验时显式用 `wall_texture` 指向生成的清单。

## 故障处置

| 现象 | 处置 |
| --- | --- |
| 采集任务无法启动 | 检查数据根（默认 `$HOME/climbot_data` 或 `CLIMBOT_DATA_ROOT`）可写；再看 `/inspection/archive/status` 的错误字段 |
| 只有预览、机器人不动 | 用 `coverage_mission.launch.py`，不是 `coverage_sim.launch.py` |
| SCAN 刚开始就零速中止 | capture gate 没有到达。查采集节点日志：任务的 `detection_forward_offset` 与相机安装外参不符时，采集节点**故意一条心跳都不发** |
| 拼接预检拒绝 | 不要修补输入；按 JSON 修复原始归档或标定问题后，重新生成一个独立的 processed-run |
| 真值 tile 有黑边或零覆盖 | 这是实际足迹缺口，须扩大或调整任务重新采集，不可用后处理填充 |
| Gazebo 无画面 | 加 `headless:=true` 走非 GUI 流程，或检查 WSLg/GPU 后端 |
| WSL 启动提示 Mesa 缺失或摘要不符 | 按 [Mesa 安装与验证](MESA_SETUP.md)恢复完整共享库与构建记录；不要用 `mesa:=system` 把缺库当成已修复 |
| 曝光明显晚于目标或长任务少图 | 查 `Slow capture`、`Capture trigger ... late` 和 archive manifest；确认 launch 同时存在独立的 `inspection_camera_bridge` 与 `inspection_trigger_bridge`，不要把全高清图像/内参或触发并回承载 `/clock`、里程计和 IMU 的 `simulation_data_bridge`。`gpu_backend:=software` 只能用于 A/B，不能代替桥接隔离。 |
| `Slow capture` 的 `source pair` 达到 0.5–3 s | Fast DDS 默认共享内存段无法同时保留 6220800 字节曝光的全部 RTPS 分片，丢分片后要等 3 s 心跳。确认 `inspection_camera_bridge`、`camera_distortion_adapter` 与 `capture_once_node` 的环境里有 `FASTRTPS_DEFAULT_PROFILES_FILE` 指向 `climbot_common/config/fastdds_inspection_image.xml`（`tr '\0' '\n' < /proc/<pid>/environ`），且 `/dev/shm/fastrtps_*` 中存在约 67 MB 的段而不是 549408 字节的默认段。自带 profile 会抑制这个默认值。 |
| Pause 之后一直停在 `Pausing` | 机器人没能在 `pause_stop_timeout_s`（`5.0 s`）内停稳，执行器按控制超时中止任务。查看是否有第二个上游控制源在同时发 `/control/cmd_vel` |
| Pause 被拒绝且提示服务不可用 | 执行器没有提供 `/coverage/executor_pause`。任务原样继续，不是停了；检查 `line_tracker_node` 是否以 `standalone_mode:=false` 启动 |
| Pause 之后状态变成 `Stopping` | 执行器接了暂停请求却在 `pause_response_timeout_s`（`2.0 s`）内没有应答。管理器无从判断机器人是在减速还是仍在全速，按失联处理：请求 hold 并取消任务 |


## M2 原生接触与局部 ODE 修复

仅用于 [M2 T0](SUCTION_M2_PLAN.md) 分项核查，尚未改变当前采集默认或完成整车验收。
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
构建仅保留历史诊断，当前加载守卫拒绝它；不要覆盖旧前缀。M1 默认仍使用系统库。
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
