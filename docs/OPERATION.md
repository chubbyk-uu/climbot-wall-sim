# 实验与故障处置手册

更新：2026-10-22。第一次跑通全链路请看 [快速启动](QUICKSTART.md#快速启动)——
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



## 专项操作

- [采集与历史 P2-06 复现](operations/CAPTURE.md)
- [图像处理、拼接与后验评价](operations/IMAGE_PIPELINE.md)
- [M2 摩擦、接触与 ODE](operations/M2_PHYSICS.md)
