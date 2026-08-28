# 实验与故障处置手册

更新：2026-08-28。第一次跑通全链路请看 [README 的快速启动](../README.md#快速启动)——
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

# 诊断墙与 realistic 定位：P2 盲测数据用这一条
ros2 launch climbot_bringup coverage_mission.launch.py \
  wall_texture:=textures/wall_diagnostic_025/wall_texture.json \
  wall_grid_spacing:=0 localization_profile:=realistic \
  inspection_output_root:="$CLIMBOT_DATA_ROOT"
```

`input_mode` 默认是 `rviz`，也就是等着有人用 `Publish Point` 点选区域。**一旦关掉 RViz，
就必须同时把它改成 `parameters`**，否则规划器会一直等一个永远不会到来的点击。改成
`parameters` 之后，区域角点来自 `planner_config_file`，而 `region_type` 和
`sweep_direction` 覆盖配置里的对应项；可用的演示配置见
[climbot_coverage README](../src/climbot_coverage/README.md)。

### P2-06 近绿框诊断采集

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

G4 对高帧数任务采用每 32 张一次的耐久提交，避免逐图 `fsync` 造成宿主磁盘长时间满载；图像和
标签本身仍是一对一原子可见。运行中 manifest 的 `staged_images` 是尚未耐久提交的尾批，任务通过
完成、取消或失败的 finalization 结束时会强制提交。只把 `outcome=completed`、`staged_images=0`
的 run 交给正式离线处理；如需调整批量，修改
`archive_recorder_node.durable_commit_batch_images`（正整数）。

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

```bash
tools/run_coverage_regression.sh -t <tag> -j 4

ros2 launch climbot_gazebo climbot_wall.launch.py use_sim_time:=true headless:=true
ros2 run climbot_gazebo evaluate_localization.py --ros-args \
  -p use_sim_time:=true -p summary_json:=results/localization_<tag>_summary.json
```

回归脚本的摘要是仿真证据，不是实机验收。需要改变噪声、控制或定位 profile 时，使用显式参数
并保留输出 provenance；不要改写现有正式结果。长时间作业按 [AGENTS.md](../AGENTS.md) 后台运行
并把完整日志留在 `/tmp`。

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

## 诊断墙后验检查

只有诊断墙数据才能做这一步，且必须在 mosaic 渲染完成之后运行——评价器是独立的，不参与
候选生成、匹配、优化或渲染决策。

`MOSAIC_ROOT` 就是上一步 `build_wall_mosaic` 的 `--output-dir`：

```bash
MOSAIC_ROOT="$CLIMBOT_DATA_ROOT/mosaic-<id>-hardcut"

ros2 run climbot_mosaic evaluate_diagnostic_mosaic \
  --mosaic-dir "$MOSAIC_ROOT" \
  --wall-manifest "$PWD/textures/wall_diagnostic_025/wall_texture.json" \
  --output-dir "$MOSAIC_ROOT-truth-<new-id>"

ros2 run climbot_mosaic inspect_diagnostic_mosaic \
  --mosaic-dir "$MOSAIC_ROOT" \
  --wall-manifest "$PWD/textures/wall_diagnostic_025/wall_texture.json" \
  --output-dir "$MOSAIC_ROOT-inspection-<new-id>"
```

重点看真值摘要里的共同锚点、局部残差、覆盖计数和未覆盖 feature。原尺寸 tile 只能证明
**已经导出**，不能证明相机拍到了它的每一个像素——这正是 P2-06 至今未关闭的原因，
当前缺口和关闭条件以 [STATUS](STATUS.md) 为准。

## 墙面贴图

墙面贴图是本地可再生产物，由 `.gitignore` 排除，不进仓库。需要时用
`tools/fetch_wall_texture.sh` 取素材，再按 `tools/bake_wall_texture.py --help` 和
`tools/create_diagnostic_wall.py --help` 烘焙普通墙或诊断墙。

## 故障处置

| 现象 | 处置 |
| --- | --- |
| 采集任务无法启动 | 检查数据根（默认 `$HOME/climbot_data` 或 `CLIMBOT_DATA_ROOT`）可写；再看 `/inspection/archive/status` 的错误字段 |
| 只有预览、机器人不动 | 用 `coverage_mission.launch.py`，不是 `coverage_sim.launch.py` |
| SCAN 刚开始就零速中止 | capture gate 没有到达。查采集节点日志：任务的 `detection_forward_offset` 与相机安装外参不符时，采集节点**故意一条心跳都不发** |
| 拼接预检拒绝 | 不要修补输入；按 JSON 修复原始归档或标定问题后，重新生成一个独立的 processed-run |
| 真值 tile 有黑边或零覆盖 | 这是实际足迹缺口，须扩大或调整任务重新采集，不可用后处理填充 |
| Gazebo 无画面 | 加 `headless:=true` 走非 GUI 流程，或检查 WSLg/GPU 后端 |
| GUI 下曝光明显晚于目标或长任务少图 | 查 `Slow capture`、`Capture trigger ... late` 和 archive manifest；确认 launch 中存在独立的 `inspection_trigger_bridge`，不要把触发与全高清图像并回同一个 bridge。`gpu_backend:=software` 只能用于 A/B，不能代替桥接隔离。 |
| Pause 之后一直停在 `Pausing` | 机器人没能在 `pause_stop_timeout_s`（`5.0 s`）内停稳，执行器按控制超时中止任务。查看是否有第二个上游控制源在同时发 `/control/cmd_vel` |
| Pause 被拒绝且提示服务不可用 | 执行器没有提供 `/coverage/executor_pause`。任务原样继续，不是停了；检查 `line_tracker_node` 是否以 `standalone_mode:=false` 启动 |
| Pause 之后状态变成 `Stopping` | 执行器接了暂停请求却在 `pause_response_timeout_s`（`2.0 s`）内没有应答。管理器无从判断机器人是在减速还是仍在全速，按失联处理：请求 hold 并取消任务 |
