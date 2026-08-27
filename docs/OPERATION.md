# 实验与故障处置手册

更新：2026-08-27。第一次跑通全链路请看 [README 的快速启动](../README.md#快速启动)——
仿真、规划预览、点选任务、离线处理和墙面拼接的主线命令都在那里，本页不重复。

本页只写主线之外的内容：批量回归、评价工具、参数变体和故障处置。接口字段见
[INTERFACES](INTERFACES.md)，历史命令和参数探索见[操作归档](archive/operations/OPERATION_2026-08-27.md)。

## 运行前准备

下面所有命令都假定已经执行过：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export CLIMBOT_DATA_ROOT=/your/chosen/data/root
```

`CLIMBOT_DATA_ROOT` 仅在采集、预处理或拼接时需要，必须是记录器主机可写的绝对路径。不要将
真实路径写入参数文件、说明、日志或提交的结果。

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

带贴图运行时一律设 `wall_grid_spacing:=0`，避免参考网格线进入巡检图像。

## 运行纪律

- 运行中只用 **Cancel** 正常停止。
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
| 采集任务无法启动 | 检查 `CLIMBOT_DATA_ROOT` 已设置、绝对且记录器可写；再看 `/inspection/archive/status` 的错误字段 |
| 只有预览、机器人不动 | 用 `coverage_mission.launch.py`，不是 `coverage_sim.launch.py` |
| SCAN 刚开始就零速中止 | capture gate 没有到达。查采集节点日志：任务的 `detection_forward_offset` 与相机安装外参不符时，采集节点**故意一条心跳都不发** |
| 拼接预检拒绝 | 不要修补输入；按 JSON 修复原始归档或标定问题后，重新生成一个独立的 processed-run |
| 真值 tile 有黑边或零覆盖 | 这是实际足迹缺口，须扩大或调整任务重新采集，不可用后处理填充 |
| Gazebo 无画面 | 加 `headless:=true` 走非 GUI 流程，或检查 WSLg/GPU 后端 |
