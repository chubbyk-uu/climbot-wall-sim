# 操作手册

更新：2026-08-27。本页提供当前可复制的完整流程与故障边界；接口字段见
[INTERFACES](INTERFACES.md)，历史命令和参数探索见[操作归档](archive/operations/OPERATION_2026-08-27.md)。

## 运行前准备

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export CLIMBOT_DATA_ROOT=/your/chosen/data/root
```

`CLIMBOT_DATA_ROOT` 仅在采集、预处理或拼接时需要，必须是记录器主机可写的绝对路径。不要将
真实路径写入参数文件、说明、日志或提交的结果。

## 仿真、规划与执行

仅预览规划：

```bash
ros2 launch climbot_bringup coverage_sim.launch.py
```

完整点选任务与巡检：

```bash
ros2 launch climbot_bringup coverage_mission.launch.py \
  inspection_output_root:="$CLIMBOT_DATA_ROOT" wall_grid_spacing:=0
```

在 RViz 中依次点选区域角点（矩形两点、梯形三点），确认 `Coverage Task` 为 Ready 后按 Start。
绿色框是机器人安全工作区；黄色相机足迹只是预测覆盖，不能作为安全边界或实际覆盖结论。

常用变体：

```bash
# 无 GUI 批处理
ros2 launch climbot_bringup coverage_mission.launch.py headless:=true rviz:=false \
  inspection_output_root:="$CLIMBOT_DATA_ROOT"

# 诊断墙与 realistic 定位
ros2 launch climbot_bringup coverage_mission.launch.py \
  wall_texture:=textures/wall_diagnostic_025/wall_texture.json \
  wall_grid_spacing:=0 localization_profile:=realistic \
  inspection_output_root:="$CLIMBOT_DATA_ROOT"
```

运行中只使用 Cancel 正常停止。归档失败、capture gate 丢失、Action 失败或看门狗超时均应等待
受控停车；不要同时启动键盘和自动控制。

## 覆盖回归与定位对照

```bash
tools/run_coverage_regression.sh -t <tag> -j 4
ros2 launch climbot_gazebo climbot_wall.launch.py use_sim_time:=true headless:=true
ros2 run climbot_gazebo evaluate_localization.py --ros-args \
  -p use_sim_time:=true -p summary_json:=results/localization_<tag>_summary.json
```

回归脚本的摘要是仿真证据，不是实机验收。需要改变噪声、控制或定位 profile 时，使用显式参数
并保留输出 provenance；不要改写现有正式结果。

## 巡检、预处理与拼接

任务完成后，先确认 G4 manifest 的状态为 `completed`，再新建独立的 processed 与 mosaic 目录：

```bash
RAW_RUN="$CLIMBOT_DATA_ROOT/<task-id>/<run-id>"
PROCESSED_RUN="$CLIMBOT_DATA_ROOT/processed-<new-id>"
MOSAIC_ROOT="$CLIMBOT_DATA_ROOT/mosaic-<new-id>"

ros2 run climbot_image_processing process_inspection_archive \
  --input-run "$RAW_RUN" --output-dir "$PROCESSED_RUN" \
  --flat-field-file "$CLIMBOT_DATA_ROOT/calibration/<flat-field>.npz" \
  --denoise none --jobs auto --memory-budget-gb 4

ros2 run climbot_mosaic validate_mosaic_inputs --input-run "$PROCESSED_RUN"
```

多个 processed-run 可传给同一后续拼接命令。按 `--help` 依次运行候选、局部匹配、位姿图和
hard-cut 渲染；输入和输出契约见 [接口合同](INTERFACES.md)，设计、门禁与资源要求见
[拼接计划](MOSAIC_PLAN.md)。缓存可重建，原始归档、processed-run 和正式 mosaic 不可覆盖。

## 诊断墙后验检查

```bash
ros2 run climbot_mosaic evaluate_diagnostic_mosaic \
  --mosaic-dir "$MOSAIC_ROOT" \
  --wall-manifest "$PWD/textures/wall_diagnostic_025/wall_texture.json" \
  --output-dir "$MOSAIC_ROOT-truth-<new-id>"

ros2 run climbot_mosaic inspect_diagnostic_mosaic \
  --mosaic-dir "$MOSAIC_ROOT" \
  --wall-manifest "$PWD/textures/wall_diagnostic_025/wall_texture.json" \
  --output-dir "$MOSAIC_ROOT-inspection-<new-id>"
```

阅读真值摘要中的共同锚点、局部残差、覆盖计数和未覆盖 feature；原尺寸 tile 只证明已导出，
不表示相机已经覆盖其所有像素。P2-06 的当前缺口和关闭条件以 [STATUS](STATUS.md) 为准。

## 墙面贴图与故障处置

墙面贴图是本地可再生产物；需要时按 `tools/fetch_wall_texture.sh`、
`tools/bake_wall_texture.py --help` 与 `tools/create_diagnostic_wall.py --help` 生成。贴图运行时将
`wall_grid_spacing:=0`，避免网格线进入巡检图像。

| 现象 | 处置 |
| --- | --- |
| 采集任务无法启动 | 检查 `CLIMBOT_DATA_ROOT` 已设置、绝对且记录器可写；检查归档 status 错误 |
| 只有预览、机器人不动 | 使用 `coverage_mission.launch.py`，而非 `coverage_sim.launch.py` |
| 拼接预检拒绝 | 不要修补输入；根据 JSON 修复原始归档/标定问题后重新生成独立 processed-run |
| 真值 tile 有黑边或零覆盖 | 这是实际足迹缺口，须扩大/调整任务并重新采集，不可用后处理填充 |
| Gazebo 无画面 | 加 `headless:=true` 做非 GUI 流程，或检查 WSLg/GPU 后端 |
