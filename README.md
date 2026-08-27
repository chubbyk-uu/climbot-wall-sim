# Climbot Sim

基于 ROS 2 Jazzy 与 Gazebo Harmonic 的垂直壁面爬壁机器人仿真。系统保留真实重力，以持续
法向吸附力贴墙；覆盖规划、直线闭环控制、位置触发采集、离线图像处理和全局墙面拼接均在本仓库
内完成。控制不依赖 Nav2，拼接算法不读取 Gazebo 真值或墙面纹理。

![Gazebo 中的墙面与机器人](docs/images/gazebo_wall.png)

## 你能做什么

- 在 Gazebo 中启动墙面、机器人、传感器、EKF 和 RViz；
- 点选矩形或等腰梯形任务区，执行横向或纵向弓字覆盖；
- 在正式扫描线自动拍摄原始灰度图，并将每张图和曝光时刻融合相机位姿原子归档；
- 离线校验归档、平场校正、去畸变、匹配、全局位姿优化和硬切墙面拼接；
- 用诊断墙真值和原尺寸 tile 检查拼接的绝对偏差、接缝和缺陷细节。

当前 A～G4 仿真链路已完成；P2 拼接已完成 P2.7e，仍需大工作区盲测补足诊断目标覆盖后冻结
最终门限。当前状态见 [STATUS](docs/STATUS.md)。

## 环境要求

| 组件 | 已验证版本 |
| --- | --- |
| 系统 | Ubuntu 24.04 / WSL2 或原生 Ubuntu |
| ROS 2 | Jazzy Jalisco |
| Gazebo | Harmonic，`gz-sim 8.x` |
| Python | 3.12 |
| 构建 | C++17、colcon、rosdep |

GUI 在 WSL2 上需要 WSLg/GPU 图形支持；无 GUI 的采集、处理、拼接和测试可使用
`headless:=true`。墙面 DDS 贴图在 `textures/`，由 `.gitignore` 排除；新克隆若需带贴图的
视觉任务按 [墙面贴图与故障处置](docs/OPERATION.md#墙面贴图与故障处置) 生成。

采集、预处理与拼接的大文件不进入仓库。启用这些功能前，在**记录器所在主机**设置一个持久化、
可写的绝对目录：`export CLIMBOT_DATA_ROOT=/your/chosen/data/root`。未设置时，带采集的任务会
明确拒绝启动；不要把真实主机路径写入文档、注释或提交的结果。

## 安装与部署

先按 [ROS 2 Jazzy 官方 Ubuntu 安装说明](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
安装 `ros-jazzy-desktop`，再安装工作区工具：

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep

cd ~/robot_ws/climbot_sim
source /opt/ros/jazzy/setup.bash
sudo rosdep init  # 首次执行；已初始化时可跳过
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
```

构建并加载工作区：

```bash
source /opt/ros/jazzy/setup.bash
cd ~/robot_ws/climbot_sim
colcon build --symlink-install
source install/setup.bash
```

验证基础环境：

```bash
gz sim --versions
ros2 doctor --report | head -20
```

## 构建与测试

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test
colcon test-result --verbose
```

需要产品源码静态分析时：

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
bash tools/run_clang_tidy.sh --log /tmp/climbot-clang-tidy.log
```

CI 在每个 main 推送和 Pull Request 上执行构建、静态分析和串行测试。完整测试策略与故障排查见
[OPERATION](docs/OPERATION.md)。

## 快速启动

以下命令均假定已执行：

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/climbot_sim/install/setup.bash
```

### 1. 只看仿真

```bash
ros2 launch climbot_gazebo climbot_wall.launch.py
```

无 GUI：

```bash
ros2 launch climbot_gazebo climbot_wall.launch.py headless:=true
```

### 2. 规划预览

启动墙面、规划器和 RViz，但不执行机器人：

```bash
ros2 launch climbot_bringup coverage_sim.launch.py
```

在 RViz 使用 `Publish Point` 点选任务可走区。矩形点 A（左下）和 B（右上）；等腰梯形点 A（左下）、
B（右上）、C（右下）。所有点必须位于绿色安全框内。

### 3. 规划、控制与自动采集

```bash
ros2 launch climbot_bringup coverage_mission.launch.py \
  inspection_output_root:="$CLIMBOT_DATA_ROOT" \
  wall_grid_spacing:=0
```

在 RViz 的 **Coverage Task** 面板中点选、Replan；在 `Capture` 页确认原始归档开关与根目录，
再按 Start。系统只在正式 `SCAN` 段采图，归档原始 `mono8` 图像、曝光标签、标定和 manifest。
执行中只使用 Cancel；归档与受控停止语义见 [操作手册](docs/OPERATION.md#仿真规划与执行)。

![RViz 中执行覆盖任务](docs/images/rviz_coverage_task.png)

绿色虚线是安全工作区，橙色是选定任务区，蓝线是机器人中心路径，黄色带是相机预测足迹。

常用变体：

```bash
# 纵向扫描
ros2 launch climbot_bringup coverage_mission.launch.py sweep_direction:=vertical

# realistic 定位诊断墙
ros2 launch climbot_bringup coverage_mission.launch.py \
  wall_texture:=textures/wall_diagnostic_025/wall_texture.json \
  wall_grid_spacing:=0 localization_profile:=realistic
```

### 4. 离线图像处理

输入必须是已完成的 G4 原始归档；输出目录必须为不存在的绝对路径，且不得位于输入目录内：

```bash
RAW_RUN="$CLIMBOT_DATA_ROOT/<task-id>/<run-id>"
PROCESSED_RUN="$CLIMBOT_DATA_ROOT/processed-<new-id>"

ros2 run climbot_image_processing process_inspection_archive \
  --input-run "$RAW_RUN" \
  --output-dir "$PROCESSED_RUN" \
  --flat-field-file "$CLIMBOT_DATA_ROOT/calibration/<flat-field>.npz" \
  --denoise none --jobs auto --memory-budget-gb 4
```

该链路校验原始 SHA-256，保持原图不变，再执行可选暗场/平场、去噪和畸变校正。完整参数和
平场标定流程见 [image processing README](src/climbot_image_processing/README.md)。

### 5. 离线墙面拼接

将一个或多个 processed run 代入下列变量。所有输出目录均应为新的绝对路径；`WORK` 只是可删除
缓存，正式证据是每一步原子发布的输出目录。

拼接按下面的单向阶段运行。前一阶段失败时先修复输入或参数，不要跳过它强行运行后续阶段。

| 阶段 | 命令 | 做什么 | 关键产物 |
| --- | --- | --- | --- |
| 0 | `validate_mosaic_inputs` | 只读校验 processed 图像、标签、标定、位姿与 SHA-256 | 终端 JSON 摘要；不写目录 |
| 1 | `build_overlap_candidates` | 用曝光位姿投影足迹，并找出真正空间重叠的照片对 | `overlap_candidates.json` |
| 2 | `build_local_matches` | 仅在候选重叠区做特征匹配、RANSAC 与局部 SE(2) 约束 | `local_matches.json`；`WORK` 中是可重建缓存 |
| 3 | `build_pose_graph` | 结合 EKF 绝对先验与视觉边，优化每张照片的 `x/y/yaw` 修正 | `optimized_poses.json`、`pose_graph.json` |
| 4 | `build_wall_mosaic` | 以同一 hard-cut 规则渲染 pose-only 与 optimized 母版 | BigTIFF、覆盖/不确定度图、预览和 manifest |

`build_initial_projection` 是可选的几何诊断命令：它只输出每张图的墙面足迹预览，适合先检查
相机朝向、尺度和任务覆盖范围；阶段 1 会自行执行同一投影，因此常规流水无需单独运行它。

```bash
RUN_H="$CLIMBOT_DATA_ROOT/processed-<horizontal-id>"
RUN_V="$CLIMBOT_DATA_ROOT/processed-<vertical-id>"
ROOT="$CLIMBOT_DATA_ROOT/mosaic-<new-id>"

ros2 run climbot_mosaic validate_mosaic_inputs \
  --input-run "$RUN_H" --input-run "$RUN_V"

# 阶段 1：只保留足迹确有正面积相交的照片对。
ros2 run climbot_mosaic build_overlap_candidates \
  --input-run "$RUN_H" --input-run "$RUN_V" \
  --output-dir "$ROOT-candidates"

ros2 run climbot_mosaic build_local_matches \
  --input-run "$RUN_H" --input-run "$RUN_V" \
  --output-dir "$ROOT-matches" --work-dir "$ROOT-work-match" --jobs auto

ros2 run climbot_mosaic build_pose_graph \
  --input-run "$RUN_H" --input-run "$RUN_V" \
  --local-matches "$ROOT-matches/local_matches.json" \
  --output-dir "$ROOT-pose-graph"

ros2 run climbot_mosaic build_wall_mosaic \
  --input-run "$RUN_H" --input-run "$RUN_V" \
  --pose-graph-dir "$ROOT-pose-graph" \
  --output-dir "$ROOT-hardcut" --work-dir "$ROOT-work-fusion" \
  --resolution-mm-per-pixel 0.25 --jobs auto --memory-budget-gb 4
```

`mosaic_pose_only.tif` 与 `mosaic_optimized.tif` 使用相同输入、网格与 hard-cut 像素归属；唯一变量
是位姿图修正。`coverage_count.tif` 的零值表示相机从未覆盖该墙面像素；`uncertainty.tif` 的
nodata 同样不能被当作有效图像。诊断墙还可在拼接后运行[真值评价与原尺寸检查](docs/OPERATION.md#诊断墙后验检查)。

## 文档导航

| 需要了解 | 唯一入口 |
| --- | --- |
| 文档职责、归档和写作边界 | [docs/README.md](docs/README.md) |
| 项目目标、范围、硬约束与规范验收 | [PROJECT_GUIDE.md](PROJECT_GUIDE.md) |
| 包职责、依赖和数据流 | [ARCHITECTURE](docs/ARCHITECTURE.md) |
| 话题、服务、Action、参数、文件格式 | [INTERFACES](docs/INTERFACES.md) |
| 完整操作、参数和故障处置 | [OPERATION](docs/OPERATION.md) |
| 当前验收状态与正式证据 | [ACCEPTANCE](docs/ACCEPTANCE.md) |
| 当前项目状态、风险与下一步 | [STATUS](docs/STATUS.md) |
| 离线拼接设计与门禁 | [MOSAIC_PLAN](docs/MOSAIC_PLAN.md) |
| 结果有效性、基线与重生成入口 | [results/README.md](results/README.md) |
| 本地大数据保留与清理规则 | [DATA_RETENTION](docs/DATA_RETENTION.md) |

## 安全边界

Gazebo DiffDrive 会持续执行最后收到的速度命令。系统以速度看门狗作为 `/cmd_vel` 的唯一发布者；
键盘、自动控制和脚本统一向 `/control/cmd_vel` 发布，运行时只能启用一个上游控制源。ROS 的受控
停车不等同于实机硬件急停；硬件失效关闭链路和实机门限仍是项目外部待验事项。
