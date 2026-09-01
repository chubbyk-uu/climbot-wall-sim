# Climbot Sim

基于 ROS 2 Jazzy 与 Gazebo Harmonic 的垂直壁面爬壁机器人仿真。系统保留真实重力，以持续
法向吸附力贴墙；覆盖规划、直线闭环控制、位置触发采集、离线图像处理和全局墙面拼接均在本仓库
内完成。控制不依赖 Nav2，拼接算法不读取 Gazebo 真值或墙面纹理。

![Gazebo 中的墙面与机器人](docs/images/gazebo_wall.png)

## 你能做什么

- 在 Gazebo 中启动墙面、机器人、传感器、EKF 和 RViz；
- 点选矩形或等腰梯形任务区，执行横向或纵向弓字覆盖；
- 在正式扫描线上自动拍摄原始灰度图，把每张图连同曝光那一刻的融合相机位姿一起原子归档；
- 离线校验归档、平场校正、去畸变、匹配、全局位姿优化和硬切墙面拼接；
- 用诊断墙真值和原尺寸 tile 检查拼接的绝对偏差、接缝和缺陷细节。

A～G4、P1 和 P2 的仿真链路均已完成；P2-06 的门限已由三组数据推导、由一组独立数据检验，
并完成人工全分辨率复核。当前状态与实机边界见 [STATUS](docs/STATUS.md)。

## 环境要求

| 组件 | 已验证版本 |
| --- | --- |
| 系统 | Ubuntu 24.04 / WSL2 或原生 Ubuntu |
| ROS 2 | Jazzy Jalisco |
| Gazebo | Harmonic，`gz-sim 8.x` |
| Python | 3.12 |
| 构建 | C++17、colcon、rosdep |
| CUDA（可选） | Toolkit 12.8；已验证 `sm_120` |

GUI 在 WSL2 上需要 WSLg/GPU 图形支持；无 GUI 的采集、处理、拼接和测试可使用
`headless:=true`。墙面 DDS 贴图放在 `textures/`，由 `.gitignore` 排除；刚克隆下来的仓库如果要跑带贴图的
视觉任务，按 [墙面贴图](docs/OPERATION.md#墙面贴图) 自行生成。

采集、预处理与拼接的大文件不进入仓库。默认写入**记录器所在主机**当前用户的
`$HOME/climbot_data`，并在首次归档时创建。要使用另一块数据盘或共享目录时，再设置一个持久化、
可写的绝对目录：`export CLIMBOT_DATA_ROOT=/your/chosen/data/root`。不要把真实主机路径写入
文档、注释或提交的结果。

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
# 下面的离线命令使用这个变量；已有自定义值会被保留。
export CLIMBOT_DATA_ROOT="${CLIMBOT_DATA_ROOT:-$HOME/climbot_data}"
```

验证基础环境：

```bash
gz sim --versions
ros2 doctor --report | head -20
```

### 可选：安装 CUDA 拼接后端

CUDA 只加速 `build_wall_mosaic` 的 hard-cut 融合；普通构建、ROS 2 在线节点和 CPU 拼接均不要求
CUDA，也不需要 CUDA OpenCV、Torch、Conda 或 cuDNN。本项目当前在 CUDA Toolkit `12.8`、
`sm_120` 上完成验收。WSL2 先在 Windows 安装或更新 NVIDIA 驱动，**不要在 WSL 内安装 Linux
显示驱动**；然后在 WSL 内安装 Toolkit：

```bash
cd /tmp
wget https://developer.download.nvidia.com/compute/cuda/repos/wsl-ubuntu/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update
sudo apt-get install -y cuda-toolkit-12-8

export PATH=/usr/local/cuda-12.8/bin:$PATH
nvidia-smi
nvcc --version
```

仓库钉住 `cuda-toolkit-12-8` 是为了复现已验证环境；安装其他版本前先按
[NVIDIA 的 WSL CUDA 指南](https://docs.nvidia.com/cuda/archive/12.8.2/cuda-installation-guide-linux/index.html#wsl)
和[当前下载选择器](https://developer.nvidia.com/cuda-downloads)确认安装命令。构建 CUDA 扩展：

```bash
source /opt/ros/jazzy/setup.bash
cd ~/robot_ws/climbot_sim
CUDACXX=/usr/local/cuda-12.8/bin/nvcc \
  colcon build --symlink-install --packages-up-to climbot_mosaic \
  --cmake-args -DCLIMBOT_CUDA_ARCHITECTURES=120
source install/setup.bash
```

`120` 是本项目开发机 GPU 的 compute capability；其他 GPU 应改成 NVIDIA 公布的对应值。配置
阶段如果没有 CUDA compiler，本包仍会构建 CPU 后端，但 `--backend cuda` 会明确失败。

## 构建与测试

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --executor parallel --parallel-workers 8
colcon test-result --verbose
```

不要把高并发失败当成噪声。此前 `-j8` 暴露的域号冲突、目标句柄 abort、DDS 发现等待、
错误的等待期限和归档终态竞态都是真实缺陷。最后一项在修复前第 8 次完整运行复现；补上
确定性回归后，当时的完整套件以 `-j8` 连续 20 次全绿（每次 1222 tests，约 43 s），因此恢复
为默认测试并行度；当前套件为 1370 tests。若以后再次出现低频失败，仍按真实竞态定位，不以
重跑通过作为关闭依据。

需要产品源码静态分析时：

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
bash tools/run_clang_tidy.sh --log /tmp/climbot-clang-tidy.log
```

CI 在每个 main 推送和 Pull Request 上执行构建、静态分析和串行测试。批量回归与故障排查见
[OPERATION](docs/OPERATION.md)。

`climbot_bringup` 的 `test_workspace_consistency` 守两条没有任何单个包能自查的约定：launch
文件里写出的每个 `executable` 必须真的被安装，以及每个 launch test 的 `ROS_DOMAIN_ID` 在整个
工作区内唯一。两类错误都能正常构建、也能通过所属包自己的测试，只在真正启动时才暴露。

## 快速启动

以下命令均假定已经完成构建，并在一个终端执行过：

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/climbot_sim/install/setup.bash
```

### 1. 只看仿真

想先确认机器人、墙面和传感器是否正常，运行：

```bash
ros2 launch climbot_gazebo climbot_wall.launch.py
```

没有图形界面时，改用无窗口模式：

```bash
ros2 launch climbot_gazebo climbot_wall.launch.py headless:=true
```

### 2. 规划预览

想先画出覆盖路径但不让机器人动，运行：

```bash
ros2 launch climbot_bringup coverage_sim.launch.py
```

随后在 RViz 选择 `Publish Point`，点击任务区域：矩形点左下和右上两个角；等腰梯形再点右下角。
所有点都必须在绿色安全框内。此模式只显示路径，不会控制机器人。

### 3. 规划、控制与自动采集

想让机器人执行路径并保存巡检照片，先设置数据保存位置，再启动完整任务：

```bash
ros2 launch climbot_bringup coverage_mission.launch.py \
  wall_grid_spacing:=0
```

在 RViz 的 **Coverage Task** 面板中完成点选后按 **Replan**，确认 `Capture` 页显示的是管理器从
launch 参数、`CLIMBOT_DATA_ROOT` 或 `$HOME/climbot_data` 解析出的保存目录，再按 **Start**；手动
修改该输入框只覆盖从这个面板启动的任务，不修改环境变量。机器人只会在直线扫描段拍照。完成后，
保存目录下会出现一个新的任务目录，
其中包含原始图像、每张图的拍摄位姿、相机标定和 `manifest.json`。想让它停一下再接着跑，按
**Pause**，恢复按 **Resume**——任务和归档都不会因此结束；确定不跑了才按 **Stop**。

![Gazebo 与 RViz 中同步执行的覆盖任务](docs/images/coverage_run.gif)

左边是 Gazebo，右边是 RViz；机器人正在跑一条梯形区的横向弓字路径，面板同步显示段进度和已保存
的照片数。

![RViz 中执行覆盖任务](docs/images/rviz_coverage_task.png)

绿色虚线是安全工作区，橙色是选定任务区，蓝线是机器人中心路径，黄色带是相机预测足迹。这张是
纵向扫描（上面的动图是横向），任务正处于 **Paused**：段进度、任务号和归档 run 都还在，Resume
会从当前位姿继续同一段。

改成纵向扫描：

```bash
ros2 launch climbot_bringup coverage_mission.launch.py sweep_direction:=vertical
```

无 GUI 批处理、诊断墙贴图和 realistic 定位属于实验用法，见
[OPERATION 的启动变体](docs/OPERATION.md#启动变体)。

### 4. 离线图像处理

照片采集完成后，这一步对原图做校验、平场校正和去畸变，生成拼接可以读取的新目录。`RAW_RUN` 是
上一步生成的任务目录；`PROCESSED_RUN` 必须是一个还不存在的新目录：

```bash
RAW_RUN="$CLIMBOT_DATA_ROOT/<task-id>/<run-id>"
PROCESSED_RUN="$CLIMBOT_DATA_ROOT/processed-<new-id>"

ros2 run climbot_image_processing process_inspection_archive \
  --input-run "$RAW_RUN" \
  --output-dir "$PROCESSED_RUN" \
  --flat-field-file "$CLIMBOT_DATA_ROOT/calibration/<flat-field>.npz" \
  --denoise none --jobs auto --memory-budget-gb 4
```

原始照片不会被修改。命令成功后，在 `PROCESSED_RUN` 中得到校正后的图像和对应标签。平场文件
不可用且明确不做校正时，把 `--flat-field-file ...` 换成 `--no-flat-field`；完整参数见
[image processing README](src/climbot_image_processing/README.md)。

### 5. 离线墙面拼接

这一步把多张照片拼成一张墙面图。通常把横向和竖向两次采集一起输入：`RUN_H`、`RUN_V` 是上一步
得到的两个 processed 目录；`ROOT` 是本次拼接的目录前缀，下面每一步都在它后面加一个不同的
后缀，作为自己的输出目录。

按顺序执行下面五步。某一步报错时，先看该步输出的错误，不要跳到后面继续运行。

| 步骤 | 命令 | 它在做什么 | 成功后得到什么 |
| --- | --- | --- | --- |
| 0 | `validate_mosaic_inputs` | 检查照片、标签和标定是否齐全、是否匹配 | 终端里的检查结果；不创建目录 |
| 1 | `build_overlap_candidates` | 找出哪些照片拍到了同一块墙 | 候选照片对列表 |
| 2 | `build_local_matches` | 比较重叠部分，估计两张照片之间的细小偏移 | 照片之间的匹配结果；临时缓存可删除 |
| 3 | `build_pose_graph` | 把所有照片的偏移一起调整，让整面墙尽量对齐 | 调整后的照片位置与质量报告 |
| 4 | `build_wall_mosaic` | 把调整前和调整后的照片各拼成一张完整墙面图 | 两张 BigTIFF、预览图、覆盖图和报告 |

第 0、1 两步的产物是给人查阅的记录，后面的命令并不读取它们：第 2 步会用同一套代码自己重算
一遍候选。照做一遍的好处是出了问题能分步定位，也留下可复查的证据。

`build_initial_projection` 同样是可选的检查命令：它画出每张照片预计落在墙上的范围，适合
先看相机方向、比例和任务区域是否合理。正常拼接不必单独运行，因为第 1 步会做同样的计算。

```bash
RUN_H="$CLIMBOT_DATA_ROOT/processed-<horizontal-id>"
RUN_V="$CLIMBOT_DATA_ROOT/processed-<vertical-id>"
ROOT="$CLIMBOT_DATA_ROOT/mosaic-<new-id>"

ros2 run climbot_mosaic validate_mosaic_inputs \
  --input-run "$RUN_H" --input-run "$RUN_V"

# 第 1 步：找出拍到同一块墙的照片对。
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
  --resolution-mm-per-pixel 0.25 --jobs auto --memory-budget-gb 4 \
  --preview-max-side-px 4096 --backend cuda
```

上面的命令是本项目 CUDA 开发机的标准拼接测试方式；运行后应检查
`mosaic_manifest.json` 中 `execution.backend.effective` 为 `cuda`。CLI 默认仍是 `--backend cpu`，
因此普通 ROS 2 机器可把最后一个参数改为 `--backend cpu`。`--backend auto` 会在 CUDA 不可用或运行失败时清理
临时目录并从头改跑 CPU，但不会用回退掩盖输入、哈希或共享磁盘错误。CUDA 只加速第 4 步的
hard-cut 融合，不改变前面的特征匹配和位姿图。

最终重点看两个文件：`mosaic_pose_only.tif` 是直接按拍摄位姿拼出的结果；
`mosaic_optimized.tif` 是经过第 3 步对齐后拼出的结果。两者使用同一批照片，因此可以直接比较。
`coverage_count.tif` 中为零的位置表示相机从未拍到，那些区域不能当作有效墙面图。需要检查诊断墙时，
继续运行[真值评价与原尺寸检查](docs/OPERATION.md#诊断墙后验检查)。

[![pose-only、optimized 与两者差值的三联对照](docs/images/mosaic_comparison.jpg)](docs/images/mosaic_comparison_full.jpg)

左起分别是 pose-only、optimized 和两者的绝对差值。点开可看原尺寸（7887 × 4096）。差值图上仍然
发亮的位置就是全局优化真正改动过的地方，其余接近全黑说明两条路径在那里本来就一致。

## 文档导航

| 需要了解 | 唯一入口 |
| --- | --- |
| 文档职责、归档和写作边界 | [docs/README.md](docs/README.md) |
| 项目目标、范围、硬约束与规范验收 | [PROJECT_GUIDE.md](PROJECT_GUIDE.md) |
| 包职责、依赖和数据流 | [ARCHITECTURE](docs/ARCHITECTURE.md) |
| 话题、服务、Action、参数、文件格式 | [INTERFACES](docs/INTERFACES.md) |
| 批量回归、参数变体与故障处置 | [OPERATION](docs/OPERATION.md) |
| 当前验收状态与正式证据 | [ACCEPTANCE](docs/ACCEPTANCE.md) |
| 当前项目状态、风险与下一步 | [STATUS](docs/STATUS.md) |
| 离线拼接设计与门禁 | [MOSAIC_PLAN](docs/MOSAIC_PLAN.md) |
| 隐蔽故障、偶发竞态与排查经验 | [INCIDENTS](docs/INCIDENTS.md) |
| 结果有效性、基线与重生成入口 | [results/README.md](results/README.md) |
| 本地大数据保留与清理规则 | [DATA_RETENTION](docs/DATA_RETENTION.md) |

单个包的参数、边界和测试命令写在各自的 README：
[bringup](src/climbot_bringup/README.md)、
[common](src/climbot_common/README.md)、
[control](src/climbot_control/README.md)、
[coverage](src/climbot_coverage/README.md)、
[description](src/climbot_description/README.md)、
[gazebo](src/climbot_gazebo/README.md)、
[image_processing](src/climbot_image_processing/README.md)、
[inspection](src/climbot_inspection/README.md)、
[interfaces](src/climbot_interfaces/README.md)、
[mosaic](src/climbot_mosaic/README.md)、
[rviz_plugins](src/climbot_rviz_plugins/README.md)。

## 安全边界

Gazebo DiffDrive 会持续执行最后收到的速度命令。系统以速度看门狗作为 `/cmd_vel` 的唯一发布者；
键盘、自动控制和脚本统一向 `/control/cmd_vel` 发布，运行时只能启用一个上游控制源。ROS 的受控
停车不等同于实机硬件急停；硬件的失效关闭链路和实机门限都在本项目范围之外，尚待验证。

## 许可

Apache License 2.0，见 [LICENSE](LICENSE)。
