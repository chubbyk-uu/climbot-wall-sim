# 安装、资产准备与测试

首次使用按本页安装，再读[快速启动](QUICKSTART.md)。

## 环境要求

| 组件 | 项目基线 |
| --- | --- |
| 系统 | Ubuntu 24.04 / WSL2 或原生 Ubuntu |
| ROS 2 | Jazzy Jalisco |
| Gazebo | Harmonic，`gz-sim 8.x` |
| Python | 3.12 |
| 构建 | C++17、colcon、rosdep |
| CUDA（可选） | Toolkit 12.8；已验证 `sm_120` |

先按所在系统选择图形环境；ROS、构建和运行命令共用，不要把 WSL 配置套到原生 Linux 上。

| 项目 | 原生 Ubuntu 24.04 | WSL2 中的 Ubuntu 24.04 |
| --- | --- | --- |
| 图形驱动 | Linux 主机的正常 GPU 驱动 | Windows 主机驱动 + WSLg |
| `gpu_backend:=auto` | 自动选择 `native` | 自动选择 `wsl_d3d12`；当前启动配置偏好 NVIDIA |
| Mesa 补丁 | 不需要这份 D3D12 专用补丁 | 默认必须有修复版 Mesa；缺失会报错，不静默回退 |
| headless 采集 | 关闭窗口，但相机仍需渲染 | 同样仍使用 D3D12，需要修复版 Mesa |
| 可选 CUDA | Linux 驱动 + Toolkit | Windows 驱动 + WSL 内 Toolkit；不安装 Linux 显示驱动 |

本机运行验收在 WSL2/NVIDIA 上完成；原生 Linux 使用独立的 native 启动路径，不能把本机
长任务结果当作原生 Linux 的验收。无窗口仿真可用 `headless:=true`，离线处理与 CPU 拼接则
不需要图形环境。当前墙面 DDS 放在数据根 `assets/`，旧参考贴图保留在 `textures/`，均不随 Git 分发；视觉任务按
[从源码生成资产](#4-从源码生成资产) 生成。

采集、预处理与拼接的大文件不进入仓库。默认写入**记录器所在主机**当前用户的
`$HOME/climbot_data`，并在首次归档时创建。要使用另一块数据盘或共享目录时，再设置一个持久化、
可写的绝对目录：`export CLIMBOT_DATA_ROOT=/your/chosen/data/root`。不要把真实主机路径写入
文档、注释或提交的结果。

## 安装与部署

以下 Linux 命令以已有 Ubuntu 24.04 为前提；全新 Windows 先完成第 1 节的 WSL 发行版安装。
示例位置为 `~/robot_ws/climbot_sim`。首次获取源码：

```bash
sudo apt update
sudo apt install -y git
mkdir -p ~/robot_ws
git clone https://github.com/chubbyk-uu/climbot-wall-sim.git ~/robot_ws/climbot_sim
cd ~/robot_ws/climbot_sim
```

已有仓库跳过克隆，不要覆盖现有工作区。接着按系统准备图形环境，再安装共同依赖并构建。

### 1. 准备对应系统的图形环境

**原生 Ubuntu：**先确保桌面和主机 GPU 驱动正常。NVIDIA 主机通过 Ubuntu 的驱动管理方式
安装匹配的 Linux 驱动；不要加载 WSL 的私有 Mesa，也不必设置 `CLIMBOT_MESA_PREFIX`。
launch 的默认 `auto` 会选原生后端；需要明确指定时使用 `gpu_backend:=native`。

**WSL2：**在 Windows 安装匹配的 GPU 驱动，并按
[微软 WSLg 指南](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps)确认 WSL2 和
Linux GUI 可用。全新 Windows 可先在管理员 PowerShell 运行
`wsl --install -d Ubuntu-24.04`，按提示重启并创建 Linux 用户；已有发行版则按需更新：

```powershell
wsl --update
wsl --status
```

更新后若需要重启 WSL，先结束正在运行的任务；`wsl --shutdown` 会停止所有 WSL 发行版。
在 WSL 内还须按 [Mesa 安装与验证](MESA_SETUP.md)准备共享修复库。本仓库已包含构建
脚本、补丁和版本锁；已有合格的共享安装可直接复用，不必重复编译。不要在 WSL 内安装 Linux
显示驱动，也不要把私有库的 `LD_PRELOAD` 写进全局 shell 配置。

按[微软的文件存储建议](https://learn.microsoft.com/en-us/windows/wsl/filesystems)，仓库和数据放在
WSL 的 Linux 文件系统，而不是 `/mnt/c`；大图读写尤其应避免跨文件系统。

### 2. 安装共同依赖

先按 [ROS 2 Jazzy 官方 Ubuntu 安装说明](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
配置 ROS 软件源并安装 `ros-jazzy-desktop`，再安装 Gazebo 桥接和工作区工具。
[Gazebo 官方兼容说明](https://gazebosim.org/docs/harmonic/ros_installation/)推荐 Jazzy + Harmonic，
`ros-jazzy-ros-gz` 会安装对应组件，不要混装其他 Gazebo 主版本：

```bash
sudo apt update
sudo apt install -y ros-jazzy-ros-gz python3-colcon-common-extensions python3-rosdep \
  mesa-utils build-essential curl wget unzip python3-pil python3-tifffile

cd ~/robot_ws/climbot_sim
source /opt/ros/jazzy/setup.bash
sudo rosdep init  # 首次执行；已初始化时可跳过
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy \
  --skip-keys python3-tifffile -y
```

Ubuntu 24.04 提供 `python3-tifffile`，但当前 rosdep 没有这个键的映射，所以先用 apt 安装，
再仅跳过这个键；其余依赖解析失败必须处理。Python 工具使用系统 Python，不需要 Conda。

### 3. 构建、加载和检查

以下路径以仓库放在 `~/robot_ws/climbot_sim` 为例，其他位置相应修改。构建并加载工作区：

```bash
source /opt/ros/jazzy/setup.bash
cd ~/robot_ws/climbot_sim
colcon build --symlink-install
source install/setup.bash
# 下面的离线命令使用这个变量；已有自定义值会被保留。
export CLIMBOT_DATA_ROOT="${CLIMBOT_DATA_ROOT:-$HOME/climbot_data}"
```

默认的起伏墙使用打过哈希回绕补丁的局部 ODE（不替换系统库）。新机器首次构建一次；
已有同路径且构建记录／库摘要匹配时直接复用：

```bash
sudo apt install -y cmake patch libccd-dev
python3 tools/build_private_ode.py \
  --prefix "$HOME/opt/climbot-ode-0.16.2-hashfix-ccd" --jobs 4
```

缺少该库时启动会报错并给出上述命令；`wall_surface:=flat` 的 M1 平墙仍用系统 ODE。
配方、校验和其它前缀见 [OPERATION](operations/M2_PHYSICS.md#m2-原生接触与局部-ode-修复)。

验证基础环境：

```bash
gz sim --versions
ros2 doctor --report | head -20
glxinfo -B
```

`glxinfo -B` 在有图形会话的终端执行。原生 Linux 检查是否使用预期 GPU；WSL 普通终端显示的
只是系统库渲染器，不能据此证明启动时加载了修复库，按 [Mesa 验证步骤](MESA_SETUP.md#验证是否真正生效)
检查。`nvidia-smi` 能看到 GPU 也不能代替图形渲染验证。

### 4. 从源码生成资产

机器人底盘、轮子、密封圈、风机、提手、相机和灯均由仓库中的配置与脚本生成，启动时自动
写入运行目录；不用下载私有 CAD，也不依赖开发机的照片目录或另两个机器人项目。
尺寸来源是 [vacuum_chassis.yaml](../src/climbot_description/config/vacuum_chassis.yaml)，
建模／外观脚本为 [vacuum_layout.py](../tools/vacuum_layout.py) 与
[vacuum_visuals.py](../tools/vacuum_visuals.py)，构建时安装到 `climbot_description`；
运行装配入口是 [vacuum_assets.py](../src/climbot_gazebo/climbot_gazebo/vacuum_assets.py)。

视觉采集还需本地生成墙面贴图。源素材是 ambientCG 的
[Concrete044D](https://ambientcg.com/view?id=Concrete044D)，选择 **8K-JPG**；素材为
[CC0](https://docs.ambientcg.com/license/)，约 540 MiB。
[下载脚本](../tools/fetch_wall_texture.sh)固定 URL 和 ZIP SHA-256，
只提取颜色图；下载失败或摘要不符会中止。诊断裂缝／涂鸦图集
`tools/assets/diagnostic_wall_decals_v2.png` 已随仓库分发，无需额外下载。

在仓库根目录依次执行，先生成局部去重／D4 随机变换底纹，再叠加诊断特征。
这是新采集任务的默认资产，输出到数据根，旧目录不会被覆盖：

```bash
cd ~/robot_ws/climbot_sim
export CLIMBOT_DATA_ROOT="${CLIMBOT_DATA_ROOT:-$HOME/climbot_data}"
WALL_SOURCE="$HOME/.cache/climbot/Concrete044D_8K-JPG"
bash tools/fetch_wall_texture.sh "$WALL_SOURCE"

python3 tools/bake_wall_texture.py \
  --source-dir "$WALL_SOURCE/maps" \
  --output-dir "$CLIMBOT_DATA_ROOT/assets/wall_local_v6" \
  --source-size-m 2.5 --region-m 10 8 --region-origin-m 0 0 \
  --scale-mm-per-px 0.25 --block-px 2048 --gutter-px 128 \
  --randomize --source-catalog local --patch-px 1000 --overlap-fraction 0.20 \
  --seam-feather-px 64 --reuse-radius-m 1.0 --max-source-overlap 0 \
  --candidates 96 --tolerance 0.10 --seed 20261018 --jobs 4 \
  --host-memory-budget-gb 12 --video-memory-budget-gb 3

python3 tools/create_diagnostic_wall.py \
  --base-manifest "$CLIMBOT_DATA_ROOT/assets/wall_local_v6/wall_texture.json" \
  --surface-config src/climbot_gazebo/config/m2_wall_surface.yaml \
  --output-dir "$CLIMBOT_DATA_ROOT/assets/wall_local_m2_v6"
```

最后一步按默认起伏墙的物理源（板缝、两处真实裂缝）绘制开口，并加上八幅仅为贴图的裂缝、涂鸦和修补；
加载时核对绘制源与物理源的哈希。M1 平墙复查若需要原诊断墙（画线填缝），另行生成：

```bash
python3 tools/create_diagnostic_wall.py \
  --base-manifest "$CLIMBOT_DATA_ROOT/assets/wall_local_v6/wall_texture.json" \
  --output-dir "$CLIMBOT_DATA_ROOT/assets/wall_local_diagnostic_v6" \
  --joint-profile filled --seed 20260827 --jobs 4
```

起伏墙拒绝这种填缝贴图。

`2.5 m` 是项目声明的源图物理尺度，素材本身没有标注实测尺度。生成墙面为
**10×8 m、40000×32000 像素、0.25 mm/px**，与现有拍照精度一致；每套 320 个 BC1 DDS
块及 `wall_texture.json`，每套约 1.1 GB。缓存源 ZIP、两套贴图及临时空间应预留至少 5 GB；
全尺寸烘焙的基础内存估计约 9.66 GB，另外还有编码进程与系统开销，建议给 WSL 至少
16 GiB 可用内存。脚本会检查预算；资源不足先减少 `--jobs` 或仅生成较小任务区，
不要通过降低采样精度来替代正式拍照资产。全墙显存估计约 2.14 GB，另需给相机和 GUI 留余量。

颜色图 SHA-256 应为
`8179451010485cdb9268356ee96de5dcdfc45845c98d1bea11ff267ae6e5649c`，生成清单记录
源哈希、随机种子、尺度、分块和诊断特征。时间、提交号以及运行中的相机噪声会不同，
不能要求新机器的整个清单或照片与历史验收逐字节一致。烘焙和诊断输出目录必须不存在；
已有资产直接复用，重生成时换新目录，工具拒绝覆盖。

新版采用 250 mm 裁块、50 mm 重叠、16 mm 羽化、1 m 局部来源排除，以及四个直角旋转和
镜像组合（D4 八方向）；变换在离线烘焙时确定，同一物理位置在不同照片中保持一致。
参数和适用范围见[当前协议](WALL_TEXTURE_PROTOCOL_CURRENT.md)，历史失败试验仍保留。
独立种子、整墙验证与适用范围见[墙面资产说明](WALL_TEXTURE_PLAN.md)，不在安装步骤重复历史性能数值。

采集入口 `coverage_mission.launch.py` 默认加载
`$CLIMBOT_DATA_ROOT/assets/wall_local_m2_v6/wall_texture.json`，并关闭 Gazebo 墙面网格；
未设置数据根时使用 `$HOME/climbot_data`。缺少资产会报错，须先运行上述生成命令。
显式 `wall_texture:=<清单路径>` 可选择其他资产；`wall_texture:=''` 可使用纯色墙。
单独运行 `climbot_wall.launch.py` 或规划预览 `coverage_sim.launch.py` 仍默认纯色墙。
旧 `textures/wall_025`／`wall_diagnostic_025` 仅用于历史数据复查，不用于新任务默认值。
生成资产和下载缓存不提交 Git。

### 可选：安装 CUDA 拼接后端

CUDA 只加速 `build_wall_mosaic` 的 hard-cut 融合；普通构建、ROS 2 在线节点和 CPU 拼接均不要求
CUDA，也不需要 CUDA OpenCV、Torch、Conda 或 cuDNN。本项目当前在 CUDA Toolkit `12.8`、
`sm_120` 上完成验收。先确认对应系统的 NVIDIA 驱动正常，再安装 Toolkit。

**WSL2 使用 WSL 软件源**（驱动仅安装在 Windows）：

```bash
cd /tmp
wget https://developer.download.nvidia.com/compute/cuda/repos/wsl-ubuntu/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
```

**原生 Ubuntu 24.04 x86_64 使用 Ubuntu 软件源**（主机须已有匹配的 Linux NVIDIA 驱动）：

```bash
cd /tmp
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
```

两种系统选上面对应的一段，再执行共同步骤：

```bash
sudo apt-get update
sudo apt-get install -y cuda-toolkit-12-8

export PATH=/usr/local/cuda-12.8/bin:$PATH
nvidia-smi
nvcc --version
```

WSL 不要安装会拉入 Linux 驱动的 `cuda`、`cuda-12-8` 或 `cuda-drivers` 元包；不要将链接用的
CUDA `stubs` 目录加入运行时库路径。仓库钉住 `cuda-toolkit-12-8` 是为了复现已验证环境；安装其他版本前先按
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
为默认测试并行度；当前用例数以 `colcon test-result` 为准。若以后再次出现低频失败，仍按真实竞态定位，不以
重跑通过作为关闭依据。

需要产品源码静态分析时：

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
bash tools/run_clang_tidy.sh --log /tmp/climbot-clang-tidy.log
```

CI 在每个 main 推送和 Pull Request 上执行构建、静态分析和八 worker 并行测试。批量回归与故障排查见
[OPERATION](OPERATION.md)。

`climbot_bringup` 的 `test_workspace_consistency` 守两条没有任何单个包能自查的约定：launch
文件里写出的每个 `executable` 必须真的被安装，以及每个 launch test 的 `ROS_DOMAIN_ID` 在整个
工作区内唯一。两类错误都能正常构建、也能通过所属包自己的测试，只在真正启动时才暴露。

