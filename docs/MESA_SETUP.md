# WSL D3D12：Mesa 修复库的安装与验证

本页只适用于 **Ubuntu 24.04 x86_64 / WSL2 + WSLg / D3D12**。原生 Linux 使用正常的
主机图形驱动，不需要这份补丁；只做离线图像处理或 CUDA 拼接也不需要它。共同安装步骤见
[README](../README.md#安装与部署)。

## 为什么需要补丁

Mesa 的 D3D12 命令签名缓存查找把 `key` 错传为 `&key`，造成缓存查找异常、重复创建命令
签名，长期运行可能持续占用内存。本仓库的
[源码补丁](../tools/patches/mesa-d3d12-command-signature-key.patch)修正这一处参数。
它不改变机器人控制、物理步长或图像处理算法，也不是所有 GPU 卡顿的通用修复。

修复库以 Ubuntu Mesa `25.2.8-0ubuntu0.24.04.2` 为基线，保留发行版补丁，再叠加上述修复。
仅构建 D3D12，不包含 llvmpipe 软件驱动。Climbot 在 WSL 上默认选择它；**headless 只是
关闭窗口，相机离屏渲染仍需要它**。

## 已有共享安装：直接复用

默认安装前缀是 `$HOME/opt/agv-mesa-25.2.8/install`。这是公共图形运行库目录，多个工作区
可以共用，不放在采集数据根或仓库 `build/` 内，也不应随数据清理删除。

其他安装位置在当前终端指定：

```bash
export CLIMBOT_MESA_PREFIX="$HOME/opt/mesa-shared/install"
```

前缀解析顺序为 `CLIMBOT_MESA_PREFIX` → `AGV_MESA_PREFIX` → 默认前缀；模式由 launch 参数
`mesa` 或 `CLIMBOT_MESA` 指定，默认 `private`。路径不能包含空白或冒号。

完整安装至少包含：

```text
<构建根>/
  build-result.json                 版本及库 SHA-256
  install/
    lib/libgallium-25.2.8.so
    lib/libgbm.so.1
    lib/libGLX_mesa.so.0
    lib/libEGL_mesa.so.0
    lib/dri/
    lib/gbm/
    share/glvnd/egl_vendor.d/50_mesa.json
```

`CLIMBOT_MESA_PREFIX` 指向 `install/`，不是构建根。分发已有库时须保留目录结构、软链接和
相邻的 `build-result.json`；只有一个 `.so` 或只有 `install/` 不够。它仍依赖目标主机系统库，
不能视为跨发行版的通用二进制包。

## 新机器：从本仓库构建

需要的构建脚本、源码补丁和版本/哈希锁均在本仓库 `tools/` 内，不依赖其他工作区。先安装
基础开发包；这些是编译依赖，不是用私有库替换系统 Mesa：

```bash
sudo apt-get update
sudo apt-get install -y build-essential ninja-build pkg-config dpkg-dev patch python3 \
  libglvnd-dev zlib1g-dev libzstd-dev libexpat1-dev libdrm-dev libudev-dev \
  libelf-dev libunwind-dev libwayland-dev libx11-dev libxext-dev libx11-xcb-dev \
  libxxf86vm-dev libxrandr-dev x11proto-dev spirv-tools \
  libxcb-glx0 libxcb-shm0 libxcb-shape0 libxcb-dri2-0 libxcb-dri3-0 \
  libxcb-randr0 libxcb-present0 libxcb-sync1 libxcb-xfixes0 libxcb-render0 libxshmfence1
```

在仓库根目录运行；不需要先构建 ROS 工作区：

```bash
python3 tools/build_private_mesa.py --jobs 8
```

脚本按 [版本锁](../tools/patches/mesa-build-lock.json)下载源码及固定版本的私有开发依赖，
逐项校验 SHA-256，解包 Ubuntu 源码及发行版补丁，再应用本项目补丁。默认构建根为
`$HOME/opt/agv-mesa-25.2.8`，安装到其 `install/`。整个构建阶段不使用 sudo，不安装系统包，
不改系统 Mesa 或全局环境；完整日志写在构建根的 `build.log`。成功后生成
`build-result.json` 并检查共享库依赖。

已有构建根会被拒绝，不覆盖或自动删除。需要重建时使用新目录，然后切换前缀：

```bash
python3 tools/build_private_mesa.py --root "$HOME/opt/mesa-rebuild-25.2.8" --jobs 8
export CLIMBOT_MESA_PREFIX="$HOME/opt/mesa-rebuild-25.2.8/install"
```

可追加 `--cache "$HOME/opt/agv-mesa-25.2.8"` 复用旧根下的 `downloads/` 和 `deps/`，缓存仍
逐项校验。构建根会写入 `COLCON_IGNORE`，防止 colcon 误发现 Mesa 内部构建目录。

下载需要网络或完整且合格的缓存；APT 固定版本若被镜像移除，脚本明确失败，不自动换版。
此时应核对版本锁和缓存，不要为了绕过错误删除校验。源码签名文件校验的是已锁定哈希，脚本
不声称验证维护者 PGP 签名。基础系统开发包未全部钉住，因此不承诺跨机器二进制逐位相同。

## 启用范围与切换

| 配置 | 行为 |
| --- | --- |
| WSL 的 `gpu_backend:=auto` 或 `wsl_d3d12`，`mesa:=private` | 默认路径；校验并加载私有修复库 |
| 原生 Linux 的 `gpu_backend:=auto` 或 `native` | 不注入这份私有库 |
| `gpu_backend:=software` | 使用系统 llvmpipe 做诊断对照，资源占用通常更高 |
| `mesa:=system` | D3D12 显式使用未修复的系统库，打印警告，仅用于对照 |

缺库、缺构建记录、版本不符或 Gallium SHA-256 不匹配时，默认 D3D12 启动会在渲染场景前
报错。不要把 `mesa:=system` 当成修复缺库的方法，也不要改写摘要假装校验通过。

launch 为子进程树设置库搜索路径，并预加载配套 Gallium、GLX、EGL、GBM；Gazebo 服务端、
GUI 和 RViz 都应使用同一修复库。不渲染的子进程也继承这些设置，但不改变其他终端或系统库。
**不要在 `.bashrc` 全局设置私有 `LD_PRELOAD` 或 `LIBGL_DRIVERS_PATH`**；仅设置
`LD_LIBRARY_PATH` 也不能保证替换了系统 Gallium 的不同 SONAME。

私有库不能退回软件渲染，D3D12 初始化失败就会失败。软件对照应从未手工预加载私有库的干净
终端运行 `gpu_backend:=software`。CUDA 是另一条计算链，装好 Mesa 不等于装好 CUDA。

## 验证是否真正生效

构建完成只证明编译和依赖检查通过，不代表 GUI 或长任务验收完成。在已经加载 ROS 和
Climbot 工作区的 WSL 图形终端中，先验证安装完整性，并用相同库探测渲染器：

```bash
python3 - <<'PY'
import os
import subprocess
from climbot_gazebo.mesa_runtime import default_prefix, environment, validate

prefix = default_prefix()
print(validate(prefix), flush=True)
env = environment('private', prefix, os.environ)
env.update(GALLIUM_DRIVER='d3d12', MESA_D3D12_DEFAULT_ADAPTER_NAME='NVIDIA')
subprocess.run(['glxinfo', '-B'], env=env, check=True)
PY
```

预期是 `D3D12` 和目标 GPU，而不是 llvmpipe。NVIDIA 适配器设置只是偏好，不是硬件身份
的证明；其他 GPU 未在本项目完成同等验收。`glxinfo` 来自 README 中安装的 `mesa-utils`。

随后按 README 启动仿真，启动日志应包含 `Mesa: patched D3D12 Gallium sha256=...`。
确认实际进程时，使用 Gazebo **服务端、GUI 和 RViz 各自的 PID**（不是 `ros2 launch`、
Ruby 启动器或 supervisor 的 PID），逐个执行：

```bash
# 将 12345 换成实际渲染进程 PID；headless 时没有 GUI/RViz 可供检查。
RENDER_PID=12345
rg 'libgallium|libGLX_mesa|libEGL_mesa|libgbm' "/proc/$RENDER_PID/maps"
```

映射路径应指向所选前缀，不能同时加载另一份系统 Gallium。普通终端的 `glxinfo`、启动日志
或 `nvidia-smi` 单独一项都不能证明 Gazebo 真正加载了补丁；进程映射才是实际选库证据。
新主机还需要短时相机采集和长任务检查，不能直接沿用本机的稳定性结论。

## 常见问题

| 现象 | 处理 |
| --- | --- |
| `Patched Mesa is missing` | 按本页构建或恢复完整共享安装；核对前缀是否指向 `install/` |
| `build record is missing or invalid` | 恢复同次构建的 `build-result.json`，或在新根重建 |
| `does not match its build record` | 库与记录不是同一构建；不要手改 SHA-256 来绕过检查 |
| `Build root already exists` | 已有合格库直接复用；重建指定新的 `--root` |
| 下载、固定版本或 SHA-256 失败 | 看 `build.log` 和终端错误，核对网络、APT 索引、版本锁及缓存 |
| D3D12 初始化失败、黑屏或实际走软件渲染 | 查 Windows 驱动、WSLg、渲染器和进程映射；不要把 headless 当成禁用 GPU |

Git 只保存脚本、补丁和版本锁，不提交下载包、源码树、构建库、机器日志、代理配置或凭据。
