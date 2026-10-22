# 采集专项与历史 P2-06 复现

前置环境见[操作入口](../OPERATION.md#运行前准备)。

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
[MESA_SETUP](../MESA_SETUP.md)，本页不维护第二套安装命令。headless 相机采集同样需要渲染库。

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
“两档从未出现”；完整口径见[长任务时序稳定性](../LOCALIZATION_GAP_PLAN.md)的调查归档。

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

