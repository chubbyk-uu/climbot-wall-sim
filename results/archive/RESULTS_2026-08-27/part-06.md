# 历史分册 6

#### 纹素精度：纯匹配指标由 `0.26` 胜出，`0.50` 仍是工程工作点

第一次让三档各自重新 quilting 时，三张墙的混凝土内容不同，`0.75` 六点内点数甚至从
`376` 跨到 `5234`；这把“抽到的墙面内容”和“纹素精度”混在一起，结果未归档。正式
实验先烘焙一份 `0.26 mm/px` canonical，再从同一 BC1 mip 0 用 LANCZOS 生成
`0.50/0.75`，所以三档的内容、接缝、seed 和墙面坐标一致，只改变纹素尺度。六个位置的
结果见 [wall_texture_resolution_2026-08-21_summary.json](../../wall_texture_resolution_2026-08-21_summary.json)：

| 精度 | 局部产物相对显存 | RANSAC 内点 最小／中位／最大 | 内点率中位 | 描述子余量中位 |
| ---: | ---: | ---: | ---: | ---: |
| **`0.26 mm/px`** | `1.000` | **`1995 / 3958 / 5114`** | `0.904` | `0.222` |
| `0.50 mm/px` | `0.257` | `1049 / 3807.5 / 5444` | **`0.958`** | `0.210` |
| `0.75 mm/px` | `0.118` | `385 / 2132 / 4782` | `0.856` | **`0.236`** |

真实渲染否定了“`0.50` 是纯匹配最优点”：`0.26` 的内点中位数高 `4.0%`，六个位置中
五个不低于 `0.50`，低特征位置的最小值则高 `90%`。源图模拟中 `0.26` 只有约
`146` 个内点，是因为近似重采样没有复现 Ogre2 的纹理过滤、mip、BC1 和相机成像链；
那组 `11×` 差距不得继续用于选型。

但正式整墙**保持 `0.50 mm/px`**。理由从“匹配最优”改为“资源与充分性折中”：
`0.26` 的纹理显存和产物约是 `0.50` 的 `3.9×`，而 `0.50` 的最弱画幅仍有 `1049`
个几何内点、内点率中位 `95.8%`；下面的整墙检索也有很大正确位置余量。将来若真实相机
在低纹理墙段达不到这一余量，可以有数据地切到 `0.26`，但当前没有理由为 `4%` 的中位
提升长期承担近四倍资源。

边界：coarse 两档从已 BC1 编码的 canonical 解码后再压缩，多走一次有损编码；此前同
精度 BC1/RGBA8 实测差异落在种子波动内，因此不影响当前数量级结论，但如果未来两档差异
缩到噪声范围，应从未压缩 canonical 重做。

#### `Concrete044D` 整墙位置可区分性

在正式 `10×8 m @ 0.50 mm/px` 墙上均匀取 `5×4=20` 个参考位置，每个扰动查询与全部
20 个参考画幅匹配，而不是只与自己的 A 帧做局部配对。结果见
[wall_texture_global_2026-08-21_summary.json](../../wall_texture_global_2026-08-21_summary.json)：

| 指标 | 结果 |
| --- | ---: |
| Top-1 正确位置 | **`20/20 = 100%`** |
| 正确位置内点 最小／中位／最大 | `1461 / 3557.5 / 5287` |
| 最强错误位置内点 最小／中位／最大 | `6 / 1478.5 / 3112` |
| 正确减最强错误的内点余量 最小／中位／最大 | **`508 / 1206.5 / 4732`** |

因此 `044D` 大尺度结构少和源图约 `12.8×` 复用确实会产生强错误候选，但当前 quilting
没有造成位置歧义：最弱画幅仍以 508 个 RANSAC 内点的余量选中真实位置。该结论只覆盖
当前 seed、当前整墙与规定扰动；它证明仿真场景可用于第一版图像定位测试，不等于真实
墙面天然具有同样的全局可区分性。

### 分块接缝修复（2026-08-21）

近距离 GUI 检查发现，名义上无空洞、无重叠的 80 个 visual 仍可能在各自独立的纹理
过滤和 mipmap 边缘形成细线。定案烘焙为每块携带 `128 px` 真实邻域，visual 按该采样
范围重叠；边框宽度是 `4 × 2^5`，所以 BC1 的 `4×4` 编码相位经过五级 mip 仍一致。
抽查横、纵及边缘相邻块，mip `0~5` 的重叠像素均为 `max_delta = 0`。

同时比较了 `1024 / 1536 / 2048 px` 三档 quilting patch：`1024` 的结构变化较密，
`2048` 会放大显眼特征的重复，最终选择 `1536 px`、`25%` overlap，并只在最小误差
裁切附近做 `96 px` 羽化。最终产物是 `252 MiB`、252 个 quilting patch；D3D12 Gazebo
整墙和近景均未再看到规则 DDS 直线或矩形硬切边。该检查只回答渲染接缝；画幅尺度
全局位置可区分性与渲染相机精度复测已由上面的 2026-08-21 正式结果另行闭合。

产物不进仓库：源素材 `540 MB`、烘焙产物 `252 MiB`，仓库里只有
`tools/fetch_wall_texture.sh`（含源素材 `sha256`）、`tools/bake_wall_texture.py`
和清单里记录的种子 `20260820`。命令见
[docs/OPERATION.md](../../../docs/OPERATION.md) 的「墙面贴图」。

## 重新生成覆盖基线

```bash
tools/run_coverage_regression.sh                    # 全八工况，四路并行，标签用今天日期
tools/run_coverage_regression.sh -j 6               # 六路
tools/run_coverage_regression.sh -t 2026-08-19      # 指定标签
tools/run_coverage_regression.sh -m distance        # 位置控制（默认 time）
tools/run_coverage_regression.sh -s 4242 -i 1717    # 换全站仪/IMU 噪声种子
tools/run_coverage_regression.sh bigV bigTH         # 只跑指定工况
tools/run_coverage_regression.sh -k                 # 保留未压缩轨迹
```

该脚本显式传入 `inspection_geometry_profile:=configured`，因此历史八工况严格使用各自
YAML 的冻结检测几何（`0.01 m` 长度、零前向偏移、`20%` 横向重叠），可重生成已归档的
2026-08-20 基线。实际相机巡检使用标准 launch 的 `calibrated` profile；其物理相机几何
不得倒灌到历史控制回归。

脚本从新启动的无界面世界执行每个工况，写出 `results/coverage_<工况>_<标签>_*`，
末尾打印验收汇总表；**任一工况不达标返回退出码 1** 并列出工况名，可直接进 CI。
八个工况四路并行约 11 分钟，串行约 70 分钟。

**跑正式基线前先把 `src/` 提交干净。** 脚本开跑前检查工作树，非干净时会在标签后
自动加 `-dirty` 并打印醒目告警，评价器也会把 `provenance.git.traceable` 写成
`false` 并在每个工况结束时以 WARNING 打印。带 `-dirty` 的结果只能当过程记录，不能
作为验收证据引用——这条规矩是补出来的，`2026-08-18d` 和 `timeE` 就是没有它的后果。

中途被打断的运行不会污染归档：被杀的工况自己把 `completed=false` 和
`failure_reason` 写进摘要，重跑同一标签会原地覆盖。

### 并行需要两层隔离，不是一层

`ROS_DOMAIN_ID` 只隔离 DDS。Gazebo 走的是 **gz-transport**，一套独立的发现机制，
两个实例的话题路径（`/world/climbot_wall/model/climbot/...`）完全相同，不设
`GZ_PARTITION` 就会让桥接订阅到另一个世界。**这个故障不会崩溃**——数据看起来正常，
只是来自别人的仿真，比直接报错危险得多。脚本给每路配一组
`ROS_DOMAIN_ID` + `GZ_PARTITION`，收尾时读每个进程自己的 `/proc/<pid>/environ`
匹配分区来定位，一条路杀不到另一条路。

隔离验证：同一工况并行与串行的**覆盖率逐位相同**（`99.93%` / `99.74%`），这是整条
轨迹在 10 mm 栅格上的积分，任何串台都会立刻体现。

并行不降速：瓶颈是 1 ms 物理步长的单线程串行，四个实例各占约 70% 的一个核，实时
因子 `0.85~0.94`，与单独运行的 `0.84~0.94` 一致。24 核机器上还有余量再加路。

### 逐工况手动运行

需要单独调试某个工况时，四个终端分别启动：

```bash
# 终端 1：仿真、TF、传感器、EKF 和速度看门狗
ros2 launch climbot_gazebo climbot_wall.launch.py use_sim_time:=true headless:=true

# 终端 2：规划器（工况之间只有这一行的配置和 sweep_direction 不同）
ros2 launch climbot_coverage coverage_planner.launch.py \
  use_sim_time:=true rviz:=false \
  config_file:="$(pwd)/src/climbot_coverage/config/coverage_horizontal_demo.yaml" \
  inspection_geometry_profile:=configured \
  input_mode:=parameters region_type:=rectangle sweep_direction:=horizontal

# 终端 3：跟踪器和管理器
ros2 launch climbot_control coverage_executor.launch.py use_sim_time:=true

# 终端 4：发送 Action 并按真值评价
ros2 run climbot_gazebo evaluate_coverage_execution.py --ros-args \
  -p use_sim_time:=true -p case:=planned_task \
  -p startup_timeout_s:=30.0 -p execution_timeout_s:=600.0 \
  -p trajectory_csv:=results/coverage_horizontal_manual_trajectory.csv \
  -p summary_json:=results/coverage_horizontal_manual_summary.json
```

八个工况的配置与扫描方向：

| 工况 | `config_file` | `region_type` | `sweep_direction` |
| --- | --- | --- | --- |
| 矩形横向 | `coverage_horizontal_demo.yaml` | `rectangle` | `horizontal` |
| 矩形竖向 | `coverage_vertical_demo.yaml` | `rectangle` | `vertical` |
| 梯形横向 | `coverage_trapezoid_horizontal_demo.yaml` | `trapezoid` | `horizontal` |
| 梯形竖向 | `coverage_trapezoid_vertical_demo.yaml` | `trapezoid` | `vertical` |
| 大矩形横向 | `coverage_horizontal_large.yaml` | `rectangle` | `horizontal` |
| 大矩形竖向 | `coverage_vertical_large.yaml` | `rectangle` | `vertical` |
| 大梯形横向 | `coverage_trapezoid_horizontal_large.yaml` | `trapezoid` | `horizontal` |
| 大梯形竖向 | `coverage_trapezoid_vertical_large.yaml` | `trapezoid` | `vertical` |

必须用 `planner_config_file` / `control_config_file` 这类互不冲突的名字，或者像上面
这样分别启动：被包含的 launch 会继承父作用域的同名参数，一个 `config_file` 会同时
落到规划器和跟踪器上，使后者静默退回内置默认值（包括侧滑补偿为 `0`）。

评价器在任一验收项不达标时返回失败，但无论成功、超时还是异常都会先写出 CSV 和
摘要；失败运行的摘要里 `completed=false` 并记录 `failure_reason`，不要把这类文件
当作基线。

## §14.4 侧滑补偿专项验收

2026-08-17 在**同一个**无界面仿真、同一段墙面上依次执行两个阶段，提交
`515e5e3`、工作树干净。两个阶段的速度（`0.15 m/s`）、名义直线长度（`1.20 m`）、
吸附力和 WheelSlip 参数完全相同，差别只有横轨闭环的开关。

| 指标 | 补偿关闭 | 补偿开启 | §14.4 要求 |
| --- | ---: | ---: | --- |
| 三次净高度误差 | `-122.1 / -121.2 / -121.0 mm` | `-14.5 / +3.8 / -0.6 mm` | — |
| 三次实测直线长度 | `1.169 / 1.169 / 1.169 m` | `1.201 / 1.201 / 1.199 m` | — |
| 平均净高度误差绝对值 | `121.43 mm` | `6.29 mm` | 降低 ≥ 70% → **94.82%** |
| 平均下降/前进比 | `10.387%` | `0.313%` | 归一化降幅 **96.99%** |
| 三次下降比 | `10.45 / 10.36 / 10.35%` | — | 变异系数 ≤ 5% → **0.41%** |
| 漂移方向 | 三次全部向下 | — | 稳定、可重复的向下漂移 ✔ |
| 平均机体航向偏角 | `+0.45°` | `+6.13°`（向上） | 允许小幅向上偏角 ✔ |
| 位置轨迹相对水平线的倾角 | — | `0.69 / 0.18 / 0.03°` | 位置比机体朝向更贴近规划直线 ✔ |
| 可见反向往复（`20 mm` 门限） | — | `0` | 无明显蛇形 ✔ |

六条要求全部满足。最后两行是这一节的关键证据：补偿开启后机体持续向上偏
`6.13°`，而**实际走出来的轨迹**相对水平线只倾斜 `0.03~0.69°`——机器人是"斜着身
子走直线"，正是 §10.4 期望的形态，而不是把整条线走歪。

补偿关闭态与 2026-08-13 的独立标定基线互相印证：下降比 `10.39%` vs `10.56%`，
变异系数 `0.41%` vs `0.64%`。两者用不同脚本、不同仿真实例测得。本次实验中该阶段
还被完整重复了三遍（三次独立仿真给出 `10.39 / 10.39 / 10.38%`，变异系数
`0.41 / 0.34 / 0.41%`）。

**第一条扫描线不计入**：掉头对准会带来约 `85 mm` 下坠，§10.7 把参考线平移到实际
位置而不是爬回名义线，因此第一条扫描线的起始偏差必然超过
`parallel_scan_offset_m`（`45 mm`），跟踪器改用一次前进小弧线入轨，吃掉约
`0.33 m`。它照常执行但不参与统计；其后三条扫描线之间没有转向，是与开环段可比的
稳态直线。加长引入段无效，实测 `0.5 m` 和 `1.2 m` 都仍把 `81~87 mm` 交给扫描段。

## 重新生成 §14.4 配对实验

两个阶段必须在**同一个**仿真里依次运行，而且 `open_loop` 阶段不能有
`line_tracker`：空闲的跟踪器同样以 `50 Hz` 在 `/control/cmd_vel` 上发布零速，会
覆盖开环指令。速度看门狗由 `climbot_wall.launch.py` 启动，`/cmd_vel` 的安全门与
正常任务一致。

```bash
# 终端 1：仿真、传感器、EKF 和速度看门狗，不含跟踪器
ros2 launch climbot_gazebo climbot_wall.launch.py \
  use_sim_time:=true headless:=true

# 终端 2，阶段一：补偿关闭
ros2 run climbot_gazebo evaluate_slip_compensation.py --ros-args \
  -p use_sim_time:=true -p mode:=open_loop \
  -p trajectory_csv:=results/slip_compensation_off_2026-08-17_trajectory.csv.gz \
  -p summary_json:=results/slip_compensation_off_2026-08-17_summary.json

# 阶段一结束后再启动跟踪器和管理器
ros2 launch climbot_control coverage_executor.launch.py use_sim_time:=true

# 终端 2，阶段二：补偿开启，并按关闭态摘要计算降幅
ros2 run climbot_gazebo evaluate_slip_compensation.py --ros-args \
  -p use_sim_time:=true -p mode:=compensated \
  -p reference_summary_json:=results/slip_compensation_off_2026-08-17_summary.json \
  -p trajectory_csv:=results/slip_compensation_on_2026-08-17_trajectory.csv.gz \
  -p summary_json:=results/slip_compensation_on_2026-08-17_summary.json
```

任一验收项不达标时进程返回失败，摘要仍会写出并带 `failure_reason`。

## 原地转向下滑基线

2026-08-17 在全新无界面仿真中重测：5 个角度（`30/45/90/135/180°`）× 2 个角速度
（`0.3/0.6 rad/s`）× 2 次重复，每次交替转向，共 20 次。

| 指标 | 结果 |
| --- | ---: |
| 拟合 `turn_slip_per_degree_m` | **`0.00050`**（即 `0.50 mm/度`） |
| 拟合出的参考点偏移（自检） | **`0.2 mm`** |
| 模型残差 RMS | `2.0 mm` |
| 与 `control.yaml` 现值之差 | `0.4%` |

**下滑量与角速度无关**：`0.3` 和 `0.6 rad/s` 两组在同一角度上几乎相同
（`90°`：`-43.5` vs `-43.9 mm`；`180°`：`-90.3` vs `-90.8 mm`），说明这是准静态
滑移，只取决于转过的角度。因此单个 `mm/度` 常数是合适的模型形式，不需要按转向
方向或角速度拆成多个参数。

