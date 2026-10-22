# 历史分册 7

### 为什么旧数据不能直接用

`turn_slip_2026-08-13_uncorrected.csv` 采于把 `base_link` 移到主动轮轴中点
（提交 `89feb21`，13:56）**之前** 15 分钟（提交 `0598525`，13:41）。参考点当时在
旋转中心后方约 `79 mm`，原地转向会让它绕中心划一段弧——这段**运动学摆动**和真实
下滑一起进了 `vertical_mm`。后果是逐方向的数值毫无物理意义：

| 转角 | 旧数据 CW | 旧数据 CCW | 新数据 CW | 新数据 CCW |
| ---: | ---: | ---: | ---: | ---: |
| `90°` | `-123.8 mm` | **`+34.2 mm`** | `-42.2 mm` | `-45.2 mm` |
| `180°` | `-243.6 mm` | **`+68.3 mm`** | `-90.5 mm` | `-90.6 mm` |

旧数据里逆时针大角度净位移**向上**——重力滑移不可能这样。新数据两个方向一致到
`0.0 mm/度`。

**但聚合系数一直是对的**：标定扫描两个方向对称，摆动在总体斜率里互相抵消，所以
两份数据拟合出的系数都是 `0.00049~0.00050`。`control.yaml` 里那个手工填的
`0.0005` 因此并没有错，只是当时无法证明。

## 重新标定转向下滑

```bash
ros2 launch climbot_gazebo climbot_wall.launch.py \
  use_sim_time:=true headless:=true

ros2 run climbot_gazebo measure_turn_slip.py --ros-args \
  -p use_sim_time:=true -p output_csv:=results/turn_slip.csv
```

脚本结束时联合拟合参考点偏移和下滑系数，直接打印可填进 `control.yaml` 的
`turn_slip_per_degree_m`，并把偏移作为自检报出：超过
`maximum_reference_offset_m`（默认 `0.05 m`）即报错，说明上报位姿不在旋转中心，
CSV 里的逐角度数值不能当作真实滑移读。换墙面、改摩擦或 WheelSlip 参数后都必须
重标定。

## 起步滑移带

`measure_turn_slip.py` 从机器人当时朝向起转，因此漏掉了一个只取决于**起始朝向**的
效应。`measure_turn_band.py` 把起始朝向固定成自变量，扫「起始朝向 × 转角 × 转向
方向」：

```bash
ros2 launch climbot_gazebo climbot_wall.launch.py \
  use_sim_time:=true headless:=true

# 全圆映射（results/turn_map.csv）：每 15° 一个朝向，正反各转 30°
ros2 run climbot_gazebo measure_turn_band.py --ros-args \
  -p use_sim_time:=true -p output_csv:=results/turn_map.csv

# 带形细扫（results/turn_band.csv）
ros2 run climbot_gazebo measure_turn_band.py --ros-args \
  -p use_sim_time:=true -p output_csv:=results/turn_band.csv \
  -p headings_deg:='[100.0, 105.0, 110.0, 112.0, 114.0, 116.0, 120.0, 125.0]' \
  -p angles_deg:='[30.0]'
```

**这条带只存在于 220 N 配置**：起步做原地转向时，若朝向偏离竖直 12°~40° 且转向
方向使车头继续下压，起步瞬间沿车身轴滑约 **68 mm**，与转角无关；反向转、或已扫过
该带，都正常。峰值在偏离竖直 24°（即 114°/66°/246°/294°）。

**400 N 下带已消失**，`turn_map.csv` 全圆 48 次转向读数 `0.392~0.441 mm/度`，六个
原危险航向与其余航向无异。曾按这两份 CSV 标定的 `turnLeadOut()` 已随之删除。改
墙面、摩擦、吸附力、机器人质量或 WheelSlip 参数后都必须重扫这张全圆图，确认它仍
是平的。

### 带的成因：主动轮摩擦饱和

`results/band_variants/` 是把吸附力和摩擦系数当自变量的对照扫描，每个变体在
`100/105/110/114/120/125°` 起转 `+30°`。复现方式是改
`src/climbot_gazebo/config/simulation.yaml`（`install/` 是符号链接，不必重编），
每个变体重启一次仿真后：

```bash
ros2 run climbot_gazebo measure_turn_band.py --ros-args \
  -p use_sim_time:=true \
  -p output_csv:=results/band_variants/<变体名>.csv \
  -p headings_deg:='[100.0, 105.0, 110.0, 114.0, 120.0, 125.0]' \
  -p angles_deg:='[30.0]'
```

| 变体 | 峰值 mm/度 |
| --- | ---: |
| `baseline`（220 N，`wheel_mu` 1.1） | 2.536 |
| `lateral_0`（`slip_lateral` → 0） | 2.637 |
| `longitudinal_0`（`slip_longitudinal` → 0） | 2.530 |
| `caster_mu_1p1`（0.35 → 1.1） | 2.536 |
| `wheel_mu_2.2`（墙面与主动轮同时 1.1 → 2.2） | 0.416 |
| `wheel_mu_0.7` | 整机脱落 |
| `suction_210` / `suction_215` | 整机脱落 |
| `suction_222` | 1.270 |
| `suction_225` / `230` / `240` / `260` | 0.531 / 0.525 / 0.519 / 0.508 |
| `suction_440` / `880` | 0.416 / 0.338 |

`wheel_mu` 加倍与吸附力加倍给出**逐点相同**的曲线，所以支配量是乘积 `μN`，带是
主动轮的库仑摩擦饱和。万向轮 μ 无关——它是滚动的球，只抢载荷不出切向力。

`suction_210` 与 `suction_215` 的 CSV 里 `slide_mm` 是几百米到几千公里、
`wall_height_m` 一路变负，那是脱落后的自由落体，不是量测量，只用来定脱落门槛。

详细核算见 `docs/STATUS.md` 的「220 N 贴在脱落悬崖边」。

## Phase G2 位置触发正式验收

当前正式证据为 `g2_accept_horizontal_2026-08-26_summary.json`、
`g2_accept_vertical_2026-08-26_summary.json` 和
`g2_accept_trapezoid_2026-08-26_summary.json`，均产自 `commit=6f2e6f6` 干净树。
三工况只在冻结的正式 SCAN 段触发，图像与元数据逐帧同时间戳，并同时完成 G4 原图归档。

| 工况 | 前后最大间距 | 左右最大间距 | 离散照片覆盖率 | 最大位置误差 | 最大航向误差 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 横向矩形（2 SCAN / 10 张） | 203.413 mm | 414.508 mm | 81.100% | 3.179 mm | 0.150° |
| 竖向矩形（2 SCAN / 12 张） | 202.022 mm | 399.609 mm | 84.528% | 3.343 mm | 0.118° |
| 短顶边梯形（2 SCAN / 9 张） | 207.005 mm | 415.140 mm | 78.666% | 2.725 mm | 0.176° |

有效照片足迹为 `0.500 × 0.28125 m`。横向扫描带和纵向拍照重叠是两个独立参数，当前
名义值均为 `20%`；实际验收下限为 `15%`，对应相邻中心间距不超过 `0.2390625 m` 和
`0.425 m`。可走区优先语义下，离散照片覆盖率仅作报告，不能以不足某个隐含面积百分比
判定任务失败。
评价器以每张实际曝光位置的离散足迹求并集，不用连续扫描带代替照片覆盖率。
每个任务的归档 manifest 都满足冻结 `expected_images == saved_images`（`10/12/9`）。

`2026-08-24` 的三份摘要（`25%` 名义／`20%` 实际重叠、`+0.300 m` 外参、每工况 3 条
SCAN 和 18 张）保留为历史策略证据，不能代替上表的当前机械和重叠策略基线。

## 首批离线处理／拼接输入数据集（2026-08-25）

这两组不是小型 G2 门禁工况，而是供后续 `climbot_image_processing` 和
`climbot_mosaic` 使用的原始数据。二者均使用带混凝土贴图、`wall_grid_spacing:=0`、
`+0.340 m` 光心、`20%` 名义重叠和位置触发；原图保持畸变、未做光照补偿。任务配置在
`src/climbot_coverage/config/coverage_inspection_dataset_{horizontal,vertical}.yaml`。

| 数据集 | 可走区 | 扫描方向／主要航向 | 原图数 | 数据目录 | 质量结果 |
| --- | --- | --- | ---: | --- | --- |
| horizontal | `6.0 × 4.1 m` | 左右交替；重力补偿使相机航向带有上偏 | 270 | `${CLIMBOT_DATA_ROOT}/inspection-dataset-horizontal-20260825/<run-id>/` | 最大沿轨／横向间距 `228.165/403.854 mm`，最大位姿误差 `4.227 mm`，实际照片覆盖 `96.752%` |
| vertical | `4.1 × 6.0 m` | 上下交替；包含相反竖向航向 | 270 | `${CLIMBOT_DATA_ROOT}/inspection-dataset-vertical-20260825/<run-id>/` | 最大沿轨／横向间距 `226.324/403.166 mm`，最大位姿误差 `4.294 mm`，实际照片覆盖 `97.286%` |

两份 archive manifest 都是 `outcome=completed`、`expected_images=saved_images=270`、
`failed_images=0`，并各自记录 10 条冻结 SCAN 参考。原始 PNG、逐帧 JSON 标签和标定快照
总计约 `431 MB`，有意不复制到仓库；同目录内的 manifest 已保存文件哈希，可作为离线程序
的输入完整性门禁。

## 当前侧滑基线

2026-08-13 在全新启动的无界面仿真中完成一次正式实验。参数为横向 WheelSlip
`0.12`、纵向 WheelSlip `0.04`、速度 `0.15 m/s`、静止 `30 s`、各运动段
`8 s`、三次重复。水平段只保持融合航向为 `0°`，不使用横轨位置纠偏。

| 指标 | 结果 |
| --- | ---: |
| 静止高度变化 | < 0.1 mm |
| 三次水平下降/前进比 | 10.47% / 10.64% / 10.57% |
| 水平下降比均值 | 10.56% |
| 水平下降比总体变异系数 | 0.64% |
| 平均上行速度 | 0.14086 m/s |
| 平均下行速度 | 0.15197 m/s |
| 下行比上行快 | 7.89% |
| 重复 `phase + time_s` | 0 |

三重复总体变异系数低于 `5%` 验收阈值。正式 CSV 中水平段真值平均航向为
`0.28°～0.41°`，因此该下降率主要反映重力侧滑，而不是未约束的航向漂移。

## 重新生成侧滑基线

每次正式标定必须从全新启动的仿真开始；不能在同一世界状态中连续运行两次。
在工作区根目录分别运行：

```bash
ros2 launch climbot_gazebo climbot_wall.launch.py headless:=true
```

然后在另一终端运行：

```bash
ros2 run climbot_gazebo calibrate_wall_slip.py --ros-args \
  -p use_sim_time:=true \
  -p repetitions:=3 \
  -p static_duration_s:=30.0 \
  -p drive_duration_s:=8.0 \
  -p horizontal_repeatability_max_cv:=0.05 \
  -p trajectory_csv:=results/wall_slip_trajectory.csv.gz

ros2 run climbot_gazebo plot_wall_slip.py results/wall_slip_trajectory.csv.gz
```

命令在水平下降比总体变异系数超过阈值时返回失败，但仍会先保存 CSV 供诊断。
替换基线前还应确认同一 `phase + time_s` 无重复，并记录当前配置和代码提交。
