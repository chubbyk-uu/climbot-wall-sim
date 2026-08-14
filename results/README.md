# 实验结果说明

本目录保存可追溯的实验输出，不是所有文件都代表当前正式验收基线。正式实验
应同时记录代码提交、配置、随机种子、仿真时长和生成命令。

> **三份覆盖基线已于 2026-08-14 在同一版代码、同一组参数下重跑。** 每份摘要都带
> `provenance` 段（提交、分支、`src` 是否有未提交改动、评价器全部参数、任务名义
> 几何），此后不会再出现"不知道哪一版评价器产生"的情况。重跑的原因是终点停车
> 精度修复改变了扫描线落位，同时转向结束航向误差改为真值口径。

| 文件 | 状态 | 说明 |
| --- | --- | --- |
| `normal_loads_before_p2.csv` | 历史对照 | P2 调整前的法向载荷 |
| `normal_loads_after_p2.csv` | 历史对照 | P2 调整后的法向载荷 |
| `normal_loads_after_base_link_move.csv` | 当前几何参考 | `base_link` 移至主动轮轴中点后的七工况载荷 |
| `turn_slip.csv` | 当前参考 | 多角度、多角速度原地转向下滑结果 |
| `wall_slip_trajectory.csv` | 当前正式基线 | 10209 行；真值时间戳去重；包含真值与融合航向 |
| `wall_slip.png` | 当前正式基线 | 由当前侧滑轨迹 CSV 生成 |
| `coverage_vertical_2026-08-14_trajectory.csv` | 阶段 F 正式结果 | 大型竖向矩形完整真值、融合位姿、动态参考、状态和横轨误差 |
| `coverage_vertical_2026-08-14_summary.json` | 阶段 F 正式结果 | Action 结果、15 段误差和实际二维检测足迹覆盖率 |
| `coverage_trapezoid_horizontal_2026-08-14_trajectory.csv` | 阶段 F 正式结果 | 大型等腰梯形横向扫描完整轨迹 |
| `coverage_trapezoid_horizontal_2026-08-14_summary.json` | 阶段 F 正式结果 | 横向梯形 13 段误差和实际覆盖率 |
| `coverage_trapezoid_vertical_2026-08-14_trajectory.csv` | 阶段 F 正式结果 | 同一等腰梯形竖向扫描完整轨迹 |
| `coverage_trapezoid_vertical_2026-08-14_summary.json` | 阶段 F 正式结果 | 竖向梯形 19 段误差和实际覆盖率 |

## 大型竖向矩形覆盖基线

2026-08-14 在全新无界面仿真中执行 `3.30 × 4.50 m` 竖向矩形任务。规划路径包含
8 条竖向扫描线、15 段；评价器以 `10 mm` 栅格和任务配置的
`0.50 × 0.01 m` 检测足迹计算实际覆盖，不累计转向、换道和入轨运动。

| 指标 | 结果 |
| --- | ---: |
| Action 结果 | 成功，15/15 段 |
| 执行时间 | 288.667 s |
| 实际覆盖率 | 99.632% |
| 漏扫比例 | 0.368% |
| 最大单段横轨 RMS | 2.41 mm |
| 最大绝对横轨误差 | 5.34 mm |
| 最大动态终点误差 | 3.37 mm |
| 最大扫描线偏离名义 | 7.52 mm |
| 最大相邻扫描线间距误差 | 8.66 mm |
| 可见反向往复 | 0 |

该矩形工况高出 `95%` 覆盖率门限 `4.63` 个百分点，因此当前参数下不增加顶部收边扫描。

## 大型等腰梯形覆盖基线

2026-08-14 在两个全新无界面仿真中，对底边 `4.00 m`、上底 `2.60 m`、高
`2.80 m` 的同一等腰梯形分别执行横向和竖向任务。两次均使用 `0.50 × 0.01 m`
检测足迹和 `10 mm` 覆盖栅格。

| 指标 | 横向扫描 | 竖向扫描 |
| --- | ---: | ---: |
| Action 结果 | 成功，13/13 段 | 成功，19/19 段 |
| 执行时间 | 232.093 s | 284.433 s |
| 实际覆盖率 | 99.798% | 98.122% |
| 漏扫比例 | 0.202% | 1.878% |
| 最大单段横轨 RMS | 1.77 mm | 2.91 mm |
| 最大绝对横轨误差 | 10.89 mm | 6.37 mm |
| 最大动态终点误差 | 3.11 mm | 3.57 mm |
| 最大转向结束航向误差（真值口径） | 1.71° | 1.85° |
| 最大扫描线偏离名义 | 10.32 mm | 8.92 mm |
| 最大相邻扫描线间距误差 | 15.83 mm | 9.70 mm |
| 最大水平扫描净高度误差 | 1.16 mm | 不适用 |
| 实际/名义线段总长比 | 1.0453 | 1.0231 |
| 最大机体航向补偿角 | 6.71° | 7.00° |
| 正式直线最大指令角速度 | 0.078 rad/s | 0.073 rad/s |
| 可见反向往复 | 0 | 0 |

三个工况相对 §14.3 于 2026-08-14 放宽后的 `95%` 门限，余量分别为
`4.63 / 4.80 / 3.12` 个百分点，因此本版不增加顶部收边扫描。修改检测足迹、行距、
转向下滑或入轨参数后仍必须重新评价，不能把”不需要收边”视为永久成立。

**覆盖率比上一版略低，这是终点停车精度修复的代价，不是回归。** 修复前扫描线
带有 `+12~+23 mm` 的系统性偏置，恰好把上部各线推向梯形较窄的一侧、把首线压低，
额外扫到了一些边缘面积；旧基线的覆盖率因此偏高（横向 `99.312%`、竖向
`98.186%`）。现在扫描线落在名义位置附近，覆盖率回到规划路径本身所能提供的水平。

名义覆盖率与实测覆盖率之差即执行损失，三个工况分别为 `0.37 / 0.57 / -0.04` 个
百分点（横向梯形实测略高于名义，因为真值足迹在转向前后仍扫过名义直线之外的少量
面积）。规划期门限 `minimum_nominal_coverage_ratio` 取 `0.965`，即验收门限加上
略高于最大执行损失的裕度，避免出现”规划通过、跑几分钟后评价失败”。

## 重新生成覆盖基线

每次正式实验都从全新启动的无界面仿真开始。`climbot_wall.launch.py` 不启动规划器
和执行器，需分别启动；三个工况只有 `--params-file` 与输出文件名不同：

```bash
ros2 launch climbot_gazebo climbot_wall.launch.py headless:=true

ros2 run climbot_coverage coverage_planner_node --ros-args \
  --params-file src/climbot_coverage/config/coverage_vertical_demo.yaml \
  -p use_sim_time:=true \
  -p robot_length:=0.76 -p robot_width:=0.475 -p edge_clearance:=0.10 \
  -p wall_width:=10.0 -p wall_height:=8.0

ros2 run climbot_control line_tracker_node --ros-args \
  --params-file src/climbot_control/config/control.yaml \
  -p use_sim_time:=true -p standalone_mode:=false \
  -p wheel_separation:=0.43 -p wheel_speed_limit:=0.30 \
  -p wheel_acceleration_limit:=0.40

ros2 run climbot_gazebo evaluate_coverage_execution.py --ros-args \
  -p use_sim_time:=true -p case:=planned_task -p execution_timeout_s:=600.0 \
  -p trajectory_csv:=results/coverage_vertical_2026-08-14_trajectory.csv \
  -p summary_json:=results/coverage_vertical_2026-08-14_summary.json
```

评价器在任一验收项不达标时返回失败，但无论成功、超时还是异常都会先写出 CSV 和
摘要；失败运行的摘要里 `completed=false` 并记录 `failure_reason`，不要把这类文件
当作基线。摘要的 `provenance.git.source_modified` 为 `true` 表示该结果产生时
`src` 有未提交改动，此时记录的提交号只是父提交。

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
  -p trajectory_csv:=results/wall_slip_trajectory.csv

ros2 run climbot_gazebo plot_wall_slip.py results/wall_slip_trajectory.csv
```

命令在水平下降比总体变异系数超过阈值时返回失败，但仍会先保存 CSV 供诊断。
替换基线前还应确认同一 `phase + time_s` 无重复，并记录当前配置和代码提交。
