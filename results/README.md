# 实验结果说明

本目录保存可追溯的实验输出，不是所有文件都代表当前正式验收基线。正式实验
应同时记录代码提交、配置、随机种子、仿真时长和生成命令。

| 文件 | 状态 | 说明 |
| --- | --- | --- |
| `normal_loads_before_p2.csv` | 历史对照 | P2 调整前的法向载荷 |
| `normal_loads_after_p2.csv` | 历史对照 | P2 调整后的法向载荷 |
| `normal_loads_after_base_link_move.csv` | 当前几何参考 | `base_link` 移至主动轮轴中点后的七工况载荷 |
| `turn_slip.csv` | 当前参考 | 多角度、多角速度原地转向下滑结果 |
| `wall_slip_trajectory.csv` | 当前正式基线 | 10209 行；真值时间戳去重；包含真值与融合航向 |
| `wall_slip.png` | 当前正式基线 | 由当前侧滑轨迹 CSV 生成 |

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
