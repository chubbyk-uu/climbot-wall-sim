# 实验结果说明

本目录保存可追溯的实验输出，不是所有文件都代表当前正式验收基线。正式实验
应同时记录代码提交、配置、随机种子、仿真时长和生成命令。

| 文件 | 状态 | 说明 |
| --- | --- | --- |
| `normal_loads_before_p2.csv` | 历史对照 | P2 调整前的法向载荷 |
| `normal_loads_after_p2.csv` | 历史对照 | P2 调整后的法向载荷 |
| `normal_loads_after_base_link_move.csv` | 当前几何参考 | `base_link` 移至主动轮轴中点后的七工况载荷 |
| `turn_slip.csv` | 当前参考 | 多角度、多角速度原地转向下滑结果 |
| `wall_slip_trajectory.csv` | 待替换，不作正式轨迹基线 | 由旧轮询逻辑生成，约 91% 为同一真值时间戳的重复行 |
| `wall_slip.png` | 待重新生成 | 来自上述旧轨迹 CSV |

重复行不会改变旧实验按阶段首末位姿计算的端点比值，但会扭曲采样密度和任何
按行加权的统计，因此旧轨迹 CSV 和图片不得用于新的回归验收。

## 重新生成侧滑基线

先启动仿真，再在工作区根目录运行：

```bash
ros2 run climbot_gazebo calibrate_wall_slip.py --ros-args \
  -p repetitions:=3 \
  -p static_duration_s:=30.0 \
  -p drive_duration_s:=8.0 \
  -p trajectory_csv:=results/wall_slip_trajectory.csv

ros2 run climbot_gazebo plot_wall_slip.py results/wall_slip_trajectory.csv
```

替换文件前应先确认：同一 `phase + time_s` 无重复、三次水平下降比满足确定的
重复性阈值，并在 [STATUS.md](../docs/STATUS.md) 中关闭对应未决事项。
