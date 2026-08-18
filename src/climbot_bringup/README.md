# climbot_bringup

整系统启动入口。本包只有 launch 编排，没有节点、算法和参数文件。

## 启动

仿真、规划器和 RViz，只预览不执行：

```bash
ros2 launch climbot_bringup coverage_sim.launch.py
```

再加跟踪器和任务管理器的完整任务入口，默认在 RViz 中点选区域：

```bash
ros2 launch climbot_bringup coverage_mission.launch.py
```

两个 launch 都接受 `config_file`（`coverage_sim`）或 `planner_config_file` 与
`control_config_file`（`coverage_mission`）、`region_type` 和 `sweep_direction`。
完整参数和演示配置见仓库根目录 [README](../../README.md)。

## 为什么单独成包

组合 launch 会在运行时查找 `climbot_gazebo`、`climbot_coverage` 和
`climbot_control`。这两个文件此前放在 `climbot_coverage` 里，使规划器包在运行时
依赖仿真包和控制包——依赖关系并非算法需要，只是启动编排的副作用。移到本包后，
依赖单向指出：`climbot_bringup → {gazebo, coverage, control}`，没有任何包依赖
本包，编排可以随意点名而不污染算法包的依赖表。

单包入口（`climbot_gazebo/climbot_wall.launch.py`、
`climbot_coverage/coverage_planner.launch.py`、
`climbot_control/coverage_executor.launch.py`）留在各自包内，本包不重复封装。
`ekf_wall.yaml` 也没有移过来：它是 `climbot_wall.launch.py` 里 EKF 节点的参数，
和喂给它的仿真传感器同属定位链路，不是编排文件；移过来只会让 `climbot_gazebo`
反向依赖本包。
