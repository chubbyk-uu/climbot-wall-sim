# climbot_bringup

整系统启动入口。本包只有 launch 编排，没有节点、算法和参数文件。

## 启动

主线启动命令只在[快速启动](../../docs/QUICKSTART.md)维护：
`coverage_sim` 只预览，`coverage_mission` 启动执行器与管理器。
两个入口均使用新负压底盘，默认起伏墙、柔顺轮、0.30 m/s；有效摩擦 0.8。
`wall_surface:=flat wheel_compliance:=false` 保留 M1 几何对照，摩擦另行显式选择。

两个 launch 都接受 `config_file`（`coverage_sim`）或 `planner_config_file` 与
`control_config_file`（`coverage_mission`）、`region_type` 和 `sweep_direction`。
`coverage_mission` 另有 `tracking_mode`（`distance` 或 `time`），透传给执行器选择
直线段控制律；运行中也可以从面板的 Algorithm 下拉框改。
主线启动命令见 [快速启动](../../docs/QUICKSTART.md#快速启动)，批处理与实验变体见
[docs/OPERATION.md](../../docs/OPERATION.md)。

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
