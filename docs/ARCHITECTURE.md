# 系统架构与目录职责

更新：2026-10-22。本文档描述当前代码结构和依赖边界。项目目标与验收标准以
[PROJECT_GUIDE.md](../PROJECT_GUIDE.md) 为准，运行接口见
[INTERFACES.md](INTERFACES.md)，实施状态见 [STATUS.md](STATUS.md)。

## 包依赖方向

在线 ROS 图，箭头指向被依赖方（`package.xml` 的实际声明）：

```text
                            climbot_bringup
         ┌───────────┬───────────┼───────────┬───────────┐
         v           v           v           v           v
      gazebo      coverage    control    description  inspection
         │           │
         │           └────> rviz_plugins
         └────> control                       （exec：启动速度看门狗）
         └────> inspection                    （test：契约测试读对方配置）

{gazebo, coverage, control, rviz_plugins, inspection} ──> interfaces
{gazebo, coverage, control, inspection}               ──> description
{gazebo, inspection, mosaic}                           ──> common
```

（图中省略 `climbot_` 前缀。）

`climbot_image_processing` 和 `climbot_mosaic` 不进入在线 ROS 图：它们只按目录读取已封存的
产物。`climbot_mosaic` 对 `climbot_common` 的依赖是纯 Python 工具依赖，不引入 ROS 运行时。

自上而下读：`climbot_bringup` 只有组合 launch，在运行时点名下游各包；
`climbot_common`、`climbot_interfaces` 和 `climbot_description` 是共享上游，都不依赖其他
项目包。

`climbot_rviz_plugins` 只依赖 `climbot_interfaces`、`std_srvs` 和 RViz/Qt，不依赖
规划或控制实现。`climbot_coverage` 运行时依赖它，是因为 `coverage.rviz` 载入该面板。

`climbot_description` 是共享物理描述的唯一上游。规划器和控制器不得依赖
`climbot_gazebo`，也不得读取 Gazebo 真值或仿真专有参数。`climbot_gazebo` 对
`climbot_inspection` 的依赖是 `test_depend` 而非 `exec_depend`：契约测试要读对方
安装到 ament index 的配置，因此要求它先构建，这不是运行期算法依赖。

组合入口 `coverage_sim.launch.py` 和 `coverage_mission.launch.py` 集中在
`climbot_bringup`。它们在运行时查找 `climbot_gazebo`、`climbot_coverage` 和
`climbot_control`，这是启动编排，不是算法依赖；把它们放在 `climbot_coverage` 里
会让规划器包的依赖表读起来像规划算法依赖 Gazebo 和控制器，因此单独成包，且没有
任何包依赖 `climbot_bringup`。`climbot_gazebo` 为启动速度看门狗而运行时依赖
`climbot_control`，同样只属于仿真编排；控制包不反向依赖 Gazebo。

## 分册

- [包职责](architecture/PACKAGES.md)：每个包的边界与实现归属。
- [配置与数据流](architecture/RUNTIME.md)：参数唯一来源、坐标系和跨包约定。

当前 M2 起伏／柔顺与传感器标定见[M2 模型](m2/MODEL.md)；实际接口见[接口合同](INTERFACES.md)。
