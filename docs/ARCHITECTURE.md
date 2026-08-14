# 系统架构与目录职责

本文档描述当前代码结构和依赖边界。项目目标与验收标准以
[PROJECT_GUIDE.md](../PROJECT_GUIDE.md) 为准，运行接口见
[INTERFACES.md](INTERFACES.md)，实施状态见 [STATUS.md](STATUS.md)。

## 包依赖方向

```text
                    climbot_interfaces
                       ^          ^
                       │          │
              climbot_coverage  climbot_control
                       │          │
                       v          v
                    climbot_description
                            ^
                            │
                     climbot_gazebo
```

`climbot_interfaces` 是无业务实现的公共 ROS 接口包，不依赖其他项目包。
`climbot_description` 是共享物理描述的唯一上游。规划器和控制器不得依赖
`climbot_gazebo`，也不得读取 Gazebo 真值或仿真专有参数。

`climbot_coverage/launch/coverage_sim.launch.py` 是当前阶段用于联合启动仿真
和规划器的临时集成入口，会在运行时查找 `climbot_gazebo`。它不代表规划器
算法依赖 Gazebo。`climbot_gazebo` 为启动速度看门狗而运行时依赖
`climbot_control`，该依赖仅属于仿真编排；控制包不反向依赖 Gazebo。完整任务
状态机出现后，再评估是否抽出独立的 `climbot_bringup` 统一承载组合 launch。

## 包职责

### `climbot_interfaces`（阶段 E 新建）

只包含跨包通信定义：

- `msg/CoverageTask.msg`：不可分割的名义覆盖任务；
- `action/ExecuteCoverage.action`：任务执行、取消、反馈和结果。

该包不得读取 YAML，不包含几何规划、控制算法、Gazebo 代码或节点实现。

### `climbot_description`

共享给仿真、规划、控制和未来实机部署的描述包：

- `config/robot.yaml`：机器人几何、质量、轮系、驱动限幅和规划轮廓；
- `config/wall.yaml`：墙面工作坐标系及可作业表面尺寸；
- `urdf/climbot.urdf.xacro`：`robot_state_publisher` 使用的机器人描述；
- `climbot_description/`：墙面坐标变换和通用几何函数。

该包不得包含摩擦、WheelSlip、吸附施力、出生位姿、仿真传感器噪声或
Gazebo 真值接口。

### `climbot_gazebo`

仿真环境、定位链路和实验工具：

- `config/simulation.yaml`：Gazebo 专有的接触、摩擦、滑移、吸附和传感器参数；
- `config/ekf_wall.yaml`：当前阶段的 `robot_localization` 配置；
- `worlds/`、`models/`：Gazebo 世界与 SDF 模型；
- `launch/climbot_wall.launch.py`：墙面仿真、桥接、TF、传感器适配和 EKF；
- `scripts/`：全站仪模拟、协方差适配、标定和评估工具。

Gazebo 真值只能用于模拟传感器生成、记录和独立评价，不得反馈给规划器或
未来控制闭环。

### `climbot_coverage`

C++ 覆盖规划器和 RViz 可视化：

- `coverage_geometry`：矩形/等腰梯形修正、内缩和弓字路径几何；
- `coverage_planner_node`：参数或 RViz 点选输入、路径与状态发布、重规划服务；
- `config/`：默认矩形和等腰梯形任务；
- `launch/coverage_planner.launch.py`：独立规划入口；
- `launch/coverage_sim.launch.py`：当前阶段的仿真联合入口；
- `rviz/`：墙面、区域和路径显示配置。

规划器读取 `climbot_description` 的墙面尺寸和机器人轮廓，不读取任何
Gazebo 接触参数。

### `climbot_control`

C++ 轨迹控制和速度安全：

- `line_tracker`：任意二维直线的沿轨、横轨和航向闭环及联合轮速限幅；
- `line_tracker_node`：融合位姿输入、定位超时停车和单段参考显示；
- `cmd_vel_watchdog_node`：`/control/cmd_vel` 到 `/cmd_vel` 的唯一安全出口；
- `config/control.yaml`：正常作业限幅、控制增益和超时；
- `launch/line_tracker.launch.py`：从共享机器人描述注入轮距和轮缘硬限值。

控制包不得读取 Gazebo 真值、WheelSlip 或吸附参数。

## 配置归属

| 配置 | 所有者 | 消费者 | 说明 |
| --- | --- | --- | --- |
| `robot.yaml` | description | Gazebo、coverage、未来 control | 真实物理属性和保守规划轮廓 |
| `wall.yaml` | description | Gazebo、coverage、定位、未来实机 | `world → wall` 基准和作业面尺寸 |
| `simulation.yaml` | gazebo | 仅 Gazebo | 吸附、摩擦、WheelSlip、出生位姿、仿真噪声 |
| `ekf_wall.yaml` | gazebo（暂定） | `robot_localization` | 随未来 bringup 一并评估外移 |
| `coverage_*.yaml` | coverage | 覆盖规划器 | 区域输入和扫描参数 |
| `control.yaml` | control | 直线跟踪器、速度看门狗 | 作业速度、控制增益、软件限幅和超时；不复制机器人硬件属性 |

同一物理量只能有一个权威来源。launch 可以把共享 YAML 展开为节点参数或
xacro 映射，但不得在第二个文件中复制同一数值作为长期配置。

## 运行时数据流

```text
Gazebo physics
  ├─ ground truth ───────────────> 评估/模拟全站仪（禁止进入控制）
  ├─ wheel odometry -> covariance adapter ─┐
  └─ IMU -> attitude adapter ──────────────┼─> robot_localization EKF
模拟全站仪 position-only ──────────────────┘           │
                                                        v
                                                /odometry/filtered

区域参数或 RViz 点选 -> coverage planner -> /coverage/task（权威任务预览）
                                              ├─> /coverage/path（派生显示）
                                              └─> coverage manager（待实现，显式开始并锁定版本）
                                                            │
                                                            └─> ExecuteCoverage Action goal
                                                            │
                                                            v
                                                     climbot_control
                                                            ├─> /control/reference_path
                                                            └─> /control/cmd_vel ─┐
键盘或单个实验脚本 ────────────────────────────────────────> /control/cmd_vel ─┤
                                                                                v
                                                                    cmd_vel_watchdog
                                                                                │
                                                                                v
                                                                            /cmd_vel
```

## 坐标系

```text
world
└── wall
    └── odom
        └── base_link
            ├── imu_link
            ├── left_wheel_link
            ├── right_wheel_link
            └── caster_ball_link
```

`base_link` 位于两个主动轮轴中点在墙面接触平面上的投影。覆盖路径、EKF、
里程计和未来控制器必须使用同一参考点。

## 后续包边界

`climbot_interfaces` 和 `climbot_control` 已建立。控制包最终负责 50 Hz
C++ 通用直线段跟踪、任务状态机、
线段类型执行、转向下坠补偿、左右轮联合限幅和速度看门狗。横向为主与竖向为主
的覆盖路径共用同一控制器，只由规划结果和段类型驱动。控制器保留名义覆盖路径，
并在转向后根据 EKF 实际位置冻结单独的平行直线执行参考：小偏差直接接受为平行
扫描线，较大但可恢复的偏差先执行一次前进小弧线再冻结直线。横向保留第二次转向
下滑预补偿，竖向不预补偿且不逐列倒车。暂不新建
`climbot_bringup`；等控制器和完整启动组合出现后再拆，
避免只为一个 launch 提前增加空包。

下一步增加独立任务管理器，订阅 `/coverage/task` 并缓存最新有效预览，只有收到操作员
明确的开始命令后才锁定 `task_id + revision` 并发送 Action Goal。管理器负责开始、
取消、版本锁定和执行状态汇总；规划器不直接调用控制器，RViz 面板也不直接实现安全
状态机。管理器可先落在 `climbot_control`，形成统一系统入口时再随组合 launch 一并
评估是否迁入 `climbot_bringup`。

未来面阵相机及位置触发采集归属于独立的 `climbot_inspection`。它消费冻结后的动态
执行参考、任务状态、EKF 位姿和相机图像，生成触发事件及带位姿的检测数据；不参与
底盘闭环，也不得使用 Gazebo 真值决定拍照。墙面纹理和仿真相机传感器属于
`climbot_gazebo`，真实/共享相机几何安装关系属于 `climbot_description`。
