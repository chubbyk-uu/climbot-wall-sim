# 配置、运行时数据流与坐标系

## 配置归属

| 配置 | 所有者 | 消费者 | 说明 |
| --- | --- | --- | --- |
| `robot.yaml` | description | Gazebo、coverage、control | 共享标量默认及载荷／驱动属性；详细底盘由 load_robot 同源派生 |
| `vacuum_chassis.yaml` | description | 底盘生成器、load_robot | 新底盘几何、材料、提手、风机外形、密封圈与球座 |
| `inspection_camera.yaml` | description | Gazebo、inspection、未来实机 | 分辨率、内参、畸变、有效 ROI 和标称相机外参 |
| `wall.yaml` | description | Gazebo、coverage、定位、未来实机 | `world → wall` 基准、作业面尺寸和参考网格线间距 |
| `simulation.yaml` | gazebo | 仅 Gazebo | 摩擦、WheelSlip、出生位姿、灯光、IMU 安装与仿真噪声；无旧恒力参数 |
| `vacuum_{physics,runtime,io}.yaml` | gazebo | 仅 Gazebo | 分别对应风机与裙边物理、启动／压力源选项、传感器与故障合同 |
| `ekf_wall.yaml` | gazebo | `robot_localization` | 定位链路配置，随喂给它的仿真传感器留在 gazebo；不属于编排，未随 bringup 外移 |
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
                                                        │
Gazebo triggered camera -> image_raw + CameraInfo ──────┼─> climbot_inspection
                                                        │       ├─> G1 单次采集
冻结执行参考 + coverage status ─────────────────────────┘       └─> G2 位置触发/数据绑定

区域参数或 RViz 点选 -> coverage planner -> /coverage/task（权威任务预览）
                                              ├─> /coverage/path（派生显示）
                                              └─> coverage manager（显式开始并锁定版本）
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
            ├── inspection_camera_link
            │   └── inspection_camera_optical_frame
            ├── left_wheel_link
            ├── right_wheel_link
            └── caster_ball_link
```

`base_link` 位于两个主动轮轴中点在墙面接触平面上的投影。覆盖路径、EKF、
里程计和未来控制器必须使用同一参考点。

`inspection_camera_optical_frame` 的标称平移为 `[0.340, 0.000, 0.275] m`，光学
`+z` 指向墙面，光学 `+x` 对应机器人横向，光学 `+y` 对应机器人反向。该前置偏移
意味着相机投影中心不是 `base_link`；巡检覆盖和端点计算必须显式使用 TF。

## 跨包职责的关键约定

这些约定解释各包**为什么**这样切分，是评审改动时的判据；具体字段和参数见
[INTERFACES.md](../INTERFACES.md)。

**控制器是唯一的运动执行方。** `climbot_control` 负责 50 Hz 直线段跟踪、任务状态机、
线段类型执行、转向下坠补偿、左右轮联合限幅和速度看门狗。横向与竖向覆盖共用同一个
控制器，只由规划结果和段类型驱动，不为方向分叉出第二套控制律。

**名义路径和执行参考不是一回事。** 控制器保留规划器发布的名义覆盖路径，并在每次转向后
按 EKF 实际位置另行冻结一条平行直线作为执行参考：偏差小就直接接受，偏差较大但可恢复
时先走一段采集关闭的前进小弧线再冻结。横向扫描保留第二次转向的下滑预补偿，竖向不预
补偿、也不逐列倒车。首条扫描之前还有一次采集关闭的起点进入——一条直线开到首个路点，
终点会按该处转向的预计下坠量抬高一截，与换道段共用同一套预留。这条直线不计入覆盖段。

**任务状态机留在 `climbot_control`，不在编排包也不在界面里。** `coverage_manager_node`
缓存 `/coverage/task` 的最新有效预览，只有收到操作员明确的 `/coverage/start`，才把它复制
下来、锁定 `task_id + revision`，然后发出 Action Goal。它同时提供 `/coverage/cancel`，并把
状态、任务标识、段进度和上次结果汇总到 `/coverage/manager_status`，界面因此不必自己拼装
状态。`/coverage/pause` 与 `/coverage/resume` 也在这里，但管理器只做转发和回显：暂停由执行器
判定并通过反馈宣布，管理器把请求变成一次到执行器的 `SetBool` 就不再自行推断，理由和上面
一样——一个自己猜状态的中间层，猜错的那次正是操作员在看的那次。规划器不直接
调用控制器，RViz 面板也不实现安全状态机。管理器是 Action 客户端而不是启动编排，因此不
属于只放 launch 的 `climbot_bringup`。

**失去执行器时先停机，不报完成。** 管理器进入 `STOPPING`：速度保持只是持续施加的保护，
不是任务已经终止的证据。离开这个状态只认两件事之一：再没有任何一方下达运动指令，
或者执行器最终应答了；这期间停止入口始终有效。如果应答永远不来，操作员可以二次确认、
强制放弃监督，但那也只会进入持续 hold、禁止启动的 `RECOVERY_LOCKED`——必须到现场确认过
之后再单独 Rearm，任务入口才会重新打开。

**采集不进入底盘闭环。** `climbot_inspection` 消费冻结后的执行参考、任务状态、EKF 位姿和
相机图像，产出触发事件与带位姿的检测数据，但不参与底盘控制，也不得用 Gazebo 真值决定拍照。
墙面纹理和仿真相机传感器属于 `climbot_gazebo`，共享的相机几何安装关系属于
`climbot_description`。
