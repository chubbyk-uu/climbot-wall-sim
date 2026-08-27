# climbot_interfaces

跨包 ROS 通信定义。本包没有节点、算法和配置文件，也不依赖任何其他项目包。它是依赖图最
上游的两个共享包之一，因此所有在线包都可以放心依赖它，不会绕出环来。

## 定义

| 定义 | 用途 |
| --- | --- |
| `msg/CoverageTask.msg` | 不可分割的名义覆盖任务：路径、线段类型、区域与检测足迹 |
| `msg/CoverageConfig.msg` | 面板可写的区域形状与扫描方向 |
| `msg/CoverageStatus.msg` | 管理器状态汇总，界面据此显示而不自行拼装状态 |
| `msg/ExecutionReference.msg` | 转向后按 EKF 实际位置冻结的直线与采集许可 |
| `msg/InspectionCapture.msg` | 单张成功图的任务、触发编号与曝光时刻相机位姿 |
| `msg/InspectionCaptureGate.msg` | 采集侧存活 heartbeat |
| `msg/InspectionArchiveStatus.msg` | 归档权威状态、计数、目录与错误 |
| `srv/ConfigureCoverage.srv` | 无运行任务时修改区域形状／扫描方向 |
| `srv/StartCoverage.srv` | 带采集选项的受控启动 |
| `srv/CaptureOnce.srv` | G1 单次人工触发及其拒绝码 |
| `srv/PrepareInspectionArchive.srv` | 原子创建 run 并完成目录、空间与标定预检 |
| `srv/FinalizeInspectionArchive.srv` | 封存 run 并复核 `expected_images == saved_images` |
| `action/ExecuteCoverage.action` | 执行冻结的多段路径，含取消、反馈与结果 |

## 兼容性

字段语义由消息注释和 [接口合同](../../docs/INTERFACES.md) 共同维护。

当前协议要求 `InspectionCaptureGate` 的 `active` 恒为 `false`，接收方必须拒绝
`active=true` 的消息：正常采图不允许调制扫描速度。这个字段之所以保留，是给将来某个
显式版本化的恢复协议留的位置，不是留给现在用的。

## 边界

本包不得读取 YAML，不包含几何规划、控制算法、Gazebo 代码或节点实现。
新增字段前先确认它属于跨包合同，而不是某个包的内部状态。

## 测试

```bash
colcon test --packages-select climbot_interfaces
colcon test-result --verbose
```
