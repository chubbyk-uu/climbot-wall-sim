# climbot_inspection

面阵相机巡检采集：单次人工触发、位置触发拍照、图像—任务—位姿绑定、平场标定和
原图归档。本包只依赖公共接口和共享描述，**不依赖 Gazebo API**，所以把输入换成真实
相机之后，同一套节点可以直接复用。

## 节点

| 节点 | 阶段 | 职责 |
| --- | --- | --- |
| `capture_once_node` | G1 | 桥接原图与 `CameraInfo`，提供单次触发服务并检查图像／标定／TF 一致性 |
| `automatic_capture_node` | G2 | 按冻结执行参考和 EKF 沿轨进度触发拍照，绑定任务版本、扫描线、触发编号和插值位姿 |
| `flat_field_node` | G3 | 用固定 LED 平场矩阵并行发布补偿图（默认关闭的在线调试预览） |
| `archive_recorder_node` | G4 | 把原始畸变图和曝光标签原子写入任务目录 |

`scripts/calibrate_flat_field` 用 30 次独立的纯灰板曝光算出平场矩阵。

## 启动

```bash
ros2 launch climbot_inspection inspection.launch.py \
  inspection_output_root:="$CLIMBOT_DATA_ROOT"
```

`coverage_mission.launch.py` 已经包含这一条。`inspection_output_root` 未配置时必须失败，
不能回退到机器私有目录。launch 启动时校验 `automatic_capture_node` 与
`archive_recorder_node` 共享同一个 `[0, 1)` 内的 `image_overlap_ratio`——两边不一致
就会算出不同的拍照计划，宁可启动失败也不要在封存时才发现计数对不上。相机几何
（`effective_length_m`、安装外参、图像尺寸）由 launch 从
`climbot_description/config/inspection_camera.yaml` 统一注入，不在本包另存一份。

## 采集间距

```text
spacing = effective_length_m × (1 - image_overlap_ratio)
```

`effective_length_m` 和相机前置偏移的权威来源是
`climbot_description/config/inspection_camera.yaml`（当前 `0.28125 m` 和 `0.340 m`）。
纵向 `image_overlap_ratio` 是采图策略，与规划器的横向 `overlap_ratio` 各自独立配置，
当前默认都是 `20%`。相机投影中心并不在 `base_link` 上，所以扫描线端点必须显式经过 TF
换算，不能拿底盘位置当相机位置用。

## 存活 heartbeat

`/inspection/capture_gate` 是采集侧向跟踪器报“我还活着”的心跳，reliable +
transient-local。协议要求 `active=false`——正常采图不允许调制扫描速度。

遇到根本没法采图的 SCAN——例如任务的 `detection_forward_offset` 与相机安装外参对不上——
本包**一条心跳都不发**，连 `active=false` 的也不发。这类配置故障在任务运行期间不可能
自行恢复，这条线上的每一次曝光都会丢。心跳一撤，跟踪器的存活监督就会在
`capture_gate_start_timeout_s` 之内把这条线停掉，而不是等整条线开完、封存时才发现
归档是空的。

## 归档不可变

正式归档永远订阅 `image_raw`；`image_compensated` 是调试预览，不是数据产品。
一个 run 必须有 manifest、原图 SHA-256、每图标签和相机快照，且
`expected_images == saved_images`；封存后不可改写，后续处理一律写新目录。

## 边界

Gazebo 相机传感器、渲染噪声和畸变适配留在 `climbot_gazebo`。正常触发逻辑不得订阅
Gazebo 真值。检测算法只消费已绑定的数据，不反向进入底盘控制闭环。

## 测试

```bash
colcon test --packages-select climbot_inspection
colcon test-result --verbose
```

接口字段见 [接口合同](../../docs/INTERFACES.md)，故障处置见
[实验与故障处置手册](../../docs/OPERATION.md)。
