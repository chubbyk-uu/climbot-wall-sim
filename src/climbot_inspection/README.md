# climbot_inspection

面阵相机巡检采集：单次人工触发、位置触发拍照、图像—任务—位姿绑定、平场标定和
原图归档。本包只依赖公共接口和共享描述，**不依赖 Gazebo API**，所以把输入换成真实
相机之后，同一套节点可以直接复用。

## 节点

| 节点 | 阶段 | 职责 |
| --- | --- | --- |
| `capture_once_node` | G1 | 桥接原图与 `CameraInfo`，提供单次触发服务并检查图像／标定／TF 一致性；成功发布后发送轻量 capture receipt |
| `automatic_capture_node` | G2 | 按冻结执行参考和 EKF 沿轨进度触发拍照，绑定任务版本、扫描线、触发编号和插值位姿 |
| `flat_field_node` | G3 | 用固定 LED 平场矩阵并行发布补偿图（默认关闭的在线调试预览） |
| `archive_recorder_node` | G4 | 把原始畸变图和曝光标签原子写入任务目录 |

`scripts/calibrate_flat_field` 用 30 次独立的纯灰板曝光算出平场矩阵。

## 启动

```bash
ros2 launch climbot_inspection inspection.launch.py
```

`coverage_mission.launch.py` 已经包含这一条。数据根优先使用显式传入的
`inspection_output_root`，否则读取 `CLIMBOT_DATA_ROOT`，两者都没有时使用记录器主机当前用户的
`$HOME/climbot_data`；RViz 的 Capture 页显示管理器最终解析出的默认值，手动修改只覆盖从该面板
启动的任务，不修改环境变量。launch 启动时校验 `automatic_capture_node` 与
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

## 任务暂停时会发生什么

本包不需要知道“暂停”这件事。执行器**停稳之后**发布的 `ExecutionReference` 带
`inspection_enabled=false`——和 transition 段完全同一种形状——于是本包丢掉当前参考、停发
心跳，采集门就此关闭。

关门的时点是“机器人停了”，不是“操作员按了暂停”。减速那段路仍带 `inspection_enabled=true`，
落在刹车距离内的触发目标照常在正确位置曝光；否则它们会被跳过，恢复时在停住的位置补拍，落到
目标后方几厘米，被归档按纵向重叠合同拒绝。这个时点由 `climbot_control` 保证并在那里回归。

关闭是临时的，这一点要靠三件事保证：段不会被停用（`disabled_key_` 不参与）、触发计数
`next_trigger_` 不清零、已经发出去的那次曝光照常结算。恢复时任务／版本／段序号都没变，
键相同，于是从上次的触发序号往下接着数，既不会重拍上一张，也不会跳过下一张。整个过程中
归档 run 不变，暂停不触发 finalize。

## 归档不可变

正式归档永远订阅 `image_raw`；`image_compensated` 是调试预览，不是数据产品。
`CameraInfo` 是记录器进程内不可变的会话标定，以 reliable + transient-local 发布；每次曝光
仍在 `capture_once_node` 内严格匹配同时间戳的源图和源标定，但 G2 与评价器只订阅轻量的
`/inspection/capture_receipt`，G4 使用最新的会话标定匹配每一对图像／metadata。封存时仍逐项
核对相机快照，运行中若标定内容变化会失败，不会静默混用。这样归档完整性不再依赖长任务中
数百份完全相同的 `CameraInfo` 都被重复送达。

一个 run 必须有 manifest、原图 SHA-256、每图标签和相机快照，且
`expected_images == saved_images`；封存后不可改写，后续处理一律写新目录。为避免长任务把
磁盘同步队列打满，原图和标签仍逐对原子写入，但默认每 32 张才做一次文件系统耐久提交；结束、
取消或失败时会强制提交最后一批。进行中的 manifest 会分别写出 `saved_images`、
`durably_committed_images` 和 `staged_images`，只有 `outcome=completed` 且 `staged_images=0`
的 run 才是正式输入。可用 `durable_commit_batch_images` 调整批大小，必须为正整数。

每次耐久提交单独计时并写入 manifest 的 `durable_commits`、`durable_commit_last_ms`、
`durable_commit_last_images`、`durable_commit_max_ms` 和 `durable_commit_max_images`。批大小只
决定 `syncfs(2)` 的调用频率，不限制单次耗时，所以这项统计要按最大值而不是平均值判读。

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
