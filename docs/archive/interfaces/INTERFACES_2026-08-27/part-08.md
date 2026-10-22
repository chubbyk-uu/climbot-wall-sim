# 历史分册 8

## `climbot_mosaic` P2.6 墙面融合

```bash
ros2 run climbot_mosaic build_wall_mosaic \
  --input-run <绝对 processed-run 路径> [--input-run <更多 run> ...] \
  --pose-graph-dir <绝对 P2.5 输出目录> \
  --output-dir <绝对且不存在的输出目录> \
  --work-dir <绝对临时缓存目录> \
  [--resolution-mm-per-pixel 0.25] [--jobs auto|N] [--memory-budget-gb 4]
```

命令重新执行严格输入校验，并要求 P2.5 的 `input_summary`、初始位姿和全部帧键与输入逐项
一致。位姿直投与优化拼接使用同一墙面范围、分辨率、照片、灰度插值、硬切图归属和 tile
顺序，唯一变量是 P2.5 位姿修正。每个墙面像素仅保留一张源图：选择对应源像素离边缘最远的
照片，距离相等时保留稳定输入帧顺序中较早的照片。正式目录原子生成：`mosaic_pose_only.tif`、
`mosaic_optimized.tif`、`mosaic_difference.tif`、`coverage_count.tif`、`uncertainty.tif`、
优化预览、三联对比预览和严格 JSON manifest。

融合以 512×512 tile 并行计算，由主进程单写入 BigTIFF；覆盖次数和不确定度先按 tile 顺序
写入工作目录，再流式压缩，内存不随完整画布像素数线性增长。manifest 的 `fusion` 记录
worker 数、预算、总／逐 pass 耗时、缓存命中和 1 s 采样的 Linux 进程树 PSS 峰值；
`outputs` 记录每个正式文件的字节数与 SHA-256。`uncertainty.tif` 是 `uint16`：比例为
`0.00001 m/count`（`0.01 mm/count`），`65535` 表示无覆盖，最大有限值为 `65534`；读取方
必须按 manifest 的 `uncertainty_encoding` 解码，不能把整数直接解释为米。

当前质量指标是至少两张图覆盖区域内、按源图逐像素计算的灰度标准差均值与 P95。它只用于同输入、
同网格的 pose-only／optimized 相对比较，不等价于 Gazebo 真值误差，也不能在 P2.7 前当作
冻结验收线。`work-dir` 只有可删除的中间缓存；完成证据仅是原子发布的输出目录。

`evaluate_diagnostic_mosaic` 是 P2.7c 的离线后验接口：输入一份 hard-cut mosaic 目录、诊断墙
清单和新输出目录；它从 DDS 母版中按墙面米制坐标读取可见修补块／涂鸦锚点，在两张母版中独立
定位后报告绝对锚点偏差、相似变换尺度／航向／平移及局部残差。真值清单及 DDS 不得成为拼接
输入，也不得参与候选、局部匹配或优化；相位响应低于显式门限的锚点会被记录为拒绝，最终比较
只使用两个版本共同保留的锚点。

`inspect_diagnostic_mosaic` 是 P2.7e 的独立离线后验接口。它要求同一 hard-cut 目录中的
`mosaic_pose_only.tif`、`mosaic_optimized.tif` 与 `coverage_count.tif`，以及诊断墙 manifest/DDS，
并原子发布新的检查目录。`diagnostic_inspection_summary.json` 记录输入 manifest 哈希、米制像素
比例、每个 feature 的可见边界和逐像素覆盖计数；`native_tiles/<feature>/r####_c####.png` 的每个
tile 均不缩放，三列固定为 immutable truth、pose-only、optimized。若 feature 仅与渲染网格
相交但其几何内有 `coverage_count==0`，接口必须明确记录为未覆盖，不能把它当作“已 100% 检查”。

## `climbot_inspection` G2 自动采集接口

| 名称 | 类型 | 语义 |
| --- | --- | --- |
| `/control/execution_reference` | `climbot_interfaces/msg/ExecutionReference` | 执行器当前冻结的有向直线、任务版本、段号、段类型和采集许可；不是规划器的名义预览 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 触发位置和曝光时间位姿插值的唯一业务定位源 |
| `/inspection/capture_metadata` | `climbot_interfaces/msg/InspectionCapture` | 一张成功原图的任务、触发点、冻结参考和曝光时刻 EKF 相机位姿 |
| `/inspection/capture_gate` | `climbot_interfaces/msg/InspectionCaptureGate` | 自动采集器 → 跟踪器，Reliable + transient-local 的采集健康 heartbeat。启用采集的 `SCAN` 必须持续收到同一任务／版本／段的 `active=false` 消息；当前 v1 拒绝 `active=true` 的位置栅栏，正常拍照不得调制扫描速度。`maximum_camera_along_track` 是兼容保留字段，不参与 v1 控制。`reason` 是给操作员看的自由文本，调用方不得据其内容分支。 |

`ExecutionReference.inspection_enabled` 只在正式 `SCAN` 的 `TRACK_LINE`／
`FINAL_APPROACH` 为真；起点进入、对准、转向稳定、动态过渡和小弧线入轨均为假。
`detection_forward_offset` 来自不可变任务，G2 相机任务必须为 `0.340 m` 并与共享安装
外参一致。

自动采集的参考失效、等图像和等 EKF 插值括号期限全部用稳态时间测量。正常情况下，触发、
图像和 EKF 位姿绑定均异步进行，Gazebo 暂停、`use_sim_time` 的 ROS 时钟不前进也不会改变
扫描速度。服务拒绝、图像期限到期或 EKF 插值失败是明确的采集故障：采集器停止发布当前段
heartbeat，跟踪器在其期限内受控中止该段，绝不在机器人已越过目标后静默补拍。归档器仍以
实际标签独立检查相邻间距和目标后滞（默认后滞上限 `25 mm`）。可选平场节点在启动时即校验
NPZ gain 与共享相机的 `1920×1080` 分辨率相符、有限且严格为正；不兼容标定不等待第一帧
才暴露。运行中遇到异常图像只丢弃补偿预览，不影响正式 `image_raw` 归档。

gate 是存活监督，不是位置上限：跟踪器从进入启用采集的 `SCAN` 起，使用本机稳态时钟
在一次性的 `capture_gate_start_timeout_s`（默认 `2.0 s`）建立窗口内等待第一条匹配 gate，
收到后要求每条消息在 `capture_gate_timeout_s`（默认 `0.50 s`）内刷新。首次或后续心跳超时
均发布零速度并以 `TRACKING_FAILED` 中止 Action，不能把过期 gate 当作 inactive 后继续运动。
中止与等待消息都会附上最后一条 gate 的 `reason` 与其段号（匹配与否都附），因为一条来自
别的段的 gate 本身就是诊断。`active=false` 是健康 heartbeat；当采集器判定本段根本无法
拍摄——例如任务的 `detection_forward_offset` 与相机安装不符——它不发布任何 gate，让心跳
超时按上述路径快速停段，而不是放行机器人开完
一条注定没有曝光的扫描线、到归档收尾时才发现为空。
进入、对准、转场等 `inspection_enabled=false` 的参考也不发布 gate：它们不是采集心跳，
尤其不能给即将进入的同段 SCAN 留下一条 `active=false` 的伪放行。
`header.stamp` 只用于消息可追溯性，不能用于时效判断，因为
`use_sim_time` 暂停时它不会前进。完整 inspection launch 同时强制采集器与记录器的纵向
重叠一致；位置触发和曝光位姿均使用完整相机外参，支持非零横向安装偏移。

`InspectionCapture.header` 必须逐字段等于对应 `image_raw.header`。`camera_pose` 是光学
中心在 `header.frame_id` 下的 EKF 插值位姿，协方差包含前置杠杆对航向不确定度的传播；
`wall_heading_rad` 是机器人前进方向在墙面内的航向，亦为标称安装下图像向上的方向。
消费者以 `(task_id, revision, segment_index, trigger_index)` 作为业务主键，以图像时间戳
核对数据配对，不按消息到达顺序猜测。

当前接口是明确的平面墙 `x/y/yaw` 契约：相机法向坐标等于冻结安装外参
`camera_mount_z_m`，不得再叠加 EKF 中 Gazebo canonical link 的非零 `z`；相应相机协方差
也不传播 base `z`。以后实机若要估计吸附间隙、墙面起伏、roll/pitch，必须升级成显式完整
SE(3) base→camera 变换和标定协方差，不能在这个二维接口中暗中混入另一种 `z` 语义。

仿真内部接口不属于真机公共 API：Gazebo 在
`/simulation/inspection_camera/trigger` 收到触发后发布较宽视场的
`ideal_image/ideal_camera_info`，仿真畸变适配器再输出
`/simulation/inspection_camera/image_raw` 和 `camera_info`。1920×1080 RGB 图像采用
Reliable、depth 1，避免大帧在同机桥接和 Python 适配之间被 Best Effort 静默丢弃；
`climbot_inspection` 只消费适配后的成对消息。

`climbot_inspection/config/inspection.yaml` 定义源／输出话题、服务名、期望尺寸与
`frame_id`，以及 `capture_timeout_s`、传输发现稳定时间和预热重试／静默窗口。启动
预热帧只用于建立桥接与渲染传输，全部丢弃；预热输出静默后服务才进入 READY。标准
`coverage_sim.launch.py` 和 `coverage_mission.launch.py` 默认启动该节点，可用
`inspection:=false` 关闭。

## 配置文件

| 文件 | 内容 |
| --- | --- |
| `climbot_description/config/robot.yaml` | 机器人共享物理属性、限幅和 footprint |
| `climbot_description/config/inspection_camera.yaml`（G1 新增） | 相机内参、畸变、有效足迹和标称安装外参 |
| `climbot_description/config/wall.yaml` | 工作坐标系、墙面宽高、参考网格线间距 |
| `climbot_gazebo/config/simulation.yaml` | Gazebo 专有物理和传感器参数 |
| `climbot_gazebo/config/ekf_wall.yaml` | EKF 状态选择、频率和协方差输入 |
| `climbot_coverage/config/coverage_interactive.yaml` | RViz 点选工作流，`coverage_mission.launch.py` 的默认 |
| `climbot_coverage/config/coverage_rectangle.yaml` | 参数模式的矩形任务 |
| `climbot_coverage/config/coverage_trapezoid.yaml` | 默认等腰梯形任务 |
