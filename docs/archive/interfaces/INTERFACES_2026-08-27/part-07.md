# 历史分册 7

## G4 任务归档接口

| 名称 | 类型 | 方向与权威语义 |
| --- | --- | --- |
| `/inspection/archive/prepare` | `climbot_interfaces/srv/PrepareInspectionArchive` | 管理器 → 记录器。提交冻结 `CoverageTask` 与输出根目录；成功响应给出 `run_id`、绝对目录、名义容量预估。此响应只代表归档准备完成，随后才允许发送运动 Goal。 |
| `/inspection/archive/finalize` | `climbot_interfaces/srv/FinalizeInspectionArchive` | 管理器 → 记录器。按 `run_id` 封存为 `COMPLETED`／`CANCELED`／`FAILED`；完成时严格核对冻结参考计划、实际归档张数以及每段相邻实际曝光位置的最大间距。 |
| `/inspection/archive/status` | `climbot_interfaces/msg/InspectionArchiveStatus` | 记录器 → 管理器／RViz，Reliable + transient-local。它是本次归档状态、实际预计张数、保存／失败计数和最终目录的权威来源。 |
| `/simulation/inspection_camera/trigger` | `std_msgs/msg/Bool` | 仅 Gazebo 内部：`capture_once_node` → 相机传感器的单次触发 topic。真机不实现该接口。 |
| `/simulation/inspection_camera/ideal_image`、`ideal_camera_info` | `sensor_msgs/msg/Image`、`CameraInfo` | 仅 Gazebo 内部的无镜头畸变渲染输出。 |
| `/simulation/inspection_camera/image_raw`、`camera_info` | `sensor_msgs/msg/Image`、`CameraInfo` | 仅 Gazebo 内部的畸变适配器输出；桥接到正式 `/inspection/camera/*` 输入前必须时间戳成对。 |

G4 的 `climbot_inspection` 任务级记录器输入 `image_raw`、`CameraInfo`、
`InspectionCapture`、`ExecutionReference` 和冻结任务快照，输出到受配置约束的任务目录。操作员选择根目录，
节点生成并校验任务子目录；不得直接拼接未经校验的任务 ID 形成路径。第一版使用
`<output_root>/<safe-task-id>/r<revision>_<UTC-time>_<run-id>/`，同一 revision 重跑也必须
产生新目录，绝不覆盖旧数据。

建议的第一版目录契约：

```text
output_root/
└── safe-task-id/
    └── r000123_20260825T103015Z_run-id/
        ├── manifest.json
        ├── calibration/
        │   ├── camera_info.yaml
        │   ├── camera_extrinsics.yaml
        │   └── flat_field_reference.json
        ├── images/raw/000000.png
        └── metadata/000000.json
```

单张标签以 `(task_id, revision, segment_index, trigger_index)` 为业务主键，以图像 header
为曝光配对键。PNG 必须是原始畸变 `mono8` 像素的无损编码，并记录文件 SHA-256；标签
保存墙面相机位置、航向、完整协方差、目标／实际沿轨位置和所有标定哈希。`manifest`
保存任务区域、规划、软件提交、名义容量预估、冻结扫描参考、最终计数和失败清单。名义
计数只能用于开始前预留空间；每个首次生效的正式冻结 SCAN 参考按
`ceil(L / (detection_length × (1 − overlap)))` 累加最终预期数，并且必须与已归档照片数
严格相等；任务完成时也必须已经见到任务全部 SCAN 段的冻结参考。触发点不要求落在终点
精确位姿，最后一个点保留在终点前一个间隔内，以适配控制器的终点容差。只有图片和标签
均经临时文件写入、校验并原子改名后才增加成功计数；任务结束时必须验证主键、文件名和
时间戳一一对应。

不完整的 `image_raw`／`CameraInfo`／`InspectionCapture` 三元组在 `pair_timeout_s` 后仅计入
`failed_images` 与 manifest 的失败清单，记录器保持 `RECORDING`，使上游有机会补拍；是否可
以 `COMPLETED` 封存仍由冻结段、成功张数和实际几何三项最终裁决。每段相邻标签的
`actual_along_track_m` 间距不得超过 `detection_length × (1 − overlap)` 加
`actual_spacing_tolerance_m`，且单帧相对 `target_along_track_m` 的后滞不得超过
`maximum_target_lag_m`。这些是归档时的运行时质量门，不只是离线 G2 评估指标。
自动采集器只用 heartbeat 监督采集链路存活；记录器的位置与间距门仍是最终独立裁决，
不能以控制器是否收到 heartbeat 代替。

`/coverage/manager_status` 为这两个口径分别提供
`archive_preflight_expected_images` 和 `archive_expected_images`：前者是开始前固定的名义
全任务容量预估，后者是目前已冻结 SCAN 参考的累计实际计划。RViz 必须同时标示两者，
不得把尚未冻结全部扫描线时的后者显示成任务总预计数。

### G4 任务启动与 RViz 接口

现有 `/coverage/start` 不携带任务选项。G4 第一版已在 `climbot_interfaces` 增加一个
由管理器提供的带选项启动服务 `/coverage/start_configured`
（`climbot_interfaces/srv/StartCoverage`），请求至少携带 `inspection_enabled` 和
`output_root`。响应只确认“管理器已经接受这次准备请求”：归档准备是异步的，不能在 ROS
服务回调中阻塞等待另一个服务；成功后的 `run_id`、记录器解析出的绝对任务目录和错误
说明以 `/coverage/manager_status` 为权威。RViz 面板只调用这个管理器接口，管理器内部
完成“准备归档 → 成功后发送执行 Goal”的顺序，不允许 UI 自己拼接两次服务调用。旧
`/coverage/start` 保留给命令行和兼容测试，使用管理器参数中的
`inspection_default_enabled` 与 `inspection_output_root`：裸 `coverage_manager_node`
默认 motion-only，完整 `coverage_mission.launch.py` 显式传入 `inspection:=true` 和默认
根目录由 `CLIMBOT_DATA_ROOT` 或显式 launch 参数提供。具体字段可扩展，但这套异步与权威状态
语义不得改变。

管理器状态将聚合记录器的权威状态，供面板显示：是否启用、`PREPARING`／`READY`／
`RECORDING`／`FINALIZING`／`COMPLETED`／`CANCELED`／`FAILED`、预计／成功／失败照片数、
最终目录和最后错误。采集启用时，归档准备失败必须使 Start 失败且不发送运动 Goal；
执行中发生不可恢复的归档错误默认请求受控取消。取消或运动故障只改变 manifest 结果，
不得删除已经提交的照片。

归档封存 RPC 使用 `archive_finalize_timeout_s`（默认 `5.0 s`）这一稳态时间期限。记录器在
收到请求后消失、因而永远不回响应时，管理器将归档记为 `FAILED`、保留已经写入的部分目录，
并重新允许操作员启动下一项任务；迟到响应以代次隔离，不能覆盖这一失败结论。准备阶段在
创建 run 目录后若标定快照或首份 manifest 写入失败，则删除尚未发布的半成品 run 目录。

RViz 仍使用一个 `Coverage Task` dock，布局为：顶部公共状态和采集摘要；中间三个页签
`任务规划`、`巡检采集`、`详情`；底部固定 Start、Cancel / Stop。恢复操作只在管理器进入
对应异常状态时临时显示，平时不占用任务面板高度。采集页提供
任务级开关、根目录编辑／浏览／恢复默认、预计照片数量与空间、保存计数、最终目录和
错误。Start 后这些设置冻结，直到本次任务封存。路径由记录器所在主机解释；分机部署时
本地文件选择器只是一项便利，记录器返回的绝对路径才是权威结果。

离线 `climbot_image_processing` 先在畸变原图坐标中应用暗场／平场，再去畸变；
`climbot_mosaic` 以 EKF 位姿和协方差为绝对先验，以重叠特征匹配为相对约束做鲁棒位姿
图优化或束调整。优化目标是加权残差而不是直接压低协方差；输出必须附残差、内点率、
连通性和后验不确定度。

### 离线预处理目录接口（`climbot_image_processing`）

第一版不是 ROS topic 或 service，而是一个离线、单向的目录接口：

```bash
ros2 run climbot_image_processing process_inspection_archive \
  --input-run <绝对的、completed G4-run> \
  --output-dir <绝对的、不存在的独立目录> \
  [--flat-field-file <gain.npz>] [--dark-frame <dark.png>] \
  [--denoise none|median3] [--jobs auto|N] \
  [--memory-budget-gb <有限正数>]
```

输入 run 必须通过 `manifest.json`、所有 PNG SHA-256、每张标签、`camera_info.yaml` 和原始
`mono8` 尺寸的一致性验证。默认拒绝 canceled／failed／计数不一致的 run；仅取证用途可显式
传 `--allow-incomplete`。输出目录不得存在、不得在输入 run 内；所有处理完成后才原子发布，
所以失败不会留下表面完整的处理结果。输入目录只读，不会被重写或移动。

输出目录契约为：

```text
processed-run/
├── processing_manifest.json
├── calibration/camera_extrinsics.yaml
├── calibration/rectified_camera_info.yaml
├── images/000000.png
└── metadata/000000.json
```

每个输出标签保留 G4 原始标签，并添加 `source_label_file`、处理后图像相对路径与 SHA-256、
以及暗场／平场／去噪／去畸变操作记录。`processing_manifest.json` 记录源 run ID、源
manifest SHA-256、输出帧链、平场／暗场哈希和 rectified `K`，但不记录机器本地绝对输入路径。
处理顺序固定为原始像素坐标的暗场减法、平场 gain、可选去噪、`plumb_bob` 去畸变；输出
`rectified_camera_info.yaml` 的 `D` 全为零，且 `K/P` 都更新为 rectified 内参；冻结外参快照
原样复制，供后续 `climbot_mosaic` 使用。预处理通过每图多进程并行；OpenCV 去畸变 remap
在主进程只计算一次，Linux/WSL 子进程以只读 fork 写时复制继承。`--jobs=auto` 在 CPU、
显式内存预算和 8 worker 上限之间选择；每个 worker 仅使用一个 OpenCV 内部线程。manifest 的
`execution` 记录实际 worker 数、内存预算、帧处理耗时和预计算 remap 类型；帧和 manifest
始终按稳定帧序发布，不能因并发数不同改变结果。

## `climbot_mosaic` 离线输入预检

```bash
ros2 run climbot_mosaic validate_mosaic_inputs \
  --input-run <绝对 processed-run 路径> \
  [--input-run <第二个绝对 processed-run 路径> ...]
```

该命令是墙面拼接的首道只读门禁：输入必须是已经完成的 processed-run；它逐张校验处理图
SHA-256、标签—manifest 对应关系、`mono8` 尺寸、去畸变后的零 `D`、冻结 optical mount、有限的
曝光相机位姿和 `6×6` 协方差。第一版平面墙还要求所有输入 run 的相机内参和安装快照完全一致，
并以 `(source_run_id, decimal_frame_index)` 作为跨任务唯一帧键。任何一项失败均退出码 `2`，不写
任何目录；成功时只输出严格 JSON 摘要，包含 run 数、帧数、源 run ID、处理 manifest SHA-256、任务
ID 和相机签名，但不泄露机器本地绝对路径。P2.3 及后的投影、匹配和融合只接收这个预检的结果。

## `climbot_mosaic` P2.3 初始平面投影

```bash
ros2 run climbot_mosaic build_initial_projection \
  --input-run <绝对 processed-run 路径> \
  [--input-run <第二个绝对 processed-run 路径> ...] \
  --output-dir <绝对且不存在的输出目录> \
  [--wall-plane-z-m 0.0] [--preview-max-side-px 1600]
```

该离线命令复用上述预检，再将每张 rectified 图的 ROS 光学射线（`+Z` 向前、`+X` 向右、`+Y`
向下）用归档的光学中心 EKF 位姿与单位四元数变换到 `wall`，和显式的平面 `z=0` 求交。输出目录
必须不存在且不能位于任一输入 run 内；全部写入临时兄弟目录，只有 JSON 和 PNG 预览均成功后才
原子命名为正式目录。`initial_projection.json` 是 host-path-free 的严格 JSON，记录输入预检摘要、
平面、所有图的 `3×3` image→wall 单应矩阵、按像素角点顺序排列的米制足迹、光学中心落点、
相机—平面距离及总体范围；`initial_footprints_preview.png` 只画足迹轮廓，绝不重采样或融合源图。
相机不朝向墙面、射线平行、交点在相机背后、零面积或任何非有限值均为失败，退出码 `2`。

## `climbot_mosaic` P2.4a 空间重叠候选

```bash
ros2 run climbot_mosaic build_overlap_candidates \
  --input-run <绝对 processed-run 路径> \
  [--input-run <第二个绝对 processed-run 路径> ...] \
  --output-dir <绝对且不存在的输出目录> \
  [--min-overlap-area-m2 0.0]
```

命令复用 P2.2 预检和 P2.3 投影，在 `wall` 平面中先以 sweep-line 包围盒筛选，再对凸四边形
做精确裁剪。默认 `0.0` 只排除相切或分离足迹，保留所有正面积候选给后续特征证据裁决；可选
参数是显式算法实验项，不是验收门限。原子输出的 `overlap_candidates.json` 包含输入摘要、
稳定排序的全局帧键对、交集多边形、面积和相对较小足迹的重叠率，不包含机器绝对输入路径。

## `climbot_mosaic` P2.4b 局部视觉约束

```bash
ros2 run climbot_mosaic build_local_matches \
  --input-run <绝对 processed-run 路径> [--input-run <更多 run> ...] \
  --output-dir <绝对且不存在的输出目录> \
  --work-dir <绝对缓存目录> [--jobs auto|N] [--use-clahe]
```

命令重新执行同一预检、投影和候选构造，在预测重叠内做 SIFT、双向 ratio test、RANSAC
与固定尺度 SE(2) 拟合。每条候选都写成 `accepted` 或带明确原因的 `rejected`；特征缓存由
处理图 SHA-256 和算法参数寻址，损坏缓存必须失败，不能静默重算。并发只影响耗时，不得
改变帧、候选、随机种子或输出排序。

## `climbot_mosaic` P2.5 全局位姿图

```bash
ros2 run climbot_mosaic build_pose_graph \
  --input-run <绝对 processed-run 路径> [--input-run <更多 run> ...] \
  --local-matches <local_matches.json> \
  --output-dir <绝对且不存在的输出目录> \
  [--edge-recheck-floor-m 0.005]
```

每帧变量仅为相对曝光 EKF 位姿的 `δx/δy/δyaw`，绝对先验使用归档 `6×6` 协方差的
`x/y/yaw` 子矩阵；视觉边要求同一重叠中心经两张图的修正后在墙面坐标重合。求解顺序为
线性一致初值、`soft_l1` 鲁棒第一轮、按实际残差复核边、`soft_l1` 最终轮，使用解析稀疏
Jacobian。输出 `initial_poses.json`、`optimized_poses.json` 和 `pose_graph.json`，后者记录
优化前后米制残差、修正量、连通分量、保留边及后验标准差近似。孤立照片仍由 EKF 先验
定义位姿，但必须在连通分量和后续不确定度中显式暴露。

