# climbot_image_processing

离线链的第一级：把一个已封存的 G4 原始归档校正成可供拼接读取的 processed-run。

本包没有 ROS 图、Gazebo 或控制器依赖，`package.xml` 也不依赖任何项目包。它只做一件事——
读一个已完成的归档，写一个新目录，**从不修改原图和标签**。

## 命令

```bash
ros2 run climbot_image_processing process_inspection_archive \
  --input-run "$CLIMBOT_DATA_ROOT/<task-id>/<run-id>" \
  --output-dir "$CLIMBOT_DATA_ROOT/processed-<new-id>" \
  --flat-field-file "$CLIMBOT_DATA_ROOT/calibration/<flat-field>.npz" \
  --dark-frame "$CLIMBOT_DATA_ROOT/calibration/<dark>.png" \
  --denoise none --jobs auto --memory-budget-gb 4
```

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `--input-run` | 必填 | 已封存的 G4 归档目录 |
| `--output-dir` | 必填 | **绝对路径、必须不存在**，且不能落在输入 run 里面 |
| `--flat-field-file` | 无 | NPZ 平场增益；SHA-256 必须与 `config/inspection_flat_field.yaml` 一致 |
| `--no-flat-field` | 不启用 | 明确声明这一次不做平场校正 |
| `--allow-unpinned-flat-field` | 不启用 | 用一份未被钉住的平场处理 |
| `--dark-frame` | 无 | `mono8` 暗场帧；省略则跳过暗场 |
| `--denoise` | `none` | `none` 或 `median3` |
| `--jobs` | `auto` | `auto` 或正整数进程数 |
| `--memory-budget-gb` | `4.0` | 内存预算，约束 `auto` 的实际并发 |
| `--allow-incomplete` | 不启用 | 只用于明确的取证运行，见下 |

平场既不能省略也不能随便换：省略 `--flat-field-file` 或给一份 SHA-256 与
`config/inspection_flat_field.yaml` 对不上的文件，命令会在产生任何输出之前拒绝，并把记录的
文件名和摘要打出来。摘要用于身份判定，文件名用于提示，不要求实际文件沿用同一个名字。两个覆盖
开关都要显式给。这道闸的由来是一次沉默失效——P2-06 有一周是被
一份靶面材质与被摄墙不同的标定校正的，全链没有任何东西报警，最后是人工看拼接图时看出亮度
接缝才发现，那份校正在画面上留了 9% 的碗形残差。钉住的那一份换代时，同一次提交里要说明相机、
LED、曝光或被摄材质发生了什么变化，使旧的那份不再成立。

## 处理顺序

顺序是固定的，不可调换：

1. 校验归档 manifest、每张源 PNG 的 SHA-256、标签和相机标定；**全部通过后才创建输出目录**；
2. 在畸变的传感器像素上减暗场，再乘平场增益（两步都可选）；
3. 可选 `median3` 去噪；
4. 用归档里的 `plumb_bob` `K/D` 做去畸变。

平场必须作用在**去畸变之前**的传感器像素上：增益是镜头和 LED 在物理像元上的响应，
去畸变之后像元已经被重采样过，再乘增益就对不上了。

## 输出

```text
<output-dir>/
  images/                              校正后的图像
  metadata/                            复制并扩展的逐图标签
  calibration/camera_extrinsics.yaml   冻结的安装快照
  calibration/rectified_camera_info.yaml  去畸变后的相机标定（零畸变）
  processing_manifest.json             严格 JSON 处理记录
```

`processing_manifest.json` 记录 source/output 双向的 SHA-256 链接和全部处理参数，
但**有意不记录本机的绝对输入路径**；`execution` 段里是这次实际用了几个 worker、
每帧花了多久。

## 契约

- 默认只接受 manifest 中 `outcome` 为 `completed` 的归档。`--allow-incomplete` 只供明确的
  取证运行使用，不能拿它去“抢救”一次失败的采集——半个 run 的计数本来就不可信。
- 输出目录必须是绝对路径、必须不存在、而且不能落在输入 run 里面。这三条是为了把
  “原始归档不可改写”做成文件系统层面的事实，而不是一句约定。
- 并发只改变耗时，不改变结果：相机 remap 只构建一次，再以只读方式装进每个 worker；
  每个 worker 把 OpenCV 的内部线程数限为 1，免得进程并行和线程并行互相抢 CPU。

## 测试

```bash
colcon test --packages-select climbot_image_processing
colcon test-result --verbose
```

`test_processing_chain.py` 用纯 Python 造一个可重复的小型 G4 归档，验证三件事：原图哈希
不变、损坏的输入会被拒绝、输出目录不越界。

下游是 [climbot_mosaic](../climbot_mosaic/README.md)；阶段设计与验收门禁见
[拼接计划](../../docs/MOSAIC_PLAN.md)。
