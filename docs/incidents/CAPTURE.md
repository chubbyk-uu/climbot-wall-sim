# 故障案例：CAPTURE

入口与新增规则见[故障索引](../INCIDENTS.md)。

### 10. 无 request ID 的 receipt 会串到下一次曝光

- **现象**：某段服务成功后 receipt 投递超过等待期限，迟到 receipt 被下一段第一个 pending
  请求接收，最终因标签几何不匹配白丢一帧。
- **隐蔽性**：超时失败路径有 DRAINING，会丢迟到帧；真正可达的是“服务已成功、topic 仍迟到”
  这条较窄路径。扩大 QoS 队列反而让旧 receipt 更容易存活。
- **修复**：`CaptureOnce` 响应直接返回曝光 Header；Future 天生与请求关联，不再维护第二套
  topic 身份协议。归档的图像/metadata 配对和最终计数仍是交付权威。
- **回归**：automatic capture 的成功、零时间戳和迟到响应测试；提交 `87fa907`。

### 11. 耐久批处理优化扩大了完整性盲区，`syncfs` 计时又被采样周期错开

- **现象**：逐图 `fsync` 导致磁盘 I/O 尖峰；改为每 32 张提交后，写后回读校验被一并删掉，
  manifest 的 SHA-256 只证明内存字节。与此同时每 50 张 INFO 与每 32 张 commit 在 660 帧任务
  中从不重合，21–23 ms 的 commit 实际完全不可见。
- **根因与修复**：图像/标签保持原子写，逐图原位回读校验摘要；每 32 张 `syncfs` 推进耐久计数，
  并单独记录每次批大小、耗时和 run 最大值。注意 `syncfs` 刷整个文件系统，批大小只限制频率，
  不限制单次停顿。
- **回归**：archive core/recorder 的损坏、批次与 manifest 指标测试；提交 `85679bb`、`c396d0a`、
  `1eb3197`。
- **经验**：性能优化删除校验时，必须重新写出数据合同；周期采样不能替代事件本身的计时。

### 12. Fast DDS 共享内存段不足造成数秒级可靠重传等待

- **现象**：长采集中单帧偶发等待 `0.6–4.0 s`，不一定每次任务出现；一旦高频压测进入劣化，
  后续 237 次中 64 次停顿。机器人、渲染和小话题仍正常。
- **误判过程**：先后被归因为 Gazebo 渲染、GUI/D3D12、主机负载和共享 bridge 队头阻塞；这些
  相关现象都不能解释 trigger→Gazebo 正常而 gz→ROS 接近 3 秒。
- **根因**：一帧 RGB 图为 6,220,800 bytes，默认 SHM 段约 512 KiB。持续分片写入耗尽段缓冲、
  丢一个片段后，可靠读端要等默认 3 秒周期心跳发现缺口。
- **修复**：三个实际大图发布者使用 64 MiB `fastdds_inspection_image.xml`；不能只保护第一跳，
  mono8 适配器和 `capture_once_node` 仍是大图 writer。操作者自带 profile 时明确由其覆盖。
- **回归**：`test_large_image_transport.py` 静态检查所有 writer、profile 大小和 launch 注入；
  动态证据为 46,300 次曝光零慢捕获及完整纵向 `660/660`。提交 `9ff0e7a`、`8b8181f`。
- **经验**：新增超过 512 KiB 的可靠话题时，检查写端 SHM，不要只看 DDS reader 或平均 CPU。

### 46. 验收轮载记录器在 GUI reset 时被重建

- **现象**：T3 双向转向检查完成后，reset 使服务器报 `load trace already exists` 并退出；尚未派发采集。
- **根因**：观察插件未实现 `ISystemReset`，Gazebo 重建插件并再次执行拒绝覆盖文件的 Configure。
- **修复**：实现 reset 接口，刷新并保留同一记录流；时钟回退与 generation 均保留，不覆盖既有记录。
- **验证**：同一整墙模型、双 GUI 下真实 reset，服务接受、generation 递增、临时支撑重施后撤除、
  风机重新升速、负压恢复及无物理失败九项检查全部通过；新一代记录继续写入。

