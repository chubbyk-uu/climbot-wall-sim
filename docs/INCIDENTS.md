# 隐蔽故障与排查经验

本页按案例编号索引已确认故障；未定位现象留在[当前状态](STATUS.md)。

## 分类入口

- [TESTING](incidents/TESTING.md)
- [ACTIONS_TIME](incidents/ACTIONS_TIME.md)
- [CAPTURE](incidents/CAPTURE.md)
- [MOSAIC_RENDERING](incidents/MOSAIC_RENDERING.md)
- [PHYSICS](incidents/PHYSICS.md)
- [RUNTIME](incidents/RUNTIME.md)

## 案例索引

| 编号 | 故障与回归记录 |
| --- | --- |
| 1 | [ROS domain 冲突把另一用例的数据当成自己的](incidents/TESTING.md#1-ros-domain-冲突把另一用例的数据当成自己的) |
| 2 | [固定睡眠不是 DDS discovery 完成条件](incidents/TESTING.md#2-固定睡眠不是-dds-discovery-完成条件) |
| 3 | [把循环次数当秒数，既会过早失败也会无限拖长](incidents/TESTING.md#3-把循环次数当秒数既会过早失败也会无限拖长) |
| 4 | [`-j8` 失败不能归类为“时间噪声”](incidents/TESTING.md#4--j8-失败不能归类为时间噪声) |
| 5 | [活跃 Goal handle 在析构中发布结果并终止进程](incidents/ACTIONS_TIME.md#5-活跃-goal-handle-在析构中发布结果并终止进程) |
| 6 | [服务 Future 退休了，迟到 topic 仍能改写终态](incidents/ACTIONS_TIME.md#6-服务-future-退休了迟到-topic-仍能改写终态) |
| 7 | [WSL 系统时间回跳同时冻结控制器和看门狗](incidents/ACTIONS_TIME.md#7-wsl-系统时间回跳同时冻结控制器和看门狗) |
| 8 | [暂停恢复不能只把原运动曲线的时钟向后平移](incidents/ACTIONS_TIME.md#8-暂停恢复不能只把原运动曲线的时钟向后平移) |
| 9 | [参考心跳与采集门超时单边调参会造成“相机停、机器人仍走”](incidents/ACTIONS_TIME.md#9-参考心跳与采集门超时单边调参会造成相机停机器人仍走) |
| 10 | [无 request ID 的 receipt 会串到下一次曝光](incidents/CAPTURE.md#10-无-request-id-的-receipt-会串到下一次曝光) |
| 11 | [耐久批处理优化扩大了完整性盲区，`syncfs` 计时又被采样周期错开](incidents/CAPTURE.md#11-耐久批处理优化扩大了完整性盲区syncfs-计时又被采样周期错开) |
| 12 | [Fast DDS 共享内存段不足造成数秒级可靠重传等待](incidents/CAPTURE.md#12-fast-dds-共享内存段不足造成数秒级可靠重传等待) |
| 13 | [`dxg` 总数和启动噪声不能证明 GPU 驱动故障](incidents/RUNTIME.md#13-dxg-总数和启动噪声不能证明-gpu-驱动故障) |
| 14 | [图像彼此对齐，不代表绝对比例正确](incidents/MOSAIC_RENDERING.md#14-图像彼此对齐不代表绝对比例正确) |
| 15 | [验收可以在“什么都没量”时真空通过，手填 provenance 也可以自相矛盾](incidents/MOSAIC_RENDERING.md#15-验收可以在什么都没量时真空通过手填-provenance-也可以自相矛盾) |
| 16 | [平场流程正确，仍可能因标定靶材质不同而欠校正](incidents/MOSAIC_RENDERING.md#16-平场流程正确仍可能因标定靶材质不同而欠校正) |
| 17 | [复用节点却重置测试时钟，失败断言还会留下活跃任务](incidents/TESTING.md#17-复用节点却重置测试时钟失败断言还会留下活跃任务) |
| 18 | [测试客户端发现 Action server，不代表管理器内部也已发现](incidents/TESTING.md#18-测试客户端发现-action-server不代表管理器内部也已发现) |
| 19 | [原生 Gazebo 测量夹具在 reset 后不再执行 PreUpdate](incidents/TESTING.md#19-原生-gazebo-测量夹具在-reset-后不再执行-preupdate) |
| 20 | [等待首个采集心跳时反复发布参考，污染后续“无心跳”断言](incidents/TESTING.md#20-等待首个采集心跳时反复发布参考污染后续无心跳断言) |
| 21 | [时间控制将新计划与旧时间戳位姿比较，产生虚假追赶](incidents/ACTIONS_TIME.md#21-时间控制将新计划与旧时间戳位姿比较产生虚假追赶) |
| 22 | [APPEND_ENV 拼接 ROS_DOMAIN_ID，破坏测试域隔离](incidents/TESTING.md#22-append_env-拼接-ros_domain_id破坏测试域隔离) |
| 23 | [GUI 观察器在正常收尾期间误报窗口退出](incidents/RUNTIME.md#23-gui-观察器在正常收尾期间误报窗口退出) |
| 24 | [状态观察窗口没有维持合成里程计，触发真实过期保护](incidents/TESTING.md#24-状态观察窗口没有维持合成里程计触发真实过期保护) |
| 25 | [自发光标靶叠加反射，且亮度检查遗漏既有栅格边缘 allowance](incidents/MOSAIC_RENDERING.md#25-自发光标靶叠加反射且亮度检查遗漏既有栅格边缘-allowance) |
| 26 | [大纹理冷启动中服务先于时钟／定位就绪，任务提前接受](incidents/RUNTIME.md#26-大纹理冷启动中服务先于时钟定位就绪任务提前接受) |
| 27 | [密封膨胀的预测上端点因乘除舍入被误判为无括区](incidents/PHYSICS.md#27-密封膨胀的预测上端点因乘除舍入被误判为无括区) |
| 28 | [状态夹具在新传感器端点匹配前凭缓存位姿启动](incidents/TESTING.md#28-状态夹具在新传感器端点匹配前凭缓存位姿启动) |
| 29 | [吸附模型异常后真值停报，外部只剩健康旧快照](incidents/PHYSICS.md#29-吸附模型异常后真值停报外部只剩健康旧快照) |
| 30 | [首帧前 D3D12 上下文创建失败不能当成控制或实时率通过](incidents/RUNTIME.md#30-首帧前-d3d12-上下文创建失败不能当成控制或实时率通过) |
| 31 | [fork 后立即读取进程组，偶发漏记尚未完成 setsid 的子进程](incidents/RUNTIME.md#31-fork-后立即读取进程组偶发漏记尚未完成-setsid-的子进程) |
| 32 | [TERM 取消 ROS launch，双 GUI 收尾留下仍存活的 RViz](incidents/RUNTIME.md#32-term-取消-ros-launch双-gui-收尾留下仍存活的-rviz) |
| 33 | [SHM 发现端口检查持锁，单设 UDP 环境变量仍保留 SHM](incidents/RUNTIME.md#33-shm-发现端口检查持锁单设-udp-环境变量仍保留-shm) |
| 34 | [组级 INT 与 launch 转发向子进程重复发送退出信号](incidents/RUNTIME.md#34-组级-int-与-launch-转发向子进程重复发送退出信号) |
| 35 | [Bash 后台继承 SIG_IGN，使 launch 根进程收不到有序退出 INT](incidents/RUNTIME.md#35-bash-后台继承-sig_ign使-launch-根进程收不到有序退出-int) |
| 36 | [world reset 后 RViz 墙面、网格和路径消失](incidents/RUNTIME.md#36-world-reset-后-rviz-墙面网格和路径消失) |
| 37 | [退休模型的哈希守卫测试误读当前 main](incidents/TESTING.md#37-退休模型的哈希守卫测试误读当前-main) |
| 38 | [稀疏覆盖物的重压缩及局部多边形裁剪造成 gutter 差异](incidents/MOSAIC_RENDERING.md#38-稀疏覆盖物的重压缩及局部多边形裁剪造成-gutter-差异) |
| 39 | [ODE 哈希空间跨零回绕导致碰撞候选漏检](incidents/PHYSICS.md#39-ode-哈希空间跨零回绕导致碰撞候选漏检) |
| 40 | [原生高度场碰撞范围比声明尺寸小](incidents/PHYSICS.md#40-原生高度场碰撞范围比声明尺寸小) |
| 41 | [分叉试片重复量规互撞，接触误记为墙面误差](incidents/TESTING.md#41-分叉试片重复量规互撞接触误记为墙面误差) |
| 42 | [私有 ODE 链接 libccd 仍不等于发行版配置](incidents/PHYSICS.md#42-私有-ode-链接-libccd-仍不等于发行版配置) |
| 43 | [无响应 hold 的测试没有先建立故障条件](incidents/TESTING.md#43-无响应-hold-的测试没有先建立故障条件) |
| 44 | [起伏三角网下轮速正常、车体却明显降速](incidents/PHYSICS.md#44-起伏三角网下轮速正常车体却明显降速) |
| 45 | [密封边积分终点因舍入被丢弃，造成局部漏气低估](incidents/PHYSICS.md#45-密封边积分终点因舍入被丢弃造成局部漏气低估) |
| 46 | [验收轮载记录器在 GUI reset 时被重建](incidents/CAPTURE.md#46-验收轮载记录器在-gui-reset-时被重建) |
| 47 | [起伏墙 OBJ 的贴图纵坐标重复翻转，暴露纹理块边界](incidents/MOSAIC_RENDERING.md#47-起伏墙-obj-的贴图纵坐标重复翻转暴露纹理块边界) |
| 48 | [整墙冷缓存超出启动等待，留下半成品目录阻塞以后所有启动](incidents/RUNTIME.md#48-整墙冷缓存超出启动等待留下半成品目录阻塞以后所有启动) |
| 49 | [整墙双 GUI 下 RTF 约 0.6，看似 GPU 未启用](incidents/RUNTIME.md#49-整墙双-gui-下-rtf-约-06看似-gpu-未启用) |
| 50 | [采集脚本给 ros2 launch 传空参数，默认 G2 验收无法启动](incidents/RUNTIME.md#50-采集脚本给-ros2-launch-传空参数默认-g2-验收无法启动) |
| 51 | [起伏墙爬升航向摆动，看似控制增益或 WheelSlip 问题](incidents/PHYSICS.md#51-起伏墙爬升航向摆动看似控制增益或-wheelslip-问题) |
| 52 | [关闭补光灯投影后照片变暗并出现矩形亮区](incidents/MOSAIC_RENDERING.md#52-关闭补光灯投影后照片变暗并出现矩形亮区) |
| 53 | [系统时钟每约 32 秒被校一次，控制节点测试偶发提前结束](incidents/TESTING.md#53-系统时钟每约-32-秒被校一次控制节点测试偶发提前结束) |
| 54 | [每个物理步发布接触与关节状态，M2 实时率被同步开销吃掉](incidents/RUNTIME.md#54-每个物理步发布接触与关节状态m2-实时率被同步开销吃掉) |
| 55 | [旧定位包 header 乱序使离线相机插值给出错误结论](incidents/MOSAIC_RENDERING.md#55-旧定位包-header-乱序使离线相机插值给出错误结论) |

## 新增案例的规则

只有根因已由代码、受控 A/B 或明确运行证据确认，才记录为已解决。每例保留现象、误导线索、根因、修复提交及回归入口；无法确定性复现时注明判据与证据边界。并行测试失败按真实缺陷调查，不以降低并发或重跑通过代替修复。
