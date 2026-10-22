# 离线 CUDA 后端：当前合同

更新：2026-10-22。Fusion CUDA 专项已完成；开发计划和否决路线见
[原设计合同](archive/gpu/DESIGN_2026-09-01.md)与[实施结果](archive/gpu/RESULTS_2026-09-01.md)。
本页不涉及 Gazebo／RViz 图形后端，WSL 图形运行库见[Mesa](MESA_SETUP.md)。

## 当前实现

| 阶段 | 当前后端与状态 |
| --- | --- |
| P1 预处理 | CPU；CUDA OpenCV 原型端到端更慢，已撤回 |
| 局部特征／匹配、位姿图 | CPU；未采用 CUDA SIFT，不自动开展下一专项 |
| `build_wall_mosaic` hard-cut fusion | 可选 CPU／CUDA，使用自定义核保持现有离散坐标与归属规则 |

同一 P2-06 1340 帧的历史三轮 A/B：fusion 中位数 CPU 91.86 s／CUDA 47.73 s（1.92×），
全命令 108.01／64.02 s（1.69×）。这是冻结输入的历史性能，不是所有任务的速度承诺。

## 后端与数值合同

- CLI 默认 CPU；显式 CUDA 不可用应失败，`auto` 可回退但不能作为 GPU 验收证据。
- GPU 正式作业用 `--backend cuda`，结束后读取原生 manifest 的
  `execution.effective=cuda`、`execution.fallback=false`；不在摘要中补写后端字段。
- coverage、owner、空洞、掩码与离散几何逐位一致；连续灰度量按冻结合同比较。
- 原图和 processed 输入不可变，输出新目录原子发布，失败不得发布 completed。
- CPU 仅作为明确的基线／对照；普通单元测试不依赖本机 CUDA 安装。

安装见[CUDA 部署](INSTALLATION.md#可选安装-cuda-拼接后端)，完整离线主线见
[快速启动](QUICKSTART.md#6-离线墙面拼接)，参数见[mosaic 包](../src/climbot_mosaic/README.md)。
