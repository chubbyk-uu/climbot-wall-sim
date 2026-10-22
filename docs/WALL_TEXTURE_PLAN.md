# 墙面纹理：当前资产与验证边界

更新：2026-10-22。局部减复用专项已于 2026-10-19 收尾，原实施计划见
[归档](archive/plans/WALL_TEXTURE_PLAN_2026-10-19.md)。当前方法合同见
[局部减复用协议](WALL_TEXTURE_PROTOCOL_CURRENT.md)；v1/v2 均为被替代的历史试验。

## 当前资产

| 用途 | 数据根下的入口 |
| --- | --- |
| 当前 M2 采集墙 | `assets/wall_local_m2_v6/wall_texture.json` |
| 平墙诊断对照 | `assets/wall_local_diagnostic_v6/wall_texture.json` |
| 无覆盖物底纹 | `assets/wall_local_v6/wall_texture.json` |

单独模型和规划预览可不加载贴图；采集入口根据表面选择同源资产。生成配方只在
[资产安装](INSTALLATION.md#4-从源码生成资产)维护。旧贴图用于历史 P2／S6／S7 复查，不覆盖。

## 方法与证据

单张 Concrete044D 8K、250 mm 裁块、D4 八方向和局部来源排除，减少近邻人工复制；
允许远处偶尔重复，不以零错配或全墙唯一作为目标。随机变换不改变诊断文字和物理槽的坐标。
保持相机分辨率与米／像素尺度，不增加噪声或密集标记制造容易匹配的纹理。

- [局部配置验证](../results/wall_texture_local_2026-10-18/REPORT.md)：三个种子各 40/40 正重叠与 10/10 弱纹理对。
- [代表性与全墙采集](../results/wall_texture_capture_2026-10-19/REPORT.md)：70＋680 张，CUDA 与有限目视检查；边缘覆盖仍未闭合。
- [独立审核](../results/wall_texture_review_2026-10-19/REPORT.md)：DDS／mip 共区与默认入口检查。
- [早期失败试验](../results/wall_texture_randomization_2026-10-18/REPORT.md)：按原门禁保留失败，不按后续收敛口径追认通过。

平墙通过不自动覆盖 M2 曲面投影；M2 接缝／覆盖的本轮停止决定见[状态页](STATUS.md)。
主视图偶发频闪暂搁置，不因未复现宣称修复。资产、来源 sidecar 与清理回执留在本地，
实际保留范围见[数据保留](DATA_RETENTION.md)。
