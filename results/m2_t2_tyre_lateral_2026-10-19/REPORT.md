# M2 / T2：停车侧向锁死与轮胎侧向弹性

2026-10-19。**停车保持的低裕量已消除；T2 仍未整体验收**（运动窗口的严格逐帧检查仍有毫秒级瞬态，见下）。
新开关 `tyre_lateral` 默认关闭，必须与 T1 法向柔顺轮同时开启；默认场景仍为 M1。
不改 1.5 裕量门槛、摩擦、风机或滚阻，按原始逐帧判定。

## 根因

WheelSlip 的滑移柔度随车轮转速缩放，车轮停转时侧向变为完全刚性。起伏墙上，停转瞬间两轮各自
残留的侧向滑移被刚性摩擦截停，两轮锁成任意的对顶内力，常常顶到摩擦极限。若球轮恰好停在纵缝
谷底，单侧斜面约 98 N（468 N×tan 11.5°）会把这个状态撑住数秒。证据：

- 停转后约 20 ms 内侧向力从 +53／−16 N 跳到 +228／−129 N；平墙停车侧向力为 0、裕量 3.2。
- 球轮不在缝里的 `holdout_soft_c_clearance` 停车瞬间同样出现约 ±80 N 对顶，左轮裕量 1.02。
- 浮动球座只能处理球那一侧（`m2_t2_caster_float_2026-10-19` 中「停车初期低裕量仍在」）。

## 改动

每个驱动轮在 T1 法向弹簧和车轮之间插入沿轮轴方向的移动关节和弹簧，从原车轮拆出 0.02 kg 作为
载体，总质量不变；SDF／URDF 链路一致，关节加入 JointStatePublisher。参数为暂定代理值，
不是实机测量：100 N/mm、700 N·s/m（整车约 0.4 倍临界阻尼）、行程 ±5 mm（不会顶到限位）。

## 结果（1 ms，私有 hashfix＋CCD ODE，C 模式与原来源和动作一致）

| 指标 | 原 `rough_soft_c` | 加侧向弹性 |
| --- | ---: | ---: |
| 停车后 64 s 左／右最小裕量 | 1.09／1.08 | **3.01／3.05** |
| 停车后低于 1.5 的帧数 左／右 | 790／6817 | **0／0** |
| 停车球轮侧向力最大值 | 99.1 N | 1.6 N |
| 保持期逐帧需求（校验器 hold） | 不满足 | **满足** |
| 运动窗口驱动轮违规帧 左／右（最长连续） | 819／819（800） | 5／13（4） |
| 运动窗口压差 | 7.053～7.923 kPa | 6.992～7.956 kPa |

| 指标 | 原 `holdout_soft_c_clearance` | 加侧向弹性 |
| --- | ---: | ---: |
| 停车左／右最小裕量 | 1.02／2.56 | **2.23／4.26** |
| 运动窗口驱动轮违规帧 左／右（最长连续） | 433／15（365） | 52／8（13） |

平墙直行／转向配对（同为代理球＋柔顺轮，比较有无侧向弹性）通过：最大位置差 1.64 mm、
姿态差 0.085°，均在原配对容差内；两组整车运动门槛、越缝检查均通过。

## 尚未通过的项目

运动窗口中剩余的严格逐帧失败都是毫秒级瞬态：单步失触（法向为 0）、起步加速段和制动开始时
1～13 ms 的尖峰；球轮 `ball_raw_floor` 失败也都是单步失触。它们在原方案中同样存在，与停车锁死
无关，需要单独决定瞬态判据或处理速度指令拐点，不在本次范围内。

## 复现

```bash
python3 tools/probe_m2_robot_patch.py --output "$CLIMBOT_DATA_ROOT/<new>" \
  --ode-prefix "$CLIMBOT_ODE_PREFIX" --soft --tyre-lateral --native-heightmap \
  --mode fan_ode --channels --duration-s 70
python3 tools/validate_m2_caster_rolling.py --output "$CLIMBOT_DATA_ROOT/<new>" \
  --ode-prefix "$CLIMBOT_ODE_PREFIX" --tyre-lateral-pair
python3 tools/validate_m2_coupling.py --data-root "$CLIMBOT_DATA_ROOT/m2_t2_20261019" \
  --case tyre_lateral_rough_soft_c --output "$CLIMBOT_DATA_ROOT/<new>.json"
```

对照组追加 `--heading-deg 45 --seed 20261020 --patch-size-m 1.6 --drape-m 0 --duration-s 9`。
原始数据在 `$CLIMBOT_DATA_ROOT/m2_t2_20261019/tyre_lateral_*`，摘要见 [summary.json](summary.json)。
最终模型冻结后仍需复核转向补偿 0.46 mm/° 和横向重力侧滑比 0.1042。
