# Marine Planar Board 法向误差诊断

## 1. 问题定义

Marine 共面 ArUco Board 在固定正式 `4×3` safe-altitude shadow 矩阵中满足安全门 `12/12`，但 Planar Board 门仅 `9/12`。失败严格集中于 `static seed 1/2/3` 的当前 deck-normal 精度：

```text
current normal RMSE <= 1.0 deg
current normal P95  <= 1.5 deg
```

本诊断只回答一个问题：static 法向误差主要来自 Planar detector、PX4 姿态/时间转换，还是 downstream shadow estimator。此阶段不修改下降、接触、Future Twist、NMPC 或船体动力学。

## 2. 固定数据与安全边界

唯一正式历史数据源：

```text
results/deck_motion_shadow_planar_marine_local7m_20260813
```

固定 episode：

```text
static / rollpitch / combined / rigid_body_motion
× seed 1 / 2 / 3
```

分析入口：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash
python3 scripts/analyze_planar_board_orientation.py \
  --matrix-dir results/deck_motion_shadow_planar_marine_local7m_20260813
```

12 个 Bag 均能成功只读打开。Ground Truth 只在 `/aruco/pose`、`/landing/marker_pose_ned` 与 `/landing/deck_motion_shadow/state` 已经由在线系统生成后用于离线评分；分析脚本不向 detector、estimator 或 controller 发布任何数据。

历史 Bag 没有保存原始 Board corner 集合或原始相机图像，因此不能事后真正重跑 corner detector、`CORNER_REFINE_SUBPIX` 或 `solvePnPRefineLM`。历史 Bag 能支持的是输出位姿、重投影诊断、Marker 数、PX4/Gazebo 姿态与 Ground Truth 的因果后验比较。

## 3. 坐标系与时间基准

使用项目既有契约：

```text
camera_optical : +x right, +y down, +z forward
base_link_frd  : +x forward, +y right, +z down
local_ned      : +x north, +y east, +z down
```

raw Planar pose 满足：

```text
p_camera = R_camera_deck p_deck + t_camera_deck
```

离线 A 段通过 Gazebo UAV Ground Truth 姿态、固定相机外参和 raw `/aruco/pose` 重建 deck orientation NED。Gazebo UAV 的 ENU/FLU 姿态先转换为 NED/FRD。

时间处理：

- `/aruco/pose` 使用其 image header stamp 作为视觉采样时刻。
- deck/UAV Ground Truth 在该采样时刻插值，仅用于评分。
- PX4 `VehicleOdometry.timestamp_sample` 先按 Bag receipt 与 Gazebo UAV GT 对齐，使用中位 offset 映射到 ROS/Gazebo 时间，再比较图像时刻附近的 PX4 姿态。
- `marker_pose_ned` 与 raw pose、逐帧 Planar diagnostics 通过 Bag publish time 在 `50 ms` 内配对。

## 4. A/B/C 三段误差分解

定义：

```text
A = raw /aruco/pose
    + Gazebo UAV GT attitude
    + camera extrinsic
    -> deck orientation NED

B = /landing/marker_pose_ned

C = /landing/deck_motion_shadow/state
```

三者分别与 `/simulation/deck/ground_truth` 的 deck normal 比较。

### 4.1 static 三轮

| Episode | A raw+GT RMSE / P95 | B marker_pose_ned RMSE / P95 | C shadow RMSE / P95 |
|---|---:|---:|---:|
| static_s1 | 1.909° / 2.926° | 1.927° / 2.902° | 1.186° / 2.010° |
| static_s2 | 2.176° / 3.382° | 2.196° / 3.504° | 1.421° / 2.455° |
| static_s3 | 2.179° / 3.488° | 2.162° / 3.472° | 1.284° / 2.303° |

A 与 B 几乎一致，说明把 raw camera-frame Board pose 转成 NED 的在线 PX4 姿态/时间链路没有制造 static 的约 `2°` 法向误差。C 明显小于 A/B，说明 shadow 当前是在降低而不是放大 raw normal error。

### 4.2 九个动态 episode

九个动态 episode 的 A raw normal RMSE 为：

```text
0.885–1.073°
```

C shadow normal RMSE 为：

```text
0.453–0.549°
```

因此 static 与 dynamic 的主要差异已经存在于 detector 输出 A，进入 shadow 之前就出现。

## 5. true tilt 与法向误差

全矩阵 geometry-frame 级 Pearson 相关性：

```text
corr(raw normal error, true deck tilt)      = -0.5732
corr(raw normal error, reprojection RMSE)   = +0.9395
corr(raw normal error, camera distance)     = -0.2037
corr(raw normal error, center radius proxy) = -0.4602
```

### 5.1 全矩阵固定 tilt bins

| True deck tilt | Samples | raw normal RMSE | raw normal P95 |
|---|---:|---:|---:|
| 0–0.5° | 1937 | 2.064° | 3.296° |
| 0.5–1° | 72 | 1.168° | 2.487° |
| 1–2° | 145 | 1.072° | 2.189° |
| 2–3° | 1184 | 1.009° | 1.946° |
| >3° | 4125 | 0.922° | 2.016° |

### 5.2 rollpitch-only 固定 tilt bins

只看 `rollpitch`，排除 static 与其他动态 profile 混杂后：

| True deck tilt | Samples | raw normal RMSE | raw normal P95 |
|---|---:|---:|---:|
| 0–0.5° | 24 | 1.700° | 2.626° |
| 0.5–1° | 24 | 1.847° | 2.898° |
| 1–2° | 46 | 1.687° | 2.981° |
| 2–3° | 392 | 1.254° | 2.223° |
| >3° | 1380 | 0.860° | 1.564° |

rollpitch-only 仍显示近 fronto-parallel 明显更差，而真实倾角增大到 `>3°` 后显著改善。这排除了“全矩阵趋势只是 static 与 dynamic 场景混在一起”的解释。

## 6. PX4 attitude / time

图像时刻的 PX4 body-normal 相对 Gazebo UAV GT 姿态 RMSE：

```text
rollpitch_s1/s2/s3 : 0.110° / 0.119° / 0.087°
static_s1/s2/s3    : 0.092° / 0.093° / 0.089°
```

最近 PX4 sample 与 image time 的 P95 gap 约 `7–8 ms`，最近 GT sample gap P95 为约 `8 ms`；PX4→ROS offset residual P95 在这些轮次约 `8–12 ms`。

这些量级远小于 static A/B 的 `~2°` normal RMSE，而且 static 并不比 rollpitch 更差。因此 PX4 attitude/time 不是 static 法向误差主因。

## 7. Marker count

在 A/B 共同样本集合中，四个 scenario 的 `marker_count` 均为 `4`，因此 frame-level correlation 无方差、不可定义。static 的历史 evaluator 确实在视觉捕获早期看见过 2 Marker，例如 `static_s1` 的全部诊断帧中存在 `2` 与 `4` Marker，但 tracking 阶段均为 4 Marker。

因此“static 因 Marker 数更少而失败”与正式 tracking 数据不符。

## 8. 相机—Board 距离

共同样本的 camera-to-board distance：

| Scenario | median | P95 | max |
|---|---:|---:|---:|
| static | 5.516 m | 5.567 m | 5.818 m |
| rollpitch | 5.513 m | 5.536 m | 5.871 m |
| combined | 5.619 m | 5.991 m | 6.077 m |
| rigid_body_motion | 5.630 m | 5.996 m | 6.067 m |

static 与 rollpitch 的距离几乎一致，但 normal error 明显不同；更远的 combined/rigid-body 反而更好。因此距离差异不是 static 主因。

## 9. Board image-center proxy

Bag 没有保存原始 Board corners/corner centroid。分析只能使用 raw `T_camera_deck` 的：

```text
x/z, y/z, hypot(x/z, y/z)
```

作为 pose-implied normalized image-center proxy。

center-radius proxy median/P95：

```text
static            0.005 / 0.084
rollpitch         0.037 / 0.098
combined          0.140 / 0.224
rigid_body_motion 0.141 / 0.260
```

static 实际最接近图像中心，而不是更靠边。全矩阵 correlation 还是负值 `-0.4602`，因此“static 因 Board 靠图像边缘而恶化”不受证据支持。

该结论只适用于排除明显离轴；由于没有原始 corners，不能把 proxy 当成真实 corner centroid。

## 10. reprojection RMSE

共同样本的 reprojection RMSE：

| Scenario | median | RMS | P95 | max |
|---|---:|---:|---:|---:|
| static | 0.595 px | 0.627 px | 0.874 px | 1.108 px |
| rollpitch | 0.441 px | 0.465 px | 0.621 px | 1.010 px |
| combined | 0.425 px | 0.455 px | 0.648 px | 1.037 px |
| rigid_body_motion | 0.423 px | 0.454 px | 0.643 px | 0.990 px |

raw normal error 与 reprojection RMSE 的 frame-level correlation 为 `+0.9395`，是本次已观测量中最强关系。static 在同距离、同 4 Marker、最居中的条件下仍有更高 residual 与明显更高法向误差。

这与近正视平面 PnP 的 conditioning 解释一致：很小的 corner/reprojection residual 在法向方向上被更强地放大。

## 11. IPPE candidate flip

历史正式 evaluator 的 `raw_pose_normal_flip_count_zero` gate 在 12 轮均通过；例如 `static_s1` 的 `raw_pose_normal_flip_count=0`。没有观察到大规模 `90°/180°` normal flip。

这不证明 IPPE 候选选择在所有小角度抖动上绝对最优，但否定了“static 主要由频繁大角度双解翻转造成”的假设。

## 12. shadow filtering

static：

```text
A raw RMSE = 1.909–2.179°
C shadow   = 1.186–1.421°
```

动态：

```text
A raw RMSE = 0.885–1.073°
C shadow   = 0.453–0.549°
```

shadow 在两类场景都降低 normal error，因此它不是 static 错误的起点。本阶段禁止通过调 measurement noise 或 angular process noise 来掩盖 detector 根因。

## 13. Planar PnP observability / conditioning

本次证据支持的严格表述是：

> near-fronto-parallel 的共面 Board orientation estimation 存在明显 poor-conditioning；有限 corner / reprojection residual 会被放大为 deck-normal error，而真实 deck tilt 增大后 conditioning 明显改善。

不能写成“planar pose 无法求解”。当前系统事实上能稳定产生有限、正深度、方向正确且低 reprojection RMSE 的 pose；问题是 near-fronto-parallel 时 orientation 的法向分量对亚像素几何误差更敏感。

## 14. 被排除的主因假设

固定正式 Bag 支持排除以下主因：

- **PX4 attitude/time**：图像时刻 body-normal error 仅约 `0.09–0.12°`，A/B 也几乎一致。
- **shadow**：C 系统性低于 A，当前是在降误差。
- **Marker 数减少**：tracking/common 集合中均为 4 Marker。
- **距离差异**：static 与 rollpitch 基本相同，更远动态场景反而更好。
- **Board 靠图像边缘**：static center proxy 最小。
- **大规模 IPPE 90°/180° flip**：正式 12 轮 raw flip gate 全通过。

## 15. 旧 Bag 仍无法确认的因素

由于没有保存原始 image / Board corners，旧 Bag 无法直接区分：

1. ArUco corner detector 原始像素噪声；
2. `CORNER_REFINE_SUBPIX` 前后的 corner 改变量；
3. 同一帧 raw corners 上 IPPE 与 `solvePnPRefineLM` 的真实在线差异；
4. corner distribution 的真实图像 Jacobian / condition number。

这些只能通过固定 synthetic regression 或新的、不改变正式 seed/门限的在线复验来证伪。

## 16. 已实现的最小修复

固定历史证据和 synthetic regression 支持后，只修改共享 detector 根因：

```text
solvePnPGeneric(IPPE)
-> candidate physical validation
-> reprojection RMSE
-> temporal continuity selection
-> select best IPPE candidate
-> solvePnPRefineLM(all visible Board corners, selected rvec/tvec as initial guess)
-> revalidate refined pose
-> recompute reprojection RMSE
-> use refined pose only if valid and non-worse
-> otherwise preserve original valid IPPE pose
```

目标函数：

```text
(r*, t*) = arg min_(r,t) Σ_k ||u_k - project(K,D,r,t,P_k)||²
```

IPPE 继续负责 planar initialization 和 ambiguity handling；LM 只对已经选定的合法候选做同帧重投影最小化。它不读取 Ground Truth、scenario、motion phase 或 future deck state，也不新增 YAML 参数。实现没有修改 temporal continuity 权重、`CORNER_REFINE_SUBPIX`、shadow measurement/process noise、Board hard gate 或 landing controller。

## 17. Synthetic regression

固定：

```text
Marine ID4/5/6/7 geometry
marker size = 0.50 m
centers = ±0.78 m
camera-to-board distance = 5.5 m
Gaussian corner noise sigma = 0.45 px
random seed = 20260815
500 trials / tilt
```

结果：

| Tilt | Raw IPPE normal RMSE / P95 | IPPE+LM normal RMSE / P95 | Raw / refined reprojection RMS |
|---|---:|---:|---:|
| 0° | 3.338° / 4.862° | 0.484° / 0.831° | 1.166 / 0.573 px |
| 0.5° | 2.970° / 4.300° | 0.490° / 0.862° | 1.091 / 0.570 px |
| 2° | 2.350° / 3.741° | 0.499° / 0.820° | 0.947 / 0.576 px |
| 5° | 1.285° / 2.197° | 0.463° / 0.821° | 0.709 / 0.581 px |

四档 refined normal RMSE 均小于 raw IPPE 的 `0.5×`，且 reprojection RMS 同时下降。另用固定恶劣 corners 构造了 LM 收敛后 deck normal 方向非法的 case；production estimator 正确回退原合法 IPPE pose，没有把 refinement 失败传播成无效或翻面输出。

## 18. Marine smoke 与固定 4×3 复验

普通测试通过后重新运行 `static seed1` 与 `rollpitch seed1` safe-altitude smoke：

```text
static seed1    current normal RMSE/P95 = 0.176° / 0.361°
rollpitch seed1 current normal RMSE/P95 = 0.149° / 0.249°
```

两轮 Safety/Planar Board gate 均通过；descent/contact/penetration/`NAV_LAND`/Disarm/nonfinite 均为 0，touchdown confirmation 和 terminal stabilization 均未发生。

随后用固定 `static/rollpitch/combined/rigid_body_motion × seed 1/2/3`、同一 `7.0 m` rendezvous、安全边界和 hard gates 写入新目录：

```text
results/deck_motion_shadow_planar_marine_refinelm_local7m_20260815
```

结果：

```text
Safety gates       = 12/12
Planar Board gates = 12/12
Full shadow gates  = 0/12  # Future Twist 未调参，本阶段不要求
```

static 三 seed current-normal 修复前后：

| Episode | 20260813 IPPE-only RMSE / P95 | 20260815 IPPE+LM RMSE / P95 |
|---|---:|---:|
| static_s1 | 1.186° / 2.010° | 0.151° / 0.291° |
| static_s2 | 1.421° / 2.455° | 0.134° / 0.252° |
| static_s3 | 1.284° / 2.303° | 0.145° / 0.288° |

新正式矩阵 A/B/C 也不再呈现旧的 static `~2°` raw detector error：static A raw+GT RMSE 为 `0.206–0.248°`，B marker-pose RMSE 为 `0.216–0.261°`，C shadow RMSE 为 `0.134–0.151°`。修复后全矩阵 `corr(raw normal error, true tilt)=-0.0407`、`corr(raw normal error, reprojection RMSE)=-0.0053`，说明原先随 near-fronto-parallel conditioning 放大的主导误差已被同帧 LM refinement 消除到当前噪声水平。

这组结果在固定 seed、固定 threshold、固定安全边界下支持最初根因判断并完成闭环验证，不需要继续实现候选选择、SUBPIX、observability confidence 或 orientation filter 后续方案。

## 19. 安全边界

本诊断与候选修复不改变：

- Marine 默认不下降；
- relative descent / final descent / terminal contact disabled；
- 不发送 `NAV_LAND`；
- 不自动 Disarm；
- Ground Truth 不进入在线 detector / estimator / controller；
- 不调 Future Twist、shadow filter、SUBPIX、Board hard gate 或 landing controller。

只有 fixed seed、fixed threshold、fixed safety boundary 下重新运行的真实 Marine 结果才可更新正式能力结论。
