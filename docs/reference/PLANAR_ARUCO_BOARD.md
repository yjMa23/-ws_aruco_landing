# Marine 共面 ArUco Board 位姿模型

## 1. 目的与能力边界

本模型只服务于 `--environment marine` 的远距视觉捕获、约 5 m 安全高度跟踪和 shadow 验证。Legacy 环境仍使用冻结的历史 Marker 几何与检测配置，不由本模型改变。

本阶段输出统一的 landing deck center 位姿，不授权相对下降、`FINAL_DESCENT`、真实接触、terminal-contact stabilization、`NAV_LAND` 或自动 Disarm。Ground Truth、`MotionProfile` 相位和未来轨迹不得进入本估计器。

## 2. 坐标系与刚体关系

使用三个坐标系：

- `camera_optical`：相机光学坐标系，x 向右、y 向下、z 向前。
- `deck_landing_up`：landing deck center 坐标系，原点位于 2.4 m × 2.4 m 甲板上表面中心，+z 为甲板法向上方。
- `marker_i`：第 i 个 ArUco Marker 的几何中心坐标系。x/y 位于打印平面内，+z 与 `deck_landing_up` 的 +z 同向。

定义：

```text
T_camera_deck = [R_camera_deck, t_camera_deck]
T_deck_marker_i = [R_deck_marker_i, t_deck_marker_i]
```

满足：

```text
p_camera = R_camera_deck p_deck + t_camera_deck
p_deck   = R_deck_marker_i p_marker_i + t_deck_marker_i
```

Marine 第一版远距 Board 所有 Marker 的 `R_deck_marker_i = I`，因此全部打印面与甲板共面且法向一致。

## 3. Marine Board 精确几何

landing deck 边界为 `x,y ∈ [-1.20, +1.20] m`。远距 Board 使用四个 0.50 m Marker，统一视觉防 z-fighting 偏移 `z = 0.002 m`：

| ID | size (m) | center in `deck_landing_up` (m) | rpy (rad) |
|---|---:|---|---|
| 4 | 0.50 | `( +0.78, +0.78, 0.002 )` | `(0, 0, 0)` |
| 5 | 0.50 | `( +0.78, -0.78, 0.002 )` | `(0, 0, 0)` |
| 6 | 0.50 | `( -0.78, +0.78, 0.002 )` | `(0, 0, 0)` |
| 7 | 0.50 | `( -0.78, -0.78, 0.002 )` | `(0, 0, 0)` |

四个 Marker 的外边界最大为 `|x|=|y|=1.03 m`，仍位于 2.4 m 甲板内，并为中央 ID0/1/2/3 与无人机落区保留空间。

ID0/1/2/3 的 Marine 多尺度近距布局与当前实现保持不变：

```text
ID0 = 0.50 m
ID1 = 0.20 m
ID2 = 0.04 m
ID3 = 0.02 m
```

## 4. Marker 四角点定义

OpenCV ArUco 返回角点顺序为左上、右上、右下、左下。对边长 `L_i` 的 Marker，在 `marker_i` 中定义：

```text
p0 = (-L_i/2, +L_i/2, 0)
p1 = (+L_i/2, +L_i/2, 0)
p2 = (+L_i/2, -L_i/2, 0)
p3 = (-L_i/2, -L_i/2, 0)
```

转换到 `deck_landing_up`：

```text
p_deck_ij = R_deck_marker_i p_marker_ij + t_deck_marker_i
```

Marine Board 中四个 Marker 的全部 object points 均满足 `z=0.002 m`，即允许且期望为严格共面集合。该 `0.002 m` 是实际打印/仿真平面相对 deck origin 的标定，不会在配置中伪装成 `z=0`。

## 5. 共面 PnP 问题

给定本帧可见 Board Marker 的图像角点 `u_k`、相机内参 `K`、畸变 `D` 和已知 deck-frame object points `P_k`，求：

```text
u_k ≈ project(K, D, R_camera_deck P_k + t_camera_deck)
```

目标是得到统一的 `T_camera_deck`，而不是单独输出某个远距 Marker 的中心。

Marine 不再要求 object points 非共面。2、3、4 个 Board Marker 可见时均将全部可见角点一次性送入 planar PnP。

## 6. 为什么使用 OpenCV IPPE

共面目标存在典型的平面位姿双解。第一版使用：

```cpp
cv::solvePnPGeneric(..., cv::SOLVEPNP_IPPE)
```

IPPE 专门针对平面 object points，能够显式返回候选解，便于在 detector 内按物理约束和时间连续性消歧。继续使用“noncoplanar check + SOLVEPNP_ITERATIVE”会与 Marine 可打印平贴布局冲突，因此只保留给 frozen legacy 历史实现。

OpenCV 4.x 的 IPPE 实现要求送入求解器的平面在临时 object frame 中满足 `z=0`。因此实现仍读取真实 `z=0.002 m` 标定，但在调用 IPPE 前只对临时 object points 减去公共平面高度 `z_plane`；若 IPPE 返回 `R_camera_plane,t_camera_plane`，再恢复：

```text
R_camera_deck = R_camera_plane
t_camera_deck = t_camera_plane - R_camera_deck [0, 0, z_plane]^T
```

这个内部数值归一化不改变 SDF/打印标定，也不会把 deck origin 错移到 Marker 表面。

### 6.1 fronto-parallel 可观性与 IPPE 后最小非线性精化

固定正式 Bag 的离线诊断表明，共面 Board 在真实甲板倾角接近 `0°` 时，法向误差会随很小的像素残差显著放大。这个现象不是平面位姿“完全不可解”，而是近 fronto-parallel 条件下法向方向的数值条件变差：角点的亚像素扰动可以由深度、微小倾角和投影尺度的组合部分吸收，因此 closed-form planar pose 对角点噪声更敏感。

IPPE 仍负责显式生成平面双解和物理解消歧；当前实现会在候选通过正深度、法向方向、重投影门和时间连续性选择后，使用该候选作为初值，将**同一帧全部 Board corners**送入 `solvePnPRefineLM` 做最小的非线性重投影优化：

```text
(r*, t*) = arg min_(r,t) Σ_k ||u_k - project(K,D,r,t,P_k)||²
```

精化不得引入任何新观测，也不得读取 Ground Truth、场景名、运动相位或未来状态。精化后的结果必须再次满足 finite、全部点正深度、`deck normal faces camera` 和既有 reprojection hard gate；若精化失败、物理约束失效，或精化 RMSE 比已选 IPPE 候选更差，则回退到原 IPPE 候选。

采用 LM 而不是直接改写 IPPE 双解逻辑的原因是：修复前正式数据中没有 `>=90°` normal flip，A/B 两层法向误差几乎一致，而误差与 reprojection RMSE 强相关。固定 Marine geometry、`5.5 m`、`σ=0.45 px`、500 trials 的 synthetic regression 中，0°/0.5°/2°/5° 的 raw IPPE normal RMSE 分别为 `3.338°/2.970°/2.350°/1.285°`，RefineLM 后为 `0.484°/0.490°/0.499°/0.463°`，且 reprojection RMS 同时下降。2026-08-15 固定 Marine 4×3 正式复验进一步达到 Safety `12/12`、Planar Board `12/12`。因此当前只引入 OpenCV 已有的 `solvePnPRefineLM`，不增加 scenario-specific 分支、GT tilt gate、额外滤波参数或新的在线状态。

## 7. IPPE 双解与消歧

对 `solvePnPGeneric` 返回的每个候选 pose，依次执行：

1. **有限性**：`rvec/tvec`、投影和 RMSE 必须为 finite。
2. **正深度**：`tvec.z > 0`，并且所有参与 object point 变换到 `camera_optical` 后 z 均为正。
3. **甲板法向**：`deck_landing_up` 的 +z 法向变换到 `camera_optical` 后必须指向相机一侧。对于当前下视相机契约，合法解满足 `n_camera.z < 0`；翻面解被拒绝。
4. **重投影 RMSE**：计算全部参与角点的像素 RMSE，超过配置上限的候选无效。
5. **时间连续性**：若上一帧有有效 planar board pose，在通过上述硬约束的候选中使用“RMSE + 平移变化 + 旋转变化”的代价选择更连续的解；若没有上一帧，则以 RMSE 为主选择。
6. **LM 精化**：只对第 1–5 步选出的最佳 IPPE 候选执行 `solvePnPRefineLM`。refined pose 必须重新通过 finite、全部 object point 正深度、deck normal 方向、有限 reprojection RMSE 和既有最大 RMSE gate；并且 refined RMSE 不得比原候选差。任何 OpenCV refinement 异常或上述检查失败都回退原合法 IPPE pose。

时间连续性只使用 detector 自己上一帧已发布的视觉位姿，不使用 Gazebo Ground Truth、`MotionProfile` phase 或未来轨迹。上一帧失效时不会发布陈旧 pose；它只作为下一次合法候选的消歧先验。RefineLM 同样只消费当前图像已有 corners/K/D 和已选 IPPE 初值，不增加任何跨帧或仿真先验。

## 8. partial visibility 行为

| 可见 Board Marker 数 | 行为 |
|---:|---|
| 4 | 16 个角点执行 `PLANAR_BOARD_MULTI` |
| 3 | 12 个角点执行 `PLANAR_BOARD_MULTI` |
| 2 | 8 个角点执行 `PLANAR_BOARD_MULTI` |
| 1 | 不伪装为 multi-marker；对该 Marker 做单 Marker PnP，再用已知 `T_marker_deck` 转换为统一 deck center，来源为 `FAR_SINGLE` |
| 0 | 交给现有 ID0/1/2/3 `MarkerSelector`；若近距 Marker 也不可用则 `NONE` |

未知 ID 直接忽略，不参与 Board object/image point 拼接。

## 9. fallback 路由

每帧流程固定为：

```text
detectMarkers
  ├─ collect ID4/5/6/7 planar-board detections
  └─ feed ID0/1/2/3 candidates to existing MarkerSelector

>=2 valid board markers  -> PLANAR_BOARD_MULTI
1 valid board marker     -> FAR_SINGLE -> T_camera_marker * T_marker_deck
0 valid board markers    -> existing MarkerSelector ID0/1/2/3 -> NEAR_SINGLE
no valid pose            -> NONE, visible=false
```

Board estimator 不依赖 `active_marker_id`，也不要求某个 primary Marker 被 `MarkerSelector` 选中。

## 10. 失效条件

以下任一条件使本帧对应路径无效：

- CameraInfo 内参形状错误、焦距非正、齐次项无效、K/D 含 NaN/Inf。
- calibration 出现重复 ID、负/零 Marker size、NaN/Inf、非法姿态，或 Board 不共面。
- 检测 ID 与 corners 数量不一致。
- 已标定 Marker 角点不是四个，或图像角点含 NaN/Inf。
- multi-marker 少于两个有效 Board Marker。
- `solvePnPGeneric` 失败或没有返回候选。
- 候选深度非正、任一 object point 落在相机后方。
- deck normal 翻转或方向不符合当前下视相机物理约束。
- reprojection RMSE 非有限或超过上限。

无有效 pose 时不得发布新的 `/aruco/pose`；`/aruco/visible=false`，同时每帧更新 pose source、Board marker count 和 Board RMSE 诊断。

## 11. 输出与诊断

有效 pose 统一满足：

```text
p_camera = R_camera_deck p_deck + t_camera_deck
```

并发布：

```text
/aruco/pose
/aruco/visible
/aruco/pose_source                 PLANAR_BOARD_MULTI | FAR_SINGLE | NEAR_SINGLE | NONE
/aruco/board_marker_count          当前检测到且角点有效的 ID4/5/6/7 数量
/aruco/board_reprojection_rmse_px  multi 解的 RMSE；非 multi 帧为 NaN
```

## 12. 安全边界

Planar Board 只改变 Marine 视觉目标几何与位姿来源，不改变控制器安全门。Marine 继续仅允许 safe-altitude detection/tracking/shadow：

```text
relative descent = disabled
FINAL_DESCENT = disabled
real contact = disabled
terminal-contact stabilization = disabled
NAV_LAND = 0
automatic disarm = 0
```

任何正式下降或接触能力必须在独立阶段重新设计、评审和验收。
