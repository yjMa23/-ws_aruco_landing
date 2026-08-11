# 下一步计划：Planar Board 正式 5 m 验证 → Future Twist 因果诊断

## 当前状态

Marine 已完成可实船平贴部署的远距 ArUco 几何和 detector 实现：

- `--environment legacy` 继续使用 frozen ID0/4/5/6 非共面几何与历史配置。
- `--environment marine` 使用独立 `aruco_detector_marine.yaml`。
- Marine ID4/5/6/7 均为 `0.50 m`，中心为 `(±0.78, ±0.78, 0.002) m`，全部 `rpy=0`。
- `>=2` 个远距 Marker 使用 `SOLVEPNP_IPPE` planar multi-marker pose；1 个使用已知 `T_marker_deck` 转换到 deck center；0 个回退既有 ID0/1/2/3 `MarkerSelector`。
- detector 增加 `/aruco/pose_source`、`/aruco/board_marker_count`、`/aruco/board_reprojection_rmse_px`，失效帧不发布陈旧 pose。
- Marine 仍只允许 GNSS rendezvous、视觉捕获、安全高度跟踪和 deck-motion shadow；相对下降、`FINAL_DESCENT`、真实接触、terminal-contact stabilization、`NAV_LAND` 与自动 Disarm 均未开放。

此前旧非共面 Board 的约 `5 m` 相对 shadow 正式矩阵为：安全隔离 `12/12`、当前法向与未来位置主要硬门 `12/12`，Future Twist 相关水平速度/垂直速度/角速度门分别 `6/12`、`2/12`、`3/12`，全部硬门 `2/12`。这些数据仍是 frozen 历史证据，不得用新 Planar Board 结果覆盖。

## 下一目标 1：Planar Board 正式约 5 m 4×3

先固定当前代码、几何、阈值和 seed，运行：

```text
static / rollpitch / combined / rigid_body_motion × seed 1/2/3
```

共 12 runs，保持 `rendezvous_altitude_m=7.0`（约 5 m 相对甲板高度）。直接执行：

```bash
python3 scripts/run_deck_motion_shadow_experiments.py \
  --environment marine \
  --output results/deck_motion_shadow_planar_marine_5m_<date>
```

不得换 seed、删失败轮次、重挑结果或为了通过验收放宽 detector/shadow 阈值。

本轮 Planar Board 的主要验收不是 Future Twist `12/12`，而是：

1. ArUco valid coverage 足够稳定，远距主要来源为 `PLANAR_BOARD_MULTI`。
2. current deck position error 与 deck-normal error 不劣化到不可用。
3. `board_reprojection_rmse_px` finite 且无系统性爆炸。
4. planar ambiguity 不产生 frame-to-frame pose/normal flip。
5. 4/3/2 Marker partial visibility 能继续输出统一 deck pose；1 Marker 正确进入 `FAR_SINGLE`。
6. `NAV_LAND=0`、Disarm command=0、`touchdown_confirmed=false`，没有任何下降或接触路径被意外打开。

同时与 frozen 旧非共面结果比较：

```text
ArUco valid coverage
current position error
normal error
board reprojection RMSE
pose flip count
0.5 s position prediction
0.5 s normal prediction
future twist
```

比较只用于确认几何替换的代价与收益，不允许为追求更好结果改 seed 或筛选轮次。

## 下一目标 2：Future Twist 因果可观测性诊断

只有 Planar Board 的 4×3 正式 safe-altitude 验证完成后，才继续 Future Twist 诊断：

1. 使用新的固定 Planar Board 正式 Bag 做严格因果回放，分解 ArUco 相对位姿、PX4 UAV velocity、局部常加速度拟合与 SO(3) 角导数对 `0.5 s` twist 误差的贡献。Ground Truth 只允许离线评分，不能进入在线参数、相位、频率或 future trajectory。
2. 若 ArUco-only 因果估计在跨 seed 固定参数下仍无法通过，再单独评审最小额外观测契约，例如甲板 IMU 的线/角加速度或船舶 GNSS velocity 绝对锚；未经批准不实现。
3. 任意新预测模型必须先在 `static/combined seed1` 用预注册参数通过全部旧安全硬门，再从头运行新的 4×3；失败 seed 不替换、门限不放宽。

## 后续边界

本计划不授权：

```text
acados NMPC
dynamic deck descent
real contact
NAV_LAND
automatic disarm
JONSWAP / PM spectrum
RAO
wave-driven WAM-V dynamics
Buoyancy / Hydrodynamics
wind / current
```

VRX `WaveVisual + Gerstner` 已经是动态 visual-only ocean，`MotionProfile` 仍是唯一 vessel motion source。只有 Future Twist 因果可观测性被独立解决后，才可另起计划评审动态姿态控制或真实 sea-state → vessel-response 链。
