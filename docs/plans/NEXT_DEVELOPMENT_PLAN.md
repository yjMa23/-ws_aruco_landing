# 下一步计划：Future Twist causal diagnosis

## 当前已冻结事实

Marine Planar Board static 法向根因已经完成修复与正式验收。固定
`static/rollpitch/combined/rigid_body_motion × seed 1/2/3`、固定门限与 safe-altitude
安全边界下，2026-08-15 RefineLM 复验达到：

```text
Safety gates       = 12/12
Planar Board gates = 12/12
Full shadow gates  = 0/12
```

Board 当前位姿不再是阻塞项。不得继续为了 Future Twist 调整 Board estimator、
`CORNER_REFINE_SUBPIX`、Board hard gate 或 shadow current-pose filter。

## 下一目标

只诊断 `/landing/deck_motion_shadow/trajectory` 的 `0.5 s` Future Twist 因果误差，限定为：

```text
prediction normal
horizontal velocity
vertical velocity
angular velocity
```

当前位置、当前法向、Board 路由和 Board reprojection 只作为已冻结输入质量证据，不再作为
调参目标。

## 数据与因果边界

优先使用已经生成且安全门通过的固定 Marine 正式 Bag：

```text
results/deck_motion_shadow_planar_marine_refinelm_local7m_20260815
```

时刻 `t` 的任何回放估计只能读取 `t` 及以前实际可在线获得的：

- ArUco/Planar Board pose 与时间戳；
- PX4 UAV pose/velocity 与已建立的时间映射；
- deck-motion estimator 自身历史状态；
- 当前配置中的 prediction horizon 与采样周期。

Ground Truth 只能在 Future Twist 已经生成后离线评分。禁止读取未来 Ground Truth、
`MotionProfile` phase、scenario 名或仿真器内部未来轨迹来生成/修正预测。

## 诊断顺序

1. **先分离 current-state 与 extrapolation error**：证明 `t` 时刻 current pose/twist 的误差
   与 `t+0.5 s` prediction error 各占多少，避免把未来外推失败误判为 Board current-pose
   问题。
2. **时间基准检查**：核对 image stamp、PX4 mapped time、estimator update time、trajectory
   header time 和 `0.5 s` target time；量化 sample gap、age 与 horizon 实际偏差。
3. **四个目标通道分别评分**：normal、horizontal velocity、vertical velocity、angular
   velocity 不共享一个模糊总分；按 episode 与 scenario 输出 RMSE/P95/max 和失败帧比例。
4. **检查因果运动学模型**：对比现有 estimator 的速度/角速度估计、有限差分/拟合窗口和
   0.5 s 外推公式，定位误差来自 current twist 估计、加速度假设还是时间对齐。
5. **按场景做可证伪对照**：static、rollpitch、combined、rigid-body 必须使用同一算法和
   固定参数；不允许为单场景建立 phase-aware 分支。
6. **只在证据支持后提出最小修复**：若需要改模型，先补理论与构建说明，再写 regression，
   最后修改共享 estimator/predictor 根因位置。

## 第一轮禁止项

在因果回放给出明确根因前，不允许：

- 调 Planar Board detector 或 RefineLM；
- 放宽 Future Twist hard gate；
- 为不同 scenario/seed 设不同参数；
- 使用未来 Ground Truth 或已知 motion profile phase；
- 开启动态姿态下降、真实接触、`NAV_LAND`、自动 Disarm；
- 引入 NMPC、波浪船舶动力学或新的控制器调参。

## 输出与验收

第一轮必须形成可复现离线诊断，至少输出：

```text
per episode + aggregate
current vs 0.5 s prediction error decomposition
prediction normal RMSE/P95
horizontal velocity RMSE/P95
vertical velocity RMSE/P95
angular velocity RMSE/P95
actual prediction horizon/time-alignment statistics
scenario comparison
root-cause evidence
next minimal falsifiable experiment
```

若仅靠现有 Bag 不能确认某一因素，必须明确记录缺失观测，不得通过换 seed 或调阈值
制造结论。只有根因有足够证据支持后，才能进入对应最小模型修复和新的固定 4×3 复验。

## 安全边界

本计划仍只允许 Marine safe-altitude GNSS rendezvous、视觉跟踪与 deck-motion shadow。
relative descent、final descent、dynamic deck contact、terminal-contact stabilization、
`NAV_LAND` 和 automatic disarm 继续禁止。Ground Truth 继续只用于离线评分。
