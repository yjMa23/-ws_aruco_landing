# 下一步计划：Future Twist estimator-confidence replay

## 当前已冻结事实

Marine Planar Board static 法向根因已经修复并完成正式验收。固定：

```text
static / rollpitch / combined / rigid_body_motion
× seed 1 / 2 / 3
```

以及固定 hard gates 下，2026-08-15 RefineLM 正式矩阵重新计算得到：

```text
Safety gates                         12/12
Planar Board gates                   12/12
Full shadow gates                     0/12

future horizontal position           12/12
future vertical position             12/12
future normal                        12/12
future yaw                           12/12

future horizontal velocity            0/12
future vertical velocity              5/12
future angular velocity               3/12
```

`prediction normal` 已不再是阻塞项，只保留为 regression metric。不得继续为了 Future Twist 调整 Planar Board estimator、RefineLM、SUBPIX、Board hard gate 或 current-pose filter。

## 已完成的第一轮 causal diagnosis

固定目录：

```text
results/deck_motion_shadow_planar_marine_refinelm_local7m_20260815
```

已用 `scripts/analyze_future_twist_causality.py` 完成 7333 个可评分 causal origin 的 current / acceleration / CA-model-residual 分解、严格 publish-time `0.5 s` target 对齐和 CA/CV counterfactual。完整证据见：

```text
docs/reference/FUTURE_TWIST_CAUSAL_DIAGNOSIS.md
```

已确认：

- observation age median/P95/max 为约 `0.032/0.052/0.128 s`，与 H/V/angular future error 的相关性都接近 0；time alignment 不是主因。
- decomposition closure 在数值精度内闭合。
- horizontal / vertical 的 `h * acceleration estimation error` P95 分别约 `0.170/0.070 m/s`，均明显高于对应 CA model residual `0.077/0.019 m/s`，并与最终误差高度相关。
- angular-vector 的 current / acceleration contribution / CA residual / final P95 约为 `1.211/1.340/0.489/2.758 deg/s`；acceleration contribution 与 final error 相关系数约 `0.925`。
- static 中 CV 明显优于 CA；combined/rigid-body 的 horizontal translation 又明显需要 acceleration。全局永久切换 CV 已被固定 Bag 证伪。
- translation 与 rotation 共享“缺少 acceleration confidence 时，错误 acceleration 被满幅外推 0.5 s”的 failure mechanism，但 acceleration 来源不同：translation 来自 `0.30 s` quadratic fit，rotation 来自 SO(3) error-state。

因此当前根因归类为 **Case A 为主，伴随不可忽略的 current-twist baseline error**。现有证据足以定位 failure mechanism，但还不足以证明某个 production confidence law、blend 或 threshold。

## 当前唯一下一任务

只做 **offline estimator-confidence replay**。第一目标不是让 hard gate 变成 `12/12`，而是证伪或支持：

> estimator 自身的 acceleration uncertainty 是否能够识别会在 `0.5 s` Future Twist 中造成大尾部误差的 acceleration estimate。

### 1. 固定数据

仍只使用现有 2026-08-15 12 Bag。禁止：

- 换 seed 或重新跑更好数据；
- 修改旧 `evaluation.json`；
- 读取 scenario/motion phase 生成候选预测；
- Ground Truth 进入 estimator 或 candidate generation。

### 2. deterministic causal replay

用 Bag 中当时在线可获得的输入重放 `DeckMotionEstimator`，首先证明 replay origin：

```text
velocity
acceleration
angular velocity
angular acceleration
sample time
```

与已发布 shadow trajectory 的 `time_from_start=0` 一致到预先定义的数值容差。

若无法复现，不允许继续 confidence 实验，先定位 replay 数据流缺口。

### 3. 只读取 estimator 自身 uncertainty

replay 成功后离线记录：

```text
translation fitted velocity/acceleration covariance
rotation angular velocity/angular acceleration covariance
```

这些量必须来自 estimator 当时内部 causal state。Ground Truth 仍只能在记录完成后评分。

### 4. 单一可证伪问题

分别对 translation 与 rotation 检查：

```text
estimator acceleration uncertainty
vs
h * acceleration estimation error
vs
final Future Twist error
```

输出 episode / scenario / global 的 correlation、分位 bins 和 tail capture ratio。

不得直接调：

```text
kinematic_fit_window_s
linear/angular jerk std
measurement noise
process noise
```

### 5. 决策规则

如果 estimator uncertainty 对 acceleration-error tail 有稳定单调关系，下一阶段才允许预注册一个 covariance-derived causal confidence law，并在固定 12 Bag 上离线 A/B。

如果没有稳定关系，则拒绝“现有 covariance 足以做 confidence gating”假设，随后分开诊断：

```text
translation: quadratic-fit derivative quality
rotation: SO(3) angular acceleration state quality
```

不得为了获得改善引入 scenario-specific threshold。

## production 与安全边界

在 confidence replay 给出明确证据前，不修改：

```text
deck_motion_estimator.cpp/.hpp
px4_aruco_landing_node.cpp
px4_aruco_landing.yaml
Planar Board detector
shadow hard gates
controller
```

也不运行新的 SITL smoke 或正式 4×3，因为本轮没有 production algorithm change 可验证。

relative descent、final descent、dynamic deck contact、terminal-contact stabilization、`NAV_LAND` 和 automatic disarm 继续禁止。Future position/normal/yaw 的 `12/12` 必须作为未来修改的 regression boundary。
