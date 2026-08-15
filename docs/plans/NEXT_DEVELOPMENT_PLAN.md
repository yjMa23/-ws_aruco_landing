# 下一步计划：Future Twist replay timing provenance closure

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

## estimator-confidence replay 已执行但被 equivalence gate 阻塞

完整证据见：

```text
docs/reference/FUTURE_TWIST_ESTIMATOR_CONFIDENCE.md
```

固定 12 Bag 已经成功用 production C++ `DeckMotionEstimator`、`VehiclePoseHistory` 和 coordinate transform 重放到全部 `7464/7464` trajectory origin，sample-time 也达到全矩阵精确一致；但 origin 数值仍不能达到预注册 equivalence tolerance：

```text
linear velocity worst episode P95       0.019300 m/s
linear acceleration worst episode P95   0.071471 m/s²
angular velocity worst episode P95      0.002175 rad/s
angular acceleration worst episode P95  0.003104 rad/s²
```

已确认两项 replay 关键事实：

1. 20 Hz shadow state header 的唯一变化不能完整恢复 30 Hz estimator update history；两个 control tick 之间会有中间 ArUco sample 被 production estimator 消费。
2. 补回所有 visual-state causal ArUco 后，剩余误差由 fixed Bag 中未记录的 `vehicle_odometry_callback()` ROS-time receipt history 主导。该 receipt time 参与 PX4→ROS filtered clock offset，并进一步决定 `VehiclePoseHistory` sample time/interpolation。

统一 `+1–3 ms` receipt-time sensitivity scan 能同步显著降低 linear/angular replay error，但即使最有利 shift 仍远高于预注册 `1e-6` P95 门；因此这不是可以通过放宽 floating-point tolerance 合法忽略的 serialization 误差。

按预注册停止规则，本轮没有提取/解释 covariance，没有做 Ground Truth correlation、quintile 或 tail capture，translation/rotation confidence hypothesis 均为 `NOT TESTED`。

## 当前唯一下一任务

先做 **replay timing provenance closure**，而不是 covariance-derived confidence A/B。

### 1. 让 estimator 真正消费的 timing/input 可观测

只允许 shadow-only diagnostic，不改变 production estimator 或 controller。优先记录：

```text
image sample time
relative_deck_pose_ned
image-time uav_velocity_ned
marker_id
vehicle_odometry callback ROS receipt time
VehiclePoseHistory mapped odometry sample time
```

可以使用现有标准消息组合；不要为了本任务创建复杂新框架。若只需验证 clock provenance，至少必须记录 `/clock` 或等价的 callback receipt/mapped sample-time 诊断。

### 2. 先做最小 replay-observability smoke

新增诊断本身不允许接入控制路径。完成 build/test 后，仅用最小：

```text
static seed1
combined 或 rigid_body_motion seed1
```

验证新记录能否让 offline production replay 达到冻结 equivalence gate。这个阶段的目的只是证明 replay input 完整，不评 Future Twist hard gate，不调 estimator 参数。

### 3. equivalence 通过后再决定正式数据

若最小 smoke replay 可以达到：

```text
coverage >= 99%
sample time max <= 1e-6 s
state vector P95 <= 1e-6
state vector max <= 1e-5
```

再冻结一套 replay-observable 正式 Marine 4×3 数据；不得把 2026-08-15 缺失 timing provenance 的旧 Bag 当成 covariance ground truth replay。

只有新正式数据的 replay equivalence 先通过，才恢复上一轮尚未执行的：

```text
covariance extraction
→ uncertainty vs acceleration-error correlation
→ equal-count quintiles
→ tail capture
→ Confidence-Supported / Confidence-Rejected decision
```

不得先实现 confidence law，也不得用 scenario-specific threshold。

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
