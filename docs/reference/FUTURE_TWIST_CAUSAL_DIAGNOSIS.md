# Future Twist 因果误差诊断

## 1. 问题定义与边界

本诊断只研究 Marine safe-altitude deck-motion shadow 的 `0.5 s` Future Twist：

```text
horizontal velocity
vertical velocity
angular velocity
```

固定输入为：

```text
results/deck_motion_shadow_planar_marine_refinelm_local7m_20260815
```

覆盖：

```text
static / rollpitch / combined / rigid_body_motion
× seed 1 / 2 / 3
```

全部 12 个 Bag 均可读取。Ground Truth 只在 causal prediction 已经生成后用于离线评分、有限差分加速度和误差归因；任何 CA/CV counterfactual 都只使用当时 trajectory 已经发布的 origin twist 与 acceleration，不读取未来真值、scenario phase 或 MotionProfile 内部状态。

本轮没有修改 Planar Board detector、RefineLM、SUBPIX、Board hard gate、下降控制、动态接触、NMPC、波浪船舶动力学，也没有把 Ground Truth 接入在线 estimator/controller。

## 2. 重新计算后的真实 gate

从固定 2026-08-15 的 12 个 Bag 重新运行现有 evaluator，而不是读取旧文档数字：

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

因此 `prediction normal` 已不是阻塞项，只保留为 regression metric。当前任务只剩三个 Future Twist velocity gate。

原 evaluator 重新计算得到每轮 `0.5 s` P95 范围：

```text
horizontal velocity   0.190–0.409 m/s
vertical velocity     0.073–0.143 m/s
angular velocity      1.447–3.096 deg/s
```

current twist P95 范围：

```text
horizontal velocity   0.096–0.190 m/s
vertical velocity     0.020–0.068 m/s
angular velocity      0.822–1.520 deg/s
```

## 3. 时间与坐标语义

### 3.1 时间契约

代码中的真实语义为：

```text
shadow state header stamp
= estimator last accepted sample time

trajectory header stamp
= publish now

trajectory point time_from_start
= relative prediction time from publish now
```

节点先调用：

```text
estimate()
predict(now)
```

`predict(now)` 会先把 sample-time state propagation 到 `now`，随后才从 `now` 生成未来 trajectory。因此 `0.5 s` 评分目标必须是：

```text
target_time = trajectory.header.stamp + 0.5 s
```

而不是：

```text
state.sample_time + 0.5 s
```

trajectory 的 `time_from_start=0` 点已经是 publish-time causal origin twist，故本诊断用该点作为 `v_hat(t)` / `omega_hat(t)`；state header 只用于 observation age。

### 3.2 rosbag 跨 topic 配对

state 与 trajectory 在同一 `publish_deck_motion_shadow()` 调用中依次 publish，但 rosbag 跨 topic receipt order 可以反转。分析器因此按 bag receipt time 最近邻配对同一 publish tick，不要求 state 必须先到达。

固定矩阵中每轮 state/trajectory 数量均为 `622/622`；最近邻 receipt-time 配对无缺失，`unpaired_trajectory=0`。末尾因缺少 `t+0.5 s` Ground Truth 而排除约 10–11 帧/轮，这是严格 future-truth 闭区间评分的预期行为。

### 3.3 observation age

全矩阵 7333 个可评分 causal origin：

```text
observation age
min      = 0.008 s
median   = 0.032 s
P95      = 0.052 s
max      = 0.128 s

requested horizon = 0.500 s
actual target offset P95 = 0.500 s
state-sample -> target horizon
median   = 0.532 s
P95      = 0.552 s
max      = 0.628 s
```

observation age 与最终误差的 Pearson 相关性：

```text
horizontal velocity  -0.010
vertical velocity    +0.036
angular velocity     +0.022
```

因此当前数据不支持 Case D：Future Twist 失败不是 observation age 或 target-time mismatch 主导。

## 4. 误差分解

对 publish time `t` 和 `h=0.5 s`：

```text
current_velocity_error
= v_hat(t) - v_gt(t)

acceleration_estimation_contribution
= h * (a_hat(t) - a_gt(t))

constant_acceleration_model_residual
= v_gt(t) + h * a_gt(t) - v_gt(t+h)
```

angular channel 同理：

```text
current_angular_velocity_error
= omega_hat(t) - omega_gt(t)

angular_acceleration_estimation_contribution
= h * (alpha_hat(t) - alpha_gt(t))

angular_constant_acceleration_model_residual
= omega_gt(t) + h * alpha_gt(t) - omega_gt(t+h)
```

最终检查：

```text
future error
≈ current error
 + acceleration estimation contribution
 + CA model residual
```

GT acceleration / angular acceleration 只通过 `t±0.02 s` GT twist 中心有限差分用于离线评分。

### 4.1 全局 linear P95

| channel | current error | `h * acceleration error` | CA model residual | final future error | accel contribution vs final correlation |
| --- | ---: | ---: | ---: | ---: | ---: |
| horizontal | 0.1546 m/s | 0.1705 m/s | 0.0767 m/s | 0.3283 m/s | 0.891 |
| vertical | 0.0578 m/s | 0.0702 m/s | 0.0187 m/s | 0.1134 m/s | 0.834 |

对应 current-error 与 final-error 相关性分别为 `0.724 / 0.634`；model-residual 相关性只有 `0.246 / 0.141`。

linear decomposition closure residual：

```text
horizontal P95 ≈ 5.0e-16 m/s
vertical   P95 ≈ 1.2e-16 m/s
```

说明误差分解与生产 trajectory 的 CA 外推数学上一致。

### 4.2 全局 angular P95

单位均为 `deg/s`：

| axis | current error | `h * angular accel error` | CA model residual | final future error |
| --- | ---: | ---: | ---: | ---: |
| roll | 0.883 | 1.119 | 0.426 | 2.284 |
| pitch | 0.768 | 0.931 | 0.300 | 1.896 |
| yaw | 0.671 | 0.456 | 0.139 | 0.963 |
| roll/pitch vector | 1.043 | 1.302 | 0.485 | 2.661 |
| 3D vector | 1.211 | 1.340 | 0.489 | 2.758 |

3D angular-vector contribution 与 final-error 的相关性：

```text
current angular velocity error              0.784
angular acceleration estimation contribution 0.925
CA model residual                           0.740
```

closure residual 约 `1e-15 deg/s` 数量级。

roll/pitch 是主要 angular gate 压力来源；yaw current error 仍可见，但其 future P95 显著低于 roll/pitch。

## 5. static 与动态场景

下表给出 scenario aggregate P95，格式为：

```text
current / acceleration contribution / CA model residual / final
```

| scenario | horizontal m/s | vertical m/s | angular vector deg/s |
| --- | --- | --- | --- |
| static | 0.113 / 0.100 / 0.000 / 0.204 | 0.034 / 0.071 / 0.000 / 0.099 | 0.892 / 0.671 / 0.000 / 1.530 |
| rollpitch | 0.110 / 0.101 / 0.016 / 0.205 | 0.024 / 0.056 / 0.003 / 0.077 | 0.954 / 1.266 / 0.502 / 2.569 |
| combined | 0.156 / 0.198 / 0.080 / 0.343 | 0.066 / 0.076 / 0.019 / 0.124 | 1.364 / 1.432 / 0.494 / 2.956 |
| rigid_body_motion | 0.175 / 0.187 / 0.079 / 0.369 | 0.067 / 0.076 / 0.019 / 0.131 | 1.497 / 1.442 / 0.508 / 2.990 |

关键事实：

1. `static` 真值 acceleration 和 CA model residual 为零，但 estimator acceleration 仍额外制造约 `0.100 m/s` horizontal、`0.071 m/s` vertical、`0.671 deg/s` angular P95 contribution。
2. `rollpitch` 的 linear CA model residual 很小，尤其 vertical 只有约 `0.003 m/s`，但 acceleration contribution 为约 `0.056 m/s`。
3. `combined/rigid_body_motion` 中真实 acceleration 对未来速度确实有价值，所以不能简单永久删除 acceleration；但 acceleration-estimation contribution 仍明显大于 CA model residual，并高度相关于最终尾部误差。
4. angular dynamic 的 CA model residual 约 `0.5 deg/s`，说明 0.5 s 内并非完美 constant-angular-acceleration；但它仍低于 current + acceleration-estimation 两项，不是单独主因。

## 6. causal CA vs CV counterfactual

候选预测均在 Ground Truth 评分前生成：

```text
CA: v(t+h) = v_hat(t) + h * a_hat(t)
CV: v(t+h) = v_hat(t)
```

angular 同理。

### 6.1 scenario P95

| scenario | H CA / CV m/s | V CA / CV m/s | angular CA / CV deg/s |
| --- | ---: | ---: | ---: |
| static | 0.204 / 0.113 | 0.099 / 0.034 | 1.530 / 0.892 |
| rollpitch | 0.205 / 0.126 | 0.077 / 0.026 | 2.569 / 2.614 |
| combined | 0.343 / 0.455 | 0.124 / 0.135 | 2.956 / 2.755 |
| rigid_body_motion | 0.369 / 0.484 | 0.131 / 0.136 | 2.990 / 2.783 |

全局 P95：

```text
horizontal: CA 0.328 m/s, CV 0.410 m/s
vertical:   CA 0.113 m/s, CV 0.122 m/s
angular:    CA 2.758 deg/s, CV 2.675 deg/s
```

全局 frame ratio 也不是单边结论：horizontal CA 在约 `57.1%` 帧更好；vertical CV 在约 `65.2%` 帧更好；angular CV 在约 `57.6%` 帧更好。

因此“把 production 从 CA 全局改成 CV”被固定 Bag 直接证伪：它会明显恶化 `combined/rigid_body_motion` 的 horizontal prediction。相反，“所有场景无条件满幅使用 acceleration”也被 static/rollpitch 的结果证伪。

## 7. estimator 机制检查

### 7.1 translation

`DeckMotionEstimator::update_translation_kinematic_fit()` 使用最近：

```text
kinematic_fit_window_s = 0.30 s
```

的位置历史进行二次最小二乘：

```text
p(tau) = p0 + v * tau + 0.5 * a * tau^2
```

当 fit valid 后，`make_estimate()` 会用：

```text
fitted_deck_velocity_ned_mps_
fitted_deck_acceleration_ned_mps2_
```

覆盖 translation Kalman state 的 velocity/acceleration。随后 `predict(now)` 又把该 acceleration 完整用于 sample→now propagation 和未来 `0.5 s` extrapolation。

固定 Bag 证据与“短窗二次导数对位姿噪声敏感，并在 0.5 s 外推中放大”一致：即使 static 真值 acceleration 为零，仍产生显著非零 acceleration contribution；dynamic 中 acceleration 本身又确实包含有用信息。因此问题不是简单 horizon/CA 模型错误，而是 **acceleration estimate 没有按可信度决定是否应满幅进入 0.5 s Future Twist**。

本轮没有修改 `kinematic_fit_window_s`、measurement/process noise 或 jerk 参数。

### 7.2 rotation

rotation 不使用 translation quadratic fit。它通过 SO(3) orientation innovation 更新 error-state 中的：

```text
orientation
angular velocity
angular acceleration
```

并用 constant-angular-acceleration propagation 生成未来 twist。

rotation 同样表现为 acceleration contribution 高相关、static 中 CV 明显优于 CA，但 dynamic roll/pitch 中 acceleration 有时仍提供有效预测。因此 translation 与 rotation **共享“无可信度的 acceleration extrapolation 放大误差”这一 failure mechanism，但不是同一个 estimator 根因**：translation acceleration 来自 0.30 s quadratic fit，rotation acceleration 来自 SO(3) error-state。

## 8. 排除的假设

固定 Bag 当前可以排除：

- Planar Board / prediction normal 仍是主要阻塞项：false，normal gate 已 `12/12`。
- target time 使用错误：false，严格 `trajectory.header + 0.5 s` 后 closure 正确。
- observation age 主导：false，age 与三类最终误差相关性接近 0。
- pure CA model mismatch 单独主导：false，linear model residual 明显小于 acceleration-estimation contribution；angular 也不是单独由 residual 解释。
- 全局 CV 是正确修复：false，会显著恶化 combined/rigid horizontal。
- translation 与 rotation 应共享同一个 estimator 参数修复：没有证据支持。

## 9. 最终根因判断

第一轮归类为 **Case A 为主，伴随不可忽略的 current-twist baseline error**：

> 当前 Future Twist 的主要可证伪失败机制，是 acceleration / angular-acceleration estimate 在缺少 causal confidence 判别时被满幅用于 0.5 s extrapolation；其误差 contribution 对最终误差高度相关，并在 static 场景把本应为零的 acceleration 放大成明显 future-twist tail。constant-acceleration model residual 和时间对齐都不是主要阻塞项。

但固定 Bag 同时证明动态 combined/rigid translation 不能直接退化成 CV。因此当前证据 **足以定位 failure mechanism，却不足以证明某一个 production acceleration-confidence 算法或阈值**。

## 10. 为什么本轮不修改 production

本轮没有修改：

```text
deck_motion_estimator.cpp/.hpp
px4_aruco_landing_node.cpp
px4_aruco_landing.yaml
```

原因不是根因未知，而是修复形式尚未被现有 Bag 充分证明：

- 全局 CV 已被动态场景证伪；
- 固定 scalar blend / acceleration threshold 会变成无证据调参；
- translation 与 rotation acceleration 来源不同，不应先假设共享同一阈值；
- 当前 Bag 发布了 origin acceleration，但没有把 estimator acceleration covariance / fit confidence 作为离线评分信号暴露出来，无法验证“按内部置信度抑制 acceleration”是否真的能同时保留 dynamic benefit 并消除 static tail。

因此遵守“先证明 error 来自哪里，再修改对应根因”的边界，本轮停在诊断结论，不运行新的 SITL smoke 或 4×3 复验。

## 11. 下一条最小可证伪实验

下一步只做 **offline estimator-confidence replay**，不调 Board、不跑新 seed：

1. 用固定 12 Bag 的 causal 输入流重放 `DeckMotionEstimator`，验证 replay 的 origin twist/acceleration 与已发布 shadow trajectory 一致。
2. 离线提取每个 origin 的 translation fitted kinematic covariance 与 rotation acceleration covariance；这些量只能来自 estimator 当时内部状态，不能使用 GT 生成。
3. 先只回答一个问题：`h * acceleration error` 的大尾部是否与 estimator 自身 acceleration uncertainty 单调相关。
4. 若相关性成立，再预注册一个 covariance-derived confidence law，离线同时评分 static/rollpitch/combined/rigid，不按 scenario/seed 调参数；只有它能保留 dynamic CA benefit 且压低 static/rollpitch tail，才进入 regression + production。
5. 若 uncertainty 与真实 acceleration error 不相关，则拒绝 confidence-gating 假设，转而分别定位 translation quadratic-fit derivative 与 rotation angular-acceleration state。

这条实验可以直接证伪“现有 covariance 足以决定 acceleration 是否可信”，且不需要新 SITL 或 Ground Truth 进入在线算法。

## 12. 验证证据

正式分析入口已直接运行：

```bash
python3 scripts/analyze_future_twist_causality.py \
  --matrix-dir results/deck_motion_shadow_planar_marine_refinelm_local7m_20260815
```

CLI 成功读取 12 个 episode、4 个 scenario，输出 7333 个可评分 frame 的 strict JSON，`allow_nan=False` 验证通过。

`test_deck_motion_shadow.py` 在原有 19 个 unittest case 基础上新增 7 个 Future Twist regression，当前直接运行 `26/26` 通过。该 Python 文件在 CMake 中作为单个 `deck_motion_shadow_tests` CTest 注册，因此 `colcon test-result` 的全仓聚合计数仍保持冻结口径：

```text
391 tests, 0 errors, 0 failures, 0 skipped
```

新增回归覆盖 publish-time `0.5 s` horizon、state/trajectory 跨 topic receipt-order 配对、linear/angular decomposition closure、CA/CV counterfactual、scenario aggregate 与 empty/nonfinite strict JSON。

## 13. 安全边界

本诊断不改变系统授权：

```text
relative descent disabled
final descent disabled
dynamic deck contact disabled
NAV_LAND disabled
automatic disarm disabled
```

Marine 继续只允许 safe-altitude GNSS rendezvous、视觉跟踪与 deck-motion shadow。Future position/normal/yaw 的 `12/12` 结果必须作为后续任何 estimator 修改的 regression boundary。
