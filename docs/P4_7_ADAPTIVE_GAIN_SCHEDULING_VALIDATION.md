# P4.7 加速度感知相对速度增益调度验收记录

## 1. 阶段范围

P4.7 针对 P4.6 暴露的固定增益冲突，实现基于视觉甲板估计加速度的连续相对速度增益调度：

- 匀速阶段维持低阻尼，避免稳态滞后；
- 加速和换向阶段提升阻尼，抑制相对速度过冲；
- 不使用 Ground Truth 作为控制输入；
- 不修改 PX4 内部位置控制器、状态估计模型、状态机和安全高度；
- 不实现着陆窗口、下降或触地。

## 2. 实现内容

新增纯 C++ 模块：

```text
include/aruco_precision_landing_cpp/adaptive_relative_velocity_gain.hpp
src/adaptive_relative_velocity_gain.cpp
test/adaptive_relative_velocity_gain_test.cpp
```

调度逻辑：

```text
a_raw = (v_deck[k] - v_deck[k-1]) / dt

a_limited = clamp_norm(a_raw, max_acceleration)

a_filtered = a_filtered
           + filter_gain * (a_limited - a_filtered)

alpha = clamp(
  (|a_filtered| - low_threshold) /
  (high_threshold - low_threshold),
  0,
  1)

gain = min_gain
     + smoothstep(alpha) * (max_gain - min_gain)
```

其中：

```text
smoothstep(alpha) = alpha² × (3 - 2alpha)
```

调度器只在新的 `TargetStateEstimate.sample_time_s` 到达时更新。控制循环重复读取同一估计时保持上一周期增益，避免将重复状态误判为零加速度。

## 3. 最终默认参数

```yaml
tracking.mode: PREDICTED_POSITION_VELOCITY_FF
motion_predictor.additional_prediction_horizon_s: 0.10
tracking.velocity_feedforward_gain: 1.0
tracking.relative_velocity_gain: 0.25

tracking.adaptive_relative_velocity_gain.enabled: true
tracking.adaptive_relative_velocity_gain.min_gain: 0.25
tracking.adaptive_relative_velocity_gain.max_gain: 1.20
tracking.adaptive_relative_velocity_gain.acceleration_low_threshold_mps2: 0.05
tracking.adaptive_relative_velocity_gain.acceleration_high_threshold_mps2: 0.35
tracking.adaptive_relative_velocity_gain.max_acceleration_mps2: 1.50
tracking.adaptive_relative_velocity_gain.acceleration_filter_gain: 0.20
```

固定增益消融可通过：

```bash
./scripts/start_sitl.sh --fixed-relative-gain ...
```

## 4. 新增调试接口

```text
/landing/effective_relative_velocity_gain
std_msgs/msg/Float64

/landing/estimated_deck_acceleration
geometry_msgs/msg/TwistStamped
```

语义：

- 有效增益为本周期实际参与相对速度反馈的值；
- 加速度话题使用 `local_ned`，`linear.x/y` 分别为 North/East 过滤后甲板加速度；
- 两个话题只用于调试和离线评测，不进入控制输入。

`evaluate_p4_bag.py` 已支持自动统计：

- 有效增益最小值、均值和最大值；
- 过滤后甲板加速度 RMSE 和峰值。

## 5. 单元测试

新增和扩展测试覆盖：

1. 第一帧返回最小增益；
2. 匀速保持最小增益；
3. 低于低阈值保持最小值；
4. 高于高阈值达到最大值；
5. 中间区间 smoothstep 连续单调；
6. 加速度模长限幅；
7. 一阶低通；
8. 非法输入不改变内部状态；
9. reset 清除历史；
10. 非法参数拒绝；
11. 调度关闭时固定增益行为保持不变；
12. 新加速度估计提升有效增益；
13. 重复估计保持上一调度结果；
14. 短时丢标保留增益并继续现有前馈衰减；
15. 状态 reset 后重新从最小增益开始。

全工作区结果：

```text
118 tests
0 errors
0 failures
0 skipped
```

## 6. 第一版参数验收

第一版：

```text
min_gain = 0.25
max_gain = 1.00
low/high acceleration = 0.05 / 0.35 m/s²
filter_gain = 0.20
```

### 6.1 0.4 m/s 匀速

Bag：

```text
p4_constant_predff_h0p10_vff1p0_rvg0p25_adapt_g0p25-1p0_a0p05-0p35_f0p20_20260724_225414
```

结果：

```text
水平位置 RMSE = 0.0562 m
最大水平误差 = 0.1281 m
有效增益 min / mean / max = 0.2500 / 0.2501 / 0.2572
```

匀速门槛通过，说明估计噪声不会误触发高阻尼。

### 6.2 XY 正弦

Bag：

```text
p4_sinusoidal_predff_h0p10_vff1p0_rvg0p25_adapt_g0p25-1p0_a0p05-0p35_f0p20_20260724_225658
```

结果：

```text
水平位置 RMSE = 0.3705 m
最大水平误差 = 0.5641 m
有效增益 min / mean / max = 0.2500 / 0.9540 / 1.0000
```

调度行为正确，但略高于预设门槛 `0.36 m / 0.55 m`。由于正弦阶段增益已经接近饱和，继续降低加速度阈值没有意义，因此只进行一次小范围修正：将 `max_gain` 从 `1.0` 提高到 `1.2`。

## 7. 最终参数验收

### 7.1 XY 正弦

Bag：

```text
p4_sinusoidal_predff_h0p10_vff1p0_rvg0p25_adapt_g0p25-1p20_a0p05-0p35_f0p20_20260724_230121
```

| 指标 | P4.5 固定默认 | P4.7 自适应默认 | 改善 |
| --- | ---: | ---: | ---: |
| 水平位置 RMSE | 0.4812 m | 0.3490 m | 27.5% |
| 相对速度 RMSE | 0.5086 m/s | 0.3663 m/s | 28.0% |
| 最大水平误差 | 0.7367 m | 0.5131 m | 30.4% |
| 预测位置 RMSE | 0.1873 m | 0.1647 m | 12.1% |

动力学与调度：

```text
最大水平速度 = 1.2265 m/s
最大水平加速度 = 1.2224 m/s²
最大滚转角 = 6.139°
最大俯仰角 = 3.717°
有效增益 min / mean / max = 0.2500 / 1.1406 / 1.2000
估计甲板加速度 RMSE / max = 0.5142 / 0.7908 m/s²
```

满足：

```text
水平位置 RMSE <= 0.36 m
最大水平误差 <= 0.55 m
```

### 7.2 0.4 m/s 匀速交叉验证

Bag：

```text
p4_constant_predff_h0p10_vff1p0_rvg0p25_adapt_g0p25-1p20_a0p05-0p35_f0p20_20260724_230425
```

结果：

```text
水平位置 RMSE = 0.0554 m
相对速度 RMSE = 0.0187 m/s
最大水平误差 = 0.1036 m
预测位置 RMSE = 0.0471 m
有效增益 min / mean / max = 0.2500 / 0.2502 / 0.2697
估计甲板加速度 RMSE / max = 0.0302 / 0.0757 m/s²
```

满足：

```text
水平位置 RMSE <= 0.065 m
最大水平误差 <= 0.13 m
稳定阶段有效增益接近 0.25
```

相比 P4.5 固定默认的 `0.0510 m`，匀速 RMSE 仅增加约 `0.0044 m`，仍明显优于固定高阻尼 `0.0858~0.0954 m`。

## 8. 安全与日志检查

四轮 P4.7 SITL 均满足：

- 正常进入并持续保持 `TRACK_TARGET`；
- Marker 丢失次数为 0；
- GNSS 恢复次数为 0；
- 未进入下降、Land、`DONE` 或 `ABORT`；
- 无 `timestamp synchronization failed`；
- 无 `pose history insertion failed`；
- 无 `full camera-to-NED transform failed`；
- 无 `Moving target tracking input unavailable`；
- rosbag 和全部子进程按 SIGINT 正常清理。

## 9. 阶段结论

P4.7 已完成代码、单元测试、调试接口和真实 PX4 SITL 交叉验证。

最终自适应调度同时实现：

- 匀速阶段保持接近 `0.25` 的低阻尼和较小稳态误差；
- 正弦换向阶段连续提升至 `1.2`，显著降低位置与相对速度误差；
- 同一套统一参数覆盖两种运动类型，不需要离散场景分类或手工切换参数。

因此 P4.7 参数正式设为主配置默认值。传统基线的下降前水平跟踪阶段通过，下一阶段可以进入 P5：规则式着陆窗口与分阶段下降。
