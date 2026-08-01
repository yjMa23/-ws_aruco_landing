# P4.6 XY 正弦运动跟踪参数优化验收记录

## 1. 阶段范围

P4.6 在 P4.5 时间对齐基础上，通过真实 PX4 SITL 参数扫描，分析固定常速度预测时域和
相对速度阻尼对 XY 正弦换向跟踪的影响。

本阶段没有修改：

- Kalman Filter 状态模型；
- PX4 内部位置控制器；
- 跟踪控制公式；
- 状态机；
- 安全高度；
- 着陆窗口、下降和触地逻辑。

## 2. 可复现实验入口

`px4_aruco_landing.launch.py` 和 `scripts/start_sitl.sh` 新增参数覆盖入口：

```text
--tracking-mode
--prediction-horizon
--velocity-ff-gain
--relative-velocity-gain
```

默认值仍与主 YAML 一致：

```text
tracking_mode = PREDICTED_POSITION_VELOCITY_FF
prediction_horizon_s = 0.10
velocity_feedforward_gain = 1.0
relative_velocity_gain = 0.25
```

使用调参参数时，rosbag 目录名自动记录模式和数值，例如：

```text
p4_sinusoidal_estff_h0p00_vff1p0_rvg1p00_<timestamp>
```

因此每轮结果可以从目录名追溯到启动参数。

## 3. P4.5 基准

基准场景：

```text
XY 正弦
amplitude_xy = [1.0, 0.5] m
period_xy = [10.0, 6.0] s
```

基准参数：

```text
PREDICTED_POSITION_VELOCITY_FF
additional_prediction_horizon_s = 0.10
velocity_feedforward_gain = 1.0
relative_velocity_gain = 0.25
```

基准结果：

```text
水平位置 RMSE = 0.4812 m
相对速度 RMSE = 0.5086 m/s
最大水平误差 = 0.7367 m
预测位置 RMSE = 0.1873 m
```

## 4. 预测时域扫描

固定：

```text
PREDICTED_POSITION_VELOCITY_FF
velocity_feedforward_gain = 1.0
relative_velocity_gain = 0.25
```

| 额外预测时域 | Bag | 水平位置 RMSE | 相对速度 RMSE | 最大水平误差 | 预测位置 RMSE |
| ---: | --- | ---: | ---: | ---: | ---: |
| 0.10 s | `p4_sinusoidal_20260724_215104` | 0.4812 m | 0.5086 m/s | 0.7367 m | 0.1873 m |
| 0.05 s | `p4_sinusoidal_predff_h0p05_vff1p0_rvg0p25_20260724_220440` | 0.4740 m | 0.4987 m/s | 0.7110 m | 0.1573 m |
| 0.00 s | `p4_sinusoidal_predff_h0p00_vff1p0_rvg0p25_20260724_220140` | 0.4643 m | 0.4870 m/s | 0.7282 m | 0.1284 m |

结论：

- 固定额外预测时域越大，正弦换向场景越容易过预测；
- 将额外时域从 `0.10 s` 降至 `0.00 s`，位置 RMSE 仅改善约 `3.5%`；
- 预测位置 RMSE 明显改善，但无人机实际跟踪 RMSE 仍高，说明主要瓶颈不只在位置外推。

## 5. 模式对比

使用：

```text
ESTIMATED_POSITION_VELOCITY_FF
additional_prediction_horizon_s = 0.00
velocity_feedforward_gain = 1.0
relative_velocity_gain = 0.25
```

结果：

| 模式 | Bag | 水平位置 RMSE | 相对速度 RMSE | 最大水平误差 |
| --- | --- | ---: | ---: | ---: |
| 零额外预测 | `p4_sinusoidal_predff_h0p00_vff1p0_rvg0p25_20260724_220140` | 0.4643 m | 0.4870 m/s | 0.7282 m |
| 估计位置前馈 | `p4_sinusoidal_estff_h0p00_vff1p0_rvg0p25_20260724_220737` | 0.4644 m | 0.4896 m/s | 0.7060 m |

两者位置 RMSE 几乎相同，说明在当前时延和采样率下，额外位置外推不是正弦场景的主要
误差来源。

## 6. 相对速度阻尼扫描

固定：

```text
ESTIMATED_POSITION_VELOCITY_FF
additional_prediction_horizon_s = 0.00
velocity_feedforward_gain = 1.0
```

| relative_velocity_gain | 水平位置 RMSE | 相对速度 RMSE | 最大水平误差 | 最大水平速度 | 最大水平加速度 |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 0.10 | 0.5094 m | 0.5337 m/s | 0.7734 m | 1.3696 m/s | — |
| 0.25 | 0.4644 m | 0.4896 m/s | 0.7060 m | — | — |
| 0.40 | 0.4295 m | 0.4508 m/s | 0.6508 m | 1.2534 m/s | 1.0304 m/s² |
| 0.60 | 0.3919 m | 0.4114 m/s | 0.5906 m | 1.2039 m/s | 0.9942 m/s² |
| 0.80 | 0.3637 m | 0.3810 m/s | 0.5398 m | 1.1661 m/s | 0.9593 m/s² |
| 1.00 | 0.3439 m | 0.3604 m/s | 0.5138 m | 1.1229 m/s | 0.9312 m/s² |

对应 rosbag：

```text
p4_sinusoidal_estff_h0p00_vff1p0_rvg0p10_20260724_221100
p4_sinusoidal_estff_h0p00_vff1p0_rvg0p25_20260724_220737
p4_sinusoidal_estff_h0p00_vff1p0_rvg0p40_20260724_221411
p4_sinusoidal_estff_h0p00_vff1p0_rvg0p60_20260724_221758
p4_sinusoidal_estff_h0p00_vff1p0_rvg0p80_20260724_222105
p4_sinusoidal_estff_h0p00_vff1p0_rvg1p00_20260724_222403
```

结论：

- 降低相对速度增益会明显恶化跟踪，说明该项不是换向时的错误放大项；
- 增大相对速度增益能够抑制无人机相对甲板的速度过冲；
- 在本轮扫描范围内，`1.00` 是正弦场景最佳值；
- 位置 RMSE、相对速度 RMSE、最大误差、速度和加速度峰值均同步下降，没有以更激烈运动换取位置误差改善。

## 7. 最佳正弦结果

正弦场景最佳实验参数：

```text
tracking_mode = ESTIMATED_POSITION_VELOCITY_FF
additional_prediction_horizon_s = 0.00
velocity_feedforward_gain = 1.0
relative_velocity_gain = 1.00
```

与 P4.5 基准对比：

| 指标 | P4.5 基准 | P4.6 最佳 | 改善 |
| --- | ---: | ---: | ---: |
| 水平位置 RMSE | 0.4812 m | 0.3439 m | 28.5% |
| 相对速度 RMSE | 0.5086 m/s | 0.3604 m/s | 29.1% |
| 最大水平误差 | 0.7367 m | 0.5138 m | 30.3% |
| 预测位置 RMSE | 0.1873 m | 0.1168 m | 37.6% |
| 最大水平速度 | 1.3518 m/s | 1.1229 m/s | 16.9% |
| 最大水平加速度 | 1.1699 m/s² | 0.9312 m/s² | 20.4% |

此外：

- Marker 丢失次数为 0；
- GNSS 恢复次数为 0；
- 最大滚转角约 `5.09°`；
- 最大俯仰角约 `3.28°`；
- 没有时间同步、位姿历史、坐标变换、控制输入不可用、恢复或中止异常。

P4.6 第一阶段 `RMSE < 0.40 m` 目标已达到，但最终建议目标 `RMSE < 0.30 m` 尚未达到。

## 8. 匀速交叉验证

为判断最佳正弦参数能否作为统一默认参数，额外运行 `0.4 m/s` 匀速场景。

| 模式 | horizon | rvg | 水平位置 RMSE | 最大水平误差 |
| --- | ---: | ---: | ---: | ---: |
| 当前默认预测前馈 | 0.10 | 0.25 | 0.0510 m | 0.1137 m |
| 估计位置前馈 | 0.00 | 1.00 | 0.0954 m | 0.1535 m |
| 预测前馈 | 0.00 | 1.00 | 0.0858 m | 0.1551 m |

对应交叉验证 bag：

```text
p4_constant_estff_h0p00_vff1p0_rvg1p00_20260724_222722
p4_constant_predff_h0p00_vff1p0_rvg1p00_20260724_223011
```

高阻尼参数虽然改善周期换向运动，但会使匀速跟踪产生更大的稳态滞后。因此不能将
`relative_velocity_gain=1.0` 直接设为所有场景的统一默认值。

## 9. 默认参数决策

主配置保持不变：

```text
tracking.mode = PREDICTED_POSITION_VELOCITY_FF
motion_predictor.additional_prediction_horizon_s = 0.10
tracking.velocity_feedforward_gain = 1.0
tracking.relative_velocity_gain = 0.25
```

原因：

- 当前默认参数在静止、0.2 m/s 和 0.4 m/s 匀速场景中表现稳定；
- 正弦最佳高阻尼参数不具备跨运动类型泛化性；
- 在没有可靠运动状态分类或增益调度前，不应通过修改统一默认值牺牲匀速基线。

正弦实验参数可使用：

```bash
./scripts/start_sitl.sh \
  --scenario sinusoidal \
  --headless \
  --record \
  --tracking-mode ESTIMATED_POSITION_VELOCITY_FF \
  --prediction-horizon 0.00 \
  --relative-velocity-gain 1.00
```

## 10. 阶段结论

P4.6 已证明：

1. 固定额外常速度预测会在周期换向时产生过预测，但不是主要误差来源；
2. 相对速度阻尼是当前正弦跟踪改善的主要有效参数；
3. 单一固定阻尼增益无法同时最优覆盖匀速和周期换向运动；
4. 继续进行更大范围的单参数扫描收益有限，并可能形成场景过拟合。

因此，本阶段停止继续增大阻尼或进行大网格搜索。若希望将正弦 RMSE 进一步降至
`0.30 m` 以下，下一步应研究以下任一方向：

- 基于甲板估计加速度或速度方向变化率的增益调度；
- 具有加速度状态的目标运动模型；
- 对位置误差、相对速度和目标加速度进行统一约束的 MPC。

在传统规则基线中，优先建议先实现简单、可解释的加速度感知增益调度，再决定是否引入
更复杂的常加速度滤波或 MPC。
