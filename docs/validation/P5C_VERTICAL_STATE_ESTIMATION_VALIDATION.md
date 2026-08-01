# P5C 低高度垂直状态估计与标定验收

## 1. 验收结论

P5C 已完成并通过当前传统基线阶段验收。

本阶段完成：

- 修正 PX4 `x500_mono_cam_down` 的相机 z 外参；
- 新增独立二状态垂直 Kalman Filter；
- 新增 Marker z、独立垂直状态、原始相对高度和相对垂直速度调试链；
- 新增 P5C rosbag 离线评测脚本；
- 在相对下降中加入甲板垂直速度和相对高度参考变化速度前馈；
- 完成静止、升沉、不同高度和不同垂直前馈增益对照；
- 保持 P4.7 水平跟踪、P5B 暂停/恢复和禁止自动 Land 的安全边界。

当前最低测试高度仍为 `0.50 m`，没有进入触地、`NAV_LAND` 或 Disarm。

---

## 2. 关键问题定位：相机 z 外参

旧配置：

```yaml
camera_extrinsic.translation_frd_m: [0.0, 0.0, -0.10]
```

Gazebo 实际模型查询：

```text
x500_mono_cam_down/base_link  world z = 0.24 m
x500_mono_cam_down/camera_link world z = 0.10 m
```

因此相机相对 PX4 机体参考点位于下方 `0.14 m`，FRD 中应为：

```yaml
camera_extrinsic.translation_frd_m: [0.0, 0.0, 0.14]
```

旧外参与真实模型相差 `0.24 m`，与旧 P5B 中约 `0.22～0.24 m` 的垂直系统偏差一致。

修正前旧 bag：

| 场景 | 相对高度 bias | 相对高度 RMSE |
| --- | ---: | ---: |
| 静止下降 | `-0.2225 m` | `0.2237 m` |
| 0.4 m/s 匀速下降 | `-0.2261 m` | `0.2271 m` |
| 升沉下降 | `-0.2348 m` | `0.2369 m` |

修正后 `5 m` 安全高度：

| 场景 | 相对高度 bias | 相对高度 RMSE | P95 绝对误差 |
| --- | ---: | ---: | ---: |
| 静止 | `+0.0441 m` | `0.0477 m` | `0.0782 m` |
| 升沉 | `+0.0372 m` | `0.0498 m` | `0.0824 m` |

系统误差由约 `0.23 m` 降至约 `0.05 m`。

---

## 3. 独立垂直状态估计器

新增：

```text
src/aruco_precision_landing_cpp/include/aruco_precision_landing_cpp/vertical_state_estimator.hpp
src/aruco_precision_landing_cpp/src/vertical_state_estimator.cpp
src/aruco_precision_landing_cpp/test/vertical_state_estimator_test.cpp
```

状态：

```text
x = [deck_z_ned, deck_vz_ned]
```

支持：

- 独立 z 轴过程噪声、测量噪声和初始协方差；
- 可配置测量偏置；
- 图像采样时间更新；
- 重复、倒退、NaN、Inf 和离群点拒绝；
- 长间隔重初始化；
- 预测时域上限；
- reset。

新增调试话题：

```text
/landing/vertical_state
/landing/raw_relative_height
/landing/relative_vertical_velocity
```

垂直估计器模块 11 项测试全部通过。

### 控制接入结论

独立估计器在 `5 m` 升沉安全高度中将 deck z RMSE 从旧预测值 `0.0544 m` 降至 `0.0377 m`；但在低高度动态下降中，其位置输出没有持续稳定优于旧三维估计器。

因此当前实现采用：

- P4.7 水平位置和 P5B 垂直位置目标继续使用旧三维状态估计器；
- 独立垂直估计器用于垂直速度前馈和并行诊断；
- 没有为满足计划形式而强行切换位置控制源。

Ground Truth 仅进入离线评测脚本，从未进入控制器。

---

## 4. 垂直速度前馈

实现：

```text
velocity_target_z = gain * estimated_deck_vz + height_reference_rate_ned
```

其中：

- NED Down 为正；
- 下降时 `height_reference_rate_ned > 0`；
- 暂停和测试高度保持时参考速度为 `0`；
- 恢复爬升时参考速度为负；
- 无有效垂直估计时不发布 z 速度前馈；
- 独立限幅默认 `0.60 m/s`。

最终默认参数：

```yaml
vertical_velocity_feedforward.enabled: true
vertical_velocity_feedforward.deck_velocity_gain: 1.0
vertical_velocity_feedforward.max_abs_mps: 0.60
```

消融入口：

```bash
./scripts/start_sitl.sh --disable-vertical-ff
```

### 增益对照：升沉甲板，测试高度 0.70 m

| 设置 | 实际参考跟踪 RMSE | Ground Truth 最低高度 | 结论 |
| --- | ---: | ---: | --- |
| z 前馈关闭 | `0.1605 m` | `0.5210 m` | 跟踪滞后明显 |
| gain `1.0` | `0.0918 m` | `0.5521 m` | 最优候选 |
| gain `1.5` | `0.1338 m` | `0.5005 m` | 过度前馈，停止提高 |
| 默认链复验 gain `1.0` | `0.0886 m` | `0.5604 m` | 通过 |

默认参数相对关闭前馈：

- 实际参考跟踪 RMSE 改善约 `44.8%`；
- 最低 Ground Truth 相对高度提高约 `0.039 m`；
- z 目标最大单步变化仍为 `0.0309 m`。

### 静止交叉验证：测试高度 0.70 m

```text
实际参考跟踪 RMSE: 0.0258 m
Ground Truth 最低高度: 0.6406 m
z 目标最大单步变化: 0.0198 m
```

垂直前馈没有破坏静止甲板表现。

---

## 5. 低高度标定结果

### 5.1 静止甲板，测试高度 0.70 m

Bag：

```text
bags/p5b_static_descent_predff_h0p10_vff1p0_rvg0p25_adapt_g0p25-1p2_a0p05-0p35_f0p20_hmin0p70_20260725_130104
```

无 z 前馈的初始标定：

```text
Ground Truth 最低高度: 0.6767 m
0.5～1.0 m 相对高度 RMSE: 0.0174 m
参考最低高度: 0.7000 m
NAV_LAND / Disarm: 0 / 0
```

### 5.2 静止甲板，测试高度 0.50 m

Bag：

```text
bags/p5b_static_descent_predff_h0p10_vff1p0_rvg0p25_adapt_g0p25-1p2_a0p05-0p35_f0p20_20260725_130618
```

结果：

```text
Ground Truth 最低高度: 0.4654 m
参考最低高度: 0.5000 m
0.5～1.0 m 相对高度 RMSE: 0.0136 m
低于 0.5 m 样本相对高度 RMSE: 0.0255 m
P95 绝对误差: 0.0413 m
z 目标最大单步变化: 0.0198 m
NAV_LAND / Disarm: 0 / 0
```

Ground Truth 最低高度高于 `0.45 m` 验收线，但余量仅约 `0.015 m`，因此仍不允许继续向触地高度下降。

### 5.3 升沉甲板，默认 z 前馈，测试高度 0.70 m

Bag：

```text
bags/p5b_heave_descent_zff1p0_predff_h0p10_vff1p0_rvg0p25_adapt_g0p25-1p2_a0p05-0p35_f0p20_hmin0p70_20260725_140727
```

结果：

```text
Ground Truth 最低高度: 0.5604 m
参考最低高度: 0.7000 m
实际参考跟踪 RMSE: 0.0886 m
0.5～1.0 m 相对高度 RMSE: 0.0338 m
P95 绝对误差: 0.0612 m
相对垂直速度 RMSE: 0.0569 m/s
z 目标最大单步变化: 0.0309 m
NAV_LAND / Disarm: 0 / 0
```

由于升沉 `0.70 m` 的最低真实高度仍只有 `0.5604 m`，本阶段没有运行升沉 `0.50 m`。这是主动安全限制，不属于缺失执行。

---

## 6. P4.7 水平跟踪回归

### 升沉默认下降

```text
水平位置 RMSE: 0.0251 m
相对速度 RMSE: 0.0169 m/s
最大水平误差: 0.0510 m
Marker 丢失 / GNSS 恢复: 0 / 0
```

### 静止默认下降

```text
水平位置 RMSE: 0.0333 m
相对速度 RMSE: 0.0192 m/s
最大水平误差: 0.0618 m
Marker 丢失 / GNSS 恢复: 0 / 0
```

相机 z 外参修正和垂直控制扩展没有破坏 P4.7 水平链。

---

## 7. 安全边界

所有正式验收 bag 均满足：

```text
NAV_LAND 命令: 0
Disarm 命令: 0
旧下降状态: 未进入
ABORT: 未进入
窗口关闭时参考继续下降: 0
时间同步异常: 0
位姿历史异常: 0
相机到 NED 变换异常: 0
```

当前默认启动仍需显式传入：

```bash
--enable-relative-descent
```

否则只在 `WAIT_LANDING_WINDOW` 保持安全高度。

---

## 8. 自动评测

```bash
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash

python3 scripts/evaluate_p5c_vertical_estimation.py <bag_directory>
python3 scripts/evaluate_p5b_bag.py <bag_directory>
python3 scripts/evaluate_p4_bag.py <bag_directory>
```

P5C 脚本输出：

- Marker、旧三维估计、旧预测和独立垂直估计的 deck z 误差；
- 原始和控制使用的相对高度误差；
- 新旧相对垂直速度误差；
- 按真实相对高度分箱统计；
- 时延估计；
- 最低参考和 Ground Truth 高度。

---

## 9. 测试结果

```text
3 packages finished
166 tests
0 errors
0 failures
0 skipped
```

新增测试覆盖：

- 垂直估计器初始化、静止、匀速、偏置、噪声抑制；
- 重复/倒退时间和非法输入；
- 离群点和长间隔重初始化；
- 协方差和 reset；
- 相对下降输出的垂直参考速度；
- 下降、暂停、测试高度保持和恢复爬升速度方向。

---

## 10. P5C 最终判定

| 验收项 | 门槛 | 结果 | 判定 |
| --- | ---: | ---: | --- |
| 静止 0.5～1.0 m 相对高度 RMSE | `≤ 0.10 m` | `0.0136 m` | 通过 |
| 升沉 0.5～1.0 m 相对高度 RMSE | `≤ 0.15 m` | `0.0338 m` | 通过 |
| 相对高度 P95 绝对误差 | `≤ 0.20 m` | 最大正式结果 `0.0634 m` | 通过 |
| z 目标最大单步变化 | `≤ 0.05 m` | `0.0309 m` | 通过 |
| Ground Truth 最低高度 | `≥ 0.45 m` | 静止 `0.4654 m`，升沉 `0.5604 m` | 通过 |
| Land / Disarm | `0 / 0` | `0 / 0` | 通过 |
| P4.7 水平回归 | 不恶化 | RMSE `0.0251～0.0333 m` | 通过 |
| 全工作区测试 | 全部通过 | `166/166` | 通过 |

P5C 可以标记为完成。

但进入 P6 时必须保持以下限制：

1. 第一版只实现触地判据和状态确认，不直接自动 Disarm；
2. 触地判据不能仅依赖视觉相对高度；
3. 升沉甲板不得直接从 `0.70 m` 自动下降到触地；
4. 必须使用 PX4 着陆检测、垂直速度、推力或加速度等多源证据；
5. 所有触地逻辑必须默认关闭，并先在静止甲板 SITL 中验证。
