# P4.7 加速度感知相对速度增益调度执行计划

## 1. 阶段目标

解决 P4.6 暴露的固定增益冲突：

- `relative_velocity_gain=0.25` 在静止和匀速场景稳态误差较小，但正弦换向误差较大；
- `relative_velocity_gain=1.00` 可将正弦位置 RMSE 降至 `0.3439 m`，但会将 0.4 m/s 匀速 RMSE 从 `0.0510 m` 恶化到 `0.0858~0.0954 m`。

P4.7 根据甲板估计水平加速度，在低加速度阶段使用低阻尼，在加速和换向阶段连续提升
阻尼，使同一套参数兼顾匀速和周期运动。

## 2. 明确不做

本阶段不实现：

- 常加速度 Kalman Filter；
- MPC；
- 强化学习；
- 着陆窗口或下降；
- 根据 Ground Truth 调度；
- 离散场景分类器；
- 在线参数优化。

## 3. 调度器设计

新增纯 C++ 模块：

```text
include/aruco_precision_landing_cpp/adaptive_relative_velocity_gain.hpp
src/adaptive_relative_velocity_gain.cpp
test/adaptive_relative_velocity_gain_test.cpp
```

输入：

```text
deck_velocity_xy
sample_dt_s
```

内部状态：

```text
previous_deck_velocity_xy
filtered_deck_acceleration_xy
```

原始加速度：

```text
a_raw = (v_deck[k] - v_deck[k-1]) / dt
```

先对加速度模长进行限幅，再执行一阶低通：

```text
a_limited = clamp_norm(a_raw, max_acceleration)
a_filtered = a_filtered + filter_gain * (a_limited - a_filtered)
```

根据过滤后的水平加速度模长连续调度：

```text
alpha = clamp(
  (|a_filtered| - low_threshold) /
  (high_threshold - low_threshold),
  0,
  1)

gain = min_gain + smoothstep(alpha) * (max_gain - min_gain)
```

其中：

```text
smoothstep(alpha) = alpha^2 * (3 - 2 * alpha)
```

第一版参数：

```yaml
tracking.adaptive_relative_velocity_gain.enabled: true
tracking.adaptive_relative_velocity_gain.min_gain: 0.25
tracking.adaptive_relative_velocity_gain.max_gain: 1.00
tracking.adaptive_relative_velocity_gain.acceleration_low_threshold_mps2: 0.05
tracking.adaptive_relative_velocity_gain.acceleration_high_threshold_mps2: 0.35
tracking.adaptive_relative_velocity_gain.max_acceleration_mps2: 1.50
tracking.adaptive_relative_velocity_gain.acceleration_filter_gain: 0.20
```

## 4. 控制器接入

修改：

```text
moving_target_tracking_controller.hpp/.cpp
px4_aruco_landing_node.hpp/.cpp
config/px4_aruco_landing.yaml
launch/px4_aruco_landing.launch.py
scripts/start_sitl.sh
```

要求：

- 仅在带速度前馈的两种模式中启用调度；
- 调度关闭时严格保持现有固定 `relative_velocity_gain` 行为；
- 视觉短时丢失时使用最近有效调度增益，但速度前馈仍按现有 loss scale 衰减；
- reset、退出 `TRACK_TARGET` 或切换模式时清除加速度历史；
- 调度器输入只使用视觉估计甲板速度，不使用 Ground Truth；
- 无有效 `dt`、速度或历史时返回 `min_gain`，不产生 NaN；
- 输出命令增加有效相对速度增益和过滤后甲板加速度，便于调试。

## 5. 调试接口

新增：

```text
/landing/effective_relative_velocity_gain
std_msgs/msg/Float64

/landing/estimated_deck_acceleration
geometry_msgs/msg/TwistStamped
```

语义：

```text
frame_id = local_ned
effective gain = 当前实际用于相对速度反馈的增益
linear.x / linear.y = 过滤后的 North / East 甲板加速度
```

两个话题加入 P4 rosbag 记录列表，但不进入控制输入。

## 6. 单元测试

至少覆盖：

1. 构造参数校验；
2. 第一帧返回最小增益；
3. 匀速输入增益保持最小值；
4. 低于低阈值保持最小值；
5. 高于高阈值达到最大值；
6. 中间区间连续单调；
7. 加速度限幅；
8. 低通滤波；
9. 非法 dt、NaN 和乱序输入拒绝；
10. reset 清除历史；
11. 控制器调度关闭时与原固定增益结果完全一致；
12. 控制器调度开启时使用有效动态增益；
13. 短时丢失仍执行现有前馈衰减。

## 7. PX4 SITL 验收

只运行两个关键场景：

### 7.1 0.4 m/s 匀速

目标：

```text
水平位置 RMSE <= 0.065 m
最大水平误差 <= 0.13 m
稳定阶段有效增益接近 0.25
```

### 7.2 XY 正弦

目标：

```text
水平位置 RMSE <= 0.36 m
最大水平误差 <= 0.55 m
换向阶段有效增益能够连续接近 1.0
```

共同要求：

- Marker 丢失次数为 0；
- GNSS 恢复次数为 0；
- 无时间同步、位姿历史、坐标变换和控制输入异常；
- 不进入下降或 Land。

## 8. 默认参数决策

只有两个场景同时通过门槛时，才将自适应调度设为主 YAML 默认启用。

若匀速或正弦任一场景失败：

- 保持当前固定增益默认配置；
- 根据实际有效增益曲线调整阈值或滤波系数；
- 最多进行两轮小范围修正，不进行大网格扫描。

## 9. 执行状态（2026-07-24）

- 纯 C++ 调度器、控制器接入、ROS 参数和调试话题已完成。
- 全工作区 118 项测试全部通过。
- 第一版 `max_gain=1.0` 完成匀速与正弦交叉验证，正弦指标略高于门槛。
- 按计划仅进行一轮小修正，将 `max_gain` 提升到 `1.2`。
- 最终参数在 0.4 m/s 匀速和 XY 正弦场景均通过门槛。
- 自适应调度已设为统一默认，固定增益保留为消融入口。
- 详细验收见 `docs/P4_7_ADAPTIVE_GAIN_SCHEDULING_VALIDATION.md`。
