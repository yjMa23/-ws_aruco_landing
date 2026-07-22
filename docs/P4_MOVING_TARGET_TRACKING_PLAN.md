# P4 移动甲板水平跟踪详细实施计划

## 1. 阶段目标

在 P3 已完成视觉位置、速度估计与短时预测的基础上，将预测甲板位置和甲板速度前馈接入 PX4 Offboard `TrajectorySetpoint`，实现安全高度下的移动甲板水平跟踪。

本阶段只解决水平跟踪，不执行下降、着陆窗口判断、触地检测或自动 Land。

目标主路径：

```text
TRACK_TARGET
→ 预测甲板水平位置作为 position setpoint
→ 甲板估计速度与相对速度阻尼作为 velocity feedforward
→ PX4 内部位置/速度环完成闭环跟踪
```

异常路径保持：

```text
视觉短时丢失
→ 受限短时预测 + 速度前馈衰减

视觉长时丢失
→ RECOVER_TO_GNSS
```

## 2. 设计原则

### 2.1 不重复实现 PX4 位置环

PX4 多旋翼位置控制支持同时发送位置和速度设定点：

- 有限位置设定点决定运行位置控制器；
- 有限速度设定点作为位置控制器输出的速度前馈。

因此 P4 外部控制器不重复加入 `Kp * position_error`，避免与 PX4 内部位置环重复放大。

外部水平速度前馈采用：

```text
v_ff_xy = velocity_feedforward_gain * v_deck_xy
        + relative_velocity_gain * (v_deck_xy - v_uav_xy)
```

位置误差由 PX4 内部位置环根据预测甲板位置设定点处理。

### 2.2 控制与估计解耦

P4 控制器只接收以下数学量：

- 原始视觉水平位置；
- 估计甲板水平位置和速度；
- 短时预测水平位置；
- 无人机 local NED 水平位置和速度；
- 当前水平目标；
- 视觉是否新鲜、估计年龄和控制周期。

纯控制模块不依赖 ROS、PX4 消息或状态机。

### 2.3 保留消融模式

运行参数 `tracking.mode` 支持：

```text
RAW_VISUAL_POSITION
ESTIMATED_POSITION
ESTIMATED_POSITION_VELOCITY_FF
PREDICTED_POSITION_VELOCITY_FF
```

含义：

| 模式 | 水平位置目标 | 速度前馈 |
| --- | --- | --- |
| `RAW_VISUAL_POSITION` | P2D 原始视觉位置 | 无 |
| `ESTIMATED_POSITION` | Kalman 估计位置 | 无 |
| `ESTIMATED_POSITION_VELOCITY_FF` | Kalman 估计位置 | 甲板速度 + 相对速度阻尼 |
| `PREDICTED_POSITION_VELOCITY_FF` | 控制时刻预测位置 | 甲板速度 + 相对速度阻尼 |

默认使用：

```text
PREDICTED_POSITION_VELOCITY_FF
```

## 3. 计划文件

```text
src/aruco_precision_landing_cpp/
├── include/aruco_precision_landing_cpp/
│   └── moving_target_tracking_controller.hpp
├── src/
│   └── moving_target_tracking_controller.cpp
└── test/
    └── moving_target_tracking_controller_test.cpp
```

节点继续在：

```text
src/aruco_precision_landing_cpp/src/px4_aruco_landing_node.cpp
```

完成 ROS 和 PX4 消息接入。

## 4. 纯控制模块接口

### 4.1 输入

```text
current_target_xy
raw_visual_position_xy       optional
estimated_state              optional
predicted_position_xy        optional
uav_position_xy
uav_velocity_xy              optional
visual_fresh
estimate_age_s
dt_s
```

### 4.2 输出

```text
position_target_xy
velocity_feedforward_xy      optional
source_mode
used_prediction
used_short_loss_prediction
```

### 4.3 失败行为

以下情况返回失败，不生成新指令：

- 当前目标、无人机状态或时间含 NaN / Inf；
- `dt <= 0`；
- 当前模式所需位置源不存在；
- 估计年龄为负或超过短时预测范围；
- 需要相对速度反馈但无人机速度无效；
- 估计速度或预测位置非法。

节点失败时保持最近安全水平目标，并清除速度前馈，不进入下降。

## 5. 位置目标约束

期望位置源选定后，对目标变化同时施加：

```text
max_position_target_speed_mps
max_position_target_step_m
```

单周期最大位移：

```text
max_delta_position
=
min(max_position_target_speed_mps * dt,
    max_position_target_step_m)
```

超过范围时沿期望方向缩放。

## 6. 速度前馈约束

### 6.1 速度幅值限制

```text
norm(v_ff_xy) <= max_velocity_feedforward_mps
```

### 6.2 加速度限制

控制器保存上一周期有效前馈速度：

```text
norm(v_ff_k - v_ff_{k-1})
<= max_velocity_feedforward_acceleration_mps2 * dt
```

进入非前馈模式、退出 `TRACK_TARGET` 或执行 reset 时清除历史。

### 6.3 视觉短时丢失衰减

估计年龄未超过 `max_prediction_age_s` 时，预测位置仍可使用。

速度前馈缩放：

```text
loss_scale
=
clamp(1 - estimate_age_s / max_prediction_age_s, 0, 1)

v_ff_xy = loss_scale * v_ff_xy
```

防止视觉丢失后持续按旧速度无限运动。

原始视觉模式不支持无新鲜视觉时继续更新目标，节点保持最近目标。

## 7. PX4 接入

### 7.1 OffboardControlMode

继续使用：

```text
position = true
velocity = false
```

位置字段有限时 PX4 运行位置控制器；`TrajectorySetpoint.velocity` 的有限水平分量作为前馈。

### 7.2 TrajectorySetpoint

P4 全模式都发布：

```text
position = [target_x, target_y, -rendezvous_altitude_m]
```

前馈模式发布：

```text
velocity = [velocity_ff_north, velocity_ff_east, NaN]
```

无前馈模式发布：

```text
velocity = [NaN, NaN, NaN]
```

垂直速度始终不由 P4 控制。

## 8. ROS 2 调试输出

新增：

```text
/landing/tracking_velocity_setpoint
geometry_msgs/msg/TwistStamped
```

坐标：

```text
frame_id = local_ned
linear.x = North velocity feedforward
linear.y = East velocity feedforward
linear.z = NaN 或 0，仅用于调试，不下发垂直前馈
```

`/landing/guidance_source` 在 `TRACK_TARGET` 中根据模式输出：

```text
VISION_RAW
VISION_ESTIMATED
VISION_ESTIMATED_FF
VISION_PREDICTED_FF
```

## 9. 参数

建议默认值：

```yaml
tracking.mode: PREDICTED_POSITION_VELOCITY_FF
tracking.max_position_target_speed_mps: 2.0
tracking.max_position_target_step_m: 0.20
tracking.velocity_feedforward_gain: 1.0
tracking.relative_velocity_gain: 0.25
tracking.max_velocity_feedforward_mps: 1.5
tracking.max_velocity_feedforward_acceleration_mps2: 1.0
tracking.max_prediction_age_s: 0.75
```

参数必须满足：

- 所有速度、加速度、步长和预测年龄为有限正数；
- `velocity_feedforward_gain >= 0`；
- `relative_velocity_gain >= 0`；
- 模式字符串必须属于允许集合。

## 10. 状态机接入

### 10.1 `VISUAL_HANDOVER`

保持 P2D 线性接管逻辑，不使用速度前馈，避免同时引入两种切换动态。

### 10.2 `TRACK_TARGET`

每周期：

1. 获取视觉丢失状态；
2. 获取估计状态与控制时刻预测位置；
3. 构造控制器输入；
4. 计算受限位置目标和速度前馈；
5. 水平位置写入 `set_target()`；
6. 有效速度前馈写入 TrajectorySetpoint；
7. z 始终为 `-rendezvous_altitude_m`。

### 10.3 短时视觉丢失

- 原始视觉模式：保持最近目标、清除速度前馈；
- 估计/预测模式：在 `max_prediction_age_s` 内继续受限预测；
- 速度前馈随估计年龄衰减；
- 超过预测年龄但尚未达到视觉长时丢失阈值时保持最近目标并清除前馈。

### 10.4 长时视觉丢失

保持现有：

```text
TRACK_TARGET → RECOVER_TO_GNSS
```

## 11. 单元测试

至少覆盖：

1. 模式字符串解析；
2. 原始视觉模式使用原始位置且无前馈；
3. 估计位置模式不使用预测；
4. 估计位置 + 速度前馈；
5. 预测位置 + 速度前馈；
6. 相对速度阻尼方向；
7. 位置目标速度和单步限制；
8. 速度幅值限制；
9. 速度加速度限制；
10. 短时丢失前馈衰减；
11. 超过预测年龄拒绝继续预测；
12. reset 清除前馈历史；
13. NaN、Inf、非法参数和无效输入拒绝。

## 12. 消息级验收

### 12.1 静止目标

- 状态进入 `TRACK_TARGET`；
- 预测位置稳定；
- 速度前馈收敛到接近零；
- 目标高度始终 `-5 m`；
- 不进入下降状态。

### 12.2 NED East `0.4 m/s`

- `estimated_deck_odometry.twist.twist.linear.y ≈ 0.4 m/s`；
- `tracking_velocity_setpoint.twist.linear.y` 为正并受加速度限制逐渐接近目标；
- `TrajectorySetpoint.velocity[1]` 与调试话题一致；
- 预测位置和 position setpoint 沿 East 增加；
- z 仍为 `-5 m`。

### 12.3 短时视觉丢失

- 在预测年龄内继续输出连续位置目标；
- 速度前馈逐渐减小；
- 不立即进入 GNSS 恢复；
- 达到长时丢失阈值后进入 `RECOVER_TO_GNSS`。

## 13. 真实仿真验收

建议按顺序：

1. 静止甲板；
2. `0.2 m/s` 匀速甲板；
3. `0.4 m/s` 匀速甲板；
4. XY 正弦甲板。

对比模式：

```text
RAW_VISUAL_POSITION
ESTIMATED_POSITION_VELOCITY_FF
PREDICTED_POSITION_VELOCITY_FF
```

记录：

- 水平位置 RMSE；
- 水平相对速度 RMSE；
- 最大水平误差；
- Marker 丢失次数；
- 状态恢复次数；
- setpoint 最大速度和加速度；
- 预测位置相对 Ground Truth 的 RMSE。

Ground Truth 只能进入 rosbag 离线评测，不得进入控制器。

## 14. P4 完成标准

以下全部满足才标记完成：

- 纯控制模块和单元测试通过；
- 预测位置和速度前馈已接入 `TRACK_TARGET`；
- 原始视觉模式可以通过 YAML 恢复；
- 速度和加速度有限且可配置；
- 短时丢失行为连续，长时丢失回 GNSS；
- 全过程保持安全高度；
- 控制器不订阅 Ground Truth；
- 完整工作区构建和测试通过；
- 消息级静止与 `0.4 m/s` 匀速验收通过；
- 真实 PX4 仿真 RMSE 在进入 P5 前由用户手工确认。

## 15. 阶段标签

```text
baseline-moving-tracking-v0.1
```
