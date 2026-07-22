# P4 移动甲板水平跟踪验收记录

## 1. 阶段范围

P4 在 P3 视觉状态估计和短时预测基础上，将预测位置与甲板速度前馈接入 PX4 Offboard `TrajectorySetpoint`，实现安全高度下的移动目标水平跟踪。

本阶段不实现：

- 着陆窗口；
- 垂直下降；
- 触地检测；
- 自动 Land；
- Ground Truth 控制输入。

## 2. 实现内容

新增纯 C++ 模块：

```text
include/aruco_precision_landing_cpp/moving_target_tracking_controller.hpp
src/moving_target_tracking_controller.cpp
test/moving_target_tracking_controller_test.cpp
```

新增详细计划：

```text
docs/P4_MOVING_TARGET_TRACKING_PLAN.md
```

默认跟踪模式：

```text
PREDICTED_POSITION_VELOCITY_FF
```

控制输出：

```text
position setpoint = 受限预测甲板水平位置
velocity feedforward
  = 甲板估计速度
  + 相对速度阻尼
```

PX4 内部位置控制器继续负责位置误差反馈，外部不重复叠加位置 P。

## 3. 支持的消融模式

```text
RAW_VISUAL_POSITION
ESTIMATED_POSITION
ESTIMATED_POSITION_VELOCITY_FF
PREDICTED_POSITION_VELOCITY_FF
```

已验证：

- `RAW_VISUAL_POSITION` 可以通过参数正常启动；
- 非法模式启动失败并返回退出码 1；
- 默认模式启动日志明确显示 `PREDICTED_POSITION_VELOCITY_FF`。

## 4. 新增 ROS 2 接口

```text
/landing/tracking_velocity_setpoint
geometry_msgs/msg/TwistStamped
```

语义：

```text
frame_id = local_ned
linear.x = North velocity feedforward
linear.y = East velocity feedforward
linear.z = 0，仅用于调试
```

PX4 实际接收：

```text
TrajectorySetpoint.position = [target_north, target_east, -5.0]
TrajectorySetpoint.velocity = [ff_north, ff_east, NaN]
```

无速度前馈时三个速度字段均为 NaN。

## 5. 单元测试

`moving_target_tracking_controller_test` 覆盖 13 项：

1. 四种模式字符串解析和稳定命名；
2. 原始视觉位置模式；
3. 估计位置模式；
4. 预测位置与甲板速度前馈；
5. 相对速度阻尼方向；
6. 位置目标速度限制；
7. 单周期位置步长限制；
8. 前馈速度幅值限制；
9. 前馈加速度限制；
10. 短时丢失预测和前馈衰减；
11. 超过预测年龄拒绝；
12. reset 清除速度历史；
13. 非法参数、NaN、缺失输入拒绝。

当前控制包和全工作区累计：

```text
93 tests
0 errors
0 failures
0 skipped
```

## 6. 静止消息级验收

合成输入：

```text
UAV local NED position = [0, 0, -5]
UAV local NED velocity = [0, 0]
Marker local NED = [0, 0, 0.2]
Deck velocity = [0, 0, 0]
```

状态结果：

```text
ACQUIRE_ARUCO
→ VISUAL_HANDOVER
→ TRACK_TARGET
```

引导来源：

```text
VISION_PREDICTED_FF
```

速度前馈：

```text
/landing/tracking_velocity_setpoint
[0.0, 0.0, 0.0]
```

PX4 TrajectorySetpoint：

```text
velocity = [0.0, 0.0, NaN]
position = [0.0, 0.0, -5.0]
```

结论：静止目标下不会引入虚假水平速度，安全高度保持不变。

## 7. NED East 0.4 m/s 消息级验收

连续 20 Hz 合成视觉序列：

```text
camera_optical.x 按 0.4 m/s 增加
→ local NED East 按 0.4 m/s 增加
```

估计甲板速度：

```text
vx ≈ -8.9e-17 m/s
vy ≈ 0.3999999999 m/s
vz = 0
```

无人机合成速度为零，参数：

```text
velocity_feedforward_gain = 1.0
relative_velocity_gain = 0.25
```

因此期望 East 前馈：

```text
0.4 + 0.25 * (0.4 - 0.0) = 0.5 m/s
```

实测调试前馈：

```text
North ≈ 0
East  ≈ 0.5000000000 m/s
```

PX4 TrajectorySetpoint：

```text
velocity = [约 0, 0.5, NaN]
```

同一时刻：

```text
position target East ≈ 4.2232 m
predicted deck East ≈ 4.5805 m
position target z = -5.0 m
```

位置目标小于预测位置，是位置设定点变化率限制生效的结果，不是方向错误。

结论：

- East 方向正确；
- 速度前馈计算正确；
- PX4 收到与调试话题一致的速度前馈；
- 位置目标连续受限；
- 高度保持安全值。

## 8. 视觉丢失验收

停止连续视觉位姿后：

```text
短时阶段：TRACK_TARGET 保持
超过预测年龄：保持最近安全位置目标并清除速度前馈
超过视觉长时丢失阈值：
TRACK_TARGET → RECOVER_TO_GNSS → ACQUIRE_ARUCO
```

实际状态转换日志符合预期。

纯逻辑测试同时验证了在允许预测年龄内：

```text
loss_scale = clamp(1 - estimate_age / max_prediction_age, 0, 1)
```

速度前馈随视觉丢失时间衰减。

## 9. 安全检查

已确认：

- P4 只修改 `TRACK_TARGET` 水平目标和水平速度前馈；
- `VISUAL_HANDOVER` 不使用速度前馈；
- 每个非跟踪周期清除速度前馈；
- 每次状态切换重置前馈加速度历史；
- 输入不足时保持最近安全位置目标并清除前馈；
- z 始终为 `-rendezvous_altitude_m`；
- 默认 `enable_auto_land=false`；
- P4 主路径没有进入下降状态；
- 控制器与检测器没有订阅 Ground Truth。

## 10. 尚未完成的真实仿真验收

以下项目需要用户在真实 PX4 SITL + Gazebo 相机链路中运行 rosbag 后确认：

- 静止甲板实际悬停误差；
- `0.2 m/s` 和 `0.4 m/s` 匀速甲板水平 RMSE；
- 正弦甲板水平 RMSE；
- `RAW_VISUAL_POSITION` 与前馈/预测模式的定量对比；
- Marker 丢失次数和 GNSS 恢复次数；
- 飞行器实际速度、加速度和姿态是否满足约束；
- 不同前馈增益的稳定性。

Ground Truth 只允许进入 rosbag 离线评测，不允许接入控制器。

## 11. 阶段结论

P4 代码、纯逻辑测试和消息级数据链验收完成。真实 PX4 动力学下的跟踪 RMSE 仍需用户手工测试后确认，再进入 P5 着陆窗口与下降阶段。

阶段标签：

```text
baseline-moving-tracking-v0.1
```
