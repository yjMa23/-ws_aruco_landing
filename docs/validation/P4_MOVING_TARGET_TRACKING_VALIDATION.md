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
docs/plans/P4_MOVING_TARGET_TRACKING_PLAN.md
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

`moving_target_tracking_controller_test` 覆盖 15 项：

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
12. 原始视觉模式不依赖估计器年龄；
13. 相对速度反馈缺少无人机速度时拒绝输出；
14. reset 清除速度历史；
15. 非法参数、NaN、缺失输入拒绝。

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

## 10. 真实 PX4 SITL 验收

2026-07-23 在 PX4 SITL、Gazebo Harmonic 和理想船舶 GNSS 下完成实测。
无人机从 Gazebo world ENU `[-4, 0, 0.2] m` 的固定地面起飞，避免动态甲板在
解锁前拖动机体并触发 PX4 水平位置漂移预检失败。每轮进入 `TRACK_TARGET` 后丢弃
前 5 秒过渡数据，再统计至少 40 秒。

Ground Truth 只进入 rosbag 离线评测。统计时使用
`VehicleLocalPosition.ref_lat/ref_lon/ref_alt` 将 Gazebo world ENU Ground Truth
转换到 PX4 local NED；不能假设两个坐标系原点重合后直接交换 ENU/NED 分量。

### 10.1 默认预测前馈模式

| 场景 | 稳定统计时长 | 水平位置 RMSE | 相对速度 RMSE | 最大水平误差 | 预测位置 RMSE | 丢标 / GNSS 恢复 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 静止 | 64.7 s | 0.029 m | 0.017 m/s | 0.075 m | 0.028 m | 0 / 0 |
| 匀速 0.2 m/s | 47.1 s | 0.039 m | 0.020 m/s | 0.078 m | 0.031 m | 0 / 0 |
| 匀速 0.4 m/s | 40.9 s | 0.064 m | 0.017 m/s | 0.106 m | 0.044 m | 0 / 0 |
| XY 正弦 | 65.1 s | 0.488 m | 0.513 m/s | 0.739 m | 0.194 m | 0 / 0 |

飞行器实际动力学峰值：

| 场景 | 水平速度 | 水平加速度 | 横滚角绝对值 | 俯仰角绝对值 |
| --- | ---: | ---: | ---: | ---: |
| 静止 | 0.035 m/s | 0.067 m/s² | 0.376° | 0.286° |
| 匀速 0.2 m/s | 0.240 m/s | 0.076 m/s² | 0.390° | 0.432° |
| 匀速 0.4 m/s | 0.428 m/s | 0.068 m/s² | 0.355° | 0.586° |
| XY 正弦 | 1.374 m/s | 1.157 m/s² | 6.143° | 3.588° |

四个场景均满足：

- 正常进入并持续保持 `TRACK_TARGET`；
- `guidance_source=VISION_PREDICTED_FF`；
- 高度目标始终为 `-5.0 m`；
- 飞行器实际 NED z 保持在 `[-5.023, -4.981] m`；
- 未进入下降、Land 或 `DONE`；
- Marker 无丢失，未触发 GNSS 恢复。

正弦场景的水平速度前馈最大值为 `0.828 m/s`，小于
`1.5 m/s` 限制。按 rosbag 接收时间差分得到的最大前馈加速度为
`1.007 m/s²`，与 `1.0 m/s²` 配置限制的差异为时间戳离散误差。

### 10.2 模式对比

| 场景 | 模式 | 水平位置 RMSE | 相对速度 RMSE | 最大水平误差 |
| --- | --- | ---: | ---: | ---: |
| 匀速 0.4 m/s | `RAW_VISUAL_POSITION` | 0.503 m | 0.011 m/s | 0.538 m |
| 匀速 0.4 m/s | `ESTIMATED_POSITION_VELOCITY_FF` | 0.119 m | 0.018 m/s | 0.172 m |
| 匀速 0.4 m/s | `PREDICTED_POSITION_VELOCITY_FF` | 0.064 m | 0.017 m/s | 0.106 m |
| XY 正弦 | `RAW_VISUAL_POSITION` | 0.654 m | 0.591 m/s | 0.933 m |
| XY 正弦 | `ESTIMATED_POSITION_VELOCITY_FF` | 0.459 m | 0.481 m/s | 0.664 m |
| XY 正弦 | `PREDICTED_POSITION_VELOCITY_FF` | 0.488 m | 0.513 m/s | 0.739 m |

结论：

- `0.4 m/s` 匀速场景中，默认预测前馈相对原始视觉将位置 RMSE 降低约
  `87%`，相对估计位置前馈降低约 `46%`；
- XY 正弦场景中，两种前馈模式都优于原始视觉；
- XY 正弦场景的常速度预测比估计位置前馈高约 `6%` RMSE，说明固定
  `0.10 s` 额外预测时域在周期反向运动中存在过预测，进入低高度下降前应继续调参。

### 10.3 验收入口修正

实测中修正了两个只影响仿真与评测的入口问题：

- `scripts/start_sitl.sh --record` 增加 `/simulation/deck/ground_truth`；
- PX4 默认生成位置从移动甲板表面改为固定地面 `[-4, 0, 0.2] m`，避免动态甲板
  在解锁前拖动无人机。

## 11. 阶段结论

P4 代码、纯逻辑测试、消息级数据链和真实 PX4 SITL 验收均已完成。安全高度跟踪
稳定、误差有界、前馈模式相对原始视觉有明确改善，P4 功能门槛通过。

进入 P5 前仍需用户确认是否接受当前正弦场景 `0.488 m` RMSE；如果低高度着陆窗口
要求更小的动态误差，应先调节预测时域或在正弦场景使用估计位置前馈模式。

阶段标签：

```text
baseline-moving-tracking-v0.1
```
