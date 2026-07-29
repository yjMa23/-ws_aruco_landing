# P5A 甲板升沉/倾斜仿真与规则式着陆窗口执行计划

## 1. 阶段目标

在保持无人机 `5 m` 安全高度、不发布任何下降目标的前提下，完成：

1. 甲板升沉、横摇、纵摇和水平运动组合仿真；
2. 甲板三维位置、姿态、线速度和角速度 Ground Truth；
3. 独立纯 C++ 规则式着陆窗口模块；
4. `TRACK_TARGET → WAIT_LANDING_WINDOW` 主路径接入；
5. 着陆窗口开启、关闭、迟滞和持续时间的真实 PX4 SITL 验收。

本阶段完成后，系统仍保持安全高度，不执行下降。P5B 才实现相对高度分阶段下降。

## 2. 明确不做

P5A 不实现：

- 垂直下降；
- 触地检测；
- `NAV_LAND` 或自动 Disarm；
- 低高度视觉丢失恢复；
- MPC 或强化学习；
- Ground Truth 进入降落控制器；
- 复杂海浪水动力学。

## 3. Task 1：扩展解析甲板运动模型

修改：

```text
src/moving_deck_sim/include/moving_deck_sim/motion_profile.hpp
src/moving_deck_sim/src/motion_profile.cpp
src/moving_deck_sim/test/motion_profile_test.cpp
```

新增场景：

```text
S3_HEAVE
S4_ROLL_PITCH
S5_COMBINED
```

扩展参数：

```text
amplitude_z_m
period_z_s
initial_rpy_rad[3]
amplitude_rpy_rad[3]
period_rpy_s[3]
```

扩展输出：

```text
position_enu[3]
orientation_rpy_enu[3]
linear_velocity_enu[3]
angular_velocity_body[3]
```

解析公式：

```text
z(t) = z0 + Az sin(wz t)
vz(t) = Az wz cos(wz t)

roll(t) = roll0 + Ar sin(wr t)
roll_rate(t) = Ar wr cos(wr t)

pitch(t) = pitch0 + Ap sin(wp t)
pitch_rate(t) = Ap wp cos(wp t)
```

第一版不使用 yaw 摆动，`yaw amplitude = 0`。

要求：

- `S3_HEAVE` 只启用 z 升沉；
- `S4_ROLL_PITCH` 只启用 roll/pitch；
- `S5_COMBINED` 启用 XY 正弦、z 升沉和 roll/pitch；
- 旧 S0/S1/S2 数值行为保持不变；
- reset 后位置、姿态、线速度和角速度确定性复现；
- 参数 NaN、Inf、非正周期和不合理姿态幅值拒绝。

## 4. Task 2：Gazebo 控制与 Ground Truth

修改：

```text
src/moving_deck_sim/src/moving_deck_controller.cpp
src/moving_deck_sim/models/moving_deck/model.sdf
```

要求：

- `VelocityControl` 同时接收线速度和角速度；
- reset 时通过 `/world/<world>/set_pose` 恢复初始位置和初始姿态；
- Ground Truth 保留 Gazebo 实际姿态、线速度和角速度；
- reset 后短时间内用解析速度覆盖旧差分污染，同时包含角速度；
- 甲板模型原点继续位于着陆平面中心；
- 不将 Ground Truth 接入控制节点。

## 5. Task 3：新增可复现场景配置

新增：

```text
src/moving_deck_sim/config/heave.yaml
src/moving_deck_sim/config/roll_pitch.yaml
src/moving_deck_sim/config/combined.yaml
```

第一版建议参数：

```yaml
# S3_HEAVE
amplitude_z_m: 0.30
period_z_s: 8.0

# S4_ROLL_PITCH
roll_amplitude_deg: 5.0
roll_period_s: 8.0
pitch_amplitude_deg: 3.0
pitch_period_s: 6.0

# S5_COMBINED
amplitude_xy: [1.0, 0.5]
period_xy: [10.0, 6.0]
amplitude_z_m: 0.30
period_z_s: 8.0
roll_amplitude_deg: 5.0
roll_period_s: 8.0
pitch_amplitude_deg: 3.0
pitch_period_s: 6.0
```

`start_sitl.sh` 新增：

```text
--scenario heave
--scenario rollpitch
--scenario combined
```

## 6. Task 4：规则式着陆窗口纯逻辑模块

新增：

```text
src/aruco_precision_landing_cpp/include/aruco_precision_landing_cpp/landing_window.hpp
src/aruco_precision_landing_cpp/src/landing_window.cpp
src/aruco_precision_landing_cpp/test/landing_window_test.cpp
```

输入：

```text
visual_fresh
estimate_valid
prediction_valid
horizontal_error_m
horizontal_relative_speed_mps
deck_roll_rad
deck_pitch_rad
relative_height_m
now_s
```

输出：

```text
window_open
conditions_currently_satisfied
satisfied_duration_s
reject_reason bitmask
```

进入阈值：

```yaml
landing_window.enter_horizontal_error_m: 0.15
landing_window.enter_relative_speed_mps: 0.15
landing_window.enter_max_tilt_deg: 5.0
landing_window.max_visual_age_s: 0.20
landing_window.minimum_relative_height_m: 0.5
landing_window.maximum_relative_height_m: 6.0
landing_window.required_duration_s: 1.0
```

退出阈值：

```yaml
landing_window.exit_horizontal_error_m: 0.25
landing_window.exit_relative_speed_mps: 0.25
landing_window.exit_max_tilt_deg: 8.0
```

要求：

- `enter < exit`，形成迟滞；
- 任一必要输入无效时窗口关闭；
- 所有进入条件连续满足 `required_duration_s` 后才打开；
- 已打开后仅在退出阈值或硬失效条件触发时关闭；
- 时间回退或 reset 清除持续时间；
- 模块不依赖 ROS、PX4 或 Ground Truth。

## 7. Task 5：甲板姿态估计输入

P5A 第一版控制器只能使用视觉 Marker 姿态，不使用 Ground Truth。

要求：

- 从时间对齐后的 `marker_pose_ned.rotation` 提取甲板 roll/pitch；
- 对四元数进行有效性检查和角度归一化；
- 使用一阶低通或有限窗口平均抑制 PnP 姿态抖动；
- 发布 `/landing/estimated_deck_attitude` 调试话题；
- Ground Truth 姿态只在 rosbag 离线比较。

## 8. Task 6：状态机接入但不下降

新增状态：

```text
WAIT_LANDING_WINDOW
```

主路径：

```text
TRACK_TARGET
→ WAIT_LANDING_WINDOW
```

行为：

- 两个状态都继续使用当前 P4.7 水平跟踪；
- z 始终保持 `-rendezvous_altitude_m`；
- 窗口打开只发布状态和调试结果，不进入下降；
- 窗口关闭继续等待；
- 视觉长时丢失仍按现有路径恢复 GNSS；
- 不接回旧 `DESCEND_WITH_TRACKING`。

新增调试话题：

```text
/landing/window_open
/landing/window_reject_reasons
/landing/window_satisfied_duration
/landing/estimated_deck_attitude
```

## 9. 测试顺序

### 9.1 纯逻辑测试

- S3/S4/S5 解析位置、姿态和速度；
- reset 确定性；
- 着陆窗口进入、退出、迟滞、持续时间；
- 视觉失效、估计失效、倾角超限、速度超限；
- 时间回退和 reset；
- 旧 P4.7 控制测试全部回归。

### 9.2 Gazebo 消息级验证

- S3 z 和 vz 符号正确；
- S4 roll/pitch 和角速度正确；
- S5 全部维度连续；
- Ground Truth 与解析轨迹一致；
- reset 后相位归零。

### 9.3 PX4 SITL 安全高度验收

依次运行：

```text
static
constant 0.4 m/s
heave
rollpitch
combined
```

共同要求：

- 始终保持 5 m 安全高度；
- 不进入下降或 Land；
- 窗口仅在所有条件持续满足时打开；
- 倾角过大时窗口关闭；
- 无丢标、GNSS 恢复和控制异常。

## 10. 阶段完成标准

P5A 完成必须同时满足：

- S3/S4/S5 可重复运行；
- 甲板姿态估计只使用视觉输入；
- 着陆窗口纯逻辑模块测试通过；
- 状态机只进入 `WAIT_LANDING_WINDOW`，不下降；
- 真实 SITL 中窗口行为与离线 Ground Truth 条件一致；
- 全工作区测试全部通过。

## 11. 当前执行状态（2026-07-25）

- Task 1：已完成，S3/S4/S5 解析轨迹与测试通过。
- Task 2：已完成，Gazebo 三维线速度、角速度、姿态 reset 和完整 Ground Truth 已验证。
- Task 3：已完成，`heave`、`rollpitch`、`combined` 场景已接入一键脚本。
- Task 4：已完成，规则式着陆窗口纯逻辑模块与测试通过。
- Task 5：已完成，采用 Marker 向上法向量而非直接欧拉角估计甲板倾角。
- Task 6：已完成，主路径进入 `WAIT_LANDING_WINDOW`，高度仍固定为 `5 m`。
- 静止、0.4 m/s 匀速、升沉、横摇/纵摇和组合场景 PX4 SITL 验收已完成。
- 完整工作区 `142 tests`，全部通过。

详细验收记录：

```text
docs/P5A_DECK_DYNAMICS_AND_LANDING_WINDOW_VALIDATION.md
```
