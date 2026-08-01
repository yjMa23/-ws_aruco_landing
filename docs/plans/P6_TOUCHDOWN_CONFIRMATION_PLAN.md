# P6A 多源触地候选与确认执行计划

## 1. 阶段目标

在 P5C 已完成低高度垂直状态估计和 `0.50 m` 安全高度标定的基础上，先实现一个与 ROS、PX4 和仿真器解耦的多源触地检测器，用于判断：

```text
证据不足
→ 空中
→ 触地候选
→ 触地确认
```

P6A 只建立触地判据、持续时间、迟滞、状态锁存和调试链路，不执行最终下降，不发送 `NAV_LAND`，不自动 Disarm。

---

## 2. 明确不做

P6A 不实现：

- `0.50 m` 以下最终下降；
- 自动 Land；
- 自动 Disarm；
- 触地后关闭 Offboard；
- 仅凭视觉相对高度确认触地；
- Ground Truth 进入控制器；
- 预先已知甲板运动相位或轨迹；
- 修改 P4.7 水平跟踪控制律；
- 修改 P5B/P5C 的默认下降高度与恢复策略；
- MPC 或强化学习。

若后续需要真实接触正向验收，必须先另存 P6B 最终下降计划，再开始修改最终下降代码。

---

## 3. 数据来源与边界

### 3.1 控制器允许使用

PX4：

```text
/fmu/out/vehicle_land_detected
```

消息：

```text
px4_msgs/msg/VehicleLandDetected
```

使用字段：

- `ground_contact`
- `maybe_landed`
- `landed`
- `at_rest`
- `has_low_throttle`
- `vertical_movement`
- `horizontal_movement`
- `rotational_movement`
- `close_to_ground_or_skipped_check`
- `freefall`

现有视觉与状态估计：

- 相对高度；
- 相对垂直速度；
- 无人机 NED 垂直速度；
- 视觉数据年龄和有效性；
- 当前状态是否处于允许触地判定的低高度阶段。

### 3.2 禁止使用

- `/simulation/deck/ground_truth`；
- Gazebo 碰撞状态；
- 仿真模型内部接触标志；
- 评测脚本计算结果；
- 已知甲板真实 z 或 vz。

Ground Truth 仍只用于离线评测误报和安全边界。

---

## 4. Task 1：纯 C++ TouchdownDetector

新增：

```text
src/aruco_precision_landing_cpp/include/aruco_precision_landing_cpp/touchdown_detector.hpp
src/aruco_precision_landing_cpp/src/touchdown_detector.cpp
src/aruco_precision_landing_cpp/test/touchdown_detector_test.cpp
```

### 4.1 输入

```text
sample_time_s
state_allows_touchdown_detection
px4_land_status_valid
px4_land_status_age_s
freefall
ground_contact
maybe_landed
landed
at_rest
has_low_throttle
vertical_movement
horizontal_movement
rotational_movement
close_to_ground
visual_height_valid
visual_height_age_s
relative_height_m
relative_vertical_velocity_mps
uav_vertical_velocity_mps
```

### 4.2 输出

```text
status
evidence_mask
candidate_duration_s
confirmed_latched
```

状态枚举：

```text
kInsufficientEvidence
kAirborne
kCandidate
kConfirmed
kRejectedUnsafe
```

### 4.3 证据分组

触地确认必须至少包含两个独立证据族，不允许单一视觉高度直接确认。

PX4 接触证据族：

```text
ground_contact || maybe_landed || landed
```

低运动证据族：

```text
abs(relative_vertical_velocity) <= threshold
abs(uav_vertical_velocity) <= threshold
!vertical_movement
```

稳定/静止证据族：

```text
at_rest
或 landed && !horizontal_movement && !rotational_movement
```

视觉低高度证据族，只能作为辅助：

```text
relative_height <= low_height_threshold
```

强触地路径：

```text
landed && at_rest
```

普通触地路径：

```text
PX4 接触证据
&& 低运动证据
&& 视觉低高度证据
```

### 4.4 安全拒绝

以下任一条件成立时不得积累候选时间：

- `freefall=true`；
- 当前状态不允许触地检测；
- PX4 land 状态超时或无效；
- 时间戳重复、倒退或非有限；
- 高度、速度输入为 NaN/Inf；
- 视觉无效或超时，且没有 `landed && at_rest` 强证据；
- 仍存在明显垂直、水平或旋转运动；
- 相对高度明显高于候选上限。

### 4.5 持续时间和迟滞

- 候选证据连续满足 `candidate_required_duration_s` 后才确认；
- 接触标志短暂抖动不得立即确认；
- 候选期间证据失效时清零候选持续时间；
- `kConfirmed` 一旦成立即锁存，只有显式 `reset()` 才清除；
- 时间回退清空候选，不保留旧证据。

---

## 5. Task 2：单元测试

至少覆盖：

1. 视觉高度单独满足不能确认；
2. PX4 `landed && at_rest` 连续满足后确认；
3. `ground_contact + 低高度 + 低相对速度` 连续满足后确认；
4. 接触标志单帧抖动只能成为候选，不能确认；
5. 高垂直速度清零候选；
6. 水平或旋转运动拒绝普通触地路径；
7. `freefall` 进入不安全拒绝；
8. PX4 状态超时不能确认；
9. 视觉超时且无强触地证据时不能确认；
10. 高相对高度不能确认；
11. 重复或倒退时间拒绝；
12. 确认后状态锁存；
13. reset 清除锁存和候选历史；
14. 非法参数拒绝构造。

---

## 6. Task 3：节点并行接入

P6A 只并行接入，不改变状态机和控制目标。

新增订阅：

```text
/fmu/out/vehicle_land_detected
```

新增调试话题：

```text
/landing/touchdown_status
/landing/touchdown_evidence
/landing/touchdown_candidate_duration
/landing/touchdown_confirmed
```

要求：

- 默认启用检测和调试发布；
- 只有低高度测试或后续最终下降状态允许积累候选；
- 当前 `TEST_HEIGHT_HOLD` 的 `0.50 m` 负向验收不应确认触地；
- `WAIT_LANDING_WINDOW`、高空跟踪和恢复爬升状态不得积累触地候选；
- PX4 状态或视觉输入超时后立即停止积累；
- `kConfirmed` 不触发 Land、Disarm 或状态跳转。

---

## 7. Task 4：启动与 rosbag

启动脚本新增 P6A 调试话题记录。

Launch 参数至少包括：

```text
touchdown_detector.enabled
touchdown_detector.px4_status_timeout_s
touchdown_detector.visual_timeout_s
touchdown_detector.low_height_enter_m
touchdown_detector.low_height_exit_m
touchdown_detector.max_relative_vertical_speed_mps
touchdown_detector.max_uav_vertical_speed_mps
touchdown_detector.candidate_required_duration_s
```

默认参数只允许在低高度状态观察，不改变 P5C 控制。

---

## 8. Task 5：SITL 负向验收

首先复用现有 P5C 场景：

```text
静止甲板 0.50 m 测试高度
升沉甲板 0.70 m 测试高度
5 m 安全高度跟踪
恢复爬升
```

预期：

- 全部不出现 `kConfirmed`；
- 视觉低高度本身不能造成误报；
- `ground_contact` 或 `maybe_landed` 短暂噪声不能确认；
- 高空和恢复阶段候选时间始终为零；
- 不发送 Land/Disarm；
- P4.7 水平跟踪和 P5C 垂直控制指标不回归。

正向真实接触测试不属于 P6A，必须在 P6B 计划保存后执行。

---

## 9. P6A 验收标准

- 纯 C++ 触地检测器测试全部通过；
- 视觉高度单独满足的误确认次数为 0；
- P5B/P5C 现有 bags 离线回放或 SITL 负向验收确认次数为 0；
- PX4 land 状态超时、视觉超时、时间异常和高运动均不能确认；
- `kConfirmed` 锁存行为正确；
- 节点并行接入后不改变任何 TrajectorySetpoint；
- 不发送 `NAV_LAND` 或 Disarm；
- 不进入旧下降状态；
- 全工作区构建和测试通过；
- Ground Truth 不进入控制器。

---

## 10. 执行顺序

```text
保存本计划
→ 实现纯 C++ TouchdownDetector
→ 单元测试
→ 全工作区回归
→ 节点并行接入
→ 新增调试话题和 rosbag 记录
→ 静止/升沉/恢复负向验收
→ 冻结 P6A 验收文档
→ 另存 P6B 最终下降计划后才允许继续真实接触测试
```

## 11. 当前状态

- 本计划已保存并执行完成。
- 纯 C++ `TouchdownDetector`、ROS 并行接入、调试话题和离线评测脚本已完成。
- 静止 `0.50 m`、升沉 `0.70 m` 和恢复爬升三类负向 PX4 SITL 验收均通过。
- 三轮候选样本数、最大候选持续时间和确认次数均为 0；无 `NAV_LAND` 或 Disarm。
- 全工作区 `182 tests`，全部通过。
- 详细结果见 `docs/validation/P6_TOUCHDOWN_CONFIRMATION_VALIDATION.md`。
- P6B 真实接触正向试验尚未开始，必须先保存独立计划。
