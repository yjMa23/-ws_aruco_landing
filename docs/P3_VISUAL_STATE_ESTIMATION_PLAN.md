# P3 视觉状态估计与短时运动预测详细实施计划

## 1. 文档状态

- 阶段：`P3`
- 前置阶段：`P2D` 已完成并通过用户手工检测链验收
- 确认日期：2026-07-22
- 实施目标：在不改变当前飞行控制行为的前提下，增加甲板视觉位置、速度、协方差和短时预测输出
- 本阶段建议标签：`baseline-visual-estimator-v0.1`

本文件是 P3 的执行依据。实现完成后，`AGENTS.md`、总体计划和包 README 应同步到 P3 完成状态。

---

## 2. 阶段边界

### 2.1 本阶段必须完成

1. 实现不依赖 ROS 的三维常速度 Kalman Filter。
2. 使用 Marker 的 `local_ned` 视觉位置估计：
   - 位置 `[x, y, z]`；
   - 速度 `[vx, vy, vz]`；
   - 6×6 状态协方差。
3. 处理：
   - 首帧初始化；
   - 不规则采样间隔；
   - 重复时间戳；
   - 时间倒退；
   - NaN、Inf；
   - 大残差离群点；
   - 短时丢帧；
   - 长时丢帧后的重新初始化。
4. 实现独立短时运动预测器。
5. 发布：
   - `/landing/estimated_deck_odometry`；
   - `/landing/predicted_deck_pose`。
6. 为估计器、预测器和控制器消息链增加测试。
7. 保持 P2D 原有 GNSS—视觉接管和恢复逻辑可用。

### 2.2 本阶段明确不做

1. 不将预测位置用于 PX4 setpoint。
2. 不加入甲板速度前馈。
3. 不修改 `TRACK_TARGET` 的控制律。
4. 不实现着陆窗口。
5. 不下降。
6. 不触地检测。
7. 不使用 Gazebo Ground Truth 作为估计器输入。
8. 不实现 EKF、UKF、粒子滤波或学习型估计器。
9. 第一版不实现图像采样时刻的 PX4 位姿历史插值；该项作为 P3 可选增强或 P4 前置优化。

这样可以保证：P3 只验证“状态估计和预测是否正确”，P4 再验证“如何使用这些估计量改善动态跟踪”。

---

## 3. 当前输入与时间语义

### 3.1 视觉测量

估计器输入来自 P2D 已完成的完整变换：

```text
T_local_ned_marker
=
T_local_ned_body_frd
*
T_body_frd_camera_optical
*
T_camera_optical_marker
```

输入位置为：

```text
p_marker_ned = [North, East, Down]
```

估计器不直接订阅 `/aruco/pose`，而是在控制节点完成合法性、坐标系、外参、GNSS 一致性和测量跳变检查后接收 `local_ned` 位置。

### 3.2 采样时间

滤波更新 `dt` 优先使用：

```text
/aruco/pose.header.stamp
```

规则：

1. 时间戳非零：转换为 `sample_time_s`。
2. 时间戳为零：退化使用控制器回调到达时间。
3. 重复时间戳：拒绝。
4. 时间倒退：拒绝。
5. 与上一帧间隔超过重初始化阈值：用当前测量重新初始化，不继承旧速度。

### 3.3 控制时刻预测时间

当前 Gazebo 图像时间、ROS 节点时钟和 PX4 内部时间尚未建立严格统一映射，因此第一版禁止直接计算：

```text
controller_now - image_header_stamp
```

第一版预测时域采用：

```text
prediction_horizon
=
last_measurement_receipt_age
+
additional_prediction_horizon
```

其中：

- `last_measurement_receipt_age`：当前控制器时间减最后一次有效观测到达时间；
- `additional_prediction_horizon`：可配置的固定视觉/控制链延迟补偿；
- 最终时域限制在 `[0, max_prediction_horizon]`。

图像时间戳只用于滤波器内部相邻观测 `dt`，回调时间只用于控制周期的新鲜度和预测时域。两种时间不得混合相减。

---

## 4. 状态估计模型

### 4.1 状态

```text
x = [px, py, pz, vx, vy, vz]^T
```

坐标系：PX4 `local_ned`。

单位：

```text
p: m
v: m/s
```

### 4.2 常速度状态转移

```text
p(k+1) = p(k) + v(k) * dt
v(k+1) = v(k)
```

矩阵形式：

```text
x(k+1) = F(dt) x(k) + w
```

```text
F = [I, dt*I
     0,    I]
```

### 4.3 观测模型

视觉只观测位置：

```text
z = [px, py, pz]^T
```

```text
z(k) = H x(k) + v
```

```text
H = [I, 0]
```

### 4.4 过程噪声

第一版使用每轴独立的离散白噪声加速度模型：

```text
G(dt) = [0.5 * dt^2
         dt]
```

每轴过程协方差：

```text
Q_axis = sigma_a^2 * G * G^T
```

三轴采用相同默认过程加速度标准差，后续可根据水平/垂直运动特性拆分。

### 4.5 观测噪声

第一版允许水平和垂直位置噪声分开配置：

```text
R = diag(
  sigma_xy^2,
  sigma_xy^2,
  sigma_z^2)
```

### 4.6 初始化

首帧有效位置：

```text
p = measurement
v = 0
```

初始协方差：

```text
P_position = initial_position_std^2
P_velocity = initial_velocity_std^2
```

长时丢失后的首帧也采用相同方式重新初始化，避免将过期速度带入新目标轨迹。

### 4.7 离群点门限

更新前计算创新：

```text
y = z - H*x_pred
S = H*P_pred*H^T + R
```

归一化创新平方：

```text
NIS = y^T * S^-1 * y
```

当：

```text
NIS > innovation_gate^2
```

拒绝本次测量，但保留预测后的状态和时间，避免一个离群点导致时间轴停止。

---

## 5. 纯 C++ 接口设计

### 5.1 文件结构

```text
src/aruco_precision_landing_cpp/
├── include/aruco_precision_landing_cpp/
│   ├── target_state_estimator.hpp
│   └── motion_predictor.hpp
├── src/
│   ├── target_state_estimator.cpp
│   └── motion_predictor.cpp
└── test/
    ├── target_state_estimator_test.cpp
    └── motion_predictor_test.cpp
```

### 5.2 `TargetStateEstimatorParameters`

建议参数：

```text
process_acceleration_std_mps2
measurement_horizontal_std_m
measurement_vertical_std_m
initial_position_std_m
initial_velocity_std_mps
minimum_sample_dt_s
maximum_sample_dt_s
reinitialize_gap_s
innovation_gate_mahalanobis
```

### 5.3 更新状态枚举

```text
kInitialized
kUpdated
kReinitialized
kRejectedInvalidInput
kRejectedNonMonotonicTime
kRejectedOutlier
```

调用方必须能够区分“测量被拒绝”和“测量触发重初始化”。

### 5.4 估计输出

```text
TargetStateEstimate
├── position_ned
├── velocity_ned
├── covariance 6x6
└── sample_time_s
```

### 5.5 预测器输入输出

输入：

```text
TargetStateEstimate
observation_receipt_age_s
```

参数：

```text
additional_prediction_horizon_s
max_prediction_horizon_s
```

输出：

```text
MotionPrediction
├── position_ned
├── velocity_ned
└── horizon_s
```

预测公式：

```text
p_pred = p_est + v_est * horizon
v_pred = v_est
```

---

## 6. ROS 控制器接入

### 6.1 更新位置

控制器在成功完成以下检查后更新估计器：

1. ArUco frame 正确。
2. 时间戳未重复、未倒退。
3. 完整相机到 NED 变换成功。
4. 输入有限。
5. P2D 视觉跳变门限通过。
6. 接管前 GNSS—视觉一致性通过。

估计器拒绝离群点时：

- 不更新最后有效观测到达时间；
- 不刷新对外发布的新鲜度；
- 使用节流日志说明拒绝原因。

### 6.2 状态生命周期

估计器在以下阶段允许接收视觉测量：

```text
ACQUIRE_ARUCO
VISUAL_HANDOVER
TRACK_TARGET
```

进入以下状态时清空估计器：

```text
WAIT_DECK_GNSS
RECOVER_TO_GNSS
```

进入 `ACQUIRE_ARUCO` 时从空状态重新建立视觉轨迹。

### 6.3 输出话题

#### `/landing/estimated_deck_odometry`

类型：

```text
nav_msgs/msg/Odometry
```

语义：

```text
header.frame_id = local_ned
child_frame_id = estimated_deck
pose.position = filter position
pose.orientation = identity
pose covariance = state position covariance
linear velocity = filter velocity
linear velocity covariance = state velocity covariance
```

本阶段不估计甲板姿态角速度，因此相关协方差使用大值表示未估计。

#### `/landing/predicted_deck_pose`

类型：

```text
geometry_msgs/msg/PoseStamped
```

语义：

```text
header.frame_id = local_ned
position = short-horizon predicted position
orientation = identity
```

### 6.4 控制隔离

P3 中以下控制目标保持不变：

```text
TRACK_TARGET 仍使用 P2D 原始视觉位置目标
```

预测位置只发布调试，不进入：

```text
/fmu/in/trajectory_setpoint
```

P4 才允许显式切换到预测目标和速度前馈。

---

## 7. 初始参数建议

```yaml
target_state_estimator:
  process_acceleration_std_mps2: 1.0
  measurement_horizontal_std_m: 0.08
  measurement_vertical_std_m: 0.12
  initial_position_std_m: 0.20
  initial_velocity_std_mps: 1.0
  minimum_sample_dt_s: 0.001
  maximum_sample_dt_s: 0.50
  reinitialize_gap_s: 2.0
  innovation_gate_mahalanobis: 5.0

motion_predictor:
  additional_prediction_horizon_s: 0.10
  max_prediction_horizon_s: 0.50

estimator_output_timeout_s: 2.0
estimated_deck_child_frame_id: estimated_deck
```

参数含义：

- `process_acceleration_std_mps2` 越大，速度适应运动变化越快，但输出更抖。
- `measurement_*_std_m` 越大，视觉测量权重越低。
- `innovation_gate_mahalanobis` 越小，离群点拒绝更严格。
- `additional_prediction_horizon_s` 用于补偿固定链路延迟。
- `max_prediction_horizon_s` 防止长时丢帧时无限外推。

---

## 8. 单元测试计划

### 8.1 估计器

必须覆盖：

1. 首帧位置初始化、速度为零。
2. 静止输入下速度收敛到零。
3. 匀速输入下速度收敛到真值。
4. 不规则 `dt` 仍能正确估计。
5. 重复时间戳拒绝。
6. 时间倒退拒绝。
7. NaN、Inf 拒绝。
8. 大残差离群点拒绝。
9. 离群点后后续正常测量仍可恢复。
10. 长时间隔触发重新初始化。
11. 重初始化后速度清零。
12. 协方差保持有限、对称，主对角线非负。
13. 非法参数拒绝构造。
14. `reset()` 清空状态。

### 8.2 预测器

必须覆盖：

1. 零速度位置不变。
2. 匀速位置按时域外推。
3. 预测时域等于观测年龄加附加补偿。
4. 时域超过上限时正确限幅。
5. NaN、Inf 和负年龄拒绝。
6. 未初始化或非法估计拒绝。
7. 输入估计不被修改。
8. 非法参数拒绝构造。

---

## 9. 消息级验收计划

使用合成 PX4、GNSS 和 ArUco 消息，不依赖真实飞行动力学。

### 9.1 静止 Marker

输入：

```text
marker_ned = constant
```

期望：

```text
estimated velocity → 0
predicted position ≈ estimated position
```

建议验收：

```text
|vx|, |vy| < 0.05 m/s
```

### 9.2 匀速 Marker

输入：

```text
East velocity = 0.4 m/s
```

期望：

```text
estimated vy → 0.4 m/s
predicted y > estimated y
```

第一版软件验收建议：

```text
steady-state velocity error < 0.08 m/s
```

### 9.3 短时丢帧

停止视觉时间小于：

```text
visual_loss_long_timeout_s
```

期望：

- 估计状态不突跳；
- 预测位置仍可发布；
- 预测时域不超过配置上限；
- P2D 控制状态按原逻辑保持或恢复。

### 9.4 长时丢帧与重捕获

长时丢失后恢复视觉：

- 估计器重新初始化；
- 不继承旧速度；
- 无几米级预测跳变。

### 9.5 离群点

单帧注入超过正常轨迹的大跳变：

- 更新状态为 `kRejectedOutlier`；
- 估计位置不跳到离群点；
- 后续正常观测继续更新。

---

## 10. 仿真验收建议

用户实际运行时按顺序验证：

1. 静止甲板 + 理想 GNSS。
2. `0.4 m/s` 匀速甲板 + 理想 GNSS。
3. XY 正弦甲板。
4. 含噪、延迟 GNSS 只用于确认 P2D 接管未退化，P3 估计器输入仍是视觉。

记录：

```text
/landing/marker_pose_ned
/landing/estimated_deck_odometry
/landing/predicted_deck_pose
/landing/state
/landing/guidance_source
/simulation/deck/ground_truth  # 仅录包/离线评测，不进入控制器
```

P3 仿真通过的最低标准：

- 静止场景速度估计接近零。
- 匀速场景速度方向正确且误差有界。
- 正弦场景估计连续、不发散。
- 单帧视觉跳变不会污染估计状态。
- 短时丢帧预测连续。
- 长时丢帧后能够重新初始化。
- P2D 状态机和安全高度行为不退化。
- 控制器无 Ground Truth 订阅。

---

## 11. 实施顺序

### P3-0：计划冻结

- 新增本文件。
- 确认时间域、状态、参数、输出和阶段边界。

### P3A：纯估计器

- 新增 `target_state_estimator`。
- 完成 Kalman predict/update、NIS 门限、时间异常和重初始化。
- 完成估计器 GTest。

### P3B：纯预测器

- 新增 `motion_predictor`。
- 完成受限短时外推。
- 完成预测器 GTest。

### P3C：ROS 接入

- 添加 `nav_msgs` 依赖。
- 从 ArUco 接收链更新估计器。
- 增加参数。
- 发布估计里程计和预测位姿。
- 保持 P2D 控制律不变。

### P3D：回归与消息级验收

- 全工作区构建和测试。
- 合成静止、匀速、丢帧、离群点测试。
- Ground Truth 隔离检查。
- 验收文档。

### P3E：阶段提交

建议提交：

```text
feat: add visual target state estimation and prediction
```

建议标签：

```text
baseline-visual-estimator-v0.1
```

---

## 12. 进入 P4 的门槛

只有同时满足以下条件才进入 P4：

1. 全部单元测试通过。
2. 静止速度估计接近零。
3. 匀速速度估计方向和数量级正确。
4. 预测时域和限幅正确。
5. 短时丢帧估计连续。
6. 长时丢帧后重新初始化。
7. 离群点不会污染状态。
8. P2D 接管和恢复未退化。
9. P3 输出尚未进入 PX4 setpoint。
10. 控制器仍无 Ground Truth 依赖。

P4 再实现：

```text
预测位置目标
+
甲板速度前馈
+
水平位置/相对速度反馈
```
