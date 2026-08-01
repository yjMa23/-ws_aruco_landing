# P3 视觉状态估计与短时预测验收记录

## 1. 验收范围

本记录验证 P3 的以下内容：

- 三维常速度 Kalman Filter。
- 视觉 Marker `local_ned` 位置、速度和 6×6 协方差估计。
- 图像采样时间驱动的滤波 `dt`。
- 重复/倒退时间、NaN/Inf、离群点和长时丢失处理。
- 基于观测到达年龄的受限短时运动预测。
- ROS 2 估计与预测调试话题。
- P2D GNSS—视觉接管、安全高度和恢复逻辑回归。

本阶段不验证预测位置控制、速度前馈、着陆窗口、下降或触地。

详细设计见：

```text
docs/plans/P3_VISUAL_STATE_ESTIMATION_PLAN.md
```

---

## 2. 新增实现

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

新增 ROS 输出：

```text
/landing/estimated_deck_odometry  nav_msgs/msg/Odometry
/landing/predicted_deck_pose      geometry_msgs/msg/PoseStamped
```

估计器输入仍来自 P2D 完整变换后的视觉 Marker `local_ned` 位置。控制器和检测器不订阅仿真 Ground Truth。

---

## 3. 状态估计模型

状态：

```text
x = [px, py, pz, vx, vy, vz]^T
```

模型：

```text
p(k+1) = p(k) + v(k) * dt
v(k+1) = v(k)
```

视觉观测：

```text
z = [px, py, pz]^T
```

过程噪声使用离散白噪声加速度模型，观测噪声区分水平与垂直方向。

离群点使用归一化创新平方判断：

```text
NIS = innovation^T * S^-1 * innovation
```

超出配置门限时拒绝观测，但滤波状态时间仍推进到当前采样时刻。

---

## 4. 时间语义

滤波器相邻观测 `dt`：

```text
优先使用 /aruco/pose.header.stamp
```

零时间戳时退化使用回调到达时间。

预测时域：

```text
observation_receipt_age
+
additional_prediction_horizon
```

并限制为：

```text
[0, max_prediction_horizon]
```

本阶段没有直接混减图像采样时间与控制器时钟，避免不同时间域造成错误年龄。

---

## 5. 单元测试

执行：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash

colcon build --symlink-install \
  --packages-select aruco_precision_landing_cpp \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
colcon test --packages-select aruco_precision_landing_cpp
colcon test-result --verbose
```

结果：

```text
77 tests
0 errors
0 failures
0 skipped
```

其中 P3 新增：

```text
target_state_estimator_test  13 tests
motion_predictor_test         7 tests
```

估计器覆盖：

- 首帧初始化。
- 静止速度收敛。
- 三维匀速速度收敛。
- 不规则采样间隔。
- 大于单次传播步长、但未达到重初始化阈值的间隔。
- 重复时间戳。
- 时间倒退。
- NaN、Inf。
- 离群点拒绝和后续恢复。
- 连续离群点不会触发错误重初始化。
- 长时无观测后重新初始化和速度清零。
- 协方差有限、对称和非负主对角线。
- reset 和非法参数。

预测器覆盖：

- 观测年龄与固定补偿叠加。
- 零预测时域。
- 零速度。
- 最大时域限幅。
- 输入不被修改。
- 非法估计和非法年龄。
- 非法参数。

---

## 6. 静止消息级验收

使用合成 PX4、船舶 GNSS 和 ArUco 消息验证 ROS 接口与状态机。

输入：

```text
UAV local NED position = [0.0, 0.0, -5.0]
Marker camera optical  = [0.0, 0.0, 5.3]
Marker local NED       = [0.0, 0.0, 0.2]
```

状态到达：

```text
TRACK_TARGET
```

实测估计速度：

```text
vx = 0.0 m/s
vy = 0.0 m/s
vz = 0.0 m/s
```

实测估计位置：

```text
[0.0, 0.0, 0.2] m
```

实测预测位置：

```text
[0.0, 0.0, 0.2] m
```

控制目标高度：

```text
target_z = -5.0 m
```

说明静止输入下估计速度为零，预测位置未产生漂移，并且 P3 没有改变 P2D 安全高度控制。

---

## 7. 匀速消息级验收

构造带明确图像采样时间的视觉序列：

```text
camera optical x 每 0.1 s 增加 0.04 m
```

名义速度为：

```text
body right / local NED East = 0.4 m/s
```

状态保持：

```text
TRACK_TARGET
```

实测估计速度：

```text
vx = -8.89e-17 m/s
vy = 0.4002329 m/s
vz = 0.0 m/s
```

East 方向速度误差：

```text
约 0.00023 m/s
```

最后估计位置：

```text
x = approximately 0.0 m
y = 0.76009 m
z = 0.2 m
```

短时预测位置：

```text
x = approximately 0.0 m
y = 0.96021 m
z = 0.2 m
```

预测位置沿 East 正方向领先于估计位置，符合常速度外推。控制目标 z 仍为 `-5.0 m`。

---

## 8. 控制隔离检查

P3 只发布：

```text
/landing/estimated_deck_odometry
/landing/predicted_deck_pose
```

`TRACK_TARGET` 仍使用 P2D 原始视觉位置和原目标限幅。预测位置没有进入：

```text
/fmu/in/trajectory_setpoint
```

因此当前 P3 结果不能被理解为已经完成速度前馈或预测跟踪控制。

---

## 9. Ground Truth 隔离

执行：

```bash
grep -R "/simulation/deck/ground_truth" \
  src/aruco_precision_landing_cpp src/aruco_detector
```

实测无输出。

Ground Truth 仍只允许进入：

- 仿真传感器节点；
- 后续离线评测器；
- rosbag 记录和 RMSE 统计。

---

## 10. 尚未完成的真实仿真指标

本记录不声明以下项目已经通过：

- 真实 PX4 动力学和真实相机图像下的速度 RMSE。
- `0.4 m/s` 移动甲板完整闭环估计误差曲线。
- XY 正弦甲板速度和位置 RMSE。
- 图像延迟下的估计偏差。
- 图像采样时刻的 PX4 位姿插值。
- 预测位置用于控制后的跟踪性能提升。

这些项目需要在实际仿真运行时录制 Marker、估计、预测和 Ground Truth，再进行离线统计。

---

## 11. 阶段结论

P3 软件实现与消息级验收通过：

- 常速度 Kalman Filter 正常。
- 静止速度收敛到零。
- 匀速速度估计方向和数值正确。
- 不规则时间、离群点、重初始化和协方差行为有测试。
- 短时预测方向正确并受时域上限限制。
- P2D 状态机、安全高度和 Ground Truth 隔离未退化。
- 预测结果尚未进入控制。

下一阶段：

```text
P4：使用预测位置和甲板速度前馈改善移动甲板水平跟踪
```
