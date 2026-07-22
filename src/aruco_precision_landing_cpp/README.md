# aruco_precision_landing_cpp

`aruco_precision_landing_cpp` 是 ROS 2 Humble 下的 PX4 Offboard 控制包。当前实现阶段为：

```text
P3：船舶 GNSS 会合、GNSS—视觉接管、视觉状态估计和短时运动预测
```

当前主流程：

```text
PX4 状态有效
→ Offboard 预发布、解锁和起飞
→ 等待稳定船舶 GNSS
→ 飞向移动甲板 GNSS 上方
→ 围绕实时 GNSS 中心搜索 ArUco
→ 完整刚体变换得到 Marker local NED 位姿
→ GNSS—视觉线性接管
→ 安全高度视觉跟踪
→ 估计甲板位置、速度和协方差
→ 发布受限短时预测位置
→ 视觉长时丢失时回退到 GNSS
```

P3 **不会执行下降**，也不会把预测位置用于 PX4 setpoint。当前 `TRACK_TARGET` 仍使用
P2D 原始视觉位置目标，P4 才会接入预测位置和速度前馈。默认：

```yaml
enable_auto_land: false
```

控制器禁止订阅 Gazebo 甲板 Ground Truth。

---

## 1. 依赖与环境

- Ubuntu 22.04
- ROS 2 Humble
- PX4 SITL 与 Gazebo Harmonic
- PX4 uXRCE-DDS Agent
- 与 PX4 版本匹配的 `px4_msgs`
- Eigen3
- `geometry_msgs`、`nav_msgs`、`sensor_msgs`、`std_msgs`
- `moving_deck_sim` 提供船舶 GNSS
- `aruco_detector` 提供 ArUco 位姿和可见性

## 2. 构建

```bash
cd ~/ws_aruco_landing
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash

colcon build --symlink-install \
  --packages-select aruco_precision_landing_cpp \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
```

---

## 3. 推荐启动顺序

### 3.1 uXRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

### 3.2 PX4 SITL

移动甲板 world 由 `moving_deck_sim` 启动，因此 PX4 使用 standalone 模式：

```bash
cd ~/PX4-Autopilot
PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD=aruco \
PX4_GZ_MODEL_POSE=0,0,2.2 \
make px4_sitl gz_x500_mono_cam_down
```

### 3.3 移动甲板和船舶 GNSS

```bash
cd ~/ws_aruco_landing
source /opt/ros/humble/setup.bash
source install/setup.bash
source ~/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh

ros2 launch moving_deck_sim moving_deck_sim.launch.py
```

切换含噪 GNSS：

```bash
SHARE=$(ros2 pkg prefix --share moving_deck_sim)
ros2 launch moving_deck_sim moving_deck_sim.launch.py \
  gnss_config_file:=$SHARE/config/gnss_noisy.yaml
```

### 3.4 ArUco 检测器

```bash
ros2 launch aruco_detector aruco_detector.launch.py
```

`/aruco/pose` 数值语义固定为相机光学坐标系：

```text
x：图像向右
y：图像向下
z：镜头前方
```

当前消息字符串 `header.frame_id` 默认为 `camera_link`。控制器通过
`expected_aruco_pose_frame_id` 检查字符串，但按 `camera_optical` 数值语义解释。

### 3.5 控制器

```bash
ros2 launch aruco_precision_landing_cpp px4_aruco_landing.launch.py
```

PX4 v1.18 默认 remap：

```text
/fmu/out/vehicle_status
→ /fmu/out/vehicle_status_v4

/fmu/out/vehicle_local_position
→ /fmu/out/vehicle_local_position_v1
```

---

## 4. ROS 2 接口

### 4.1 输入

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/fmu/out/vehicle_status` | `px4_msgs/msg/VehicleStatus` | 模式和解锁状态 |
| `/fmu/out/vehicle_local_position` | `px4_msgs/msg/VehicleLocalPosition` | local NED 和 WGS84 参考原点 |
| `/fmu/out/vehicle_odometry` | `px4_msgs/msg/VehicleOdometry` | `body_frd → local_ned` 位姿 |
| `/deck/gps/fix` | `sensor_msgs/msg/NavSatFix` | 船舶 WGS84 粗位置 |
| `/deck/gps/velocity` | `geometry_msgs/msg/TwistStamped` | 船舶 ENU 速度 |
| `/aruco/pose` | `geometry_msgs/msg/PoseStamped` | Marker `camera_optical` 位姿 |
| `/aruco/visible` | `std_msgs/msg/Bool` | ArUco 可见性 |

船舶速度默认要求：

```text
header.frame_id = world_enu
linear.x = East
linear.y = North
linear.z = Up
```

### 4.2 输出

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/fmu/in/offboard_control_mode` | `px4_msgs/msg/OffboardControlMode` | PX4 位置控制模式声明 |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/msg/TrajectorySetpoint` | 当前 local NED 控制目标 |
| `/fmu/in/vehicle_command` | `px4_msgs/msg/VehicleCommand` | Offboard 和 Arm 命令 |
| `/landing/state` | `std_msgs/msg/String` | 状态机状态 |
| `/landing/guidance_source` | `std_msgs/msg/String` | `GNSS_*`、`BLENDING`、`VISION` |
| `/landing/target_pose` | `geometry_msgs/msg/PoseStamped` | 实际发送给 PX4 的位置目标 |
| `/landing/deck_gnss_pose_ned` | `geometry_msgs/msg/PoseStamped` | 船舶 GNSS local NED 粗位置 |
| `/landing/marker_pose_ned` | `geometry_msgs/msg/PoseStamped` | 完整变换后的原始视觉位置 |
| `/landing/estimated_deck_odometry` | `nav_msgs/msg/Odometry` | 滤波位置、速度和协方差 |
| `/landing/predicted_deck_pose` | `geometry_msgs/msg/PoseStamped` | 短时常速度预测位置 |

常用监控：

```bash
ros2 topic echo /landing/state
ros2 topic echo /landing/guidance_source
ros2 topic echo /landing/marker_pose_ned
ros2 topic echo /landing/estimated_deck_odometry
ros2 topic echo /landing/predicted_deck_pose
ros2 topic echo /landing/target_pose
```

---

## 5. 坐标变换

视觉位姿统一使用：

```text
T_local_ned_marker
=
T_local_ned_body_frd
*
T_body_frd_camera_optical
*
T_camera_optical_marker
```

来源：

- `T_local_ned_body_frd`：PX4 `VehicleOdometry.position + q`。
- `T_body_frd_camera_optical`：YAML 相机外参。
- `T_camera_optical_marker`：ArUco PnP 位姿。

默认外参：

```yaml
camera_extrinsic.translation_frd_m: [0.0, 0.0, -0.10]
camera_extrinsic.rotation_wxyz: [0.70710678, 0.0, 0.0, 0.70710678]
```

方向为：

```text
T_body_frd_camera_optical
```

四元数顺序为 `[w, x, y, z]`。

---

## 6. P2D 状态机保持不变

```text
INIT
→ WAIT_FOR_PX4
→ OFFBOARD_PRE_STREAM
→ ARM_AND_TAKEOFF
→ WAIT_DECK_GNSS
→ RENDEZVOUS_GNSS
→ ACQUIRE_ARUCO
→ VISUAL_HANDOVER
→ TRACK_TARGET
```

视觉长时丢失：

```text
VISUAL_HANDOVER / TRACK_TARGET
→ RECOVER_TO_GNSS
→ RENDEZVOUS_GNSS 或 ACQUIRE_ARUCO
```

全部状态保持：

```text
target_z = -rendezvous_altitude_m
```

P3 没有增加新的飞行状态，也没有修改原 P2D setpoint 控制律。

---

## 7. P3 状态估计

### 7.1 状态

```text
x = [px, py, pz, vx, vy, vz]^T
```

坐标系：PX4 `local_ned`。

### 7.2 模型

```text
p(k+1) = p(k) + v(k) * dt
v(k+1) = v(k)
```

视觉只观测位置。过程噪声使用离散白噪声加速度模型，测量噪声区分水平与垂直方向。

### 7.3 时间处理

滤波 `dt`：

```text
优先使用 /aruco/pose.header.stamp
零时间戳时使用回调到达时间
```

规则：

- 重复或倒退时间戳拒绝。
- 超过重初始化间隔后，用当前测量重新初始化并清零速度。
- 大观测间隔在内部拆成受限预测步长。

### 7.4 离群点

使用归一化创新平方：

```text
NIS = innovation^T * S^-1 * innovation
```

超过门限的测量不会更新滤波观测，也不会刷新最后有效观测到达时间。

### 7.5 估计输出

`/landing/estimated_deck_odometry`：

```text
header.frame_id = local_ned
child_frame_id = estimated_deck
pose.position = estimated position
linear velocity = estimated velocity
```

位置和线速度协方差来自 6×6 状态协方差。甲板姿态和角速度未估计，对应协方差使用大值表示未知。

---

## 8. P3 短时预测

预测时域：

```text
prediction_horizon
=
last_valid_measurement_receipt_age
+
additional_prediction_horizon
```

并限制在：

```text
[0, max_prediction_horizon]
```

预测公式：

```text
p_pred = p_est + v_est * prediction_horizon
v_pred = v_est
```

当前图像采样时间、控制器时间和 PX4 内部时间没有建立严格统一映射，因此不会直接计算：

```text
controller_now - image_header_stamp
```

`/landing/predicted_deck_pose` 目前只用于监控和评估，不进入 PX4 setpoint。

---

## 9. 主要 P3 参数

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `target_state_estimator.process_acceleration_std_mps2` | `1.0` | 过程加速度标准差 |
| `target_state_estimator.measurement_horizontal_std_m` | `0.08` | 水平视觉位置噪声 |
| `target_state_estimator.measurement_vertical_std_m` | `0.12` | 垂直视觉位置噪声 |
| `target_state_estimator.initial_position_std_m` | `0.20` | 初始位置标准差 |
| `target_state_estimator.initial_velocity_std_mps` | `1.0` | 初始速度标准差 |
| `target_state_estimator.minimum_sample_dt_s` | `0.001` | 最小有效采样间隔 |
| `target_state_estimator.maximum_sample_dt_s` | `0.50` | 单次最大预测步长 |
| `target_state_estimator.reinitialize_gap_s` | `2.0` | 长时丢失重初始化阈值 |
| `target_state_estimator.innovation_gate_mahalanobis` | `5.0` | 创新门限 |
| `motion_predictor.additional_prediction_horizon_s` | `0.10` | 固定链路补偿 |
| `motion_predictor.max_prediction_horizon_s` | `0.50` | 最大外推时域 |
| `estimator_output_timeout_s` | `2.0` | 估计输出最长有效年龄 |
| `estimated_deck_child_frame_id` | `estimated_deck` | 估计里程计子坐标系 |

全部参数位于：

```text
config/px4_aruco_landing.yaml
```

---

## 10. 测试

```bash
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash

colcon build --symlink-install \
  --packages-select aruco_precision_landing_cpp \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
colcon test --packages-select aruco_precision_landing_cpp
colcon test-result --verbose
```

当前结果：

```text
77 tests
0 errors
0 failures
0 skipped
```

关键测试：

- `coordinate_transform_test`
- `geodetic_converter_test`
- `gnss_rendezvous_guidance_test`
- `visual_handover_guidance_test`
- `target_state_estimator_test`
- `motion_predictor_test`

详细文档：

```text
docs/P3_VISUAL_STATE_ESTIMATION_PLAN.md
docs/P3_VISUAL_STATE_ESTIMATION_VALIDATION.md
```

---

## 11. 安全提示

节点会自动发送 Offboard 和 Arm 命令。当前主路径：

- 不下降；
- 不发送 Land；
- 不使用预测位置控制；
- 不订阅 Ground Truth。

只应先在 SITL 中验证。实机测试前必须重新核对相机外参、PX4 坐标系、时间同步、
failsafe、人工接管和解锁策略。
