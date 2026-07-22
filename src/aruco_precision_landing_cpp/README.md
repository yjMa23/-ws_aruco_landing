# aruco_precision_landing_cpp

`aruco_precision_landing_cpp` 是 ROS 2 Humble 下的 PX4 Offboard 移动甲板自主降落传统基线控制包。
当前实现阶段为：

```text
P4：安全高度移动甲板水平跟踪
```

当前主路径：

```text
PX4 状态有效
→ Offboard 预发布、解锁和起飞
→ 等待稳定船舶 GNSS
→ GNSS 会合和移动中心搜索
→ ArUco 完整 local NED 位姿变换
→ GNSS—视觉平滑接管
→ 视觉位置/速度状态估计
→ 预测位置 + 甲板速度前馈水平跟踪
→ 视觉长时丢失时恢复到 GNSS
```

P4 全程保持固定安全高度，不执行下降，不判断着陆窗口，不发送 Land。默认：

```yaml
enable_auto_land: false
```

控制器禁止订阅 Gazebo 甲板 Ground Truth。

## 1. 依赖与环境

- Ubuntu 22.04
- ROS 2 Humble
- PX4 SITL 与 Gazebo Harmonic
- PX4 uXRCE-DDS Agent
- 与运行中 PX4 版本匹配的 `px4_msgs`
- `moving_deck_sim` 发布船舶 GNSS
- `aruco_detector` 发布 ArUco 位姿与可见性

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

## 3. 推荐启动顺序

### 3.1 uXRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

### 3.2 PX4 SITL

移动甲板 world 由 `moving_deck_sim` 启动，因此 PX4 使用 standalone：

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

静止场景：

```bash
SHARE=$(ros2 pkg prefix --share moving_deck_sim)
ros2 launch moving_deck_sim moving_deck_sim.launch.py \
  config_file:=$SHARE/config/static.yaml
```

正弦场景：

```bash
ros2 launch moving_deck_sim moving_deck_sim.launch.py \
  config_file:=$SHARE/config/sinusoidal_xy.yaml
```

### 3.4 相机桥接与 ArUco 检测

```bash
ros2 run ros_gz_bridge parameter_bridge \
  '/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
```

```bash
cd ~/ws_aruco_landing
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch aruco_detector aruco_detector.launch.py
```

### 3.5 控制器

```bash
cd ~/ws_aruco_landing
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash
source install/setup.bash

ros2 launch aruco_precision_landing_cpp px4_aruco_landing.launch.py
```

PX4 v1.18 默认 remap：

```text
/fmu/out/vehicle_status       → /fmu/out/vehicle_status_v4
/fmu/out/vehicle_local_position → /fmu/out/vehicle_local_position_v1
```

## 4. 当前状态机

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

恢复路径：

```text
VISUAL_HANDOVER / TRACK_TARGET
→ RECOVER_TO_GNSS
→ ACQUIRE_ARUCO 或 RENDEZVOUS_GNSS
```

旧静态下降状态仍作为 P0 历史基线保留，但当前主路径不可达。

## 5. 坐标与完整视觉变换

统一使用：

```text
camera_optical = [Right, Down, Forward]
body_frd       = [Forward, Right, Down]
local_ned      = [North, East, Down]
```

视觉位姿：

```text
T_local_ned_marker
=
T_local_ned_body_frd
*
T_body_frd_camera_optical
*
T_camera_optical_marker
```

默认相机外参：

```yaml
camera_extrinsic.translation_frd_m: [0.0, 0.0, -0.10]
camera_extrinsic.rotation_wxyz: [0.70710678, 0.0, 0.0, 0.70710678]
```

## 6. P3 状态估计和预测

状态：

```text
[x, y, z, vx, vy, vz]
```

使用三维常速度 Kalman Filter：

- 图像采样时间计算滤波 `dt`；
- NIS 门限拒绝离群点；
- 处理乱序、重复时间和大时间间隔；
- 长时丢失后重新初始化；
- 输出位置、速度和 6×6 协方差。

短时预测：

```text
p_pred = p_est + v_est * horizon
```

预测时域包含滤波状态年龄和可配置附加补偿，并受最大预测时域限制。

## 7. P4 水平跟踪控制

PX4 使用“位置目标 + 速度前馈”。

位置目标：

```text
position_sp_xy = 受限视觉/估计/预测甲板位置
position_sp_z  = -rendezvous_altitude_m
```

速度前馈：

```text
v_ff_xy
=
velocity_feedforward_gain * v_deck_xy
+
relative_velocity_gain * (v_deck_xy - v_uav_xy)
```

位置误差反馈由 PX4 内部位置控制器完成，外部不重复叠加位置 P。

约束：

- 水平位置目标最大速度；
- 单周期最大位置变化；
- 水平前馈最大速度；
- 前馈最大加速度；
- 短时视觉丢失时预测年龄限制；
- 短时丢失时前馈逐步衰减；
- 输入不足时保持最近安全位置并清除前馈。

PX4 TrajectorySetpoint：

```text
position = [North target, East target, -safe altitude]
velocity = [North feedforward, East feedforward, NaN]
```

## 8. 可配置跟踪模式

参数：

```yaml
tracking.mode: PREDICTED_POSITION_VELOCITY_FF
```

支持：

| 模式 | 位置目标 | 速度前馈 |
| --- | --- | --- |
| `RAW_VISUAL_POSITION` | P2D 原始视觉位置 | 无 |
| `ESTIMATED_POSITION` | Kalman 估计位置 | 无 |
| `ESTIMATED_POSITION_VELOCITY_FF` | Kalman 估计位置 | 有 |
| `PREDICTED_POSITION_VELOCITY_FF` | 控制时刻预测位置 | 有 |

切换为原始视觉对照：

```bash
ros2 launch aruco_precision_landing_cpp px4_aruco_landing.launch.py \
  --show-args
```

推荐复制默认 YAML，修改：

```yaml
tracking.mode: RAW_VISUAL_POSITION
```

再使用：

```bash
ros2 launch aruco_precision_landing_cpp px4_aruco_landing.launch.py \
  config_file:=/path/to/raw_visual.yaml
```

## 9. ROS 2 接口

### 9.1 输入

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/fmu/out/vehicle_status` | `px4_msgs/msg/VehicleStatus` | PX4 模式和解锁状态 |
| `/fmu/out/vehicle_local_position` | `px4_msgs/msg/VehicleLocalPosition` | UAV NED 位置、速度和地理参考 |
| `/fmu/out/vehicle_odometry` | `px4_msgs/msg/VehicleOdometry` | UAV NED 位姿 |
| `/deck/gps/fix` | `sensor_msgs/msg/NavSatFix` | 船舶 GNSS 粗位置 |
| `/deck/gps/velocity` | `geometry_msgs/msg/TwistStamped` | 船舶 ENU 粗速度 |
| `/aruco/pose` | `geometry_msgs/msg/PoseStamped` | Marker camera optical 位姿 |
| `/aruco/visible` | `std_msgs/msg/Bool` | Marker 可见性 |

### 9.2 输出

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/fmu/in/offboard_control_mode` | `px4_msgs/msg/OffboardControlMode` | Offboard 控制声明 |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/msg/TrajectorySetpoint` | NED 位置目标和水平速度前馈 |
| `/fmu/in/vehicle_command` | `px4_msgs/msg/VehicleCommand` | Offboard 和 Arm 命令 |
| `/landing/state` | `std_msgs/msg/String` | 状态机状态 |
| `/landing/guidance_source` | `std_msgs/msg/String` | GNSS、视觉接管或跟踪模式 |
| `/landing/target_pose` | `geometry_msgs/msg/PoseStamped` | 当前 NED 位置目标 |
| `/landing/deck_gnss_pose_ned` | `geometry_msgs/msg/PoseStamped` | 船舶 GNSS NED 粗位置 |
| `/landing/marker_pose_ned` | `geometry_msgs/msg/PoseStamped` | Marker 完整变换位置 |
| `/landing/estimated_deck_odometry` | `nav_msgs/msg/Odometry` | 估计位置、速度和协方差 |
| `/landing/predicted_deck_pose` | `geometry_msgs/msg/PoseStamped` | 控制时刻预测位置 |
| `/landing/tracking_velocity_setpoint` | `geometry_msgs/msg/TwistStamped` | 实际水平速度前馈调试值 |

监控：

```bash
ros2 topic echo /landing/state
ros2 topic echo /landing/guidance_source
ros2 topic echo /landing/target_pose
ros2 topic echo /landing/estimated_deck_odometry
ros2 topic echo /landing/predicted_deck_pose
ros2 topic echo /landing/tracking_velocity_setpoint
```

## 10. 主要参数

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `rendezvous_altitude_m` | `5.0` | 会合、接管和跟踪安全高度 |
| `visual_loss_short_timeout_s` | `0.5` | 视觉新鲜阈值 |
| `visual_loss_long_timeout_s` | `2.0` | GNSS 恢复阈值 |
| `tracking.mode` | `PREDICTED_POSITION_VELOCITY_FF` | 跟踪模式 |
| `tracking.max_position_target_speed_mps` | `2.0` | 位置目标最大移动速度 |
| `tracking.max_position_target_step_m` | `0.20` | 单周期最大位置变化 |
| `tracking.velocity_feedforward_gain` | `1.0` | 甲板速度前馈系数 |
| `tracking.relative_velocity_gain` | `0.25` | 相对速度阻尼系数 |
| `tracking.max_velocity_feedforward_mps` | `1.5` | 前馈速度上限 |
| `tracking.max_velocity_feedforward_acceleration_mps2` | `1.0` | 前馈加速度上限 |
| `tracking.max_prediction_age_s` | `0.75` | 短时预测最大年龄 |
| `enable_auto_land` | `false` | 禁止自动降落 |

全部参数见：

```text
config/px4_aruco_landing.yaml
```

## 11. 测试

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
93 tests
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
- `moving_target_tracking_controller_test`

详细文档：

```text
docs/P3_VISUAL_STATE_ESTIMATION_PLAN.md
docs/P3_VISUAL_STATE_ESTIMATION_VALIDATION.md
docs/P4_MOVING_TARGET_TRACKING_PLAN.md
docs/P4_MOVING_TARGET_TRACKING_VALIDATION.md
```

## 12. 当前验收边界

已完成：

- 纯控制模块与 13 项跟踪测试；
- 静止消息级前馈为零；
- East `0.4 m/s` 消息级估计与前馈方向正确；
- PX4 TrajectorySetpoint 收到位置和速度前馈；
- 长时视觉丢失回 GNSS；
- 原始视觉模式可通过 YAML 恢复；
- 全程保持安全高度。

尚需用户真实 PX4 SITL 验证：

- 静止、`0.2 m/s`、`0.4 m/s` 和正弦甲板水平 RMSE；
- 原始视觉、速度前馈、预测前馈的定量对比；
- 实际无人机速度、加速度和 Marker 丢失次数；
- 参数稳定性与必要调参。

真实跟踪验收通过前，不进入 P5 下降阶段。

## 13. 安全提示

节点会自动发送 Offboard 和 Arm 命令，只应先在 SITL 中运行。P4 默认不下降、不发送 Land。
实机测试前必须重新核对相机外参、时间同步、PX4 坐标系、速度/加速度限制、failsafe、人工接管和解锁策略。
