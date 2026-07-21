# aruco_precision_landing_cpp

`aruco_precision_landing_cpp` 是 ROS 2 Humble 下的 PX4 Offboard 控制包。当前实现阶段为
**P2D：船舶 GNSS 会合、ArUco 完整位姿转换、GNSS—视觉平滑接管和下降前恢复**。

当前默认流程：

```text
PX4 状态有效
→ Offboard 预发布、解锁和起飞
→ 等待稳定船舶 GNSS
→ 飞向移动甲板 GNSS 上方
→ 以实时船舶 GNSS 为中心搜索 ArUco
→ 完整刚体变换得到 Marker local NED 位姿
→ GNSS 与视觉位置一致性检查
→ GNSS—视觉线性接管
→ 安全高度视觉跟踪
→ 视觉长时丢失时回退到 GNSS
```

P2D **不会执行下降**。所有 GNSS、接管、视觉跟踪和恢复状态均保持固定
`rendezvous_altitude_m`。默认 `enable_auto_land=false`。

控制器禁止订阅 Gazebo 甲板 Ground Truth。船舶粗位置只能来自经过传感器模型处理的：

```text
/deck/gps/fix
/deck/gps/velocity
```

## 依赖与环境

- Ubuntu 22.04
- ROS 2 Humble
- PX4 SITL 与 Gazebo Harmonic
- PX4 uXRCE-DDS Agent
- 与运行中 PX4 版本匹配的 `px4_msgs`
- `moving_deck_sim` 发布的船舶 GNSS
- `aruco_detector` 发布的 `/aruco/pose` 和 `/aruco/visible`

## 构建

```bash
cd ~/ws_aruco_landing
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash

colcon build --symlink-install --packages-select aruco_precision_landing_cpp
source install/setup.bash
```

## 推荐启动顺序

### 1. 启动 uXRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

### 2. 启动 PX4 SITL

移动甲板 world 由 `moving_deck_sim` 启动，因此 PX4 使用 standalone 模式：

```bash
cd ~/PX4-Autopilot
PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD=aruco \
PX4_GZ_MODEL_POSE=0,0,2.2 \
make px4_sitl gz_x500_mono_cam_down
```

### 3. 启动移动甲板和船舶 GNSS

```bash
cd ~/ws_aruco_landing
source /opt/ros/humble/setup.bash
source install/setup.bash
source ~/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh

ros2 launch moving_deck_sim moving_deck_sim.launch.py
```

含噪 GNSS：

```bash
SHARE=$(ros2 pkg prefix --share moving_deck_sim)
ros2 launch moving_deck_sim moving_deck_sim.launch.py \
  gnss_config_file:=$SHARE/config/gnss_noisy.yaml
```

### 4. 启动 ArUco 检测器

```bash
ros2 launch aruco_detector aruco_detector.launch.py
```

检测器的 `/aruco/pose` 数值语义固定为：

```text
camera_optical
x：图像向右
y：图像向下
z：镜头前方
```

当前消息字符串 `header.frame_id` 默认为 `camera_link`。控制器通过
`expected_aruco_pose_frame_id` 检查字符串，但始终按 `camera_optical` 数值语义解释。

### 5. 启动控制器

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

## ROS 2 接口

### 输入

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/fmu/out/vehicle_status` | `px4_msgs/msg/VehicleStatus` | 模式和解锁状态 |
| `/fmu/out/vehicle_local_position` | `px4_msgs/msg/VehicleLocalPosition` | local NED 和 WGS84 参考原点 |
| `/fmu/out/vehicle_odometry` | `px4_msgs/msg/VehicleOdometry` | `body_frd → local_ned` 完整姿态和位置 |
| `/deck/gps/fix` | `sensor_msgs/msg/NavSatFix` | 船舶 WGS84 粗位置 |
| `/deck/gps/velocity` | `geometry_msgs/msg/TwistStamped` | 船舶 ENU 速度 |
| `/aruco/pose` | `geometry_msgs/msg/PoseStamped` | Marker 在 `camera_optical` 中的完整位姿 |
| `/aruco/visible` | `std_msgs/msg/Bool` | ArUco 可见性 |

船舶速度默认要求：

```text
header.frame_id = world_enu
linear.x = East
linear.y = North
linear.z = Up
```

### 输出

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/fmu/in/offboard_control_mode` | `px4_msgs/msg/OffboardControlMode` | PX4 Offboard 模式声明 |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/msg/TrajectorySetpoint` | local NED 位置目标 |
| `/fmu/in/vehicle_command` | `px4_msgs/msg/VehicleCommand` | Offboard 和 Arm 命令 |
| `/landing/state` | `std_msgs/msg/String` | 当前状态 |
| `/landing/target_pose` | `geometry_msgs/msg/PoseStamped` | 当前 local NED 目标 |
| `/landing/deck_gnss_pose_ned` | `geometry_msgs/msg/PoseStamped` | 船舶 GNSS 粗位置 |
| `/landing/marker_pose_ned` | `geometry_msgs/msg/PoseStamped` | 完整刚体变换后的 Marker 位姿 |
| `/landing/guidance_source` | `std_msgs/msg/String` | `GNSS_*`、`BLENDING` 或 `VISION` |

调试：

```bash
ros2 topic echo /landing/state
ros2 topic echo /landing/guidance_source
ros2 topic echo /landing/target_pose
ros2 topic echo /landing/deck_gnss_pose_ned
ros2 topic echo /landing/marker_pose_ned
```

## 完整视觉坐标变换

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
- `T_camera_optical_marker`：ArUco PnP 完整位置和姿态。

默认外参：

```yaml
camera_extrinsic.translation_frd_m: [0.0, 0.0, -0.10]
camera_extrinsic.rotation_wxyz: [0.70710678, 0.0, 0.0, 0.70710678]
```

参数方向明确为：

```text
T_body_frd_camera_optical
```

四元数顺序为 `[w, x, y, z]`。输入 NaN、Inf、零范数四元数、错误 PX4
`pose_frame` 或错误 ArUco `frame_id` 会被拒绝。

旧静态基线的两个相机正负号参数仍保留在配置中，只供不可达的历史静态下降代码参考；
P2D 主路径不再使用它们计算视觉目标。

## P2D 状态机

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

异常恢复：

```text
VISUAL_HANDOVER / TRACK_TARGET
→ 视觉短时丢失：保持当前水平目标和安全高度
→ 视觉长时丢失：RECOVER_TO_GNSS
→ RENDEZVOUS_GNSS 或 ACQUIRE_ARUCO
```

### `ACQUIRE_ARUCO`

搜索中心始终跟随实时船舶 GNSS。只有同时满足以下条件才进入接管：

- ArUco 连续可见达到 `aruco_acquire_duration_s`。
- 位姿回调到达时间新鲜。
- 完整刚体变换成功。
- Marker local NED 位置有限。
- GNSS 与视觉水平距离小于 `handover_max_horizontal_difference_m`。
- 视觉测量没有超过 `max_visual_measurement_jump_m` 的异常跳变。

单帧检测不会触发接管。

### `VISUAL_HANDOVER`

接管权重：

```text
alpha = clamp(valid_handover_time / visual_handover_duration_s, 0, 1)
target_xy = (1-alpha) * gnss_xy + alpha * visual_xy
```

只有视觉有效的控制周期才累加接管时间。混合目标继续受最大速度和单周期步长限幅。
目标 z 始终为：

```text
-rendezvous_altitude_m
```

### `TRACK_TARGET`

- 水平目标来自完整变换后的 Marker local NED 位置。
- 目标变化继续限幅。
- 不使用 GNSS 混合控制。
- 不下降。

### `RECOVER_TO_GNSS`

- 清除旧视觉测量。
- 保持安全高度。
- 使用有效 GNSS 粗位置恢复。
- 无人机仍在会合半径内时回到 `ACQUIRE_ARUCO`，否则回到 `RENDEZVOUS_GNSS`。
- GNSS 也不可用时进入 `WAIT_DECK_GNSS` 并锁定当前水平位置。

## 时间约定

当前 ROS `/clock`、相机图像时间和 PX4 时间域尚未统一，因此：

- 视觉控制新鲜度使用控制器回调到达时间。
- 非零图像采样时间戳只用于拒绝重复或乱序帧。
- `/landing/marker_pose_ned` 时间戳表示完成转换的控制器时间。
- 当前变换使用回调时最新 `VehicleOdometry`，尚未对图像采样时刻插值。

P3 必须补充跨传感器时间对齐、滤波和运动预测。

## 主要参数

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `rendezvous_altitude_m` | `5.0` | GNSS、接管和视觉跟踪安全高度 |
| `rendezvous_radius_m` | `2.0` | 会合完成水平半径 |
| `aruco_acquire_duration_s` | `0.5` | 进入接管前连续可见时间 |
| `visual_handover_duration_s` | `0.5` | GNSS 到视觉混合时长 |
| `handover_max_horizontal_difference_m` | `3.0` | GNSS 与视觉最大水平差 |
| `max_visual_measurement_jump_m` | `0.5` | 相邻有效视觉测量最大水平跳变 |
| `visual_loss_short_timeout_s` | `0.5` | 视觉测量短时新鲜阈值 |
| `visual_loss_long_timeout_s` | `2.0` | 触发 GNSS 恢复的长时丢失阈值 |
| `max_rendezvous_speed_mps` | `2.0` | GNSS和视觉目标最大移动速度 |
| `max_target_step_m` | `0.20` | 单周期水平目标最大步长 |
| `expected_aruco_pose_frame_id` | `camera_link` | 允许的 ArUco 消息 frame 字符串 |
| `enable_auto_land` | `false` | 默认禁止自动降落 |

全部参数位于：

```text
config/px4_aruco_landing.yaml
```

## 测试

```bash
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash

colcon build --symlink-install \
  --packages-select aruco_precision_landing_cpp \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
colcon test --packages-select aruco_precision_landing_cpp
colcon test-result --verbose
```

关键测试：

- `coordinate_transform_test`
- `geodetic_converter_test`
- `gnss_rendezvous_guidance_test`
- `visual_handover_guidance_test`

P2D 消息级状态机验收见：

```text
docs/P2D_GNSS_VISION_HANDOVER_VALIDATION.md
```

## 安全提示

节点会自动发送 Offboard 和 Arm 命令。P2D 默认不发送 Land，不进入下降状态。只应先在
SITL 中验证；实机测试前必须重新核对相机外参、PX4 坐标系、failsafe、人工接管和解锁策略。
