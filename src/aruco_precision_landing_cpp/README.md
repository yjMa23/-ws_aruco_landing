# aruco_precision_landing_cpp

`aruco_precision_landing_cpp` 是 ROS 2 Humble 下的 PX4 Offboard 移动甲板自主降落传统基线控制包。
当前实现阶段为：

```text
P8B：水平相对运动线性 MPC（VALIDATION PASS）
P8C-3：FAILURE EVIDENCE PRESERVED
P8C-4：TERMINAL CONTACT STABILIZATION VALIDATION PASS
P8C fixed T1：VALIDATION PASS / P8C-3 DESIGN GATE CLOSED
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
→ P4.7 水平跟踪或显式 P8B MPC
→ 规则式着陆窗口
→ 相对高度和最终下降
→ 多源触地确认
→ 接触后相对保持

并行 shadow：甲板向上法向 → 甲板平面/X500 四滑橇间隙 → 法向/切向诊断
```

相对下降和最终下降均需显式授权。默认：

```yaml
enable_auto_land: false
descent.enabled: false
final_descent.enabled: false
```

控制器禁止订阅 Gazebo 甲板 Ground Truth，不发送 `NAV_LAND`，不自动 Disarm。P8C-1/P8C-2 的几何与独立 shadow 法向滤波保持诊断隔离；P8C-4 只在固定正 `+2° roll/pitch` 且显式授权的终端阶段进入生产控制，通过 Offboard position setpoint 内的法向整形、接触锚点顺应、切向阻尼、candidate/HOLD 法向锁存和受限预压稳定接触，不直接发送 PX4 attitude setpoint。最终 roll/pitch 真实触地 6/6、旧路径回归 9/9；负倾角、动态姿态和 combined touchdown 继续关闭。

## 1. 依赖与环境

- Ubuntu 22.04
- ROS 2 Humble
- PX4 SITL 与 Gazebo Harmonic
- PX4 uXRCE-DDS Agent
- 与运行中 PX4 版本匹配的 `px4_msgs`
- OSQP `v1.0.0` 与 OsqpEigen `v0.11.2`
- `moving_deck_sim` 发布船舶 GNSS
- `aruco_detector` 发布 ArUco 位姿与可见性

## 2. 构建

```bash
cd ~/ws_aruco_landing
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash

export P8B_MPC_PREFIX="$HOME/.local/p8b-mpc/osqp-1.0.0-osqpeigen-0.11.2"
export CMAKE_PREFIX_PATH="$P8B_MPC_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$P8B_MPC_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

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
PX4_GZ_MODEL_POSE=-4,0,0.2 \
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
camera_extrinsic.translation_frd_m: [0.0, 0.0, 0.14]
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

| 模式 | 位置目标 | 前馈/控制 |
| --- | --- | --- |
| `RAW_VISUAL_POSITION` | P2D 原始视觉位置 | 无 |
| `ESTIMATED_POSITION` | Kalman 估计位置 | 无 |
| `ESTIMATED_POSITION_VELOCITY_FF` | Kalman 估计位置 | 甲板速度 |
| `PREDICTED_POSITION_VELOCITY_FF` | 控制时刻预测位置 | P4.7 速度前馈与自适应阻尼 |
| `RELATIVE_MPC` | 控制时刻预测位置 | MPC 水平加速度，失败或终端阶段回退 P4.7 |

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

## P8C-0/P8C-1/P8C-2 倾斜甲板 shadow 几何、安全高度与安全下降

`DeckPlaneGeometry` 是不依赖 ROS/Gazebo/Ground Truth 的 C++17 纯数学模块。统一语义：

```text
local_ned      = North, East, Down
base_link_frd  = Forward, Right, Down
n              = 指向甲板上方的单位法向；水平甲板为 [0, 0, -1]
h_body         = nᵀ(p_uav - p_deck)
h_i            = nᵀ(p_skid_i - p_deck)
P_t            = I - n nᵀ
```

默认 X500 四个 FRD 等效接触点来自真实 `x500_base/model.sdf` 两条
`0.25×0.015×0.015 m` 滑橇碰撞盒最低中心线：

```text
[-0.125, -0.132, 0.227] m
[+0.125, -0.132, 0.227] m
[-0.125, +0.132, 0.227] m
[+0.125, +0.132, 0.227] m
```

该模型忽略碰撞盒 7.5 mm 半宽支持面、柔性和接触求解压入。甲板角速度尚未在线估计时，参考点法向速度仍可发布，但四端点动态法向速度保持无效，不填零冒充。

```yaml
deck_plane_geometry.enabled: true
deck_plane_geometry.shadow_only: true
deck_plane_geometry.normal_filter_gain: 0.08
deck_plane_geometry.apply_marker_plane_offset: true
deck_plane_geometry.contact_points_body_frd_m: [...12 values...]
deck_plane_geometry.marker_plane_offsets_m: [0.001, 0.002, 0.003, 0.004]
```

离线检查：

```bash
python3 scripts/evaluate_p8c_tilted_deck.py --help
python3 scripts/evaluate_p8c_tilted_deck.py /path/to/bag --json
```

P8C-1 已完成固定 ±2° 安全高度 12/12 验收，完整法向最差 RMSE/P95 为 `0.702°/1.353°`。P8C-2 只对白名单 `tilt_roll_pos_2deg / tilt_pitch_pos_2deg` 允许相对下降到严格 `0.50 m`，完成正倾角 3+3、static 和 constant02 共 8/8 PASS；最差水平 RMSE/max 为 `0.020931/0.068704 m`，最低真实滑橇间隙 `0.210051 m`，完整法向最差 RMSE/P95 为 `0.317644°/0.562188°`。P8C-3 失败证据已保留，P8C-4 fixed T1 touchdown 已完成 roll/pitch 6/6 与旧路径 9/9 验收；负倾角、动态 roll/pitch/combined final descent 和 PX4 attitude setpoint 姿态对齐仍未开放。

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
| `/fmu/out/vehicle_land_detected` | `px4_msgs/msg/VehicleLandDetected` | 多源触地证据 |

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
| `/landing/effective_relative_velocity_gain` | `std_msgs/msg/Float64` | 当前实际使用的相对速度阻尼增益 |
| `/landing/estimated_deck_acceleration` | `geometry_msgs/msg/TwistStamped` | local NED 过滤后甲板水平加速度 |
| `/landing/relative_mpc/*` | 多种标准消息 | MPC 求解、控制、约束和预测诊断 |
| `/landing/deck_plane/*` | 多种标准消息 | P8C 法向、机体/四滑橇间隙、首接触、法向速度、平面内误差、状态和 Marker 法向标定 shadow 诊断 |
| `/landing/touchdown_*` | 多种标准消息 | 触地证据、候选时间、确认和接触后保持诊断 |

监控：

```bash
ros2 topic echo /landing/state
ros2 topic echo /landing/guidance_source
ros2 topic echo /landing/target_pose
ros2 topic echo /landing/estimated_deck_odometry
ros2 topic echo /landing/predicted_deck_pose
ros2 topic echo /landing/tracking_velocity_setpoint
ros2 topic echo /landing/effective_relative_velocity_gain
ros2 topic echo /landing/estimated_deck_acceleration
ros2 topic echo /landing/deck_plane/status
ros2 topic echo /landing/deck_plane/skid_clearances
ros2 topic echo /landing/deck_plane/clearance_spread
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
| `tracking.relative_velocity_gain` | `0.25` | 调度关闭时使用的固定相对速度阻尼系数 |
| `tracking.adaptive_relative_velocity_gain.enabled` | `true` | 启用加速度感知连续增益调度 |
| `tracking.adaptive_relative_velocity_gain.min_gain` | `0.25` | 匀速阶段最小阻尼增益 |
| `tracking.adaptive_relative_velocity_gain.max_gain` | `1.20` | 加速和换向阶段最大阻尼增益 |
| `tracking.adaptive_relative_velocity_gain.acceleration_low_threshold_mps2` | `0.05` | 开始提升增益的加速度阈值 |
| `tracking.adaptive_relative_velocity_gain.acceleration_high_threshold_mps2` | `0.35` | 达到最大增益的加速度阈值 |
| `tracking.adaptive_relative_velocity_gain.max_acceleration_mps2` | `1.50` | 速度差分加速度模长上限 |
| `tracking.adaptive_relative_velocity_gain.acceleration_filter_gain` | `0.20` | 加速度一阶低通系数 |
| `tracking.max_velocity_feedforward_mps` | `1.5` | 前馈速度上限 |
| `tracking.max_velocity_feedforward_acceleration_mps2` | `1.0` | 前馈加速度上限 |
| `tracking.max_prediction_age_s` | `0.75` | 短时预测最大年龄 |
| `deck_attitude.filter_gain` | `0.20` | Marker 向上法向量低通系数 |
| `deck_plane_geometry.enabled` | `true` | 启用 P8C shadow 几何诊断 |
| `deck_plane_geometry.shadow_only` | `true` | P8C 安全门；必须保持为 true |
| `deck_plane_geometry.normal_filter_gain` | `0.08` | 仅供 P8C shadow 几何使用的独立法向低通增益；不改变 production `deck_attitude.filter_gain=0.20` |
| `deck_plane_geometry.contact_points_body_frd_m` | X500 四端点 | FRD 等效滑橇接触点，12 个数值 |
| `deck_plane_geometry.marker_plane_offsets_m` | `[0.001,0.002,0.003,0.004]` | 四 Marker visual 平面到碰撞顶面的名义偏移 |
| `landing_window.enter_horizontal_error_m` | `0.15` | 窗口进入水平误差阈值 |
| `landing_window.exit_horizontal_error_m` | `0.25` | 窗口退出水平误差阈值 |
| `landing_window.enter_relative_speed_mps` | `0.15` | 窗口进入相对速度阈值 |
| `landing_window.exit_relative_speed_mps` | `0.25` | 窗口退出相对速度阈值 |
| `landing_window.enter_max_tilt_deg` | `5.0` | 窗口进入甲板倾角阈值 |
| `landing_window.exit_max_tilt_deg` | `8.0` | 窗口退出甲板倾角阈值 |
| `landing_window.required_duration_s` | `1.0` | 所有进入条件连续满足时间 |
| `vehicle_pose_history.history_duration_s` | `2.0` | PX4 local NED 位姿历史时长 |
| `vehicle_pose_history.max_endpoint_hold_s` | `0.03` | 图像时刻最多允许的端点保持时间 |
| `vehicle_pose_history.clock_offset_filter_gain` | `0.05` | PX4→ROS 时钟偏移低通更新系数 |
| `vehicle_pose_history.max_clock_offset_jump_s` | `0.10` | 触发历史重置的最大时钟偏移跳变 |
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
294 tests
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
- `vehicle_pose_history_test`
- `deck_attitude_estimator_test`
- `deck_plane_geometry_test`
- `p8c_tilted_deck_tests`
- `landing_window_test`
- `relative_descent_controller_test`
- `vertical_state_estimator_test`
- `touchdown_detector_test`

详细文档：

```text
docs/plans/P3_VISUAL_STATE_ESTIMATION_PLAN.md
docs/validation/P3_VISUAL_STATE_ESTIMATION_VALIDATION.md
docs/plans/P4_MOVING_TARGET_TRACKING_PLAN.md
docs/validation/P4_MOVING_TARGET_TRACKING_VALIDATION.md
docs/plans/P4_5_EXECUTION_PLAN.md
docs/validation/P4_5_TIME_ALIGNMENT_VALIDATION.md
docs/plans/P5A_DECK_DYNAMICS_AND_LANDING_WINDOW_PLAN.md
docs/validation/P5A_DECK_DYNAMICS_AND_LANDING_WINDOW_VALIDATION.md
docs/plans/P5B_RELATIVE_DESCENT_PLAN.md
docs/validation/P5B_RELATIVE_DESCENT_VALIDATION.md
docs/plans/P5C_VERTICAL_STATE_ESTIMATION_PLAN.md
docs/validation/P5C_VERTICAL_STATE_ESTIMATION_VALIDATION.md
docs/plans/P6_TOUCHDOWN_CONFIRMATION_PLAN.md
docs/validation/P6_TOUCHDOWN_CONFIRMATION_VALIDATION.md
docs/plans/P6B_FINAL_DESCENT_AND_TOUCHDOWN_PLAN.md
docs/validation/P6B_FINAL_DESCENT_AND_TOUCHDOWN_VALIDATION.md
docs/plans/P7_BATCH_EVALUATION_PLAN.md
docs/plans/P8A_HEAVE_TOUCHDOWN_PLAN.md
docs/validation/P8A_HEAVE_TOUCHDOWN_VALIDATION.md
docs/plans/P8B_RELATIVE_MPC_PLAN.md
docs/validation/P8B_RELATIVE_MPC_VALIDATION.md
docs/research/P8C_TILTED_DECK_LANDING_REVIEW.md
docs/plans/P8C_TILTED_DECK_LANDING_PLAN.md
docs/validation/P8C_TILTED_DECK_LANDING_VALIDATION.md
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

P4～P4.6 已完成静止、`0.2 m/s`、`0.4 m/s`、正弦、时间对齐和参数扫描。
P4.7 已完成加速度感知连续增益调度，并设为统一默认：0.4 m/s 匀速位置 RMSE 为
`0.0554 m`，XY 正弦位置 RMSE 为 `0.3490 m`，Marker 丢失和 GNSS 恢复均为 0。

P5A～P6B 已完成着陆窗口、相对下降、垂直估计、最终下降和真实接触。P7-lite 真实 3+3 冒烟为 6/6 PASS；P8A H1/H2 升沉触地均为 3/3 PASS；P8B 已完成固定 OSQP/OsqpEigen 依赖、完整 P4.7 回退和严格顺序真实 SITL。P8C-3 的 +2° roll seed2 滑移硬门失败及姿态发散、离板、恢复证据完整保留；P8C-4 已完成终端接触稳定化实现和分级验证。P9 自动化、真实实验和聚合已完成：smoke `20/27`、baseline `40/40`、formal ablation `60/60`，三批次合计 `120/127`；B2 constant02、B4 heave_h1、B5 pitch `+2°` 保持 `NOT_APPLICABLE`，B5 roll `+2°` 正式 touchdown 为 10/10。正式运行提交为 `71af1cc`，全程 `NAV_LAND / Disarm = 0 / 0`。

## 13. 安全提示

节点会自动发送 Offboard 和 Arm 命令，只应先在 SITL 中运行。相对下降和最终下降默认关闭；固定正 +2° 场景只有在 relative descent、严格 0.50 m、final descent 和 P8C-4 active 终端稳定化同时显式启用时开放。负倾角、动态 roll/pitch、combined 和 dynamic attitude final descent 继续拒绝；当前方案不是 PX4 attitude setpoint 姿态对齐。不发送 `NAV_LAND` 或 Disarm。
实机测试前必须重新核对相机外参、时间同步、PX4 坐标系、速度/加速度限制、failsafe、人工接管和解锁策略。
