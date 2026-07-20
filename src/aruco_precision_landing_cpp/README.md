# aruco_precision_landing_cpp

`aruco_precision_landing_cpp` 是 ROS 2 Humble 下的 PX4 Offboard 控制包。当前实现阶段为
**P2C：船舶 GNSS 会合与移动甲板上方粗跟踪**。

当前默认流程：

```text
PX4 状态有效
→ Offboard 预发布、解锁和起飞
→ 等待稳定船舶 GNSS
→ 飞向移动甲板 GNSS 上方
→ 在安全高度持续粗跟踪
→ 以实时船舶 GNSS 为中心搜索 ArUco
→ 稳定识别后回到 GNSS 中心悬停
```

P2C **不会执行下降**。稳定识别 ArUco 后只在安全高度悬停，等待后续 P2D 实现完整
坐标变换和 GNSS—视觉接管。

控制器禁止订阅 Gazebo 甲板 Ground Truth。船舶位置只能来自经过传感器模型处理的：

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
- 可选：发布 `/aruco/pose` 和 `/aruco/visible` 的 `aruco_detector`

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

默认启用理想 GNSS。切换到含噪配置：

```bash
SHARE=$(ros2 pkg prefix --share moving_deck_sim)
ros2 launch moving_deck_sim moving_deck_sim.launch.py \
  gnss_config_file:=$SHARE/config/gnss_noisy.yaml
```

启动控制器前确认：

```bash
ros2 topic echo /deck/gps/fix --once \
  --qos-reliability best_effort
ros2 topic echo /deck/gps/velocity --once \
  --qos-reliability best_effort
ros2 topic echo /fmu/out/vehicle_local_position_v1 --once
```

PX4 `VehicleLocalPosition` 必须满足：

```text
xy_global = true
z_global = true
ref_lat / ref_lon / ref_alt 有限
```

### 4. 启动 ArUco 检测器

P2C 在没有视觉时仍会完成 GNSS 会合和移动搜索。需要验证 ArUco 捕获时再启动：

```bash
ros2 launch aruco_detector aruco_detector.launch.py
```

### 5. 启动控制器

```bash
ros2 launch aruco_precision_landing_cpp px4_aruco_landing.launch.py
```

当前 PX4 v1.18 默认 remap：

```text
/fmu/out/vehicle_status
→ /fmu/out/vehicle_status_v4

/fmu/out/vehicle_local_position
→ /fmu/out/vehicle_local_position_v1
```

可以覆盖 PX4 或船舶 GNSS 话题：

```bash
ros2 launch aruco_precision_landing_cpp px4_aruco_landing.launch.py \
  vehicle_status_topic:=/fmu/out/vehicle_status \
  vehicle_local_position_topic:=/fmu/out/vehicle_local_position \
  deck_gps_fix_topic:=/deck/gps/fix \
  deck_gps_velocity_topic:=/deck/gps/velocity
```

## ROS 2 接口

### 输入

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/fmu/out/vehicle_status` | `px4_msgs/msg/VehicleStatus` | 模式和解锁状态 |
| `/fmu/out/vehicle_local_position` | `px4_msgs/msg/VehicleLocalPosition` | local NED 和 WGS84 参考原点 |
| `/fmu/out/vehicle_odometry` | `px4_msgs/msg/VehicleOdometry` | 无人机姿态 |
| `/deck/gps/fix` | `sensor_msgs/msg/NavSatFix` | 船舶 WGS84 位置 |
| `/deck/gps/velocity` | `geometry_msgs/msg/TwistStamped` | 船舶 ENU 速度 |
| `/aruco/pose` | `geometry_msgs/msg/PoseStamped` | ArUco 相机光学位姿 |
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
| `/landing/deck_gnss_pose_ned` | `geometry_msgs/msg/PoseStamped` | 船舶 GNSS 转换后的 local NED |
| `/landing/guidance_source` | `std_msgs/msg/String` | 当前引导来源和子状态 |

调试：

```bash
ros2 topic echo /landing/state
ros2 topic echo /landing/guidance_source
ros2 topic echo /landing/target_pose
ros2 topic echo /landing/deck_gnss_pose_ned
```

## P2C 状态机

```text
INIT
→ WAIT_FOR_PX4
→ OFFBOARD_PRE_STREAM
→ ARM_AND_TAKEOFF
→ WAIT_DECK_GNSS
→ RENDEZVOUS_GNSS
→ ACQUIRE_ARUCO
```

### `WAIT_DECK_GNSS`

- 锁定进入状态时的水平位置。
- 目标高度为 `rendezvous_altitude_m`。
- 等待 PX4 地理参考、船舶位置和速度连续稳定。

### `RENDEZVOUS_GNSS`

- 将船舶 WGS84 转为以 PX4 `ref_lat/ref_lon/ref_alt` 为原点的 local NED。
- 水平目标跟随实时船舶位置。
- 使用最大速度和单周期目标步长双重限幅。
- 到达会合半径和安全高度后进入 ArUco 捕获。

### `ACQUIRE_ARUCO`

没有稳定视觉时，搜索点相对实时船舶中心循环：

```text
中心 → 北 → 东 → 南 → 西
```

稳定识别达到 `aruco_acquire_duration_s` 后：

- 停止搜索偏移。
- 返回船舶 GNSS 中心。
- 保持 `rendezvous_altitude_m`。
- 不使用 ArUco 位置控制，不下降。

### GNSS 丢失

在 `RENDEZVOUS_GNSS` 或 `ACQUIRE_ARUCO` 中，如果位置或速度超时：

```text
→ WAIT_DECK_GNSS
```

进入等待时立即锁定当前无人机水平位置，不继续追踪旧船舶位置。

## 坐标与时间约定

- PX4 setpoint：`local_ned`，North、East、Down。
- 船舶 GNSS 速度输入：`world_enu`，East、North、Up。
- 地理转换参考：PX4 `VehicleLocalPosition.ref_lat/ref_lon/ref_alt`。
- 普通 GNSS 高度不用于控制会合高度。
- 会合高度始终使用 PX4 local NED 的固定安全高度。

当前 ROS `/clock` 与 PX4 时间域尚未统一，P2C 的 GNSS 新鲜度使用控制器回调到达时间。
消息原始采样时间仍保留在输入消息中，后续 P2D 再统一跨传感器时间对齐。

## 主要参数

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `control_rate_hz` | `20.0` | 控制和 setpoint 发布频率 |
| `takeoff_alt` | `3.0` | 起飞高度，米 |
| `rendezvous_altitude_m` | `5.0` | GNSS 会合和搜索安全高度，米 |
| `rendezvous_radius_m` | `2.0` | 进入 ArUco 捕获的水平半径，米 |
| `gnss_fix_timeout_s` | `1.0` | 船舶位置超时 |
| `gnss_velocity_timeout_s` | `1.0` | 船舶速度超时 |
| `gnss_stable_duration_s` | `1.0` | 位置和速度连续稳定时间 |
| `max_gnss_jump_m` | `5.0` | 新鲜 GNSS 水平跳变阈值 |
| `max_rendezvous_speed_mps` | `2.0` | 水平目标最大移动速度 |
| `max_target_step_m` | `0.20` | 单周期水平目标最大步长 |
| `search_offset_m` | `1.0` | ArUco 搜索偏移 |
| `search_point_hold_s` | `1.0` | 每个搜索点保持时间 |
| `aruco_acquire_duration_s` | `0.5` | 稳定视觉持续时间 |
| `gnss_max_geodetic_range_m` | `10000.0` | 地理转换允许的局部范围 |
| `deck_gnss_velocity_frame_id` | `world_enu` | 允许的船舶速度 frame |
| `enable_auto_land` | `false` | P2C 默认禁止自动降落 |

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

`gnss_rendezvous_guidance_test` 覆盖：

- PX4 地理参考缺失。
- WGS84 / ENU / NED 转换。
- 位置和速度独立稳定窗口。
- GNSS 跳变拒绝和超时后重新建立基准。
- 参考原点变化清空旧状态。
- 目标速度与步长限幅。
- 移动中心搜索序列。
- NaN、Inf 和非法参数。

## 安全提示

节点会自动发送 Offboard 和 Arm 命令，只能在 SITL 或具备完整安全措施的环境中运行。
P2C 默认 `enable_auto_land=false`，且主状态路径不会进入下降状态。实机测试前必须重新完成
消息版本、坐标方向、GNSS 质量、failsafe 和人工接管验证。
