# aruco_precision_landing_cpp

`aruco_precision_landing_cpp` 是一个 ROS 2 Humble C++ 控制包。它通过 PX4
Offboard position setpoint 驱动多旋翼起飞、飞往 ArUco 搜索区域、水平对中、
带跟踪下降并切换到 PX4 自动降落。

控制节点只订阅 ArUco 位姿结果，不直接处理图像，也不依赖 Gazebo 专用话题。

## 依赖与环境

- Ubuntu 22.04、ROS 2 Humble
- PX4 SITL 与 Gazebo Harmonic
- PX4 uXRCE-DDS Agent
- 与运行中 PX4 版本匹配的 `px4_msgs`
- 已发布 `/aruco/pose` 和 `/aruco/visible` 的 ArUco 检测节点

本工作区中的 `aruco_detector` 当前不发布 `/aruco/id`。控制节点会订阅该话题并
记录最近的 ID，但不会因为没有收到 ID 而阻塞降落流程。

## 构建

先加载 ROS 2 和包含 `px4_msgs` 的 underlay，再构建本包：

```bash
cd ~/ws_aruco_landing
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash

colcon build --symlink-install --packages-select aruco_precision_landing_cpp
source install/setup.bash
```

如果 `px4_msgs` 位于其他工作区，请改为加载对应的 `install/setup.bash`。

## 仿真启动顺序

### 1. 启动 uXRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

### 2. 启动 PX4 SITL 与 ArUco world

```bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=aruco make px4_sitl gz_x500_mono_cam_down
```

确认 ROS 2 中已经出现 PX4 话题：

```bash
ros2 topic echo /fmu/out/vehicle_status_v4 --once
ros2 topic echo /fmu/out/vehicle_local_position_v1 --once
ros2 topic echo /fmu/out/vehicle_odometry --once
```

当前 PX4 v1.18 消息版本会在 `VehicleStatus` 和 `VehicleLocalPosition`
话题名后分别添加 `_v4` 和 `_v1`。本包 launch 文件默认已完成这两个 remap。

### 3. 启动相机桥接和 ArUco 检测

按 `aruco_detector` 包 README 启动 Gazebo 相机桥接，然后运行：

```bash
cd ~/ws_aruco_landing
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch aruco_detector aruco_detector.launch.py
```

开始控制前确认检测输出存在：

```bash
ros2 topic echo /aruco/visible
ros2 topic echo /aruco/pose
```

### 4. 启动精准降落控制器

```bash
cd ~/ws_aruco_landing
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash
source install/setup.bash

ros2 launch aruco_precision_landing_cpp px4_aruco_landing.launch.py
```

该命令等价于手动启动节点时加入：

```text
-r /fmu/out/vehicle_status:=/fmu/out/vehicle_status_v4
-r /fmu/out/vehicle_local_position:=/fmu/out/vehicle_local_position_v1
```

可以覆盖默认参数文件：

```bash
ros2 launch aruco_precision_landing_cpp px4_aruco_landing.launch.py \
  config_file:=/absolute/path/to/px4_aruco_landing.yaml
```

如果使用不带版本后缀的旧版 PX4/`px4_msgs`，可以覆盖输入话题：

```bash
ros2 launch aruco_precision_landing_cpp px4_aruco_landing.launch.py \
  vehicle_status_topic:=/fmu/out/vehicle_status \
  vehicle_local_position_topic:=/fmu/out/vehicle_local_position
```

## 状态与调试输出

状态机状态发布到：

```text
/landing/state  std_msgs/msg/String
```

当前发送给 PX4 的 local NED 目标点发布到：

```text
/landing/target_pose  geometry_msgs/msg/PoseStamped
```

查看状态变化：

```bash
ros2 topic echo /landing/state
ros2 topic echo /landing/target_pose
```

正常流程为：

```text
INIT
WAIT_FOR_PX4
OFFBOARD_PRE_STREAM
ARM_AND_TAKEOFF
GOTO_ARUCO_AREA
WAIT_ARUCO
CENTER_ABOVE_MARKER
DESCEND_WITH_TRACKING
FINAL_LAND
DONE
```

下降阶段丢失 marker 超过 `marker_lost_timeout` 后会进入 `ABORT`，锁定进入
ABORT 时的水平位置并飞到 `abort_hover_alt`。节点不会自动 disarm。

## 坐标约定

PX4 setpoint 使用 local NED：

- `x`：North
- `y`：East
- `z`：Down；3 米高度附近约为 `-3`

默认 Gazebo 下视相机映射：

- camera optical `x` → body right，符号由 `camera_x_to_body_y_sign` 控制
- camera optical `y` → body backward，因此
  `camera_y_to_body_x_sign` 默认为 `-1`

如果水平修正方向相反，应在 YAML 中调整这两个符号参数，不需要修改代码。

## 主要参数

默认配置位于 `config/px4_aruco_landing.yaml`。常用参数包括：

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `control_rate_hz` | `20.0` | 控制和 setpoint 发布频率 |
| `takeoff_alt` | `3.0` | 起飞高度，单位米 |
| `search_x`, `search_y` | `0.0`, `0.0` | ArUco 搜索区域 local NED 坐标 |
| `search_alt` | `3.0` | 搜索高度 |
| `offboard_prestream_count` | `20` | 切 Offboard 前有效 setpoint 数量 |
| `stable_detect_count` | `10` | 进入对中前连续可见帧数 |
| `max_xy_step` | `0.20` | 单周期最大水平目标修正 |
| `center_xy_threshold` | `0.15` | 开始下降的水平误差阈值 |
| `max_descent_rate` | `0.20` | 最大下降速率，单位 m/s |
| `final_alt` | `0.30` | 切换最终降落的高度 |
| `marker_lost_timeout` | `1.0` | 下降阶段 marker 丢失超时 |
| `enable_auto_land` | `true` | 是否发送 `VEHICLE_CMD_NAV_LAND` |

所有控制参数都可以通过 YAML 修改。参数非法时节点会拒绝启动并打印原因。

## 安全提示

该节点会自动发送 Offboard、Arm 和 Land 命令。仅在 SITL 或具备完整安全措施的
测试环境中运行。启动前确认 PX4 的估计器、解锁检查、failsafe 和通信链路正常。
