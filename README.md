# ws_aruco_landing

`ws_aruco_landing` 是一个面向 PX4 SITL + Gazebo + ROS 2 Humble 的 ArUco
精准降落工作空间。它包含 ArUco marker 检测节点和 PX4 Offboard 精准降落控制节点，
用于在仿真环境中完成无人机起飞、搜索 marker、水平对中、跟踪下降和最终降落流程。

根目录 README 只提供项目总览和快速启动入口。更完整的参数说明、调试方法和实机测试步骤请查看各包文档。

## 包结构

| 包 | 说明 |
| --- | --- |
| [`src/aruco_detector`](src/aruco_detector/README.md) | 从相机图像中检测指定 ArUco marker，并发布 `/aruco/pose`、`/aruco/visible` 和调试图像。 |
| [`src/aruco_precision_landing_cpp`](src/aruco_precision_landing_cpp/README.md) | 订阅 ArUco 位姿结果，通过 PX4 Offboard setpoint 控制无人机执行精准降落流程。 |
| [`src/moving_deck_sim`](src/moving_deck_sim/README.md) | 提供静止、水平匀速和水平正弦移动甲板，以及仅供评测使用的 Gazebo Ground Truth。 |

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- PX4 SITL 与 Gazebo Harmonic
- PX4 uXRCE-DDS Agent
- 与当前 PX4 版本匹配的 `px4_msgs`
- `colcon`、`ament_cmake`、OpenCV 4.x 与 `ros_gz_bridge`

## 快速构建

在工作空间根目录执行：

```bash
cd ~/ws_aruco_landing
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如果 `px4_msgs` 位于其他工作空间，请将 `~/ws_sensor_combined/install/setup.bash`
替换为对应 underlay 的 `install/setup.bash`。

## 仿真运行顺序

### 1. 启动 uXRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

### 2. 启动 PX4 SITL 与 ArUco world

```bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=aruco make px4_sitl gz_x500_mono_cam_down
```

### 3. 启动相机桥接和 ArUco 检测

先启动 Gazebo 相机到 ROS 2 的桥接，
```
source /opt/ros/humble/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  '/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
```
验证：
```
ros2 topic hz /world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image
```

再运行：

```bash
cd ~/ws_aruco_landing
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch aruco_detector aruco_detector.launch.py
```

确认检测输出存在：

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

常用状态和目标点话题：

```bash
ros2 topic echo /landing/state
ros2 topic echo /landing/guidance_source
ros2 topic echo /landing/target_pose
ros2 topic echo /landing/deck_gnss_pose_ned
```

## P1 水平移动甲板

水平移动甲板由独立仿真包启动，PX4 使用 standalone 模式连接。完整的环境准备、三种
场景配置、无界面运行和重置方法见：

- [moving_deck_sim README](src/moving_deck_sim/README.md)

移动甲板 world 仍命名为 `aruco`，因此现有相机桥接话题和 ArUco 检测器默认配置保持
不变。`/simulation/deck/ground_truth` 使用 Gazebo world ENU，禁止输入降落控制器。
仿真传感器节点会将其处理为 `/deck/gps/fix` 和 `/deck/gps/velocity`。控制器已在 P2C 中使用这些传感器输出完成安全高度 GNSS 会合、移动中心 ArUco 搜索和 GNSS 超时回退。当前稳定识别 ArUco 后仍保持 GNSS 中心悬停，不下降。

## 详细文档

- [下一阶段完整开发计划](docs/NEXT_DEVELOPMENT_PLAN.md)
- [坐标系与变换契约](docs/COORDINATE_FRAMES.md)
- [传统基线实施计划](docs/TRADITIONAL_BASELINE_PLAN.md)
- [ArUco 检测节点 README](src/aruco_detector/README.md)
- [PX4 ArUco 精准降落控制节点 README](src/aruco_precision_landing_cpp/README.md)
- [水平移动甲板仿真 README](src/moving_deck_sim/README.md)
- [P1 水平移动甲板仿真验收记录](docs/P1_MOVING_DECK_VALIDATION.md)
- [P2B 船舶 GNSS 传感器仿真验收记录](docs/P2B_DECK_GNSS_VALIDATION.md)
- [P2C GNSS 会合与移动搜索验收记录](docs/P2C_GNSS_RENDEZVOUS_VALIDATION.md)

## 安全提示

控制节点会自动发送 Offboard 和 Arm 命令。P2C 默认 `enable_auto_land=false`，当前主路径不会下降或发送 Land 命令。建议只在 SITL 中验证；实机测试前请确认 PX4 估计器、解锁检查、failsafe、通信链路和人工接管手段都已准备好。
