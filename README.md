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
ros2 topic echo /landing/target_pose
```

## 详细文档

- [ArUco 检测节点 README](src/aruco_detector/README.md)
- [PX4 ArUco 精准降落控制节点 README](src/aruco_precision_landing_cpp/README.md)

## 安全提示

精准降落控制节点会自动发送 Offboard、Arm 和 Land 命令。建议先在 SITL 中验证完整流程；
实机测试前请确认 PX4 估计器、解锁检查、failsafe、通信链路和人工接管手段都已准备好。
