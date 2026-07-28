# aruco_detector

`aruco_detector` 是一个 ROS 2 Humble C++ ArUco 标记检测节点，用于从相机图像中识别指定 ID 的 ArUco marker，并发布该 marker 相对于相机坐标系的位姿。

当前默认配置面向 PX4 Gazebo `aruco` world 中的 `gz_x500_mono_cam_down` 下视单目相机。

## 运行系统环境

推荐 / 已验证的运行环境如下：

| 项目 | 要求 |
| --- | --- |
| 操作系统 | Ubuntu 22.04 |
| ROS 版本 | ROS 2 Humble |
| 构建工具 | `colcon`、`ament_cmake` |
| 图像处理库 | OpenCV 4.x，需包含 `aruco` 模块 |
| 仿真环境 | PX4 SITL + Gazebo `aruco` world |
| ROS-Gazebo 桥接 | `ros_gz_bridge`，用于桥接相机图像和相机内参 |
| 调试工具 | 可选安装 `rqt_image_view` 查看调试图像 |

运行节点前，需要先启动 PX4、Gazebo 和 `ros_gz_bridge`，并确保 ROS 2 中可以收到相机图像和 `CameraInfo` 话题。

## 环境准备

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions libopencv-dev ros-humble-rqt-image-view
rosdep install --from-paths src --ignore-src -r -y
```

如果系统中还没有初始化过 `rosdep`，需要先执行：

```bash
sudo rosdep init
rosdep update
```

## 构建

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select aruco_detector
source install/setup.bash
```

## 运行

先启动 PX4、Gazebo 和 `ros_gz_bridge` 相机桥接，然后运行：

```bash
ros2 launch aruco_detector aruco_detector.launch.py
```

默认参数文件为：

```text
src/aruco_detector/config/aruco_detector.yaml
```

也可以通过 `config_file` 指定其他参数文件：

```bash
ros2 launch aruco_detector aruco_detector.launch.py config_file:=/path/to/aruco_detector.yaml
```

## USB 摄像头实机测试

如果只想用本机 USB 摄像头快速验证检测链路，可以不修改当前默认配置，通过运行时参数覆盖输入话题。

先在另一台电脑上全屏显示 ArUco marker：

- 字典：`DICT_4X4_50`
- ID：`0`
- `marker_length`：屏幕上黑色 marker 外边框的实际边长，单位为米

例如黑色 marker 外边框量到 `12 cm`，后续命令中使用 `marker_length:=0.12`。注意屏幕反光、过曝和摩尔纹可能影响识别。

启动 USB 摄像头图像发布。这里使用 ROS 2 自带的 `image_tools cam2image`，假设摄像头设备编号为 `0`：

```bash
source /opt/ros/humble/setup.bash

ROS_LOG_DIR=/tmp/ros_logs ros2 run image_tools cam2image --ros-args \
  -r image:=/usb_cam/image_raw \
  -p device_id:=0 \
  -p width:=640 \
  -p height:=480 \
  -p reliability:=best_effort \
  -p show_camera:=false
```

`cam2image` 只发布图像，不发布相机内参。为了快速验证检测和 `/aruco/pose` 链路，可以临时发布一个近似 `CameraInfo`：

```bash
source /opt/ros/humble/setup.bash

ROS_LOG_DIR=/tmp/ros_logs python3 - <<'PY'
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo


class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__("usb_cam_temporary_camera_info")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.pub = self.create_publisher(CameraInfo, "/usb_cam/camera_info", qos)
        self.timer = self.create_timer(1.0 / 30.0, self.publish_info)

    def publish_info(self):
        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"
        msg.height = 480
        msg.width = 640
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [600.0, 0.0, 320.0,
                 0.0, 600.0, 240.0,
                 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
        msg.p = [600.0, 0.0, 320.0, 0.0,
                 0.0, 600.0, 240.0, 0.0,
                 0.0, 0.0, 1.0, 0.0]
        self.pub.publish(msg)


rclpy.init()
node = CameraInfoPublisher()
rclpy.spin(node)
PY
```

再启动检测节点，覆盖输入图像和内参话题：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ROS_LOG_DIR=/tmp/ros_logs ros2 run aruco_detector aruco_detector_node --ros-args \
  -p image_topic:=/usb_cam/image_raw \
  -p camera_info_topic:=/usb_cam/camera_info \
  -p dictionary:=DICT_4X4_50 \
  -p target_id:=0 \
  -p marker_length:=0.12
```

其中 `marker_length:=0.12` 需要按实际显示尺寸修改。

验证图像和检测结果：

```bash
source /opt/ros/humble/setup.bash

ROS_LOG_DIR=/tmp/ros_logs rqt_image_view
ROS_LOG_DIR=/tmp/ros_logs ros2 topic echo /aruco/visible
ROS_LOG_DIR=/tmp/ros_logs ros2 topic echo /aruco/pose
```

在 `rqt_image_view` 中选择 `/aruco/debug_image`。识别成功时，调试图像会显示 marker 边框和坐标轴，`/aruco/visible` 会输出 `true`。

当前发布的位姿使用相机光学坐标系：

- `x`：图像向右为正
- `y`：图像向下为正
- `z`：摄像头前方为正

因此 marker 在图像左上角时，`x` 和 `y` 通常都是负值。临时 `CameraInfo` 只适合验证链路，精确位姿需要对 USB 摄像头做正式标定。

## 默认输入

默认订阅的话题来自 `config/aruco_detector.yaml`：

```text
/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image
/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info
```

其中：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `image_topic` | `/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image` | 输入图像话题 |
| `camera_info_topic` | `/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info` | 相机内参话题 |
| `marker_length` | `0.5` | 旧单 Marker 模式边长，单位为米 |
| `dictionary` | `DICT_4X4_50` | ArUco 字典 |
| `target_id` | `0` | 旧单 Marker 模式目标 ID |
| `marker_ids` | `[0, 1, 2, 3]` | 多尺度 Marker ID，顺序与其他数组参数一致 |
| `marker_lengths_m` | `[0.50, 0.20, 0.04, 0.02]` | 各 Marker 物理边长，初次捕获和挑战者排序优先使用更大边长 |
| `marker_priorities` | `[3, 2, 1, 0]` | 仅在物理边长相同时用于确定性排序 |
| `marker_min_switch_areas_px2` | `[400, 400, 400, 400]` | 各 Marker 成为新 active 的最小角点面积 |
| `active_hold_area_ratio` | `0.60` | active 保持面积相对进入门限的比例 |
| `minimum_border_margin_px` | `12.0` | 挑战者进入门限及 active 靠近边界判定距离 |
| `switch_required_consecutive_frames` | `5` | 同一挑战者完成切换前需要连续可靠的帧数 |
| `active_missing_grace_frames` | `2` | 短时漏检时仅保留内部 active 状态的帧数 |
| `sync_queue_size` | `10` | 图像和相机内参同步队列长度 |

多尺度模式采用有状态选择：当前 active Marker 面积与边界质量可靠时始终保持；只有 active 接近边界、面积不足或丢失后，满足进入门限的挑战者才开始累计，并在连续稳定达到配置帧数后切换。初次捕获优先选择物理边长最大的可靠 Marker。单 Marker 模式仍接受任意有限正面积检测，不受多尺度切换门限影响。

`active_missing_grace_frames` 只保留选择器内部状态。漏检帧不会复用上一帧位姿，`/aruco/visible` 仍为 `false`，`/aruco/id` 也不会发布陈旧 ID。

## 输出话题

节点会发布以下话题：

```text
/aruco/pose                       geometry_msgs/msg/PoseStamped
/aruco/id                         std_msgs/msg/Int32
/aruco/visible                    std_msgs/msg/Bool
/aruco/active_marker_id           std_msgs/msg/Int32
/aruco/selected_corner_area_px2   std_msgs/msg/Float64
/aruco/selected_border_margin_px  std_msgs/msg/Float64
/aruco/selection_reason           std_msgs/msg/String
/aruco/debug_image                sensor_msgs/msg/Image
```

说明：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/aruco/pose` | `geometry_msgs/msg/PoseStamped` | 本帧选中 Marker 经偏移补偿后的统一目标位姿 |
| `/aruco/id` | `std_msgs/msg/Int32` | 与本帧有效 `/aruco/pose` 对应的 Marker ID |
| `/aruco/visible` | `std_msgs/msg/Bool` | 本帧是否存在有效位姿 |
| `/aruco/active_marker_id` | `std_msgs/msg/Int32` | 选择器内部 active ID；无 active 时为 `-1` |
| `/aruco/selected_corner_area_px2` | `std_msgs/msg/Float64` | 本帧 selected Marker 角点面积；无 selected 时为 `NaN` |
| `/aruco/selected_border_margin_px` | `std_msgs/msg/Float64` | 本帧 selected Marker 最小边界距离；无 selected 时为 `NaN` |
| `/aruco/selection_reason` | `std_msgs/msg/String` | 每帧稳定选择原因，如 `HOLD_ACTIVE`、`CHALLENGER_STABILIZING`、`SWITCH_STABLE` |
| `/aruco/debug_image` | `sensor_msgs/msg/Image` | 绘制检测框、坐标轴和选择器状态后的调试图像 |

当没有本帧有效 selected Marker、相机内参无效或位姿估计失败时，节点会发布 `/aruco/visible = false`，并且不会发布新的 `/aruco/pose` 或 `/aruco/id`。诊断话题仍会逐帧发布，便于区分 active 状态、挑战者累计和真实位姿可用性。

## 验证

查看目标 marker 是否可见：

```bash
ros2 topic echo /aruco/visible
```

查看目标 marker 位姿：

```bash
ros2 topic echo /aruco/pose
```

查看调试图像：

```bash
rqt_image_view
```

在 `rqt_image_view` 中选择 `/aruco/debug_image`，可以检查图像中是否绘制出 marker 边框和位姿坐标轴。

## 常见排查

如果没有检测结果，优先检查：

- PX4、Gazebo 和 `ros_gz_bridge` 是否已经启动。
- ROS 2 中是否存在默认图像和 `CameraInfo` 话题。
- marker 是否在相机画面内，且没有严重模糊、遮挡或过曝。
- `dictionary` 是否与实际 marker 使用的字典一致。
- `target_id` 是否与实际 marker ID 一致。
- `marker_length` 是否与实际 marker 边长一致，单位是否为米。
- `CameraInfo` 中的相机内参是否有效。

可以先列出当前话题确认桥接是否正常：

```bash
ros2 topic list
```

更多检测流程和原理说明见 `ARUCO_DETECTION_FLOW.md`。
