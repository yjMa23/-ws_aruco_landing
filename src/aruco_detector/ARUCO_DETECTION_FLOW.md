# ArUco 识别流程与原理

本文档记录 `aruco_detector` 包中 ArUco 标记识别节点的处理流程、核心原理、参数含义、输入输出话题以及异常处理逻辑。当前实现位于 `src/aruco_detector/src/aruco_detector_node.cpp`。

## 1. 节点定位

`aruco_detector` 是一个 ROS 2 C++ 节点，用于从相机图像中识别指定 ID 的 ArUco marker，并估计该 marker 相对于相机坐标系的位姿。

节点默认面向 PX4 Gazebo `aruco` world 中的下视单目相机：

```text
/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image
/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info
```

节点输出：

```text
/aruco/pose         geometry_msgs/msg/PoseStamped
/aruco/visible      std_msgs/msg/Bool
/aruco/debug_image  sensor_msgs/msg/Image
```

## 2. 参数说明

参数默认值定义在 `config/aruco_detector.yaml` 中。

| 参数名 | 默认值 | 作用 |
| --- | --- | --- |
| `image_topic` | `/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image` | 输入图像话题 |
| `camera_info_topic` | `/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info` | 相机内参话题 |
| `marker_length` | `0.5` | ArUco marker 的真实边长，单位为米 |
| `dictionary` | `DICT_4X4_50` | ArUco 字典类型 |
| `target_id` | `0` | 需要跟踪和发布位姿的 marker ID |
| `sync_queue_size` | `10` | 图像和 CameraInfo 近似时间同步队列长度 |

节点启动时会检查部分参数：

- `marker_length <= 0.0` 时回退为 `0.5 m`。
- `sync_queue_size < 1` 时回退为 `10`。
- `dictionary` 不在支持列表中时回退为 `DICT_4X4_50`。

## 3. 整体识别流程

### 3.1 订阅并同步输入

节点同时订阅：

- `sensor_msgs/msg/Image`
- `sensor_msgs/msg/CameraInfo`

图像和相机内参使用 `message_filters::Synchronizer` 配合 `ApproximateTime` 策略做近似时间同步。这样可以避免图像帧和相机内参时间戳不完全一致时无法进入检测流程。

同步成功后，回调函数 `handleImage()` 会同时收到一帧图像和对应的 `CameraInfo`。

### 3.2 图像格式转换

回调中先使用 `cv_bridge::toCvCopy()` 将 ROS 图像消息转换为 OpenCV 图像，并指定编码为 `BGR8`。

随后将 BGR 图像转换成灰度图：

```text
BGR image -> grayscale image
```

ArUco 检测主要依赖 marker 的黑白二值图案和边界轮廓，因此灰度图足够用于角点检测和 ID 解码。

### 3.3 ArUco marker 检测

节点通过 OpenCV ArUco 模块执行检测：

```text
cv::aruco::detectMarkers(gray_image, dictionary, corners, ids, detector_params)
```

检测结果包括：

- `ids`：识别出的 marker ID 列表。
- `corners`：每个 marker 的四个角点在图像像素坐标系下的位置。

如果当前图像中检测到了 marker，节点会在调试图像上绘制 marker 边框和 ID。

### 3.4 筛选目标 ID

节点不会对所有检测到的 marker 都发布位姿，而是只查找参数 `target_id` 指定的 marker。

如果 `ids` 中不存在 `target_id`：

- 发布 `/aruco/visible = false`
- 发布当前调试图像
- 不发布新的 `/aruco/pose`

### 3.5 检查相机内参

位姿估计需要有效的相机内参。节点会检查 `CameraInfo.k` 中的关键元素：

```text
k[0] > 0
k[4] > 0
k[8] != 0
```

这些值对应相机内参矩阵中的焦距和齐次坐标项。若内参无效，节点会跳过位姿估计，并发布 `/aruco/visible = false`。

### 3.6 构造相机矩阵和畸变参数

当目标 marker 可见且 `CameraInfo` 有效时，节点从 `CameraInfo.k` 构造 3x3 相机内参矩阵：

```text
[ fx  0 cx ]
[  0 fy cy ]
[  0  0  1 ]
```

实际代码直接使用 `CameraInfo.k` 的九个元素生成矩阵，因此也兼容非零 skew 项。

节点还会从 `CameraInfo.d` 读取畸变参数：

- 如果 `CameraInfo.d` 为空，则使用全零畸变参数。
- 如果不为空，则将其转换为 OpenCV 所需的单行矩阵。

### 3.7 单个 marker 位姿估计

节点只取目标 marker 的四个角点，调用：

```text
cv::aruco::estimatePoseSingleMarkers(...)
```

输入包括：

- marker 四个角点的像素坐标
- marker 真实边长 `marker_length`
- 相机内参矩阵
- 相机畸变参数

输出包括：

- `rvec`：旋转向量
- `tvec`：平移向量

其中 `tvec` 表示 marker 中心在相机坐标系下的位置，单位与 `marker_length` 一致。当前默认 `marker_length` 使用米，因此 `/aruco/pose` 中的位置单位也是米。

`rvec` 表示 marker 坐标系相对于相机坐标系的旋转。代码使用 `cv::Rodrigues()` 将旋转向量转换为旋转矩阵，然后再转换为 ROS 消息中的四元数。

### 3.8 发布结果

位姿估计成功后，节点会发布三个结果：

1. `/aruco/pose`

   类型为 `geometry_msgs/msg/PoseStamped`。

   - `header` 复用输入图像的 `header`。
   - `position.x/y/z` 来自 `tvec[0/1/2]`。
   - `orientation` 来自 `rvec` 转换后的四元数。

2. `/aruco/visible`

   类型为 `std_msgs/msg/Bool`。

   - 目标 marker 成功检测并完成位姿估计时发布 `true`。
   - 未检测到目标 marker、相机内参无效或位姿估计失败时发布 `false`。

3. `/aruco/debug_image`

   类型为 `sensor_msgs/msg/Image`。

   调试图像包含：

   - 已检测 marker 的边框和 ID。
   - 目标 marker 位姿估计成功时绘制的坐标轴。

## 4. ArUco 识别原理

### 4.1 ArUco marker 与字典

ArUco marker 是一种黑白方形人工标记。每个 marker 内部包含一个二值编码图案，外侧通常有明显黑色边框，便于图像算法定位。

ArUco 字典定义了一组合法 marker 图案。比如 `DICT_4X4_50` 表示：

- 每个 marker 的内部编码区域是 4x4。
- 字典中包含 50 个不同 ID。

检测时必须使用与实际 marker 生成时一致的字典，否则可能无法识别，或者识别出错误 ID。

### 4.2 角点检测与 ID 解码

OpenCV ArUco 检测大致包含以下步骤：

1. 在灰度图中寻找候选方形轮廓。
2. 对候选区域做透视矫正，使 marker 图案变成正视方形。
3. 根据字典规则读取内部黑白格子。
4. 与字典中的合法编码匹配，得到 marker ID。
5. 返回 marker 在原图中的四个角点像素坐标。

四个角点是后续位姿估计的关键输入，因为它们建立了真实 marker 平面点与图像像素点之间的对应关系。

### 4.3 CameraInfo 的作用

仅靠图像中的角点位置，只能知道 marker 在图像平面上的投影。要恢复三维位姿，还需要相机模型。

`CameraInfo` 提供：

- 相机内参矩阵：描述三维相机坐标如何投影到二维像素坐标。
- 畸变参数：描述镜头畸变，帮助 OpenCV 修正角点投影误差。

如果内参无效，三维位姿估计没有可靠几何依据，因此当前节点会跳过 pose 发布。

### 4.4 PnP 位姿估计

`estimatePoseSingleMarkers()` 本质上解决的是 PnP 问题，即：

```text
已知 marker 四个角点在真实世界中的相对位置
已知这四个角点在图像中的像素位置
已知相机内参和畸变参数
求 marker 坐标系到相机坐标系的旋转和平移
```

因为 marker 是边长已知的正方形，所以 OpenCV 可以构造 marker 自身坐标系下的四个三维角点。结合图像中的四个二维角点后，即可估计出 `rvec` 和 `tvec`。

### 4.5 rvec、tvec 与 ROS Pose

OpenCV 输出的 `rvec` 和 `tvec` 描述 marker 坐标系到相机坐标系的变换：

- `tvec`：marker 中心在相机坐标系下的位置。
- `rvec`：marker 坐标系相对于相机坐标系的旋转。

当前节点将它们转换为 ROS `PoseStamped`：

```text
pose.position    = tvec
pose.orientation = quaternion(Rodrigues(rvec))
```

因此 `/aruco/pose` 表示目标 marker 在输入图像 `header.frame_id` 对应相机坐标系下的位姿。

## 5. 异常与降级处理

当前节点包含以下异常处理路径：

| 场景 | 处理方式 |
| --- | --- |
| 图像无法转换为 `BGR8` | 记录限频警告，发布 `visible=false`，结束本帧处理 |
| 未检测到任何 marker | 发布 `visible=false` 和调试图像 |
| 检测到 marker，但没有 `target_id` | 发布 `visible=false` 和调试图像 |
| `CameraInfo` 内参无效 | 记录限频警告，发布 `visible=false` 和调试图像 |
| 位姿估计结果为空 | 发布 `visible=false` 和调试图像 |
| 参数 `marker_length` 非正 | 启动时回退为 `0.5 m` |
| 参数 `sync_queue_size` 小于 1 | 启动时回退为 `10` |
| 参数 `dictionary` 未知 | 启动时回退为 `DICT_4X4_50` |

注意：当目标不可见或位姿估计失败时，节点不会发布新的 `/aruco/pose`。下游模块如果需要判断 pose 是否有效，应结合 `/aruco/visible` 使用。

## 6. 调试建议

可以通过以下命令观察检测状态：

```bash
ros2 topic echo /aruco/visible
ros2 topic echo /aruco/pose
```

也可以使用 `rqt_image_view` 查看 `/aruco/debug_image`：

```bash
rqt_image_view
```

如果图像中能看到 marker 边框但没有坐标轴，通常说明 marker 被检测到了，但目标 ID 不匹配、相机内参无效或位姿估计失败。

如果图像中完全没有 marker 边框，应优先检查：

- 相机图像话题是否正确。
- marker 是否在画面内且足够清晰。
- `dictionary` 是否与实际 marker 一致。
- 光照、模糊、遮挡是否影响检测。
