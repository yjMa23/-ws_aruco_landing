# 坐标系与变换契约

## 1. 文档目的

本文档冻结 `ws_aruco_landing` 后续 GNSS 会合、ArUco 视觉接管、移动目标跟踪和下降控制所使用的坐标系、四元数、地理原点与变换方向。

后续实现必须以本文档为准。任何坐标变换不得继续依赖散落在业务代码中的正负号经验映射。

本文档依据当前本地环境核对：

- PX4 SITL：`~/PX4-Autopilot`
- `px4_msgs`：`~/ws_sensor_combined/src/px4_msgs`
- 下视相机模型：`Tools/simulation/gz/models/x500_mono_cam_down/model.sdf`
- 相机基础模型：`Tools/simulation/gz/models/mono_cam/model.sdf`
- PX4 Gazebo 桥：`src/modules/simulation/gz_bridge/GZBridge.cpp`
- 当前移动甲板 world：`src/moving_deck_sim/worlds/aruco_moving_deck.sdf`

核对日期：2026-07-20。

---

## 2. 统一变换记号

统一使用：

```text
T_A_B
```

表示将 B 坐标系中的点转换到 A 坐标系：

```text
p_A = T_A_B * p_B
```

刚体变换展开为：

```text
p_A = R_A_B * p_B + t_A_B
```

其中：

- `R_A_B`：B 坐标系到 A 坐标系的旋转矩阵。
- `t_A_B`：B 坐标系原点在 A 坐标系中的位置。
- 所有旋转矩阵均为右手系正交矩阵。
- 所有位置单位均为米。
- 所有角度在代码中均使用弧度。

Marker 完整变换链固定为：

```text
T_local_ned_marker
=
T_local_ned_body_frd
*
T_body_frd_camera_optical
*
T_camera_optical_marker
```

禁止将该公式的任一变换方向静默求逆后继续沿用原变量名。

---

## 3. 坐标系定义

### 3.1 `world_enu`

Gazebo world 坐标系：

```text
+x：East
+y：North
+z：Up
```

当前 world 明确设置：

```xml
<world_frame_orientation>ENU</world_frame_orientation>
```

当前 `/simulation/deck/ground_truth` 的位置和速度均使用该坐标系。

### 3.2 `base_link_flu`

Gazebo 中 x500 的机体坐标：

```text
+x：Forward
+y：Left
+z：Up
```

PX4 `GZBridge.cpp` 明确将 Gazebo 机体数据按 `FLU → FRD` 转换后再输入 PX4。

该坐标系仅用于解释 Gazebo 模型和传感器安装，不作为降落控制器的公共业务坐标系。

### 3.3 `base_link_frd`

PX4 机体坐标：

```text
+x：Forward
+y：Right
+z：Down
```

`base_link_flu` 到 `base_link_frd` 的向量变换为：

```text
x_frd =  x_flu
y_frd = -y_flu
z_frd = -z_flu
```

矩阵形式：

```text
R_frd_flu = diag(1, -1, -1)
```

### 3.4 `local_ned`

PX4 本地导航坐标：

```text
+x：North
+y：East
+z：Down
```

Gazebo ENU 到 PX4 NED 的位置变换为：

```text
x_ned =  y_enu
y_ned =  x_enu
z_ned = -z_enu
```

矩阵形式：

```text
R_ned_enu =
[0  1  0]
[1  0  0]
[0  0 -1]
```

PX4 `GZBridge.cpp` 对位置使用的正是该映射。

### 3.5 `camera_link`

Gazebo 相机 link 坐标遵循 Gazebo 相机约定：

```text
+x：相机朝向前方
+y：左
+z：上
```

Gazebo / SDFormat 文档明确指出 Gazebo 相机看向 `+X`，而 ROS 光学坐标看向 `+Z`。

`camera_link` 不是 OpenCV PnP 数值的直接坐标语义。

### 3.6 `camera_optical`

ROS / OpenCV 光学坐标：

```text
+x：图像向右
+y：图像向下
+z：镜头前方
```

`sensor_msgs/Image` 与 `CameraInfo` 的消息约定要求 `header.frame_id` 指向该光学坐标。

当前 ArUco PnP 的 `rvec/tvec` 数值语义始终是 `camera_optical`，不取决于字符串 frame 名称。

### 3.7 `marker`

Marker 自身坐标由 OpenCV ArUco 位姿估计定义。

当前控制器近期只需要 Marker 原点位置；后续甲板姿态估计使用其完整旋转。

### 3.8 `WGS84`

船舶 GNSS 和 PX4 全局定位使用：

```text
latitude：纬度，degree
longitude：经度，degree
altitude：海拔，m
```

地理坐标转换模块内部不得将经纬度直接当作平面米制坐标使用。

---

## 4. PX4 `VehicleOdometry` 契约

当前 `px4_msgs/msg/VehicleOdometry.msg` 定义：

```text
q[0] = w
q[1] = x
q[2] = y
q[3] = z
```

四元数采用 Hamilton 约定。

当前 PX4 Gazebo 桥对仿真机体里程计设置：

```text
pose_frame = POSE_FRAME_NED
q = q_FRD_to_NED
```

因此在本项目中，只有满足以下条件才接受该姿态：

```text
vehicle_odometry.pose_frame == POSE_FRAME_NED
```

数值使用方式固定为：

```text
p_local_ned
=
position_local_ned_body
+
R_local_ned_body_frd * p_body_frd
```

也就是说，构造出的 `R_local_ned_body_frd` 将 FRD 机体系向量旋转到 local NED。

实现要求：

1. 先检查 `pose_frame`。
2. 按 `[w,x,y,z]` 读取。
3. 检查四个分量有限。
4. 检查范数大于最小阈值。
5. 对可恢复的非单位四元数归一化。
6. 零四元数、NaN、Inf 直接返回失败。
7. 使用 `0°、90°、180°、-90°` 偏航单元测试验证方向。

不得继续只提取 yaw 后忽略 roll 和 pitch 完成主要坐标变换。

---

## 5. ArUco 位姿契约

`aruco_detector` 使用 OpenCV PnP，发布：

```text
T_camera_optical_marker
```

其位置分量：

```text
pose.position.x：Marker 在图像右方向的偏移
pose.position.y：Marker 在图像下方向的偏移
pose.position.z：Marker 沿相机视线方向的距离
```

其旋转分量表示 Marker 坐标到相机光学坐标的旋转。

当前检测器直接复用输入图像 header。当前 SDF 设置：

```xml
<gz_frame_id>camera_link</gz_frame_id>
```

但 PnP 数值语义仍是 ROS / OpenCV 光学坐标。

因此现阶段必须遵守：

- 业务逻辑按 `camera_optical` 解释 `/aruco/pose` 数值。
- 不得因为 `header.frame_id == camera_link` 就按 Gazebo `+X` 前向 link 坐标解释。
- 后续可将相机模型补充独立 `optical_frame_id`，或由检测节点发布明确的光学 frame 名；在完成该修改前，本文档是语义来源。
- 接入控制器时必须校验允许的 frame 名，发现未知名称应拒绝而不是猜测。

---

## 6. 当前下视相机外参

### 6.1 本地模型定义

`x500_mono_cam_down/model.sdf` 将 `mono_cam` 合入 x500：

```xml
<pose>0 0 .10 0 1.5707 0</pose>
```

含义为相机 link 名义上相对 Gazebo x500 `base_link_flu`：

```text
translation_flu = [0.0, 0.0, +0.10] m
rotation = pitch +1.5707 rad
```

`mono_cam/model.sdf` 中相机传感器相对 `camera_link` 的 pose 为单位变换，Gazebo 相机沿 link `+X` 看向前方。

`pitch +90°` 将相机 link 的 `+X` 转到机体 FLU 的 `-Z`，因此相机朝下。

### 6.2 `T_body_frd_camera_optical` 名义值

将 Gazebo相机 link 约定转换为 ROS 光学约定，再将机体 FLU 转为 FRD，得到名义变换：

```text
t_body_frd_camera_optical = [0.0, 0.0, -0.10] m
```

名义旋转：

```text
R_body_frd_camera_optical =
[0 -1  0]
[1  0  0]
[0  0  1]
```

其含义：

```text
body_forward = -camera_optical_y
body_right   =  camera_optical_x
body_down    =  camera_optical_z
```

对应名义四元数：

```text
q_body_frd_camera_optical [w,x,y,z]
≈ [0.70710678, 0.0, 0.0, 0.70710678]
```

当前旧控制器中的两个符号参数：

```text
camera_x_to_body_y_sign = +1
camera_y_to_body_x_sign = -1
```

恰好对应上述旋转的水平部分，因此旧静态基线方向不是随机凑出的；但它仍然缺少：

- `0.10 m` 外参平移。
- 相机完整三维旋转。
- 无人机 roll / pitch。
- Marker 高度和姿态。
- 四元数与异常输入检查。

### 6.3 参数设计

P2A 数学模块接入控制器时使用显式参数：

```yaml
camera_extrinsic.translation_frd_m: [0.0, 0.0, 0.14]
camera_extrinsic.rotation_wxyz: [0.70710678, 0.0, 0.0, 0.70710678]
```

参数语义固定为：

```text
T_body_frd_camera_optical
```

即将 `camera_optical` 点变换到 `body_frd`。

禁止使用含糊名称：

```text
camera_pose
camera_rotation
extrinsic
```

如果名称没有明确变换方向，不得加入公共参数。

第一版参数应来自 YAML，不硬编码本机 PX4 模型绝对路径。

模型升级或换相机后必须重新核对外参。

---

## 7. 完整视觉变换

输入：

```text
T_local_ned_body_frd      来自 VehicleOdometry position + q
T_body_frd_camera_optical 来自 YAML 外参
T_camera_optical_marker   来自 ArUco PoseStamped
```

输出：

```text
T_local_ned_marker
```

公式：

```text
T_local_ned_marker
=
T_local_ned_body_frd
*
T_body_frd_camera_optical
*
T_camera_optical_marker
```

位置展开：

```text
p_local_ned_marker
=
p_local_ned_body
+
R_local_ned_body_frd
*
(
  t_body_frd_camera_optical
  +
  R_body_frd_camera_optical * p_camera_optical_marker
)
```

控制器中的水平误差应由同一坐标系位置相减得到：

```text
error_ned_xy
=
p_local_ned_marker_xy
-
p_local_ned_body_xy
```

禁止再从相机 x/y 直接构造 NED 误差。

---

## 8. Gazebo spherical origin 与 PX4 local origin

当前移动甲板 world 设置：

```text
latitude  = 47.397971057728974 deg
longitude = 8.546163739800146 deg
elevation = 0 m
```

当前启动命令将 PX4 模型放在：

```text
PX4_GZ_MODEL_POSE=-4,0,0.2
```

注意以下两个原点不是同一个概念：

### 8.1 Gazebo spherical origin

定义 `world_enu = [0,0,0]` 对应的 WGS84 原点。

### 8.2 PX4 local NED origin

PX4 `GZBridge::navSatCallback()` 在收到第一帧无人机 NavSat 后初始化本地地理参考：

```text
ref_lat = first_uav_gps_lat
ref_lon = first_uav_gps_lon
ref_alt = first_uav_gps_alt
```

因此 PX4 local NED 原点对应无人机启动时的首个 GPS 参考位置，而不是无条件等于 Gazebo world 原点。

当前无人机启动时 ENU 水平位置为 `(-4,0)`，所以在默认启动配置下：

- PX4 local origin 相对 Gazebo spherical origin 有明确水平偏移，评测 Ground Truth
  必须先转换到 PX4 local NED，不能直接交换 ENU/NED 分量。
- PX4 `ref_alt` 对应无人机初始海拔，包含约 `0.2 m` 的启动高度。
- Gazebo `world_enu.z = 0` 与 PX4 `local_ned.z = 0` 不能直接视为同一垂直原点。

如果后续改变 `PX4_GZ_MODEL_POSE` 的 x/y，或者设置 `PX4_HOME_LAT/LON/ALT`，上述水平重合关系也会改变。

---

## 9. 船舶 GNSS 到 PX4 local NED

后续 `deck_gnss_simulator` 将甲板 `world_enu` Ground Truth 转为 WGS84，并只发布经过传感器模型处理后的：

```text
/deck/gps/fix
/deck/gps/velocity
```

控制器禁止订阅 `/simulation/deck/ground_truth`。

控制器将船舶 WGS84 转为 local NED 时，参考原点必须来自 PX4：

```text
/fmu/out/vehicle_local_position
```

且要求：

```text
xy_global == true
z_global == true（仅在使用 GNSS 高度时要求）
ref_lat/ref_lon/ref_alt 有限
```

转换链：

```text
deck WGS84
→ 以 PX4 ref_lat/ref_lon/ref_alt 为原点的 local ENU
→ local NED
```

ENU 到 NED：

```text
N = ENU.y
E = ENU.x
D = -ENU.z
```

第一版 GNSS 会合只使用水平 `N/E`。

普通 GNSS 高度不参与最终精降，也不直接驱动低高度横向接管。

---

## 10. 地理转换有效范围

第一版 `geodetic_converter` 用于局部会合，不用于跨区域导航。

设计范围：

```text
距离参考点不超过数千米
高度差远小于地球半径
```

实现必须采用可靠的 WGS84 局部切平面转换，或明确误差界限的局部近似。

至少测试：

- 原点转换为零。
- 正东位移得到 ENU x 正。
- 正北位移得到 ENU y 正。
- 升高得到 ENU z 正。
- ENU/NED 符号关系。
- 正向与逆向闭环误差。

不得用固定“每度约多少米”同时处理所有纬度和经度。

---

## 11. 时间戳契约

P4.5 已实现图像与 PX4 位姿的跨时间域对齐。实现仍必须严格区分：

- 图像采样时间。
- ArUco 位姿采样时间。
- PX4 `timestamp_sample`。
- ROS 回调到达时间。
- 控制循环时间。

坐标模块本身只处理同一时刻的几何量，不自行猜测时间对齐。

当前控制器使用 `VehicleOdometry.timestamp` 估计 PX4→ROS 时钟偏移，使用
`timestamp_sample` 得到机体位姿采样时刻，并在 `VehiclePoseHistory` 中对位置执行
线性插值、对姿态执行四元数 Slerp。ArUco 坐标变换只允许使用图像采样时刻对应的
机体位姿；时间戳为零、历史不足或超出端点保持范围时拒绝视觉帧。

---

## 12. P2A 数学模块接口约束

计划新增：

```text
coordinate_transform.hpp/.cpp
geodetic_converter.hpp/.cpp
```

纯数学模块要求：

- 不依赖 ROS Node。
- 不读取参数服务器。
- 不发布话题。
- 输入结构明确包含坐标语义。
- 无效输入返回 `std::nullopt` 或显式失败结果。
- 不抛出难以恢复的运行时异常处理普通无效传感器数据。

建议基本类型：

```text
Eigen::Vector3d
Eigen::Quaterniond
Eigen::Isometry3d
```

建议公开接口：

```text
make_isometry(translation, quaternion)
transform_marker_to_local_ned(...)
enu_to_ned(...)
ned_to_enu(...)
wgs84_to_local_enu(...)
local_enu_to_wgs84(...)
```

所有公开函数 Doxygen 必须注明：

- 输入坐标系。
- 输出坐标系。
- 单位。
- 四元数顺序。
- 变换方向。
- 失败条件。

---

## 13. 必须覆盖的单元测试

### 13.1 刚体变换

- 三个位姿均为单位变换。
- 相机只有平移。
- 相机只有固定旋转。
- 下视相机名义外参。
- 无人机 yaw：`0°、90°、180°、-90°`。
- 无人机 roll。
- 无人机 pitch。
- roll / pitch / yaw 组合。
- Marker 同时包含平移和旋转。

### 13.2 异常输入

- NaN 位置。
- Inf 位置。
- 零范数四元数。
- 极小范数四元数。
- 未归一化但可恢复的四元数。
- 错误 `pose_frame`。

### 13.3 地理转换

- 原点。
- 正东。
- 正北。
- 正上。
- ENU/NED。
- WGS84 往返闭环。
- 超出设计范围时的行为。

---

## 14. 运行接入验证清单

数学单元测试通过后，接入控制器时必须在 SITL 或消息级验收中记录以下运行值：

```bash
ros2 topic echo /fmu/out/vehicle_odometry --once
ros2 topic echo /fmu/out/vehicle_local_position --once
ros2 topic echo <camera_image_topic> --once
ros2 topic echo <camera_info_topic> --once
```

确认：

- `VehicleOdometry.pose_frame` 实际为 NED。
- `VehicleOdometry.q` 静止水平时接近预期。
- `VehicleLocalPosition.ref_lat/ref_lon/ref_alt` 有效。
- Image 与 CameraInfo frame 名一致。
- Marker 位于图像右侧时转换后为 body right。
- Marker 位于图像上侧时转换后为 body forward。
- 下视方向对应 body down。

P2D 已通过合成 PX4、GNSS 和 ArUco 消息完成该链路的消息级验证；真实相机插件与 PX4
动力学联合验证仍需在环境完整后执行。

---

## 15. 当前阶段结论

已确认：

1. Gazebo world 使用 ENU。
2. Gazebo x500 机体使用 FLU。
3. PX4 机体使用 FRD。
4. PX4 local position 使用 NED。
5. `VehicleOdometry.q` 顺序为 `[w,x,y,z]`。
6. 当前 Gazebo 桥输出 `q_FRD_to_NED`。
7. ArUco PnP 输出为 `camera_optical`。
8. 当前下视相机名义外参对应 `body_forward=-camera_y`、`body_right=camera_x`、`body_down=camera_z`，并包含约 `0.10 m` 垂直平移。
9. PX4 local 地理原点由无人机首帧 GPS 初始化，不能无条件直接使用 Gazebo world 原点。
10. GNSS 粗引导必须以 PX4 `ref_lat/ref_lon/ref_alt` 为转换参考。

纯 C++ 数学模块已经实现并通过测试，并已在 P2D 接入运行控制器。控制器现在使用完整
相机外参和 PX4 `VehicleOdometry` 生成 `/landing/marker_pose_ned`，不再由相机 x/y 的
手写正负号生成 P2D 主路径目标。

---

## 16. P2A 数学模块实现结果

已新增：

```text
src/aruco_precision_landing_cpp/include/aruco_precision_landing_cpp/coordinate_transform.hpp
src/aruco_precision_landing_cpp/src/coordinate_transform.cpp
src/aruco_precision_landing_cpp/test/coordinate_transform_test.cpp
src/aruco_precision_landing_cpp/include/aruco_precision_landing_cpp/geodetic_converter.hpp
src/aruco_precision_landing_cpp/src/geodetic_converter.cpp
src/aruco_precision_landing_cpp/test/geodetic_converter_test.cpp
```

实现内容：

- `Pose3d` 和带参考系检查的 Marker 完整刚体变换。
- 有限性、四元数范数检查和可恢复四元数归一化。
- ENU / NED 双向向量转换。
- 基于 WGS84 椭球、ECEF 中间坐标和局部切平面的 WGS84 / ENU 双向转换。
- 默认 `10 km` 局部三维有效范围，可在创建转换器时调整。
- 无效经纬度、NaN、Inf 和超范围输入显式返回失败。

P2A 完成时，刚体坐标测试覆盖 10 个 GTest 用例，地理转换测试覆盖 8 个 GTest 用例。
截至 P2D，完整工作区结果为：

```text
3 packages finished
55 tests
0 errors
0 failures
0 skipped
```

P2D 消息级验证中，输入：

```text
UAV local NED = [0.0, 0.0, -5.0] m
camera_optical Marker = [0.0, 0.0, 5.3] m
T_body_frd_camera_optical.translation = [0.0, 0.0, -0.10] m
```

得到：

```text
Marker local NED ≈ [0.0, 0.0, 0.2] m
```

结果与完整变换链一致。详细验收见：

```text
docs/P2D_GNSS_VISION_HANDOVER_VALIDATION.md
```

P4.5 已解决图像采样时刻与 PX4 位姿的时间对齐。详细实现与验收边界见：

```text
docs/P4_5_EXECUTION_PLAN.md
docs/P4_5_TIME_ALIGNMENT_VALIDATION.md
```
