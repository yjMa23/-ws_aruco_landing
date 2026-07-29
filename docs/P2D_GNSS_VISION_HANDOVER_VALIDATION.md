# P2D GNSS—视觉接管与下降前恢复验收记录

## 1. 验收范围

本记录验证：

```text
ArUco camera_optical 完整位姿
→ body_frd 相机外参
→ PX4 local NED
→ GNSS / 视觉一致性检查
→ VISUAL_HANDOVER
→ TRACK_TARGET
→ 视觉长时丢失后 RECOVER_TO_GNSS
```

本阶段只在固定安全高度运行，不验证下降、触地、视觉速度估计或运动预测。

## 2. 新增实现

```text
src/aruco_precision_landing_cpp/
├── include/aruco_precision_landing_cpp/
│   └── visual_handover_guidance.hpp
├── src/
│   └── visual_handover_guidance.cpp
└── test/
    └── visual_handover_guidance_test.cpp
```

控制节点新增状态：

```text
VISUAL_HANDOVER
TRACK_TARGET
RECOVER_TO_GNSS
```

新增调试话题：

```text
/landing/marker_pose_ned
```

## 3. 完整坐标变换

实现链：

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

输入校验包括：

- `VehicleOdometry.pose_frame == POSE_FRAME_NED`。
- PX4 和 ArUco 位置有限。
- 四元数有限且范数有效。
- ArUco `header.frame_id` 与配置一致。
- 非零采样时间戳严格递增。
- GNSS—视觉水平距离不超过阈值。
- 相邻视觉测量水平跳变不超过阈值。

## 4. 单元测试

执行：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash

colcon build --symlink-install \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
colcon test
colcon test-result --verbose
```

结果：

```text
3 packages finished
55 tests
0 errors
0 failures
0 skipped
```

其中：

```text
coordinate_transform_test        10 passed
geodetic_converter_test           8 passed
gnss_rendezvous_guidance_test     9 passed
visual_handover_guidance_test     8 passed
```

视觉接管纯逻辑测试覆盖：

- 乱序视觉测量拒绝。
- 相邻视觉测量大跳变拒绝。
- 长时丢失后的重新初始化。
- GNSS 与视觉水平一致性检查。
- 接管权重从 0 单调增加到 1。
- GNSS / 视觉线性混合。
- 短时丢失和长时丢失分类。
- 最大目标速度和单周期步长限幅。
- 非法参数、NaN、Inf 和非法 `dt`。

## 5. 合成消息状态机验收

该验收使用真实 ROS 2 控制节点，但 PX4、GNSS 和 ArUco 输入由合成消息提供，不驱动
PX4 飞行动力学。

输入配置：

```text
UAV local NED position = [0.0, 0.0, -5.0] m
UAV attitude = identity body_frd → local_ned
PX4 WGS84 reference altitude = 2.2 m
Deck GNSS altitude = 2.0 m
ArUco camera_optical position = [0.0, 0.0, 5.3] m
```

按默认外参计算：

```text
Marker local NED ≈ [0.0, 0.0, 0.2] m
```

实测 `/landing/marker_pose_ned`：

```text
position.x = 0.0
position.y = 0.0
position.z = 0.20000000000000018
orientation.z ≈ 0.70710678
orientation.w ≈ 0.70710678
```

位置和旋转均与完整刚体变换一致。

## 6. 正常接管路径

实测状态日志：

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

关键转换原因：

```text
ACQUIRE_ARUCO → VISUAL_HANDOVER
stable full-transform ArUco pose is consistent with deck GNSS

VISUAL_HANDOVER → TRACK_TARGET
GNSS-to-visual handover weight reached one
```

视觉跟踪阶段实测目标：

```text
/landing/target_pose
position = [0.0, 0.0, -5.0]
```

说明接管期间和接管完成后均保持固定安全高度，没有下降。

## 7. 单帧误检

测试只发布一次：

```text
/aruco/visible = true
/aruco/pose = one frame
```

随后等待超过单帧新鲜时间，但持续时间不足 `aruco_acquire_duration_s`。

实测最终状态：

```text
ACQUIRE_ARUCO
```

日志中不存在：

```text
ACQUIRE_ARUCO → VISUAL_HANDOVER
```

因此单帧误检不会触发视觉接管。

## 8. 视觉丢失恢复

在进入 `TRACK_TARGET` 后停止 ArUco 位姿与可见性发布。

短时丢失期间：

- 保持最近水平目标。
- 目标 z 继续保持 `-rendezvous_altitude_m`。
- 不进入下降或 Land。

超过长时丢失阈值后，实测日志：

```text
TRACK_TARGET → RECOVER_TO_GNSS
reason: visual pose was lost for the long timeout before descent

RECOVER_TO_GNSS → ACQUIRE_ARUCO
reason: GNSS recovery target is valid; resume coarse guidance
```

最终引导来源：

```text
GNSS_SEARCH
```

最终目标 z：

```text
-5.0 m
```

## 9. Ground Truth 与下降隔离

执行：

```bash
grep -R "/simulation/deck/ground_truth" \
  src/aruco_precision_landing_cpp src/aruco_detector
```

实测无输出。

当前从 `INIT` 出发的主路径只到达：

```text
TRACK_TARGET
RECOVER_TO_GNSS
ACQUIRE_ARUCO
RENDEZVOUS_GNSS
WAIT_DECK_GNSS
```

没有到达旧静态基线的：

```text
CENTER_ABOVE_MARKER
DESCEND_WITH_TRACKING
FINAL_LAND
```

默认：

```yaml
enable_auto_land: false
```

## 10. 时间边界

当前实现使用：

- 回调到达时间判断视觉新鲜度。
- 非零图像采样时间戳检查重复和乱序。
- 回调时最新 `VehicleOdometry` 完成刚体变换。

尚未实现：

- 图像采样时刻的 PX4 位姿插值。
- ROS `/clock`、相机时间与 PX4 时间域统一。
- 视觉滤波和延迟补偿。

这些内容进入 P3 状态估计与预测阶段。

## 11. 尚未完成的真实飞行验证

本记录不声明以下项目已经通过：

- PX4 SITL 实际动力学下的 GNSS—视觉接管。
- 匀速、正弦移动甲板上的真实视觉水平跟踪。
- 含噪、延迟和丢包 GNSS 下的实际接管稳定性。
- 相机图像延迟下的视觉跟踪误差。
- 真实相机插件输出下的接管成功率。

当前 Gazebo 环境仍存在：

```text
Failed to load system plugin libGstCameraSystem.so
```

该问题不影响纯逻辑和合成消息验收，但需要在真实视觉联合飞行前处理。

## 12. 阶段结论

P2D 代码和消息级验收通过：

- 完整相机外参和 PX4 姿态已接入。
- Marker 完整位姿可转换到 local NED。
- 单帧误检不会触发接管。
- GNSS 到视觉接管平滑且有目标限幅。
- 视觉跟踪全程保持安全高度。
- 视觉长时丢失能够恢复到 GNSS 粗引导。
- 控制器不使用 Ground Truth。
- 主路径不下降。

建议阶段标签：

```text
baseline-gnss-vision-handover-v0.1
```
