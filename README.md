# ws_aruco_landing

`ws_aruco_landing` 是一个基于 **PX4 SITL + Gazebo Harmonic + ROS 2 Humble** 的移动船舶无人机自主降落传统基线工作空间。

当前已经完成从船舶 GNSS 粗引导到视觉预测跟踪的完整安全高度链路：

```text
船舶 GNSS 会合
→ 移动中心搜索 ArUco
→ GNSS—视觉平滑接管
→ 甲板视觉状态估计
→ 短时运动预测
→ 预测位置 + 自适应水平速度前馈跟踪
→ 视觉甲板倾角估计
→ 规则式着陆窗口判断
→ 视觉丢失时恢复到 GNSS
```

当前阶段为 **P5C：低高度垂直状态估计与误差标定**。P5B 已完成静止、0.4 m/s 匀速和升沉甲板的相对高度分阶段下降，并在 `0.50 m` 安全测试高度停止；恢复爬升和恢复后重新授权锁止也已通过 PX4 SITL 验收。

> 相对下降默认关闭，只有显式传入 `--enable-relative-descent` 才允许下降到 `0.50 m`。当前不会触地、不会发送 `NAV_LAND`，也不会自动 Disarm。

---

## 1. 当前进度

| 阶段 | 状态 | 主要成果 | 标签 |
| --- | --- | --- | --- |
| P0 静态基线冻结 | 已完成 | 整理仓库，保留静态 ArUco 降落历史基线 | `baseline-static-v0.1` |
| P1 移动甲板仿真 | 已完成 | 静止、匀速、XY 正弦甲板，确定性 reset | `baseline-moving-deck-v0.1` |
| P2A 坐标与地理转换 | 已完成 | WGS84、ECEF、ENU、NED、FRD、相机光学坐标完整转换 | — |
| P2B 船舶 GNSS 仿真 | 已完成 | 位置、速度、噪声、延迟、丢包和固定种子 | `baseline-deck-gnss-sim-v0.1` |
| P2C GNSS 会合 | 已完成 | 飞向移动甲板 GNSS 上方并执行移动中心搜索 | `baseline-gnss-rendezvous-v0.1` |
| P2D GNSS—视觉接管 | 已完成 | 完整 Marker NED 位姿、平滑接管和视觉丢失恢复 | `baseline-gnss-vision-handover-v0.1` |
| P3 状态估计与预测 | 已完成 | 三维常速度 Kalman Filter、速度估计和短时预测 | `baseline-visual-estimator-v0.1` |
| P4 移动目标跟踪 | 已完成 | 预测位置目标、水平速度前馈、短时丢失衰减和真实 PX4 SITL 验收 | `baseline-moving-tracking-v0.1` |
| P4.5 实验复现与时间对齐 | 已完成 | rosbag 自动评测、统一仿真时钟、PX4 位姿历史、图像时刻插值和四场景 SITL 回归 | — |
| P4.6 正弦参数优化 | 已完成 | 预测时域、模式和相对速度阻尼扫描；正弦 RMSE 最佳 `0.3439 m`，但不修改全局默认值 | — |
| P4.7 加速度感知增益调度 | 已完成 | 匀速有效增益约 `0.25`，正弦换向提升至 `1.2`；统一参数通过交叉验证 | — |
| P5A 动态甲板与着陆窗口 | 已完成 | 升沉/倾斜/组合仿真、法向量倾角估计、迟滞窗口和五场景 SITL 验收 | — |
| P5B 相对高度下降 | 已完成 | 分阶段下降到 `0.50 m`、窗口暂停、恢复到 `2.0 m` 和恢复后重新授权锁止 | — |
| P5C 垂直状态估计与标定 | 当前阶段 | Marker z 动态偏差、独立垂直估计和多高度相对高度误差验收 | — |
| P6 触地与安全恢复 | 未开始 | `0.50 m` 以下最终下降、触地确认、恢复和安全中止 | — |
| P7/P8 评测与消融 | 未开始 | 批量实验、指标统计和传统方法消融 | — |

当前完整工作区测试结果：

```text
3 packages finished
154 tests
0 errors
0 failures
0 skipped
```

---

## 2. 包结构

| 包 | 说明 |
| --- | --- |
| [`src/aruco_detector`](src/aruco_detector/README.md) | 检测指定 ArUco Marker，发布位姿、可见性和调试图像。 |
| [`src/aruco_precision_landing_cpp`](src/aruco_precision_landing_cpp/README.md) | 完成 PX4 Offboard、GNSS 会合、视觉接管、状态估计、预测和移动目标跟踪。 |
| [`src/moving_deck_sim`](src/moving_deck_sim/README.md) | 提供移动甲板、船舶 GNSS 传感器模型和仅供评测使用的 Ground Truth。 |

控制器禁止订阅：

```text
/simulation/deck/ground_truth
```

Ground Truth 只允许用于仿真传感器和后续评测。

---

## 3. 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- PX4 SITL
- Gazebo Harmonic
- PX4 uXRCE-DDS Agent
- 与 PX4 版本匹配的 `px4_msgs`
- `ros_gz_bridge`
- OpenCV 4.x
- `colcon`

本文档默认目录：

```text
PX4：~/PX4-Autopilot
工作空间：~/ws_aruco_landing
px4_msgs underlay：~/ws_sensor_combined
```

如实际路径不同，请替换命令中的对应目录。

所有终端应使用相同的 `ROS_DOMAIN_ID`。未特别配置时保持默认即可。

---

## 4. 构建与测试

```bash
cd ~/ws_aruco_landing
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash

colcon build --symlink-install \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

source install/setup.bash
colcon test
colcon test-result --verbose
```

预期结果：

```text
154 tests, 0 errors, 0 failures, 0 skipped
```

如果 `px4_msgs` 位于其他工作空间，请替换：

```bash
source ~/ws_sensor_combined/install/setup.bash
```

---

# 5. 完整启动流程

推荐直接使用一键脚本（默认静止甲板）：

```bash
./scripts/start_sitl.sh
```

可选参数：

```bash
./scripts/start_sitl.sh --scenario constant02 --headless --record  # 0.2 m/s
./scripts/start_sitl.sh --scenario constant --headless --record    # 0.4 m/s
./scripts/start_sitl.sh --scenario sinusoidal
```

脚本会启动 Agent、PX4、Gazebo、甲板/GNSS、相机桥接和 ArUco 检测；确认 PX4/QGroundControl 状态后按回车，才会启动自动 Offboard/Arm 的控制器。`Ctrl-C` 会统一关闭本次启动的进程。如果 PX4 或 `px4_msgs` 不在默认目录，可通过 `PX4_DIR` 和 `PX4_MSGS_WS` 覆盖。

以下多终端流程保留用于逐项调试。

建议按以下终端顺序启动。控制器最后启动，因为它会自动发送 Offboard 和 Arm 命令。

## 终端 1：启动 uXRCE-DDS Agent

```bash
source /opt/ros/humble/setup.bash
MicroXRCEAgent udp4 -p 8888
```

PX4 启动后应看到客户端连接日志。

---

## 终端 2：启动 PX4 SITL

移动甲板 world 由 `moving_deck_sim` 启动，因此 PX4 必须使用 standalone 模式：

```bash
cd ~/PX4-Autopilot

PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD=aruco \
PX4_GZ_MODEL_POSE=-4,0,0.2 \
make px4_sitl gz_x500_mono_cam_down
```

PX4 会等待 Gazebo world 启动。

建议同时打开 QGroundControl，并确认：

- 飞行器已连接；
- Local Position 有效；
- 没有持续的严重 EKF/GPS 错误；
- 可以进入 Offboard 和解锁状态。

---

## 终端 3：启动移动甲板和船舶 GNSS

```bash
cd ~/ws_aruco_landing
source /opt/ros/humble/setup.bash
source install/setup.bash
source ~/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh
```

### 5.3.1 默认匀速场景

默认配置为水平匀速甲板：

```bash
ros2 launch moving_deck_sim moving_deck_sim.launch.py
```

当前默认速度为 Gazebo world ENU 的 East 正方向运动。

### 5.3.2 静止甲板

```bash
SHARE=$(ros2 pkg prefix --share moving_deck_sim)

ros2 launch moving_deck_sim moving_deck_sim.launch.py \
  config_file:=$SHARE/config/static.yaml \
  gnss_config_file:=$SHARE/config/gnss_ideal.yaml
```

建议第一次真实飞行测试先使用静止甲板。

### 5.3.3 XY 正弦运动甲板

```bash
SHARE=$(ros2 pkg prefix --share moving_deck_sim)

ros2 launch moving_deck_sim moving_deck_sim.launch.py \
  config_file:=$SHARE/config/sinusoidal_xy.yaml \
  gnss_config_file:=$SHARE/config/gnss_ideal.yaml
```

### 5.3.4 含噪 GNSS

```bash
SHARE=$(ros2 pkg prefix --share moving_deck_sim)

ros2 launch moving_deck_sim moving_deck_sim.launch.py \
  gnss_config_file:=$SHARE/config/gnss_noisy.yaml
```

### 5.3.5 无界面运行

```bash
ros2 launch moving_deck_sim moving_deck_sim.launch.py \
  headless:=true
```

---

## 终端 4：启动 Gazebo 相机到 ROS 2 的桥接

```bash
source /opt/ros/humble/setup.bash
source ~/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh

ros2 run ros_gz_bridge parameter_bridge \
  '/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
```

检查图像频率：

```bash
ros2 topic hz \
  /world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image
```

正常情况下约为：

```text
30 Hz
```

查看图像：

```bash
rqt_image_view
```

如果 Gazebo 出现：

```text
Failed to load system plugin libGstCameraSystem.so
```

但相机话题仍有稳定频率，可以继续测试；如果 Gazebo 和 ROS 2 都没有图像，则需先修复相机插件或模型资源路径。

---

## 终端 5：启动 ArUco 检测器

```bash
cd ~/ws_aruco_landing
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch aruco_detector aruco_detector.launch.py
```

检查检测结果：

```bash
ros2 topic echo /aruco/visible
ros2 topic echo /aruco/pose
```

查看调试图像：

```bash
rqt_image_view
```

选择：

```text
/aruco/debug_image
```

`/aruco/pose` 数值语义固定为 OpenCV 相机光学坐标系：

```text
x：图像向右
y：图像向下
z：镜头前方
```

---

## 终端 6：启动监控或 rosbag

建议在控制器启动前记录关键数据：

```bash
cd ~/ws_aruco_landing
mkdir -p bags

ros2 bag record \
  -o bags/p4_tracking_$(date +%Y%m%d_%H%M%S) \
  /landing/state \
  /landing/guidance_source \
  /landing/target_pose \
  /landing/deck_gnss_pose_ned \
  /landing/marker_pose_ned \
  /landing/estimated_deck_odometry \
  /landing/predicted_deck_pose \
  /landing/tracking_velocity_setpoint \
  /simulation/deck/ground_truth \
  /aruco/visible \
  /aruco/pose \
  /fmu/out/vehicle_local_position_v1 \
  /fmu/out/vehicle_odometry \
  /fmu/in/trajectory_setpoint
```

记录完成后使用统一评测入口：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash
python3 scripts/evaluate_p4_bag.py bags/<bag_name>
```

`bags/` 默认被 Git 忽略，Ground Truth 只在该离线脚本中用于误差统计。

常用监控命令：

```bash
ros2 topic echo /landing/state
ros2 topic echo /landing/guidance_source
ros2 topic echo /landing/target_pose
ros2 topic echo /landing/estimated_deck_odometry
ros2 topic echo /landing/predicted_deck_pose
ros2 topic echo /landing/tracking_velocity_setpoint
```

---

## 终端 7：启动控制器

```bash
cd ~/ws_aruco_landing
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash
source install/setup.bash

ros2 launch aruco_precision_landing_cpp px4_aruco_landing.launch.py
```

默认 PX4 v1.18 话题 remap：

```text
/fmu/out/vehicle_status
→ /fmu/out/vehicle_status_v4

/fmu/out/vehicle_local_position
→ /fmu/out/vehicle_local_position_v1
```

如果你的 PX4 消息版本不同，可在 launch 时覆盖：

```bash
ros2 launch aruco_precision_landing_cpp px4_aruco_landing.launch.py \
  vehicle_status_topic:=/fmu/out/vehicle_status \
  vehicle_local_position_topic:=/fmu/out/vehicle_local_position
```

---

# 6. 正常运行时的状态链

正常情况下状态应依次经过：

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

视觉长时丢失后：

```text
TRACK_TARGET
→ RECOVER_TO_GNSS
→ ACQUIRE_ARUCO 或 RENDEZVOUS_GNSS
```

P4 当前主路径不能进入：

```text
DESCEND_WITH_TRACKING
FINAL_LAND
DONE
```

运行时高度目标应持续为：

```text
/landing/target_pose.pose.position.z ≈ -5.0 m
```

PX4 使用 NED 坐标，负 z 表示向上。

---

# 7. P4 跟踪模式切换

配置文件：

```text
src/aruco_precision_landing_cpp/config/px4_aruco_landing.yaml
```

默认模式：

```yaml
tracking.mode: PREDICTED_POSITION_VELOCITY_FF
```

支持四种模式：

| 模式 | 位置目标 | 水平速度前馈 | 用途 |
| --- | --- | --- | --- |
| `RAW_VISUAL_POSITION` | 原始 Marker NED 位置 | 无 | P2D 原始视觉基线 |
| `ESTIMATED_POSITION` | Kalman 滤波位置 | 无 | 验证滤波效果 |
| `ESTIMATED_POSITION_VELOCITY_FF` | Kalman 滤波位置 | 估计甲板速度 | 验证速度前馈 |
| `PREDICTED_POSITION_VELOCITY_FF` | 短时预测位置 | 估计甲板速度 | 当前默认完整 P4 方法 |

可以复制配置文件后启动：

```bash
cp src/aruco_precision_landing_cpp/config/px4_aruco_landing.yaml \
   /tmp/p4_raw_visual.yaml
```

修改 `/tmp/p4_raw_visual.yaml`：

```yaml
tracking.mode: RAW_VISUAL_POSITION
```

然后运行：

```bash
ros2 launch aruco_precision_landing_cpp px4_aruco_landing.launch.py \
  config_file:=/tmp/p4_raw_visual.yaml
```

不要直接在飞行过程中动态修改模式。每轮实验应落地、关闭控制器并重新启动。

---

# 8. P4 关键参数

```yaml
tracking.mode: PREDICTED_POSITION_VELOCITY_FF
tracking.max_position_target_speed_mps: 2.0
tracking.max_position_target_step_m: 0.20
tracking.velocity_feedforward_gain: 1.0
tracking.relative_velocity_gain: 0.25
tracking.max_velocity_feedforward_mps: 1.5
tracking.max_velocity_feedforward_acceleration_mps2: 1.0
tracking.max_prediction_age_s: 0.75

motion_predictor.additional_prediction_horizon_s: 0.10
motion_predictor.max_prediction_horizon_s: 0.50

rendezvous_altitude_m: 5.0
enable_auto_land: false
```

调参建议顺序：

1. 先使用静止甲板确认坐标和悬停正常；
2. 使用 `RAW_VISUAL_POSITION` 跑通原始视觉基线；
3. 切换 `ESTIMATED_POSITION`；
4. 切换 `ESTIMATED_POSITION_VELOCITY_FF`；
5. 最后使用 `PREDICTED_POSITION_VELOCITY_FF`；
6. 每种模式使用相同甲板场景、初始位置和记录时间进行对比。

---

# 9. 如何判断 P4 运行正常

## 9.1 静止甲板

建议至少连续运行 20 秒，满足：

- 状态稳定在 `TRACK_TARGET`；
- `/landing/guidance_source` 与选择的跟踪模式一致；
- 估计甲板水平速度接近零；
- `/landing/tracking_velocity_setpoint` 接近零；
- 无人机水平误差不持续增大；
- 目标 z 始终接近 `-5.0 m`；
- 不频繁切换 GNSS 和视觉状态。

## 9.2 East 方向 0.4 m/s 匀速甲板

Gazebo ENU East 对应 PX4 local NED 的 `y`，因此应看到：

```text
/landing/deck_gnss_pose_ned.pose.position.y 增加
/landing/marker_pose_ned.pose.position.y 增加
/landing/predicted_deck_pose.pose.position.y 增加
/landing/target_pose.pose.position.y 增加
```

同时：

```text
/landing/estimated_deck_odometry.twist.twist.linear.y ≈ +0.4 m/s
```

预测和前馈模式下：

```text
/landing/tracking_velocity_setpoint.twist.linear.y > 0
```

不能出现目标方向反向、位置持续发散或高度目标变化。

## 9.3 正弦甲板

重点检查：

- 目标位置连续，无米级瞬时跳变；
- 估计速度方向随运动方向改变；
- 预测位置领先于估计位置但不持续发散；
- 速度前馈受最大速度和加速度限制；
- 短时丢帧不会立刻失控；
- 长时丢失后能够回退 GNSS。

---

# 10. 进入 P5B 前的验收条件

P5A 已完成以下安全高度验收：

1. S3 升沉、S4 横摇/纵摇和 S5 组合甲板均可重复运行；
2. Marker 向上法向量倾角估计在五场景中的总倾角 RMSE 为 `0.78°～1.61°`；
3. 静止、0.4 m/s 匀速和升沉场景在条件连续满足 `1.00 s` 后开启窗口；
4. 倾斜场景验证 `5°/8°` 进入/退出迟滞，组合场景因误差、速度和倾角持续超限而保持关闭；
5. 五轮 target z 均严格为 `-5.0000 m`，未进入下降、Land 或旧静态状态。

当前进入：

```text
P5B：相对甲板高度分阶段下降
```

---

# 11. 常见问题检查

## PX4 数据是否正常

```bash
ros2 topic echo /fmu/out/vehicle_status_v4 \
  --once \
  --qos-reliability best_effort \
  --qos-durability transient_local
```

```bash
ros2 topic echo /fmu/out/vehicle_local_position_v1 \
  --once \
  --qos-reliability best_effort \
  --qos-durability transient_local
```

应重点检查：

```text
xy_valid: true
z_valid: true
v_xy_valid: true
xy_global: true
z_global: true
```

## 船舶 GNSS 是否正常

```bash
ros2 topic hz /deck/gps/fix
ros2 topic echo /deck/gps/fix --once --qos-reliability best_effort
ros2 topic echo /deck/gps/velocity --once --qos-reliability best_effort
```

理想 GNSS 默认约为：

```text
5 Hz
```

## 是否存在重复发布者

```bash
ros2 topic info /deck/gps/fix -v
ros2 topic info /aruco/pose -v
```

正常情况下每个传感器话题只应有一个有效发布者。

## 是否有残留进程

```bash
ps -ef | grep -E \
'MicroXRCEAgent|px4|gz sim|moving_deck|deck_gnss|aruco_detector|px4_aruco' \
| grep -v grep
```

结束旧实验后应先确认相关进程已退出，再开始下一轮实验。

---

# 12. 停止顺序

测试结束时建议：

1. 在 QGroundControl 中执行 Land；
2. 确认无人机已经落地；
3. 关闭控制器；
4. 关闭 ArUco 检测器；
5. 关闭相机桥接；
6. 关闭移动甲板与 Gazebo；
7. 关闭 PX4 SITL；
8. 关闭 uXRCE-DDS Agent。

不要在飞行过程中仅调用甲板 reset 服务，因为该服务不会同时重置 PX4 飞行器状态。

---

# 13. 详细文档

- [下一阶段完整开发计划](docs/NEXT_DEVELOPMENT_PLAN.md)
- [传统基线实施计划](docs/TRADITIONAL_BASELINE_PLAN.md)
- [坐标系与变换契约](docs/COORDINATE_FRAMES.md)
- [移动甲板仿真 README](src/moving_deck_sim/README.md)
- [ArUco 检测器 README](src/aruco_detector/README.md)
- [PX4 控制器 README](src/aruco_precision_landing_cpp/README.md)
- [P1 移动甲板验收](docs/P1_MOVING_DECK_VALIDATION.md)
- [P2B 船舶 GNSS 验收](docs/P2B_DECK_GNSS_VALIDATION.md)
- [P2C GNSS 会合验收](docs/P2C_GNSS_RENDEZVOUS_VALIDATION.md)
- [P2D GNSS—视觉接管验收](docs/P2D_GNSS_VISION_HANDOVER_VALIDATION.md)
- [P3 状态估计详细计划](docs/P3_VISUAL_STATE_ESTIMATION_PLAN.md)
- [P3 状态估计验收](docs/P3_VISUAL_STATE_ESTIMATION_VALIDATION.md)
- [P4 移动目标跟踪详细计划](docs/P4_MOVING_TARGET_TRACKING_PLAN.md)
- [P4 移动目标跟踪验收](docs/P4_MOVING_TARGET_TRACKING_VALIDATION.md)
- [P4.5 执行计划](docs/P4_5_EXECUTION_PLAN.md)
- [P4.5 时间对齐验收](docs/P4_5_TIME_ALIGNMENT_VALIDATION.md)
- [P4.6 正弦调参计划](docs/P4_6_SINUSOIDAL_TUNING_PLAN.md)
- [P4.6 正弦调参验收](docs/P4_6_SINUSOIDAL_TUNING_VALIDATION.md)
- [P4.7 自适应增益调度计划](docs/P4_7_ADAPTIVE_GAIN_SCHEDULING_PLAN.md)
- [P4.7 自适应增益调度验收](docs/P4_7_ADAPTIVE_GAIN_SCHEDULING_VALIDATION.md)
- [P5A 动态甲板与着陆窗口计划](docs/P5A_DECK_DYNAMICS_AND_LANDING_WINDOW_PLAN.md)
- [P5A 动态甲板与着陆窗口验收](docs/P5A_DECK_DYNAMICS_AND_LANDING_WINDOW_VALIDATION.md)
- [P5B 相对高度下降计划](docs/P5B_RELATIVE_DESCENT_PLAN.md)
- [P5B 相对高度下降验收](docs/P5B_RELATIVE_DESCENT_VALIDATION.md)
- [P5A 动态甲板与着陆窗口计划](docs/P5A_DECK_DYNAMICS_AND_LANDING_WINDOW_PLAN.md)
- [P5A 动态甲板与着陆窗口验收](docs/P5A_DECK_DYNAMICS_AND_LANDING_WINDOW_VALIDATION.md)

---

# 14. 安全提示

控制节点会自动发送 Offboard 和 Arm 命令。当前 P5A 默认：

```yaml
enable_auto_land: false
rendezvous_altitude_m: 5.0
```

相对下降默认关闭；显式启用时只允许下降到 `0.50 m` 安全测试高度，并继续保持水平跟踪。当前不会触地或发送 Land/Disarm 命令，仅应先在 SITL 中验证。

实机测试前必须重新确认：

- 相机内参与外参；
- PX4、NED、FRD、ENU 和相机光学坐标方向；
- 图像和 PX4 时间同步；
- 水平速度和加速度限制；
- Offboard failsafe；
- GNSS 和视觉丢失策略；
- 人工接管方式；
- 解锁、降落和紧急停机流程。
