# AGENTS.md

## 项目目标

本项目基于 PX4 SITL、Gazebo Harmonic 和 ROS 2 Humble，实现无人机在移动船舶甲板上的自主降落传统基线。

最终传统基线必须完成：

```text
船舶 GNSS / 遥测粗引导
→ 移动甲板上方会合
→ ArUco 稳定捕获与视觉接管
→ 完整坐标变换
→ 甲板状态估计
→ 短时运动预测
→ 移动目标水平跟踪
→ 规则式着陆窗口判断
→ 相对高度下降
→ 持续跟踪直至触地
→ 安全恢复或中止
→ 批量实验和指标统计
```

详细需求、目录结构、状态机、实验场景和验收指标见：

```text
docs/TRADITIONAL_BASELINE_PLAN.md
docs/NEXT_DEVELOPMENT_PLAN.md
docs/COORDINATE_FRAMES.md
```

---

## 当前实现

当前已完成：

* `P0` 仓库整理与静态基线冻结。
  * `aruco_detector` 已作为普通 ROS 2 包纳入主仓库。
  * 静态基线标签：`baseline-static-v0.1`。

* `P1` 水平移动甲板仿真。
  * `moving_deck_sim` 已支持静止、水平匀速和 XY 正弦运动。
  * 支持确定性 reset 和仅供评测使用的 Ground Truth。
  * 移动甲板标签：`baseline-moving-deck-v0.1`。

* `P2A` 坐标与地理转换数学基础。
  * 已实现独立 `coordinate_transform` 和 `geodetic_converter`。
  * 已覆盖刚体变换、ENU/NED、WGS84/ENU、异常输入和局部范围测试。

* `P2B` 船舶 GNSS 传感器仿真。
  * 已实现 `/deck/gps/fix` 和 `/deck/gps/velocity`。
  * 支持理想/含噪配置、固定频率、固定延迟、丢包和固定种子复现。
  * reset 后清空延迟队列并重置采样相位与随机序列。

* `P2C` GNSS 会合与移动甲板上方粗跟踪。
  * 已实现 `WAIT_DECK_GNSS`、`RENDEZVOUS_GNSS` 和 `ACQUIRE_ARUCO`。
  * 已实现船舶 WGS84 转 PX4 local NED、跳变拒绝、超时回退和目标限幅。
  * 搜索中心随实时船舶 GNSS 移动。
  * 验收记录见 `docs/P2C_GNSS_RENDEZVOUS_VALIDATION.md`。

* `P2D` GNSS—视觉接管与下降前恢复。
  * 已将完整相机外参和 PX4 `VehicleOdometry` 接入视觉坐标链。
  * 已实现 `VISUAL_HANDOVER`、`TRACK_TARGET` 和 `RECOVER_TO_GNSS`。
  * 已实现 GNSS—视觉一致性、测量跳变拒绝、线性接管、短时保持和长时恢复。
  * 全程保持安全高度，不下降；验收记录见 `docs/P2D_GNSS_VISION_HANDOVER_VALIDATION.md`。

* `P3` 视觉状态估计与短时预测。
  * 已实现独立三维常速度 Kalman Filter 和受限常速度预测器。
  * 使用视觉采样时间处理滤波 `dt`，处理乱序、离群点、短时丢帧和长时重初始化。
  * 已发布 `/landing/estimated_deck_odometry` 和 `/landing/predicted_deck_pose`。
  * 验收记录见 `docs/P3_VISUAL_STATE_ESTIMATION_VALIDATION.md`。

* `P4` 安全高度移动甲板水平跟踪。
  * 已实现原始视觉、估计位置、估计位置+速度前馈、预测位置+速度前馈四种模式。
  * 默认将预测位置写入 PX4 position setpoint，并将甲板速度与相对速度阻尼写入水平 velocity feedforward。
  * 已实现位置目标、前馈速度和前馈加速度限制，以及短时丢失衰减和长时 GNSS 恢复。
  * 代码与消息级验收完成；真实 PX4 跟踪 RMSE 待用户确认，见 `docs/P4_MOVING_TARGET_TRACKING_VALIDATION.md`。

现有运行包：

* `aruco_detector`
  * 输出 `/aruco/pose`、`/aruco/visible`、`/aruco/debug_image`。

* `aruco_precision_landing_cpp`
  * 订阅 PX4 状态、船舶 GNSS、ArUco 完整位姿和可见性。
  * 通过 PX4 Offboard 位置设定点完成起飞、GNSS 会合、移动搜索、视觉接管、移动甲板水平跟踪和 GNSS 恢复。
  * 估计并发布甲板视觉位置、速度、协方差和短时预测位置。
  * 在 `TRACK_TARGET` 中发布预测位置目标和受限水平速度前馈；当前 P4 主路径仍不下降。
  * 旧静态下降代码仅作为历史基线保留且从主路径不可达。

* `moving_deck_sim`
  * 驱动水平移动甲板并发布 `/simulation/deck/ground_truth`。
  * 仿真传感器节点将真值处理为船舶 GNSS 位置和 ENU 速度。
  * Ground Truth 禁止进入降落控制器。

当前缺少：

* 图像采样时刻的 PX4 位姿历史插值和严格跨时间域对齐。
* P4 静止、匀速和正弦真实 PX4 仿真 RMSE 与增益调参。
* 规则式着陆窗口和相对高度下降。
* 移动甲板持续跟踪触地。
* 触地检测、恢复策略和批量评测。

---

## 实现顺序

Codex 必须按以下顺序推进，除非用户明确改变优先级：

1. `P0`：仓库结构与静态基线冻结，已完成。
2. `P1`：水平移动甲板仿真，已完成。
3. `P2.0`：同步项目状态和总体设计文档，已完成。
4. `P2A`：完整刚体坐标与 WGS84 / ENU / NED 转换，纯数学模块已完成。
5. `P2B`：船舶 GNSS 传感器仿真，已完成。
6. `P2C`：GNSS 会合和移动甲板上方粗跟踪，已完成，不下降。
7. `P2D`：ArUco 完整变换、GNSS 到视觉接管和下降前恢复，已完成，不下降。
8. `P3`：甲板视觉状态估计和短时预测，已完成。
9. `P4`：移动甲板视觉水平跟踪，代码与消息级验收已完成，真实飞行验收待确认。
10. `P5`：规则式着陆窗口和分阶段下降；P4 真实跟踪验收通过后才能开始。
11. `P6`：触地确认、恢复和安全中止。
12. `P7`：批量评测。
13. `P8`：传统方法消融实验。

未完成 `P0~P7` 前，不实现强化学习或 MPC。

---

## 核心约束

### 控制器允许使用

* PX4 飞行状态和无人机估计状态。
* PX4 local/global 定位参考。
* 经过传感器模型处理的船舶 GNSS 位置和速度。
* 相机图像和相机内参。
* 相机外参。
* ArUco 位姿和可见性。
* 控制器内部估计状态。
* YAML 配置参数。

普通 GNSS 只用于远距离会合、视觉捕获和下降前恢复，不参与最终精确下降和低高度横向接管。

### 控制器禁止使用

* Gazebo 甲板 Ground Truth。
* 仿真器内部运动相位。
* 预先知道的甲板未来轨迹。
* 评测器计算结果。

Ground Truth 只能用于评测和误差统计。

---

## 坐标系

统一使用：

* `camera_optical`：右、下、前。
* `base_link_frd`：前、右、下。
* `local_ned`：北、东、下。

Marker 位姿转换统一为：

```text
T_local_ned_marker
=
T_local_ned_body_frd
*
T_body_frd_camera_optical
*
T_camera_optical_marker
```

要求：

* `T_camera_optical_marker` 来自 ArUco PnP。
* `T_body_frd_camera_optical` 来自明确方向的参数或静态 TF。
* `T_local_ned_body_frd` 来自 PX4 `VehicleOdometry`，必须检查 `pose_frame`。
* 船舶 GNSS 转 local NED 使用 PX4 `VehicleLocalPosition.ref_lat/ref_lon/ref_alt`。
* 禁止在业务代码中散落手写符号转换。
* 刚体坐标转换集中在 `coordinate_transform` 模块。
* WGS84 / ENU / NED 转换集中在 `geodetic_converter` 模块。
* 所有坐标转换必须有单元测试。

---

## 控制原则

移动目标跟踪采用：

```text
预测甲板位置目标
+
甲板速度前馈
+
水平位置和相对速度反馈
```

下降采用相对甲板高度，不使用固定世界高度：

```text
relative_height = deck_z_ned - uav_z_ned
position_sp_z = predicted_deck_z_ned - height_ref
```

只有满足以下条件并持续一定时间才允许下降：

* Marker 可见且数据新鲜。
* 水平误差小于阈值。
* 水平相对速度小于阈值。
* 甲板横摇和纵摇小于阈值。
* 状态估计有效。

移动甲板场景下，默认保持 Offboard 水平跟踪直至确认触地，不得在较高高度直接切换 PX4 `NAV_LAND`。

---

## 推荐状态机

```text
INIT
WAIT_FOR_PX4
OFFBOARD_PRE_STREAM
ARM_AND_TAKEOFF
WAIT_DECK_GNSS
RENDEZVOUS_GNSS
ACQUIRE_ARUCO
VISUAL_HANDOVER
TRACK_TARGET
WAIT_LANDING_WINDOW
DESCEND
FINAL_DESCENT
TOUCHDOWN_CONFIRM
DONE
RECOVER_TO_GNSS
RECOVER_CLIMB
ABORT
```

每次状态转换必须记录：

* 原状态。
* 新状态。
* 转换原因。
* 时间戳。
* 当前误差和对应阈值。

---

## 代码修改规则

* 每次只完成一个明确任务。
* 只修改与当前任务直接相关的文件。
* 不顺手重构相邻模块。
* 不为一次性需求创建复杂抽象。
* 优先复用现有节点、参数和 launch。
* 第一版估计器、预测器和控制器作为独立 C++ 类保留在同一个 ROS 节点内，不急于拆成多个节点。
* 不新增自定义消息包，除非标准消息已无法清晰表达稳定接口。
* 所有参数必须同时更新声明、校验、YAML、文档和测试。
* 禁止硬编码用户目录、绝对路径和 PX4 版本后缀话题。

---

## C++ 注释规则

使用 C++17。

所有新增公开函数必须使用 Doxygen 中文注释，说明：

* 函数实现的功能。
* 输入参数的坐标系、单位和有效范围。
* 返回值或输出内容。
* 失败条件。

复杂坐标变换、滤波更新、运动预测、状态转换和控制公式必须添加中文注释。

不要添加无意义的逐行注释。

示例：

```cpp
/**
 * @brief 将相机坐标系下的 Marker 位姿转换到 PX4 local NED 坐标系。
 *
 * @param camera_marker_pose Marker 在 camera_optical 坐标系下的位姿，位置单位为米。
 * @param ned_body_pose 无人机机体在 local_ned 坐标系下的位姿。
 * @return Marker 在 local_ned 坐标系下的位姿；输入无效时返回 std::nullopt。
 */
std::optional<Pose3d> transform_marker_to_local_ned(...);
```

---

## 测试要求

新增数学模块优先写成不依赖 ROS 的普通 C++ 类，并使用 GTest 测试。

至少覆盖：

* 坐标变换。
* Kalman Filter 预测和更新。
* 异常时间间隔。
* 观测超时。
* 着陆窗口迟滞。
* 正常状态路径。
* 丢失目标恢复路径。
* ABORT 路径。
* 参数非法输入。

每个阶段完成后执行：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash

colcon build --symlink-install \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

colcon test
colcon test-result --verbose
```

---

## 安全要求

* 默认配置只用于 SITL。
* 自动 Arm、Land 和 Disarm 必须由显式参数控制。
* PX4 状态无效时禁止发布有效运动目标。
* 观测过期时禁止继续正常速度下降。
* NaN、Inf 和非法四元数不得进入 PX4 setpoint。
* 位置、速度、加速度和目标变化率必须限幅。
* 测试代码不能误触发实机自动解锁。

---

## 完成定义

传统基线完成必须同时满足：

1. 普通 clone 后可以构建整个工作区。
2. 静态 ArUco 降落功能保持可用。
3. 控制器不使用仿真 Ground Truth。
4. 船舶 GNSS 能够引导无人机到移动甲板上方。
5. GNSS 到视觉接管平滑且可恢复。
6. WGS84、ENU、NED、FRD 和相机光学坐标变换正确并有测试。
7. 能够稳定跟踪水平移动甲板。
8. 能够估计甲板速度并短时预测。
9. 能够判断规则式着陆窗口。
10. 能够持续跟踪移动甲板直至触地。
11. 丢失目标时能够减速、恢复或中止。
12. 能够批量运行实验并输出统一指标。
13. 能够完成 B0 到 B4 的传统方法消融对比。

---

## 默认下一任务

`P0`～`P3` 已完成；`P4` 代码、单元测试和消息级验收已完成。没有额外指令时，不直接实现 P5，先执行 P4 真实 PX4 SITL 验收：

```text
依次运行静止、0.2 m/s、0.4 m/s 和 XY 正弦甲板，
对比 RAW_VISUAL_POSITION、ESTIMATED_POSITION_VELOCITY_FF 和 PREDICTED_POSITION_VELOCITY_FF，
记录水平位置 RMSE、相对速度 RMSE、最大误差、Marker 丢失和 GNSS 恢复次数，
根据 rosbag 结果调节前馈增益、相对速度增益、速度/加速度限制和预测时域。
用户确认 P4 真实跟踪正常后，再进入 P5 着陆窗口和分阶段下降。
```
