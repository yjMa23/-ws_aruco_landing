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
docs/plans/TRADITIONAL_BASELINE_PLAN.md
docs/plans/NEXT_DEVELOPMENT_PLAN.md
docs/reference/COORDINATE_FRAMES.md
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
  * 验收记录见 `docs/validation/P2C_GNSS_RENDEZVOUS_VALIDATION.md`。

* `P2D` GNSS—视觉接管与下降前恢复。
  * 已将完整相机外参和 PX4 `VehicleOdometry` 接入视觉坐标链。
  * 已实现 `VISUAL_HANDOVER`、`TRACK_TARGET` 和 `RECOVER_TO_GNSS`。
  * 已实现 GNSS—视觉一致性、测量跳变拒绝、线性接管、短时保持和长时恢复。
  * 全程保持安全高度，不下降；验收记录见 `docs/validation/P2D_GNSS_VISION_HANDOVER_VALIDATION.md`。

* `P3` 视觉状态估计与短时预测。
  * 已实现独立三维常速度 Kalman Filter 和受限常速度预测器。
  * 使用视觉采样时间处理滤波 `dt`，处理乱序、离群点、短时丢帧和长时重初始化。
  * 已发布 `/landing/estimated_deck_odometry` 和 `/landing/predicted_deck_pose`。
  * 验收记录见 `docs/validation/P3_VISUAL_STATE_ESTIMATION_VALIDATION.md`。

* `P4` 安全高度移动甲板水平跟踪。
  * 已实现原始视觉、估计位置、估计位置+速度前馈、预测位置+速度前馈四种模式。
  * 默认将预测位置写入 PX4 position setpoint，并将甲板速度与相对速度阻尼写入水平 velocity feedforward。
  * 已实现位置目标、前馈速度和前馈加速度限制，以及短时丢失衰减和长时 GNSS 恢复。
  * 已完成静止、0.2 m/s、0.4 m/s 和 XY 正弦真实 PX4 SITL 验收，见 `docs/validation/P4_MOVING_TARGET_TRACKING_VALIDATION.md`。

* `P4.5` 实验可复现与视觉时间对齐。
  * 已新增 `scripts/evaluate_p4_bag.py`，统一计算 P4 rosbag 指标，`bags/` 默认不进入 Git。
  * 已统一 SITL 下 ArUco 检测器和降落控制器的 `use_sim_time`。
  * 已实现 `VehiclePoseHistory`、PX4→ROS 时间映射和图像采样时刻位姿插值。
  * 代码、103 项测试和静止、0.2 m/s、0.4 m/s、XY 正弦完整 PX4 SITL 回归均通过，见 `docs/validation/P4_5_TIME_ALIGNMENT_VALIDATION.md`。

* `P5A` 动态甲板与规则式着陆窗口。
  * 已实现升沉、横摇/纵摇和组合甲板轨迹，并发布完整姿态、线速度和角速度 Ground Truth。
  * 已实现基于 Marker 向上法向量的甲板倾角估计，Ground Truth 只用于离线比较。
  * 已实现 `LandingWindow` 迟滞、连续满足时间和多原因拒绝，并接入 `WAIT_LANDING_WINDOW`。
  * 静止、0.4 m/s、升沉、倾斜和组合场景 SITL 验收通过，全程保持 `5 m`，见 `docs/validation/P5A_DECK_DYNAMICS_AND_LANDING_WINDOW_VALIDATION.md`。

* `P5B` 相对甲板高度分阶段下降。
  * 已实现 `RelativeDescentController`、`DESCEND`、`TEST_HEIGHT_HOLD` 和 `RECOVER_CLIMB`。
  * 已完成静止、0.4 m/s 匀速和升沉甲板下降到 `0.50 m` 安全测试高度的 PX4 SITL 验收。
  * 已验证组合运动窗口未打开时不会绕过条件下降，严重失效时可恢复到 `2.0 m`。
  * 恢复后会锁止再次下降，必须重新完成视觉接管或重启任务才解除。
  * 默认 `descent.enabled=false`，不触地、不 `NAV_LAND`、不 Disarm，见 `docs/validation/P5B_RELATIVE_DESCENT_VALIDATION.md`。

* `P5C` 低高度垂直状态估计与标定。
  * 已将 `x500_mono_cam_down` 相机外参修正为 FRD `[0, 0, 0.14]`，消除约 `0.24 m` 垂直系统偏差。
  * 已实现独立 `VerticalStateEstimator`、P5C 离线评测和垂直状态调试话题。
  * 已将甲板垂直速度前馈 `gain=1.0` 设为相对下降默认；`--disable-vertical-ff` 保留用于消融。
  * 静止 `0.50 m` 和升沉 `0.70 m` 验收通过，见 `docs/validation/P5C_VERTICAL_STATE_ESTIMATION_VALIDATION.md`。

* `P6A` 多源触地候选与确认。
  * 已实现独立 `TouchdownDetector`，联合 PX4 land detector、视觉相对高度和垂直速度证据。
  * 视觉高度不能单独确认；候选必须连续满足并具备迟滞，确认后锁存。
  * 已并行接入四个触地调试话题，但不改变状态机、TrajectorySetpoint 或 Land/Disarm。
  * 静止 `0.50 m`、升沉 `0.70 m` 和恢复爬升三类负向 SITL 验收均无候选和确认，见 `docs/validation/P6_TOUCHDOWN_CONFIRMATION_VALIDATION.md`。

* `P6B` 最终下降与真实接触功能。
  * 已实现 `FINAL_DESCENT`、`TOUCHDOWN_CANDIDATE_HOLD`、`TOUCHDOWN_HOLD`、分段最终下降、动态平台相对水平速度证据和视觉短时丢帧去抖。
  * 四尺度有状态 Marker 选择器和项目内 `near=0.02 m` 相机模型已接入。
  * 2026-07-30 已完成终端落板修补与复验：参考在安全终端段继续降到 `0.05 m`，static/constant02 均进入 `TOUCHDOWN_CANDIDATE_HOLD → TOUCHDOWN_HOLD` 并保持 10 秒，P6B 正向验收 PASS，见 `docs/validation/P6B_FINAL_DESCENT_AND_TOUCHDOWN_VALIDATION.md`。

* `P7-lite` 批量评测开发基线。
  * 已实现单轮运行、顺序批量、seed 展开、resume、统一失败分类、轻量 Bag、参数快照和基础聚合。
  * 第一版支持 static 和 constant02；2026-07-30 已完成真实 3+3 冒烟，6/6 PASS、0 failure，录包、清理、评测和聚合结果完整。
  * P7-lite 已冻结为高级功能开发基线；static 20 次 + constant02 20 次不再是进入 P8A 的硬门槛，配置和自动化保留，大规模实验延后到 P9。
  * 执行计划见 `docs/plans/P7_BATCH_EVALUATION_PLAN.md`，高级路线见 `docs/plans/P8_ADVANCED_LANDING_ROADMAP.md`。

现有运行包：

* `aruco_detector`
  * 输出 `/aruco/pose`、`/aruco/visible`、`/aruco/debug_image`。

* `aruco_precision_landing_cpp`
  * 订阅 PX4 状态、船舶 GNSS、ArUco 完整位姿和可见性。
  * 通过 PX4 Offboard 位置设定点完成起飞、GNSS 会合、移动搜索、视觉接管、移动甲板水平跟踪和 GNSS 恢复。
  * 估计并发布甲板视觉位置、速度、协方差、短时预测位置和视觉倾角。
  * 默认运行 P4.7 水平跟踪；显式选择时运行 P8B 水平相对 MPC，并在失败或终端阶段安全回退 P4.7。
  * 相对下降和最终下降仅在显式授权时启用；P5C 垂直速度前馈只在相对下降状态生效。
  * 已支持 static、纯水平运动和 P8A 分级升沉场景的真实接触与接触后保持。
  * 默认不下降，不发送 `NAV_LAND`，不自动 Disarm。
  * 旧静态下降代码仅作为历史基线保留且从主路径不可达。

* `moving_deck_sim`
  * 驱动水平、升沉、横摇/纵摇和组合移动甲板并发布 `/simulation/deck/ground_truth`。
  * 仿真传感器节点将真值处理为船舶 GNSS 位置和 ENU 速度。
  * Ground Truth 禁止进入降落控制器。

高级阶段状态与剩余工作：

* `P8A` 已完成升沉甲板最终下降、真实接触和接触后相对保持验收，H1 3/3、H2 3/3 PASS。
* `P8B` 已完成固定 OSQP/OsqpEigen 依赖、4 状态水平相对 MPC、约束、warm start、完整 P4.7 fallback、`TERMINAL_PHASE_P47` 安全 handoff、诊断、`271` 项全工作区测试和严格顺序真实 SITL；安全高度 15/15、下降 6/6、最终代码真实触地 6/6 PASS，状态为 `VALIDATION PASS`。
* `P8C` 固定倾斜与低频 roll/pitch 甲板的几何建模、计划、实现和验收。
* `P9` 统一批量评测、消融和论文实验；P7 的 20+20 配置将在该阶段执行。
* 触地后的 Land/Disarm 授权和最终恢复策略；当前仍保持 `NAV_LAND / Disarm = 0 / 0`。

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
9. `P4`：移动甲板视觉水平跟踪，代码、测试和真实 PX4 SITL 验收已完成。
10. `P4.5`：实验可复现与视觉时间对齐，代码、测试和完整 SITL 回归已完成。
11. `P4.6`：正弦运动预测时域和相对速度阻尼优化，已完成；正弦最佳 RMSE `0.3439 m`，但高阻尼会恶化匀速跟踪。
12. `P4.7`：基于估计甲板加速度的连续增益调度，已完成并设为统一默认。
13. `P5A`：动态甲板与规则式着陆窗口，已完成。
14. `P5B`：相对甲板高度分阶段下降，已完成 `0.50 m` 安全测试高度验收。
15. `P5C`：低高度垂直状态估计、偏差标定和相对高度跟踪优化，已完成。
16. `P6A`：多源触地候选、确认和负向验收，已完成。
17. `P6B`：最终下降与终端接触确认已通过 static/constant02 单轮和 P7 3+3 冒烟。
18. `P7-lite`：批量评测管线和真实 3+3 冒烟已完成并冻结；20+20 延后到 P9。
19. `P8A`：升沉甲板最终下降与真实接触，已完成 H1/H2 真实验收。
20. `P8B`：水平相对运动线性 MPC，综述、固定依赖、实现、全量测试和严格顺序真实 SITL 已完成，验收见 `docs/validation/P8B_RELATIVE_MPC_VALIDATION.md`。
21. `P8C`：固定倾斜及低频 roll/pitch 甲板降落，必须先完成综述和几何模型。
22. `P9`：统一批量评测、消融和论文实验。

P8A、P8B 已通过真实验收。当前只允许开始 P8C 倾斜甲板综述和几何建模；必须先完成 `docs/research/P8C_TILTED_DECK_LANDING_REVIEW.md` 并达到 `RESEARCH PASS`，再保存独立执行计划，未完成前不得修改倾斜甲板终端控制生产代码。强化学习不属于本轮传统高级基线实现范围。

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

* 对于需要在论文中详细建模的复杂方法，未完成可验证的综述文档和独立执行计划前，不得编写生产实现。
* 每次只完成一个明确任务。
* 只修改与当前任务直接相关的文件。
* 不顺手重构相邻模块。
* 不为一次性需求创建复杂抽象。
* 优先复用现有节点、参数和 launch。
* 第一版估计器、预测器和控制器作为独立 C++ 类保留在同一个 ROS 节点内，不急于拆成多个节点。
* 不新增自定义消息包，除非标准消息已无法清晰表达稳定接口。
* 所有参数必须同时更新声明、校验、YAML、文档和测试。
* 禁止硬编码用户目录、绝对路径和 PX4 版本后缀话题。
* 每次修改后必须同步更新所有受影响的说明文档，并在交付时说明更新了哪些文档；至少检查根 `README.md`、对应包级 README、`docs/guides/OPERATIONS.md` 以及相关计划和验收文档，确保功能、参数、命令、路径与当前状态一致。

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

`P0`～`P6B` 已完成，P7-lite 真实 3+3 冒烟已于 2026-07-30 以 6/6 PASS 完成并冻结为开发基线。没有额外指令时：

```text
冻结当前终端落板、触地确认、Marker 和 close-range 相机参数；
P8A 已完成 H1/H2 升沉触地真实验收；
P8B 已完成 research → model → fixed solver dependency → implementation → full test → strict SITL validation，验收为安全高度 15/15、下降 6/6、真实触地 6/6 PASS；
下一步只推进 P8C research：先完成倾斜甲板综述、几何模型和独立计划，不直接编码终端控制；
P7 20+20 配置继续保留并延后到 P9 统一论文实验；
全程保持 NAV_LAND / Disarm = 0 / 0，Ground Truth 只能用于离线评测。
```
