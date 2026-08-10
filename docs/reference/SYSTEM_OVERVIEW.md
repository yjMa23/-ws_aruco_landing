# 移动船舶无人机自主降落系统总览

本文档是当前实现与验证边界的唯一事实源。未来工作见[下一步计划](../plans/NEXT_DEVELOPMENT_PLAN.md)，控制与几何推导见[控制理论](LANDING_CONTROL_THEORY.md)。

## 1. 系统目标

系统在 PX4 SITL、Gazebo Harmonic 和 ROS 2 Humble 中完成：

```text
船舶 GNSS 粗引导
→ 移动甲板上方会合
→ ArUco 捕获与视觉接管
→ 甲板状态估计和预测
→ 水平相对跟踪
→ 着陆窗口判断
→ 相对高度下降
→ 终端接触与触地确认
→ 接触后保持或安全恢复
```

默认配置只保持安全高度，不下降、不发送 `NAV_LAND`、不自动 Disarm。

## 2. 软件组成

| 包 | 当前职责 |
| --- | --- |
| `aruco_detector` | 读取图像与相机内参，完成多尺度 Marker 选择、非共面远距联合 PnP、单 Marker 回退和调试图像发布。 |
| `aruco_precision_landing_cpp` | PX4 Offboard、GNSS 会合、视觉接管、估计、预测、跟踪、下降、触地检测和接触保持。 |
| `moving_deck_sim` | 生成 legacy 甲板或 marine vessel 刚体运动、deck-center Ground Truth，并提供船舶 GNSS 传感器模型。 |

仓库根目录脚本负责 SITL 编排、单轮/批量实验、离线评测、聚合和论文统计。

## 3. 仿真与传感器

### 3.1 甲板运动

`moving_deck_sim` 支持：

- 静止。
- 水平匀速和 XY 正弦运动。
- 升沉。
- 固定正负 `2°` roll 或 pitch。
- 低频动态 `rollpitch`。
- 同时包含水平运动、升沉和姿态运动的 `combined`。
- 在 `combined` 基础上增加小幅周期 yaw 的 `rigid_body_motion`。

运动控制器支持确定性 reset、固定更新频率和固定随机种子。Ground Truth 发布完整位置、姿态、线速度和角速度，只能供传感器仿真与离线评测使用。

Marine M2 的独立 `aruco_marine_vessel.sdf` 已切换到 `vrx_wamv_landing`。world 仍名为 `aruco`，保持 ENU、球面坐标和 250 Hz 物理更新；场景包含官方 VRX WAM-V base mesh/PBR visual、约 `300×300 m` visual-only VRX-style PBR ocean、独立静态 UAV launch platform，以及 WAM-V 上新增的 `2.4×2.4 m` UAV landing platform。legacy `aruco_moving_deck.sdf` 和 `models/moving_deck` 保持独立且仍是默认环境。

marine 中同一 `MotionProfile` 驱动与官方 `wamv/base_link` 对齐的 canonical `vessel_body`，neutral reference 为 world z≈0.2 m；固定 `T_vessel_deck` 使用 `r_VD=[0,0,1.8] m`、`R_V_D=I`，把 raw vessel state 转为 landing deck center，neutral deck center 仍为 z≈2.0 m。`rigid_body_kinematics` 显式加入 `R(ω×r)` lever-arm 线速度，因此 roll/pitch 同时改变 deck orientation 与 deck-center position/velocity。`/simulation/deck/ground_truth_raw` 在 marine 表示 vessel raw state，最终 `/simulation/deck/ground_truth` 仍表示 deck center。

M2 的 WAM-V / ocean 单模型通过 `gz sdf -k`，完整 marine world 在 `gz sdf -k` 中会触发 sdformat CLI 缺少 `model://` find callback 的已知限制；实际 `gz sim` server smoke 已成功解析 include 并加载 `vrx_wamv_landing` 的 VelocityControl / OdometryPublisher，未出现 missing mesh/texture/material。全仓构建与测试为 `374 tests, 0 errors, 0 failures, 0 skipped`。Marine `static` 与 `rigid_body_motion` 各完成一轮有限 headless PX4 SITL smoke：两轮都完成 GNSS rendezvous、非共面 ArUco 捕获并停留在安全高度 `WAIT_LANDING_WINDOW`；rigid-body 运行时 shadow 为 `UPDATED:TRUSTED`、terminal stabilization=false、touchdown_confirmed=false。rigid-body 抓取 `3578` 组同时间戳 raw/deck GT，位置刚体关系最大误差 `8.95e-16 m`，lever-arm 速度关系最大误差 `1.57e-16 m/s`，全部 finite；最早 deck GT 为 z≈2.00 m，随后 6-DoF profile 使 deck z 覆盖约 `1.453–2.274 m`。两轮 PX4 ULog 均为 `NAV_LAND=0`、Disarm command=0，结束时 landed=false。当前验证 shell 无 DISPLAY/Wayland，因此未做 GUI 人工视觉验收。

### 3.2 船舶 GNSS

传感器节点发布：

```text
/deck/gps/fix       sensor_msgs/msg/NavSatFix
/deck/gps/velocity  geometry_msgs/msg/Vector3Stamped
```

支持理想/含噪配置、采样频率、固定延迟、丢包、固定种子和 reset 后确定性复现。速度语义为 world ENU；控制器统一转换到 local NED。

### 3.3 ArUco

检测器发布：

```text
/aruco/pose
/aruco/visible
/aruco/debug_image
```

四尺度 Marker 使用有状态选择，避免近距离尺寸切换抖动；PnP 位姿先补偿到统一甲板参考点，再进入坐标链。相机模型支持普通和近距配置。

约 `5 m` 远距目标另有 ID 0/4/5/6 非共面板：ID 0 为 `0.50 m` 共面主 Marker，
ID 4/5/6 为 `0.75 m`、分别沿两个甲板轴正负倾斜 `45°` 的副 Marker。检测器把当前
可见的已标定角点一次联合求解到 `deck_landing_up`；只有角点集合退化为共面时才回退
单 Marker。三个倾斜纹理为 `120×120`，避免 Gazebo 斜视采样把码元插值成灰块。

## 4. 坐标和时间

统一坐标：

- `camera_optical`：右、下、前。
- `base_link_frd`：前、右、下。
- `local_ned`：北、东、下。
- `deck_landing_up`：x 甲板前、y 甲板左、z 甲板上。
- `vessel_body`：marine WAM-V canonical 船体运动参考系，与官方 `wamv/base_link` 轴语义对齐，neutral world z≈0.2 m。
- `landing_deck`：marine 固定 UAV 着陆平面中心 frame，相对 `vessel_body` z=+1.8 m。
- Gazebo world：ENU。

视觉位姿使用完整刚体链：

```text
T_local_ned_deck_landing_up
= T_local_ned_body_frd
* T_body_frd_camera_optical
* T_camera_optical_marker
* T_marker_deck_landing_up
```

控制器检查 PX4 odometry `pose_frame`，使用 `VehicleLocalPosition.ref_lat/ref_lon/ref_alt` 建立 WGS84/local NED 参考。图像时间与 PX4 时间先映射到统一 ROS 时间域，再从 `VehiclePoseHistory` 插值图像采样时刻的机体位姿，避免把当前姿态错误用于历史图像。

## 5. 引导和视觉接管

GNSS 会合路径会：

- 校验 WGS84 输入、年龄、跳变和速度。
- 将船舶位置转换为 PX4 local NED。
- 限制位置目标变化率和搜索半径。
- 以实时船舶位置为 ArUco 搜索中心。

视觉接管会：

- 校验完整外参、消息 frame、观测顺序和测量跳变。
- 比较 GNSS 与视觉甲板位置一致性。
- 对 GNSS 目标和视觉目标做有限时间线性混合。
- 短时丢帧保持并衰减前馈，长时丢失恢复到 GNSS。

## 6. 状态估计和预测

三维常速度 Kalman Filter 使用视觉采样时间更新甲板位置、速度和协方差，处理：

- 重复或倒退时间。
- 异常时间间隔。
- Mahalanobis 离群点。
- 短时预测和长时重初始化。

主要输出：

```text
/landing/estimated_deck_odometry
/landing/predicted_deck_pose
/landing/vertical_state
/landing/raw_relative_height
/landing/relative_vertical_velocity
```

受限预测器限制预测时域和最大位置偏移。垂直估计独立处理低高度甲板 z、相对高度和相对垂直速度；下视相机 FRD 外参为 `[0, 0, 0.14] m`。

节点还实现独立的甲板 6-DoF shadow 误差状态估计器。当前平移是
`uav_centered_ned` 中的 `deck-uav` 相对位移，线速度/加速度是甲板自身
NED 导数；姿态状态为 `R/ω/α`。输入只使用完成图像时间对齐和 Marker
坐标统一后的 ArUco 甲板中心相对位姿，以及同时刻 PX4 NED 速度用于补偿
无人机观测原点移动；不使用 PX4 绝对位置、甲板 GNSS 或 Ground Truth。

名义平移使用白噪声 jerk 常加速度模型，接受观测的最近 `0.30 s` 滤波
位置另做局部常加速度拟合以发布线导数；姿态创新使用 SO(3) 旋转对数，
预测使用中点角速度积分。误差状态和拟合导数都发布有限、半正定协方差。

Shadow 输出为：

```text
/landing/deck_motion_shadow/state              nav_msgs/msg/Odometry
/landing/deck_motion_shadow/trajectory         trajectory_msgs/msg/MultiDOFJointTrajectory
/landing/deck_motion_shadow/status             std_msgs/msg/String
/landing/deck_motion_shadow/trusted_horizon_s  std_msgs/msg/Float64
```

轨迹在每条消息发布时把无人机原点冻结为 `uav_origin_ned`，以 `0.05 s`
采样到最后视觉样本后的 `1.0 s`；累计外推不超过 `0.5 s` 的区间由独立
可信时域话题标记。非法输入、时间倒退、离群、翻转姿态和长期丢失不会刷新可信
时域。该 shadow 在 `TrajectorySetpoint` 发布后运行，未接入 MPC、生产位置估计、
着陆窗口、下降参考、触地或状态机。

## 7. 水平跟踪

### 7.1 默认规则式控制

默认模式使用预测甲板位置、甲板速度前馈、位置反馈和相对速度阻尼。相对速度增益根据估计甲板加速度连续调度，以兼顾匀速与换向场景。

控制器对位置目标、速度前馈、加速度前馈和目标变化率分别限幅。观测变旧时前馈平滑衰减，严重失效时进入恢复路径。

### 7.2 可选相对 MPC

显式选择 `tracking.mode=RELATIVE_MPC` 后启用四状态二维相对 MPC：

- 状态为水平位置误差和相对速度。
- 输出为 `TrajectorySetpoint.acceleration[x,y]` 前馈。
- 固定稀疏 QP 使用 OSQP `v1.0.0` 与 OsqpEigen `v0.11.2`。
- 支持输入、速度和控制增量约束、warm start、求解耗时与状态诊断。

任一输入非法、求解失败、超时或输出非有限时，本周期使用并行维护的完整规则式输出。进入接触敏感终端阶段后主动切换为 `TERMINAL_RULE_BASED_TRACKING`，该切换不是 solver failure。

## 8. 着陆窗口与相对下降

着陆窗口联合检查：

- Marker 可见性和观测年龄。
- 水平误差和水平相对速度。
- 甲板倾角。
- 状态估计与预测有效性。
- 相对高度。

窗口包含进入/退出迟滞和连续满足时间，并发布拒绝原因。相对下降使用甲板相对高度而不是固定世界高度；高、中、低三段下降速率独立限幅，甲板垂直速度默认作为前馈。

窗口短时关闭时暂停下降；严重失效时恢复到更高相对高度。恢复后锁止再次下降，必须重新完成视觉接管或重启任务才能解除。

## 9. 最终下降与触地

最终下降必须显式启用，并要求相对下降已启用且测试高度严格为 `0.50 m`。下降参考分为接近、近接触和安全终端段，最低命令相对高度为 `0.05 m`。

`TouchdownDetector` 联合：

- PX4 land detector。
- 视觉相对高度。
- 垂直相对速度。
- 动态平台水平相对速度。
- 在线视觉甲板平面给出的滑橇间隙证据。

视觉高度不能单独确认触地。候选必须连续满足并带迟滞；候选期间冻结下降参考，确认后进入 `TOUCHDOWN_HOLD` 并锁存。

升沉甲板接触后保持甲板相对高度和相对垂直速度。固定正 `+2° roll/pitch` 还可显式启用终端接触稳定化：

- 甲板法向到有限水平加速度偏置的平滑映射。
- 接触锚点和中心顺应。
- 切向速度阻尼。
- 候选/保持阶段的受限参考预压与向下加速度预压。
- 姿态与角速度安全监视器。

该功能继续使用 PX4 Offboard position setpoint，不发送 attitude setpoint。

## 10. 状态机与恢复

主路径：

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
→ WAIT_LANDING_WINDOW
→ DESCEND
→ FINAL_DESCENT
→ TOUCHDOWN_CANDIDATE_HOLD
→ TOUCHDOWN_HOLD
→ DONE
```

恢复路径包括 `RECOVER_TO_GNSS`、`RECOVER_CLIMB` 和 `ABORT`。终端触地开始后发生恢复视为本轮失败，不能用再次下降覆盖首次失败。

## 11. 实验与结果

实验工具支持单轮、顺序批量、seed 展开、resume、失败分类、轻量/诊断 Bag、参数快照、离线重评测和聚合。

6-DoF shadow 另有专用 evaluator 与冻结的
`static/rollpitch/combined/rigid_body_motion × seed 1/2/3` 顺序运行脚本。
相对方案约 `5 m` 正式矩阵见
[`results/deck_motion_shadow_relative_5m_20260809`](../../results/deck_motion_shadow_relative_5m_20260809/manifest.json)，
实际相对高度 RMSE 为 `5.175–5.346 m`：

- `12/12` 均保持 `DESCEND/contact/penetration/NAV_LAND/Disarm = 0`；时间同步错误、
  Ground Truth 无效样本和非有限输出也均为 `0`。
- 有效覆盖率全部为 `100%`。当前法向 RMSE/P95 范围为
  `0.122°–0.249° / 0.200°–0.403°`，对应门为 `12/12`。
- `0.5 s` 预测水平/垂直位置 P95 为 `0.029–0.142 m / 0.037–0.045 m`，
  法向 P95 为 `0.457°–1.145°`，三项门均为 `12/12`；wrapped yaw 为 `11/12`。
- `0.5 s` 水平速度、垂直速度和角速度 P95 范围为
  `0.081–0.364 m/s`、`0.088–0.124 m/s`、`0.856°/s–2.930°/s`，门通过数分别为
  `6/12`、`2/12`、`3/12`。
- 全部硬门总结果为 `2/12`（static seed1/3）。失败集中在未来 twist，而不是
  约 `5 m` 的当前位姿或未来位置，因此本轮未触发约 `3 m` 像素分辨率对照。
  结果不授权 NMPC、动态姿态下降或真实接触。

冻结数据的主要结论：

- smoke：`20/27`，7 个失败均为 `SAFETY_GATE_FAILURE`。
- static/constant02 正式基线：`40/40`。
- 正式消融：`60/60`。
- 关闭组合：`30 NOT_APPLICABLE`，不进入失败分母。
- 正式总体观测成功率：`100/100`，Wilson 95% CI `[0.963, 1.000]`。

完整论文统计和哈希见[论文结果](../results/PAPER_RESULTS.md)与[数据来源](../results/DATA_PROVENANCE.md)。

## 12. 已验证范围

- 静止、`0.2 m/s`、`0.4 m/s` 和 XY 正弦安全高度跟踪。
- 静止、水平匀速和升沉相对下降及真实接触。
- 四状态相对 MPC 的安全高度、下降、接触与规则式回退。
- 固定正 `+2° roll/pitch` 的终端接触稳定化和 10 秒保持。
- 全工作区当前实现记录为 `361 tests, 0 failures, 0 skipped`；相对 shadow
  正式 12 轮的安全隔离为 `12/12`，全性能硬门为 `2/12`。

固定正倾角的成功不能外推到负倾角、动态 `rollpitch` 或 `combined`。

## 13. 安全边界

- 默认 `descent.enabled=false`、final descent disabled。
- `NAV_LAND / Automatic Disarm = 0 / 0`。
- 负固定倾角、动态姿态、combined 和 rigid_body_motion 禁止下降与接触。
- Ground Truth 不得进入控制器、窗口、估计器或状态机。
- `--environment legacy` 仍是默认路径；`--environment marine` 必须显式选择。
- Marine M2 只允许 GNSS rendezvous、视觉捕获、安全高度跟踪和 deck-motion shadow；相对下降、最终下降和任意 terminal-contact stabilization 会在启动前拒绝。
- Marine M2 已引入固定版本的 VRX WAM-V visual 和 VRX-style ocean visual，但没有启用动态 WaveVisual、wave-driven vessel dynamics、JONSWAP/PM 船体响应、RAO、Buoyancy、Hydrodynamics、wind、current 或 CFD。`MotionProfile` 仍是唯一 vessel motion source。
- 非有限 setpoint、非法四元数、无效 PX4 frame 和过期观测必须拒绝。
- 实机自动解锁不属于默认配置；所有自动动作仅面向 SITL 并需显式授权。
