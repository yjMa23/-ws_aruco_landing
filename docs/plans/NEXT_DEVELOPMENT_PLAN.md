# 移动船舶无人机自主降落下一阶段完整开发计划

## 1. 文档目的

本文档用于指导 `ws_aruco_landing` 在现有静态 ArUco 精准降落基线和水平移动甲板仿真的基础上，继续实现完整的传统方法移动船舶自主降落基线。

本计划重点补充以下关键流程：

```text
船舶 GNSS / 遥测粗引导
→ 无人机远距离会合
→ 跟踪到移动甲板上方
→ ArUco 稳定捕获
→ GNSS 到视觉平滑接管
→ 视觉状态估计与运动预测
→ 移动甲板水平跟踪
→ 规则式着陆窗口判断
→ 相对高度分阶段下降
→ 持续跟踪直到触地
→ 丢失恢复、安全中止和批量评测
```

本文档是传统基线的阶段执行计划。当前 `P0`～`P8C fixed T1` 已完成代码、测试和真实 PX4 SITL 验收：P7-lite 真实 3+3 冒烟为 6/6 PASS；P8A 升沉触地 H1/H2 均为 3/3 PASS；P8B 水平相对 MPC 完成固定依赖、生产实现、P4.7 安全回退和严格顺序验收；P8C-3 水平机体失败证据完整保留，P8C-4 在固定正 `+2° roll/pitch` 完成分级验证。P9 统一评测第一版的 smoke、正式 baseline、正式消融与聚合均已完成：`20/27 + 40/40 + 60/60 = 120/127`，7 个失败全部是 smoke `SAFETY_GATE_FAILURE`；30 个被关闭的正式槽位准确记录为 `NOT_APPLICABLE`。

---

## 2. 当前项目状态

### 2.1 已完成阶段

| 阶段 | 状态 | 主要成果 |
| --- | --- | --- |
| `P0` 仓库整理与静态基线冻结 | 已完成 | `aruco_detector` 纳入主仓库，工作区可统一构建，标签 `baseline-static-v0.1` |
| `P1` 水平移动甲板仿真 | 已完成 | 静止、匀速、XY 正弦甲板，确定性 reset，Ground Truth，标签 `baseline-moving-deck-v0.1` |
| `P2.0` 项目状态和设计文档同步 | 已完成 | 更新项目阶段、GNSS 到视觉流程、Ground Truth 边界和默认任务 |
| `P2A-1` 坐标契约确认 | 已完成 | 新增 `docs/reference/COORDINATE_FRAMES.md`，核对 PX4、Gazebo、相机和地理原点语义 |
| `P2A-2～4` 纯数学模块与测试 | 已完成 | 实现 `coordinate_transform`、`geodetic_converter` 和 GTest |
| `P2B` 船舶 GNSS 传感器仿真 | 已完成 | 实现理想/含噪 GNSS、ENU 速度、固定采样/延迟/丢包、确定性 reset 和端到端冒烟验证 |
| `P2C` GNSS 会合与移动甲板上方粗跟踪 | 已完成 | 实现 WGS84→local NED、GNSS 校验、会合目标限幅、移动中心搜索和超时回退 |
| `P2D` GNSS—视觉接管与下降前恢复 | 已完成 | 实现完整相机外参、Marker local NED、线性接管、视觉跟踪、短时保持和长时 GNSS 恢复 |
| `P3` 视觉状态估计与短时预测 | 已完成 | 实现三维常速度 Kalman Filter、离群/时间异常处理、位置速度协方差和受限短时预测 |
| `P4` 安全高度移动甲板水平跟踪 | 已完成 | 实现四种跟踪模式、预测位置目标、水平速度前馈、位置/速度/加速度限制和丢失恢复，并完成静止、匀速和正弦真实 PX4 SITL 验收 |
| `P4.5` 实验可复现与视觉时间对齐 | 已完成 | 新增 rosbag 自动评测、统一 `use_sim_time`、PX4 位姿历史、PX4→ROS 时间映射、图像时刻位姿插值和四场景 SITL 回归 |
| `P4.6` 正弦参数优化 | 已完成 | 完成预测时域、控制模式和相对速度阻尼扫描，识别固定增益无法兼顾匀速和换向 |
| `P4.7` 加速度感知增益调度 | 已完成 | 统一参数在 0.4 m/s 匀速达到 `0.0554 m` RMSE、正弦达到 `0.3490 m` RMSE |
| `P5A` 动态甲板与规则式着陆窗口 | 已完成 | S3/S4/S5、Marker 法向量倾角估计、迟滞窗口、`WAIT_LANDING_WINDOW` 和五场景 SITL 验收 |
| `P5B` 相对甲板高度分阶段下降 | 已完成 | 分段下降、窗口暂停、恢复爬升、恢复后重新授权锁止和 `0.50 m` 安全高度验收 |
| `P5C` 低高度垂直状态估计与标定 | 已完成 | 相机 z 外参修正、独立垂直估计、低高度标定和甲板垂直速度前馈 |
| `P6A` 多源触地候选与确认 | 已完成 | PX4 land detector、视觉高度和垂直速度联合判据及三类负向 SITL 验收 |
| `P6B` 最终下降与真实接触 | 已完成 | 分段最终下降、动态平台证据、多尺度 Marker、触地候选/确认和接触保持 |
| `P7-lite` 批量评测基线 | 已冻结 | 单轮、批量、resume、失败分类和聚合；真实 3+3 冒烟 6/6 PASS |
| `P8A` 升沉甲板触地 | VALIDATION PASS | H1/H2 各 3/3，真实接触与 10 秒相对甲板保持通过 |
| `P8B` 水平相对 MPC | VALIDATION PASS | 4 状态 MPC、约束、warm start、完整 P4.7 回退和终端 handoff；三类验收共 27/27 PASS |
| `P8C-1` 固定倾角安全高度 | VALIDATION PASS | ±2° 四方向 12/12、static 1/1、共享路径回归 3/3；完整法向最差 RMSE/P95 `0.702°/1.353°` |
| `P8C-2` 固定正倾角安全下降 | SAFE DESCENT PASS | +2° roll/pitch 3+3、static/constant02 2/2；最差水平 RMSE/max `0.020931/0.068704 m`，最低真实滑橇间隙 `0.210051 m` |
| `P8C-3` 水平机体触地诊断 | FAILURE EVIDENCE PRESERVED | seed2 滑移硬门失败及姿态发散、离板、恢复证据完整归档，未放宽阈值、未删失败轮 |
| `P8C-4` 终端接触稳定化 | VALIDATION PASS | Offboard position 模式内的法向整形、锚点顺应、切向阻尼和受限预压；shadow 12/12、rehearsal 6/6、fixed T1 touchdown 6/6、旧路径 9/9 |
| `P8C fixed T1` | VALIDATION PASS | `P8C-4 VALIDATION PASS / P8C T1 VALIDATION PASS / P8C-3 DESIGN GATE CLOSED` |

### 2.2 当前已有 ROS 2 包

```text
src/aruco_detector
src/aruco_precision_landing_cpp
src/moving_deck_sim
```

### 2.3 当前控制器能力

`aruco_precision_landing_cpp` 当前生产路径已经具备：

- PX4 Offboard 预发布、自动切换 Offboard、解锁和起飞。
- 使用 PX4 `VehicleLocalPosition.ref_lat/ref_lon/ref_alt` 建立 local NED 地理参考。
- 订阅船舶 `NavSatFix` 和 ENU 速度，拒绝非法、超时和大跳变输入。
- 使用受限位置目标飞到移动甲板 GNSS 上方，并围绕实时船舶中心搜索 ArUco。
- 使用 `T_local_ned_body_frd * T_body_frd_camera_optical * T_camera_optical_marker` 转换完整视觉位姿。
- 检查 GNSS—视觉一致性、ArUco frame、采样顺序和视觉测量跳变。
- 实现 `VISUAL_HANDOVER`、`TRACK_TARGET`、`WAIT_LANDING_WINDOW`、`DESCEND`、`TEST_HEIGHT_HOLD`、`RECOVER_CLIMB` 和 `RECOVER_TO_GNSS`。
- 使用 Marker 向上法向量估计甲板 roll、pitch 和总倾角。
- 使用位置误差、相对速度、视觉年龄、预测有效性、甲板倾角和相对高度判断迟滞着陆窗口。
- 着陆窗口打开后可显式启用相对高度分阶段下降，窗口恶化时暂停，严重失效时恢复到更高相对高度。
- 发布 GNSS 和 Marker local NED 调试位姿及当前引导来源。
- 使用视觉采样时间运行三维常速度 Kalman Filter，估计甲板位置、速度和协方差。
- 发布 `/landing/estimated_deck_odometry` 和 `/landing/predicted_deck_pose`。
- 处理重复/倒退时间、离群点、大观测间隔和长时重初始化。
- 支持原始视觉、估计位置、估计位置+速度前馈、预测位置+速度前馈四种模式。
- 默认将受限预测位置写入 PX4 position setpoint，将甲板速度和相对速度阻尼写入水平 velocity feedforward。
- 实现位置目标、前馈速度和前馈加速度限制，以及短时丢失衰减和长时 GNSS 恢复。
- 修正 PX4 下视相机 z 外参并实现独立 `VerticalStateEstimator`。
- 在相对下降状态默认使用甲板垂直速度和相对高度参考变化速度前馈。
- 发布 `/landing/vertical_state`、`/landing/raw_relative_height` 和 `/landing/relative_vertical_velocity`。
- 订阅 PX4 `VehicleLandDetected`，联合视觉低高度和垂直速度运行多源触地候选与确认。
- 发布 `/landing/touchdown_status`、`/landing/touchdown_evidence`、`/landing/touchdown_candidate_duration` 和 `/landing/touchdown_confirmed`。
- P6A 负向阶段只并行评估触地证据；显式启用 P6B 后，候选会冻结最终下降参考，确认后进入 `TOUCHDOWN_HOLD`，仍不发送 `NAV_LAND` 或 Disarm。
- 使用四尺度有状态 Marker 选择器和 `near=0.02 m` 项目相机模型持续观测至接触。
- 在升沉甲板触地后按甲板相对高度和垂直速度执行接触保持。
- 显式选择 `RELATIVE_MPC` 时使用四状态水平相对 MPC；求解异常和终端阶段自动回退完整 P4.7。

旧静态对中、下降和 `NAV_LAND` 代码仍保留用于历史基线参考，但从当前主路径不可达。相对下降默认关闭，`enable_auto_land=false`。

### 2.4 当前核心缺口

- P8A 已完成升沉甲板最终下降、真实接触、相对垂直速度语义和接触后相对保持，H1 3/3、H2 3/3 PASS。
- P8B 已完成综述、计划、固定 OSQP/OsqpEigen 依赖、4 状态水平相对 MPC、约束、warm start、完整 P4.7 fallback、终端安全 handoff、诊断、`271` 项全工作区测试和严格顺序真实 SITL；安全高度 15/15、下降 6/6、最终代码真实触地 6/6 PASS，状态为 `VALIDATION PASS`。
- P8C fixed T1 已完成：P8C-3 失败证据保留，P8C-4 终端接触稳定化与固定正 `+2° roll/pitch` 真实触地验收通过，设计门关闭。该结论不能外推到负倾角、动态 roll/pitch 或 combined。
- P9 统一批量评测第一版已完成真实实验和聚合；下一步转入论文结果复核、统计置信区间与图表定稿，不扩大当前触地安全边界。
- 没有触地后的 Land/Disarm 授权和最终恢复策略；当前继续保持 `NAV_LAND / Disarm = 0 / 0`。

### 2.5 2026-07-29 P7 第一版状态

已新增：

```text
scripts/run_single_experiment.py
scripts/run_batch_experiments.py
scripts/aggregate_results.py
scripts/p7_experiment_utils.py
config/experiments/p7_smoke.yaml
config/experiments/p7_baseline.yaml
docs/plans/P7_BATCH_EVALUATION_PLAN.md
```

第一版只支持 `static` 和 `constant02`，顺序执行，单轮失败后继续。成功判据来自 `/landing/state` 的 `TOUCHDOWN_HOLD` 连续保持至少 10 秒，并由 `evaluate_p6b_touchdown.py` 复核；不使用固定 sleep 判定成功。真实执行还修复了 ROS CLI daemon 失效、PX4 就绪探测、状态监控、episode 计时和失败分类问题。2026-07-30 真实 3+3 冒烟已完成，6/6 PASS、0 failure，结果目录为 `results/p7_smoke_terminal_20260730/`；P7-lite 已冻结，20+20 配置延后到 P9。

---

## 3. 总体设计原则

### 3.1 分层引导

系统采用两级引导：

| 层级 | 数据来源 | 作用 | 精度要求 |
| --- | --- | --- | --- |
| 粗引导 | 船舶 GNSS / 遥测 | 远距离会合、将无人机带入视觉范围 | 米级 |
| 精引导 | ArUco 视觉位姿 | 精确对中、速度匹配、下降和触地 | 厘米级 |

GNSS 不负责最终精确降落，ArUco 不负责远距离搜索。

### 3.2 Ground Truth 隔离

控制器禁止直接订阅：

```text
/simulation/deck/ground_truth
```

Ground Truth 只能用于：

- GNSS 传感器仿真节点的内部输入。
- 实验评测和误差统计。
- 调试验证，但不得进入控制决策链。

控制器只允许订阅经过传感器模型处理后的：

```text
/deck/gps/fix
/deck/gps/velocity
```

以及 ArUco、PX4 和配置参数。

### 3.3 先建立坐标契约，再实现控制

所有控制和估计前必须先固定：

```text
camera_optical
base_link_frd
local_ned
world_enu
WGS84
```

禁止继续在业务代码中散落手写正负号转换。

### 3.4 GPS 和视觉不同时拉扯控制目标

- GNSS 会合阶段：控制目标来自 GNSS。
- 视觉接管阶段：短时间平滑过渡。
- 视觉跟踪阶段：控制目标主要来自视觉估计器。
- GNSS 仅作为恢复和安全冗余信息。

### 3.5 最终下降必须持续水平跟踪

移动甲板场景下，不允许在较高高度直接切换 PX4 `NAV_LAND` 并停止甲板水平跟踪。

最终下降应保持 Offboard 水平跟踪，直到触地得到确认。

### 3.6 每个阶段独立验收

每个阶段必须同时完成：

- 代码实现。
- 参数声明和校验。
- YAML 配置。
- 单元测试。
- 仿真冒烟测试。
- 验收记录。
- Git 提交或阶段标签。

未完成前一阶段验收，不推进下一阶段。

---

## 4. 总体系统架构

```text
Gazebo moving deck Ground Truth
              │
              ├──────────────→ landing_evaluation
              │                    仅评测
              │
              ↓
       deck_gnss_simulator
       噪声 / 延迟 / 丢包
              │
              ├── /deck/gps/fix
              └── /deck/gps/velocity
                         │
                         ↓
PX4 状态 ─────────→ px4_aruco_landing_node
                         ↑
相机图像 → aruco_detector
             │
             ├── /aruco/pose
             └── /aruco/visible

px4_aruco_landing_node 内部：

GNSS 转换与状态校验
→ GNSS 会合引导
→ ArUco 完整坐标变换
→ 视觉状态估计
→ 短时预测
→ 导航源接管管理
→ 跟踪控制
→ 着陆窗口
→ 分阶段下降
→ 触地 / 恢复 / 中止
```

---

## 5. 计划新增或扩展的接口

### 5.1 船舶 GNSS 仿真输出

| 话题 | 类型 | 坐标系 / 语义 |
| --- | --- | --- |
| `/deck/gps/fix` | `sensor_msgs/msg/NavSatFix` | WGS84 纬度、经度、海拔及协方差 |
| `/deck/gps/velocity` | `geometry_msgs/msg/TwistStamped` | 第一版统一使用 local ENU 或明确标记的 NED 速度 |

第一版推荐将速度话题定义为 `world_enu`，并在坐标转换模块集中转换到 NED。

### 5.2 控制器调试输出

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/landing/deck_gnss_pose_ned` | `geometry_msgs/msg/PoseStamped` | GNSS 转换后的甲板粗位置 |
| `/landing/marker_pose_ned` | `geometry_msgs/msg/PoseStamped` | ArUco 转换后的甲板视觉位置 |
| `/landing/estimated_deck_odometry` | `nav_msgs/msg/Odometry` | 滤波后的甲板位置和速度 |
| `/landing/predicted_deck_pose` | `geometry_msgs/msg/PoseStamped` | 控制时刻预测位置 |
| `/landing/guidance_source` | `std_msgs/msg/String` | `GNSS`、`BLENDING`、`VISION`、`PREDICTION`、`NONE` |
| `/landing/state` | `std_msgs/msg/String` | 当前状态机状态 |
| `/landing/target_pose` | `geometry_msgs/msg/PoseStamped` | 当前发送给 PX4 的 NED 位置目标 |

### 5.3 PX4 输入

继续使用：

```text
/fmu/out/vehicle_status
/fmu/out/vehicle_local_position
/fmu/out/vehicle_odometry
```

P6 触地确认阶段再增加：

```text
/fmu/out/vehicle_land_detected
```

P2C 已确认不需要额外订阅 `VehicleGlobalPosition`；WGS84 参考直接来自
`VehicleLocalPosition.ref_lat/ref_lon/ref_alt`。具体 PX4 消息字段必须以当前安装的
`px4_msgs` 版本为准，不在代码中假设字段名称。

---

## 6. 坐标系与变换链

### 6.1 坐标系

#### `camera_optical`

```text
x：图像向右
y：图像向下
z：镜头前方
```

#### `base_link_frd`

```text
x：机头前方
y：机体右方
z：机体下方
```

#### `local_ned`

```text
x：North
y：East
z：Down
```

#### `world_enu`

```text
x：East
y：North
z：Up
```

#### `WGS84`

```text
latitude
longitude
altitude
```

### 6.2 视觉位姿变换

```text
T_local_ned_marker
=
T_local_ned_body_frd
*
T_body_frd_camera_optical
*
T_camera_optical_marker
```

其中：

- `T_camera_optical_marker` 来自 ArUco PnP。
- `T_body_frd_camera_optical` 来自实际 PX4 模型相机安装外参。
- `T_local_ned_body_frd` 来自 PX4 `VehicleOdometry`，并检查 `pose_frame`。

### 6.3 GNSS 位置变换

推荐流程：

```text
WGS84 deck fix
→ 以 PX4 home / map origin 建立局部切平面
→ local ENU
→ local NED
```

需要明确：

- Gazebo `<spherical_coordinates>` 原点。
- PX4 home 经纬度和 Gazebo 原点是否一致。
- 海拔是椭球高还是仿真局部高。
- 初版是否仅使用 GNSS 水平位置。

### 6.4 第一版 GNSS 高度策略

普通 GNSS 垂直误差较大。第一版建议：

- GNSS 仅用于水平会合。
- 会合高度使用配置的安全高度。
- ArUco 接管后，再使用视觉相对高度。
- 不直接使用带噪 GPS 海拔驱动最终下降。

---

## 7. 推荐状态机

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
→ TOUCHDOWN_CONFIRM
→ DONE

恢复与异常：
RECOVER_TO_GNSS
RECOVER_CLIMB
ABORT
```

### 7.1 `WAIT_DECK_GNSS`

进入条件：

- 无人机完成起飞。

行为：

- 保持起飞位置和安全高度。
- 等待 PX4 定位和船舶 GNSS 有效。

转出条件：

- 船舶 GNSS 连续有效一段时间。
- 坐标转换原点已初始化。

异常：

- 超时后进入 `ABORT` 或继续安全悬停，由参数决定。

### 7.2 `RENDEZVOUS_GNSS`

行为：

- 根据船舶 GNSS 粗位置实时更新水平目标。
- 保持配置的会合高度。
- 可加入船舶 GNSS 速度前馈，但第一版可先只做限速位置跟踪。

转出条件：

- 无人机与 GNSS 目标水平距离小于 `rendezvous_radius`。
- 状态持续稳定。

异常：

- GNSS 短时丢失：保持最后安全目标或限速预测。
- GNSS 长时丢失：保持、返航或 `ABORT`。

### 7.3 `ACQUIRE_ARUCO`

行为：

- 继续围绕实时 GNSS 目标跟随移动甲板。
- 保持安全高度。
- 等待 ArUco 连续稳定检测。
- GPS 误差较大时执行小范围搜索。

搜索中心：

```text
实时更新的 GNSS 目标位置
```

第一版搜索轨迹：

```text
中心
→ 北偏移
→ 东偏移
→ 南偏移
→ 西偏移
→ 中心
```

不先实现复杂螺旋搜索。

### 7.4 `VISUAL_HANDOVER`

进入条件：

- ArUco 连续稳定可见。
- 视觉位姿新鲜。
- GNSS 和视觉目标偏差未超过接管阈值。

行为：

```text
target = (1 - alpha) * gnss_target + alpha * visual_target
alpha: 0 → 1
```

转出条件：

- 混合权重达到 1。
- 视觉仍然有效。

异常：

- 视觉中断：返回 `ACQUIRE_ARUCO`。
- 目标跳变过大：拒绝接管。

### 7.5 `TRACK_TARGET`

行为：

- 使用视觉甲板估计和短时预测。
- 保持安全相对高度。
- 暂不下降。
- 计算水平误差和水平相对速度。

转出条件：

- 跟踪稳定。
- 满足进入着陆窗口等待阶段的前置条件。

### 7.6 `WAIT_LANDING_WINDOW`

行为：

- 保持水平跟踪。
- 判断：
  - Marker 新鲜度。
  - 水平位置误差。
  - 水平相对速度。
  - 甲板横滚、纵摇。
  - 相对高度。
  - 状态估计有效性。
- 使用持续时间和迟滞，避免状态抖动。

### 7.7 `DESCEND`

行为：

- 中等高度分阶段下降。
- 持续视觉水平跟踪。
- 条件恶化时暂停下降。

### 7.8 `FINAL_DESCENT`

行为：

- 更低下降速度。
- 持续水平跟踪。
- 不直接退出 Offboard。
- 视觉短时丢失时停止下降并短时预测。

### 7.9 `TOUCHDOWN_CONFIRM`

行为：

- 结合 `VehicleLandDetected`、高度、垂直速度和持续时间确认触地。
- 触地确认后再进入完成或受控 Disarm 流程。

### 7.10 `RECOVER_TO_GNSS`

适用：

- 尚未下降或处于高位跟踪。
- ArUco 长时间丢失。

行为：

- 切换回 GNSS 粗跟踪。
- 返回 `RENDEZVOUS_GNSS` 或 `ACQUIRE_ARUCO`。

### 7.11 `RECOVER_CLIMB`

适用：

- 已经下降。
- 视觉长时间丢失。

行为：

- 停止下降。
- 优先垂直上升到恢复高度。
- 到达安全高度后切回 GNSS 会合。

### 7.12 `ABORT`

触发示例：

- PX4 状态无效。
- 位置或姿态出现 NaN / Inf。
- GNSS 和视觉同时长时间无效。
- 控制目标跳变过大。
- 达到最大任务时长。
- 恢复次数超过限制。

行为：

- 停止下降。
- 进入安全悬停、返航或配置的安全模式。

---

## 8. 分阶段开发计划

# P2.0：同步项目状态和设计文档

## 目标

让仓库文档与当前实际进度一致，并冻结下一阶段接口和状态机。

## 修改范围

```text
AGENTS.md
docs/plans/TRADITIONAL_BASELINE_PLAN.md
docs/plans/NEXT_DEVELOPMENT_PLAN.md
README.md（仅在需要增加入口时修改）
```

## 任务

- 标记 P0、P1 已完成。
- 将默认下一任务改为 P2。
- 增加 GNSS 粗引导和视觉接管总体流程。
- 更新状态机。
- 明确 Ground Truth 隔离规则。
- 明确 GNSS 不参与最终精降。

## 验收

- 只修改文档。
- 文档之间没有阶段描述冲突。
- 未创建算法代码。

---

# P2A：完整坐标变换与地理坐标转换

## 目标

建立后续 GNSS 会合和视觉跟踪共用的可靠坐标基础。

## 计划文件

```text
src/aruco_precision_landing_cpp/
├── include/aruco_precision_landing_cpp/
│   ├── coordinate_transform.hpp
│   └── geodetic_converter.hpp
├── src/
│   ├── coordinate_transform.cpp
│   └── geodetic_converter.cpp
└── test/
    ├── coordinate_transform_test.cpp
    └── geodetic_converter_test.cpp

docs/reference/COORDINATE_FRAMES.md
```

## 任务 A：确认实际坐标语义

- 检查 PX4 `VehicleOdometry`：
  - 四元数元素顺序。
  - 旋转方向。
  - `pose_frame`。
  - 机体系 FRD 语义。
- 检查 PX4 下视相机模型的实际安装位姿。
- 确认 ArUco 输入图像 `frame_id`。
- 确认 Gazebo world 使用 ENU。
- 确认 Gazebo spherical origin 和 PX4 home 的关系。

## 任务 B：实现视觉刚体变换

核心公式：

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

- 使用 Eigen 或等价可靠数学类型。
- 输入和输出坐标系写入 Doxygen 注释。
- 检查四元数有限性和范数。
- 无效输入返回失败，不输出错误设定点。
- 不依赖 ROS 节点即可测试。

## 任务 C：实现 WGS84 / ENU / NED 转换

至少支持：

```text
WGS84 → local ENU
local ENU → local NED
local NED → WGS84（测试和调试使用）
```

第一版可采用局部切平面近似或成熟地理转换公式，但必须：

- 明确有效距离范围。
- 编写已知点测试。
- 验证正东、正北、升高的符号方向。

## 单元测试

### 视觉变换

- 单位变换。
- 相机平移。
- 相机固定旋转。
- 无人机偏航 `0°、90°、180°、-90°`。
- 无人机横滚和俯仰。
- 组合姿态。
- NaN、Inf、零四元数、未归一化四元数。

### 地理转换

- 原点转换为零。
- 向东位移后 ENU x 为正。
- 向北位移后 ENU y 为正。
- 升高后 ENU z 为正。
- ENU 到 NED：

```text
NED.x = ENU.y
NED.y = ENU.x
NED.z = -ENU.z
```

- 正向和逆向转换闭环误差。

## 验收标准

- 所有单元测试通过。
- 代码中不再需要两个相机正负号参数完成主要变换。
- 暂不修改下降逻辑。
- 暂不实现 Kalman Filter。

## 建议标签

```text
baseline-coordinate-transform-v0.1
```

---

# P2B：船舶 GNSS 传感器仿真

## 目标

将甲板 Ground Truth 转换为具有真实传感器特征的 GNSS / 遥测输出，供控制器进行粗引导。

## 计划文件

```text
src/moving_deck_sim/
├── src/deck_gnss_simulator.cpp
├── config/gnss_ideal.yaml
├── config/gnss_noisy.yaml
├── launch/deck_gnss_sim.launch.py
└── test/deck_gnss_model_test.cpp
```

也可以先将纯数学部分拆为：

```text
include/moving_deck_sim/gnss_sensor_model.hpp
src/gnss_sensor_model.cpp
```

ROS 节点只负责收发消息。

## 输入

```text
/simulation/deck/ground_truth
nav_msgs/msg/Odometry
world_enu
```

## 输出

```text
/deck/gps/fix
sensor_msgs/msg/NavSatFix

/deck/gps/velocity
geometry_msgs/msg/TwistStamped
```

## 传感器模型参数

建议初始参数：

```yaml
publish_rate_hz: 5.0
horizontal_noise_std_m: 0.8
vertical_noise_std_m: 1.5
velocity_noise_std_mps: 0.1
latency_ms: 100
packet_drop_probability: 0.0
random_seed: 1
```

另提供理想配置：

```yaml
horizontal_noise_std_m: 0.0
vertical_noise_std_m: 0.0
velocity_noise_std_mps: 0.0
latency_ms: 0
packet_drop_probability: 0.0
```

## 实现顺序

1. 理想 GNSS：只完成坐标转换和降频发布。
2. 固定种子高斯噪声。
3. 固定延迟队列。
4. 可配置丢包。
5. 发布协方差和有效状态。

## 约束

- GNSS 节点可以订阅 Ground Truth。
- 降落控制器禁止订阅 Ground Truth。
- 随机噪声必须可复现。
- reset 后随机序列和初始状态必须可配置地重复。
- 时间戳表示原始采样时间或发布时刻必须明确，推荐保留采样时间。

## 单元测试

- 零噪声时转换结果正确。
- 固定种子输出可重复。
- 发布频率抽样逻辑正确。
- 延迟队列顺序正确。
- 丢包概率边界 `0` 和 `1`。
- 协方差与配置一致。
- NaN Ground Truth 不发布有效 GNSS。

## 仿真验收

- 静止场景 GNSS 均值接近真值。
- 匀速场景位置连续。
- 正弦场景轨迹形状正确。
- 开启噪声后误差统计与配置量级一致。
- reset 后输出行为确定。

## 建议标签

```text
baseline-deck-gnss-sim-v0.1
```

---

# P2C：GNSS 会合与移动甲板上方跟踪

## 目标

使无人机不依赖 ArUco 视野，能够先根据船舶 GNSS 飞到移动甲板上方并持续粗跟踪。

## 主要修改

```text
src/aruco_precision_landing_cpp/
├── include/aruco_precision_landing_cpp/px4_aruco_landing_node.hpp
├── src/px4_aruco_landing_node.cpp
├── config/px4_aruco_landing.yaml
├── launch/px4_aruco_landing.launch.py
└── test/（增加纯逻辑测试）
```

第一版可保留单 ROS 节点，不急于拆多个节点。

## 新增输入

```text
/deck/gps/fix
/deck/gps/velocity
/fmu/out/vehicle_local_position
```

P2C 实际使用 `VehicleLocalPosition.ref_lat/ref_lon/ref_alt` 作为 local NED 地理参考，不额外订阅 `VehicleGlobalPosition`。

## 新增状态

```text
WAIT_DECK_GNSS
RENDEZVOUS_GNSS
ACQUIRE_ARUCO
```

替换原固定：

```text
GOTO_ARUCO_AREA
WAIT_ARUCO
```

## 会合控制第一版

水平目标：

```text
deck_gnss_position_ned
```

垂直目标：

```text
固定安全会合高度
```

第一版不直接使用 GNSS 垂直高度做精确相对高度控制。

## 目标限幅

必须限制：

- 每周期目标位置变化。
- 最大水平速度。
- 最大加速度或目标变化率。
- GNSS 跳变最大接受距离。
- GNSS 超时。

## 初始建议参数

```yaml
rendezvous_altitude_m: 5.0
rendezvous_radius_m: 2.0
gnss_fix_timeout_s: 1.0
gnss_velocity_timeout_s: 1.0
gnss_stable_duration_s: 1.0
max_gnss_jump_m: 5.0
max_rendezvous_speed_mps: 2.0
max_target_step_m: 0.2
```

## ACQUIRE_ARUCO 搜索

参数建议：

```yaml
search_offset_m: 1.0
search_point_hold_s: 1.0
aruco_acquire_duration_s: 0.5
aruco_pose_timeout_s: 0.3
```

搜索中心始终使用实时 GNSS 目标，而不是固定世界坐标。

## 测试

### 单元测试

- GNSS 未初始化时不进入会合。
- GNSS 超时时不追踪旧位置。
- GNSS 跳变被拒绝。
- 到达会合半径后进入捕获状态。
- 搜索偏移相对移动中心正确。
- 状态转换原因正确记录。

### 仿真验收

场景：

1. 静止甲板。
2. 匀速 `0.2 m/s`。
3. 匀速 `0.4 m/s`。
4. XY 正弦甲板。

要求：

- 无人机能到达甲板 GNSS 上方。
- ArUco 不可见时仍持续跟踪甲板粗位置。
- GNSS 短时丢包不造成目标跳变。
- GNSS 长时丢失后进入安全行为。
- 控制器无 Ground Truth 订阅。

## 本阶段不做

- 不下降。
- 不做 Kalman Filter。
- 不做视觉速度前馈。
- 不做最终触地。

## 实现与验收状态

P2C 已实现并通过纯逻辑测试和合成 PX4 消息状态机冒烟测试。验收记录：

```text
docs/validation/P2C_GNSS_RENDEZVOUS_VALIDATION.md
```

真实 PX4 动力学下的移动甲板飞行闭环仍需在 QGroundControl/心跳和相机插件环境完整后继续验证。

## 建议标签

```text
baseline-gnss-rendezvous-v0.1
```

---

# P2D：ArUco 完整变换与 GNSS—视觉接管

## 目标

在无人机到达移动甲板上方后，稳定捕获 ArUco，并将控制来源从 GNSS 平滑切换到视觉。

## 新增状态

```text
VISUAL_HANDOVER
TRACK_TARGET
RECOVER_TO_GNSS
```

## 视觉接管条件

- ArUco 连续可见达到最小时间。
- ArUco 消息时间戳新鲜。
- 完整坐标变换成功。
- 视觉位置没有 NaN / Inf。
- GNSS 和视觉估计水平偏差小于阈值。
- 当前目标和视觉目标跳变小于阈值。

## 初始建议参数

```yaml
visual_handover_duration_s: 0.5
handover_max_horizontal_difference_m: 3.0
handover_max_target_jump_m: 0.5
visual_loss_short_timeout_s: 0.5
visual_loss_long_timeout_s: 2.0
```

## 接管方式

推荐第一版使用线性平滑：

```text
alpha = clamp(elapsed / handover_duration, 0, 1)
target_xy = (1-alpha) * gnss_target_xy + alpha * visual_target_xy
```

接管期间高度保持不变。

## 恢复逻辑

尚未下降时：

```text
视觉长时丢失
→ RECOVER_TO_GNSS
→ RENDEZVOUS_GNSS 或 ACQUIRE_ARUCO
```

## 调试输出

必须发布：

```text
/landing/deck_gnss_pose_ned
/landing/marker_pose_ned
/landing/guidance_source
```

## 单元测试

- 接管权重从 0 单调增加到 1。
- 视觉跳变超过阈值时拒绝接管。
- 接管中视觉丢失时正确回退。
- 成功接管后来源变为 `VISION`。
- 设定点无超过限制的跳变。

## 仿真验收

- 静止甲板接管稳定。
- 匀速甲板接管稳定。
- 正弦甲板接管稳定。
- 单帧误检不触发接管。
- 视觉断开后能回到 GNSS 会合。
- 接管过程中无人机高度不下降。

## 实现与验收状态

P2D 已实现并通过 55 项工作区测试、完整变换消息级验收、单帧误检验收和视觉丢失恢复验收。记录：

```text
docs/validation/P2D_GNSS_VISION_HANDOVER_VALIDATION.md
```

真实 PX4 动力学下的视觉联合飞行仍需在相机插件环境完整后继续验证。

## 建议标签

```text
baseline-gnss-vision-handover-v0.1
```

---

# P3：甲板视觉状态估计与短时预测

详细实施计划和验收记录：

```text
docs/plans/P3_VISUAL_STATE_ESTIMATION_PLAN.md
docs/validation/P3_VISUAL_STATE_ESTIMATION_VALIDATION.md
```

## 目标

从视觉 NED 位姿估计甲板位置和速度，并在图像延迟和短时丢帧时预测控制时刻位置。

## 计划文件

```text
src/aruco_precision_landing_cpp/
├── include/aruco_precision_landing_cpp/
│   ├── target_state_estimator.hpp
│   └── motion_predictor.hpp
├── src/
│   ├── target_state_estimator.cpp
│   └── motion_predictor.cpp
└── test/
    ├── target_state_estimator_test.cpp
    └── motion_predictor_test.cpp
```

## 状态

```text
[x, y, z, vx, vy, vz]
```

## 第一版模型

常速度 Kalman Filter：

```text
x(k+1) = F(dt) x(k) + w
z(k) = H x(k) + v
```

## 输入处理

- 使用 ArUco 消息采样时间戳计算 `dt`。
- 异常 `dt` 拒绝或重置。
- NaN、Inf 拒绝。
- 大残差跳变拒绝。
- 短时无观测只预测。
- 长时无观测设置估计无效或重新初始化。

## 预测

```text
p_pred = p_est + v_est * prediction_horizon
```

预测时域包括：

- 视觉消息年龄。
- 控制链路补偿时域。
- 可配置附加预测时域。

## 输出

```text
/landing/estimated_deck_odometry
/landing/predicted_deck_pose
```

## 单元测试

- 静态输入速度收敛到零。
- 匀速输入速度收敛到真值。
- 不规则 dt。
- 重复时间戳。
- 时间倒退。
- 短时丢帧。
- 长时丢帧。
- 离群点拒绝。
- 重捕获。

## 仿真验收

- 静态场景速度估计接近零。
- 匀速场景统计位置和速度 RMSE。
- 正弦场景估计连续。
- 短时丢帧时目标不突跳。
- Ground Truth 只用于离线统计。

## 实现与验收状态

P3 已完成：

- 新增 `target_state_estimator` 和 `motion_predictor`。
- 发布估计里程计和预测位姿。
- 全工作区累计 77 项测试通过。
- 静止消息级输入估计速度为零。
- NED East `0.4 m/s` 匀速输入估计为约 `0.40023 m/s`。
- 预测输出尚未进入控制，P2D 安全高度路径未改变。

真实 PX4 动力学下的位置/速度 RMSE 和正弦场景统计仍需实际仿真 rosbag 离线评估。

## 建议标签

```text
baseline-visual-estimator-v0.1
```

---

# P4：移动甲板水平跟踪

## 目标

在安全高度稳定跟踪移动甲板，不执行下降。

## 已实现控制结构

PX4 使用位置目标和水平速度前馈：

```text
position_sp_xy = 受限视觉 / 估计 / 预测甲板位置
velocity_ff_xy = velocity_feedforward_gain * v_deck_xy
               + relative_velocity_gain * (v_deck_xy - v_uav_xy)
```

位置误差反馈由 PX4 内部位置控制器完成，外部不重复叠加位置 P。

支持模式：

```text
RAW_VISUAL_POSITION
ESTIMATED_POSITION
ESTIMATED_POSITION_VELOCITY_FF
PREDICTED_POSITION_VELOCITY_FF
```

默认：

```text
PREDICTED_POSITION_VELOCITY_FF
```

## 已实现约束

- 水平位置目标最大速度。
- 单周期最大位置目标变化。
- 水平速度前馈最大幅值。
- 水平速度前馈最大加速度。
- 视觉短时丢失时受限预测和前馈衰减。
- 超过预测年龄后保持最近安全位置并清除前馈。
- 视觉长时丢失回退 GNSS。
- z 始终保持 `-rendezvous_altitude_m`。

## 参数

```yaml
tracking.mode: PREDICTED_POSITION_VELOCITY_FF
tracking.max_position_target_speed_mps: 2.0
tracking.max_position_target_step_m: 0.20
tracking.velocity_feedforward_gain: 1.0
tracking.relative_velocity_gain: 0.25
tracking.max_velocity_feedforward_mps: 1.5
tracking.max_velocity_feedforward_acceleration_mps2: 1.0
tracking.max_prediction_age_s: 0.75
```

## 实现与消息级验收状态

已完成：

- 新增 `moving_target_tracking_controller` 纯 C++ 模块。
- 新增 13 项跟踪控制测试。
- 全工作区累计 93 项测试通过。
- 静止目标速度前馈为零。
- NED East `0.4 m/s` 输入估计速度约 `0.4 m/s`。
- 无人机速度为零时 East 前馈约 `0.5 m/s`，与公式和 PX4 TrajectorySetpoint 一致。
- 预测位置和位置目标同方向连续增加。
- 视觉长时丢失按 `TRACK_TARGET → RECOVER_TO_GNSS → ACQUIRE_ARUCO` 恢复。
- `RAW_VISUAL_POSITION` 可通过 YAML 启动，非法模式会拒绝启动。

详细计划和验收：

```text
docs/plans/P4_MOVING_TARGET_TRACKING_PLAN.md
docs/validation/P4_MOVING_TARGET_TRACKING_VALIDATION.md
```

## P4.5 SITL 回归结果

P4.5 已使用新的图像—PX4 位姿时间对齐实现完成静止、`0.2 m/s`、`0.4 m/s` 和
XY 正弦四个场景回归。四轮均无丢标、无 GNSS 恢复、无时间同步或位姿历史告警；
详细指标见 `docs/validation/P4_5_TIME_ALIGNMENT_VALIDATION.md`。

Ground Truth 仍只进入 rosbag 离线评测。P5A、P5B 和 P5C 的计划与验收分别见对应文档。下一任务为 P6：多源触地确认、最终下降与安全中止。

## 建议标签

```text
baseline-moving-tracking-v0.1
```

---

# P5：规则式着陆窗口与分阶段下降

## 目标

只有当无人机与移动甲板的相对状态满足安全条件时才开始下降，并在下降过程中持续判断。

## 仿真扩展前置任务

在 `moving_deck_sim` 增加：

```text
S3_HEAVE
S4_ROLL_PITCH
S5_COMBINED
```

对应：

- 升沉。
- 横滚。
- 纵摇。
- 水平运动与姿态组合。

## 着陆窗口条件

至少包括：

- Marker 可见。
- 视觉数据新鲜。
- 估计器有效。
- 水平位置误差小于阈值。
- 水平相对速度小于阈值。
- 甲板横滚小于阈值。
- 甲板纵摇小于阈值。
- 预测位置有效。
- 条件持续时间达到阈值。

## 迟滞

进入下降和退出下降使用不同阈值：

```text
enter_threshold < exit_threshold
```

避免条件在边界处反复切换。

## 相对高度

```text
relative_height = deck_z_ned - uav_z_ned
```

目标：

```text
position_sp_z = predicted_deck_z_ned - height_ref
```

## 分阶段下降

建议阶段：

```text
高位跟踪
→ 快速下降
→ 中速下降
→ 最终慢速下降
```

下降速度随以下因素限制：

- 当前相对高度。
- 水平误差。
- 相对速度。
- 甲板倾角。
- 视觉消息年龄。

## 视觉丢失

### 短时丢失

- 停止正常下降。
- 使用预测短时维持水平跟踪。

### 长时丢失且高度较高

- 进入 `RECOVER_CLIMB`。

### 长时丢失且接近触地

- 采用更保守策略，不允许直接使用 GNSS 横向接管。
- 优先停止下降或受控上升。

## 单元测试

- 着陆窗口进入。
- 着陆窗口退出。
- 迟滞。
- 条件持续时间。
- 观测超时。
- 倾角超限。
- 相对速度超限。
- 下降暂停和恢复。

## 仿真验收

- 条件不满足时不下降。
- 匀速甲板可完成跟踪下降。
- 升沉场景相对高度不发散。
- 倾角过大时暂停下降。
- 下降全过程不提前切断水平跟踪。

## 建议标签

```text
baseline-landing-window-v0.1
```

---

# P6：触地确认、恢复和安全中止

## 目标

完成移动甲板降落的最后闭环，并确保异常情况下不继续危险下降。

## 新增 PX4 输入

```text
/fmu/out/vehicle_land_detected
```

## 触地确认

建议组合：

- PX4 `landed` 状态。
- 相对高度低于阈值。
- 垂直速度低于阈值。
- 状态持续时间达到阈值。

不能仅凭单帧 `landed` 进入 DONE。

## 恢复策略

### 高度较高

```text
停止下降
→ 垂直上升到恢复高度
→ GNSS 会合
→ 重新捕获 ArUco
```

### 恢复次数限制

```yaml
max_recovery_attempts: 3
```

超过后进入 `ABORT`。

## ABORT 策略

第一版只实现一个明确安全策略，不同时实现多个复杂分支：

```text
上升 / 保持到 abort_hover_altitude
```

是否切换 Return 或 Land 由显式参数控制。

## 状态转换日志

每次转换记录：

- 原状态。
- 新状态。
- 转换原因。
- 时间戳。
- 当前水平误差和阈值。
- 当前相对速度和阈值。
- 当前高度。
- GNSS / 视觉数据年龄。

## 单元测试

- 正常触地路径。
- 假触地单帧不完成。
- 恢复上升。
- 恢复次数超限。
- GNSS 和视觉同时失效。
- PX4 状态无效。
- ABORT 后不继续下降。
- 不反复 Arm / Land / Disarm。

## 仿真验收

- 成功降落能够确认。
- Marker 丢失时不继续正常下降。
- 恢复后可以重新捕获。
- ABORT 进入安全高度。
- 最终状态可由日志解释。

## 建议标签

```text
baseline-safe-touchdown-v0.1
```

---

# P7：批量实验和指标统计

## 目标

形成可以直接用于毕业论文实验的数据管线。

## 新增包

```text
src/landing_evaluation/
├── landing_evaluation/
│   ├── episode_manager.py
│   ├── metrics_collector.py
│   ├── failure_classifier.py
│   └── report_generator.py
├── config/
├── launch/
├── test/
├── setup.py
└── package.xml
```

## 根目录脚本

```text
scripts/run_single_experiment.sh
scripts/run_batch_experiments.py
scripts/aggregate_results.py
results/.gitkeep
```

## 单次实验流程

```text
加载配置
→ reset Gazebo / 甲板 / PX4
→ 等待系统就绪
→ 启动任务
→ 记录数据
→ 成功、失败或超时
→ 保存结果
→ 清理并进入下一次
```

## 每次实验输出

建议 JSON：

```text
episode_id
scenario
seed
controller_config
start_time
end_time
success
failure_reason
touchdown_error_xy
touchdown_vertical_speed
max_roll_pitch
tracking_rmse_xy
max_tracking_error_xy
vision_loss_duration
gnss_loss_duration
recovery_count
landing_time
```

建议 CSV 用于汇总，ROS bag 用于问题复现。

## 失败分类

至少包括：

```text
GNSS_TIMEOUT
RENDEZVOUS_TIMEOUT
ARUCO_NOT_ACQUIRED
VISION_LOST
TRACKING_DIVERGED
LANDING_WINDOW_TIMEOUT
HARD_LANDING
DECK_MISS
PX4_ABORT
RECOVERY_LIMIT
SIMULATION_ERROR
UNKNOWN
```

## 验收

- 单条命令连续运行不少于 20 次。
- 单次失败不阻塞后续实验。
- 每个实验有唯一 ID 和配置快照。
- 每个失败都有统一分类。
- 可以生成成功率、均值、标准差、中位数和 P95。

## 建议标签

```text
baseline-batch-evaluation-v0.1
```

---

# P8/P9：高级降落与统一论文实验

## P8A：升沉甲板最终下降与真实接触

执行顺序：

```text
项目内垂直语义分析
→ docs/plans/P8A_HEAVE_TOUCHDOWN_PLAN.md
→ 先写测试
→ 最小实现
→ static/constant02 回归
→ H1/H2/H3 分级真实 SITL
→ docs/validation/P8A_HEAVE_TOUCHDOWN_VALIDATION.md
```

H1/H2/H3 分别为 `0.10 m / 10 s`、`0.20 m / 8 s`、`0.30 m / 8 s`。第一次只开放 H1，H2/H3 逐级开放；rollpitch 和 combined 继续阻断最终下降。P8A 必须验证相对垂直速度语义和 `TOUCHDOWN_HOLD` 是否需要随甲板 z 相对保持。

## P8B：水平相对运动线性 MPC

P8B 综述、统一模型、候选方案、固定求解器、执行计划、生产实现和真实验收均已完成。第一版 MPC 只负责自由飞行与安全下降阶段的水平相对运动，P4.7 保持默认并作为 solver 失败回退；从 `FINAL_DESCENT` 起使用 `TERMINAL_PHASE_P47` 安全 handoff，没有接管最终垂直下降、touchdown detector、landing window 或姿态对齐。

严格顺序结果为安全高度 15/15、下降 6/6、真实触地 6/6 PASS，所有有效 MPC 轮次均为 0 deadline miss、0 solver failure、0 unexpected fallback；详细记录见 `docs/validation/P8B_RELATIVE_MPC_VALIDATION.md`。

## P8C：固定倾斜及低频 roll/pitch 甲板降落

P8C 已完成综述、独立计划、P8C-0～P8C-2，以及由 P8C-3 失败证据触发的 P8C-4 终端接触稳定化。当前代码包含纯数学甲板平面/X500 四滑橇几何、独立视觉法向、T1 主轴约束、在线滑橇近接触证据、状态化接触锚点、锚点中心顺应、相对高度与向下加速度预压、HOLD 法向锁存和姿态安全保护。最终 fixed T1 active touchdown 为 roll `3/3`、pitch `3/3`，旧路径回归 `9/9`，全工作区 `340 tests, 0 failures, 0 skipped`。状态为：

```text
P8C RESEARCH PASS
P8C PLAN PASS
P8C-0 IMPLEMENTATION PASS
P8C-1 VALIDATION PASS
P8C-2 SAFE DESCENT PASS
P8C-4 VALIDATION PASS
P8C T1 VALIDATION PASS
P8C-3 DESIGN GATE CLOSED
```

P8C-3 水平机体失败 Bag 和设计门文档继续保留，作为 P8C-4 设计依据。负倾角与动态 roll/pitch/combined final descent 仍关闭，后续必须建立新的独立阶段和验收计划，不能直接沿用 T1 结论。

## P9：统一批量评测、消融和论文实验

P9 复用并扩展 P7 自动化，当前状态为 `PLAN PASS / IMPLEMENTATION PASS / TEST PASS / SMOKE COMPLETE / BASELINE COMPLETE / FORMAL ABLATION COMPLETE / AGGREGATION PASS`。P9 smoke 共执行 27 轮，20 成功、7 个 `SAFETY_GATE_FAILURE`；B2 constant02、B4 heave_h1 和 B5 pitch `+2°` 被安全门关闭，不调参、不换 seed、不进入正式实验。

正式 baseline 基于 `71af1cc` 完成 B0 static/constant02 `20+20`，结果为 `40/40`。正式消融只执行 B0/B1/B3 constant02、B0/B3 sinusoidal 和 B5 roll `+2°`，每组 10 轮，结果为 `60/60`；B2 constant02、B4 heave_h1 和 B5 pitch `+2°` 的 `30` 个计划槽位标记为 `NOT_APPLICABLE`。P9 第一版总计执行 `127` 个 episode，成功 `120`，7 个失败均来自 smoke。

推荐方法：

```text
B0：完整 P4.7 规则式基线
B1：关闭额外预测
B2：关闭速度前馈
B3：完整水平相对 MPC
B4：MPC + 升沉处理
B5：固定正 T1 场景下的当前终端接触稳定化方案（Offboard position 模式内法向整形、接触顺应与受限预压；不是 PX4 attitude setpoint 姿态对齐）
```

统一输出 `overall`、`by_scenario`、`by_method`、`by_method_scenario` 和 failure breakdown，并保留失败轮完整诊断与成功轮轻量 Bag。正式目录为 `results/p9_baseline_20x20_20260804_71af1cc/` 与 `results/p9_ablation_20260804_71af1cc/`；`results/p9_baseline_20x20_20260803/` 是旧提交上的 interrupted pre-freeze batch，仅完成 `4/40`，只作历史证据并排除在最终 baseline 统计之外。

---

## 9. 推荐文件结构演进

```text
ws_aruco_landing/
├── docs/
│   ├── README.md
│   ├── guides/
│   ├── reference/
│   ├── plans/
│   ├── research/
│   └── validation/
├── src/
│   ├── aruco_detector/
│   ├── aruco_precision_landing_cpp/
│   │   ├── include/aruco_precision_landing_cpp/
│   │   │   ├── coordinate_transform.hpp
│   │   │   ├── geodetic_converter.hpp
│   │   │   ├── target_state_estimator.hpp
│   │   │   ├── motion_predictor.hpp
│   │   │   ├── landing_guidance.hpp
│   │   │   ├── landing_window.hpp
│   │   │   └── px4_aruco_landing_node.hpp
│   │   ├── src/
│   │   ├── test/
│   │   ├── config/
│   │   └── launch/
│   ├── moving_deck_sim/
│   │   ├── include/moving_deck_sim/gnss_sensor_model.hpp
│   │   ├── src/deck_gnss_simulator.cpp
│   │   ├── config/
│   │   └── test/
│   └── landing_evaluation/
├── scripts/
└── results/
```

第一版仍然不新增自定义消息包。标准消息无法稳定表达接口时，再评估 `landing_interfaces`。

---

## 10. 参数组织建议

后续不再将所有参数堆在同一个无层次 YAML 中。建议按功能分组命名，但仍使用 ROS 2 扁平参数：

```yaml
gnss.fix_timeout_s: 1.0
gnss.max_jump_m: 5.0
rendezvous.altitude_m: 5.0
rendezvous.radius_m: 2.0
search.offset_m: 1.0
search.point_hold_s: 1.0
handover.duration_s: 0.5
handover.max_difference_m: 3.0
vision.pose_timeout_s: 0.3
estimator.max_dt_s: 0.5
tracking.max_speed_mps: 2.0
landing.enter_xy_error_m: ...
landing.exit_xy_error_m: ...
recovery.altitude_m: 4.0
recovery.max_attempts: 3
```

所有参数必须同步更新：

- 参数声明。
- 参数校验。
- YAML。
- README / 文档。
- 测试。

---

## 11. 安全约束

- 默认配置仅用于 SITL。
- 自动 Arm、Land 和 Disarm 必须显式启用。
- 未收到有效 PX4 状态时禁止发布有效运动目标。
- GNSS 或视觉过期时禁止继续使用旧数据正常下降。
- NaN、Inf、非法四元数不得进入 PX4 setpoint。
- 位置、速度、加速度和目标变化率必须限幅。
- GNSS 跳变必须拒绝。
- 视觉目标跳变必须拒绝。
- 测试不得误触发实机自动解锁。
- Ground Truth 禁止进入控制器。
- 最终下降阶段禁止直接用普通 GNSS 横向接管。

---

## 12. 构建和测试要求

每个阶段执行：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash

colcon build --symlink-install \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

colcon test
colcon test-result --verbose
```

阶段验收还应检查：

```bash
git status --short
git diff --check
```

控制器 Ground Truth 隔离检查：

```bash
grep -R "/simulation/deck/ground_truth" \
  src/aruco_precision_landing_cpp src/aruco_detector
```

期望无控制器订阅代码。

---

## 13. 推荐分支和提交顺序

### P2.0

```text
分支：docs/next-development-plan
提交：docs: update GNSS-to-vision landing roadmap
```

### P2A

```text
分支：feat/p2-coordinate-transform
提交 1：docs: define coordinate frames and transform conventions
提交 2：feat: add rigid body coordinate transform module
提交 3：feat: add geodetic ENU NED conversion
提交 4：test: cover coordinate and geodetic transforms
```

### P2B

```text
分支：feat/p2-deck-gnss-sim
提交 1：feat: add deterministic deck GNSS sensor model
提交 2：feat: publish deck GNSS fix and velocity
提交 3：test: validate noise latency and reset behavior
提交 4：docs: add deck GNSS simulator validation
```

### P2C

```text
分支：feat/p2-gnss-rendezvous
提交 1：feat: add deck GNSS subscriptions and validation
提交 2：feat: add GNSS rendezvous states
提交 3：feat: add moving GNSS-centered ArUco search
提交 4：test: cover rendezvous state transitions
```

### P2D

```text
分支：feat/p2-visual-handover
提交 1：feat: transform ArUco pose to local NED
提交 2：feat: add GNSS-to-vision handover
提交 3：feat: add GNSS recovery before descent
提交 4：test: cover handover and recovery paths
```

其余阶段继续使用单阶段独立分支，验收后再合并到 main。

---

## 14. 推荐实际执行顺序

用户确认计划后，严格按以下顺序开始：

```text
第一步：P2.0
同步 AGENTS.md 和总体计划文档

第二步：P2A-1
只确认并记录 PX4、Gazebo、相机坐标语义

第三步：P2A-2
实现纯 C++ coordinate_transform

第四步：P2A-3
实现纯 C++ geodetic_converter

第五步：P2A-4
完成全部数学单元测试

第六步：P2B-1
实现理想 deck GNSS 输出

第七步：P2B-2
增加噪声、延迟、丢包和固定种子

第八步：P2B-3
完成 GNSS 仿真验收

第九步：P2C
实现 GNSS 会合，不下降

第十步：P2D
实现 ArUco 捕获和视觉接管，不下降

第十一步：P3
实现视觉状态估计和预测

第十二步：P4
实现安全高度移动目标跟踪

第十三步：P5
实现着陆窗口和下降

第十四步：P6
实现触地、恢复和中止

第十五步：P7-lite
冻结 static/constant02 3+3 自动冒烟开发基线

第十六步：P8A
升沉甲板最终下降与真实接触

第十七步：P8B
先完成 MPC 综述、建模和计划，再实现与验收

第十八步：P8C
先完成倾斜几何综述和计划，再实现与验收

第十九步：P9
统一批量评测、消融和论文实验
```

---

## 15. 当前下一项任务

`P0`～`P8C T1` 已完成当前传统基线的真实验收，P7-lite 真实 3+3 冒烟已冻结。P8C-3 的失败诊断触发了独立 P8C-4 研究、TDD、实现和分级验证，最终状态为：

```text
P8C-4 VALIDATION PASS
P8C T1 VALIDATION PASS
P8C-3 DESIGN GATE CLOSED
```

最终 roll/pitch active touchdown `6/6 PASS`，static/constant02/H1/H2/RELATIVE_MPC 回归 `9/9 PASS`，全工作区 `340 tests, 0 failures, 0 skipped`。原 P8C-3 seed2 滑移、灾难性姿态发散、离板和恢复证据仍完整保留，没有删除或被成功轮覆盖。

P9 统一批量评测第一版已完成实验和聚合。下一项工作应优先复核论文统计、补充置信区间并定稿表格与图表；负倾角、动态 `rollpitch/combined` 和更复杂船舶姿态运动需另建独立阶段，不得把 fixed T1 结论直接外推。

---

## 16. 完成定义

最终传统基线只有同时满足以下条件才算完成：

1. 普通 clone 后可以构建完整工作区。
2. 静态 ArUco 降落保持可用。
3. 控制器不使用仿真 Ground Truth。
4. 船舶 GNSS 可以引导无人机到移动甲板上方。
5. GPS 误差下仍能通过移动搜索捕获 ArUco。
6. GNSS 到视觉接管平滑且可恢复。
7. 相机、机体、NED、ENU、WGS84 变换正确并有测试。
8. 能够估计甲板速度并短时预测。
9. 能够稳定跟踪水平移动甲板。
10. 能够判断规则式着陆窗口。
11. 能够使用相对高度分阶段下降。
12. 最终下降持续水平跟踪直到触地。
13. 视觉丢失能够停止下降、恢复或中止。
14. 触地能够可靠确认。
15. 能够批量运行实验并输出统一指标。
16. 能够完成 B0 到 B4 的传统方法消融对比。
17. 所有关键坐标、参数、状态转换和实验配置均有文档。
18. 所有核心数学和状态逻辑具备单元测试。
