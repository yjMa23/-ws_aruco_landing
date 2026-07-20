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

本文档是下一阶段的执行计划。用户已于 2026-07-20 确认按本计划执行；当前已完成 `P2.0`、`P2A` 和 `P2B`，下一阶段为 `P2C` GNSS 会合与移动甲板上方粗跟踪。新坐标模块和船舶 GNSS 尚未接入现有控制器。

---

## 2. 当前项目状态

### 2.1 已完成阶段

| 阶段 | 状态 | 主要成果 |
| --- | --- | --- |
| `P0` 仓库整理与静态基线冻结 | 已完成 | `aruco_detector` 纳入主仓库，工作区可统一构建，标签 `baseline-static-v0.1` |
| `P1` 水平移动甲板仿真 | 已完成 | 静止、匀速、XY 正弦甲板，确定性 reset，Ground Truth，标签 `baseline-moving-deck-v0.1` |
| `P2.0` 项目状态和设计文档同步 | 已完成 | 更新项目阶段、GNSS 到视觉流程、Ground Truth 边界和默认任务 |
| `P2A-1` 坐标契约确认 | 已完成 | 新增 `docs/COORDINATE_FRAMES.md`，核对 PX4、Gazebo、相机和地理原点语义 |
| `P2A-2～4` 纯数学模块与测试 | 已完成 | 实现 `coordinate_transform`、`geodetic_converter` 和 GTest；尚未接入控制器 |
| `P2B` 船舶 GNSS 传感器仿真 | 已完成 | 实现理想/含噪 GNSS、ENU 速度、固定采样/延迟/丢包、确定性 reset 和端到端冒烟验证 |

### 2.2 当前已有 ROS 2 包

```text
src/aruco_detector
src/aruco_precision_landing_cpp
src/moving_deck_sim
```

### 2.3 当前控制器能力

`aruco_precision_landing_cpp` 当前属于静态 Marker 降落 V0，已经具备：

- PX4 Offboard 预发布。
- 自动切换 Offboard。
- 自动解锁和起飞。
- 飞往固定搜索区域。
- 等待 ArUco。
- 基于相机平面误差进行水平对中。
- 固定速率下降。
- 在固定世界高度附近切换 `NAV_LAND`。
- Marker 长时间丢失后进入 `ABORT`。

### 2.4 当前核心缺口

- 固定搜索点无法跟随移动甲板。
- 甲板离开下视相机视野后无法重新捕获。
- 没有船舶 GNSS / 遥测粗引导。
- 没有完整相机外参和刚体坐标变换。
- 没有 GNSS、ENU、NED 之间的统一转换。
- 没有甲板位置和速度估计。
- 没有运动预测和速度前馈。
- 没有 GNSS 到 ArUco 的安全接管逻辑。
- 没有基于相对速度和甲板姿态的着陆窗口。
- 没有相对甲板高度控制。
- 最终下降阶段过早退出 Offboard 水平跟踪。
- 没有触地确认、恢复、失败分类和批量评测。

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

后续增加：

```text
/fmu/out/vehicle_global_position
/fmu/out/vehicle_land_detected
```

具体 PX4 消息字段必须以当前安装的 `px4_msgs` 版本为准，不在代码中假设字段名称。

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
docs/TRADITIONAL_BASELINE_PLAN.md
docs/NEXT_DEVELOPMENT_PLAN.md
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

docs/COORDINATE_FRAMES.md
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
/fmu/out/vehicle_global_position
```

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

## 建议标签

```text
baseline-gnss-vision-handover-v0.1
```

---

# P3：甲板视觉状态估计与短时预测

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

## 建议标签

```text
baseline-visual-estimator-v0.1
```

---

# P4：移动甲板水平跟踪

## 目标

在安全高度稳定跟踪移动甲板，不执行下降。

## 控制结构

```text
预测甲板位置目标
+
甲板速度前馈
+
水平位置误差反馈
+
水平相对速度反馈
```

建议形式：

```text
v_cmd_xy = v_deck_pred
         + Kp * position_error_xy
         + Kv * relative_velocity_error_xy
```

根据 PX4 接口选择：

- 位置 + 速度前馈；或
- 纯速度设定点。

第一版优先使用 PX4 已支持且容易验证的“位置目标 + 速度前馈”。

## 约束

- 最大水平速度。
- 最大加速度。
- 最大设定点变化率。
- 视觉短时丢失时逐步减弱控制。
- 视觉长时丢失回退 GNSS。

## 参数

```yaml
tracking_altitude_m: 4.0
position_gain_xy: ...
relative_velocity_gain_xy: ...
max_tracking_speed_mps: ...
max_tracking_acceleration_mps2: ...
prediction_horizon_s: ...
```

数值在仿真调试后确定，不在编码阶段盲目固定。

## 对比基线

- 无速度前馈。
- 有速度前馈。
- 有速度前馈 + 预测。

## 验收

- 静止场景稳定悬停。
- 匀速 `0.4 m/s` 不持续发散。
- 正弦场景目标连续。
- 加前馈后水平 RMSE 明显优于无前馈。
- Marker 短时丢失不会立即失控。
- 全过程保持安全高度。

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

# P8：传统方法消融实验

## 目标

形成论文传统基线对比结果。

## 推荐基线

```text
B0：当前静态位置增量控制
B1：完整坐标变换 + GNSS 会合
B2：B1 + 视觉位置滤波和速度估计
B3：B2 + 速度前馈和短时预测
B4：B3 + 着陆窗口和分阶段下降
```

可选：

```text
B5：线性 MPC 跟踪与约束下降
```

未完成 B0 到 B4 的稳定实验前，不开始 MPC 或强化学习对比。

## 实验场景

```text
S0_STATIC
S1_CONSTANT_XY_SLOW
S1_CONSTANT_XY_MEDIUM
S2_SINUSOIDAL_XY
S3_HEAVE
S4_ROLL_PITCH
S5_COMBINED
S6_SENSOR_DEGRADED
```

`S6_SENSOR_DEGRADED` 包括：

- GNSS 噪声增加。
- GNSS 延迟。
- GNSS 丢包。
- 视觉短时遮挡。
- 视觉测量噪声。

## 每个配置运行次数

开发验收：

```text
3 次冒烟
20 次固定场景回归
```

论文实验：

```text
每配置至少 50 次
```

## 指标

- 成功率。
- 触地点水平误差。
- 触地垂直速度。
- 跟踪 RMSE。
- 最大水平误差。
- 平均降落时间。
- ArUco 捕获时间。
- GNSS 到视觉接管时间。
- 恢复次数。
- 失败类型占比。
- 均值、标准差、中位数、P90/P95。

---

## 9. 推荐文件结构演进

```text
ws_aruco_landing/
├── docs/
│   ├── NEXT_DEVELOPMENT_PLAN.md
│   ├── COORDINATE_FRAMES.md
│   ├── SYSTEM_ARCHITECTURE.md
│   ├── EXPERIMENT_PROTOCOL.md
│   ├── FAILURE_CATALOG.md
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

第十五步：P7/P8
批量评测和消融实验
```

---

## 15. 用户确认后的第一项代码任务

确认本文档后，第一项任务限定为：

```text
P2.0 + P2A 的坐标契约确认部分
```

具体只做：

1. 更新 `AGENTS.md` 中当前阶段。
2. 新增 `docs/COORDINATE_FRAMES.md`。
3. 检查当前 PX4 模型的下视相机安装位姿。
4. 检查当前 `px4_msgs` 中 VehicleOdometry 四元数和 frame 定义。
5. 确认 Gazebo spherical origin 与 PX4 home 的关系。
6. 输出明确的变换方向和参数设计。
7. 暂不接入控制器。
8. 暂不实现 GNSS 节点。
9. 暂不修改状态机。
10. 等坐标契约再次检查通过后，再实现数学模块。

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
