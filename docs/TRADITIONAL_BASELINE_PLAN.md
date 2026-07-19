# 移动船舶无人机自主降落传统基线实施计划

## 1. 文档用途

本文件是 `ws_aruco_landing` 的项目级开发约束和传统基线实现路线，供 Codex 在后续编码时自动参考。

项目目标不是只完成静态 ArUco 精准降落，而是实现一套可重复评测的传统方法基线：

> 无人机在 Gazebo + PX4 SITL + ROS 2 环境中，仅依赖机载状态和视觉 ArUco 观测，对移动、升沉和倾斜的船舶甲板进行目标跟踪、着陆窗口判断、安全下降和最终触地。

强化学习不是本阶段任务。传统基线完成后，强化学习仅用于替换或增强高层着陆窗口决策、下降速度参考或参数自适应，不能替代 PX4 姿态内环和基础安全逻辑。

---

## 2. 当前代码状态

当前工作区已经具备以下基础能力：

1. `aruco_detector`
   - 订阅相机图像和 `CameraInfo`。
   - 检测指定 ID 的 ArUco Marker。
   - 使用 PnP 估计 Marker 在相机光学坐标系下的位姿。
   - 发布：
     - `/aruco/pose`
     - `/aruco/visible`
     - `/aruco/debug_image`

2. `aruco_precision_landing_cpp`
   - 订阅 PX4 状态、局部位置、里程计和 ArUco 位姿。
   - 发布 PX4 Offboard 模式、轨迹设定点和飞行命令。
   - 已实现状态：
     - `INIT`
     - `WAIT_FOR_PX4`
     - `OFFBOARD_PRE_STREAM`
     - `ARM_AND_TAKEOFF`
     - `GOTO_ARUCO_AREA`
     - `WAIT_ARUCO`
     - `CENTER_ABOVE_MARKER`
     - `DESCEND_WITH_TRACKING`
     - `FINAL_LAND`
     - `DONE`
     - `ABORT`

3. 当前实现仍属于“静态标志物降落 V0”，主要缺口如下：
   - 搜索点是固定 NED 坐标，不能跟踪移动甲板。
   - 只使用 ArUco `position.x/y`，没有利用高度和甲板姿态。
   - 相机到机体仅通过两个符号参数映射，没有完整相机外参和 TF。
   - 没有甲板位置、速度、升沉速度和姿态的状态估计。
   - 没有目标运动预测和速度前馈。
   - 下降速度固定，没有基于相对速度、甲板倾角和可见性的着陆窗口。
   - `FINAL_LAND` 过早切换 PX4 自动降落，移动甲板继续运动时可能失去水平跟踪。
   - 没有触地检测、批量实验、指标统计和失败原因归类。

---

## 3. 仓库整理要求

在新增功能前，先完成仓库结构修复。

当前 `src/aruco_detector` 不能继续以不透明的 Gitlink 形式存在。优先采用单仓库方式，将 `aruco_detector` 作为普通 ROS 2 包完整放入本仓库，确保：

```text
src/aruco_detector/CMakeLists.txt
src/aruco_detector/package.xml
src/aruco_detector/include/
src/aruco_detector/src/
src/aruco_detector/config/
src/aruco_detector/launch/
```

所有开发者和 Codex 在普通 `git clone` 后都必须能够直接构建全部源码。除非明确要求，不新增嵌套 Git 仓库。

完成整理后创建静态基线标签：

```text
baseline-static-v0.1
```

---

## 4. 最终传统基线的边界

### 4.1 必须实现

传统基线最终必须覆盖以下完整闭环：

```text
移动甲板仿真
    ↓
下视相机图像
    ↓
ArUco 检测与 PnP 位姿
    ↓
完整坐标变换
    ↓
甲板状态估计与短时预测
    ↓
移动目标水平跟踪
    ↓
规则式着陆窗口判断
    ↓
相对高度下降控制
    ↓
持续水平跟踪直至触地
    ↓
触地检测、完成或安全中止
    ↓
批量实验与指标统计
```

### 4.2 控制器禁止使用的信息

控制器只能使用：

- PX4 飞行状态和无人机估计状态。
- 相机图像、相机内参和相机外参。
- ArUco 检测结果。
- 配置参数和控制器内部估计状态。

以下信息只能用于仿真器和评测器，禁止输入控制器：

- Gazebo 中甲板真实位姿。
- Gazebo 中甲板真实速度。
- 仿真器内部运动相位。
- 预先知道的甲板未来轨迹。

任何直接使用 Ground Truth 完成跟踪或着陆的实现都不属于有效基线。

### 4.3 暂不实现

传统基线阶段暂不实现：

- 端到端强化学习控制。
- 图像直接输出电机或姿态命令。
- 多机协同。
- 复杂目标检测网络。
- 实船通信和硬件在环。
- 为未来需求预先搭建大型插件化框架。

---

## 5. 目标仓库结构

保留现有包，按最少新增模块的原则扩展：

```text
ws_aruco_landing/
├── AGENTS.md
├── README.md
├── docs/
│   ├── SYSTEM_ARCHITECTURE.md
│   ├── COORDINATE_FRAMES.md
│   ├── EXPERIMENT_PROTOCOL.md
│   └── FAILURE_CATALOG.md
├── src/
│   ├── aruco_detector/
│   │   ├── include/aruco_detector/
│   │   ├── src/
│   │   ├── config/
│   │   ├── launch/
│   │   ├── test/
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── aruco_precision_landing_cpp/
│   │   ├── include/aruco_precision_landing_cpp/
│   │   │   ├── coordinate_transform.hpp
│   │   │   ├── target_state_estimator.hpp
│   │   │   ├── motion_predictor.hpp
│   │   │   ├── landing_guidance.hpp
│   │   │   ├── landing_state_machine.hpp
│   │   │   └── px4_aruco_landing_node.hpp
│   │   ├── src/
│   │   │   ├── coordinate_transform.cpp
│   │   │   ├── target_state_estimator.cpp
│   │   │   ├── motion_predictor.cpp
│   │   │   ├── landing_guidance.cpp
│   │   │   ├── landing_state_machine.cpp
│   │   │   └── px4_aruco_landing_node.cpp
│   │   ├── config/
│   │   │   ├── static_platform.yaml
│   │   │   ├── moving_platform.yaml
│   │   │   └── robustness_test.yaml
│   │   ├── launch/
│   │   ├── test/
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── moving_deck_sim/
│   │   ├── config/
│   │   │   ├── static.yaml
│   │   │   ├── constant_velocity.yaml
│   │   │   ├── sinusoidal_xy.yaml
│   │   │   ├── heave_roll_pitch.yaml
│   │   │   └── combined_motion.yaml
│   │   ├── launch/
│   │   ├── models/
│   │   ├── worlds/
│   │   ├── src/
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── landing_bringup/
│   │   ├── config/
│   │   ├── launch/
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── landing_evaluation/
│       ├── landing_evaluation/
│       │   ├── episode_manager.py
│       │   ├── metrics_collector.py
│       │   ├── failure_classifier.py
│       │   └── report_generator.py
│       ├── config/
│       ├── launch/
│       ├── scripts/
│       ├── test/
│       ├── setup.py
│       └── package.xml
├── scripts/
│   ├── run_single_experiment.sh
│   ├── run_batch_experiments.py
│   └── aggregate_results.py
└── results/
    └── .gitkeep
```

不要一开始创建 `landing_interfaces` 自定义消息包。第一版优先使用标准消息，接口稳定且确实无法表达时再新增。

---

## 6. 节点和数据流

### 6.1 运行时节点

| 节点 | 包 | 职责 |
| --- | --- | --- |
| `aruco_detector_node` | `aruco_detector` | ArUco 检测、PnP 位姿和调试图像 |
| `moving_deck_controller` | `moving_deck_sim` | 按配置驱动甲板运动，仅仿真使用 |
| `px4_aruco_landing_node` | `aruco_precision_landing_cpp` | 坐标变换、状态估计、预测、状态机和 Offboard 控制 |
| `episode_manager` | `landing_evaluation` | 重置场景、开始和结束单次实验 |
| `metrics_collector` | `landing_evaluation` | 记录 Ground Truth、控制状态和触地指标 |
| `failure_classifier` | `landing_evaluation` | 对失败原因进行统一分类 |

第一版不必将估计器、预测器和控制器拆成多个 ROS 节点。它们应优先作为 `aruco_precision_landing_cpp` 内部的独立 C++ 类，由一个控制节点调用，避免不必要的通信和同步复杂度。

### 6.2 主要话题

#### 感知输入与输出

| 方向 | 话题 | 类型 |
| --- | --- | --- |
| 输入 | 相机图像话题 | `sensor_msgs/msg/Image` |
| 输入 | 相机内参话题 | `sensor_msgs/msg/CameraInfo` |
| 输出 | `/aruco/pose` | `geometry_msgs/msg/PoseStamped` |
| 输出 | `/aruco/visible` | `std_msgs/msg/Bool` |
| 输出 | `/aruco/debug_image` | `sensor_msgs/msg/Image` |

#### PX4 输入

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/fmu/out/vehicle_status` | `px4_msgs/msg/VehicleStatus` | 模式和解锁状态 |
| `/fmu/out/vehicle_local_position` | `px4_msgs/msg/VehicleLocalPosition` | NED 位置和速度 |
| `/fmu/out/vehicle_odometry` | `px4_msgs/msg/VehicleOdometry` | 姿态 |
| `/fmu/out/vehicle_land_detected` | `px4_msgs/msg/VehicleLandDetected` | 触地检测 |

PX4 带版本后缀的话题必须在 launch 中重映射，业务代码内部保持固定话题名。

#### 控制输出

| 话题 | 类型 |
| --- | --- |
| `/fmu/in/offboard_control_mode` | `px4_msgs/msg/OffboardControlMode` |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/msg/TrajectorySetpoint` |
| `/fmu/in/vehicle_command` | `px4_msgs/msg/VehicleCommand` |
| `/landing/state` | `std_msgs/msg/String` |
| `/landing/target_pose` | `geometry_msgs/msg/PoseStamped` |
| `/landing/estimated_deck_odometry` | `nav_msgs/msg/Odometry` |
| `/landing/predicted_deck_pose` | `geometry_msgs/msg/PoseStamped` |

#### 仿真真值

| 话题 | 类型 | 使用范围 |
| --- | --- | --- |
| `/simulation/deck/ground_truth` | `nav_msgs/msg/Odometry` | 仅评测器 |
| `/simulation/episode/reset` | `std_msgs/msg/Empty` 或服务 | 仿真器和评测器 |
| `/simulation/episode/id` | `std_msgs/msg/UInt32` | 评测记录 |

---

## 7. 坐标系约定

必须在 `docs/COORDINATE_FRAMES.md` 中固定以下坐标系：

- `camera_optical`
  - `x`：图像向右。
  - `y`：图像向下。
  - `z`：镜头前方。

- `base_link_frd`
  - `x`：机头前方。
  - `y`：机体右方。
  - `z`：机体下方。

- `local_ned`
  - `x`：North。
  - `y`：East。
  - `z`：Down。

位姿链统一写为：

```text
T_local_ned_marker
=
T_local_ned_body
*
T_body_camera
*
T_camera_marker
```

要求：

1. `T_camera_marker` 来自 ArUco PnP。
2. `T_body_camera` 来自参数或静态 TF，禁止继续只用正负号近似。
3. `T_local_ned_body` 来自 PX4 里程计。
4. 所有四元数顺序必须在接口处显式说明。
5. ENU、FLU、FRD 和 NED 的转换必须集中在 `coordinate_transform` 中。
6. 业务控制代码中禁止散落手写坐标符号转换。
7. 对每个坐标变换编写单元测试，至少验证：
   - Marker 位于图像右侧时，控制方向正确。
   - Marker 位于图像前后方向时，控制方向正确。
   - 无人机偏航 `0°、90°、180°、-90°` 时，NED 误差方向正确。
   - 下视相机存在安装旋转时，变换结果正确。

---

## 8. 状态估计和运动预测

### 8.1 估计状态

传统基线至少估计：

```text
deck_position_ned = [x, y, z]
deck_velocity_ned = [vx, vy, vz]
deck_attitude = [roll, pitch, yaw]
measurement_valid
measurement_age
```

第一版使用常速度 Kalman Filter：

```text
state = [x, y, z, vx, vy, vz]
measurement = [x, y, z]
```

要求：

- 使用消息时间戳计算 `dt`，不能只使用回调接收时间。
- 对异常 `dt`、NaN、跳变和过期数据进行拒绝。
- ArUco 丢失时允许短时纯预测。
- 重新检测后限制一次性状态跳变，避免目标设定点突变。
- 估计器参数全部放入 YAML。
- Ground Truth 只能用于离线计算位置和速度 RMSE。

### 8.2 预测模型

第一版使用常速度短时预测：

```text
p_deck_predicted = p_deck + v_deck * prediction_horizon
```

`prediction_horizon` 初始建议范围为 `0.15~0.50 s`，具体值通过实验确定。

增强版可加入：

- 升沉正弦拟合。
- 甲板姿态角速度预测。
- 延迟补偿。
- 基于历史窗口的最小二乘预测。

未完成常速度预测和基础实验前，不实现复杂神经网络预测器。

---

## 9. 传统跟踪与下降控制

### 9.1 水平跟踪

移动目标阶段不能只在无人机当前位置上累加 ArUco 误差。

建议使用“预测位置目标 + 甲板速度前馈”：

```text
position_sp_xy = predicted_deck_position_xy
velocity_sp_xy =
    estimated_deck_velocity_xy
    + kp_xy * horizontal_position_error
    - kd_xy * horizontal_relative_velocity
```

PX4 `TrajectorySetpoint` 中：

- 水平位置用于约束跟踪目标。
- 水平速度作为移动目标前馈。
- 所有速度、加速度和步长均需要限幅。
- 设定点变化率必须连续，不能在检测恢复时突然跳变。

### 9.2 相对高度控制

定义无人机相对甲板高度：

```text
relative_height = deck_z_ned - uav_z_ned
```

在 NED 坐标中，无人机位于甲板上方时 `relative_height > 0`。

下降过程中维护参考高度 `height_ref`：

```text
position_sp_z = predicted_deck_z_ned - height_ref
```

`height_ref` 按阶段逐渐减小，初始建议：

| 阶段 | 相对高度 | 最大下降速度 |
| --- | ---: | ---: |
| 跟踪等待 | `2.0~3.0 m` | `0 m/s` |
| 初始下降 | `0.8~2.0 m` | `0.3~0.5 m/s` |
| 精细下降 | `0.3~0.8 m` | `0.15~0.25 m/s` |
| 最终触地 | `<0.3 m` | `0.05~0.15 m/s` |

具体值必须参数化，不写死在状态机中。

### 9.3 着陆窗口

只有同时满足以下条件并持续一定时间，才允许进入下降：

```text
marker_visible == true
measurement_age < max_measurement_age
horizontal_error < landing_xy_threshold
horizontal_relative_speed < landing_relative_speed_threshold
abs(deck_roll) < max_landing_roll
abs(deck_pitch) < max_landing_pitch
estimated_state_valid == true
```

最终触地阶段应增加：

```text
vertical_relative_speed < max_touchdown_vertical_speed
```

窗口条件必须支持迟滞，避免状态在边界附近频繁切换。

### 9.4 最终触地

移动甲板场景下，默认不得在较高高度直接切换 `VEHICLE_CMD_NAV_LAND`。

基线策略应当：

1. 保持 Offboard 水平跟踪。
2. 按受限垂直速度继续下降。
3. 持续使用甲板预测位置和速度前馈。
4. 通过 `VehicleLandDetected`、相对高度和垂直速度联合判断触地。
5. 确认稳定触地后进入 `DONE`，再按配置决定是否 Disarm。

`NAV_LAND` 仅作为静态平台兼容模式，默认关闭。

---

## 10. 推荐状态机

```text
INIT
  ↓
WAIT_FOR_PX4
  ↓
OFFBOARD_PRE_STREAM
  ↓
ARM_AND_TAKEOFF
  ↓
RENDEZVOUS
  ↓
ACQUIRE_TARGET
  ↓
TRACK_TARGET
  ↓
WAIT_LANDING_WINDOW
  ↓
DESCEND
  ↓
FINAL_DESCENT
  ↓
TOUCHDOWN_CONFIRM
  ↓
DONE
```

异常分支：

```text
ACQUIRE_TARGET / TRACK_TARGET
  └─ 长时间丢失目标 → RECOVER

DESCEND / FINAL_DESCENT
  ├─ 短时丢失目标 → 保持预测并减小下降速度
  ├─ 超时或误差过大 → RECOVER
  └─ 严重异常 → ABORT

RECOVER
  ├─ 上升至安全相对高度
  ├─ 限制水平速度
  ├─ 重新捕获目标
  └─ 成功后回到 TRACK_TARGET

ABORT
  ├─ 停止下降
  ├─ 飞往安全高度
  ├─ 保持或返回预设安全点
  └─ 等待人工处理
```

每次状态转换必须记录：

- 原状态。
- 新状态。
- 转换原因。
- 时间戳。
- 关键误差和阈值。

禁止仅通过日志文本隐式表达转换原因，应使用枚举或统一原因字符串。

---

## 11. 移动甲板仿真场景

`moving_deck_sim` 必须支持固定随机种子和可重复重置。

### 11.1 场景等级

| 场景 | 甲板运动 | 目的 |
| --- | --- | --- |
| `S0_STATIC` | 静止 | 回归当前功能 |
| `S1_CONSTANT_XY` | 水平匀速直线 | 验证速度估计和前馈 |
| `S2_SINUSOIDAL_XY` | 水平正弦运动 | 验证加减速条件下跟踪 |
| `S3_HEAVE` | 垂向升沉 | 验证相对高度控制 |
| `S4_ROLL_PITCH` | 横摇和纵摇 | 验证着陆窗口 |
| `S5_COMBINED` | 平移、升沉、横摇和纵摇组合 | 最终传统基线 |
| `S6_ROBUSTNESS` | 组合运动、风扰、视觉噪声和丢帧 | 鲁棒性实验 |

### 11.2 开发阶段建议参数

仅作为初始测试范围，必须放入 YAML：

```text
水平速度：0.2 / 0.4 / 0.6 m/s
水平正弦幅值：0.5 / 1.0 m
水平正弦周期：6 / 10 s
升沉幅值：0.05 / 0.10 / 0.20 m
升沉周期：4 / 6 / 8 s
横摇幅值：2° / 5° / 8°
纵摇幅值：2° / 5° / 8°
视觉丢失：0.1 / 0.3 / 0.5 s
```

最终论文实验使用的参数必须由 `EXPERIMENT_PROTOCOL.md` 固定，不得在批量实验中临时修改。

---

## 12. 评测指标

每次实验至少记录：

```text
episode_id
scenario_name
random_seed
success
failure_type
landing_time
target_visible_ratio
maximum_marker_lost_duration
deck_position_estimation_rmse
deck_velocity_estimation_rmse
horizontal_tracking_rmse
maximum_horizontal_error
touchdown_horizontal_error
touchdown_horizontal_relative_speed
touchdown_vertical_relative_speed
touchdown_roll
touchdown_pitch
abort_count
recover_count
```

失败类型统一为：

```text
NONE
NO_TARGET_DETECTION
TARGET_LOST
ESTIMATOR_DIVERGENCE
TRACKING_DIVERGENCE
OFFBOARD_LOST
LANDING_WINDOW_TIMEOUT
DECK_MISS
HARD_LANDING
UNSTABLE_TOUCHDOWN
ABORTED
EPISODE_TIMEOUT
SIMULATION_ERROR
```

### 12.1 开发验收

每个新功能先运行：

- 单元测试。
- 3 次冒烟实验。
- 20 次固定场景回归。

### 12.2 最终实验

论文正式结果建议每个配置运行至少 50 次，并报告：

- 成功率。
- 均值和标准差。
- 中位数。
- P90 或 P95。
- 失败类型占比。

### 12.3 初始通过标准

以下标准用于开发验收，不直接视为最终论文结论：

| 场景 | 建议通过标准 |
| --- | --- |
| `S0_STATIC` | 成功率不低于 `95%`，触地点水平误差 P95 不大于 `0.30 m` |
| `S1_CONSTANT_XY`，`0.4 m/s` | 成功率不低于 `90%` |
| `S2_SINUSOIDAL_XY` | 成功率不低于 `85%` |
| `S3_HEAVE` | 无明显高度跟随发散，硬着陆率低于 `5%` |
| `S5_COMBINED` 中等难度 | 成功率不低于 `75%`，且所有失败均能被正确分类 |

若甲板尺寸或无人机模型变化，触地点误差阈值应按甲板安全区域重新定义。

---

## 13. 实现阶段和验收条件

### P0：仓库与静态基线冻结

任务：

- 将 `aruco_detector` 变为普通 ROS 2 包。
- 确认整个工作区一次构建成功。
- 修复 README 中失效路径。
- 为现有静态降落运行一次完整回归。
- 保存静态基线参数和结果。

验收：

```bash
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
colcon test
colcon test-result --verbose
```

静态场景能够完成起飞、识别、对中、下降和触地。

### P1：水平移动甲板仿真

任务：

- 创建可配置移动甲板。
- 实现静止、匀速和水平正弦运动。
- 发布 Ground Truth。
- 支持固定种子和场景重置。
- 提供一键 launch。

验收：

- 甲板运动与 YAML 一致。
- Ground Truth 时间戳连续。
- 重置后初始状态一致。
- 控制器未订阅 Ground Truth。

### P2：完整坐标变换

任务：

- 引入相机外参。
- 使用完整刚体变换替换符号映射。
- 统一 NED、FRD 和相机光学坐标。
- 编写变换单元测试。

验收：

- 四种典型偏航角测试全部通过。
- 仿真中无人机水平修正方向无误。
- 代码中不存在分散的相机坐标正负号映射。

### P3：甲板状态估计和预测

任务：

- 实现常速度 Kalman Filter。
- 输出甲板估计位置和速度。
- 实现短时常速度预测。
- 处理异常值、丢帧和重新捕获。

验收：

- 静态场景速度估计接近零。
- 匀速场景位置和速度 RMSE 可统计。
- 短时丢帧时估计连续。
- Ground Truth 仅进入评测器。

### P4：移动甲板水平跟踪

任务：

- 使用预测位置目标。
- 加入甲板速度前馈。
- 保持安全相对高度，不执行下降。
- 完成匀速和正弦目标跟踪。

验收：

- 相比无前馈方法，水平跟踪 RMSE 明显下降。
- 不发生持续发散。
- 设定点无明显跳变。
- 目标短时丢失时不会立即失控。

### P5：规则式着陆窗口与下降

任务：

- 加入相对位置、相对速度、倾角和观测时效条件。
- 实现分阶段下降速度。
- 保持水平跟踪至最终触地。
- 加入着陆窗口迟滞。

验收：

- 条件不满足时不会下降。
- 条件恶化时能够暂停下降或恢复。
- 升沉场景中相对高度控制稳定。
- 不在 `0.3 m` 高度直接失去移动目标跟踪。

### P6：触地、恢复和安全中止

任务：

- 接入 `VehicleLandDetected`。
- 实现 `TOUCHDOWN_CONFIRM`。
- 实现短时丢失减速、长时丢失恢复和严重异常中止。
- 完成统一状态转换原因记录。

验收：

- 成功触地与硬着陆能够区分。
- 恢复动作不会继续下降。
- ABORT 后进入安全高度。
- 不出现自动反复解锁、降落或 Disarm。

### P7：批量评测

任务：

- 自动重置、启动、超时和结束单次实验。
- 输出每次实验 CSV/JSON。
- 汇总成功率和误差统计。
- 自动生成基础曲线和表格。
- 固定实验配置和随机种子。

验收：

- 单条命令可连续运行不少于 20 次。
- 中途失败不会阻塞后续实验。
- 每个失败都有统一分类。
- 结果可以直接用于论文表格和绘图。

### P8：传统方法消融实验

至少比较：

```text
B0：当前静态位置增量控制
B1：完整坐标变换 + 滤波
B2：B1 + 目标速度估计
B3：B2 + 速度前馈和短时预测
B4：B3 + 着陆窗口和分阶段下降
```

可选增强基线：

```text
B5：线性 MPC 跟踪与约束下降
```

没有完成 B0 到 B4 的稳定对比前，不开始强化学习对比。

---

## 14. Codex 编码规则

### 14.1 修改范围

- 每次只完成一个明确阶段或子任务。
- 不顺手重构无关文件。
- 不为一次性需求创建抽象层。
- 优先复用现有节点、参数和启动方式。
- 若发现当前需求与代码结构冲突，先在回复中指出冲突，再进行最小可行修改。

### 14.2 C++ 规范

- 使用 C++17。
- 类名使用 `PascalCase`。
- 函数和变量使用 `snake_case`。
- 常量使用 `kPascalCase`。
- 所有新增公开函数使用 Doxygen 中文注释，必须说明：
  - 当前函数实现的功能。
  - 输入参数的坐标系、单位和有效范围。
  - 返回值或输出内容。
  - 可能的失败条件。
- 复杂坐标变换、滤波更新、状态转换和控制公式必须写中文注释。
- 禁止无意义逐行注释。
- 禁止使用裸 `new/delete`。
- 参数必须校验有限性、范围和相互关系。

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

### 14.3 ROS 2 规范

- 所有话题名可通过参数或 launch 重映射。
- 传感器数据使用合适的 SensorData QoS。
- PX4 QoS 与实际 `px4_msgs` 发布端保持兼容。
- 回调中不执行长时间阻塞任务。
- 定时控制循环必须处理异常 `dt`。
- 使用消息时间戳判断观测时效。
- 所有新增参数同步修改：
  - 参数声明。
  - 参数校验。
  - YAML。
  - README 或相关文档。
  - 测试。

### 14.4 测试要求

新增数学模块必须优先写成不依赖 ROS 的普通 C++ 类，以便使用 GTest 测试。

至少覆盖：

- 坐标变换。
- Kalman Filter 预测和更新。
- 异常时间间隔。
- 观测超时。
- 着陆窗口迟滞。
- 状态机正常路径。
- 丢失目标恢复路径。
- ABORT 路径。
- 参数非法输入。

任何阶段完成后都要执行：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
colcon test
colcon test-result --verbose
```

### 14.5 安全要求

- 默认配置只面向 SITL。
- 自动 Arm、Land 和 Disarm 参数必须显式配置。
- 未收到有效 PX4 状态时禁止发送有效运动目标。
- 观测过期时禁止继续正常速度下降。
- 任何 NaN、Inf 或非法四元数不得进入 PX4 setpoint。
- 控制命令必须限幅。
- 测试代码不能误触发实机自动解锁。

---

## 15. 完成定义

传统基线只有同时满足以下条件才算完成：

1. 普通克隆后能够完整构建。
2. 静态 ArUco 降落功能保持可用。
3. 控制器不使用仿真 Ground Truth。
4. 能够对水平移动甲板进行稳定跟踪。
5. 能够估计甲板速度并进行短时预测。
6. 能够根据相对误差、相对速度和甲板倾角判断着陆窗口。
7. 能够在移动目标上持续跟踪到触地，而不是提前切断跟踪。
8. 目标丢失时能够减速、恢复或安全中止。
9. 能够批量运行实验并输出统一指标。
10. 能够完成 B0 到 B4 的传统方法消融对比。
11. 所有关键参数、坐标系、状态机和实验配置均有文档。
12. 核心数学模块具备单元测试。

---

## 16. Codex 下一步默认任务

当没有额外指令时，Codex 应从 `P0` 开始，不直接实现 MPC 或强化学习。

第一项任务：

```text
检查并修复 src/aruco_detector 的仓库结构，
将其作为普通 ROS 2 包纳入当前工作区，
确保根目录执行 colcon build 能同时构建
aruco_detector 和 aruco_precision_landing_cpp，
且不改变现有 ROS 话题和运行行为。
```

完成第一项任务后，再开始 `P1：水平移动甲板仿真`。
