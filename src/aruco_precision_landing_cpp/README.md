# aruco_precision_landing_cpp

ROS 2 C++17 精准降落控制包，负责 PX4 Offboard、船舶 GNSS 会合、ArUco 视觉接管、甲板估计与预测、水平跟踪、相对下降、触地确认和接触后保持。

## 输入

- PX4 `VehicleStatus`、`VehicleLocalPosition`、`VehicleOdometry`、`VehicleLandDetected`。
- 船舶 `/deck/gps/fix` 和 `/deck/gps/velocity`。
- `/aruco/pose`、`/aruco/visible` 和相机时间戳。
- 相机外参和 YAML 参数。

控制器禁止订阅 `/simulation/deck/ground_truth`。

## 输出

- PX4 OffboardControlMode、TrajectorySetpoint 和 VehicleCommand。
- 甲板估计 odometry、预测 pose、垂直状态和着陆窗口诊断。
- 触地证据、候选持续时间和确认状态。
- MPC 求解状态、耗时、约束和回退诊断。
- 甲板平面、滑橇间隙和终端接触稳定化诊断。
- 独立 6-DoF shadow 状态、轨迹、状态字符串和可信时域：
  `/landing/deck_motion_shadow/{state,trajectory,status,trusted_horizon_s}`。

## 控制模式

默认 `PREDICTED_POSITION_VELOCITY_FF` 使用预测位置、甲板速度前馈和相对速度阻尼。

显式选择 `RELATIVE_MPC` 后，四状态水平相对 MPC 输出水平加速度前馈；求解失败或进入接触敏感终端阶段时使用完整规则式输出。

```bash
export RELATIVE_MPC_PREFIX=/actual/osqp-osqpeigen-prefix
./scripts/start_sitl.sh --tracking-mode RELATIVE_MPC
```

## 安全默认值

```text
relative descent: disabled
final descent: disabled
NAV_LAND: disabled
automatic Disarm: disabled
terminal contact stabilization: disabled
```

负固定倾角、动态 `rollpitch`、`combined` 和 `rigid_body_motion` 不允许下降或真实接触。6-DoF shadow 在控制 setpoint 发布后运行，不修改状态机、窗口、MPC 或下降参考。

6-DoF 参数统一位于 `deck_motion_shadow.*`：jerk/角 jerk、位置/姿态测量噪声、
初始协方差、采样与重初始化时间门、位置/姿态 Mahalanobis 门、向上法向门、
`0.30 s` 局部导数拟合窗，以及冻结的 `0.05 / 0.50 / 1.00 s` 采样/可信/最大输出
时域。完整默认值见 `config/px4_aruco_landing.yaml`。

当前 state 位姿使用 `uav_centered_ned`，trajectory 使用每条消息冻结的
`uav_origin_ned`；两者的 twist 均为甲板自身 NED twist。旧非共面 Board 的约
`5 m` 正式矩阵为安全隔离 `12/12`、冻结全性能硬门 `2/12`。Marine Planar
Board RefineLM 的 `rendezvous-altitude=7.0` 固定 4×3 正式复验为安全门 `12/12`、
Board 门 `12/12`、全性能硬门 `0/12`；static current-normal 已通过冻结的
`1.0° RMSE / 1.5° P95` 门。Shadow 仍仅用于诊断。Future Twist causal diagnosis 已完成；
固定 Bag 的 production-math estimator replay 可覆盖全部 origin，但因缺失 odometry callback
ROS receipt-time provenance 未通过冻结 equivalence gate，所以 covariance confidence 尚未评分，
也不授权 NMPC 或动态姿态下降。

## 主要状态

```text
WAIT_DECK_GNSS
RENDEZVOUS_GNSS
ACQUIRE_ARUCO
VISUAL_HANDOVER
TRACK_TARGET
WAIT_LANDING_WINDOW
DESCEND
FINAL_DESCENT
TOUCHDOWN_CANDIDATE_HOLD
TOUCHDOWN_HOLD
RECOVER_TO_GNSS
RECOVER_CLIMB
ABORT
```

## 构建与测试

```bash
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash

colcon build --symlink-install --packages-select aruco_precision_landing_cpp
colcon test --packages-select aruco_precision_landing_cpp
colcon test-result --verbose
```

## 评测

```bash
python3 scripts/evaluate_horizontal_tracking.py bags/<bag>
python3 scripts/evaluate_relative_descent.py bags/<bag>
python3 scripts/evaluate_final_descent_touchdown.py bags/<bag>
python3 scripts/evaluate_tilted_deck.py bags/<bag> --scenario tilt_roll_pos_2deg
python3 scripts/evaluate_deck_motion_shadow.py bags/<bag>
python3 scripts/replay_deck_motion_estimator.py \
  --matrix-dir results/deck_motion_shadow_planar_marine_refinelm_local7m_20260815
```

完整架构、参数语义和安全边界见：

- [`docs/reference/SYSTEM_OVERVIEW.md`](../../docs/reference/SYSTEM_OVERVIEW.md)
- [`docs/reference/LANDING_CONTROL_THEORY.md`](../../docs/reference/LANDING_CONTROL_THEORY.md)
- [`docs/guides/OPERATIONS.md`](../../docs/guides/OPERATIONS.md)
