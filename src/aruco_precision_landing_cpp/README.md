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

负固定倾角、动态 `rollpitch` 和 `combined` 不允许下降或真实接触。

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
```

完整架构、参数语义和安全边界见：

- [`docs/reference/SYSTEM_OVERVIEW.md`](../../docs/reference/SYSTEM_OVERVIEW.md)
- [`docs/reference/LANDING_CONTROL_THEORY.md`](../../docs/reference/LANDING_CONTROL_THEORY.md)
- [`docs/guides/OPERATIONS.md`](../../docs/guides/OPERATIONS.md)
