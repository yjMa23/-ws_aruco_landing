# ws_aruco_landing

基于 ROS 2 Humble、PX4 SITL 和 Gazebo Harmonic 的移动船舶无人机自主降落传统基线。

系统已经实现船舶 GNSS 会合、ArUco 视觉接管、甲板状态估计与预测、移动目标跟踪、着陆窗口、相对高度下降、多源触地确认和接触后保持。水平控制默认使用加速度感知的规则式跟踪，也可显式启用四状态相对 MPC。

另有独立的甲板 6-DoF shadow 估计与 `0–1.0 s` 预测链路。当前位姿是
`uav_centered_ned` 中的 `deck-uav`，预测轨迹相对发布时冻结的无人机原点；
输入为时间对齐 ArUco 相对位姿和 PX4 NED 速度，不使用甲板 Ground Truth。约 `5 m`
正式 SITL 的安全隔离为 `12/12`、全性能硬门为 `2/12`；当前位姿和未来位置已
通过，失败集中在未来 twist，因此尚不能接入控制。

```text
船舶 GNSS 会合
→ ArUco 捕获与视觉接管
→ 状态估计和短时预测
→ 水平相对跟踪
→ 着陆窗口
→ 相对下降与最终下降
→ 触地确认和接触保持
```

## ROS 2 包

| 包 | 职责 |
| --- | --- |
| [`aruco_detector`](src/aruco_detector/README.md) | 多尺度 ArUco 检测、完整 PnP 位姿和调试图像。 |
| [`aruco_precision_landing_cpp`](src/aruco_precision_landing_cpp/README.md) | PX4 Offboard、GNSS—视觉接管、估计、跟踪、下降和触地确认。 |
| [`moving_deck_sim`](src/moving_deck_sim/README.md) | 移动甲板、船舶 GNSS 传感器模型和离线评测 Ground Truth。 |

控制器禁止订阅 `/simulation/deck/ground_truth`；该话题只允许用于仿真传感器和离线评测。

## 构建

需要 Ubuntu 22.04、ROS 2 Humble、Gazebo Harmonic、PX4 SITL、MicroXRCEAgent、匹配的 `px4_msgs`，以及 OSQP `v1.0.0` 和 OsqpEigen `v0.11.2`。

```bash
git clone https://github.com/yjMa23/-ws_aruco_landing.git ws_aruco_landing
cd ws_aruco_landing

source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash

export RELATIVE_MPC_PREFIX="$HOME/.local/relative-mpc/osqp-1.0.0-osqpeigen-0.11.2"
export CMAKE_PREFIX_PATH="$RELATIVE_MPC_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$RELATIVE_MPC_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

colcon build --symlink-install \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
```

完整依赖安装和排障见[操作指南](docs/guides/OPERATIONS.md)。已有求解器安装可通过 `RELATIVE_MPC_PREFIX` 指向实际目录，不要求移动外部文件。

## 启动

```bash
./scripts/start_sitl.sh
```

默认只跟踪静止甲板并保持安全高度。相对下降、最终下降、`NAV_LAND` 和自动 Disarm 均关闭；确认 PX4/QGroundControl 正常后，控制器才允许切换 Offboard 并 Arm。

常用安全检查：

```bash
./scripts/start_sitl.sh --scenario static --dry-run
./scripts/start_sitl.sh --scenario rollpitch --dry-run
./scripts/start_sitl.sh --scenario combined --dry-run
./scripts/start_sitl.sh --scenario rigid_body_motion --dry-run
```

## 当前边界

- 静止、水平运动、升沉以及固定正 `+2° roll/pitch` 已完成真实 PX4 SITL 接触验证。
- 固定正倾角接触稳定化使用 Offboard position setpoint 内的法向整形、接触锚点顺应、切向阻尼和受限预压，不发送 PX4 attitude setpoint。
- 负固定倾角、动态 `rollpitch`、`combined` 和 `rigid_body_motion` 仍只允许安全高度观察，下降和真实接触关闭。
- 统一评测记录为 smoke `20/27`、正式基线 `40/40`、正式消融 `60/60`，另有 `30` 个 `NOT_APPLICABLE` 槽位。有限样本全成功不代表真实成功概率为 100%。
- 6-DoF 相对 shadow 的约 `5 m` 正式矩阵为安全 `12/12`、全性能硬门
  `2/12`；剩余限制是 ArUco-only 动态甲板 `0.5 s` 未来 twist 可观测性。
- 所有正式实验保持 `NAV_LAND / Disarm = 0 / 0`。

## 文档

- [文档索引](docs/README.md)
- [当前实现](docs/reference/SYSTEM_OVERVIEW.md)
- [控制理论](docs/reference/LANDING_CONTROL_THEORY.md)
- [坐标与时间契约](docs/reference/COORDINATE_FRAMES.md)
- [操作指南](docs/guides/OPERATIONS.md)
- [论文结果](docs/results/PAPER_RESULTS.md)
- [数据来源与哈希](docs/results/DATA_PROVENANCE.md)
- [下一步计划](docs/plans/NEXT_DEVELOPMENT_PLAN.md)
