# ws_aruco_landing

`ws_aruco_landing` 是一个基于 **ROS 2 Humble、PX4 SITL 和 Gazebo Harmonic** 的移动船舶无人机自主降落传统基线工作空间。

系统实现从船舶 GNSS 粗引导到视觉接管、移动目标跟踪、规则式着陆窗口、相对高度下降和多源触地确认的完整链路，并提供可选的水平相对运动线性 MPC。P8A 升沉触地、P8B MPC 与 P8C fixed T1 均已通过真实 PX4 SITL 验收；P8C-3 的失败 Bag 和设计门关闭证据继续完整保留。P9 统一评测第一版已完成：smoke `20/27`，正式 baseline static/constant02 `20+20` 为 `40/40`，正式消融 `60/60`，三个 smoke 关闭组合保留为 `30` 个 `NOT_APPLICABLE` 槽位。三批次合计 `120/127`，全部 7 个失败均为 smoke `SAFETY_GATE_FAILURE`；正式批次保持 `NAV_LAND / Disarm = 0 / 0`，Ground Truth 仅用于离线 evaluator。

```text
船舶 GNSS 会合
→ ArUco 捕获与视觉接管
→ 甲板状态估计和短时预测
→ 移动目标水平跟踪
→ 着陆窗口判断
→ 相对高度分阶段下降
→ 最终下降与触地确认
→ 接触后相对保持
```

## ROS 2 包

| 包 | 职责 |
| --- | --- |
| [`aruco_detector`](src/aruco_detector/README.md) | ArUco 检测、多尺度 Marker 选择、完整 PnP 位姿和调试图像。 |
| [`aruco_precision_landing_cpp`](src/aruco_precision_landing_cpp/README.md) | PX4 Offboard、GNSS—视觉接管、状态估计、跟踪、下降和触地确认。 |
| [`moving_deck_sim`](src/moving_deck_sim/README.md) | 移动甲板、船舶 GNSS 传感器模型和离线评测 Ground Truth。 |

控制器禁止订阅 `/simulation/deck/ground_truth`；该话题只允许用于仿真传感器和离线评测。

## 快速安装

以下快速构建步骤假设系统已经安装并配置：

- Ubuntu 22.04、ROS 2 Humble 和 Gazebo Harmonic；
- PX4 SITL、MicroXRCEAgent 和与 PX4 版本匹配的 `px4_msgs`；
- OSQP `v1.0.0` 与 OsqpEigen `v0.11.2`，默认前缀为 `~/.local/p8b-mpc/osqp-1.0.0-osqpeigen-0.11.2`。

Ubuntu、ROS 2 和 Gazebo 以外依赖的固定版本、源码安装和验证步骤见[完整安装教程](docs/guides/OPERATIONS.md#1-环境安装与构建)。

```bash
git clone https://github.com/yjMa23/-ws_aruco_landing.git ws_aruco_landing
cd ws_aruco_landing

source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash

export P8B_MPC_PREFIX="$HOME/.local/p8b-mpc/osqp-1.0.0-osqpeigen-0.11.2"
export CMAKE_PREFIX_PATH="$P8B_MPC_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$P8B_MPC_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

colcon build --symlink-install \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
```

如果 PX4、`px4_msgs` 或求解器位于其他目录，请在启动前设置 `PX4_DIR`、`PX4_MSGS_WS` 或 `P8B_MPC_PREFIX`。

## 快速启动

默认启动静止甲板安全场景：

```bash
./scripts/start_sitl.sh
```

脚本会启动 Agent、PX4、Gazebo、移动甲板、相机桥接、ArUco 检测器和控制器。确认 PX4/QGroundControl 状态正常后按回车，控制器才会自动切换 Offboard 并 Arm。
QGC需手动打开。

默认配置只跟踪甲板并保持安全高度：相对下降和最终下降均关闭，不发送 `NAV_LAND`，不自动 Disarm。按 `Ctrl-C` 可统一停止本轮 SITL 进程。

## P8C fixed T1 当前边界

P8C-3 已实现固定正 `+2° roll/pitch` 的严格 final-descent 白名单、无人值守状态监控、Bag 和离线接触评测，并保留了水平机体方案的失败证据。P8C-4 随后完成独立研究、接口冻结、TDD、终端接触稳定化实现和分级真实验证；该方案继续使用 PX4 Offboard position setpoint，在终端阶段叠加法向整形、接触锚点顺应、切向阻尼和受限预压，不是直接发送 attitude setpoint 的姿态对齐。

```text
P8C-3 FAILURE EVIDENCE PRESERVED
P8C-4 VALIDATION PASS
P8C T1 VALIDATION PASS
P8C-3 DESIGN GATE CLOSED
```

固定 T1 结论只覆盖正 `+2° roll/pitch`。负倾角 touchdown、动态 `rollpitch/combined`、动态姿态 final descent、Ground Truth 控制、`NAV_LAND` 和自动 Disarm 仍关闭，不能由 fixed T1 结果直接外推。历史失败证据位于 `results/p8c3_validation_20260802/`，最终成功证据位于 `results/p8c4_validation_20260802/`。

## P9 当前状态

```text
P9 PLAN / IMPLEMENTATION / TEST: PASS
P9 smoke: COMPLETE（20/27，7 SAFETY_GATE_FAILURE）
P9 baseline 20+20: COMPLETE（40/40）
P9 formal ablation: COMPLETE（60/60，30 NOT_APPLICABLE slots）
P9 aggregation: PASS
P9 unified evaluation v0.1: PASS
```

正式运行提交为 `71af1cc`；baseline 目录为 `results/p9_baseline_20x20_20260804_71af1cc/`，ablation 目录为 `results/p9_ablation_20260804_71af1cc/`。B2 constant02、B4 heave_h1 和 B5 pitch `+2°` 被 smoke 安全门关闭，不调参、不换 seed、不进入正式失败分母；B5 roll `+2°` 是 fixed T1 唯一正式 touchdown 组合。完整统计、失败证据和图表路径见 `docs/validation/P9_UNIFIED_EVALUATION_VALIDATION.md`。

## 详细文档

- [文档索引](docs/README.md)
- [安装、构建、启动、实验与故障排查](docs/guides/OPERATIONS.md)
- [系统架构与接口总览](docs/reference/SYSTEM_OVERVIEW.md)
