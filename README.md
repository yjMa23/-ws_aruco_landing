# ws_aruco_landing

基于 ROS 2 Humble、PX4 SITL 和 Gazebo Harmonic 的移动船舶无人机自主降落传统基线。

系统已经实现船舶 GNSS 会合、ArUco 视觉接管、甲板状态估计与预测、移动目标跟踪、着陆窗口、相对高度下降、多源触地确认和接触后保持。水平控制默认使用加速度感知的规则式跟踪，也可显式启用四状态相对 MPC。

另有独立的甲板 6-DoF shadow 估计与 `0–1.0 s` 预测链路。当前位姿是
`uav_centered_ned` 中的 `deck-uav`，预测轨迹相对发布时冻结的无人机原点；
输入为时间对齐 ArUco 相对位姿和 PX4 NED 速度，不使用甲板 Ground Truth。旧非共面
Board 的约 `5 m` 正式 SITL 为安全隔离 `12/12`、全性能硬门 `2/12`。Marine
Planar Board 在 IPPE 双解消歧后加入同帧全角点 `solvePnPRefineLM`，固定
`rendezvous-altitude=7.0` 的 4×3 正式复验为安全门 `12/12`、Board 门 `12/12`、
全性能硬门 `0/12`；Board static 法向阻塞已解决，当前下一任务是 Future Twist 因果诊断。

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
| [`moving_deck_sim`](src/moving_deck_sim/README.md) | legacy 移动甲板、marine 船体/固定着陆甲板、船舶 GNSS 传感器模型和离线评测 Ground Truth。 |

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

默认环境仍为 `legacy`，保持历史 `aruco_moving_deck.sdf`、`moving_deck` 和 UAV spawn 行为不变。Marine 环境需要显式选择：

```bash
./scripts/start_sitl.sh --environment marine --scenario static
./scripts/start_sitl.sh --environment marine --scenario rigid_body_motion --rendezvous-altitude 7.0
```

marine 已切换到官方 VRX WAM-V base mesh/PBR 资产，并启用 `waterlow.dae + WaveVisual + Gerstner shader` 的动态 visual-only ocean；海面顶点与 bump-map UV 随 Gazebo simulation time 变化，但不产生 collision、浮力或波浪力。WAM-V 上增加 `2.4×2.4 m` UAV landing platform。Marine 远距视觉目标独立改为平贴甲板的 ID4/5/6/7 四 Marker Planar Board（均 `0.50 m`，中心 `(±0.78,±0.78,0.002) m`），使用 IPPE multi-marker 双解消歧后 `solvePnPRefineLM` 精化、单远距 Marker deck-center fallback，并继续保留 ID0/1/2/3 近距 MarkerSelector；legacy 历史非共面几何不变。`MotionProfile` 仍是 `vessel_body` 唯一运动源，固定 `T_vessel_deck`（`r_VD=[0,0,1.8] m`）将 WAM-V canonical vessel state 转换成 landing deck center Ground Truth；neutral vessel z≈0.2 m 时 deck center 仍为 world z≈2.0 m。marine 仍只保持安全高度，会在启动前拒绝相对下降、最终下降和全部 terminal-contact 模式，`NAV_LAND` 与自动 Disarm 仍未启用。

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
- 旧非共面 Board 的 6-DoF 相对 shadow 约 `5 m` 正式矩阵为安全 `12/12`、全性能硬门 `2/12`。
- Marine Planar Board RefineLM 正式 4×3 已完成：安全 `12/12`、Board `12/12`、全性能硬门 `0/12`。static seed1/2/3 current-normal `RMSE/P95` 为 `0.151°/0.291°`、`0.134°/0.252°`、`0.145°/0.288°`；Board static 法向任务已冻结，当前进入 Future Twist causal diagnosis。
- marine 已完成 WAM-V 视觉资产、固定 UAV landing platform、visual-only `WaveVisual` 动态 Gerstner 海面和可实船平贴部署的 Planar ArUco Board 实现；仍没有 wave-driven vessel dynamics、JONSWAP/PM 船体响应、RAO、浮力、水动力、洋流或风载。
- 所有正式实验保持 `NAV_LAND / Disarm = 0 / 0`。

## 文档

- [文档索引](docs/README.md)
- [当前实现](docs/reference/SYSTEM_OVERVIEW.md)
- [控制理论](docs/reference/LANDING_CONTROL_THEORY.md)
- [坐标与时间契约](docs/reference/COORDINATE_FRAMES.md)
- [Marine vessel 刚体运动学](docs/reference/MARINE_VESSEL_KINEMATICS.md)
- [操作指南](docs/guides/OPERATIONS.md)
- [Marine scene 构建与验证](docs/guides/MARINE_SCENE_BUILD.md)
- [VRX WAM-V 集成契约](docs/reference/VRX_WAMV_INTEGRATION.md)
- [VRX WAM-V 构建说明](docs/guides/VRX_WAMV_BUILD.md)
- [Marine Planar ArUco Board 理论](docs/reference/PLANAR_ARUCO_BOARD.md)
- [Marine Planar ArUco Board 构建与验证](docs/guides/PLANAR_ARUCO_BOARD_BUILD.md)
- [Marine Planar Board 法向误差诊断](docs/reference/PLANAR_BOARD_ORIENTATION_DIAGNOSIS.md)
- [论文结果](docs/results/PAPER_RESULTS.md)
- [数据来源与哈希](docs/results/DATA_PROVENANCE.md)
- [下一步计划](docs/plans/NEXT_DEVELOPMENT_PLAN.md)
