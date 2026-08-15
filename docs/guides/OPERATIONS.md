# 安装、运行与实验操作指南

## 1. 环境

推荐环境：

- Ubuntu 22.04。
- ROS 2 Humble。
- Gazebo Harmonic。
- PX4 SITL 和 MicroXRCEAgent。
- 与 PX4 分支匹配的 `px4_msgs` 工作区。
- OSQP `v1.0.0` 与 OsqpEigen `v0.11.2`。

示例环境变量：

```bash
export PX4_DIR="$HOME/PX4-Autopilot"
export PX4_MSGS_WS="$HOME/ws_sensor_combined"
export RELATIVE_MPC_PREFIX="$HOME/.local/relative-mpc/osqp-1.0.0-osqpeigen-0.11.2"

source /opt/ros/humble/setup.bash
source "$PX4_MSGS_WS/install/setup.bash"
export CMAKE_PREFIX_PATH="$RELATIVE_MPC_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$RELATIVE_MPC_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

已有求解器安装不必移动；把 `RELATIVE_MPC_PREFIX` 指向包含 `lib/cmake/osqp` 和 `lib/cmake/OsqpEigen` 的实际前缀即可。

## 2. 构建和测试

```bash
cd /path/to/ws_aruco_landing
source /opt/ros/humble/setup.bash
source "$PX4_MSGS_WS/install/setup.bash"

colcon build --symlink-install \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash

colcon test
colcon test-result --verbose
```

只构建主包：

```bash
colcon build --symlink-install \
  --packages-select moving_deck_sim aruco_detector aruco_precision_landing_cpp
```

## 3. 启动 SITL

默认静止甲板、安全高度：

```bash
./scripts/start_sitl.sh
```

脚本会启动 MicroXRCEAgent、PX4、Gazebo、甲板仿真、相机桥接、ArUco 检测器和控制器。非自动批量模式下必须在终端确认，控制器才会切换 Offboard 并 Arm。

常用场景：

```bash
./scripts/start_sitl.sh --scenario constant02
./scripts/start_sitl.sh --scenario constant
./scripts/start_sitl.sh --scenario sinusoidal
./scripts/start_sitl.sh --scenario heave_h1
./scripts/start_sitl.sh --scenario tilt_roll_pos_2deg
./scripts/start_sitl.sh --scenario rollpitch
./scripts/start_sitl.sh --scenario combined
./scripts/start_sitl.sh --scenario rigid_body_motion
```

无 GUI：

```bash
./scripts/start_sitl.sh --scenario static --headless
```

只验证参数和安全门：

```bash
./scripts/start_sitl.sh --scenario static --dry-run
./scripts/start_sitl.sh --scenario rollpitch --dry-run
./scripts/start_sitl.sh --scenario combined --dry-run
./scripts/start_sitl.sh --scenario rigid_body_motion --dry-run
```

动态 `rollpitch`、`combined` 和 `rigid_body_motion` 的下降、最终下降与主动接触控制会在启动前被拒绝。

### 3.1 Legacy 与 Marine 环境

默认环境是 legacy：

```bash
./scripts/start_sitl.sh --environment legacy --scenario static
```

它继续使用 `aruco_moving_deck.sdf`、`moving_deck` 和历史 UAV spawn，不改变 P9/P10 复现路径。

Marine M2 需要显式选择：

```bash
./scripts/start_sitl.sh --environment marine --scenario static
./scripts/start_sitl.sh --environment marine --scenario rigid_body_motion \
  --rendezvous-altitude 7.0
```

marine 使用 `aruco_marine_vessel.sdf`、`vrx_wamv_landing/vessel_body`、固定 `landing_deck` frame 和独立 `x≈-12 m` UAV launch platform。WAM-V visual 来自固定 VRX commit 的官方 mesh/PBR maps；海面启用 `waterlow.dae + WaveVisual + Gerstner shader` 的 dynamic visual-only VRX ocean。launch 会把既有 scenario 的 neutral deck z=`2 m` 转成 WAM-V vessel reference z=`0.2 m`，并用固定 `r_VD=[0,0,1.8] m` 恢复 `/simulation/deck/ground_truth` 的 deck-center 语义。Marine 启动脚本同时改用 `aruco_detector_marine.yaml`，legacy 继续使用历史 `aruco_detector.yaml`。

marine 只允许 GNSS rendezvous、视觉捕获、安全高度跟踪和 deck-motion shadow。以下命令必须在启动前失败：

```bash
./scripts/start_sitl.sh --environment marine --scenario static \
  --enable-relative-descent --dry-run
./scripts/start_sitl.sh --environment marine --scenario static \
  --terminal-contact-stabilization-shadow --dry-run
```

M2 已使用 VRX WAM-V 视觉资产并启用 VRX `WaveVisual`：`waterlow.dae` 顶点与 bump-map UV 由 Gerstner shader 随 simulation time 动态更新。它仍是纯渲染能力，没有 wave-driven vessel dynamics、RAO、Buoyancy、Hydrodynamics、wind 或 current；`MotionProfile` 仍是唯一 vessel motion source。M1 基础构建说明见 [Marine Scene 构建说明](MARINE_SCENE_BUILD.md)，M2 上游/资产/build/SDF/GUI 验证见 [VRX WAM-V 构建说明](VRX_WAMV_BUILD.md)。

### 3.2 甲板 6-DoF Shadow

默认参数启用独立 shadow，但不改变飞行控制。检查话题：

```bash
ros2 topic echo /landing/deck_motion_shadow/state
ros2 topic echo /landing/deck_motion_shadow/trajectory
ros2 topic echo /landing/deck_motion_shadow/status
ros2 topic echo /landing/deck_motion_shadow/trusted_horizon_s
```

Marine 使用 `rendezvous-altitude=7.0` 时应在 `/aruco/debug_image` 中确认 ID4/5/6/7 至少多个同帧可见，并检查：

```bash
ros2 topic echo /aruco/pose_source
ros2 topic echo /aruco/board_marker_count
ros2 topic echo /aruco/board_reprojection_rmse_px
```

正常远距来源应主要为 `PLANAR_BOARD_MULTI`；只有一个大 Marker 可用时才是 `FAR_SINGLE`，远距全部离开视场后才由 ID0/1/2/3 `MarkerSelector` 给出 `NEAR_SINGLE`。若多 Marker 可见但持续 `NONE`，先检查共面 SDF/calibration、CameraInfo、角点质量、法向与 reprojection RMSE，不要调 shadow 创新门掩盖视觉退化。legacy 环境仍保留历史 ID0/4/5/6 非共面日志和语义。

旧非共面 Board 的正式约 `5 m` 甲板相对高度 12 轮矩阵（runner 固定
`rendezvous_altitude_m=7.0`，因为甲板表面位于 world `z=2.0 m`）：

```bash
python3 scripts/run_deck_motion_shadow_experiments.py \
  --output results/deck_motion_shadow_relative_5m_<date>
```

当前相对方案冻结矩阵位于
`results/deck_motion_shadow_relative_5m_20260809`，结果为安全隔离 `12/12`、全性能
硬门 `2/12`。新运行必须使用新目录，不得覆盖现有正式 Bag。

Marine Planar Board 正式矩阵必须显式选择环境；runner 会自动启用
`--planar-board` 评测，并保存 scenario、controller 和 detector 参数快照：

```bash
python3 scripts/run_deck_motion_shadow_experiments.py \
  --environment marine \
  --output results/deck_motion_shadow_planar_marine_local7m_<date>
```

2026-08-13 IPPE-only 冻结结果位于
`results/deck_motion_shadow_planar_marine_local7m_20260813`：安全门 `12/12`、Planar
Board 门 `9/12`、完整 shadow 硬门 `0/12`，作为 static 法向修复前基准保留。
2026-08-15 RefineLM 后固定复验位于
`results/deck_motion_shadow_planar_marine_refinelm_local7m_20260815`：安全门 `12/12`、
Planar Board 门 `12/12`、完整 shadow 硬门仍为 `0/12`。后续新运行仍必须使用新目录，
不得覆盖任一正式 Bag。Future Twist causal diagnosis 已完成；固定 Bag production-math replay
因缺失 odometry callback ROS receipt-time provenance 未通过 equivalence gate。当前先补 replay
timing provenance，不回调 Board hard gate、SUBPIX 或 shadow current-pose filter。命令中的 `7.0 m` 是本地高度目标，
不是相机到甲板的直接距离；不同正式矩阵不得未经证据声明严格等高非劣效。

单 Bag 重新评测：

```bash
python3 scripts/evaluate_deck_motion_shadow.py \
  results/deck_motion_shadow_relative_5m_20260809/static_s1/bag \
  --output results/deck_motion_shadow_relative_5m_20260809/static_s1/evaluation.json
```

Marine Planar Board Bag 单独重评测时增加模式参数：

```bash
python3 scripts/evaluate_deck_motion_shadow.py \
  results/deck_motion_shadow_planar_marine_refinelm_local7m_20260815/static_s1/bag \
  --planar-board \
  --output results/deck_motion_shadow_planar_marine_refinelm_local7m_20260815/static_s1/evaluation.json
```

`rendezvous_altitude_m` 是 PX4 local NED 原点上的高度目标，不是相机到甲板的直接距离；实际相对高度由 evaluator 报告。只有约 `5 m` 正式结果失败且证据明确指向视觉分辨率时，才以本地高度 `5.0 m` 运行同 seed 的约 `3 m` 无下降诊断：

```bash
./scripts/start_sitl.sh --scenario <scenario> --seed <seed> \
  --rendezvous-altitude 5.0 --headless --record
```

`3 m` 不替代 `5 m` 失败。当前相对方案在 `5 m` 的当前位姿和未来位置门已
`12/12` 通过，失败集中在未来 twist，因此没有触发新的 `3 m` 分辨率对照。
不要修改 `deck_motion_shadow.*` 门限后重跑正式 Bag。

固定 12 Bag 的 estimator replay 可运行：

```bash
python3 scripts/replay_deck_motion_estimator.py \
  --matrix-dir results/deck_motion_shadow_planar_marine_refinelm_local7m_20260815
```

该脚本直接调用 C++ `deck_motion_estimator_replay`，复用 production `DeckMotionEstimator`、
`VehiclePoseHistory` 和 coordinate transform。当前固定 Bag 预期会因 replay equivalence 未通过而
返回非零；这是 fail-closed，不应通过放宽 tolerance 或读取 covariance 绕过。完整诊断见
[`FUTURE_TWIST_ESTIMATOR_CONFIDENCE.md`](../reference/FUTURE_TWIST_ESTIMATOR_CONFIDENCE.md)。

## 4. 跟踪模式

默认规则式跟踪：

```bash
./scripts/start_sitl.sh \
  --tracking-mode PREDICTED_POSITION_VELOCITY_FF
```

相对 MPC：

```bash
RELATIVE_MPC_PREFIX=/actual/osqp-osqpeigen-prefix \
./scripts/start_sitl.sh \
  --tracking-mode RELATIVE_MPC
```

MPC 求解失败或进入接触敏感终端阶段时自动使用规则式跟踪。启用 MPC 不会自动打开下降。

## 5. 下降和接触安全门

相对下降必须显式启用：

```bash
./scripts/start_sitl.sh \
  --scenario static \
  --enable-relative-descent \
  --descent-test-height 0.50
```

最终下降还必须额外启用：

```bash
./scripts/start_sitl.sh \
  --scenario static \
  --enable-relative-descent \
  --descent-test-height 0.50 \
  --enable-final-descent
```

固定正 `+2°` 倾角终端稳定化：

```bash
./scripts/start_sitl.sh \
  --scenario tilt_roll_pos_2deg \
  --enable-relative-descent \
  --descent-test-height 0.50 \
  --enable-final-descent \
  --enable-terminal-contact-stabilization
```

该命令仅限已经白名单的正固定倾角。负倾角、`rollpitch` 和 `combined` 会被拒绝；`--environment marine` 对所有 scenario 都额外拒绝 relative descent、final descent 和任何 terminal-contact stabilization 模式。

以下行为始终保持关闭，除非未来有独立授权：

```text
NAV_LAND
Automatic Disarm
动态姿态下降或接触
Ground Truth 控制
```

## 6. Bag 录制

```bash
./scripts/start_sitl.sh \
  --scenario constant02 \
  --record \
  --bag-output bags/constant02_tracking
```

`bags/` 默认不进入 Git。成功批次可使用轻量 Bag，失败批次应保留诊断话题和参数快照。

## 7. 离线评测

```bash
python3 scripts/evaluate_horizontal_tracking.py bags/<bag>
python3 scripts/evaluate_landing_window.py bags/<bag>
python3 scripts/evaluate_relative_descent.py bags/<bag>
python3 scripts/evaluate_vertical_estimation.py bags/<bag>
python3 scripts/evaluate_touchdown_evidence.py bags/<bag>
python3 scripts/evaluate_final_descent_touchdown.py bags/<bag>
python3 scripts/evaluate_heave_touchdown.py bags/<bag>
python3 scripts/evaluate_tilted_deck.py bags/<bag> \
  --scenario tilt_roll_pos_2deg
```

固定倾角安全下降评测：

```bash
python3 scripts/evaluate_tilted_deck.py bags/<bag> \
  --scenario tilt_roll_pos_2deg \
  --fixed-tilt-safe-descent
```

固定倾角触地与终端稳定化评测：

```bash
python3 scripts/evaluate_tilted_deck.py bags/<bag> \
  --scenario tilt_roll_pos_2deg \
  --tilted-deck-touchdown \
  --terminal-stabilization \
  --terminal-stabilization-mode active
```

Ground Truth 只在这些离线 evaluator 中用于误差和接触几何比较。

## 8. 单轮与批量实验

单轮 dry-run：

```bash
python3 scripts/run_single_experiment.py \
  --scenario static \
  --seed 1 \
  --dry-run
```

预置批次：

```bash
python3 scripts/run_batch_experiments.py config/experiments/touchdown_smoke.yaml
python3 scripts/run_batch_experiments.py config/experiments/heave_h1_smoke.yaml
python3 scripts/run_batch_experiments.py config/experiments/paper_evaluation_smoke.yaml
python3 scripts/run_batch_experiments.py config/experiments/paper_baseline_20x20.yaml
python3 scripts/run_batch_experiments.py config/experiments/paper_ablation.yaml
```

恢复未完成批次：

```bash
python3 scripts/run_batch_experiments.py \
  config/experiments/paper_ablation.yaml \
  --resume \
  --batch-id <existing_batch_id>
```

`NOT_APPLICABLE` 组合不执行，也不计入失败分母。真实失败不得通过换 seed、删除轮次或放宽门槛覆盖。

## 9. 聚合和论文结果

聚合批次：

```bash
python3 scripts/aggregate_results.py results/<batch_directory>
```

从三个冻结输入重建论文结果：

```bash
python3 scripts/finalize_paper_results.py \
  --smoke results/paper_evaluation_smoke_20260803 \
  --baseline results/paper_baseline_20x20_20260804_71af1cc \
  --ablation results/paper_ablation_20260804_71af1cc \
  --output results/paper_results_v0.1
```

脚本只读取结构化证据，不运行 SITL、不改控制器参数。结果摘要见 `docs/results/PAPER_RESULTS.md`，来源和哈希见 `docs/results/DATA_PROVENANCE.md`。

## 10. 结束和清理

前台运行时按 `Ctrl-C`，启动脚本会按记录的 PID 清理子进程。若异常退出，先检查：

```bash
pgrep -af 'px4|gz sim|MicroXRCEAgent|ros2 bag|aruco|moving_deck'
```

不要使用宽泛的递归删除或对整个工作区执行 `pkill -9`。只终止本轮明确记录的进程。

## 11. 常见问题

### 找不到 OSQP/OsqpEigen

确认：

```bash
test -f "$RELATIVE_MPC_PREFIX/lib/cmake/osqp/osqp-config.cmake"
test -f "$RELATIVE_MPC_PREFIX/lib/cmake/OsqpEigen/OsqpEigenConfig.cmake"
```

### PX4 ROS 话题未就绪

确认 MicroXRCEAgent、PX4 SITL 和 `px4_msgs` 版本匹配，并检查：

```bash
ros2 topic list | rg '^/fmu/'
```

### ArUco 长时间不可见

检查相机桥接、`/camera/camera_info`、`/aruco/debug_image`、仿真时间和 Marker 尺寸配置。不要用 Ground Truth 绕过视觉问题。

### 动态场景无法下降

这是当前安全门的预期行为。`rollpitch`、`combined` 和 `rigid_body_motion` 只允许安全高度 shadow，见[下一步计划](../plans/NEXT_DEVELOPMENT_PLAN.md)。
