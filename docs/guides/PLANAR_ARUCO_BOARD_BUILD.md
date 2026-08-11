# Marine 共面 ArUco Board 构建与验证

## 1. 范围

本说明只描述 Marine 的共面远距 Board。Legacy 继续使用 `src/aruco_detector/config/aruco_detector.yaml` 与 frozen historical Marker geometry，不修改其实验语义。

Marine 使用独立 detector 参数文件 `src/aruco_detector/config/aruco_detector_marine.yaml`，由 `scripts/start_sitl.sh --environment marine` 显式选择。

## 2. Marker 打印与安装参数

远距 Board 使用 OpenCV `DICT_4X4_50`：

| ID | 实际边长 | deck-frame xyz (m) | rpy (rad) | SDF texture |
|---|---:|---|---|---|
| 4 | 0.50 m | `+0.78 +0.78 0.002` | `0 0 0` | `model://moving_deck/aruco_id4.pgm` |
| 5 | 0.50 m | `+0.78 -0.78 0.002` | `0 0 0` | `model://moving_deck/aruco_id5.pgm` |
| 6 | 0.50 m | `-0.78 +0.78 0.002` | `0 0 0` | `model://moving_deck/aruco_id6.pgm` |
| 7 | 0.50 m | `-0.78 -0.78 0.002` | `0 0 0` | `model://moving_deck/aruco_id7.pgm` |

上述 `0.50 m` 指 Marker 黑白图案平面在仿真和 detector 标定中的几何边长。四个 Marker 必须平贴，打印面法向与 landing deck +z 一致。

中央近距 Marker 继续为：

```text
ID0 0.50 m
ID1 0.20 m
ID2 0.04 m
ID3 0.02 m
```

不得为了安装远距 Board 改变 ID0/1/2/3 的 MarkerSelector 阈值、迟滞、border margin、stable challenger 或 missing grace。

## 3. SDF 落点

文件：

```text
src/moving_deck_sim/models/vrx_wamv_landing/model.sdf
```

所有 ID4/5/6/7 visual 必须：

- `pose relative_to="landing_deck"`；
- `roll=pitch=yaw=0`；
- `plane normal = 0 0 1`；
- `plane size = 0.50 0.50`；
- z offset 固定为 `0.002 m`；
- 不超出 ±1.20 m 甲板边界；
- 不遮挡中央 ID0/1/2/3。

ID7 纹理沿用仓库现有 PGM 资产方式，不引入额外运行时依赖。

## 4. Detector calibration

Marine 参数文件：

```text
src/aruco_detector/config/aruco_detector_marine.yaml
```

关键参数：

```text
marker_ids: [0, 1, 2, 3]
marker_lengths_m: [0.50, 0.20, 0.04, 0.02]

far_board.enabled: true
far_board.marker_ids: [4, 5, 6, 7]
far_board.marker_lengths_m: [0.50, 0.50, 0.50, 0.50]
far_board.marker_poses_deck_xyz_rpy:
  [
    +0.78, +0.78, 0.002, 0, 0, 0,
    +0.78, -0.78, 0.002, 0, 0, 0,
    -0.78, +0.78, 0.002, 0, 0, 0,
    -0.78, -0.78, 0.002, 0, 0, 0
  ]
far_board.max_reprojection_rmse_px: 5.0
```

Legacy/default `aruco_detector.yaml` 保持现有 ID0/4/5/6 非共面配置，用于 frozen baseline 路径。

## 5. Detector 路由

Marine 每帧执行：

```text
detectMarkers
  -> collect ID4/5/6/7
  -> MarkerSelector only sees ID0/1/2/3

>=2 Board markers : SOLVEPNP_IPPE -> PLANAR_BOARD_MULTI
1 Board marker    : single marker pose + known T_marker_deck -> FAR_SINGLE
0 Board markers   : current MarkerSelector -> NEAR_SINGLE
invalid result    : NONE / visible=false
```

Planar Board 使用上一帧有效视觉 pose 作为 IPPE 候选消歧先验；不读取 Ground Truth。

## 6. 构建

```bash
cd <workspace>/ws_aruco_landing
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash

colcon build --symlink-install \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

预期：`aruco_detector_core` 编译 `planar_board_pose.cpp`，新增 GTest 被 CMake 注册，moving_deck_sim 安装 ID7 PGM 与更新后的 WAM-V model。

## 7. 单元与一致性测试

```bash
colcon test
colcon test-result --verbose
git diff --check
```

Planar C++ GTest 至少验证：

- calibration：重复 ID、非法 size、NaN/Inf、非法 pose、合法共面；
- pose：4/3/2-marker、检测顺序变化、未知 ID、非法相机内参、NaN/Inf 图像角点；
- 姿态：fronto-parallel、roll、pitch、yaw、roll+pitch；
- IPPE ambiguity：positive depth、deck normal、RMSE、上一帧连续性，不发生无理由 flip；
- position / rotation / normal error 与 finite RMSE。

Marine Python consistency test 必须比较 SDF 与 `aruco_detector_marine.yaml` 的 ID、size、xyz/rpy，并检查 legacy model/config 未被改写。

## 8. SITL smoke

安全 smoke：

```bash
./scripts/start_sitl.sh \
  --environment marine \
  --scenario static \
  --headless \
  --auto-confirm-controller \
  --rendezvous-altitude 7.0

./scripts/start_sitl.sh \
  --environment marine \
  --scenario rigid_body_motion \
  --headless \
  --auto-confirm-controller \
  --rendezvous-altitude 7.0
```

记录/检查：

```text
/aruco/pose_source
/aruco/board_marker_count
/aruco/board_reprojection_rmse_px
/aruco/visible
/aruco/pose
/landing/shadow/*
state machine final state
NAV_LAND count
automatic Disarm count
touchdown_confirmed
terminal-contact stabilization
```

成功条件：完成 GNSS rendezvous、捕获 Board、进入 safe-altitude tracking/shadow，且不下降、不接触。

## 9. GUI 人工检查

若 DISPLAY/Wayland 可用，检查：

- ID4/5/6/7 全部水平贴在 deck；
- 没有悬空或 ±45° 倾斜 Marker；
- 下视相机在远距可同时看到多个大 Marker；
- 中央落区与 ID0/1/2/3 未被遮挡。

若当前环境无 GUI，验证记录必须明确写：

```text
GUI visual verification pending
```

不得把 headless SDF 检查表述为已经完成肉眼 GUI 验收。

## 10. 正式约 5 m 4×3 shadow

只有 smoke 完全通过后，才执行：

```bash
python3 scripts/run_deck_motion_shadow_experiments.py \
  --environment marine \
  --output results/deck_motion_shadow_planar_marine_5m_<date>
```

该 runner 固定运行 `static / rollpitch / combined / rigid_body_motion × seed 1/2/3`，共 12 runs，并保持 `rendezvous_altitude=7.0 m`。`--environment` 默认仍为 `legacy`，因此历史调用语义不变。不得换 seed、放宽阈值、删除失败轮次或重挑结果。重点统计当前 deck pose coverage、position error、normal error、Board RMSE 和 pose flip；Future Twist 12/12 不是本阶段验收条件。

若本轮不执行 12-run，下一步计划必须保留“Planar Board 正式 5 m safe-altitude 4×3 验证”，并使用上面的 Marine batch 命令直接续跑。

## 11. 2026-08-11 本地验证记录

本次实现完成后的真实验证结果：

```text
colcon build: PASS
colcon test-result: 389 tests, 0 errors, 0 failures, 0 skipped
gz sdf -k src/moving_deck_sim/models/vrx_wamv_landing/model.sdf: Valid
```

Marine `static` headless smoke：

```text
final state: WAIT_LANDING_WINDOW
shadow: UPDATED:TRUSTED
pose_source: 401 PLANAR_BOARD_MULTI + 419 pre-acquisition NONE
board count: 2/3/4 Marker 均实际出现
board RMSE median/P95/max: 0.554 / 0.810 / 0.964 px
raw pose normal flip count: 0
shadow evaluator valid coverage: 1.0
current horizontal/vertical position P95: 0.030 / 0.006 m
current normal RMSE/P95: 1.129° / 1.929°
NAV_LAND: 0
Disarm command: 0
touchdown_confirmed: false
terminal stabilization: false
```

注意：static 的 current-normal 指标超过 frozen 旧非共面 evaluator hard gate `RMSE≤1.0° / P95≤1.5°`，不能把 smoke 写成全性能通过；需要正式 4×3 判断这是短 smoke 瞬态还是 Planar Board 的持续可观测性代价。

Marine `rigid_body_motion` headless smoke：

```text
final state: WAIT_LANDING_WINDOW
shadow: UPDATED:TRUSTED
pose_source: 435 PLANAR_BOARD_MULTI + 1 FAR_SINGLE + 430 pre-acquisition NONE
board count: 1/2/3/4 Marker 均实际出现
board RMSE median/P95/max: 0.411 / 0.482 / 0.847 px
raw pose normal flip count: 0
shadow evaluator valid coverage: 1.0
current horizontal/vertical position P95: 0.048 / 0.011 m
current normal RMSE/P95: 0.364° / 0.656°
NAV_LAND: 0
Disarm command: 0
touchdown_confirmed: false
terminal stabilization: false
```

当前验证 shell：

```text
DISPLAY=
WAYLAND_DISPLAY=
GUI visual verification pending
```

因此本次未伪造 GUI 人工检查。正式 Marine Planar Board 12-run 本轮未执行；runner 的 `--environment marine --dry-run` 已确认固定产生 `4 scenarios × seeds 1/2/3` 共 12 个 `rendezvous-altitude=7.0` episode。
