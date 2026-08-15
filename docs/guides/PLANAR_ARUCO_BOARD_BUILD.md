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

### 5.1 Planar orientation refinement

根因诊断确认近正视共面观测的法向敏感性后，可在 IPPE 完成候选选择后调用一次 `cv::solvePnPRefineLM`。它只使用当前帧已有的 Board 角点、CameraInfo K/D 和已选 IPPE 位姿；不能使用场景信息、Ground Truth 或未来状态。

精化结果必须再次通过有限性、正深度、甲板法向方向和既有重投影阈值检查。只有重投影 RMSE 不比原候选差时才采用精化结果，否则保留原 IPPE 解。现有 `CORNER_REFINE_SUBPIX` 保持不变，也不修改 MarkerSelector、Board 阈值或 shadow 参数。单元测试需覆盖近正视亚像素扰动收益和倾斜姿态下的稳定性。

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

## 10. 正式 local 7 m 4×3 shadow

只有 smoke 完全通过后，才执行：

```bash
python3 scripts/run_deck_motion_shadow_experiments.py \
  --environment marine \
  --output results/deck_motion_shadow_planar_marine_local7m_<date>
```

该 runner 固定运行 `static / rollpitch / combined / rigid_body_motion × seed 1/2/3`，共 12 runs，并保持 `rendezvous_altitude=7.0 m`。Marine 模式自动启用 Planar Board evaluator，并在每轮保存 detector、controller 和 scenario 参数快照。`--environment` 默认仍为 `legacy`，因此历史调用语义不变。不得换 seed、放宽阈值、删除失败轮次或重挑结果。重点统计当前 deck pose coverage、position error、normal error、Board RMSE 和 pose flip；Future Twist 12/12 不是本阶段验收条件。

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

因此本次未伪造 GUI 人工检查。

## 12. 2026-08-13 正式矩阵记录

正式输出位于 `results/deck_motion_shadow_planar_marine_local7m_20260813`。原始 12 个
Bag 全部保留，未替换 seed、删除失败轮次或调整 detector/shadow 门限。结果为：

- 安全门 `12/12`：无下降、接触、穿透、`NAV_LAND`、Disarm、触地确认、实际
  terminal stabilization、时间同步错误或非法输出。
- Planar Board 门 `9/12`：9 个动态轮次通过；3 个 static seed 均仅因当前法向门
  失败，法向 `RMSE=1.186–1.421°`、`P95=2.010–2.455°`。
- 12 轮 ArUco 与 shadow 跟踪覆盖率均为 `100%`，有效视觉帧 multi 来源占比均为
  `100%`。全矩阵观察到 2/3/4 Marker；8 个单 Marker 帧全部路由到 `FAR_SINGLE`。
- multi pose reprojection RMSE P95 为 `0.584–0.913 px`、最大值为
  `0.819–1.108 px`；12 轮 raw deck-normal `>=90°` flip 均为 `0`。
- 当前水平/垂直位置 P95 为 `0.018–0.035 m / 0.005–0.011 m`；动态 9 轮当前
  法向 `RMSE/P95` 为 `0.453–0.549° / 0.885–1.183°`。
- `0.5 s` 预测水平/垂直位置与 yaw 门均为 `12/12`，法向、水平速度、垂直速度、
  角速度门分别为 `0/12`、`0/12`、`3/12`、`0/12`；完整 shadow 硬门 `0/12`。
- `rendezvous_altitude=7.0 m` 产生的实际相对高度总体为 `5.262–6.186 m`，不能与
  旧非共面 Board 矩阵声明严格等高非劣效。

因此该 2026-08-13 结果作为 **IPPE-only 修复前基准** 保留；它没有达到 Board `12/12`，
后续只据此定位 static 法向根因，没有调 Future Twist、shadow 或控制器。

## 13. 2026-08-15 RefineLM 修复与正式复验

固定历史 Bag 的 A/B/C 离线分解与 tilt-bin 诊断见
[`PLANAR_BOARD_ORIENTATION_DIAGNOSIS.md`](../reference/PLANAR_BOARD_ORIENTATION_DIAGNOSIS.md)。
证据支持 near-fronto-parallel 共面 Board orientation poor-conditioning 后，按本文第 5.1 节
实现最小修复：IPPE 完成合法候选选择后，以该 pose 为初值，对同一帧全部可见 Board
corners 调用 `solvePnPRefineLM`。refined pose 只有在 finite、全部点正深度、deck normal
方向正确、既有 reprojection gate 通过且 RMSE 不比原 IPPE 差时才采用；否则回退原解。

固定 synthetic regression 使用 Marine ID4/5/6/7 几何、`distance=5.5 m`、
`Gaussian σ=0.45 px`、固定 seed、每个倾角 500 trials：

```text
tilt    raw IPPE normal RMSE/P95    IPPE+LM normal RMSE/P95
0°      3.338° / 4.862°             0.484° / 0.831°
0.5°    2.970° / 4.300°             0.490° / 0.862°
2°      2.350° / 3.741°             0.499° / 0.820°
5°      1.285° / 2.197°             0.463° / 0.821°
```

同时用固定恶劣 corners 验证：当 LM 收敛到 deck normal 方向非法的 refined pose 时，
production estimator 保留原合法 IPPE pose。

全仓普通测试为：

```text
391 tests, 0 errors, 0 failures, 0 skipped
```

原 389 项全部保留，新增 2 个 Planar Board C++ regression。

随后重新运行 Marine safe-altitude smoke：

```text
static seed1:    current normal RMSE/P95 = 0.176° / 0.361°
rollpitch seed1: current normal RMSE/P95 = 0.149° / 0.249°
```

两轮 Safety/Planar Board gate 均通过，且 descent/contact/penetration/`NAV_LAND`/
Disarm/nonfinite 均为 0，touchdown confirmation 与 terminal stabilization 均未发生。

固定正式新结果位于：

```text
results/deck_motion_shadow_planar_marine_refinelm_local7m_20260815
```

没有覆盖 2026-08-13 历史结果，也没有换 seed、删轮次或放宽门限。仍固定运行
`static/rollpitch/combined/rigid_body_motion × seed 1/2/3`，结果为：

- Safety gates `12/12`。
- Planar Board gates `12/12`。
- 完整 shadow/Future Twist hard gates `0/12`，本阶段不要求通过且未调参。
- static seed1/2/3 current-normal `RMSE/P95` 从
  `1.186°/2.010°`、`1.421°/2.455°`、`1.284°/2.303°` 降为
  `0.151°/0.291°`、`0.134°/0.252°`、`0.145°/0.288°`。
- 9 个动态 episode current-normal RMSE 为 `0.148–0.249°`、P95 为
  `0.247–0.449°`。
- 全矩阵观察到 2/3/4 Marker；14 个 single-marker frame 全部路由到 `FAR_SINGLE`。

因此 Planar Board static 法向任务已经在固定 seed、固定 gate、固定安全边界下达到
`12/12`，下一任务切换为 Future Twist causal diagnosis。GUI 外观检查仍为
`GUI visual verification pending`，不影响本次 headless 算法验收。
