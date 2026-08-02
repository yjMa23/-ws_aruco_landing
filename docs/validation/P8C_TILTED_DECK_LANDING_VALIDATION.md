# P8C T1 固定 2° 倾斜甲板完整验收记录

## 1. 阶段结论

```text
P8C RESEARCH PASS
P8C PLAN PASS
P8C-0 IMPLEMENTATION PASS
P8C-1 VALIDATION PASS
P8C-2 SAFE DESCENT PASS
P8C-3 FAILURE EVIDENCE PRESERVED
P8C-4 VALIDATION PASS
P8C T1 VALIDATION PASS
P8C-3 DESIGN GATE CLOSED
```

P8C-1 已完成固定 `±2° roll / ±2° pitch` 甲板场景、启动安全门、甲板法向与 X500 四滑橇 shadow 几何、完整法向角误差门、跨 Bag 汇总、历史四尺度 Marker 重放、全工作区测试和真实 PX4 SITL 安全高度验收。P8C-2 在此基础上只对白名单 `tilt_roll_pos_2deg / tilt_pitch_pos_2deg` 开放相对下降到 `0.50 m`。P8C-3 的水平机体失败证据继续完整保留；P8C-4 随后完成终端主轴法向整形、在线滑橇近接触证据、状态化接触锚点、受限垂直预压、HOLD 法向锁存和姿态安全保护，并在最终代码下完成 roll/pitch 真实触地 6/6 与旧路径回归。

P8C-2 全程保持 UAV 水平，不开放 final descent、真实接触、姿态对齐、`NAV_LAND` 或自动 Disarm。P8C 几何与独立法向滤波继续只发布 `/landing/deck_plane/*` 诊断，不进入 landing window、状态机、下降、触地、hold 或 PX4 setpoint；Ground Truth 仅由离线 evaluator 读取。

最终结果：

```text
固定倾角安全高度：12/12 PASS
P8C-2 正倾角安全下降：6/6 PASS
P8C-4 shadow 安全高度：6/6 PASS
P8C-4 shadow 安全下降：6/6 PASS
P8C-4 主动无接触 rehearsal：6/6 PASS
P8C-4 固定 T1 真实触地：roll 3/3 + pitch 3/3 = 6/6 PASS
旧路径最终回归：9/9 PASS
全工作区：340 tests, 0 failures, 0 skipped
完整法向最差 RMSE：0.702°
完整法向最差 P95： 1.353°
最差符号正确率：   100%
Marker ID：        0/1/2/3 全覆盖
NAV_LAND：         0
Disarm：           0
```

## 2. 最终实现边界

### 2.1 固定倾角场景

新增独立配置：

```text
src/moving_deck_sim/config/tilt_roll_pos_2deg.yaml
src/moving_deck_sim/config/tilt_roll_neg_2deg.yaml
src/moving_deck_sim/config/tilt_pitch_pos_2deg.yaml
src/moving_deck_sim/config/tilt_pitch_neg_2deg.yaml
```

共同约束：

```text
位置固定
线速度 = 0
姿态固定
角速度 = 0
amplitude_rpy = 0
确定性 seed/reset
```

Gazebo 配置使用 world ENU RPY，P8C 诊断使用 PX4 local NED，因此固定单轴场景映射为：

```text
Gazebo roll  ±2° -> NED pitch ±2°
Gazebo pitch ±2° -> NED roll  ±2°
```

### 2.2 启动安全门

以下场景在启动任何 PX4/Gazebo/ROS 进程前拒绝下降开关：

```text
tilt_roll_pos_2deg
tilt_roll_neg_2deg
tilt_pitch_pos_2deg
tilt_pitch_neg_2deg
```

拒绝：

```text
--enable-relative-descent
--enable-final-descent
```

历史 `rollpitch` 和 `combined` 最终下降阻断保持不变。

### 2.3 production 与 shadow 法向隔离

生产 landing-window 姿态估计保持：

```yaml
deck_attitude.filter_gain: 0.20
```

P8C 几何诊断使用独立滤波器：

```yaml
deck_plane_geometry.normal_filter_gain: 0.08
```

`deck_plane_shadow_attitude_estimator_` 只更新：

```text
/landing/deck_plane/upward_normal_ned
/landing/deck_plane/body_clearance
/landing/deck_plane/skid_clearances
/landing/deck_plane/minimum_skid_clearance
/landing/deck_plane/maximum_skid_clearance
/landing/deck_plane/clearance_spread
/landing/deck_plane/first_contact_point_index
/landing/deck_plane/normal_relative_velocity
/landing/deck_plane/tangential_position_error
/landing/deck_plane/tangential_relative_velocity
Marker 法向与切换诊断
```

静态隔离测试证明 shadow 结果不进入：

```text
LandingWindowInput
RelativeDescentInput
FinalDescentInput
TouchdownDetectorInput
TouchdownHoldInput
TrajectorySetpoint
VehicleCommand
状态机转换
```

## 3. TDD 与问题闭环

### 3.1 场景、安全门和 evaluator

先保存真实失败证据，再实现：

```text
results/p8c1_validation_20260802/tdd_failure_scenarios.txt
results/p8c1_validation_20260802/tdd_failure_evaluator.txt
```

覆盖固定场景唯一映射、下降开关拒绝、有符号姿态、完整法向 RMSE/P95、Marker 分组、同步/NaN/内部一致性、状态安全和命令计数。

### 3.2 gain 0.12 真实失败

`tilt_pitch_neg_2deg seed2` 在 shadow gain `0.12` 下真实失败：

```text
完整法向 RMSE = 0.844°
完整法向 P95  = 1.629° > 1.500°
符号正确率    = 100%
同步/NaN/一致性错误 = 0
NAV_LAND / Disarm = 0 / 0
```

证据：

```text
results/p8c1_validation_20260802/tdd_failure_gain012_real_seed2.txt
results/p8c1_validation_20260802/gain012_tilt_pitch_neg_2deg_seed2/
```

原始 Marker 重放与全部已采集 Bag 扫描表明 gain `0.08` 的最差离线完整法向 P95 为 `1.254°`，因此只降低独立 shadow gain；production gain、landing window 和控制输出不变。

## 4. 构建与测试

最终执行：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash

export P8B_MPC_PREFIX="$HOME/.local/p8b-mpc/osqp-1.0.0-osqpeigen-0.11.2"
export CMAKE_PREFIX_PATH="$P8B_MPC_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$P8B_MPC_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

python3 -m unittest \
  src/aruco_precision_landing_cpp/test/test_p8c_tilted_deck.py
python3 -m py_compile scripts/evaluate_p8c_tilted_deck.py

colcon build --symlink-install \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
colcon test
colcon test-result --verbose
```

结果：

```text
P8C Python tests：27/27 PASS
3 packages finished
294 tests
0 errors
0 failures
0 skipped
```

同时通过：

```text
git diff --check
Ground Truth 生产订阅静态检查
shadow/control 隔离静态检查
固定倾角下降开关错误路径
完整法向角门与历史 Marker 重放测试
```

## 5. 真实 SITL 顺序与安全性

最终 gain `0.08` 代码重新从零采集：

```text
1. static 0° seed1
2. tilt_roll_pos_2deg seeds 1/2/3
3. tilt_roll_neg_2deg seeds 1/2/3
4. tilt_pitch_pos_2deg seeds 1/2/3
5. tilt_pitch_neg_2deg seeds 1/2/3
6. constant02 回归
7. heave_h1 回归
8. RELATIVE_MPC static 回归
```

每轮状态序列均止于：

```text
WAIT_FOR_PX4
-> OFFBOARD_PRE_STREAM
-> ARM_AND_TAKEOFF
-> WAIT_DECK_GNSS
-> RENDEZVOUS_GNSS
-> ACQUIRE_ARUCO
-> VISUAL_HANDOVER
-> TRACK_TARGET
-> WAIT_LANDING_WINDOW
```

全部轮次满足：

```text
禁止下降/触地状态 = 空
tracking 后 target z span = 0
tracking 后 trajectory z span = 0
同步失败 = 0
几何内部一致性失败 = 0
first-contact index mismatch = 0
tracking 后 NaN/Inf = 0
NAV_LAND = 0
Disarm = 0
```

## 6. 固定倾角法向结果

冻结门：

```text
平均有符号主轴误差绝对值 <= 0.5°
完整法向夹角 RMSE <= 1.0°
完整法向夹角 P95 <= 1.5°
符号正确率 >= 95%
Marker 切换跳变 <= 1.0°
```

| 场景 | seeds | 主轴估计均值 | 最差完整法向 RMSE | 最差完整法向 P95 | 最差符号率 | 与 static 最小均值间隔 |
|---|---:|---:|---:|---:|---:|---:|
| static | 1 | tilt `0.548°` | `0.629°` | `1.136°` | N/A | N/A |
| Gazebo roll +2° / NED pitch +2° | 3 | `+1.876°` | `0.614°` | `1.126°` | `100%` | `1.960°` |
| Gazebo roll -2° / NED pitch -2° | 3 | `-1.855°` | `0.667°` | `1.166°` | `100%` | `1.703°` |
| Gazebo pitch +2° / NED roll +2° | 3 | `+2.144°` | `0.563°` | `1.054°` | `100%` | `2.093°` |
| Gazebo pitch -2° / NED roll -2° | 3 | `-1.956°` | `0.702°` | `1.353°` | `100%` | `1.918°` |

0°/±2° 跨 Bag 可分辨性阈值在计划中没有独立冻结，因此 `1.703°` 作为观察项记录；四个方向的符号和冻结误差门均已通过。

## 7. 四尺度 Marker 一致性

5 m 固定倾角 Bag 按有状态 MarkerSelector 正常保持最大可靠 ID0。为避免为了覆盖 ID1–3 而在倾斜甲板上下降到接触附近，使用已通过真实 P6B 验收的水平下降/触地 Bag做只读重放：

```text
results/p6b_completion/p6b_freeze_static_seed301/bag
```

重放使用与最终 P8C shadow 相同的 gain `0.08`，Ground Truth 仅由离线 evaluator 读取。

| Marker | 样本数 | 平均 roll 偏差 | 平均 pitch 偏差 | 法向 RMSE | 法向 P95 |
|---|---:|---:|---:|---:|---:|
| ID0 | 118 | `+0.168°` | `-0.021°` | `0.308°` | `0.666°` |
| ID1 | 128 | `+0.079°` | `+0.070°` | `0.142°` | `0.253°` |
| ID2 | 279 | `+0.062°` | `+0.054°` | `0.233°` | `0.539°` |
| ID3 | 3671 | `+0.919°` | `-0.309°` | `0.975°` | `1.000°` |

真实切换序列：

```text
0->1
1->2
2->3
3->2
2->3
```

切换跳变：

```text
count = 5
mean  = 0.107°
RMSE  = 0.193°
P95   = 0.352°
max   = 0.426°
```

ID3 固定偏差接近 1°，虽满足本阶段门槛，仍是 P8C-2/P8C-3 的重点风险。该历史重放证明四尺度固定偏差和实际切换一致性，不替代 ID1–3 在真实倾斜图像下的后续验证。

## 8. shadow 几何结果

最终 static 对照的代表性结果：

```text
h_body mean = 3.2926 m
h_min mean  = 3.0635 m
h_max mean  = 3.0679 m
delta_h mean = 0.00441 m
delta_h P95  = 0.01329 m
```

Ground Truth 离线重算对比：

```text
h_body error mean = 0.0431 m
h_body error RMSE = 0.0497 m
h_body error P95  = 0.0755 m

clearance-spread error mean = 0.00220 m
clearance-spread error RMSE = 0.00292 m
clearance-spread error P95  = 0.00586 m
```

计划没有为 P8C-1 冻结 Ground Truth clearance 误差门，因此 evaluator 将这些值明确标为 observation-only，不把未冻结指标伪装为 PASS 条件。

## 9. 共享路径回归

最终 gain `0.08` 代码重新运行：

| 回归 | 结果 | 最终状态 | shadow 数据 | 同步/一致性失败 | NAV_LAND / Disarm |
|---|---|---|---|---:|---:|
| constant02 | PASS | `WAIT_LANDING_WINDOW` | valid | `0 / 0` | `0 / 0` |
| heave_h1 | PASS | `WAIT_LANDING_WINDOW` | valid | `0 / 0` | `0 / 0` |
| RELATIVE_MPC static | PASS | `WAIT_LANDING_WINDOW` | valid | `0 / 0` | `0 / 0` |

P4.7 默认跟踪、P8A 升沉路径和 P8B 显式 MPC 路径均未因 P8C shadow 接入发生状态或命令退化。

## 10. Ground Truth 隔离

生产控制与检测源代码不包含：

```text
/simulation/deck/ground_truth
```

Ground Truth 仅用于：

```text
scripts/evaluate_p8c_tilted_deck.py
历史 Marker Bag 离线重放
SITL 评测误差统计
```

控制器只使用视觉 Marker、PX4 UAV 状态、船舶 GNSS 传感器输出和内部估计。

## 11. 结果路径

最终 gain `0.08` Bag 与单轮 JSON：

```text
results/p8c1_validation_20260802/gain008_static_seed1/
results/p8c1_validation_20260802/gain008_tilt_roll_pos_2deg_seed{1,2,3}/
results/p8c1_validation_20260802/gain008_tilt_roll_neg_2deg_seed{1,2,3}/
results/p8c1_validation_20260802/gain008_tilt_pitch_pos_2deg_seed{1,2,3}/
results/p8c1_validation_20260802/gain008_tilt_pitch_neg_2deg_seed{1,2,3}/
```

共享路径回归：

```text
results/p8c1_validation_20260802/gain008_regression_constant02/
results/p8c1_validation_20260802/gain008_regression_heave_h1/
results/p8c1_validation_20260802/gain008_regression_relative_mpc/
```

综合结果：

```text
results/p8c1_validation_20260802/gain008_p8c1_final_summary.json
```

## 12. P8C-2 启动白名单与代码边界

`start_sitl.sh` 的最终门控如下：

```text
tilt_roll_pos_2deg / tilt_pitch_pos_2deg:
  safe altitude: allowed
  relative descent: only --descent-test-height 0.50
  final descent: rejected as P8C-3 not open

tilt_roll_neg_2deg / tilt_pitch_neg_2deg:
  safe altitude only
  relative descent: rejected
  final descent: rejected

rollpitch / combined:
  final descent: rejected
```

默认 YAML 继续保持：

```yaml
descent.enabled: false
final_descent.enabled: false
enable_auto_land: false
deck_plane_geometry.shadow_only: true
deck_plane_geometry.normal_filter_gain: 0.08
```

本阶段没有修改 P6B 分段最终下降、MarkerSelector、close-range 相机、P8A touchdown detector/hold、P8B MPC、P4.7 handoff、landing-window 阈值或任何生产姿态控制。

## 13. P8C-2 最低真实滑橇间隙门推导

X500 四等效接触点在 `base_link_frd` 中为：

```text
(-0.125, -0.132, 0.227)
( 0.125, -0.132, 0.227)
(-0.125,  0.132, 0.227)
( 0.125,  0.132, 0.227) m
```

在 `0.50 m` body/relative height 和固定 `2°` 倾角下，对每个端点使用：

```text
h_i = n_d * 0.50 - n · R_nb r_i
```

分别取 roll/pitch 方向最不利水平投影，得到：

```text
roll 理论最小间隙  = 0.268471 m
pitch 理论最小间隙 = 0.268227 m
最坏理论最小间隙  = 0.268227 m
```

P8C-1 已观察到的最坏滑橇几何绝对误差为 `0.166529 m`。在真实实验前冻结 Ground Truth 安全门为 `0.090 m`，因此：

```text
总保留余量                 = 0.268227 - 0.090000 = 0.178227 m
扣除 P8C-1 最坏误差后余量 = 0.178227 - 0.166529 = 0.011698 m
```

该门要求所有真实轮次最低滑橇间隙至少 `9 cm`，同时 evaluator 单独硬判 `contact_count=0` 和 `penetration_count=0`。阈值在首轮真实下降前写入纯函数测试和 evaluator，没有根据实验结果放宽。

## 14. P8C-2 测试与故障修复

TDD 红阶段证据：

```text
results/p8c2_validation_20260802/tdd_failure_start_gate.txt
```

现状脚本首先按预期拒绝正倾角 `0.50 m` relative descent；随后实现最小白名单并新增门控/纯函数测试。首轮自动 smoke 在启动任何 PX4/Gazebo 进程前暴露一个脚本问题：残留进程检测会把外层自动化命令行中出现的 `MicroXRCEAgent/PX4` 字样误认为真实残留。修复方式是只排除当前脚本及完整祖先进程链，独立的真实残留进程仍会阻断启动。修复后新增静态回归测试，并重新执行定向测试、完整构建和全工作区测试。

最终测试：

```text
P8C Python tests: 34 tests, OK
colcon build:      3 packages PASS
full workspace:   294 tests, 0 errors, 0 failures, 0 skipped
git diff --check: PASS
Ground Truth grep: production controller/detector 0 matches
```

构建记录：

```text
results/p8c2_validation_20260802/build_and_test.txt
```

## 15. P8C-2 真实 PX4 SITL 结果

所有轮次均使用：

```text
camera-model=close-range
tracking-mode=PREDICTED_POSITION_VELOCITY_FF
relative_descent=true
descent_test_height=0.50 m
final_descent=false
auto-confirm-controller=true
headless=true
确定性 seed
独立轻量 Bag
```

每轮状态路径均包含：

```text
WAIT_LANDING_WINDOW -> DESCEND -> TEST_HEIGHT_HOLD
```

| 场景 | seed | hold s | 水平 RMSE / max m | 最低 GT 滑橇间隙 m | 法向 RMSE / P95 ° | 符号率 | Marker 最大跳变 ° | 结果 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| tilt_roll_pos_2deg | 1 | 102.94 | 0.0145 / 0.0687 | 0.2121 | 0.145 / 0.212 | 100% | 0.083 | PASS |
| tilt_roll_pos_2deg | 2 | 47.11 | 0.0137 / 0.0531 | 0.2207 | 0.148 / 0.316 | 100% | 0.071 | PASS |
| tilt_roll_pos_2deg | 3 | 48.01 | 0.0158 / 0.0452 | 0.2469 | 0.185 / 0.351 | 100% | 0.427 | PASS |
| tilt_pitch_pos_2deg | 1 | 46.32 | 0.0163 / 0.0634 | 0.2134 | 0.269 / 0.481 | 99.92% | 0.253 | PASS |
| tilt_pitch_pos_2deg | 2 | 46.52 | 0.0160 / 0.0466 | 0.2240 | 0.244 / 0.545 | 100% | 0.220 | PASS |
| tilt_pitch_pos_2deg | 3 | 46.48 | 0.0159 / 0.0492 | 0.2182 | 0.318 / 0.562 | 99.92% | 0.377 | PASS |
| static | 1 | 46.57 | 0.0164 / 0.0628 | 0.2287 | 0.152 / 0.346 | 100% | 0.069 | PASS |
| constant02 | 1 | 44.95 | 0.0209 / 0.0586 | 0.2101 | 0.298 / 0.410 | 100% | 0.393 | PASS |

跨 8 轮最差值：

```text
最低 TEST_HEIGHT_HOLD 持续时间：44.947 s
水平 RMSE / 最大误差：          0.020931 / 0.068704 m
最低真实滑橇间隙：              0.210051 m  (> 0.090 m)
完整法向 RMSE / P95：           0.317644° / 0.562188°
最低符号正确率：                99.9168%
Marker 最大切换跳变：           0.427306°
切向位置 RMSE：                 0.036987 m（observation-only）
相对切向速度 RMSE：             0.019100 m/s（observation-only）
最大视觉不可见间隔：            0.068235 s（observation-only）
```

所有轮次统计：

```text
contact / penetration = 0 / 0
RECOVER_CLIMB / RECOVER_TO_GNSS = 0 / 0
FINAL_DESCENT = 0
TOUCHDOWN_CANDIDATE_HOLD / TOUCHDOWN_HOLD = 0 / 0
time sync failure / NaN-Inf = 0 / 0
NAV_LAND / Disarm = 0 / 0
每轮清理后残留实验进程 = 0
```

结果目录：

```text
results/p8c2_validation_20260802/
```

每轮目录包含 `bag/`、`evaluation.json`、`evaluation.txt` 和 `command.txt`。综合汇总为：

```text
results/p8c2_validation_20260802/p8c2_final_summary.json
results/p8c2_validation_20260802/p8c2_final_summary.txt
```

## 16. 当前风险与下一阶段边界

P8C-2 已验证固定正 `2°` 安全下降，但尚未验证：

1. 固定倾角单侧首接触、冲击、滑移和二次接触；
2. TOUCHDOWN_CANDIDATE_HOLD / TOUCHDOWN_HOLD 在倾斜碰撞平面上的行为；
3. UAV 姿态对齐策略；
4. 动态低频 roll/pitch 和 combined 场景。

下一阶段只能写成：

```text
P8C-3 固定 +2° 真实触地
```

当前仍保持：

```text
final_descent=false
无真实接触
无姿态 setpoint
NAV_LAND / Disarm = 0 / 0
负倾角 relative descent 关闭
历史 rollpitch/combined final descent 关闭
```

## 17. P8C-3 固定正 +2° 真实触地执行结果

P8C-3 已实现：

- `start_sitl.sh` 仅对白名单 `tilt_roll_pos_2deg / tilt_pitch_pos_2deg` 开放 final descent，并强制 relative descent 与严格 `0.50 m`；
- 负倾角、`rollpitch`、`combined` 继续在启动前拒绝；
- `evaluate_p8c_tilted_deck.py --p8c3-touchdown` 输出真实首接触、候选、确认、hold、四滑橇间隙、法向/切向速度、滑移、离板、二次接触、姿态、PX4 movement bits、Marker、恢复、NAV_LAND 和 Disarm；
- `run_single_experiment.py` 无人值守监控状态、要求前 10 秒完整 hold、额外录制 1 秒停机隔离余量、运行 evaluator 并清理进程；
- static/constant02 同时执行 P8C-3 和旧 P6B evaluator；
- 终端触地阶段后 recovery 立即判定失败，不允许二次降落覆盖首次失败。

最终代码验证：

```text
P8C/automation Python: 62/62 PASS
workspace: 294 tests, 0 errors, 0 failures, 0 skipped
Ground Truth production-source matches: 0
residual PX4/Gazebo/ROS/rosbag processes: 0
NAV_LAND / Disarm: 0 / 0
```

### 17.1 正 +2° roll

seed1 PASS：

- candidate/hold `0.500000/11.000593 s`；
- h_min/h_max/spread `0.000171/0.000613/0.000441 m`；
- 法向触地速度 `-0.004570 m/s`；
- 水平 RMSE/max `0.036962/0.082555 m`；
- 相对水平/切向速度 RMSE `0.017714/0.019239 m/s`；
- 滑移 `0.076707 m`，hold 切向速度 P95 `0.023738 m/s`；
- 最大 roll/pitch `2.135046°/0.175166°`；
- detach/secondary/recovery `0/0/0`。

seed2 独立重跑 FAIL：完整 hold `11.000469 s`，但滑移 `0.106767 m` 超过冻结 `0.10 m` 门。该轮无离板或恢复，说明失败不依赖灾难性倾覆才能出现。

seed2 已归档诊断轮还出现：

- hold 后约 `9.050088 s` 进入 recovery；
- 最大 roll/pitch `60.967996°/55.439155°`；
- 滑移 `0.676615 m`；
- detach/recovery `1/1`；
- PX4 rotational/vertical movement true count `5/11`；
- 姿态超过 `45°` 早于最长视觉丢失，视觉丢失不是初始原因。

按失败停止规则，roll seed3 和全部 pitch 轮次未运行，不能用 pitch 成功替代 roll 失败。

### 17.2 旧策略回归

```text
static: P8C-3 3/3, legacy P6B 3/3
constant02: P8C-3 3/3, legacy P6B 2/3
heave_h1: legacy P8A 1/1
```

constant02 seed3 的旧 P6B 物理接触最小间隙为 `-0.050144709 m`，比冻结 `-0.05 m` 下限多穿透 `0.000144709 m`。最深值发生在 hold 前 `8.329 s`，不是新增 1 秒停机余量造成，未放宽旧门。

### 17.3 失败与修复历史

- 先保存启动门和 evaluator TDD 失败证据；
- 修复 `pgrep` 无匹配在 `set -e -o pipefail` 下导致的空错误退出；
- 修复专用 PX4 消息容器被误判为“话题无消息”；
- 通过额外 1 秒录制和固定前 10 秒硬门窗口隔离异步停机尾帧，不改变接触迟滞或阈值；
- 未修改 TouchdownDetector、TouchdownHoldController、FinalDescentController、MarkerSelector、相机、landing window 或 MPC。

### 17.4 最终结论

```text
P8C-3 BLOCKED BY ATTITUDE-ALIGNMENT DECISION GATE
P8C T1 VALIDATION NOT PASSED
```

水平机体策略 A 不能在固定 T1 被保留。下一阶段必须先执行 `docs/plans/P8C3_ATTITUDE_ALIGNMENT_DECISION_GATE.md`，完成受限姿态对齐或接触/hold 重设计的独立方案选择、TDD 和分级回归。本任务未实现姿态对齐，也未开放负倾角或动态 roll/pitch。

证据目录：

```text
results/p8c3_validation_20260802/
results/p8c3_validation_20260802/p8c3_final_summary.json
results/p8c3_validation_20260802/p8c3_final_summary.txt
```

## 18. P8C-4 终端接触稳定化最终验收

### 18.1 最终实现边界

最终方案继续保持 `OffboardControlMode.position=true`、`attitude=false`，PX4 `mc_pos_control` 仍是唯一姿态 setpoint 生成者。新增控制只通过 `TrajectorySetpoint` 的受限 acceleration feedforward 与原 position target 配合实现：

- T1 `roll +2°` 只保留视觉法向的 roll 主轴，`pitch +2°` 只保留 pitch 主轴；
- 在线最小滑橇间隙 `<=0.03 m` 可在最低命令前形成 terminal geometry 接触证据；
- candidate/HOLD 以 `0.05 m/s` 将相对高度参考压到 `0.20 m`；
- candidate/HOLD 向下预压 acceleration 以 `1.0 m/s³` 渐入到 `1.0 m/s²`；
- HOLD 锁存 candidate 末端最后有效主轴法向，拒绝近距视觉幅值抖动；
- 接触锚点由甲板速度传播，位置只做 `0.05 m/s` 限速校正，静止速度死区为 `0.035 m/s`；
- 顺应目标保持锚点中心，切向速度阻尼为非积分偏移，目标速率限幅 `0.10 m/s`；
- 已确认 HOLD 的近距视觉长丢失保持最后安全接触目标，不重新起飞；姿态安全监视器仍可请求 recovery。

Ground Truth 没有进入生产节点、控制类或 setpoint 生成路径，只由 evaluator 读取。

### 18.2 分级结果

| 阶段 | 结果 |
|---|---:|
| Stage 0 历史 P8C-3 replay | PASS |
| Stage 1 shadow safe altitude，roll/pitch 3+3 | 6/6 PASS |
| Stage 2 shadow safe descent，roll/pitch 3+3 | 6/6 PASS |
| Stage 3 active rehearsal，roll/pitch 3+3 | 6/6 PASS |
| Stage 4～6 active touchdown，roll/pitch 3+3 | 6/6 PASS |
| Stage 7 static/constant02/H1/H2/RELATIVE_MPC | 9/9 PASS |
| 全工作区 | 340/340 PASS |

最终 active touchdown 轮次：

```text
active_touchdown_roll_seed1  PASS
active_touchdown_roll_seed2  PASS
active_touchdown_roll_seed3  PASS
active_touchdown_pitch_seed1 PASS
active_touchdown_pitch_seed2 PASS
active_touchdown_pitch_seed3 PASS
```

### 18.3 最差主轮指标

| 指标 | 最差值 | 冻结门 | 结果 |
|---|---:|---:|---:|
| 姿态跟踪误差 P95 | `0.238131°` | `<=1.5°` | PASS |
| 命令倾角最大值 | `2.190621°` | `<=2.5°` | PASS |
| 命令倾角 slew | `4.216282°/s` | `<=4.5°/s` | PASS |
| 合成水平 acceleration | `0.375126 m/s²` | `<=1.5 m/s²` | PASS |
| 水平 RMSE | `0.033018 m` | `<=0.08 m` | PASS |
| 水平最大误差 | `0.058881 m` | `<=0.15 m` | PASS |
| 接触后切向滑移 | `0.059209 m` | `<=0.10 m` | PASS |
| HOLD 切向速度 P95 | `0.032226 m/s` | `<=0.05 m/s` | PASS |
| 姿态发散增量 | `1.908267°` | `<=2.0°` | PASS |
| 最短 HOLD | `11.000416 s` | `>=10 s` | PASS |

六轮的 fallback、姿态保护触发、detach、secondary contact、recovery、NaN/Inf、`NAV_LAND` 和自动 Disarm 均为 0。

### 18.4 最终回归

```text
static:      3/3 PASS，P8C-3 + legacy P6B 双 evaluator
constant02:  3/3 PASS，P8C-3 + legacy P6B 双 evaluator
heave_h1:    1/1 PASS，P8A evaluator
heave_h2:    1/1 PASS，P8A evaluator
RELATIVE_MPC static safe descent: 1/1 PASS
```

所有旧路径回归中 `terminal_stabilization_applied_samples=0`，证明默认关闭与旧链路隔离成立。

### 18.5 最终结论

```text
P8C-4 VALIDATION PASS
P8C T1 VALIDATION PASS
P8C-3 DESIGN GATE CLOSED
```

P8C-3 失败 Bag、失败评测和独立设计门文档继续保留，作为为何引入 P8C-4 的历史证据；没有删除、覆盖或用最终成功轮替代失败证据。

最终证据：

```text
results/p8c4_validation_20260802/
results/p8c4_validation_20260802/p8c4_final_summary.json
results/p8c4_validation_20260802/p8c4_final_summary.txt
```
