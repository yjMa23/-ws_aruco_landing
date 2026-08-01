# P8A 升沉甲板最终下降与真实接触执行计划

## 1. 阶段状态

```text
阶段：P8A
当前状态：PLAN PASS
前置基线：P6B VALIDATION PASS，P7-lite VALIDATION PASS
后续门槛：P8A VALIDATION PASS 后才允许进入 P8B research
```

本计划只处理已有相对垂直速度语义和 `TOUCHDOWN_HOLD` 随动保持，不引入预测控制、扰动观测器、波浪预测器或新接触动力学模型，因此本阶段采用项目内技术分析，不启动复杂方法外部综述。

---

## 2. 当前仓库真实状态

### 2.1 已有控制链

当前主链为：

```text
GNSS 会合
→ GNSS—视觉接管
→ P4.7 水平跟踪
→ landing window
→ RelativeDescentController
→ FinalDescentController
→ TouchdownDetector
→ TOUCHDOWN_CANDIDATE_HOLD
→ TOUCHDOWN_HOLD
```

当前稳定基线必须保留：

- GNSS 会合和视觉接管；
- 四尺度有状态 MarkerSelector；
- 视觉状态估计和短时预测；
- P4.7 水平跟踪；
- landing window；
- P6B 终端落板门控；
- close-range `near=0.02 m` 相机；
- `NAV_LAND=0`、自动 Disarm=0；
- Ground Truth 仅用于 evaluator。

### 2.2 当前测试和实验

阶段开始前基线：

```text
3 ROS 2 packages
230 tests
0 errors
0 failures
0 skipped
```

真实实验已保存：

- `results/p6b_validation/static_evaluation.*`；
- `results/p6b_validation/constant02_evaluation.*`；
- `results/p7_smoke_terminal_20260730/`，6/6 PASS。

---

## 3. 垂直坐标与速度语义

### 3.1 坐标系

- Gazebo 世界系为 ENU，`z_enu` 向上为正；
- PX4 `local_ned` 的 `z_ned` 向下为正；
- `VehicleLocalPosition.vz` 为 NED 垂直速度，向下为正；
- `VerticalStateEstimator` 的状态为 `[deck_z_ned, deck_vz_ned]`，向下为正。

moving_deck_sim 的升沉轨迹为：

```text
z_deck_enu(t) = z0 + A sin(2πt/T)
v_deck_enu(t) = A(2π/T) cos(2πt/T)
```

转换到 NED 后：

```text
z_deck_ned = -z_deck_enu + origin_offset
v_deck_ned = -v_deck_enu
```

控制器不手写该转换，而使用视觉甲板 NED 估计。

### 3.2 相对高度

当前项目定义：

```text
h_rel = z_deck_ned - z_uav_ned
```

由于 NED Down 为正，甲板在无人机下方时 `h_rel > 0`。

### 3.3 相对垂直速度

对相对高度求导：

```text
v_rel_z = dh_rel/dt
        = v_deck_ned - v_uav_ned
```

含义：

- `v_rel_z > 0`：相对高度增加，甲板与无人机在垂直方向分离；
- `v_rel_z < 0`：相对高度减小，无人机相对甲板接近；
- `|v_rel_z|` 小：二者共同运动或相对静止。

当前节点已经按 `deck_vz_ned - uav_vz_ned` 计算并发布 `/landing/relative_vertical_velocity`，符号本身正确。

---

## 4. 当前缺陷分析

### 4.1 TouchdownDetector 仍混入世界系静止语义

当前普通触地路径要求：

```text
low_relative_speed
AND low_uav_speed
AND movement_compatible
```

当前终端接触停滞路径还要求：

```text
|v_rel_z| <= terminal_limit
AND |v_uav_z| <= terminal_limit
AND vertical_movement == false
```

这会导致：

1. 甲板和无人机共同以非零世界系垂直速度运动、但相对速度很低时，候选被 `low_uav_speed` 或 PX4 `vertical_movement` 永久阻断；
2. P8A H2 的理论峰值世界速度约为：

```text
2πA/T = 2π×0.20/8 ≈ 0.157 m/s
```

它超过当前 `max_uav_vertical_speed_mps=0.15 m/s`，即使真实相对速度接近零也可能不能确认；
3. terminal stall 的 `0.05 m/s` UAV 世界速度限制甚至会阻断 H1 峰值 `≈0.063 m/s`。

因此 P8A 必须把触地速度门槛统一为相对垂直速度语义。UAV 世界系垂直速度可以继续保留为诊断证据，但不能在甲板垂直速度估计有效时作为升沉触地硬门槛。

### 4.2 甲板垂直估计失效必须阻断

当前相对垂直速度只有在以下条件成立时才计算：

- `VerticalStateEstimator` 启用；
- 垂直视觉测量有效；
- UAV `vz` 有效；
- 甲板垂直速度估计有限。

估计失效时传入 NaN，检测器返回 `INSUFFICIENT_EVIDENCE`，不会绕过速度判据。P8A 将进一步增加显式 `relative_vertical_speed_valid` 输入，使有效性语义不依赖 NaN 隐式表达。

### 4.3 TOUCHDOWN_HOLD 冻结世界系 z

当前进入 `TOUCHDOWN_HOLD` 时执行：

```text
touchdown_hold_target_z = local_position.z
```

之后目标恒定。升沉甲板继续运动时：

- 甲板向下：固定世界目标会导致起落架相对甲板离开；
- 甲板向上：固定世界目标会导致持续压板；
- P6B evaluator 还要求 hold 后 target-z span 小于 `0.05 m`，该标准只适用于静止/纯水平甲板，不适用于 P8A。

P8A 需要保持触地时的甲板相对垂直关系，而不是冻结世界系 z。

---

## 5. 推荐最小实现

### 5.1 TouchdownDetector 语义修正

新增显式输入：

```cpp
bool relative_vertical_speed_valid;
```

定义：

```text
low_relative_vertical_speed = valid AND |v_rel_z| <= threshold
vertical_motion_compatible = !px4.vertical_movement OR low_relative_vertical_speed
```

普通和终端触地路径：

- 必须有有效相对垂直速度；
- 使用相对垂直速度限制；
- PX4 报告 `vertical_movement=true` 时，只要相对垂直速度低，仍允许候选；
- UAV 世界系垂直速度仅保留为诊断 evidence，不作为普通/终端确认硬门槛；
- `freefall`、旋转运动、水平相对速度、视觉高度、最低终端命令、参考压差和持续时间等原安全证据保持不变。

不得在 TouchdownDetector 内硬编码 `heave`、H1/H2/H3 或场景名称。

### 5.2 TouchdownHoldController

新增纯 C++ 状态类：

```text
TouchdownHoldController
```

输入：

- 当前时间步 `dt_s`；
- UAV 当前 `z_uav_ned`；
- 甲板估计 `z_deck_ned`；
- 甲板估计 `v_deck_ned`；
- 甲板估计是否有效。

初始化：

```text
h_hold = z_deck_ned_at_confirmation - z_uav_ned_at_confirmation
z_target = z_uav_ned_at_confirmation
```

正常更新：

```text
z_desired = z_deck_ned - h_hold
z_target(k) = rate_limit(z_target(k-1), z_desired, max_target_rate × dt)
```

输出：

- `relative_height_reference_m = h_hold`；
- `vertical_target_z_ned_m = z_target`；
- `deck_vertical_velocity_ned_mps`；
- `mode = RELATIVE_DECK_HOLD`；
- `reason = TRACKING_DECK`。

估计失效：

- 保持最后一个有效世界系 z 目标；
- 清除垂直速度前馈；
- `mode = HOLD_LAST_TARGET`；
- `reason = DECK_ESTIMATE_INVALID`；
- 不退出已锁存的 touchdown confirmed；
- 不继续下降，不产生目标突跳。

第一版不引入接触动力学、力控制或预测器。

### 5.3 垂直速度前馈

在 `TOUCHDOWN_HOLD` 且甲板估计有效时：

```text
v_ff_z = clamp(gain × v_deck_ned, ±vertical_ff_max)
```

相对高度参考速度为 0，不叠加继续下降速度。估计失效时清除前馈。

### 5.4 诊断接口

复用已有：

- `/landing/vertical_state`：甲板 z 与甲板 NED 垂直速度；
- `/landing/relative_vertical_velocity`；
- `/landing/target_pose`；
- `/landing/relative_height`；
- `/landing/relative_height_reference`。

新增：

- `/landing/uav_vertical_velocity`，`std_msgs/Float64`，NED Down 为正；
- `/landing/touchdown_hold_relative_height_reference`，`std_msgs/Float64`；
- `/landing/touchdown_hold_vertical_target`，`std_msgs/Float64`；
- `/landing/touchdown_hold_mode`，`std_msgs/String`；
- `/landing/touchdown_hold_reason`，`std_msgs/String`。

Bag 录制脚本同步加入这些轻量话题。

---

## 6. 升沉场景分级

采用“独立配置文件”作为唯一入口，不同时增加重复的 amplitude/period CLI 参数。

新增：

```text
src/moving_deck_sim/config/heave_h1.yaml
src/moving_deck_sim/config/heave_h2.yaml
src/moving_deck_sim/config/heave_h3.yaml
```

配置：

| profile | amplitude_z_m | period_z_s | 理论峰值速度 |
| --- | ---: | ---: | ---: |
| H1 | 0.10 m | 10.0 s | 0.0628 m/s |
| H2 | 0.20 m | 8.0 s | 0.1571 m/s |
| H3 | 0.30 m | 8.0 s | 0.2356 m/s |

保留历史 `heave.yaml` 不删除、不覆盖。`start_sitl.sh` 增加 `heave_h1/heave_h2/heave_h3` 场景映射；第一次只允许 H1 使用 `--enable-final-descent`。H2/H3 必须在上一档真实验收通过后再逐级开放，不能通过场景名自动放宽控制阈值。

rollpitch 和 combined 继续阻断最终下降。

---

## 7. P8A evaluator

新增：

```text
scripts/evaluate_p8a_heave_touchdown.py
```

复用 `evaluate_p6b_touchdown.py` 的 Bag 读取、坐标转换和通用触地统计函数，不复制整套 rosbag 解析逻辑。必要时先把 P6B evaluator 中可复用的纯函数提取到现有脚本模块。

至少统计：

- 最终阶段甲板 z 位移与跨度；
- 接触时甲板 NED 垂直速度；
- 接触时 UAV NED 垂直速度；
- 接触时相对垂直速度；
- 起落架接触间隙最小/最大值；
- hold 相对高度跨度；
- hold 相对垂直速度 P95；
- 离板次数；
- 二次接触次数；
- touchdown candidate 反复次数；
- recovery 次数；
- `NAV_LAND`；
- Disarm。

Ground Truth 只用于 evaluator 的甲板真实位置、接触间隙、离板与二次接触统计。

P8A 的 hold 判据必须检查“相对高度稳定”，不能继续沿用 P6B 的“世界系 target-z 几乎不动”。

---

## 8. 测试优先顺序

### 8.1 TouchdownDetector

新增或修改测试：

1. 静止甲板低世界速度、低相对速度可候选；
2. 升沉甲板 UAV/甲板共同非零世界速度、低相对速度可候选；
3. UAV 世界速度低但相对速度高不可候选；
4. 相对垂直速度无效时不可误确认；
5. 最低终端命令未完成时不可因停滞误判；
6. 候选后参考冻结；
7. 相对速度超限中断候选；
8. 确认锁存；
9. static 原有语义不变；
10. constant02 原有水平相对速度语义不变。

### 8.2 TouchdownHoldController

新增测试：

1. 初始化无目标跳变；
2. 甲板 z 向下变化时目标跟随；
3. 甲板 z 向上变化时目标跟随；
4. 保持触地时相对高度参考；
5. 目标变化率限制；
6. 估计失效保持最后目标；
7. 恢复估计时不产生突跳；
8. 无效 `dt`、NaN 输入被拒绝；
9. reset 清除状态；
10. 参数非法时报错。

### 8.3 脚本与 evaluator

覆盖：

- H1/H2/H3 配置值；
- `start_sitl.sh` H1 开放、H2/H3 和 rollpitch/combined 阻断规则；
- P8A evaluator 纯函数：离板、二次接触、P95、候选反复统计；
- `--help` 和缺少 topic 的失败路径。

---

## 9. 修改文件范围

计划新增：

```text
docs/P8A_HEAVE_TOUCHDOWN_PLAN.md
src/aruco_precision_landing_cpp/include/aruco_precision_landing_cpp/touchdown_hold_controller.hpp
src/aruco_precision_landing_cpp/src/touchdown_hold_controller.cpp
src/aruco_precision_landing_cpp/test/touchdown_hold_controller_test.cpp
src/moving_deck_sim/config/heave_h1.yaml
src/moving_deck_sim/config/heave_h2.yaml
src/moving_deck_sim/config/heave_h3.yaml
scripts/evaluate_p8a_heave_touchdown.py
```

计划修改：

```text
src/aruco_precision_landing_cpp/include/aruco_precision_landing_cpp/touchdown_detector.hpp
src/aruco_precision_landing_cpp/src/touchdown_detector.cpp
src/aruco_precision_landing_cpp/test/touchdown_detector_test.cpp
src/aruco_precision_landing_cpp/include/aruco_precision_landing_cpp/px4_aruco_landing_node.hpp
src/aruco_precision_landing_cpp/src/px4_aruco_landing_node.cpp
src/aruco_precision_landing_cpp/CMakeLists.txt
src/aruco_precision_landing_cpp/config/landing_params.yaml
scripts/start_sitl.sh
scripts/evaluate_p6b_touchdown.py（仅提取复用接口或兼容新 hold 语义）
README.md
AGENTS.md
docs/NEXT_DEVELOPMENT_PLAN.md
docs/TRADITIONAL_BASELINE_PLAN.md
docs/P8_ADVANCED_LANDING_ROADMAP.md
```

不得修改与 P8A 无关的 P4.7 水平控制算法和参数。

---

## 10. 构建、回归和真实验收

每次生产代码修改后：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash

colcon build --symlink-install \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

source install/setup.bash
colcon test
colcon test-result --verbose

git diff --check
bash -n scripts/start_sitl.sh
python3 -m py_compile scripts/*.py
python3 scripts/evaluate_p8a_heave_touchdown.py --help
```

低风险回归：

1. static 单轮；
2. constant02 单轮；
3. 对应 P6B evaluator PASS；
4. 无新增 recovery；
5. `NAV_LAND=0`、Disarm=0；
6. 水平误差无明显退化。

真实 SITL 顺序：

```text
H1 单轮
→ H1 seeds 101/102/103
→ H2 单轮
→ H2 seeds 201/202/203
→ H3（仅当 H2 无需放宽安全阈值即可通过）
```

P8A PASS 至少要求：

- H1 3/3 PASS；
- H2 至少单轮 PASS；
- 进入 `TOUCHDOWN_HOLD` 并保持不少于 10 秒；
- 接触时相对垂直速度满足既有安全阈值；
- hold 中无持续离板；
- 无明显二次撞击；
- 无 `NAV_LAND`；
- 无 Disarm；
- static/constant02 回归通过。

若环境无法自动完成真实 SITL，保存代码、测试、dry-run 和启动验证结果，标记 `P8A VALIDATION BLOCKED`，输出准确命令并停止，不进入 P8B。

---

## 11. 回退策略

- 新相对触地语义导致 static/constant02 回退：恢复到最近通过的检测逻辑，保留失败测试和 Bag，重新分析，不放宽阈值；
- hold 跟随目标异常：停用相对 hold，保持最后安全目标并标记 validation blocked；
- 甲板估计频繁失效：先分类视觉、时间戳和滤波问题，不延长全局视觉超时；
- H2/H3 失败：保持上一档通过配置，不为单次失败连续调参；
- 任意真实可重复失败：保存 Bag、evaluator 输出和日志，只做直接相关的最小修复。
