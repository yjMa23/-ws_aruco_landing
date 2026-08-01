# P6A 多源触地候选与确认验收

## 1. 验收结论

P6A 已完成并通过当前阶段验收。

本阶段完成：

- 新增与 ROS、PX4 和仿真器解耦的纯 C++ `TouchdownDetector`；
- 联合 PX4 `VehicleLandDetected`、视觉相对高度、相对垂直速度和无人机垂直速度进行判定；
- 实现候选持续时间、视觉高度迟滞、时间异常拒绝和确认锁存；
- 将触地检测器并行接入 ROS 节点，不改变状态机和 TrajectorySetpoint；
- 新增触地状态、证据、候选持续时间和确认调试话题；
- 新增 P6A rosbag 离线评测脚本；
- 完成静止 `0.50 m`、升沉 `0.70 m` 和恢复爬升三类 PX4 SITL 负向验收。

当前没有执行 `0.50 m` 以下最终下降，没有真实接触正向试验，没有发送 `NAV_LAND`，没有自动 Disarm。

---

## 2. 纯 C++ 触地检测器

新增：

```text
src/aruco_precision_landing_cpp/include/aruco_precision_landing_cpp/touchdown_detector.hpp
src/aruco_precision_landing_cpp/src/touchdown_detector.cpp
src/aruco_precision_landing_cpp/test/touchdown_detector_test.cpp
```

### 2.1 输出状态

```text
INSUFFICIENT_EVIDENCE
AIRBORNE
CANDIDATE
CONFIRMED
REJECTED_UNSAFE
```

### 2.2 证据位

```text
GROUND_CONTACT
MAYBE_LANDED
LANDED
AT_REST
LOW_THROTTLE
CLOSE_TO_GROUND
VISUAL_LOW_HEIGHT
LOW_RELATIVE_VERTICAL_SPEED
LOW_UAV_VERTICAL_SPEED
NO_REPORTED_MOVEMENT
```

### 2.3 确认路径

强触地证据：

```text
landed
&& at_rest
&& !vertical_movement
&& !horizontal_movement
&& !rotational_movement
```

普通触地证据：

```text
(ground_contact || maybe_landed || landed)
&& fresh_visual_low_height
&& low_relative_vertical_speed
&& low_uav_vertical_speed
&& no_reported_movement
```

视觉相对高度只能作为普通路径的辅助证据，不能单独确认触地。

### 2.4 默认参数

```yaml
touchdown_detector.enabled: true
touchdown_detector.px4_status_timeout_s: 0.20
touchdown_detector.visual_timeout_s: 0.20
touchdown_detector.low_height_enter_m: 0.18
touchdown_detector.low_height_exit_m: 0.28
touchdown_detector.max_relative_vertical_speed_mps: 0.12
touchdown_detector.max_uav_vertical_speed_mps: 0.15
touchdown_detector.candidate_required_duration_s: 0.50
```

P6A 只允许在 `TEST_HEIGHT_HOLD` 状态积累候选时间。高空跟踪、下降过程和恢复爬升都会主动保持 `AIRBORNE` 并清零候选时间。

---

## 3. ROS 并行接入

新增订阅：

```text
/fmu/out/vehicle_land_detected
```

消息：

```text
px4_msgs/msg/VehicleLandDetected
```

新增调试话题：

```text
/landing/touchdown_status
/landing/touchdown_evidence
/landing/touchdown_candidate_duration
/landing/touchdown_confirmed
```

检测器结果当前不会：

- 改变状态机；
- 改变位置或速度设定点；
- 触发最终下降；
- 触发 `NAV_LAND`；
- 触发 Disarm；
- 关闭 Offboard。

Ground Truth 从未进入触地检测器，只进入离线评测。

---

## 4. 自动评测

新增：

```text
scripts/evaluate_p6a_touchdown.py
```

运行：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash

python3 scripts/evaluate_p6a_touchdown.py <bag_directory>
```

输出：

- 状态序列；
- 触地状态计数和按飞行状态分类计数；
- 证据位计数；
- 候选样本数和最大连续持续时间；
- 确认样本数；
- 允许状态之外的候选和确认次数；
- PX4 land detector 各标志为真的样本数；
- `NAV_LAND` 和 Disarm 命令数；
- 负向触地验收 PASS/FAIL。

---

## 5. 单元测试

测试覆盖：

1. 视觉低高度单独满足不能确认；
2. `landed && at_rest` 连续满足后可确认；
3. 接触、低高度和低速度联合满足后可确认；
4. 单帧接触抖动不能确认；
5. 高垂直速度清零候选；
6. 水平或旋转运动拒绝普通触地路径；
7. 即使 `landed && at_rest`，仍报告运动时也不能确认；
8. `freefall` 进入不安全拒绝；
9. PX4 land 状态超时不能确认；
10. 视觉超时且无强证据时不能确认；
11. 视觉低高度迟滞；
12. 非允许状态不能积累候选；
13. 重复和倒退时间拒绝并清零候选；
14. 确认后状态锁存；
15. reset 清除确认和候选历史；
16. 非法输入与非法参数拒绝。

当前工作区：

```text
3 packages finished
182 tests
0 errors
0 failures
0 skipped
```

---

## 6. 静止甲板 0.50 m 负向验收

Bag：

```text
bags/p5b_static_descent_zff1p0_20260725_143828
```

状态：

```text
WAIT_LANDING_WINDOW
→ DESCEND
→ TEST_HEIGHT_HOLD
```

P6A 结果：

```text
AIRBORNE samples: 917
INSUFFICIENT_EVIDENCE samples: 1018
CANDIDATE samples: 0
最大候选持续时间: 0.000 s
CONFIRMED samples: 0
允许状态之外候选/确认: 0 / 0
NAV_LAND / Disarm: 0 / 0
P6A negative test: PASS
```

PX4 在任务初始阶段曾短暂报告：

```text
ground_contact: 7 samples
maybe_landed: 6 samples
landed: 6 samples
at_rest: 6 samples
```

这些标志出现在起飞前或非允许状态，状态门控使候选持续时间始终为零，证明 PX4 初始落地状态不会造成误确认。

P5B 安全指标：

```text
Ground Truth 最低相对高度: 0.4763 m
实际参考跟踪 RMSE: 0.0200 m
z 目标最大单步变化: 0.0197 m
窗口关闭时参考继续下降: 0
```

---

## 7. 升沉甲板 0.70 m 负向验收

Bag：

```text
bags/p5b_heave_descent_zff1p0_predff_h0p10_vff1p0_rvg0p25_adapt_g0p25-1p2_a0p05-0p35_f0p20_hmin0p70_20260725_144238
```

P6A 结果：

```text
AIRBORNE samples: 893
INSUFFICIENT_EVIDENCE samples: 1142
CANDIDATE samples: 0
最大候选持续时间: 0.000 s
CONFIRMED samples: 0
允许状态之外候选/确认: 0 / 0
NAV_LAND / Disarm: 0 / 0
P6A negative test: PASS
```

P5B/P5C 指标：

```text
Ground Truth 最低相对高度: 0.5686 m
实际参考跟踪 RMSE: 0.0881 m
0.5～1.0 m 相对高度 RMSE: 0.0315 m
z 目标最大单步变化: 0.0274 m
```

动态升沉、垂直速度变化以及 PX4 `vertical_movement` 标志没有造成候选或确认。

---

## 8. 恢复爬升负向验收

Bag：

```text
bags/p5b_static_descent_zff1p0_predff_h0p10_vff1p0_rvg0p25_adapt_g0p25-1p2_a0p05-0p35_f0p20_winmin0p60_20260725_144645
```

状态：

```text
WAIT_LANDING_WINDOW
→ DESCEND
→ RECOVER_CLIMB
→ WAIT_LANDING_WINDOW
```

P6A 结果：

```text
AIRBORNE samples: 1715
REJECTED_UNSAFE samples: 18
CANDIDATE samples: 0
最大候选持续时间: 0.000 s
CONFIRMED samples: 0
RECOVER_CLIMB 中候选/确认: 0 / 0
NAV_LAND / Disarm: 0 / 0
P6A negative test: PASS
```

早期 `REJECTED_UNSAFE` 样本发生在仿真时钟和 PX4 状态尚未稳定时；检测器没有积累候选时间。恢复阶段和恢复后重新授权锁止阶段均保持候选时间为零。

P5B 指标：

```text
Ground Truth 最低相对高度: 0.5688 m
实际参考跟踪 RMSE: 0.0246 m
z 目标最大单步变化: 0.0206 m
窗口关闭时参考继续下降: 0
```

---

## 9. 日志检查

三轮 P6A SITL 日志均未发现：

```text
timestamp synchronization failed
pose history insertion failed
camera-to-NED transform failed
invalid visual state
ABORT
NAV_LAND
disarm
failsafe
```

定时停止脚本产生的退出码 `124` 来自外部 `timeout`，不是控制器异常。

---

## 10. P6A 验收判定

| 验收项 | 结果 | 判定 |
| --- | ---: | --- |
| 视觉高度单独导致确认 | `0` | 通过 |
| 静止 0.50 m 确认次数 | `0` | 通过 |
| 升沉 0.70 m 确认次数 | `0` | 通过 |
| 恢复爬升确认次数 | `0` | 通过 |
| 允许状态之外候选/确认 | `0 / 0` | 通过 |
| 三场景最大候选持续时间 | `0.000 s` | 通过 |
| NAV_LAND / Disarm | `0 / 0` | 通过 |
| P5B/P5C 控制回归 | 无明显回归 | 通过 |
| 全工作区测试 | `182/182` | 通过 |

P6A 可以标记为完成。

---

## 11. 下一阶段边界

下一阶段为 P6B：最终下降和真实接触正向验证。

在开始任何 P6B 代码前，必须先保存独立执行计划。P6B 第一版仍应满足：

1. 只先在静止甲板 SITL 中运行；
2. 最终下降默认关闭，必须显式授权；
3. 保持 P4.7 水平跟踪和 P5C 垂直速度前馈；
4. 触地确认后先保持，不自动 Disarm；
5. Ground Truth 只用于离线评测；
6. 任何视觉、PX4 land 状态或时间异常立即暂停或恢复；
7. 在真实接触正向验收完成前，不启用自动 Land/Disarm。
