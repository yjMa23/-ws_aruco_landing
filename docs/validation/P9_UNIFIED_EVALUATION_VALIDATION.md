# P9 统一批量评测第一版验收记录

## 0. 当前状态

```text
P8C fixed T1: FROZEN
P9 PLAN: PASS
P9 IMPLEMENTATION: PASS
P9 TEST: PASS
P9 SMOKE: COMPLETE
P9 BASELINE 20+20: COMPLETE
P9 FORMAL ABLATION: COMPLETE
P9 AGGREGATION: PASS
P9 UNIFIED EVALUATION V0.1: PASS
```

本记录只接受仓库真实代码、无人值守 PX4 SITL、episode manifest、原始 Bag、离线 evaluator、聚合文件和全工作区测试作为证据。失败轮、历史中断批次和 P8C-3 失败证据均完整保留，没有通过调参、换 seed、重跑挑结果或覆盖目录获得更好统计。

---

## 1. 冻结提交与证据批次

正式仿真运行统一基于：

```text
71af1cc897136265a999c83dd6034bf156a32a50
修复仿真时钟源切换导致的控制节点退出
```

该提交只修复 `use_sim_time` 接管前后 SYSTEM_TIME/ROS_TIME 混用导致的控制节点退出，不修改控制器参数、MarkerSelector、close-range 相机、安全硬门、seed、触地白名单或控制律。

正式证据：

```text
smoke:    results/p9_smoke_20260803
baseline: results/p9_baseline_20x20_20260804_71af1cc
ablation: results/p9_ablation_20260804_71af1cc
```

离线聚合器修复提交：

```text
fc979fafc37f05fe4ae690e884153482a14d3c07
修复P9固定倾角嵌套指标聚合
```

该修复只读取 P8C evaluator 已存在的 `p8c3_touchdown_metrics` 与 `p8c4_terminal_stabilization_metrics` 嵌套标量，恢复 B5 的法向/姿态跟踪、滑移、切向速度、指令倾角和 fallback 统计；只重新离线聚合 smoke 与 ablation，没有重跑任何仿真。

P8C 冻结前提继续保持：

```text
P8C-4 VALIDATION PASS
P8C T1 VALIDATION PASS
P8C-3 DESIGN GATE CLOSED
```

P8C-4 是 PX4 Offboard position 模式内的终端主轴法向整形、接触顺应、状态化锚点、切向阻尼、candidate/HOLD 法向锁存与受限预压，不是 PX4 attitude setpoint 姿态控制。

---

## 2. 方法与安全矩阵

- B0：完整 P4.7。
- B1：P4.7，额外 prediction horizon 为 0。
- B2：P4.7，甲板 velocity feedforward gain 为 0。
- B3：水平 `RELATIVE_MPC`。
- B4：`RELATIVE_MPC` + 已验证 H1 升沉处理。
- B5：固定正 T1 终端接触稳定化。

smoke 关闭并在正式矩阵中标记为 `NOT_APPLICABLE` 的组合：

```text
B2 constant02 safe-altitude       10 slots
B4 heave_h1 touchdown             10 slots
B5 tilt_pitch_pos_2deg touchdown  10 slots
```

明确冻结结论：

```text
B2 constant02 blocked by smoke safety gate
B4 heave_h1 blocked by smoke safety gate
B5 pitch +2° blocked by smoke attitude tracking gate
B5 roll +2° is the only fixed-T1 formal touchdown combination
```

`NOT_APPLICABLE` 不启动仿真、不计失败、不伪装为 PASS。负倾角 touchdown、dynamic roll/pitch、rollpitch、combined 和 dynamic attitude final descent 继续关闭。

---

## 3. 自动化与时钟源缺陷处理

P9 自动化已实现：

- 配置与方法矩阵严格校验；
- method-aware episode ID、manifest、命令和参数快照；
- evaluator 自动路由；
- resume、失败归档和代码指纹；
- stale-process 检查与逐轮退出清理；
- overall/by_method/by_scenario/by_method_scenario 聚合；
- JSON、CSV、Markdown 和 PNG 输出；
- evaluator stdout 嵌套 JSON 解析；
- fixed-T1 嵌套论文指标聚合。

首次正式消融 `results/p9_ablation_20260804_a9d011d/` 在进入 `ACQUIRE_ARUCO` 后触发：

```text
can't subtract times with different time sources [1 != 2]
```

该批次在 `2/60` 后停止并完整保留，属于启动/控制运行语义缺陷，不计入方法统计。修复后使用相同组合和 seed 4101 做真实复现：

```text
results/p9_clock_source_fix_validation_20260804_71af1cc
ACQUIRE_ARUCO → VISUAL_HANDOVER → TRACK_TARGET → WAIT_LANDING_WINDOW
success = true
```

随后 baseline 与 ablation 均基于同一修复提交从第 1 轮重新执行。

---

## 4. smoke 结果

配置：`config/experiments/p9_smoke.yaml`

```text
batch ID: p9_smoke_20260803
executed: 27
success: 20
failure: 7
success rate: 74.07%
failure type: SAFETY_GATE_FAILURE = 7
corrupt_result_count: 0
NAV_LAND: 0
Automatic Disarm: 0
```

组合结果：

| 方法/场景/profile | 结果 |
| --- | ---: |
| B1 static safe-altitude | 3/3 |
| B1 constant02 safe-altitude | 3/3 |
| B2 static safe-altitude | 3/3 |
| B2 constant02 safe-altitude | 0/3，3 `SAFETY_GATE_FAILURE` |
| B3 constant02 touchdown | 3/3 |
| B3 sinusoidal safe-altitude | 3/3 |
| B4 heave_h1 touchdown | 2/3，1 `SAFETY_GATE_FAILURE` |
| B5 tilt_roll_pos_2deg touchdown | 3/3 |
| B5 tilt_pitch_pos_2deg touchdown | 0/3，3 `SAFETY_GATE_FAILURE` |

关闭原因：

- B2 constant02：关闭速度前馈后水平 RMSE 超过 `0.15 m` 冻结门。
- B4 heave_h1：一轮水平误差和 HOLD 垂直语义硬门未同时通过。
- B5 pitch `+2°`：三轮终端姿态跟踪 P95 超过 `1.5°` 冻结门。

失败证据目录：

```text
results/p9_smoke_20260803/p9_smoke_20260803_B2_constant02_safe_altitude_r001_s0000003401
results/p9_smoke_20260803/p9_smoke_20260803_B2_constant02_safe_altitude_r002_s0000003402
results/p9_smoke_20260803/p9_smoke_20260803_B2_constant02_safe_altitude_r003_s0000003403
results/p9_smoke_20260803/p9_smoke_20260803_B4_heave_h1_touchdown_r002_s0000003702
results/p9_smoke_20260803/p9_smoke_20260803_B5_tilt_pitch_pos_2deg_touchdown_r001_s0000003901
results/p9_smoke_20260803/p9_smoke_20260803_B5_tilt_pitch_pos_2deg_touchdown_r002_s0000003902
results/p9_smoke_20260803/p9_smoke_20260803_B5_tilt_pitch_pos_2deg_touchdown_r003_s0000003903
```

---

## 5. 正式 baseline 20+20

配置：`config/experiments/p9_baseline_20x20.yaml`

```text
batch ID: p9_baseline_20x20_20260804_71af1cc
planned/completed: 40/40
success/failure: 40/0
static: 20/20
constant02: 20/20
success rate: 100.00%
corrupt_result_count: 0
NAV_LAND: 0
Automatic Disarm: 0
```

关键指标：

| 指标 | overall mean | overall P95 | static mean | constant02 mean |
| --- | ---: | ---: | ---: | ---: |
| 水平 RMSE | `0.02853 m` | `0.03529 m` | `0.02852 m` | `0.02855 m` |
| 最大水平误差 | `0.05605 m` | `0.07229 m` | `0.05781 m` | `0.05430 m` |
| 着陆时间 | `18.2207 s` | `18.9062 s` | `18.2482 s` | `18.1932 s` |
| 触地垂直速度 | `0.00633 m/s` | `0.01459 m/s` | `0.00613 m/s` | `0.00652 m/s` |
| TOUCHDOWN_HOLD 时长 | `11.0040 s` | `11.0503 s` | `11.0028 s` | `11.0053 s` |
| HOLD 相对高度 span | `0.04243 m` | `0.06201 m` | `0.04388 m` | `0.04099 m` |

所有 40 轮 recovery count 均为 0。

---

## 6. 正式 ablation 60 轮

配置：`config/experiments/p9_ablation.yaml`

```text
batch ID: p9_ablation_20260804_71af1cc
planned/completed: 60/60
success/failure: 60/0
success rate: 100.00%
experiment_matrix rows: 9
NOT_APPLICABLE combinations: 3
NOT_APPLICABLE slots: 30
corrupt_result_count: 0
NAV_LAND: 0
Automatic Disarm: 0
```

执行组合：

| 方法/场景/profile | 结果 | 水平 RMSE mean | 水平 RMSE P95 |
| --- | ---: | ---: | ---: |
| B0 constant02 safe-altitude | 10/10 | `0.03243 m` | `0.04820 m` |
| B1 constant02 safe-altitude | 10/10 | `0.04178 m` | `0.05155 m` |
| B3 constant02 safe-altitude | 10/10 | `0.02907 m` | `0.03668 m` |
| B0 sinusoidal safe-altitude | 10/10 | `0.36425 m` | `0.37094 m` |
| B3 sinusoidal safe-altitude | 10/10 | `0.17704 m` | `0.18480 m` |
| B5 tilt_roll_pos_2deg touchdown | 10/10 | `0.02863 m` | `0.03954 m` |

MPC 指标：

| 组合 | solve time mean | episode-level solve-time P95 mean | fallback |
| --- | ---: | ---: | ---: |
| B3 constant02 | `0.18791 ms` | `0.24492 ms` | 0 |
| B3 sinusoidal | `0.18823 ms` | `0.23948 ms` | 0 |

B5 roll `+2°` fixed-T1 指标：

| 指标 | mean | empirical P95 | max |
| --- | ---: | ---: | ---: |
| 终端姿态跟踪 RMSE | `0.17489°` | `0.31257°` | `0.41677°` |
| 终端姿态跟踪 P95 | `0.21154°` | `0.70308°` | `1.11319°` |
| 触地后切向滑移 | `0.05289 m` | `0.06978 m` | `0.07505 m` |
| HOLD 切向速度 P95 | `0.02452 m/s` | `0.02838 m/s` | `0.02925 m/s` |
| 触地法向速度绝对值 | `0.01603 m/s` | `0.03750 m/s` | `0.03765 m/s` |
| 终端指令倾角最大值 | `2.09764°` | `2.22340°` | `2.22357°` |
| 指令倾角 slew 最大值 | `4.17850°/s` | `4.21309°/s` | `4.23445°/s` |
| 合成水平加速度最大值 | `0.35919 m/s²` | `0.38075 m/s²` | `0.38077 m/s²` |
| 姿态发散增量 | `0.19687°` | `1.07237°` | `1.86472°` |
| fallback count | `0` | `0` | `0` |

10 轮 B5 均无 detach、secondary contact 或 recovery。

---

## 7. 三批次最终汇总

P9 第一版真实可执行 episode：

```text
smoke 27 + baseline 40 + formal ablation 60 = 127 executed
success = 120
failure = 7
success rate = 94.49%
failure breakdown = SAFETY_GATE_FAILURE 7
```

### 7.1 by_method

| 方法 | success/executed | success rate |
| --- | ---: | ---: |
| B0 | 60/60 | 100.00% |
| B1 | 16/16 | 100.00% |
| B2 | 3/6 | 50.00% |
| B3 | 26/26 | 100.00% |
| B4 | 2/3 | 66.67% |
| B5 | 13/16 | 81.25% |

### 7.2 by_scenario

| 场景 | success/executed | success rate |
| --- | ---: | ---: |
| static | 26/26 | 100.00% |
| constant02 | 56/59 | 94.92% |
| sinusoidal | 23/23 | 100.00% |
| heave_h1 | 2/3 | 66.67% |
| tilt_roll_pos_2deg | 13/13 | 100.00% |
| tilt_pitch_pos_2deg | 0/3 | 0.00% |

### 7.3 by_method_scenario

| 方法/场景 | success/executed |
| --- | ---: |
| B0 static | 20/20 |
| B0 constant02 | 30/30 |
| B0 sinusoidal | 10/10 |
| B1 static | 3/3 |
| B1 constant02 | 13/13 |
| B2 static | 3/3 |
| B2 constant02 | 0/3 |
| B3 constant02 | 13/13 |
| B3 sinusoidal | 13/13 |
| B4 heave_h1 | 2/3 |
| B5 tilt_roll_pos_2deg | 13/13 |
| B5 tilt_pitch_pos_2deg | 0/3 |

正式 baseline 和正式 ablation 均为 100% 成功；总成功率低于 100% 完全来自 smoke 中被明确保留并关闭的三个不安全组合。

---

## 8. 历史与排除批次

以下目录完整保留，但不进入最终统计：

```text
results/p9_baseline_20x20_20260803
  interrupted pre-freeze batch, commit b65713d, 4/40 completed

results/p9_baseline_20x20_20260803_a9d011d
  orchestration-contaminated batch, 38 STARTUP_FAILURE, excluded

results/p9_baseline_20x20_20260803_a9d011d_clean1
  pre-clock-fix 40/40 batch, excluded to keep baseline/ablation run semantics identical

results/p9_ablation_20260804_a9d011d
  clock-source defect evidence, stopped at 2/60, excluded
```

这些目录没有删除、覆盖、resume 混入或用于最终成功率。

---

## 9. 聚合文件与图表

每个正式批次均包含：

```text
summary.json
summary.csv
episodes.csv
failures.csv
by_method.csv
by_scenario.csv
by_method_scenario.csv
experiment_matrix.csv
P9_RESULTS_SUMMARY.md
plots/*.png
```

smoke：

```text
results/p9_smoke_20260803/P9_RESULTS_SUMMARY.md
results/p9_smoke_20260803/summary.json
results/p9_smoke_20260803/plots/success_rate_by_method.png
results/p9_smoke_20260803/plots/failure_breakdown.png
results/p9_smoke_20260803/plots/horizontal_rmse_by_method.png
results/p9_smoke_20260803/plots/landing_time_by_scenario.png
results/p9_smoke_20260803/plots/mpc_vs_p47_horizontal_rmse.png
results/p9_smoke_20260803/plots/t1_slip_attitude.png
```

baseline：

```text
results/p9_baseline_20x20_20260804_71af1cc/P9_RESULTS_SUMMARY.md
results/p9_baseline_20x20_20260804_71af1cc/summary.json
results/p9_baseline_20x20_20260804_71af1cc/plots/success_rate_by_method.png
results/p9_baseline_20x20_20260804_71af1cc/plots/horizontal_rmse_by_method.png
results/p9_baseline_20x20_20260804_71af1cc/plots/landing_time_by_scenario.png
results/p9_baseline_20x20_20260804_71af1cc/plots/mpc_vs_p47_horizontal_rmse.png
```

ablation：

```text
results/p9_ablation_20260804_71af1cc/P9_RESULTS_SUMMARY.md
results/p9_ablation_20260804_71af1cc/summary.json
results/p9_ablation_20260804_71af1cc/plots/success_rate_by_method.png
results/p9_ablation_20260804_71af1cc/plots/horizontal_rmse_by_method.png
results/p9_ablation_20260804_71af1cc/plots/mpc_vs_p47_horizontal_rmse.png
results/p9_ablation_20260804_71af1cc/plots/t1_slip_attitude.png
```

本验收文档是三批次最终统一汇总；数值统计复用各批次 `summary.json`，不复制单批次聚合逻辑。

---

## 10. 安全隔离

```text
Ground Truth = evaluator only
NAV_LAND count = 0
Automatic Disarm count = 0
negative tilt touchdown = DISABLED
dynamic attitude touchdown = DISABLED
rollpitch touchdown = DISABLED
combined touchdown = DISABLED
```

Ground Truth 可出现在测试、脚本、仿真传感器与 evaluator 中，但生产控制节点不得订阅 `/simulation/deck/ground_truth`。

---

## 11. 测试与最终验证

已完成：

```text
时钟源修复包级测试: 344 tests, 0 errors, 0 failures, 0 skipped
P9 聚合针对性测试: 55 tests, PASS
git diff --check: PASS
```

文档修改完成后执行的最终全工作区验证结果：

```text
FINAL colcon build: 3 packages, PASS
FINAL colcon test-result --verbose: 344 tests, 0 errors, 0 failures, 0 skipped
FINAL git diff --check: PASS
FINAL residual process check: PASS，no PX4/Gazebo/Agent/rosbag/batch process
FINAL Ground Truth production subscription check: PASS，控制包内无该话题引用或订阅
```

最终安全检查同时确认：`enable_auto_land=false`，三批次 `missing_counter_episodes=0`，负倾角、dynamic attitude、rollpitch 和 combined touchdown 的拒绝测试均包含在全工作区测试中。

---

## 12. 最终结论

实验、聚合、证据保存、文档同步和最终全工作区验证均已通过。最终文档提交后，从该干净提交创建本地标签：

```text
P9 UNIFIED EVALUATION V0.1: PASS
baseline-unified-evaluation-v0.1
```

不主动 push commit 或 tag。
