# P9 统一批量评测第一版计划

## 0. 阶段状态与冻结前提

```text
前置阶段：P8C fixed T1 FROZEN
P8C-3 FAILURE EVIDENCE PRESERVED
P8C-4 VALIDATION PASS
P8C T1 VALIDATION PASS
P8C-3 DESIGN GATE CLOSED
P9 状态：PLAN PASS / IMPLEMENTATION PASS / TEST PASS / SMOKE COMPLETE / BASELINE IN PROGRESS / FORMAL ABLATION PENDING
```

P9 只评测已经通过相应安全验证的能力，不承担控制器调参、扩大触地白名单或开放新安全边界。P8C-4 的实际方案是 PX4 Offboard position 模式内的终端主轴法向整形、接触顺应、状态化锚点、切向阻尼、candidate/HOLD 法向锁存和受限预压，不是直接发送 PX4 attitude setpoint 的姿态对齐。

固定 T1 结论只能覆盖正 `+2° roll/pitch`；负倾角 touchdown、动态 roll/pitch、combined touchdown 和 dynamic attitude final descent 继续关闭。Ground Truth 只允许进入离线 evaluator，`NAV_LAND` 和自动 Disarm 保持关闭。

---

## 1. 论文实验问题

P9 第一版回答以下问题：

1. 完整 P4.7 规则式基线在 static 与 constant02 真实触地中的统计成功率和误差分布是什么？
2. 关闭额外位置预测或关闭甲板速度前馈后，安全高度水平跟踪性能如何变化？
3. 水平相对 MPC 与 P4.7 在匀速和正弦水平运动中的误差、求解时间和 fallback 表现如何？
4. 已验证的升沉处理与 MPC 组合后，H1 真实触地是否保持可重复安全？
5. fixed T1 当前终端接触稳定化方案在正 `+2° roll/pitch` 下的滑移、切向速度和姿态稳定性分布是什么？

P9 不回答负倾角、动态姿态或 combined touchdown 的问题；这些问题必须在未来独立阶段中重新研究和验收。

---

## 2. 方法定义与参数冻结

| 方法 | 定义 | 启动参数冻结 |
| --- | --- | --- |
| `B0` | 完整 P4.7 规则式跟踪基线 | `tracking=PREDICTED_POSITION_VELOCITY_FF`，prediction horizon `0.10 s`，velocity FF gain `1.0`，vertical FF 按已验收场景启用，terminal stabilization disabled |
| `B1` | P4.7，关闭额外位置预测 | 与 B0 相同，但 prediction horizon `0.00 s` |
| `B2` | P4.7，关闭甲板速度前馈 | 与 B0 相同，但 velocity FF gain `0.0` |
| `B3` | 水平相对 MPC | `tracking=RELATIVE_MPC`，prediction horizon `0.10 s`，velocity FF gain `1.0`，terminal stabilization disabled |
| `B4` | 水平相对 MPC + 已验证升沉处理 | 与 B3 相同，且仅在 H1 已验收范围内启用 vertical FF；终端仍按 P8B 设计 handoff 到 P4.7 |
| `B5` | fixed positive T1 终端接触稳定化 | P4.7 水平路径；按 profile 使用 `shadow/rehearsal/active` terminal stabilization；仅允许正 `+2° roll/pitch` |

方法通过参数组合实现，不复制控制器、不建立方法专用代码分支。每轮保存方法 ID、参数快照、精确启动命令、Git commit 和 dirty 状态。

---

## 3. 场景与 profile 适用矩阵

状态含义：

- `A`：APPLICABLE，可按该 profile 运行。
- `NA`：NOT_APPLICABLE，不运行，不计失败，不伪装为 PASS。
- touchdown 权限独立于 safe-altitude/safe-descent；低风险 profile 通过不自动授予 touchdown。

| 方法 | 场景 | safe-altitude | safe-descent | rehearsal | touchdown | 依据 |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| B0 | static | A | A | NA | A | P4/P5/P6/P7 |
| B0 | constant02 | A | A | NA | A | P4/P5/P6/P7 |
| B0 | constant | A | NA | NA | NA | P4 安全高度 |
| B0 | sinusoidal | A | NA | NA | NA | P4 安全高度 |
| B0 | heave_h1 | A | A | NA | A | P8A H1 |
| B0 | heave_h2 | A | A | NA | A | P8A H2 |
| B1 | static | A | NA | NA | NA | P4 安全高度消融 |
| B1 | constant02 | A | NA | NA | NA | P4 安全高度消融 |
| B2 | static | A | NA | NA | NA | P4 安全高度消融 |
| B2 | constant02 | A | NA | NA | NA | P4 安全高度消融 |
| B3 | static | A | NA | NA | NA | P8B 安全高度 |
| B3 | constant02 | A | A | NA | A | P8B 全链路 |
| B3 | constant | A | NA | NA | NA | P8B 安全高度 |
| B3 | sinusoidal | A | NA | NA | NA | P8B 安全高度 |
| B4 | heave_h1 | A | A | NA | A | P8B H1 全链路 |
| B5 | tilt_roll_pos_2deg | A | A | A | A | P8C-4 fixed T1 |
| B5 | tilt_pitch_pos_2deg | A | A | A | A | P8C-4 fixed T1 |

以下组合始终 `NOT_APPLICABLE`：负 `2°` touchdown、`rollpitch` touchdown、`combined` touchdown、dynamic attitude final descent、B5 非固定正倾角场景、B4 非 H1 场景，以及任何未列入白名单的 method/scenario/profile。

---

## 4. smoke 与正式实验次数

### 4.1 历史证据复用

- P7-lite B0 static/constant02 真实 3+3 smoke：历史 6/6 PASS，作为自动化前置证据保留。
- P8B 与 P8C-4 的历史真实验收作为安全授权依据，不替代 P9 新批次统计轮次。

### 4.2 P9 新组合 smoke

每个新 method/scenario/profile 组合先运行 3 个冻结 seed，3/3 全部通过后才进入该组合正式实验。控制能力失败时停止该组合正式实验，但其他组合继续；自动化或 evaluator 缺陷则保留失败证据、最小修复并仅重跑受影响轮次。

第一版 smoke 组合与冻结结果：

```text
B1 static safe-altitude                 3/3 PASS
B1 constant02 safe-altitude             3/3 PASS
B2 static safe-altitude                 3/3 PASS
B2 constant02 safe-altitude             0/3 PASS，3 SAFETY_GATE_FAILURE
B3 constant02 touchdown                 3/3 PASS
B3 sinusoidal safe-altitude             3/3 PASS
B4 heave_h1 touchdown                   2/3 PASS，1 SAFETY_GATE_FAILURE
B5 tilt_roll_pos_2deg touchdown         3/3 PASS
B5 tilt_pitch_pos_2deg touchdown        0/3 PASS，3 SAFETY_GATE_FAILURE
总计                                    27 executed，20 success，7 failure
```

smoke 结果目录为 `results/p9_smoke_20260803/`。B2 constant02 在关闭 velocity feedforward 后水平 RMSE 超过 `0.15 m` 冻结门；B4 heave_h1 有一轮水平最大误差约 `0.346 m`，水平跟踪和 HOLD 垂直语义门未同时通过；B5 pitch `+2°` 三轮 attitude tracking P95 均超过 `1.5°` 冻结门。三者属于方法能力失败，不调参、不换 seed、不重跑挑结果，并在正式消融中记为 `NOT_APPLICABLE`。

### 4.3 正式实验

1. P7 冻结基线：B0 static touchdown 20 次、B0 constant02 touchdown 20 次；严格使用原 `p7_baseline.yaml` seeds `1001..1020` 与 `2001..2020`。
2. 第一版消融每个组合 10 次，足以统一计算成功率、均值、总体标准差、中位数和经验 P95；论文最终版若需要置信区间，可在参数继续冻结后增加轮次。
3. smoke 后正式消融只执行通过安全门的组合：

```text
B0 constant02 safe-altitude              10
B1 constant02 safe-altitude              10
B3 constant02 safe-altitude              10
B0 sinusoidal safe-altitude              10
B3 sinusoidal safe-altitude              10
B5 tilt_roll_pos_2deg touchdown          10
总计                                      60 executable episodes
```

以下计划槽位保留在 `experiment_matrix.csv`，但必须标记为 `NOT_APPLICABLE`，不得启动、不得进入失败分母：

```text
B2 constant02 safe-altitude              10
B4 heave_h1 touchdown                    10
B5 tilt_pitch_pos_2deg touchdown         10
总计                                      30 gated NOT_APPLICABLE episodes
```

P9 第一版总实际计划为：smoke `27 executed` + baseline `40 planned` + formal ablation `60 executable` = `127 executable new episodes`。另有 `30` 个 smoke 后关闭的正式消融槽位仅记录适用性。历史失败 attempt 和 interrupted attempt 不计入计划轮次，但必须完整保留。

---

## 5. 统一评测指标

公共指标：

```text
success
failure_reason
landing_time_s
horizontal_error_rmse_m
horizontal_error_max_m
touchdown_vertical_speed_mps
candidate_to_confirm_delay_s
hold_duration_s
recovery_count
marker_switch_count
detach_count
secondary_contact_count
candidate_repeat_count
```

升沉附加：

```text
deck_vertical_span_final_m
hold_relative_height_span_m
hold_relative_vertical_velocity_p95_mps
```

fixed T1 附加：

```text
normal_tracking_error_rmse_deg
normal_tracking_error_p95_deg
terminal_command_tilt_max_deg
terminal_command_tilt_slew_p100_degps
combined_horizontal_acceleration_max_mps2
touchdown_slip_m
hold_tangential_velocity_p95_mps
attitude_divergence_increment_deg
fallback_count
terminal_stabilization_activation_count
```

MPC 附加：

```text
solver_success_rate
solver_failure_count
fallback_count
solve_time_mean_ms
solve_time_p95_ms
constraint_violation_count
```

读取不到的指标写 `null`；逻辑上不适用的字段在矩阵输出中写 `NOT_APPLICABLE`，不得写 0 冒充观测。

---

## 6. 失败分类

统一分类至少包含：startup/PX4 timeout/process exited/ArUco not acquired/vision lost/landing-window timeout/tracking diverged/recovery/触地未确认/PX4 abort/episode timeout/evaluator error/cleanup failure，以及 P9 新增的 safety-gate failure、method-not-applicable、configuration error。

`NOT_APPLICABLE` 是计划状态，不是实验失败。单轮失败不得终止整个 batch；失败轮目录、Bag、日志和 evaluator 输出不得被后续重试覆盖。

---

## 7. 数据、Bag 与日志保存

每个 episode 保存：

```text
manifest.json
command.txt
run_metadata.txt
method_parameters.yaml
controller_config.yaml
scenario_config.yaml
run.log
process_cleanup.txt
bag/
evaluation.json + evaluation.txt
legacy_evaluation.json + legacy_evaluation.txt（需要时）
```

成功轮保留轻量诊断 Bag；失败轮保留全部已录诊断话题、完整日志和配置快照。需要图像级诊断时使用显式 debug-image retry，并将旧轮归档为 failed/superseded attempt。批量 Bag 和结果不提交 Git；只提交代码、配置、计划、验收记录和摘要路径说明。

---

## 8. 参数冻结、resume 与重跑规则

- 正式 batch 启动后禁止修改控制参数、seed、profile、硬门或 evaluator 阈值。
- `results/p9_baseline_20x20_20260803/` 是基于 `b65713d` 启动后中断的 pre-freeze batch，仅完成 static seeds `1001..1004` 的 `4/40` 且均 PASS；该目录只作为历史证据保留，排除在最终 baseline 统计之外。解析器冻结提交后必须使用包含日期和短哈希的新 batch ID，从 static seed `1001` 重新完整运行 40 轮。
- resume 唯一键包含 batch ID、method、scenario、profile、repetition、seed。
- 完成且 Git commit/dirty fingerprint 一致的轮次可跳过。
- 同 ID 失败重试前归档旧目录为 `failed_attemptNN`。
- 代码变化后重跑前归档旧目录为 `superseded_attemptNN` 或 `interrupted_attemptNN`。
- 自动化/evaluator 修复只重跑受影响 episode；控制能力失败不得通过调参或挑 seed 修成 PASS。
- stale process 在每轮启动前检查，退出后验证清理完整。

---

## 9. 统计方法

- 成功率按 `success / executed` 计算；`NOT_APPLICABLE` 不进入分母。
- 数值统计只使用有限值；NaN/Inf 被记录为数据质量问题并排除数值统计，不转换为 0。
- 每个数值字段输出 `count/mean/stddev/median/p90/p95/min/max`。
- 分组至少包含 overall、by_scenario、by_method、by_method_and_scenario；另输出 failure breakdown。
- 统计使用总体标准差，保证与现有 P7 汇总兼容；论文中明确样本数。

---

## 10. 图表与论文表格输出

必须输出：

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
```

环境有 matplotlib 时生成：方法成功率、水平 RMSE、场景着陆时间、失败分布、MPC 与 P4.7 对比、T1 滑移与姿态稳定性 PNG。图表失败不得影响 JSON/CSV；代码错误必须显式报错并修复，缺少可选依赖则在 Markdown 中记录。

---

## 11. 完成条件

P9 第一版完成必须同时满足：

1. P8C commit/tag 已冻结且工作区证据一致。
2. P9 配置、方法矩阵、解析、runner、resume、聚合、Markdown/图表代码完成。
3. 旧 P7 YAML 继续兼容。
4. 新增相关测试通过，全工作区 build/test 为 0 error/0 failure/0 skipped。
5. 新组合 smoke 按门执行。
6. B0 static 20 与 constant02 20 完成。
7. 正式消融按冻结计划执行；被 smoke 控制能力失败阻断的组合必须准确记录，不伪造轮次。
8. 所有失败有分类和证据路径。
9. 聚合输出完整，文档同步。
10. Ground Truth 控制隔离、`NAV_LAND/Disarm=0/0`、负倾角和动态姿态 touchdown 继续关闭。
11. P9 commit 完成且工作区干净；只有上述条件全部满足才创建 `baseline-unified-evaluation-v0.1`。

---

## 12. 禁止扩大安全边界

P9 禁止：放宽滑移/姿态/离板/穿透/恢复硬门；修改 close-range Marker 参数；中途调控制器；删除失败轮；挑选有利 seed；使用 Ground Truth 控制；开放负倾角、动态 roll/pitch、combined touchdown；把 P8C-4 宣传为 PX4 attitude setpoint 姿态对齐。

---

## 13. 阶段结束后的建议

P9 完成后优先进行论文结果复核、统计置信区间和图表定稿。负倾角、动态姿态与 combined touchdown 只能作为独立的后续研究阶段，不能混入 P9 第一版。
