# P9 统一批量评测第一版验收记录

## 0. 当前状态

```text
P8C fixed T1: FROZEN
P9 PLAN: PASS
P9 IMPLEMENTATION: PASS
P9 TEST: PASS
P9 SMOKE: COMPLETE
P9 BASELINE 20+20: IN PROGRESS
P9 FORMAL ABLATION: PENDING
P9 AGGREGATION: PENDING
P9 UNIFIED EVALUATION V0.1: PENDING
```

本记录只接受仓库真实代码、全工作区测试、无人值守 PX4 SITL、episode manifest、离线 evaluator 与聚合文件作为证据。失败轮和历史 P8C-3 证据不得删除或覆盖。

---

## 1. 冻结基线

- P8C commit：`f426984 完成P8C终端接触稳定化与固定倾角验收`
- P8C tag：`baseline-tilted-t1-v0.1`
- P8C 最终证据：
  - `results/p8c3_validation_20260802/p8c3_final_summary.json`
  - `results/p8c4_validation_20260802/p8c4_final_summary.json`
- P8C 最终状态：`P8C-4 VALIDATION PASS / P8C T1 VALIDATION PASS / P8C-3 DESIGN GATE CLOSED`
- P8C-3 水平机体失败 Bag、seed2 滑移硬门失败、姿态发散、离板和恢复证据完整保留。

---

## 2. P9 方法与安全矩阵

方法定义和适用矩阵以 `docs/plans/P9_UNIFIED_EVALUATION_PLAN.md` 与 `scripts/p7_experiment_utils.py` 的冻结白名单为准：

- B0：完整 P4.7；
- B1：P4.7，额外 prediction horizon 为 0；
- B2：P4.7，甲板 velocity feedforward gain 为 0；
- B3：水平 `RELATIVE_MPC`；
- B4：`RELATIVE_MPC` + 已验证 H1 升沉处理；
- B5：固定正 T1 终端接触稳定化，不是 PX4 attitude setpoint 姿态对齐。

`NOT_APPLICABLE` 不执行、不计失败。负倾角、动态 roll/pitch、combined 和 dynamic attitude final descent 继续关闭。

---

## 3. 自动化实现证据

已完成并通过测试：

- 配置解析与严格校验；
- method-aware episode ID/manifest/参数快照；
- evaluator 自动路由与 primary + legacy 双评测；
- resume、失败归档、代码变化归档；
- stale process 检查和退出清理；
- overall/by_scenario/by_method/by_method_and_scenario 聚合；
- JSON/CSV/Markdown/PNG 输出；
- legacy P7 配置兼容；
- evaluator stdout 前置 ROS 日志与任意嵌套 JSON 的完整外层对象解析。

解析器根因是旧实现从最后一个 `{` 反向尝试，外层 JSON 含嵌套对象时可能先成功解析最内层字典并错误返回。当前实现从前向后检查每个 `{`，只接受解析结果为 `dict` 且 JSON 后仅有空白字符的完整对象；普通日志花括号、末尾垃圾和无 JSON 均会被拒绝。

---

## 4. 测试结果

```text
Targeted P9 tests: 54 tests, PASS
colcon build: 3 packages, PASS
colcon test-result --verbose: 340 tests, 0 errors, 0 failures, 0 skipped
git diff --check: PASS
```

---

## 5. 真实 smoke

配置：`config/experiments/p9_smoke.yaml`

- batch ID：`p9_smoke_20260803`
- 结果目录：`results/p9_smoke_20260803/`
- executed：`27`
- success：`20`
- failure：`7`
- success rate：`74.07%`
- failure breakdown：`SAFETY_GATE_FAILURE = 7`
- `NAV_LAND = 0`
- `Automatic Disarm = 0`

冻结组合结果：

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
```

B2 constant02 被水平 RMSE `0.15 m` 冻结门关闭；B4 heave_h1 被一轮约 `0.346 m` 的最大水平误差及 HOLD 垂直语义门关闭；B5 pitch `+2°` 三轮 attitude tracking P95 均超过 `1.5°` 冻结门。失败 episode、Bag、评测和聚合输出完整保留，不重跑挑结果。

---

## 6. B0 static/constant02 20+20

配置：`config/experiments/p9_baseline_20x20.yaml`

冻结 seeds：static `1001..1020`、constant02 `2001..2020`。

`results/p9_baseline_20x20_20260803/` 是基于旧提交 `b65713d` 的 interrupted pre-freeze batch：`4/40 completed`，static seeds `1001..1004` 为 `4/4 PASS`，`batch completed = false`。该目录仅作历史证据，排除在最终 baseline 统计之外；正式 40 轮必须基于解析器冻结后的干净提交使用新 batch ID 从 seed `1001` 重新开始。

---

## 7. 正式消融

配置：`config/experiments/p9_ablation.yaml`

smoke 后矩阵冻结为 `60 executable episodes + 30 gated NOT_APPLICABLE episodes`：

```text
APPLICABLE：B0/B1/B3 constant02、B0/B3 sinusoidal、B5 roll +2°，各 10 轮
NOT_APPLICABLE：B2 constant02、B4 heave_h1、B5 pitch +2°，各 10 个计划槽位
```

B5 roll `+2°` 是 fixed T1 唯一正式 touchdown 组合。正式 batch 尚未执行。

---

## 8. 聚合与论文初版输出

每个 batch 要求至少生成：

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
plots/*.png（matplotlib 可用时）
```

smoke 聚合输出已完整生成于 `results/p9_smoke_20260803/`。正式 baseline 和 ablation 聚合待对应 batch 完成后回填。P9 当前实际计划为 smoke `27 executed`、baseline `40 planned`、formal ablation `60 executable`，合计 `127 executable new episodes`；另记录 `30` 个 NOT_APPLICABLE 槽位。

---

## 9. 安全隔离检查

待最终回填：

```text
Ground Truth production subscription: 待最终 grep 复核
NAV_LAND count: smoke 0，正式 batch 待回填
Automatic Disarm count: smoke 0，正式 batch 待回填
negative tilt touchdown: DISABLED
rollpitch touchdown: DISABLED
combined touchdown: DISABLED
dynamic attitude final descent: DISABLED
```

---

## 10. 最终结论

```text
P9 UNIFIED EVALUATION V0.1: PENDING（baseline 与 formal ablation 尚未完成）
```

只有计划完成条件全部满足、P9 commit 完成且工作区干净时，才能创建本地标签 `baseline-unified-evaluation-v0.1`。
