# P9 统一批量评测第一版验收记录

## 0. 当前状态

```text
P8C fixed T1: FROZEN
P9 PLAN: PASS
P9 IMPLEMENTATION: IN PROGRESS
P9 TEST: PENDING
P9 REAL BATCH: PENDING
P9 AGGREGATION: PENDING
P9 FINAL VALIDATION: PENDING
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

待最终回填：

- 配置解析与严格校验；
- method-aware episode ID/manifest/参数快照；
- evaluator 自动路由与 primary + legacy 双评测；
- resume、失败归档、代码变化归档；
- stale process 检查和退出清理；
- overall/by_scenario/by_method/by_method_and_scenario 聚合；
- JSON/CSV/Markdown/PNG 输出；
- legacy P7 配置兼容。

---

## 4. 测试结果

待最终回填：

```text
Targeted P9 tests: PENDING
colcon build: PENDING
colcon test: PENDING
colcon test-result --verbose: PENDING
git diff --check: PENDING
```

---

## 5. 真实 smoke

配置：`config/experiments/p9_smoke.yaml`

待最终回填实际 batch ID、运行数、成功/失败、失败分类和证据目录。

---

## 6. B0 static/constant02 20+20

配置：`config/experiments/p9_baseline_20x20.yaml`

冻结 seeds：static `1001..1020`、constant02 `2001..2020`。待最终回填实际结果。

---

## 7. 正式消融

配置：`config/experiments/p9_ablation.yaml`

待最终回填 B0/B1/B2/B3、B4 与 B5 的实际运行结果和关键统计。

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

待最终回填结果目录和 overall/by_scenario/by_method 关键值。

---

## 9. 安全隔离检查

待最终回填：

```text
Ground Truth production subscription: PENDING
NAV_LAND count: PENDING
Automatic Disarm count: PENDING
negative tilt touchdown: DISABLED
rollpitch touchdown: DISABLED
combined touchdown: DISABLED
dynamic attitude final descent: DISABLED
```

---

## 10. 最终结论

```text
P9 UNIFIED EVALUATION V0.1: PENDING
```

只有计划完成条件全部满足、P9 commit 完成且工作区干净时，才能创建本地标签 `baseline-unified-evaluation-v0.1`。
