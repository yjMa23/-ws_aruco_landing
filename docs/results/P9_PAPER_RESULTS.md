# P9 论文结果定稿

## 1. 研究问题与统计范围

本文档使用 P9 已冻结的真实 PX4 SITL 结构化证据，回答以下问题：

1. 完整 P4.7 基线在 static 与 constant02 真实触地中的有限样本成功率和误差分布如何；
2. constant02 安全高度下，关闭额外位置预测的 B1 和水平相对 MPC 的 B3 相对 B0 有何差异；
3. sinusoidal 安全高度下，B3 相对 B0 的水平跟踪误差是否改善；
4. fixed T1 正 `+2° roll` 正式触地的滑移、姿态跟踪和触地速度分布如何；
5. smoke 安全门失败与正式 `NOT_APPLICABLE` 槽位应如何解释。

P10 没有运行新的 SITL，没有修改控制器、MarkerSelector、close-range 相机模型、evaluator 安全阈值或 seed，也没有删除失败轮、挑选轮次或扩大触地白名单。

## 2. 冻结输入与实验矩阵

冻结输入：

```text
smoke:    results/p9_smoke_20260803
baseline: results/p9_baseline_20x20_20260804_71af1cc
ablation: results/p9_ablation_20260804_71af1cc
```

正式仿真运行提交为：

```text
71af1cc897136265a999c83dd6034bf156a32a50
```

离线聚合修复提交与 P9 最终文档提交分别为：

```text
fc979fafc37f05fe4ae690e884153482a14d3c07
b20c8c9186e5869242417bd6c9539f1c0d97f54f
```

正式执行矩阵：

| 方法 | 场景 | Profile | 样本数 | 结果 |
| --- | --- | --- | ---: | ---: |
| B0 | static | touchdown | 20 | 20/20 |
| B0 | constant02 | touchdown | 20 | 20/20 |
| B0 | constant02 | safe-altitude | 10 | 10/10 |
| B1 | constant02 | safe-altitude | 10 | 10/10 |
| B3 | constant02 | safe-altitude | 10 | 10/10 |
| B0 | sinusoidal | safe-altitude | 10 | 10/10 |
| B3 | sinusoidal | safe-altitude | 10 | 10/10 |
| B5 | tilt_roll_pos_2deg | touchdown | 10 | 10/10 |

正式实验共执行 `100` 个 episode，全部成功。另有以下 `30` 个计划槽位因 smoke 安全门关闭而保持 `NOT_APPLICABLE`，未启动且不进入失败分母：

```text
B2 constant02 safe-altitude             10
B4 heave_h1 touchdown                   10
B5 tilt_pitch_pos_2deg touchdown        10
```

smoke 单独执行 `27` 轮，结果为 `20/27`；7 个失败全部为 `SAFETY_GATE_FAILURE`，不会混入正式成功率。

## 3. 成功率与 Wilson 95% 置信区间

成功率置信区间使用 Wilson score interval，`NOT_APPLICABLE` 不进入分母。

| 方法 | 场景 | Profile | 观测成功率 | Wilson 95% CI |
| --- | --- | --- | ---: | ---: |
| B0 | static | touchdown | 20/20 | [0.839, 1.000] |
| B0 | constant02 | touchdown | 20/20 | [0.839, 1.000] |
| B0 | constant02 | safe-altitude | 10/10 | [0.722, 1.000] |
| B1 | constant02 | safe-altitude | 10/10 | [0.722, 1.000] |
| B3 | constant02 | safe-altitude | 10/10 | [0.722, 1.000] |
| B0 | sinusoidal | safe-altitude | 10/10 | [0.722, 1.000] |
| B3 | sinusoidal | safe-altitude | 10/10 | [0.722, 1.000] |
| B5 | tilt_roll_pos_2deg | touchdown | 10/10 | [0.722, 1.000] |
| 正式实验总体 | 全部正式组合 | — | 100/100 | [0.963, 1.000] |

**`10/10`、`20/20` 或 `40/40` 只是当前有限样本中的观测成功率，不等价于真实成功概率为 100%。**

## 4. 连续指标与 bootstrap 95% 置信区间

均值置信区间使用确定性非参数 percentile bootstrap：

```text
bootstrap repetitions: 10000
fixed random seed: 20260804
confidence level: 0.95
```

标准差保持 P9 现有总体标准差语义。少于 2 个有限样本时均值置信区间输出 `null`，不会填 0。

### 4.1 baseline static / constant02 touchdown

| 场景 | N | 水平 RMSE 均值 (m) | 均值 95% CI (m) | 水平 RMSE P95 (m) | 降落时间均值 (s) | 均值 95% CI (s) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| static | 20 | 0.0285 | [0.0265, 0.0303] | 0.0336 | 18.2482 | [17.9807, 18.5132] |
| constant02 | 20 | 0.0285 | [0.0268, 0.0303] | 0.0357 | 18.1932 | [17.9932, 18.4057] |

### 4.2 constant02 safe-altitude 消融

| 方法 | N | 水平 RMSE 均值 (m) | 均值 95% CI (m) | P95 (m) |
| --- | ---: | ---: | ---: | ---: |
| B0 | 10 | 0.0324 | [0.0255, 0.0392] | 0.0482 |
| B1 | 10 | 0.0418 | [0.0372, 0.0462] | 0.0515 |
| B3 | 10 | 0.0291 | [0.0257, 0.0326] | 0.0367 |
| B2 | — | — | — | `NOT_APPLICABLE` |

### 4.3 sinusoidal safe-altitude 消融

| 方法 | N | 水平 RMSE 均值 (m) | 均值 95% CI (m) | P95 (m) |
| --- | ---: | ---: | ---: | ---: |
| B0 | 10 | 0.3643 | [0.3608, 0.3676] | 0.3709 |
| B3 | 10 | 0.1770 | [0.1740, 0.1803] | 0.1848 |

### 4.4 MPC 求解时间

| 场景 | N | episode 平均求解时间均值 (ms) | 均值 95% CI (ms) |
| --- | ---: | ---: | ---: |
| constant02 | 10 | 0.1879 | [0.1842, 0.1923] |
| sinusoidal | 10 | 0.1882 | [0.1785, 0.1956] |

### 4.5 fixed T1 roll `+2°` touchdown

| 指标 | N | 均值 | 均值 95% CI | P95 |
| --- | ---: | ---: | ---: | ---: |
| 触地滑移 (m) | 10 | 0.0529 | [0.0461, 0.0599] | 0.0698 |
| 法向跟踪误差 P95 (deg) | 10 | 0.2115 | [0.0968, 0.4197] | 0.7031 |
| HOLD 切向速度 P95 (m/s) | 10 | 0.0245 | [0.0227, 0.0263] | 0.0284 |
| 触地垂直速度 (m/s) | 10 | 0.0160 | [0.0088, 0.0240] | 0.0375 |

## 5. 方法差异分析

不同 method 视为独立样本，不假设 seed 配对。差值定义为 `method_a - method_b`。

| 比较 | 场景 | 差值 (m) | 相对差异 | bootstrap 差值 95% CI (m) | 解释 |
| --- | --- | ---: | ---: | ---: | --- |
| B1 - B0 | constant02 | +0.00935 | +28.83% | [0.00120, 0.01747] | 差异方向得到当前样本支持；关闭额外位置预测后误差增大 |
| B3 - B0 | constant02 | -0.00336 | -10.37% | [-0.01118, 0.00427] | 当前样本不足以确认差异方向 |
| B3 - B0 | sinusoidal | -0.18721 | -51.40% | [-0.19173, -0.18240] | 差异方向得到当前样本支持；B3 误差更低 |

不能仅根据点估计宣称统计差异。只有差值置信区间不跨 0 时，本文才写“差异方向得到当前样本支持”。

## 6. smoke 失败和 `NOT_APPLICABLE` 的解释

smoke 安全门结果：

```text
B2 constant02 safe-altitude      0/3，3 SAFETY_GATE_FAILURE
B4 heave_h1 touchdown            2/3，1 SAFETY_GATE_FAILURE
B5 pitch +2° touchdown           0/3，3 SAFETY_GATE_FAILURE
```

上述失败完整保留，不调参、不换 seed、不重跑挑结果。对应正式矩阵中的 `30` 个槽位标记为 `NOT_APPLICABLE`：它们不是 0% 成功率，也不是正式失败，而是未获正式执行授权的组合。

## 7. 安全边界和 Ground Truth 隔离

- fixed T1 正式结果只覆盖 B5 正 `+2° roll` touchdown；不能外推到负倾角、动态 roll/pitch、rollpitch 或 combined。
- B5 pitch `+2°` 已被 smoke 安全门关闭，不应与 roll 结果合并。
- P8C-4 是 PX4 Offboard position 模式内的终端法向整形、接触顺应、状态化锚点、切向阻尼和受限预压，不是 PX4 attitude setpoint 姿态控制。
- Ground Truth 只进入离线 evaluator，不进入控制器。
- 三个冻结批次合计 `NAV_LAND / 自动 Disarm = 0 / 0`。

## 8. 当前统计局限

1. 每个正式消融组合只有 10 个冻结样本，baseline 每个场景 20 个样本；置信区间仍较宽。
2. 所有结果来自固定 SITL 场景与冻结 seed，不能直接代表真实海况或实机分布。
3. Wilson 和 bootstrap 只量化当前有限样本不确定性，不覆盖 simulator-to-real 差异和场景分布漂移。
4. 未测试负倾角、动态姿态、rollpitch、combined 和更复杂海况。
5. 为避免重复读取大体积 Bag，`DATA_MANIFEST.sha256` 完整覆盖小型结构化证据；Bag 只记录目录数量、总文件数和总字节数，不逐个哈希 SQLite 分片。

## 9. 可复现命令与产物

```bash
python3 scripts/finalize_p9_paper_results.py \
  --smoke results/p9_smoke_20260803 \
  --baseline results/p9_baseline_20x20_20260804_71af1cc \
  --ablation results/p9_ablation_20260804_71af1cc \
  --output results/p9_paper_results_v0.1
```

输出包括：

```text
paper_summary.json
success_rate_confidence_intervals.csv
continuous_metric_confidence_intervals.csv
method_comparisons.csv
data_provenance.json
DATA_MANIFEST.sha256
P9_PAPER_RESULTS.md
tables/*.csv|md|tex
plots/*.png|pdf|svg
```

相同输入连续生成两次的完整输出目录树 SHA256 均为：

```text
aa3af1ccc554910174ee0d242360b593ebd80f984013a46aa2ef328c92c31a56
```

仓库内提交的小型矢量图位于 `docs/assets/p9/`，包含以下 7 张图的 PDF 与 SVG：

```text
baseline_horizontal_rmse_distribution
constant02_b0_b1_b3_horizontal_rmse
sinusoidal_b0_b3_horizontal_rmse
formal_success_rate_wilson_ci
mpc_solve_time
fixed_t1_slip_and_attitude_tracking
smoke_safety_gate_and_applicability
```
