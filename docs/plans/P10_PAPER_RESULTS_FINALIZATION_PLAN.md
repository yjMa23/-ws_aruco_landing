# P10 论文结果定稿与可复现实验包计划

## 0. 阶段状态

```text
P9 UNIFIED EVALUATION v0.1: FROZEN
P9 smoke: 20/27, 7 SAFETY_GATE_FAILURE
P9 baseline: 40/40
P9 formal ablation: 60/60
P9 NOT_APPLICABLE slots: 30
P10: PAPER RESULTS FINALIZATION AND EVIDENCE ARCHIVING
```

P10 是离线统计、论文产物和证据归档阶段，不是控制器能力开发阶段。本阶段不运行新的大规模 SITL，不继续调控制器，不扩大触地安全边界，也不通过换 seed、改阈值或挑选轮次改善结果。

## 1. 冻结输入

P10 只使用以下已有批次的 manifest、聚合文件、episode 结构化结果和配置快照：

```text
smoke:    results/p9_smoke_20260803
baseline: results/p9_baseline_20x20_20260804_71af1cc
ablation: results/p9_ablation_20260804_71af1cc
```

正式 baseline 与 ablation 的仿真运行提交必须为：

```text
71af1cc897136265a999c83dd6034bf156a32a50
```

P9 离线聚合修复提交和最终文档提交分别为：

```text
fc979fafc37f05fe4ae690e884153482a14d3c07
b20c8c9186e5869242417bd6c9539f1c0d97f54f
```

以下历史批次完整保留但排除在论文统计之外：

```text
results/p9_baseline_20x20_20260803
results/p9_baseline_20x20_20260803_a9d011d
results/p9_baseline_20x20_20260803_a9d011d_clean1
results/p9_ablation_20260804_a9d011d
```

## 2. 成功标准

P10 必须完成：

1. 冻结并校验 P9 正式输入证据；
2. 复核 manifest、summary、episodes 和实验矩阵一致性；
3. 计算 Wilson 95% 成功率置信区间；
4. 使用固定 seed 的 10000 次非参数 percentile bootstrap 计算连续指标均值 95% 置信区间；
5. 计算 B0/B1/B3 独立样本方法差异及 bootstrap 置信区间；
6. 从 episode 数据生成论文表格和 PNG/PDF/SVG 图表；
7. 生成数据 provenance、结构化文件 SHA256 清单和排除批次说明；
8. 提交仓库内论文摘要和数据来源文档；
9. 同步项目状态文档；
10. 通过针对性测试、相关包测试和一次全工作区 build/test；
11. 创建单一 P10 提交和 annotated tag `baseline-paper-results-v0.1`。

## 3. 明确禁止事项

```text
修改控制器参数
修改 MarkerSelector
修改 close-range 相机模型
修改 evaluator 安全阈值
修改 seed
删除失败轮
覆盖历史批次
开放负倾角 touchdown
开放 dynamic roll/pitch
开放 rollpitch 或 combined touchdown
启用 NAV_LAND
启用自动 Disarm
将 P8C-4 描述为 PX4 attitude setpoint 姿态控制
```

P10 输入只允许来自已有 `batch_manifest.json`、`summary.json`、`episodes.csv`、`experiment_matrix.csv`、每轮 `manifest.json`、`evaluation.json` 和配置快照。Ground Truth 仍只用于离线 evaluator。

## 4. 统计口径

### 4.1 成功率

正式成功率只使用 baseline 与 formal ablation 中实际执行且 `APPLICABLE` 的 episode；`NOT_APPLICABLE` 不进入分母，smoke 失败不混入正式实验成功率。smoke 单独报告为安全门证据。

Wilson score interval 输出：

```text
success
executed
point_estimate
ci95_lower
ci95_upper
method = wilson
```

覆盖 overall、by method、by scenario、by method/scenario 和每个正式 method/scenario/profile 组合。

### 4.2 连续指标

只统计成功 episode 中的有限值，并保持 P9 现有总体标准差语义。输出：

```text
count
mean
stddev
median
p95
min
max
mean_ci95_lower
mean_ci95_upper
```

均值置信区间使用：

```text
bootstrap repetitions: 10000
random seed: 20260804
confidence level: 0.95
method: deterministic nonparametric percentile bootstrap
```

少于 2 个有限样本时均值置信区间输出 `null` 并记录原因。

### 4.3 方法差异

使用非配对双样本 bootstrap，不假设不同 method 的 seed 成对。至少比较：

```text
constant02: B1 - B0 horizontal_error_rmse_m
constant02: B3 - B0 horizontal_error_rmse_m
sinusoidal: B3 - B0 horizontal_error_rmse_m
```

只有差值置信区间不跨 0 时，生成摘要才可写“差异方向得到当前样本支持”；否则写“当前样本不足以确认差异方向”。

## 5. 论文产物

本地可重建目录：

```text
results/p9_paper_results_v0.1/
```

包含统计 JSON/CSV、论文 Markdown、CSV/Markdown/LaTeX 表格、PNG/PDF/SVG 图表、provenance 和 `DATA_MANIFEST.sha256`。仓库只提交小型摘要、来源文档、统计代码和测试，不提交 Bag、完整 episode 目录或大体积中间结果。

## 6. 适用边界

P9 的 `10/10`、`20/20` 或 `40/40` 是当前有限样本中的观测成功率，不等价于真实成功概率为 100%。fixed T1 仅覆盖已冻结的正 `+2° roll` 正式 touchdown 组合，不能外推到负倾角、动态姿态、rollpitch 或 combined。

P8C-4 是 PX4 Offboard position 模式内的终端法向整形、接触顺应、状态化锚点、切向阻尼和受限预压，不是 PX4 attitude setpoint 姿态控制。

## 7. 外部动作

本阶段默认只创建本地提交和标签。远端同步以及原始数据外部归档仍需用户执行或明确授权；不得在本任务中自动 push。
