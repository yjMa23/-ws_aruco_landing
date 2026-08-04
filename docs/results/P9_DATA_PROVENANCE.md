# P9 论文数据来源与可复现证据

## 1. 冻结批次

P10 论文统计只使用以下三个显式输入目录：

```text
results/p9_smoke_20260803
results/p9_baseline_20x20_20260804_71af1cc
results/p9_ablation_20260804_71af1cc
```

正式 baseline 和 ablation 的每个 active episode 均记录：

```text
git_commit = 71af1cc897136265a999c83dd6034bf156a32a50
dirty_worktree = false
```

P9 离线聚合修复提交和最终文档提交为：

```text
fc979fafc37f05fe4ae690e884153482a14d3c07
b20c8c9186e5869242417bd6c9539f1c0d97f54f
```

## 2. 批次级 provenance

### 2.1 smoke

```text
batch_id: p9_smoke_20260803
batch_path: results/p9_smoke_20260803
config_path: config/experiments/p9_smoke.yaml
git_commit: 90e40401548f5d05402f9949980540efe6c9f074
dirty_state: false
planned/completed: 27/27
success/failure: 20/7
manifest SHA256: 69012928caf975fca1e3155e039ddf8453139ff0a3e22bb1f89f70f5fe420274
summary.json SHA256: 160e1a69849aa749f0028fe2e1e2085bfe32eddf255687746ea865ea1589a78f
episodes.csv SHA256: e80f62435715ba24eee4ace375a72a9722114b46dda36fdc2b1d720e11b81890
experiment_matrix.csv SHA256: df85258d25e09d10c292bc0a3be9225695b6356f06dda57bdf147c10bdc5f277
all evaluation.json count: 39
all evaluation.json aggregate SHA256: d44e4cba829d7e9d4f963ae3223c4bcd4f7fa601efad6e61ced13624bc26bfbc
Bag directory count: 28
total file count: 563
total bytes: 290440908
```

smoke 中存在失败、superseded 或重试审计目录，因此递归 `evaluation.json` 与 Bag 目录数量可能大于 active statistical episode 数 `27`。正式分母严格由根目录 `episodes.csv` 中的 active episode 决定；审计目录只保留并哈希，不进入统计。

### 2.2 baseline

```text
batch_id: p9_baseline_20x20_20260804_71af1cc
batch_path: results/p9_baseline_20x20_20260804_71af1cc
config_path: config/experiments/p9_baseline_20x20.yaml
git_commit: 71af1cc897136265a999c83dd6034bf156a32a50
dirty_state: false
planned/completed: 40/40
success/failure: 40/0
manifest SHA256: c35a2428bb026b2af41b64fb8fa8dc711ac2b303336317c712ba1f95ac8777fd
summary.json SHA256: 4cfd0f6ef882af7760768d2cd8ce555aa42eed0a25bfcfdfc64b4276849544e7
episodes.csv SHA256: fb98d68971323e9dfaea191f4090d899b4bb63f55a0100ae99cbe5cd4c747597
experiment_matrix.csv SHA256: fd3dc6cf9539147ae8e9b0ec2cf7ce2cef9806c41248462a4b293144deda7717
all evaluation.json count: 40
all evaluation.json aggregate SHA256: b5148442485e450bb1131438a5a80da192373daaf4c6b13923061db89537f77b
Bag directory count: 40
total file count: 494
total bytes: 382286939
```

### 2.3 formal ablation

```text
batch_id: p9_ablation_20260804_71af1cc
batch_path: results/p9_ablation_20260804_71af1cc
config_path: config/experiments/p9_ablation.yaml
git_commit: 71af1cc897136265a999c83dd6034bf156a32a50
dirty_state: false
planned/completed: 60/60
success/failure: 60/0
manifest SHA256: 5f3fc14ba9052f4c5f3f505c2cd5f8271b2d288b9b3517dca621c94cc13237c7
summary.json SHA256: 1a5a5165955e5086985b3aeb104ca511fd2018b81daf398b7f9e866c4f6b85b5
episodes.csv SHA256: 0a9a2cd3340af3d76bed4b18fe808c2e8e5cf4411bbebe403da9ad387d216330
experiment_matrix.csv SHA256: d7d39c70067737e39d6c1605cadbb7296954c659f10877d619f6b7ea27c952ac
all evaluation.json count: 60
all evaluation.json aggregate SHA256: 8cb9ccebff94f51f07a7fac2d19d7e05696d6dd3951b21ee5ed3e57f08ba3304
Bag directory count: 60
total file count: 734
total bytes: 282354267
```

正式 ablation 的排除组合：

```text
B2 / constant02 / safe-altitude          10 NOT_APPLICABLE
B4 / heave_h1 / touchdown                10 NOT_APPLICABLE
B5 / tilt_pitch_pos_2deg / touchdown     10 NOT_APPLICABLE
```

## 3. `DATA_MANIFEST.sha256`

本地生成文件：

```text
results/p9_paper_results_v0.1/DATA_MANIFEST.sha256
```

清单包含 `782` 条小型结构化文件记录，覆盖：

```text
batch_manifest.json
summary.json
summary.csv
episodes.csv
failures.csv
by_method.csv
by_scenario.csv
by_method_scenario.csv
experiment_matrix.csv
每轮 manifest.json
每轮 evaluation.json
每轮 method_parameters.yaml
每轮 controller_config.yaml
每轮 scenario_config.yaml
```

清单文件自身 SHA256：

```text
b7da4a9e6ca53744bf8feda08da94f6f71b28d70cd0e8b964c378e82db48c494
```

为保留 smoke 历史失败与重试审计证据，清单会哈希递归找到的指定结构化文件；正式统计只读取根 `episodes.csv` 对应的 active episode。

## 4. Bag 哈希限制

三个冻结输入批次中的 Bag SQLite 负载不逐文件计算 SHA256，原因是避免为论文定稿重复读取数百 MB 到数 GB 的大文件。provenance 已记录：

- Bag 目录数量；
- 批次总文件数量；
- 批次总字节数；
- 全部小型结构化文件哈希；
- 全部 `evaluation.json` 的数量和聚合 SHA256。

因此当前证据包可以验证统计输入、配置和 evaluator 输出是否改变，但不能仅凭 `DATA_MANIFEST.sha256` 验证每个 Bag SQLite 分片的逐字节完整性。原始 Bag 应由用户在 Git 外部单独归档。

## 5. 明确排除的历史批次

以下历史目录不得删除，但不进入 P10 论文统计：

| 批次 | 排除原因 |
| --- | --- |
| `results/p9_baseline_20x20_20260803` | interrupted pre-freeze batch，仅完成 4/40 |
| `results/p9_baseline_20x20_20260803_a9d011d` | 批量编排污染证据 |
| `results/p9_baseline_20x20_20260803_a9d011d_clean1` | 时钟修复前、旧仿真提交上的完整 baseline |
| `results/p9_ablation_20260804_a9d011d` | 暴露 SYSTEM_TIME/ROS_TIME 混用缺陷的证据批次 |

统计脚本只接收三个显式输入目录，并对上述排除路径做硬失败检查，不会自动扫描或合并其他 `results/` 目录。

## 6. 可复现命令

```bash
python3 scripts/finalize_p9_paper_results.py \
  --smoke results/p9_smoke_20260803 \
  --baseline results/p9_baseline_20x20_20260804_71af1cc \
  --ablation results/p9_ablation_20260804_71af1cc \
  --output results/p9_paper_results_v0.1
```

脚本会在写入前校验：

- 三个输入目录和根结构化文件完整；
- active episode 的 `manifest.json` 与 `evaluation.json` 可解析；
- smoke、baseline、ablation 的执行和成功计数；
- smoke 失败原因和关闭组合；
- formal `30` 个 `NOT_APPLICABLE` 槽位；
- baseline/ablation 精确仿真提交和 dirty 状态；
- NAV_LAND / 自动 Disarm = 0 / 0；
- 历史排除批次未被作为输入；
- 输出不包含硬编码 `/home/j`。

相同输入连续运行两次时，完整输出目录树 SHA256 均为：

```text
aa3af1ccc554910174ee0d242360b593ebd80f984013a46aa2ef328c92c31a56
```

## 7. 外部归档与远端同步

`results/` 受 `.gitignore` 排除，Git 只提交统计脚本、测试和仓库内摘要。原始 Bag 外部归档以及远端 push 仍需用户执行或明确授权，本任务不自动进行。
