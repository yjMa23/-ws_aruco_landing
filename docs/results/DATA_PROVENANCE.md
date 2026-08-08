# 数据来源与可复现证据

## 冻结输入

论文统计只读取三个显式目录：

```text
results/paper_evaluation_smoke_20260803
results/paper_baseline_20x20_20260804_71af1cc
results/paper_ablation_20260804_71af1cc
```

正式仿真提交为 `71af1cc897136265a999c83dd6034bf156a32a50`，离线聚合修复提交为 `fc979fafc37f05fe4ae690e884153482a14d3c07`。三个输入均保持原 commit、样本、成败分类和数值；本次只迁移名称与文本路径。

| 输入 | planned / completed | success / failure | Bag 目录 | 文件数 | 字节数 |
| --- | ---: | ---: | ---: | ---: | ---: |
| paper evaluation smoke | 27 / 27 | 20 / 7 | 28 | 563 | 290477322 |
| paper baseline | 40 / 40 | 40 / 0 | 40 | 494 | 382302689 |
| paper ablation | 60 / 60 | 60 / 0 | 60 | 734 | 282379077 |

smoke 包含失败、被替代和重试审计目录，所以递归评测文件数可能大于 27。统计分母只由根目录 `episodes.csv` 中的 active episode 决定。

## 结构化证据哈希

| 输入 | batch manifest | summary.json | episodes.csv | experiment matrix | 全部 evaluation.json 聚合 |
| --- | --- | --- | --- | --- | --- |
| smoke | `9f8eeaefaa0ceb71ddbcf145171be3a78b62bcd9721efde3fccff224aa88b02c` | `fe6236be0aa16f29a94d2bbe7eb09277af50ac788624e357559312eab2e500e1` | `a51f2ab132dd514837a55bb18e41f476026499b89893ba8b3fc3e8106b2a3e7e` | `df85258d25e09d10c292bc0a3be9225695b6356f06dda57bdf147c10bdc5f277` | `a43954c1273c110a7204af228401f58b5bd0bba8f2797752572ef0085f763a7c` |
| baseline | `fc95cfa767b684042a9c8cb9d355cc6b4c3fcaec0358c135352af745c5903659` | `3a05c5cf954078f49011bfb8db5e46933cba6c552d358f4c87385437b61e727c` | `fd05aa777d2a2daf72e36f4a030bda0f3fdf2803a65656e42a99d1f16b94656a` | `fd3dc6cf9539147ae8e9b0ec2cf7ce2cef9806c41248462a4b293144deda7717` | `79399e9db1e06b77a7607a87f600d5b884a6ea093c54123b80c05d5789af6dd0` |
| ablation | `e69d7af5432909409e43f3e4c2917d29ce6122d35f9d2ac4229c9280e8cc4630` | `b90a94d81b59627a9bf0a8aedeb5f6d771980e7a4c1495004e5667edd57f5ff9` | `aa68e7f9092e2a4e890240345b74e2a426895f23183cf4de26d41a7cb3ba6675` | `d7d39c70067737e39d6c1605cadbb7296954c659f10877d619f6b7ea27c952ac` | `60a166ccd4824df14ff77dafa56c037c4e16cd5e2e4505314c363794114b042b` |

`results/paper_results_v0.1/DATA_MANIFEST.sha256` 包含 782 条小型结构化文件记录，其 SHA256 为：

```text
0d4af75b4286dddca2e98f6289fc540f49f7089b72390364f98180d7aa4893d0
```

清单覆盖批次 manifest、汇总表、实验矩阵，以及每轮 manifest、evaluation、方法参数、控制器配置和场景配置。它不把大体积 Bag 数据库纳入论文清单。

本次本地路径迁移前后分别对全部 373 个 `.db3` / `.mcap` 文件逐一计算 SHA256；排序后的哈希集合完全一致，证明 rosbag 数据库字节未改变。

## 排除的历史批次

以下目录保留为失败或缺陷证据，但生成器会拒绝把它们作为论文输入：

| 目录 | 原因 |
| --- | --- |
| `results/paper_baseline_20x20_20260803` | 冻结前中断，仅完成 4/40 |
| `results/paper_baseline_20x20_20260803_a9d011d` | 批量编排污染证据 |
| `results/paper_baseline_20x20_20260803_a9d011d_clean1` | 时钟修复前、旧仿真提交上的完整基线 |
| `results/paper_ablation_20260804_a9d011d` | SYSTEM_TIME / ROS_TIME 混用缺陷证据 |

## 重建

```bash
python3 scripts/finalize_paper_results.py \
  --smoke results/paper_evaluation_smoke_20260803 \
  --baseline results/paper_baseline_20x20_20260804_71af1cc \
  --ablation results/paper_ablation_20260804_71af1cc \
  --output results/paper_results_v0.1
```

生成器会核对成败计数、30 个 `NOT_APPLICABLE` 槽位、commit、dirty 状态、安全门、`NAV_LAND / Disarm = 0 / 0` 和排除批次。`results/` 不进入 Git；原始 Bag 外部归档和远端同步仍需单独授权。
