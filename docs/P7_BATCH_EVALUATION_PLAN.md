# P7 批量评测第一版执行计划

## 1. 阶段目标

P7 第一版将已经人工验证的 P6B 最终下降链路转化为可重复、可恢复、可统计的自动实验管线。

本阶段只做实验编排、结果保存、失败分类和基础统计，不修改：

- 控制器算法与参数；
- Marker 参数；
- 相机参数；
- landing window；
- touchdown 阈值；
- 视觉超时；
- 下降速度；
- 安全状态机。

P7 完成后再进入 P8 传统方法消融，不在本阶段增加 MPC、强化学习、升沉触地、倾斜触地或组合运动触地。

---

## 2. 场景范围

第一版只支持：

```text
static
constant02
```

暂不支持：

```text
constant
sinusoidal
heave
rollpitch
combined
```

原因：P7 第一版优先冻结已经完成真实接触验证的静止和 `0.2 m/s` 水平匀速基线。其他场景需要独立的触地安全验证后再纳入批量统计。

---

## 3. 实验顺序

### 3.1 冒烟

先顺序运行：

```text
static:     3 次
constant02: 3 次
```

只有 3+3 自动连续运行、逐轮清理和结果保存正常后，才进入回归。

### 3.2 回归

随后顺序运行：

```text
static:     20 次
constant02: 20 次
```

第一版不并行。某一轮失败不得阻塞后续实验，失败轮必须立即写入结构化结果。

---

## 4. 每轮输入

每个 episode 至少固定：

- `episode_id`；
- `batch_id`；
- `scenario`；
- `seed`；
- Git commit；
- 工作树是否 dirty；
- 相机模型；
- 控制器参数快照；
- 场景与 GNSS 参数快照；
- episode timeout；
- startup timeout；
- `TOUCHDOWN_HOLD` 最短确认时间。

`seed` 同时覆盖移动甲板仿真和 GNSS 传感器仿真的 `random_seed`，保证同一轮的随机输入可复现。

---

## 5. 单轮运行流程

单轮运行器复用：

```text
scripts/start_sitl.sh
scripts/evaluate_p6b_touchdown.py
```

执行流程：

```text
检查残留进程
→ 生成 manifest 和配置快照
→ 非交互启动 start_sitl.sh
→ 记录轻量 Bag
→ 订阅 /landing/state
→ 按 ROS 状态监控任务
→ TOUCHDOWN_HOLD 连续达到 10 s
→ 停止完整 SITL 进程树
→ 运行 P6B evaluator
→ 保存 evaluation.json
→ 写入最终 manifest
```

成功不得通过固定 sleep 判定。必须同时满足：

1. ROS 状态进入 `TOUCHDOWN_HOLD`；
2. 该状态连续保持至少 10 秒；
3. rosbag 完整停止；
4. `evaluate_p6b_touchdown.py` 返回正向 PASS；
5. `constant02` 使用 `--require-moving-deck`；
6. 清理后没有残留 PX4、Gazebo、MicroXRCEAgent 或本项目 ROS 进程。

以下事件立即结束本轮并保存失败结果：

- `ABORT`；
- 启动进程异常退出；
- PX4/ROS 状态启动超时；
- episode 总超时；
- evaluator 错误；
- 清理失败。

---

## 6. 录包策略

成功轮默认记录轻量 Bag，包括状态、估计、控制目标、Marker 诊断、PX4 状态和 Ground Truth 评测话题。

默认不录制：

- 原始相机图像；
- camera info；
- ArUco debug image。

只有显式设置 `record_camera_debug: true` 时才录制大体积视觉话题。

失败轮保留：

- 完整 `run.log`；
- 已生成的 Bag；
- evaluator 人类可读输出；
- evaluator JSON；
- 配置快照；
- manifest 中的失败分类和失败详情。

---

## 7. 输出目录

```text
results/<batch_id>/
  batch_manifest.json
  episodes.csv
  summary.json
  summary.csv
  failures.csv
  <episode_id>/
    manifest.json
    controller_config.yaml
    scenario_config.yaml
    evaluation.json
    evaluation.txt
    run.log
    bag/
```

`manifest.json` 至少包含：

- `episode_id`；
- `batch_id`；
- `scenario`；
- `seed`；
- `git_commit`；
- `dirty_worktree`；
- `start_wall_time`；
- `end_wall_time`；
- `duration_s`；
- `camera_model`；
- `start_command`；
- `exit_code`；
- `success`；
- `failure_reason`；
- `failure_detail`；
- `bag_path`；
- `evaluation_path`；
- `state_sequence`；
- `completed`。

---

## 8. 统一失败分类

第一版支持：

```text
NONE
STARTUP_FAILURE
PX4_TIMEOUT
PROCESS_EXITED
ARUCO_NOT_ACQUIRED
VISION_LOST
LANDING_WINDOW_TIMEOUT
TRACKING_DIVERGED
RECOVERY_LIMIT
TOUCHDOWN_NOT_CONFIRMED
PX4_ABORT
EPISODE_TIMEOUT
EVALUATION_ERROR
CLEANUP_FAILURE
UNKNOWN
```

分类依据来自：

- 单轮运行事件；
- `/landing/state` 状态序列；
- `start_sitl.sh` 日志；
- 脚本退出码；
- `evaluate_p6b_touchdown.py` JSON。

不要求用户逐轮手工填写。

---

## 9. 批量执行规则

批量运行器必须：

- 读取 YAML；
- 只接受 `static` 和 `constant02`；
- 展开 repetitions 与 seeds；
- 顺序执行；
- 单轮失败后继续；
- 每轮结束立即更新 `batch_manifest.json` 和 `episodes.csv`；
- 支持 `--dry-run`；
- 支持 `--resume --batch-id <existing_batch_id>`；
- resume 时只跳过 `manifest.json` 已完整结束的 episode；
- 不覆盖已有完整 episode。

---

## 10. 聚合指标

至少统计：

- 总实验数；
- 成功数；
- 成功率；
- 各失败类型数量及占比；
- landing time；
- horizontal RMSE；
- maximum horizontal error；
- touchdown vertical speed；
- candidate-to-confirm delay；
- recovery count；
- Marker switch count；
- hold duration。

每个数值指标输出：

```text
count
mean
standard deviation
median
P90
P95
```

输出文件：

```text
summary.json
summary.csv
failures.csv
```

本阶段不生成复杂绘图或论文排版。

---

## 11. 配置文件

冒烟配置：

```text
config/experiments/p7_smoke.yaml
```

回归配置：

```text
config/experiments/p7_baseline.yaml
```

冒烟种子：

```text
static:     101, 102, 103
constant02: 201, 202, 203
```

回归种子：

```text
static:     1001..1020
constant02: 2001..2020
```

---

## 12. 命令

### 12.1 配置检查与 dry-run

```bash
python3 scripts/run_batch_experiments.py \
  config/experiments/p7_smoke.yaml \
  --dry-run
```

### 12.2 单轮

```bash
python3 scripts/run_single_experiment.py \
  --scenario static \
  --seed 101 \
  --episode-timeout 600 \
  --startup-timeout 120 \
  --touchdown-hold 10 \
  --output-directory results/manual
```

### 12.3 3+3 冒烟

```bash
python3 scripts/run_batch_experiments.py \
  config/experiments/p7_smoke.yaml
```

### 12.4 20+20 回归

```bash
python3 scripts/run_batch_experiments.py \
  config/experiments/p7_baseline.yaml
```

### 12.5 恢复已有批次

```bash
python3 scripts/run_batch_experiments.py \
  config/experiments/p7_smoke.yaml \
  --resume \
  --batch-id <EXISTING_BATCH_ID>
```

### 12.6 聚合

```bash
python3 scripts/aggregate_results.py \
  results/<BATCH_ID>
```

---

## 13. 测试边界

单元测试只使用：

- 临时目录；
- 伪造 evaluator JSON；
- 伪造单轮 runner；
- dry-run。

单元测试不得启动：

- PX4；
- Gazebo；
- MicroXRCEAgent；
- ROS 2 仿真节点。

覆盖：

- YAML 解析；
- episode ID 唯一性；
- seed 展开；
- resume；
- evaluator JSON；
- 成功、超时和启动失败分类；
- 单轮失败后 batch 继续；
- mean/median/P90/P95；
- 空结果和损坏结果；
- dry-run 不启动真实进程。

---

## 14. 完成定义

P7 代码第一版完成需要：

1. 构建和全部单元测试通过；
2. start_sitl shell 语法通过；
3. Python 语法检查通过；
4. 所有新增脚本 `--help` 正常；
5. smoke 配置 `--dry-run` 正常；
6. 聚合临时样例正常。

P7 阶段正式完成还需要：

1. P6B static/constant02 Bag 证据已冻结；
2. 真实 3+3 冒烟可自动连续运行；
3. 每轮结果结构完整；
4. 无残留仿真进程；
5. 冒烟失败不会阻塞后续轮；
6. 聚合结果正确。

当前环境若不适合真实 SITL，只声明“P7 管线代码第一版完成”，不得声明“P7 已完成”。

---

## 15. 2026-07-29 真实执行状态

已真实执行：

- static 单轮自动启动、状态监控、轻量录包、统一清理和 P6B 评测；
- constant02 单轮自动启动、状态监控、轻量录包、统一清理和带 `--require-moving-deck` 的 P6B 评测；
- 两轮均完成到 `FINAL_DESCENT`，证明 P7 单轮编排主链可运行；
- 运行中修复 ROS CLI daemon 依赖、DDS 就绪发现、状态监控、episode 计时和失败分类问题。

两轮最终结果均为 P6B FAIL：参考到达 `0.15 m` 后没有 PX4 接触位，也没有 `CANDIDATE / CONFIRMED / TOUCHDOWN_HOLD`。因此没有执行 3+3 冒烟，避免连续重复同一前置失败。

当前状态：

```text
P7 pipeline implementation: completed first version
P7 real single-run orchestration: verified
P7 3+3 smoke: blocked by P6B failure on 2026-07-29
P7 20+20 regression: not started
P7 stage: not completed at that time
```

---

## 16. 2026-07-30 真实 3+3 冒烟

P6B 终端落板逻辑修补并完成 static/constant02 单轮 PASS 后，执行：

```bash
python3 scripts/run_batch_experiments.py \
  config/experiments/p7_smoke.yaml \
  --batch-id p7_smoke_terminal_20260730
```

批次目录：

```text
results/p7_smoke_terminal_20260730/
```

执行结果：

```text
planned episodes:   6
completed episodes: 6
successful:         6
failed:             0
success rate:       100%
```

场景和种子：

```text
static:     101, 102, 103 → 3/3 PASS
constant02: 201, 202, 203 → 3/3 PASS
```

聚合指标：

| 指标 | mean | P95 |
| --- | ---: | ---: |
| 落地时间 | `24.9006 s` | `25.3006 s` |
| 水平 RMSE | `0.02508 m` | `0.02851 m` |
| 最大水平误差 | `0.05048 m` | `0.06497 m` |
| 触地垂直速度 | `0.00602 m/s` | `0.01172 m/s` |
| 候选到确认延迟 | `0.45835 s` | `0.48753 s` |
| `TOUCHDOWN_HOLD` 持续时间 | `10.00019 s` | `10.00030 s` |
| 恢复次数 | `0` | `0` |

验证项：

- 六轮均进入 `TOUCHDOWN_CANDIDATE_HOLD → TOUCHDOWN_HOLD`；
- 六轮最低参考高度均为 `0.05 m`；
- constant02 三轮均满足 moving-deck 判据；
- 每轮完整保存 manifest、配置快照、轻量 Bag 和 evaluator JSON；
- 每轮结束后无 PX4、Gazebo、MicroXRCEAgent 或 ROS SITL 残留；
- 聚合生成 `summary.json`、`summary.csv` 和 `failures.csv`。

当前状态：

```text
P7 pipeline implementation: complete
P7 3+3 smoke: complete, 6/6 PASS
P7 20+20 regression: pending
Next action: run config/experiments/p7_baseline.yaml
```
