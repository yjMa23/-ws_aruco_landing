# 论文结果

本文档只汇总冻结的 PX4 SITL 结构化证据。本次文档与接口迁移没有运行新的 SITL，没有调整控制器参数、安全阈值、seed 或样本选择。

## 冻结结论

| 数据集 | 执行结果 |
| --- | --- |
| 安全门 smoke | `20/27`；7 个失败均为 `SAFETY_GATE_FAILURE` |
| static / constant02 触地基线 | `40/40` |
| 正式消融 | `60/60` |
| 被安全门关闭的正式槽位 | `30 NOT_APPLICABLE` |
| `NAV_LAND` / 自动 Disarm | `0 / 0` |

正式仿真提交为 `71af1cc897136265a999c83dd6034bf156a32a50`。`10/10`、`20/20` 或 `40/40` 都是有限样本观测，不代表真实成功率为 100%。

## 正式观测

| 方法 | 场景 | 模式 | 成功率 | Wilson 95% 区间 |
| --- | --- | --- | ---: | ---: |
| B0 | static | touchdown | 20/20 | [0.839, 1.000] |
| B0 | constant02 | touchdown | 20/20 | [0.839, 1.000] |
| B0 | constant02 | safe-altitude | 10/10 | [0.722, 1.000] |
| B1 | constant02 | safe-altitude | 10/10 | [0.722, 1.000] |
| B3 | constant02 | safe-altitude | 10/10 | [0.722, 1.000] |
| B0 | sinusoidal | safe-altitude | 10/10 | [0.722, 1.000] |
| B3 | sinusoidal | safe-altitude | 10/10 | [0.722, 1.000] |
| B5 | tilt_roll_pos_2deg | touchdown | 10/10 | [0.722, 1.000] |

主要连续指标：

| 方法 / 场景 | 指标 | 均值 | 均值 95% 区间 | P95 |
| --- | --- | ---: | ---: | ---: |
| B0 / static touchdown | 水平 RMSE (m) | 0.0285 | [0.027, 0.030] | 0.0336 |
| B0 / constant02 touchdown | 水平 RMSE (m) | 0.0285 | [0.027, 0.030] | 0.0357 |
| B0 / sinusoidal safe-altitude | 水平 RMSE (m) | 0.3643 | [0.361, 0.368] | 0.3709 |
| B3 / sinusoidal safe-altitude | 水平 RMSE (m) | 0.1770 | [0.174, 0.180] | 0.1848 |
| B3 / constant02 safe-altitude | MPC 平均求解时间 (ms) | 0.1879 | [0.184, 0.192] | 0.1994 |
| B5 / tilt_roll_pos_2deg touchdown | 触地滑移 (m) | 0.0529 | [0.046, 0.060] | 0.0698 |
| B5 / tilt_roll_pos_2deg touchdown | 法向跟踪 P95 (deg) | 0.2115 | [0.097, 0.420] | 0.7031 |

置信区间使用固定 seed `20260804`、10000 次非参数 bootstrap。方法间按独立样本比较；不会把相同 seed 编号解释为配对样本。

## 安全门解释

以下组合在 smoke 中失败，因此正式实验保留为 `NOT_APPLICABLE`，不计为零成功率，也不隐藏失败证据：

- B2 / constant02 / safe-altitude：10 个槽位。
- B4 / heave_h1 / touchdown：10 个槽位。
- B5 / tilt_pitch_pos_2deg / touchdown：10 个槽位。

B5 / tilt_roll_pos_2deg 是唯一完成正式触地的固定正倾角组合。结论不能外推到负倾角、动态 roll/pitch、rollpitch 或 combined。终端接触稳定化仍是 Offboard position 模式内的参考整形与接触顺应，不是 PX4 attitude setpoint 控制。

## 产物

完整可复现产物位于被 Git 忽略的 `results/paper_results_v0.1/`，包括 6 组论文表格、21 个 PNG/PDF/SVG 图文件、Wilson/Bootstrap 统计和 782 条结构化证据哈希。仓库内图表位于 `docs/assets/paper_results/`，数据边界和重建命令见 [数据来源](DATA_PROVENANCE.md)。

