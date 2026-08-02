# P8C-3 姿态对齐与接触稳定性独立设计门

## 1. 状态

截至 2026-08-02，原设计门已由 P8C-4 完整执行并关闭：

```text
P8C-3 FAILURE EVIDENCE PRESERVED
P8C-4 VALIDATION PASS
P8C T1 VALIDATION PASS
P8C-3 DESIGN GATE CLOSED
```

P8C-3 已完成启动白名单、TDD、严格 evaluator、无人值守 PX4 SITL、Bag 和既有策略回归，但水平机体策略 A 在固定 `+2° roll` 真实触地中未形成可重复的 3/3 安全结果。本文件只冻结下一阶段的研究和实现决策，不在 P8C-3 内实现姿态对齐。

## 2. 触发设计门的证据

### 2.1 可通过样本

`tilt_roll_pos_2deg seed1` 完整通过：

- `TOUCHDOWN_HOLD = 11.000593 s`；
- 水平 RMSE/max `0.036962/0.082555 m`；
- 法向触地相对速度 `-0.004570 m/s`；
- 滑移 `0.076707 m`；
- 最大 roll/pitch `2.135046°/0.175166°`；
- detach/secondary/recovery/NAV_LAND/Disarm 均为 0。

这说明现有视觉、最终下降、触地确认和 hold 链路在部分条件下可工作，但不能证明策略可重复安全。

### 2.2 稳定姿态但滑移硬门失败

`tilt_roll_pos_2deg seed2` 独立重跑：

- 完整进入 `TOUCHDOWN_HOLD` 并保持 `11.000469 s`；
- 最大 roll/pitch 仅 `2.869533°/0.189767°`；
- 无 detach、secondary contact 或 recovery；
- 触地后切向滑移 `0.106767 m`，超过冻结门 `0.10 m`。

因此即使没有倾覆，水平机体策略也不能稳定满足切向接触约束。

### 2.3 灾难性姿态发散、离板与恢复

同一 seed 的已归档诊断轮：

- 首次 `TOUCHDOWN_HOLD` 后 `5.761910 s` 姿态超过 `10°`；
- `6.782327 s` 超过 `45°`；
- 最大 roll/pitch `60.967996°/55.439155°`；
- PX4 `rotational_movement` 在 hold 后 `6.202038 s` 首次为真；
- 最长视觉丢失发生在姿态已经超过 `45°` 后，不是初始原因；
- 滑移 `0.676615 m`，detach/recovery `1/1`；
- `NAV_LAND / Disarm = 0 / 0`。

这排除了“仅 evaluator 误报”“仅短时视觉丢失”或“只需多跑几次”作为充分解释。

## 3. 已排除的临时处理

本阶段明确没有：

- 放宽 `0.10 m` 滑移门；
- 放宽 `10°` 姿态门或 `2°` 发散增量门；
- 忽略离板、恢复或二次触地；
- 调整 P6B MarkerSelector、close-range 相机、FinalDescentController；
- 调整 P8A TouchdownDetector 或 TouchdownHoldController；
- 调整 P8B MPC、landing window 或 `TERMINAL_PHASE_P47`；
- 使用 Ground Truth 控制；
- 启用 `NAV_LAND` 或自动 Disarm；
- 为获得 PASS 反复重跑并挑选有利 Bag。

## 4. 根因判断

当前证据支持以下判断：

1. 固定倾斜甲板上的水平机体接触会产生不对称支撑与切向力。
2. 现有 hold 只维持位置/相对高度，不主动约束机体姿态相对甲板法向。
3. 在部分随机初始状态和接触瞬态下，该不对称可累积为滑移；更严重时形成滚转发散、PX4 movement bits、视觉丢失和恢复。
4. 视觉法向估计总体可用；失败轮中的视觉丢失晚于明显姿态发散，不能把根因归结为视觉先失效。
5. 因此下一步必须在“受限姿态对齐”与“重新设计被动接触/hold”之间进行独立架构决策。

## 5. 候选方案

### 5.1 方案 B：受限甲板法向姿态对齐

在最终接触前和 hold 中，将估计甲板法向转换为受限 roll/pitch 参考，但必须：

- 只在固定正 `+2°` 场景和显式白名单内启用；
- 设置姿态、角速度、角加速度和变化率限制；
- 法向估计过期或不一致时平滑回到水平机体，不允许跳变；
- 不改变水平相对位置控制、触地检测和 Ground Truth 隔离；
- 对 terminal handoff、PX4 attitude setpoint 与现有 position setpoint 的接口进行独立设计；
- 先在无接触安全高度验证，再做 `0.50 m` 下降，最后真实触地。

优点：直接处理接触几何不匹配。风险：新增姿态控制接口、坐标变换和 handoff，可能影响相机视野、推力分配和 PX4 控制模式。

### 5.2 方案 A2：保持水平机体，重构接触与 hold

可能包括：

- 触地后降低水平位置控制刚度或使用受限切向阻尼；
- 根据四滑橇接触顺序设计单侧接触过渡；
- 重新建模 hold 的法向/切向目标和接触约束；
- 增加独立姿态发散保护和立即中止策略。

优点：不引入甲板法向姿态 setpoint。风险：不能消除几何不匹配，可能只能缓解滑移；灾难性 roll 发散证据表明单纯调 hold 增益未必充分。

### 5.3 方案 C：主动起落架或顺应结构

不属于当前传统软件基线范围，不在近期实现。

## 6. 推荐决策

优先研究并原型验证方案 B，同时保留 A2 作为对照和安全保护层。理由：固定 `2°` 虽小，但已经出现可复现滑移硬门失败和一次灾难性姿态发散；只靠重复运行或轻微 hold 调参不足以证明物理安全。

该推荐不等于立即开放姿态对齐。必须先完成下述 TDD 和分级验收。

## 7. 实现前冻结项

必须先冻结：

- 甲板法向到机体 roll/pitch 的 NED/FRD 符号和旋转顺序；
- 姿态参考滤波、限幅和 slew-rate；
- 法向估计有效性、Marker 切换和视觉丢失退化；
- position/velocity setpoint 与 attitude setpoint 的唯一控制接口；
- `FINAL_DESCENT`、candidate、hold 各阶段何时允许姿态对齐；
- 触地后姿态发散的实时保护阈值；
- 白名单、默认关闭和故障回退；
- 旧 P6B/P8A/P8B 语义不可改变。

## 8. 必需 TDD

至少覆盖：

1. 水平法向输出零姿态。
2. `+2° roll`、`+2° pitch` 符号正确。
3. 负倾角和动态场景仍拒绝。
4. 姿态、角速度、变化率限幅。
5. 法向过期、NaN/Inf、非法四元数回退。
6. Marker 切换不产生姿态跳变。
7. final descent 前禁止姿态对齐。
8. candidate/hold handoff 连续。
9. position 与 attitude 控制接口互斥且可验证。
10. 恢复和 ABORT 不保留倾斜 setpoint。
11. Ground Truth 不进入生产控制。
12. P6B、P8A、P8B 全量回归。

## 9. 分级实验顺序

任何一级失败都停止后续：

1. 纯数学与消息级测试。
2. 固定 `+2° roll/pitch` 安全高度各 3 seeds。
3. 固定 `+2° roll/pitch` `0.50 m` 安全下降各 3 seeds。
4. 固定 `+2° roll` 单轮真实触地。
5. roll 3/3。
6. pitch 单轮和 3/3。
7. static 3/3、constant02 3/3、H1/H2 回归。
8. 只有固定 T1 全部通过后，才可讨论负倾角或动态 roll/pitch。

继续使用 P8C-3 已冻结的接触、滑移、姿态、离板、二次接触、恢复、NAV_LAND 和 Disarm 硬门。

## 10. 设计门关闭条件与结果

以下条件已全部满足，因此 P8C-3 正倾角批量验收已在 P8C-4 中恢复并完成：

- 独立方案被明确选定并形成代码边界；
- 新功能默认关闭且只对白名单显式启用；
- 新 TDD 和全工作区测试零失败零跳过；
- P6B/P8A/P8B 旧 evaluator 回归通过；
- 首个 roll 真实触地无明显单侧冲击、持续滑移、离板、二次接触或姿态发散；
- 不使用 Ground Truth 控制，不发送 NAV_LAND，不自动 Disarm。

最终执行结果：

```text
shadow safe altitude: 6/6 PASS
shadow safe descent: 6/6 PASS
active rehearsal: 6/6 PASS
active touchdown: roll 3/3 + pitch 3/3 = 6/6 PASS
legacy regressions: 9/9 PASS
full workspace: 340 tests, 0 failures, 0 skipped
```

最差 active touchdown 指标为 tracking P95 `0.238131°`、slip `0.059209 m`、HOLD 速度 P95 `0.032226 m/s`、姿态发散增量 `1.908267°`；fallback、离板、二次接触和 recovery 均为 0。

## 11. 证据路径

```text
results/p8c3_validation_20260802/p8c3_final_summary.json
results/p8c3_validation_20260802/p8c3_final_summary.txt
results/p8c3_validation_20260802/tilt_roll_pos_2deg_seed1/
results/p8c3_validation_20260802/tilt_roll_pos_2deg_seed2/
results/p8c3_validation_20260802/tilt_roll_pos_2deg_seed2_failed_attempt01/
results/p8c4_validation_20260802/p8c4_final_summary.json
results/p8c4_validation_20260802/p8c4_final_summary.txt
```

原 P8C-3 失败目录继续作为触发本设计门的历史证据；最终 P8C-4 成功目录没有覆盖或删除它们。
