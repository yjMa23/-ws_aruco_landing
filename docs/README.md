# 项目文档索引

本目录集中保存 `ws_aruco_landing` 的操作指南、系统参考、阶段计划、研究综述和验收记录。

当前开发基线已完成 P8A 升沉甲板触地、P8B 水平相对 MPC，以及 P8C T1 固定 `±2°` 几何、安全高度、安全下降和正 `+2° roll/pitch` 真实触地验收。P8C 状态为 `RESEARCH PASS / PLAN PASS / P8C-0 IMPLEMENTATION PASS / P8C-1 VALIDATION PASS / P8C-2 SAFE DESCENT PASS / P8C-4 VALIDATION PASS / P8C T1 VALIDATION PASS / P8C-3 DESIGN GATE CLOSED`；最终 active touchdown 为 roll `3/3`、pitch `3/3`，旧路径回归 `9/9`。P9 自动化实现与测试已通过，全工作区 `340 tests, 0 failures, 0 skipped`；27 轮 smoke 已完成 `20/27`，baseline 20+20 正在进行，正式消融冻结为 `60 executable + 30 NOT_APPLICABLE`。P8C-3 和 P9 smoke 的失败证据继续完整保留。

## 阅读入口

- [安装、构建、启动、实验与故障排查](guides/OPERATIONS.md)
- [系统架构与接口总览](reference/SYSTEM_OVERVIEW.md)
- [当前完整开发计划](plans/NEXT_DEVELOPMENT_PLAN.md)
- [P8 高级降落路线图](plans/P8_ADVANCED_LANDING_ROADMAP.md)
- [P8B 最新验收记录](validation/P8B_RELATIVE_MPC_VALIDATION.md)
- [P8C T1 完整验收记录](validation/P8C_TILTED_DECK_LANDING_VALIDATION.md)
- [P8C 倾斜甲板综述与几何模型](research/P8C_TILTED_DECK_LANDING_REVIEW.md)
- [P8C 独立执行计划](plans/P8C_TILTED_DECK_LANDING_PLAN.md)
- [P8C-4 终端接触稳定化研究与接口冻结](research/P8C4_TERMINAL_CONTACT_STABILIZATION_REVIEW.md)
- [P8C-4 终端接触稳定化执行计划](plans/P8C4_TERMINAL_CONTACT_STABILIZATION_PLAN.md)
- [P9 统一批量评测计划](plans/P9_UNIFIED_EVALUATION_PLAN.md)
- [P9 统一批量评测验收记录](validation/P9_UNIFIED_EVALUATION_VALIDATION.md)

## 操作指南

- [安装、构建、启动与实验操作指南](guides/OPERATIONS.md)

## 系统参考

- [系统架构与接口总览](reference/SYSTEM_OVERVIEW.md)
- [坐标系与变换契约](reference/COORDINATE_FRAMES.md)
- [ArUco 检测公式](reference/aruco_detector_formulas.md)
- [PX4 ArUco 静态降落控制公式](reference/px4_aruco_landing_formulas.md)

## 阶段计划

### 总体路线

- [传统基线实施计划](plans/TRADITIONAL_BASELINE_PLAN.md)
- [下一阶段完整开发计划](plans/NEXT_DEVELOPMENT_PLAN.md)
- [P8 高级移动甲板降落路线图](plans/P8_ADVANCED_LANDING_ROADMAP.md)

### 分阶段计划

- [P3 视觉状态估计与预测](plans/P3_VISUAL_STATE_ESTIMATION_PLAN.md)
- [P4 移动目标跟踪](plans/P4_MOVING_TARGET_TRACKING_PLAN.md)
- [P4.5 实验复现与时间对齐](plans/P4_5_EXECUTION_PLAN.md)
- [P4.6 正弦跟踪调参](plans/P4_6_SINUSOIDAL_TUNING_PLAN.md)
- [P4.7 自适应增益调度](plans/P4_7_ADAPTIVE_GAIN_SCHEDULING_PLAN.md)
- [P5A 动态甲板与着陆窗口](plans/P5A_DECK_DYNAMICS_AND_LANDING_WINDOW_PLAN.md)
- [P5B 相对高度下降](plans/P5B_RELATIVE_DESCENT_PLAN.md)
- [P5C 垂直状态估计](plans/P5C_VERTICAL_STATE_ESTIMATION_PLAN.md)
- [P6A 多源触地确认](plans/P6_TOUCHDOWN_CONFIRMATION_PLAN.md)
- [P6B 最终下降与触地](plans/P6B_FINAL_DESCENT_AND_TOUCHDOWN_PLAN.md)
- [P6B 近距多尺度视觉](plans/P6B_CLOSE_RANGE_VISUAL_PLAN.md)
- [P7 批量评测](plans/P7_BATCH_EVALUATION_PLAN.md)
- [P8A 升沉甲板触地](plans/P8A_HEAVE_TOUCHDOWN_PLAN.md)
- [P8B 水平相对 MPC](plans/P8B_RELATIVE_MPC_PLAN.md)
- [P8C 固定倾斜与低频 roll/pitch 甲板降落](plans/P8C_TILTED_DECK_LANDING_PLAN.md)
- [P9 统一批量评测、消融和论文实验](plans/P9_UNIFIED_EVALUATION_PLAN.md)

## 研究综述

- [P8B 水平相对 MPC 综述与论文级模型](research/P8B_RELATIVE_MPC_REVIEW.md)
- [P8C 固定倾斜与低频 roll/pitch 甲板综述与几何模型](research/P8C_TILTED_DECK_LANDING_REVIEW.md)

## 验收记录

- [P1 水平移动甲板仿真](validation/P1_MOVING_DECK_VALIDATION.md)
- [P2B 船舶 GNSS 仿真](validation/P2B_DECK_GNSS_VALIDATION.md)
- [P2C GNSS 会合](validation/P2C_GNSS_RENDEZVOUS_VALIDATION.md)
- [P2D GNSS—视觉接管](validation/P2D_GNSS_VISION_HANDOVER_VALIDATION.md)
- [P3 视觉状态估计](validation/P3_VISUAL_STATE_ESTIMATION_VALIDATION.md)
- [P4 移动目标跟踪](validation/P4_MOVING_TARGET_TRACKING_VALIDATION.md)
- [P4.5 时间对齐](validation/P4_5_TIME_ALIGNMENT_VALIDATION.md)
- [P4.6 正弦跟踪调参](validation/P4_6_SINUSOIDAL_TUNING_VALIDATION.md)
- [P4.7 自适应增益调度](validation/P4_7_ADAPTIVE_GAIN_SCHEDULING_VALIDATION.md)
- [P5A 动态甲板与着陆窗口](validation/P5A_DECK_DYNAMICS_AND_LANDING_WINDOW_VALIDATION.md)
- [P5B 相对高度下降](validation/P5B_RELATIVE_DESCENT_VALIDATION.md)
- [P5C 垂直状态估计](validation/P5C_VERTICAL_STATE_ESTIMATION_VALIDATION.md)
- [P6A 多源触地确认](validation/P6_TOUCHDOWN_CONFIRMATION_VALIDATION.md)
- [P6B 近距视觉](validation/P6B_CLOSE_RANGE_VISUAL_VALIDATION.md)
- [P6B 最终下降与触地](validation/P6B_FINAL_DESCENT_AND_TOUCHDOWN_VALIDATION.md)
- [P8A 升沉甲板触地](validation/P8A_HEAVE_TOUCHDOWN_VALIDATION.md)
- [P8B 水平相对 MPC](validation/P8B_RELATIVE_MPC_VALIDATION.md)
- [P8C T1 固定 2° 倾斜甲板完整验收：P8C-4 PASS，设计门关闭](validation/P8C_TILTED_DECK_LANDING_VALIDATION.md)
- [P8C-3 姿态对齐与接触稳定性独立设计门：历史证据与关闭结论](plans/P8C3_ATTITUDE_ALIGNMENT_DECISION_GATE.md)
- [P9 统一批量评测第一版](validation/P9_UNIFIED_EVALUATION_VALIDATION.md)

## 归档约定

- `guides/`：安装、构建、启动、实验和故障排查。
- `reference/`：系统架构、接口、坐标系和公式。
- `plans/`：总体路线、阶段执行计划和 roadmap。
- `research/`：需要论文级建模或方案选择的研究综述。
- `validation/`：已经执行的测试、SITL 数据和 PASS/FAIL 结论。
- 新阶段继续沿用 `<阶段>_<主题>_PLAN.md`、`<阶段>_<主题>_REVIEW.md` 和 `<阶段>_<主题>_VALIDATION.md` 命名。
