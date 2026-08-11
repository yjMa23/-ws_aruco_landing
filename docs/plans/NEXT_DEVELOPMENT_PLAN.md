# 下一步计划：动态甲板未来 Twist 可观测性

## 当前状态

甲板 6-DoF shadow 已按相对方案实现：当前位姿为
`uav_centered_ned` 中的 `deck-uav`，twist 为甲板自身 NED twist；预测轨迹
相对每条消息发布时冻结的 `uav_origin_ned`。实现、标准消息接口、测试、
evaluator 和约 `5 m` 的 `static/rollpitch/combined/rigid_body_motion × seed 1/2/3`
正式矩阵已完成。

冻结结果为：

- 安全隔离、时间同步、有限性和有效覆盖率均为 `12/12`。
- 当前法向与 `0.5 s` 水平/垂直位置、法向门均为 `12/12`，yaw 为 `11/12`。
- `0.5 s` 水平速度、垂直速度和角速度门分别为 `6/12`、`2/12`、`3/12`。
- 全部硬门总计 `2/12`；因此不得开始 acados NMPC、动态姿态下降或真实接触设计。

本轮未触发约 `3 m` 对照：约 `5 m` 的当前位姿、法向和未来位置门已全部
通过，失败集中在二阶导数驱动的未来 twist，现有证据不支持把它归因为单纯像素
分辨率问题。

Marine 已把显式 `--environment marine` 切换到 `vrx_wamv_landing/vessel_body → fixed landing_deck`，并启用与 vessel dynamics 完全解耦的 VRX `WaveVisual` 动态 visual-only ocean；legacy 仍为默认。WAM-V canonical frame 到 landing deck 的固定 offset 为 `[0,0,1.8] m`；`MotionProfile` 仍是唯一 vessel motion source。该场景升级只改变渲染，不改变 shadow 的观测契约，也不能作为跳过 Future Twist 诊断的理由。

## 下一目标

先完成未来 twist 观测契约评审，不直接继续调 CA 带宽：

1. 使用现有正式 Bag 做严格因果回放，分解 ArUco 相对位姿、PX4 无人机速度、
   局部常加速度拟合和 SO(3) 角导数对 `0.5 s` twist 误差的贡献。Ground Truth
   只允许评分，不得用于在线参数、相位或频率拟合。
2. 若 ArUco-only 因果估计在跨 seed 冻结参数下仍无法通过，先单独评审最小额外
   观测契约：甲板 IMU 只提供线/角加速度，船舶 GNSS velocity 只作远距离绝对锚。
   该方案会改变“近距离 ArUco 主导”边界，未经批准不实现。
3. 任何新模型必须先在 `static/combined seed1` 上用预注册参数通过全部旧硬门，
   再从头运行新的 `4 × 3` 正式矩阵；失败 seed 不替换，门限不放宽。

## 后续边界

只有约 `5 m` 的新正式矩阵 `12/12` 全部通过，下一份计划才允许设计
acados NMPC。首次主动动态下降最多到 `0.50 m` 保持，真实接触仍需独立计划。
本阶段不修改现有水平相对 MPC，不开放动态姿态下降、`NAV_LAND` 或自动 Disarm。

动态 Gerstner rendering 已作为纯视觉能力完成，不再属于后续算法计划。Future Twist 因果可观测性诊断完成后，如确有论文或实验需求，再单独评审真正的 sea-state → vessel-response 链，包括 JONSWAP/PM sea state、RAO-based vessel response、浮力、水动力、洋流和风载；这些动力学能力当前仍未实现，不能与 visual-only WaveVisual 混为一谈。
