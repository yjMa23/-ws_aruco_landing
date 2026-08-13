# 下一步计划：定位 Planar Board static 法向误差

## 当前阻塞

Marine Planar Board 正式 4×3 的安全门已通过 `12/12`，但 Board 门仅 `9/12`。
失败严格集中在 `static seed 1/2/3` 的当前法向：

```text
RMSE = 1.186–1.421°  > 1.0°
P95  = 2.010–2.455°  > 1.5°
```

同三轮的水平/垂直位置、覆盖、来源路由、重投影 RMSE 和 raw normal flip 均通过。
正式 Bag 固定在 `results/deck_motion_shadow_planar_marine_local7m_20260813`，不得替换
seed、删除失败轮次或放宽门限。

## 下一目标：离线定位 static 法向误差来源

只使用现有 12 个正式 Bag，按同一套固定分析方法完成：

1. 分别计算 raw Planar pose 和 shadow current pose 的法向误差，区分误差来自 detector
   还是 shadow 滤波/时间对齐。
2. 在 static 与 9 个动态轮次间比较法向误差与 Marker 数、图像位置、Board
   reprojection RMSE、pose source、相机—甲板距离和观测时序的关系。
3. 检查 IPPE 候选选择、角点噪声、相机标定和位姿采样时刻是否能解释 static 的稳定
   偏差；Ground Truth 只能在 pose 输出后离线评分，禁止用于候选选择、调参输入或控制。
4. 形成唯一可证伪的根因结论和最小修复方案。若需要新模型或算法，先补理论文档和
   构建说明，再编码；若证据不足，明确记录缺失观测，不猜测修复。

验收标准：分析脚本或测试能够在固定 Bag 上复现 3 个 static 失败，并给出 detector、
时间对齐和 shadow 三段的误差分解。任何修复必须使用预先固定的一套参数重新运行完整
Marine 4×3；只有安全门与 Board 门都达到 `12/12`，才能冻结新结果。

## 暂缓：Future Twist 因果诊断

当前 `0.5 s` 预测法向、水平速度、垂直速度、角速度门分别为
`0/12`、`0/12`、`3/12`、`0/12`。在 Board 门达到 `12/12` 前不分析或调优这些项，
也不修改控制器。

Board 验收完成后，另行使用严格因果回放：时刻 `t` 的输出只能读取 `t` 及以前的
ArUco、PX4 速度和估计器历史，Ground Truth 只在输出后评分。若跨 seed 固定参数仍
失败，再评审甲板 IMU 或船舶 GNSS velocity 等最小附加观测；未经新计划不实现。

## 安全边界

本计划不开放动态姿态下降、真实接触、NMPC、`NAV_LAND`、自动 Disarm、波浪驱动
船体动力学或 Ground Truth 控制输入。GUI 外观检查等待有显示环境时补做，不阻塞离线
根因分析。
