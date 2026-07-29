# P4.6 XY 正弦运动跟踪参数优化计划

## 1. 阶段目标

在不修改状态估计器结构、不引入常加速度模型、不实现 P5 下降的前提下，通过可复现的
PX4 SITL 参数扫描，降低 XY 正弦甲板换向阶段的水平跟踪误差。

P4.5 基准结果：

```text
水平位置 RMSE = 0.4812 m
相对速度 RMSE = 0.5086 m/s
最大水平误差 = 0.7367 m
预测位置 RMSE = 0.1873 m
```

第一阶段目标：

```text
水平位置 RMSE < 0.40 m
最大水平误差 < 0.65 m
```

最终建议目标：

```text
水平位置 RMSE < 0.30 m
最大水平误差 < 0.45 m
无 Marker 丢失
无 GNSS 恢复
不触发持续控制输入不可用
```

## 2. 明确不做

本阶段不实现：

- 着陆窗口或下降；
- 常加速度 Kalman Filter；
- α-β-γ 滤波；
- MPC；
- 强化学习；
- 在线自适应增益；
- 针对单个 rosbag 的离线控制重放替代真实 SITL。

## 3. 实验参数化入口

修改：

```text
src/aruco_precision_landing_cpp/launch/px4_aruco_landing.launch.py
scripts/start_sitl.sh
```

新增 launch / 脚本参数：

```text
tracking_mode
prediction_horizon_s
velocity_feedforward_gain
relative_velocity_gain
```

要求：

- 默认值必须与当前 YAML 完全一致；
- 参数只覆盖对应 ROS 参数，不复制整份控制器 YAML；
- rosbag 目录名必须包含模式和参数缩写，保证实验可追溯；
- 参数非法时启动前失败；
- 不改变静止、0.2 m/s、0.4 m/s 默认命令行为。

## 4. 扫描顺序

### Stage A：预测时域扫描

固定：

```text
tracking_mode = PREDICTED_POSITION_VELOCITY_FF
velocity_feedforward_gain = 1.0
relative_velocity_gain = 0.25
```

扫描：

```text
additional_prediction_horizon_s = 0.00 / 0.05 / 0.10
```

其中 `0.10 s` 使用 P4.5 已有基准 bag，不重复运行；本轮新增运行 `0.00 s` 和 `0.05 s`。

选择规则：

1. 优先最小水平位置 RMSE；
2. 若差异小于 0.02 m，优先最大误差更小者；
3. 不接受引入丢标、GNSS 恢复或明显动力学峰值恶化的组合。

### Stage B：模式对比

使用 Stage A 最佳时域，与：

```text
ESTIMATED_POSITION_VELOCITY_FF
```

进行一次真实 PX4 SITL 对比，判断额外位置预测是否确有收益。

### Stage C：小范围前馈增益扫描

仅当 Stage A/B 仍无法达到 `0.40 m` RMSE 时执行：

```text
velocity_feedforward_gain = 0.8 / 1.0
relative_velocity_gain = 0.10 / 0.25
```

不进行全排列大网格；先根据 Stage A/B 结果选择两个最有信息量的组合。

## 5. 每轮验收

每轮至少稳定运行 80 秒，并执行：

```bash
python3 scripts/evaluate_p4_bag.py bags/<bag_name>
```

记录：

- 稳定时长；
- 水平位置 RMSE；
- 相对速度 RMSE；
- 最大水平误差；
- 预测位置 RMSE；
- 最大水平速度；
- 最大水平加速度；
- 最大滚转和俯仰角；
- Marker 丢失次数；
- GNSS 恢复次数；
- 时间同步、位姿历史和控制输入告警。

## 6. 输出

新增：

```text
docs/P4_6_SINUSOIDAL_TUNING_VALIDATION.md
```

内容包括：

- 每轮参数和对应 rosbag；
- 统一指标表；
- 选定默认参数；
- 与 P4.5 基准的改善百分比；
- 是否达到进入 P5 的动态跟踪门槛；
- 若未达到，是否需要升级运动模型的结论。

## 7. 执行状态（2026-07-24）

- 参数覆盖和可追溯 rosbag 命名入口已完成。
- Stage A 预测时域 `0.00 / 0.05 / 0.10 s` 扫描已完成。
- Stage B 估计位置前馈与零额外预测模式对比已完成。
- Stage C 相对速度增益 `0.10 / 0.25 / 0.40 / 0.60 / 0.80 / 1.00` 扫描已完成。
- 最佳正弦结果为 `ESTIMATED_POSITION_VELOCITY_FF + rvg=1.00`，位置 RMSE `0.3439 m`。
- 第一阶段 `RMSE < 0.40 m` 目标已完成，最终 `RMSE < 0.30 m` 目标未完成。
- 0.4 m/s 匀速交叉验证表明高阻尼会恶化稳态跟踪，因此不修改全局默认参数。
- 本阶段停止继续单参数扫描，详细结论见 `docs/P4_6_SINUSOIDAL_TUNING_VALIDATION.md`。
