# P5C 低高度垂直状态估计与标定执行计划

## 1. 阶段目标

在 P5B 已能够安全下降并保持在 `0.50 m` 测试高度的基础上，降低低高度相对高度估计的偏差、延迟和抖动，为后续 P6 触地检测提供可靠输入。

本阶段重点解决：

```text
Marker z 测量
→ 图像采样时刻无人机 z
→ 甲板垂直位置/速度估计
→ 相对高度与相对垂直速度
→ 低高度标定与安全门限
```

阶段完成后仍不触地、不发送 `NAV_LAND`、不 Disarm，最低测试高度保持 `0.50 m`。

---

## 2. 明确不做

P5C 不实现：

- `0.50 m` 以下下降；
- 触地检测和触地确认；
- 自动 Land；
- 自动 Disarm；
- MPC 或强化学习；
- Ground Truth 进入控制器；
- 依赖预先已知甲板运动轨迹；
- 修改 P4.7 水平跟踪控制律。

---

## 3. 当前问题与验收依据

P5B SITL 已确认：

- 静止甲板 Ground Truth 最低相对高度约 `0.68 m`，参考为 `0.50 m`；
- 0.4 m/s 匀速甲板 Ground Truth 最低相对高度约 `0.71 m`；
- 升沉甲板 Ground Truth 最低相对高度约 `0.53 m`；
- 视觉估计相对高度在低高度可能低至约 `0.30 m`；
- 低高度估计误差约 `0.20～0.30 m`，尚不足以直接用于触地判断。

P5C 必须先量化以下误差来源：

1. Marker z 原始测量偏差；
2. 三维常速度 Kalman Filter 的 z 轴滤波延迟；
3. 图像采样时间到 PX4 位姿插值误差；
4. 相机外参 z 偏差；
5. 甲板升沉速度未进入垂直控制前馈造成的跟踪滞后；
6. 低高度透视、姿态和 PnP 抖动造成的测量噪声变化。

---

## 4. Task 1：新增 P5C 离线标定脚本

新增：

```text
scripts/evaluate_p5c_vertical_estimation.py
```

输入：

- P5B rosbag；
- `/landing/marker_pose_ned`；
- `/landing/estimated_deck_odometry`；
- `/landing/predicted_deck_pose`；
- `/landing/relative_height`；
- `/landing/relative_height_reference`；
- `/fmu/out/vehicle_local_position_v1`；
- `/simulation/deck/ground_truth`。

输出至少包含：

- Marker deck z 对 Ground Truth deck z 的偏差、RMSE、标准差；
- 估计 deck z 对 Ground Truth deck z 的偏差、RMSE、标准差；
- 预测 deck z 对 Ground Truth deck z 的偏差、RMSE、标准差；
- 估计相对高度对 Ground Truth 相对高度的偏差、RMSE、P95 和最大误差；
- 相对垂直速度误差；
- 按真实相对高度分箱统计：
  - `> 2.0 m`
  - `1.0～2.0 m`
  - `0.5～1.0 m`
- 最佳常量 z 偏置估计；
- 通过互相关估计 Marker z、估计 z 和 Ground Truth z 的时延；
- 非法输入和缺少话题时明确失败。

Ground Truth 只在脚本中使用。

---

## 5. Task 2：独立垂直状态估计模块

新增：

```text
src/aruco_precision_landing_cpp/include/aruco_precision_landing_cpp/vertical_state_estimator.hpp
src/aruco_precision_landing_cpp/src/vertical_state_estimator.cpp
src/aruco_precision_landing_cpp/test/vertical_state_estimator_test.cpp
```

输入：

```text
measurement_z_ned
sample_time_s
```

输出：

```text
deck_z_ned
deck_vertical_velocity_ned_mps
position_variance
velocity_variance
sample_time_s
```

第一版采用独立二状态常速度 Kalman Filter：

```text
x = [z, vz]
```

设计要求：

- z 轴过程噪声、测量噪声和初始协方差独立配置；
- 不继续共用水平 x/y 的滤波参数；
- 支持测量偏置参数；
- 使用视觉采样时间计算 `dt`；
- 拒绝重复、倒退、NaN、Inf 和过大时间间隔；
- 支持创新门限和长间隔重初始化；
- reset 后状态确定；
- 不依赖 ROS、PX4 或 Ground Truth。

单元测试至少覆盖：

- 静止收敛；
- 匀速升沉速度估计；
- 常量偏置补偿；
- 噪声抑制；
- 时间异常；
- 离群点拒绝；
- 长间隔重初始化；
- reset；
- 非法参数。

---

## 6. Task 3：垂直预测与相对高度状态

垂直预测采用：

```text
predicted_deck_z = estimated_deck_z + estimated_deck_vz * prediction_horizon
relative_height = predicted_deck_z - uav_z
relative_vertical_velocity = estimated_deck_vz - uav_vz
```

要求：

- 预测时域受上限约束；
- 使用 PX4 `VehicleLocalPosition.vz` 计算相对垂直速度；
- 新增调试话题：

```text
/landing/vertical_state
/landing/relative_vertical_velocity
/landing/raw_relative_height
```

- 原 P5B `/landing/relative_height` 改为使用独立垂直估计器输出；
- 保留旧三维估计器供水平跟踪使用，避免一次性重构 P4；
- P4.7 水平结果不得变化。

---

## 7. Task 4：甲板垂直速度前馈

P5C 第一版只在 P5B 相对高度控制中加入甲板垂直速度前馈：

```text
position_target_z = predicted_deck_z - height_reference
velocity_target_z = estimated_deck_vz + height_reference_rate_ned
```

其中下降时 NED z 速度为正：

```text
height_reference_rate_ned = descent_rate_mps
```

保持或暂停时：

```text
height_reference_rate_ned = 0
```

要求：

- 垂直速度前馈有独立增益和限幅；
- 默认先关闭，通过 SITL 对照后再决定是否设为默认；
- 不改变水平速度前馈；
- 无有效垂直估计时不发布 z 速度前馈；
- 视觉失效时继续执行 P5B 暂停/恢复策略。

---

## 8. Task 5：低高度标定实验

使用 P5B 保持模式分别采集：

```text
2.0 m
1.0 m
0.5 m
```

场景：

```text
static
heave
```

每个高度至少保持 `20 s` 稳定数据。

对照组：

1. 原三维估计器 z；
2. 独立垂直估计器；
3. 独立垂直估计器 + z 偏置；
4. 独立垂直估计器 + 甲板垂直速度前馈。

---

## 9. 阶段验收标准

P5C 完成至少满足：

- 静止甲板 `0.5～1.0 m` 相对高度 RMSE ≤ `0.10 m`；
- 升沉甲板 `0.5～1.0 m` 相对高度 RMSE ≤ `0.15 m`；
- 相对高度绝对偏差 P95 ≤ `0.20 m`；
- 垂直速度估计方向正确且无持续发散；
- z 目标最大单步变化 ≤ `0.05 m`；
- 不低于 `0.45 m` Ground Truth 相对高度；
- 不发送 `NAV_LAND` 或 Disarm；
- 不进入旧下降状态；
- P4.7 水平跟踪测试和 P5B 安全测试全部回归；
- 全工作区测试通过。

若无法达到上述误差门槛，必须在文档中明确限制，不进入 P6 触地。

---

## 10. 执行顺序

```text
保存本计划
→ 用现有 P5B bags 建立垂直误差基线
→ 实现纯 C++ VerticalStateEstimator
→ 单元测试
→ 节点默认关闭接入
→ 2.0 / 1.0 / 0.5 m 静止标定
→ 升沉标定
→ 评估 z 速度前馈
→ 冻结 P5C 验收结果
```

## 11. 当前执行状态

- 计划已保存。
- 已新增 `scripts/evaluate_p5c_vertical_estimation.py` 并完成静止、匀速和升沉旧 bag 基线分析。
- 旧外参下 Marker、估计和预测 deck z 均存在约 `-0.22～-0.24 m` 的稳定偏差；升沉时延约 `0.05 s`。
- Gazebo 实际模型查询结果为 `base_link z=0.24 m`、`camera_link z=0.10 m`，相机相对机体参考点位于 FRD Down `0.14 m`；旧配置 `-0.10 m` 与真实模型相差 `0.24 m`。
- 已将 `camera_extrinsic.translation_frd_m` 修正为 `[0.0, 0.0, 0.14]`。
- 修正外参后，静止 `5 m` 相对高度 RMSE 为 `0.0477 m`，升沉 `5 m` RMSE 为 `0.0498 m`。
- 已实现独立 `VerticalStateEstimator`，11 项模块测试通过；并行接入后全包累计 `166 tests` 通过，尚未接管控制。
- 升沉安全高度并行结果：新垂直 deck z RMSE `0.0377 m`，旧预测 z RMSE `0.0544 m`；垂直速度 RMSE 暂未改善。
- 静止 `0.70 m` 标定中 Ground Truth 最低高度 `0.6767 m`，`0.5～1.0 m` 相对高度 RMSE `0.0174 m`。
- 残余深度误差随观测距离变化：约 `>2 m` 偏高 `0.028 m`、`1～2 m` 偏高 `0.014 m`、`0.5～1 m` 偏低 `0.006 m`，因此固定 `0.04 m` 偏置不适用，默认恢复为 `0`。
- 静止 `0.50 m` 标定已通过：Ground Truth 最低相对高度 `0.4654 m`，`0.5～1.0 m` 相对高度 RMSE `0.0136 m`，无 Land/Disarm。
- 升沉 `0.70 m`、无 z 速度前馈时，Ground Truth 最低高度仅 `0.5210 m`，实际参考跟踪 RMSE `0.1605 m`，因此禁止直接继续升沉 `0.50 m`。
- 已实现独立 z 速度前馈：`velocity_target_z = gain * estimated_deck_vz + height_reference_rate_ned`，并增加增益、限幅与消融入口。
- `gain=1.0` 在升沉 `0.70 m` 将实际参考跟踪 RMSE 降至 `0.0918 m`，最低高度提升至 `0.5521 m`；静止 `0.70 m` RMSE 为 `0.0258 m`。
- `gain=1.5` 反而使升沉 RMSE 恶化到 `0.1338 m`、最低高度降至 `0.5005 m`，停止继续提高增益。
- `gain=1.0` 已设为相对下降默认，`--disable-vertical-ff` 保留为消融入口。
- 独立垂直估计器未替换 P5B 的位置参考：低高度和升沉下降中其位置误差未稳定优于旧三维估计器；当前仅用于垂直速度前馈和并行评测。
- 下一步使用默认参数复验升沉 `0.70 m`，随后冻结 P5C 验收结论；在安全余量进一步提高前，不运行升沉 `0.50 m`。
