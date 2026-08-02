# P8C-4 终端接触稳定化研究与接口冻结

## 1. 结论

截至 2026-08-02，本阶段最终验证方案为：

```text
B1：位置模式内的受限终端推力方向整形
+
A2：状态化接触锚点、锚点中心顺应与姿态安全保护
+
T1 轴约束：roll 场景只保留 roll，pitch 场景只保留 pitch
+
候选/保持阶段的受限相对高度预压与向下加速度预压
+
TOUCHDOWN_HOLD 锁存 candidate 末端最后有效法向
```

该方案没有切换到直接姿态 Offboard；水平与垂直前馈仍通过 `TrajectorySetpoint` 进入 PX4 `mc_pos_control`。

拒绝在当前项目中采用 B2 直接 `VehicleAttitudeSetpoint`。生产接口继续保持：

```text
OffboardControlMode.position = true
OffboardControlMode.attitude = false
唯一姿态 setpoint 发布者 = PX4 mc_pos_control
companion 发布 = TrajectorySetpoint position/velocity/acceleration/yaw
```

P8C-4 不修改 PX4 核心控制器，不使用 Ground Truth 控制，不启用 `NAV_LAND` 或自动 Disarm。

## 2. 证据基线

P8C-3 已证明水平机体接触方案不可重复安全：

- roll seed1：滑移 `0.076707 m`，最大 roll/pitch `2.135°/0.175°`，PASS；
- roll seed2 独立重跑：保持稳定但滑移 `0.106767 m > 0.10 m`；
- roll seed2 灾难性归档轮：最大 roll/pitch `60.968°/55.439°`，滑移 `0.676615 m`，detach/recovery `1/1`；
- 姿态发散先于最长视觉丢失，不能把根因归结为视觉先失效。

因此不得继续随机重跑、挑 Bag 或放宽 evaluator 门。

## 3. 本地 PX4 与消息接口审计

审计对象：

```text
/home/j/PX4-Autopilot
/home/j/ws_sensor_combined/src/px4_msgs/msg
src/aruco_precision_landing_cpp/src/px4_aruco_landing_node.cpp
```

### 3.1 Offboard 优先级

本地 PX4 `src/modules/commander/ModeUtil/control_mode.cpp` 对 Offboard 控制级别使用严格 `if / else if`：

```text
position
else velocity
else acceleration
else attitude
else body_rate
...
```

所以 `position=true` 时，外部同时发布 `VehicleAttitudeSetpoint` 不能构成可靠混合控制接口。切换到 `attitude=true` 又意味着 companion 必须完整承担位置、速度、推力幅值、姿态映射、模式连续性、唯一发布者和 failsafe；当前项目没有这样一套完整外部控制器。

### 3.2 mc_pos_control 消费链

本地 `mc_pos_control`：

1. 订阅 `trajectory_setpoint`；
2. `PositionControl::setInputSetpoint()` 读取 position、velocity、acceleration、yaw；
3. 位置 P 输出与 velocity feedforward 合成；
4. 速度 PID 输出与 acceleration feedforward 合成；
5. `_accelerationControl()` 根据水平加速度生成目标 body-z；
6. `ControlMath::thrustToAttitude()` 生成并发布 `vehicle_attitude_setpoint`。

`TrajectorySetpoint6dof` 在本地源码中由 UUV 和 spacecraft 控制器消费，没有多旋翼位置控制消费者。因此不能用它在当前多旋翼链中实现位置+姿态混合。

### 3.3 thrust_body 语义

当前 `px4_msgs/VehicleAttitudeSetpoint.msg` 明确：多旋翼通常 `thrust_body[0:1]=0`，`thrust_body[2]` 为负的归一化油门需求。只给四元数而不提供正确推力幅值会丢失高度控制；这进一步否决了“仅切 attitude=true 并填四元数”的做法。

### 3.4 当前项目链

当前节点发布：

```text
position=true
velocity=false
acceleration=false
attitude=false
```

但 `TrajectorySetpoint` 中 position 始终有效，并按需填写 velocity/acceleration feedforward。P8B `RELATIVE_MPC` 在自由飞行阶段可输出水平 acceleration；进入 `FINAL_DESCENT / CANDIDATE / HOLD` 后保持既有 `TERMINAL_PHASE_P47` handoff。P8C-4 只在 handoff 后追加独立 terminal bias，不改变 P8B 控制器和 handoff 语义。

## 4. 方案比较

### 4.1 B1：位置模式内的终端推力方向整形

优点：

- 保留 PX4 位置/速度闭环、推力幅值和 failsafe；
- 不引入第二个姿态 setpoint 发布者；
- 可把 terminal bias 与 P8B/跟踪 acceleration feedforward 分开诊断、分别限幅后合成；
- 默认关闭、shadow-only 和白名单容易实现；
- 法向失效时可连续回零。

风险：

- 位置反馈和速度 PID 会部分抵消 acceleration bias；
- 自由飞行中 bias 会同时产生水平运动趋势；
- 接触后位置闭环可能对抗支撑面切向力。

对应措施：只在终端阶段渐入；真实接触后叠加 A2 顺应；先进行无接触 rehearsal 验证实际姿态方向和漂移。

### 4.2 B2：直接 VehicleAttitudeSetpoint

当前拒绝。缺失项包括：

- 完整外部位置控制；
- 完整外部速度控制；
- 推力幅值与重力补偿；
- 位置到姿态/推力映射；
- position→attitude 切换连续性；
- 唯一 setpoint 发布者；
- 独立 failsafe 与回退。

仅切换 `attitude=true` 并填写四元数不满足安全边界。

### 4.3 A2：接触后的水平顺应与 hold 重构

选作 B1 的接触层：

- candidate 首周期以当前名义目标建立甲板相对锚点，不发生目标跳变；
- 锚点随估计甲板位置移动；
- 小误差落入 deadband，让位置目标有限跟随无人机，避免强行对抗接触力；
- 超出顺应区时有限恢复；
- 使用相对水平速度阻尼抑制滑移；
- 目标变化率和最大让步距离独立限制；
- 甲板估计失效时保持最后安全目标；
- recovery/abort 清空锚点。

A2 单独不能消除几何不匹配，因此不作为唯一方案。

## 5. 数学冻结

### 5.1 坐标定义

- 世界坐标：PX4 local NED；
- 机体坐标：FRD；
- 甲板向上单位法向：`n_up^N = [n_N,n_E,n_D]^T`，有效时 `n_D < 0`；
- 期望机体 body-z（FRD Down）在 NED 中：

```text
b_z,d^N = -n_up^N
```

### 5.2 acceleration bias

本地 PX4 `_accelerationControl()` 在悬停附近使用：

```text
body_z ∝ [-a_N, -a_E, g]
```

要求 `body_z = -n_up`，得到：

```text
a_xy = -g * n_up,xy / n_up,D
```

水平甲板 `n_up=[0,0,-1]` 输出零 bias。`2°` 倾角量级：

```text
g tan(2°) = 0.3425 m/s²
```

符号冻结：

- `+2° roll` → East bias 为正；
- `-2° roll` → East bias 为负，但生产白名单拒绝负倾角；
- `+2° pitch` → North bias 为负。

### 5.3 yaw 保持

使用 PX4 `ControlMath::bodyzToAttitude()` 同构构造：

```text
y_C = [-sin(yaw), cos(yaw), 0]
body_x = y_C × body_z
body_y = body_z × body_x
R_NB = [body_x body_y body_z]
```

该构造在 yaw `0/90/180/-90°` 已由纯数学测试覆盖。

## 6. 最终验证参数冻结

以下为完成 TDD、失败根因修复和整级重跑后用于最终 6/6 真实触地的固定参数。所有 evaluator 硬门保持不变：

| 参数 | 值 | 依据 |
|---|---:|---|
| maximum target tilt | `2.5°` | 覆盖固定 2°，留 0.5°估计余量，远低于 10°硬门 |
| normal filter | 沿用独立 shadow `0.08` | P8C-0～P8C-3 已验证，不改 production deck attitude |
| normal freshness timeout | `0.20 s` | 与 landing-window 视觉年龄门一致 |
| short loss hold | `0.10 s` | 仅去抖，短于 freshness timeout |
| marker switch jump gate | `1.0°` | 2°目标的一半，拒绝不连续切换 |
| tilt slew rate | `4.0°/s` | 约 0.5 s 渐入 2° |
| acceleration bias limit | `0.45 m/s²` | 高于 `g tan(2°)=0.3425`，仍远低于现有 1.5 总加速度限幅 |
| bias slew rate | `0.80 m/s³` | 约 0.43 s 到 2°所需 bias |
| activation/deactivation | `0.50/0.30 s` | 首次启用无跳变、失效平滑回零 |
| total horizontal acceleration | `1.50 m/s²` | 不扩大 P8B 已冻结上限 |
| rehearsal duration | `1.0 s` | 受控脉冲后保留足够时间验证回零与漂移恢复 |
| compliance deadband | `0.015 m` | 大于估计微抖，远小于滑移门 |
| maximum allowance | `0.040 m` | 不直接使用 evaluator 0.10 m 上限 |
| compliant target rate | `0.10 m/s` | 与 HOLD 速度硬门同量级，避免目标快速激励接触 |
| anchor correction rate | `0.05 m/s` | 绝对视觉位置只做慢校正 |
| deck velocity deadband | `0.035 m/s` | 隔离静止甲板速度估计噪声，保留 0.2 m/s 移动传播 |
| velocity damping time | `0.12 s` | 非积分相对速度阻尼 |
| maximum damping offset | `0.020 m` | 限制瞬时阻尼目标变化 |
| preload relative height | `0.20 m` | candidate/HOLD 中重新加载滑橇，避免悬停卸载 |
| preload reference slew | `0.05 m/s` | 相对高度预压连续渐入 |
| downward preload acceleration | `1.0 m/s²` | 降低旋翼承重并提高滑橇法向载荷 |
| preload acceleration slew | `1.0 m/s³` | 约 1 s 渐入，不产生 z 向跳变 |
| attitude trigger/clear | `6°/4°` | 在 evaluator 10°硬失败前动作并留迟滞 |
| angular-rate trigger | `45°/s` | 在灾难性滚转累积前动作 |
| safety duration | `0.20 s` | 拒绝短时噪声，捕获持续发散 |

参数不得在看到真实实验结果后扩大。

## 7. 状态与失效冻结

正常生产：

```text
TRACK_TARGET / WAIT_LANDING_WINDOW / DESCEND / TEST_HEIGHT_HOLD: 生产路径不应用
FINAL_DESCENT: 仅在线几何进入近接触段后允许 B1；高空保持零输出
TOUCHDOWN_CANDIDATE_HOLD: B1 渐入 + 相对高度/向下加速度预压；冻结下降
TOUCHDOWN_HOLD: 锁存 candidate 末端主轴法向 + 锚点中心 A2 + 继续预压
RECOVER/ABORT: 清零 bias、预压、锚点和安全监视器
```

rehearsal：仅 `TEST_HEIGHT_HOLD`、固定正 `+2°`、SITL 启动白名单、显式参数、最长 `1.0 s` 脉冲；结束后按 deactivation 规则回零。

HOLD 前法向过期、非法或 Marker 跳变仍按短时保持后平滑回零。已确认 `TOUCHDOWN_HOLD` 后固定 T1 法向锁存，近距视觉丢失不再触发从甲板重新起飞；姿态/角速度安全监视器始终保留并可请求 recovery。

## 8. Ground Truth 与回归隔离

生产类只接受视觉法向、目标状态估计、PX4 odometry 和状态机输入。场景名仅用于启动/参数白名单，不提供法向真值。功能默认：

```yaml
terminal_contact_stabilization.enabled: false
terminal_contact_stabilization.shadow_only: true
```

关闭时不得改变 P6B/P8A/P8B 输出，`TERMINAL_PHASE_P47`、TouchdownDetector 输入和旧参数保持不变。

## 9. 研究状态

```text
P8C-4 RESEARCH PASS
P8C-4 INTERFACE FREEZE PASS
P8C-4 VALIDATION PASS
P8C T1 VALIDATION PASS
SELECTED: B1 + stateful A2 + bounded vertical preload + T1 axis lock/latch
REJECTED: incomplete direct VehicleAttitudeSetpoint control
```

最终真实结果为 roll `3/3`、pitch `3/3`，旧路径回归 `9/9`，全工作区 `340 tests, 0 failures`。证据见 `results/p8c4_validation_20260802/p8c4_final_summary.json`。
