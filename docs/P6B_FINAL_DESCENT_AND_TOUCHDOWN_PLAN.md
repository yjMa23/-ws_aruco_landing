# P6B 最终下降与真实接触正向验证执行计划

## 1. 阶段目标

在 P6A 多源触地检测已经通过负向验收的基础上，增加一条**显式授权、静止甲板优先、低速且可中止**的最终下降路径：

```text
TEST_HEIGHT_HOLD（0.50 m）
→ FINAL_DESCENT
→ TOUCHDOWN_CANDIDATE_HOLD
→ TOUCHDOWN_HOLD
```

P6B 第一版目标是完成静止甲板 SITL 的真实物理接触和触地确认，并在确认后保持控制输出稳定。

本阶段仍不自动 Disarm，不自动退出 Offboard，不使用 Ground Truth 参与控制。

---

## 2. 强制安全边界

P6B 必须满足：

- 最终下降默认关闭；
- 必须显式传入 `--enable-final-descent`；
- 未显式启用 P5B 相对下降时，最终下降不能启动；
- 第一版只允许静止甲板场景；
- 第一版从 P5B `0.50 m` 测试高度开始；
- 最终下降速率默认不超过 `0.03 m/s`；
- 候选触地出现后立即停止降低垂直参考；
- 触地确认后冻结垂直目标和 z 速度前馈；
- 不发送 `NAV_LAND`；
- 不自动 Disarm；
- 不关闭 Offboard；
- 不进入旧 `DESCEND_WITH_TRACKING` 或 `FINAL_LAND`；
- 视觉、PX4 land 状态或时间异常时不得继续降低最终下降参考；
- Ground Truth 只能进入 rosbag 离线评测。

若静止甲板未通过，不测试匀速、升沉、倾斜或组合甲板真实接触。

---

## 3. Task 1：纯 C++ FinalDescentController

新增：

```text
src/aruco_precision_landing_cpp/include/aruco_precision_landing_cpp/final_descent_controller.hpp
src/aruco_precision_landing_cpp/src/final_descent_controller.cpp
src/aruco_precision_landing_cpp/test/final_descent_controller_test.cpp
```

### 3.1 输入

```text
current_relative_height_m
current_reference_height_m
final_descent_authorized
vertical_reference_valid
landing_window_open
touchdown_status
dt_s
```

### 3.2 输出

```text
relative_height_reference_m
vertical_reference_velocity_ned_mps
phase
reference_changed
touchdown_candidate_hold
touchdown_confirmed_hold
recovery_requested
```

### 3.3 阶段

```text
kWaitingAuthorization
kDescending
kCandidateHold
kTouchdownHold
kPaused
kRecoveryRequested
```

### 3.4 行为

- 首次进入时从当前 P5B 相对高度参考初始化，不能跳变；
- 授权、垂直参考和窗口均有效时，以 `final_descent_rate_mps` 降低相对高度参考；
- 参考最低钳制到 `minimum_command_height_m`，防止无限向甲板内部发送位置目标；
- `TouchdownStatus::kCandidate` 时立即冻结相对高度参考，z 速度前馈设为 0；
- `TouchdownStatus::kConfirmed` 时进入锁存保持；
- `kInsufficientEvidence` 或 `kRejectedUnsafe` 时暂停，不继续降低参考；
- 视觉或窗口严重失效时请求恢复到 P5B 安全高度；
- reset 清除全部历史。

### 3.5 初始参数

```text
entry_height_m = 0.50
final_descent_rate_mps = 0.03
minimum_command_height_m = 0.15
maximum_reference_tracking_error_m = 0.20
```

`minimum_command_height_m` 是命令钳制，不是触地判据。真实触地必须由 P6A 多源检测确认。

---

## 4. Task 2：单元测试

至少覆盖：

1. 未授权不能下降；
2. 首次初始化无参考跳变；
3. 正常授权按 `0.03 m/s` 降低参考；
4. 参考不低于命令钳制高度；
5. P6A 候选出现后立即保持；
6. 候选消失且证据有效时可恢复下降；
7. P6A 确认后锁存保持；
8. 触地保持时 z 参考速度为零；
9. PX4/视觉证据不足时暂停；
10. 不安全拒绝时请求恢复；
11. 跟踪误差过大时暂停；
12. 时间、输入和参数非法时拒绝；
13. reset 清除状态。

---

## 5. Task 3：状态机接入

新增状态：

```text
FINAL_DESCENT
TOUCHDOWN_CANDIDATE_HOLD
TOUCHDOWN_HOLD
```

状态转换：

```text
TEST_HEIGHT_HOLD
  -- final_descent.enabled && 显式授权 --> FINAL_DESCENT

FINAL_DESCENT
  -- P6A CANDIDATE --> TOUCHDOWN_CANDIDATE_HOLD
  -- P6A CONFIRMED --> TOUCHDOWN_HOLD
  -- 严重失效 --> RECOVER_CLIMB

TOUCHDOWN_CANDIDATE_HOLD
  -- P6A CONFIRMED --> TOUCHDOWN_HOLD
  -- 候选消失且安全 --> FINAL_DESCENT
  -- 严重失效 --> RECOVER_CLIMB
```

`TOUCHDOWN_HOLD` 第一版：

- 垂直目标冻结为确认时当前 `local_position.z`；
- z 速度前馈清零；
- 水平目标继续使用当前甲板估计，避免平台轻微移动导致横向脱离；
- 不发送 Land/Disarm；
- 发布触地确认和保持状态；
- 只允许节点重启或显式 reset 解除。

---

## 6. Task 4：P6A 检测允许状态扩展

P6A 当前只在 `TEST_HEIGHT_HOLD` 允许积累候选。

P6B 扩展为：

```text
FINAL_DESCENT
TOUCHDOWN_CANDIDATE_HOLD
TOUCHDOWN_HOLD
```

不在以下状态积累：

```text
WAIT_LANDING_WINDOW
DESCEND
RECOVER_CLIMB
RECOVER_TO_GNSS
```

触地确认仍必须包含 PX4 接触证据，视觉高度不能单独确认。

---

## 7. Task 5：启动参数与 rosbag

新增显式参数：

```text
final_descent.enabled
final_descent.rate_mps
final_descent.minimum_command_height_m
final_descent.max_reference_tracking_error_m
```

启动脚本：

```bash
./scripts/start_sitl.sh \
  --scenario static \
  --headless \
  --record \
  --enable-relative-descent \
  --enable-final-descent
```

脚本必须拒绝：

- `--enable-final-descent` 但未启用相对下降；
- 非 `static` 场景启用最终下降；
- 最终下降速率超过安全上限；
- 命令最低高度不在安全配置范围。

rosbag 继续记录：

- P4/P5/P6 全部调试话题；
- PX4 `VehicleLandDetected`；
- TrajectorySetpoint；
- VehicleCommand；
- Ground Truth，仅离线评测。

---

## 8. Task 6：静止甲板分级 SITL 验收

### 8.1 启动级验证

先只验证：

- 参数加载；
- 新状态和话题存在；
- 默认关闭时 P5C 行为完全不变。

### 8.2 第一次真实接触试验

仅静止甲板：

- 从 `0.50 m` 开始；
- 最终下降 `0.03 m/s`；
- 命令最低高度 `0.15 m`；
- QGC 保持打开；
- 不 Land、不 Disarm；
- 触地候选后立即保持；
- 触地确认后冻结垂直目标。

### 8.3 接触阈值标定

使用 rosbag 离线比较：

- P6A 触地状态；
- PX4 `ground_contact / maybe_landed / landed / at_rest`；
- 视觉相对高度；
- 相对垂直速度；
- Ground Truth 实际相对高度；
- 目标 z 和 z 速度前馈。

若物理接触时视觉相对高度高于当前 `0.18 m`，只根据传感器与模型语义调整 P6A 的视觉辅助阈值；Ground Truth 只能用于离线解释，不能在线反馈阈值。

### 8.4 正向验收要求

- 必须出现 `CANDIDATE`；
- 候选后目标参考不继续下降；
- 必须连续满足后出现 `CONFIRMED`；
- 确认后保持至少 `10 s`；
- 确认期间没有明显垂直振荡；
- 无 `NAV_LAND`；
- 无 Disarm；
- 无旧下降状态；
- 无 ABORT/failsafe；
- Ground Truth 不穿透甲板；
- P4.7 水平误差保持可控。

若首次试验只出现接触而不能确认，先分析证据缺失，不盲目降低视觉阈值或缩短持续时间。

---

## 9. Task 7：离线评测

新增或扩展 P6 评测：

- 第一次接触时间；
- 候选开始时间；
- 确认时间；
- 候选到确认延迟；
- 候选后继续下降次数；
- 确认后 z 目标变化范围；
- 确认后 z 速度前馈最大值；
- PX4 land 标志变化序列；
- Ground Truth 最低相对高度和穿透量；
- Land/Disarm 命令计数；
- 水平误差。

---

## 10. 验收标准

- 纯 C++ 最终下降控制器测试全部通过；
- 默认关闭路径与 P5C 一致；
- 非静止场景或未启用 P5B 时，脚本拒绝最终下降；
- 最终下降速率不超过 `0.03 m/s`；
- 候选出现后相对高度参考不继续降低；
- 确认后垂直目标和 z 前馈稳定；
- 静止甲板真实接触可得到 P6A `CONFIRMED`；
- 确认后保持至少 `10 s`；
- `NAV_LAND / Disarm = 0 / 0`；
- Ground Truth 不进入控制器；
- 全工作区测试通过。

---

## 11. 执行顺序

```text
保存本计划
→ 纯 C++ FinalDescentController
→ 单元测试
→ 默认关闭回归
→ 状态机并行接入
→ 参数与启动脚本安全限制
→ 静止甲板真实接触试验
→ 证据链离线分析
→ 必要的最小阈值修正
→ 正向复验
→ P6B 验收文档
```

## 12. 当前状态

- 本计划已保存。
- 尚未修改 P6B 最终下降代码。
- 当前仍不允许 `0.50 m` 以下下降，直到纯逻辑和默认关闭回归通过。
