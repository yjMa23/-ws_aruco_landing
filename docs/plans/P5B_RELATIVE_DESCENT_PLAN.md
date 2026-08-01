# P5B 相对甲板高度分阶段下降执行计划

## 1. 阶段目标

在 P5A `WAIT_LANDING_WINDOW` 和 P4.7 水平跟踪基础上，实现相对甲板高度参考的安全下降：

```text
WAIT_LANDING_WINDOW
→ DESCEND
→ TEST_HEIGHT_HOLD
```

第一版最低只下降到 `0.50 m` 安全测试高度，不触地，不发送 `NAV_LAND`，不 Disarm。

## 2. 明确不做

P5B 不实现：

- 触地检测；
- 最终 `0.50 m` 以下下降；
- PX4 自动 Land；
- 自动 Disarm；
- 真实海浪水动力学；
- MPC 或强化学习；
- Ground Truth 进入控制器；
- 复用旧 `DESCEND_WITH_TRACKING`。

## 3. 相对高度定义

PX4 local NED 中：

```text
relative_height_m = predicted_deck_z_ned - uav_z_ned
position_target_z_ned = predicted_deck_z_ned - height_reference_m
```

甲板位于无人机下方时 `relative_height_m > 0`。

进入 P5B 时，`height_reference_m` 使用当前估计相对高度初始化，避免从固定 `5 m` 目标产生跳变。

## 4. Task 1：纯 C++ 相对下降控制器

新增：

```text
src/aruco_precision_landing_cpp/include/aruco_precision_landing_cpp/relative_descent_controller.hpp
src/aruco_precision_landing_cpp/src/relative_descent_controller.cpp
src/aruco_precision_landing_cpp/test/relative_descent_controller_test.cpp
```

状态：

```text
WAITING_WINDOW
DESCENDING
PAUSED
RECOVERING
TEST_HEIGHT_HOLD
```

输入：

```text
current_relative_height_m
window_open
vertical_reference_valid
severe_failure
dt_s
```

输出：

```text
height_reference_m
phase
reference_changed
reached_test_height
```

第一版参数：

```yaml
descent.minimum_test_height_m: 0.50
descent.fast_height_threshold_m: 2.00
descent.slow_height_threshold_m: 0.80
descent.fast_rate_mps: 0.30
descent.medium_rate_mps: 0.15
descent.slow_rate_mps: 0.05
descent.recovery_height_m: 2.00
descent.recovery_rate_mps: 0.30
descent.max_reference_tracking_error_m: 0.50
```

行为：

- 未初始化时以当前相对高度初始化参考；
- 窗口关闭且无严重失效：保持当前相对高度参考；
- 窗口打开：按高度分段降低参考；
- 到 `0.50 m` 后保持，不继续下降；
- 严重失效且参考低于恢复高度：按恢复速率增大参考；
- 当前相对高度与参考差异过大时暂停进一步下降；
- 非有限输入、非正 dt 或负高度返回无效；
- reset 清除全部状态。

## 5. Task 2：状态机接入

新增正式状态：

```text
DESCEND
TEST_HEIGHT_HOLD
RECOVER_CLIMB
```

要求：

- `WAIT_LANDING_WINDOW` 只有窗口打开才进入 `DESCEND`；
- `DESCEND`、`TEST_HEIGHT_HOLD` 和 `RECOVER_CLIMB` 继续运行同一套 P4.7 水平跟踪；
- z 目标由预测甲板 z 和相对高度参考计算；
- 窗口普通关闭时进入暂停，但不改变状态机到旧逻辑；
- 视觉长时丢失或预测无效时进入 `RECOVER_CLIMB`；
- 恢复到 `2.0 m` 后等待窗口重新打开；
- 不连接旧 `DESCEND_WITH_TRACKING` 或 `FINAL_LAND`。

## 6. Task 3：调试与离线评测

新增话题：

```text
/landing/relative_height
/landing/relative_height_reference
/landing/descent_phase
```

新增评测脚本：

```text
scripts/evaluate_p5b_bag.py
```

至少输出：

- 当前相对高度最小/最大；
- 相对高度参考最小/最大；
- 参考跟踪 RMSE；
- 分阶段下降速率；
- 窗口关闭期间参考是否保持；
- 恢复爬升是否达到目标；
- target z 连续性和最大单步变化；
- 是否出现旧下降、Land、Done 或低于 `0.45 m`。

## 7. Task 4：测试顺序

### 7.1 纯逻辑

- 初始化无跳变；
- 三段下降速度；
- 窗口关闭暂停；
- 跟踪误差过大暂停；
- 严重失效恢复；
- 最低测试高度钳制；
- 非法输入和 reset。

### 7.2 PX4 SITL

依次运行：

```text
static
constant 0.4 m/s
heave
rollpitch
combined
```

第一轮只允许：

```text
static → 0.50 m
constant → 0.50 m
heave → 0.50 m
```

`rollpitch` 和 `combined` 第一轮只验证窗口关闭时不下降；确认暂停与恢复逻辑后再允许下降。

## 8. 安全验收标准

- 任何场景相对高度不得低于 `0.45 m`；
- 不发送 `NAV_LAND` 或 Disarm；
- 不进入旧 `DESCEND_WITH_TRACKING`、`FINAL_LAND` 或 `DONE`；
- 水平跟踪在下降阶段持续运行；
- 窗口关闭时下降参考不继续减小；
- 严重失效时参考向 `2.0 m` 恢复；
- Ground Truth 只用于离线评测；
- 全工作区测试通过。

## 9. 当前执行状态

- Task 1：已完成纯 C++ `RelativeDescentController` 与单元测试。
- Task 2：已完成 `DESCEND`、`TEST_HEIGHT_HOLD`、`RECOVER_CLIMB` 状态机接入。
- Task 3：已完成调试话题与 `scripts/evaluate_p5b_bag.py`。
- Task 4：已完成默认关闭、静止、0.4 m/s 匀速、升沉、组合负向和恢复爬升 PX4 SITL 验收。
- 已增加恢复后重新授权锁止，防止自动重复下降。
- 全工作区 154 项测试通过。

详细验收记录：

```text
docs/validation/P5B_RELATIVE_DESCENT_VALIDATION.md
```
