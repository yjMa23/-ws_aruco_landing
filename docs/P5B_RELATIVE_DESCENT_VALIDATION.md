# P5B 相对甲板高度分阶段下降验收记录

## 1. 阶段范围

P5B 在 P4.7 水平跟踪和 P5A 着陆窗口基础上，实现相对甲板高度参考的安全下降：

```text
WAIT_LANDING_WINDOW
→ DESCEND
→ TEST_HEIGHT_HOLD
```

严重失效时使用：

```text
DESCEND
→ RECOVER_CLIMB
→ WAIT_LANDING_WINDOW
```

第一版最低只下降到 `0.50 m` 测试高度，不触地，不发送 `NAV_LAND`，不自动 Disarm。
下降默认关闭，必须显式传入：

```text
--enable-relative-descent
```

## 2. 实现内容

新增纯 C++ 模块：

```text
include/aruco_precision_landing_cpp/relative_descent_controller.hpp
src/relative_descent_controller.cpp
test/relative_descent_controller_test.cpp
```

新增正式状态：

```text
DESCEND
TEST_HEIGHT_HOLD
RECOVER_CLIMB
```

新增调试话题：

```text
/landing/relative_height
/landing/relative_height_reference
/landing/descent_phase
```

新增离线评测入口：

```text
scripts/evaluate_p5b_bag.py
```

## 3. 相对高度控制

PX4 local NED 下：

```text
relative_height = predicted_deck_z_ned - uav_z_ned
target_z_ned = predicted_deck_z_ned - height_reference
```

高度参考在进入下降时使用当前估计相对高度初始化，避免固定世界高度目标跳变。

分段下降参数：

| 相对高度参考 | 下降速率 |
| --- | ---: |
| `> 2.0 m` | `0.30 m/s` |
| `0.8～2.0 m` | `0.15 m/s` |
| `0.5～0.8 m` | `0.05 m/s` |

最低参考严格钳制为：

```text
0.50 m
```

窗口普通关闭时保持当前参考；视觉、估计或预测硬失效时恢复到 `2.0 m`。

## 4. 默认关闭回归

Bag：

```text
bags/p4_static_20260725_010709
```

结果：

- 状态停留在 `WAIT_LANDING_WINDOW`；
- `target_z` 全程为 `-5.0000 m`；
- `/landing/relative_height`、`/landing/relative_height_reference` 和
  `/landing/descent_phase` 均无样本；
- 未进入 `DESCEND`、`TEST_HEIGHT_HOLD` 或 `RECOVER_CLIMB`。

说明普通启动不会因 P5B 接入而自动下降。

## 5. 静止甲板下降

Bag：

```text
bags/p5b_static_descent_20260725_012156
```

状态：

```text
WAIT_LANDING_WINDOW
→ DESCEND
→ TEST_HEIGHT_HOLD
```

结果：

| 指标 | 结果 |
| --- | ---: |
| 参考高度最小值 | `0.5000 m` |
| Ground Truth 相对高度最小值 | `0.6823 m` |
| 估计高度对参考 RMSE | `0.0775 m` |
| Ground Truth 高度对参考 RMSE | `0.2633 m` |
| z 目标最大单步变化 | `0.0196 m` |
| 窗口关闭时参考继续下降次数 | `0` |
| `NAV_LAND` / Disarm | `0 / 0` |

三段参考下降速率中位数约为：

```text
0.289 / 0.155 / 0.048 m/s
```

## 6. 0.4 m/s 匀速甲板下降

Bag：

```text
bags/p5b_constant_descent_20260725_012454
```

结果：

| 指标 | 结果 |
| --- | ---: |
| 参考高度最小值 | `0.5000 m` |
| Ground Truth 相对高度最小值 | `0.7077 m` |
| 估计高度对参考 RMSE | `0.0804 m` |
| Ground Truth 高度对参考 RMSE | `0.2692 m` |
| z 目标最大单步变化 | `0.0191 m` |
| 窗口关闭时参考继续下降次数 | `0` |
| `NAV_LAND` / Disarm | `0 / 0` |

状态只经历一次下降并稳定停在 `TEST_HEIGHT_HOLD`。

## 7. 升沉甲板下降

Bag：

```text
bags/p5b_heave_descent_20260725_013229
```

结果：

| 指标 | 结果 |
| --- | ---: |
| 参考高度最小值 | `0.5000 m` |
| Ground Truth 相对高度最小值 | `0.5347 m` |
| 估计高度对参考 RMSE | `0.1487 m` |
| Ground Truth 高度对参考 RMSE | `0.3033 m` |
| z 目标最大单步变化 | `0.0262 m` |
| 窗口关闭时参考继续下降次数 | `0` |
| `NAV_LAND` / Disarm | `0 / 0` |

升沉场景中视觉估计相对高度最低约 `0.30 m`，而 Ground Truth 最低约 `0.53 m`，
说明当前垂直视觉估计仍有明显动态偏差。P5B 的 `0.50 m` 测试高度边界可以使用，
但尚不足以支持触地判断。

## 8. 组合运动负向验证

Bag：

```text
bags/p5b_combined_descent_20260725_013542
```

即使显式启用 P5B：

- 着陆窗口全程没有打开；
- 未进入 `DESCEND`；
- `target_z` 全程为 `-5.0000 m`；
- 没有绕过窗口启动下降。

## 9. 恢复爬升验收

使用仅用于验收的窗口最低高度覆盖：

```text
--landing-window-min-height 0.60
```

最终 Bag：

```text
bags/p5b_static_descent_predff_h0p10_vff1p0_rvg0p25_adapt_g0p25-1p2_a0p05-0p35_f0p20_winmin0p60_20260725_113006
```

状态：

```text
WAIT_LANDING_WINDOW
→ DESCEND
→ RECOVER_CLIMB
→ WAIT_LANDING_WINDOW
```

结果：

| 指标 | 结果 |
| --- | ---: |
| 估计相对高度最小值 | `0.5879 m` |
| 参考高度最小值 | `0.5434 m` |
| Ground Truth 相对高度最小值 | `0.7942 m` |
| 估计高度对参考 RMSE | `0.1261 m` |
| Ground Truth 高度对参考 RMSE | `0.2862 m` |
| z 目标最大单步变化 | `0.0192 m` |
| 窗口关闭时参考继续下降次数 | `0` |
| `NAV_LAND` / Disarm | `0 / 0` |

恢复到 `2.0 m` 后，系统锁止再次下降并发布：

```text
LANDING_WINDOW_REAUTH_REQUIRED
```

只有重新完成视觉接管或重启任务才解除锁止，避免窗口重新满足后形成自动重复下降循环。

## 10. 调试过程中修复的问题

### 10.1 恢复后的世界高度跳变

最初 `RECOVER_CLIMB → WAIT_LANDING_WINDOW` 会重置相对高度控制器并回到固定 `-5 m`
世界高度目标，产生约 `1 m` 的 z 目标跳变。

修复后继续保持恢复完成时的相对高度参考，最大 z 目标单步变化降低到约：

```text
0.02 m
```

### 10.2 低高度估计噪声误触发恢复

`0.50 m` 参考附近视觉估计会短时低于 `0.50 m`。窗口测量有效下界与下降参考下界已解耦：

```text
landing_window.minimum_relative_height_m = 0.20 m
descent.minimum_test_height_m = 0.50 m
```

前者只判断测量是否物理有效，后者继续作为严格下降安全钳制。

### 10.3 恢复后重复自动下降

恢复验收初版会在窗口重新打开后再次自动下降。现已增加恢复后重新授权锁止：

- 一次恢复后不允许自动二次下降；
- 水平跟踪和 `2.0 m` 相对高度保持继续运行；
- 新视觉接管或任务重启后才可再次授权下降。

## 11. 测试与日志

全工作区：

```text
154 tests
0 errors
0 failures
0 skipped
```

最终恢复实验日志未出现：

- PX4→ROS 时间同步失败；
- 位姿历史插入失败；
- camera-to-NED 变换失败；
- 水平跟踪输入不可用；
- `ABORT`；
- `NAV_LAND`；
- Disarm。

## 12. 阶段结论

P5B 在以下边界内通过：

- 相对高度分阶段下降到 `0.50 m`；
- 静止、0.4 m/s 匀速和升沉甲板；
- 窗口关闭暂停；
- 硬失效恢复到 `2.0 m`；
- 恢复后重新授权锁止；
- 水平跟踪持续运行；
- 不触地、不 Land、不 Disarm。

下一阶段不能直接进入真实触地。应先完成 P5C 低高度垂直状态估计与误差标定，重点降低
Marker z 和升沉场景的动态偏差，再进入 P6 触地检测与最终下降。
