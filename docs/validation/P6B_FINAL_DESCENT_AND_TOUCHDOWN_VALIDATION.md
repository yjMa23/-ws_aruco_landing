# P6B 最终下降与真实接触正向验收记录

## 1. 最终结论

执行日期：2026-07-29；2026-07-30 再次复验。

代码基线：

```text
ef78e979136ace0fd8be4aab0f7504b66d1e8ded
```

验收时工作树包含尚未提交的 P6B 终端落板修补和 P7 管线实现，因此两个 episode 均记录 `dirty_worktree: true`。

最终结果：

```text
static:     PASS
constant02: PASS
P6B freeze: READY
P7:         可进入 3+3 冒烟
```

修补后的最终下降逻辑为：

```text
0.50 m 进入 FINAL_DESCENT
→ 0.25 m 以上按约 0.12 m/s 接近
→ 0.25 m 以下按约 0.03 m/s 慢速下降
→ 终端段允许参考继续降到 0.05 m
→ 最低落板命令到达前禁止“垂直停滞”触地证据
→ 起落架实际压到甲板且垂直运动停滞后形成候选
→ 连续候选满足时进入 TOUCHDOWN_HOLD
```

没有自动发送 `NAV_LAND` 或 Disarm。

---

## 2. 物理接触口径

Gazebo X500 模型的机体参考点不在起落架底部。根据：

```text
~/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf
```

最低起落架碰撞体相对机体参考点约为：

```text
0.227 m
```

因此离线真实接触间隙按以下方式计算：

```text
landing_gear_clearance = ground_truth_reference_height - 0.227 m
```

接触验收范围：

```text
-0.05 m <= landing_gear_clearance <= 0.03 m
```

负值表示碰撞求解中的轻微接触压入，不表示机体参考点进入甲板。

---

## 3. static 正向验收

### 3.1 文件

```text
Episode:
results/p6b_bugfix/static_terminal_descent_seed1_v2/

Bag:
results/p6b_bugfix/static_terminal_descent_seed1_v2/bag

Human-readable evaluation:
results/p6b_bugfix/static_terminal_descent_seed1_v2/evaluation.txt

JSON evaluation:
results/p6b_bugfix/static_terminal_descent_seed1_v2/evaluation.json
```

### 3.2 指标

| 验收项 | 结果 |
| --- | --- |
| 场景 | `static` |
| seed | `1` |
| 相机模型 | `close-range`，near=`0.02 m` |
| 完整状态序列 | `WAIT_FOR_PX4 → OFFBOARD_PRE_STREAM → ARM_AND_TAKEOFF → WAIT_DECK_GNSS → RENDEZVOUS_GNSS → ACQUIRE_ARUCO → VISUAL_HANDOVER → TRACK_TARGET → WAIT_LANDING_WINDOW → DESCEND → TEST_HEIGHT_HOLD → FINAL_DESCENT → TOUCHDOWN_CANDIDATE_HOLD → TOUCHDOWN_HOLD` |
| `FINAL_DESCENT` 首次时间 | `1785341256.5739262 s` |
| `CANDIDATE` 首次时间 | `1785341279.874523 s` |
| `CONFIRMED` 首次时间 | `1785341280.3246076 s` |
| `TOUCHDOWN_HOLD` 首次时间 | `1785341280.3745115 s` |
| 候选到确认延迟 | `0.4501 s` |
| 最终下降到确认时间 | `23.7507 s` |
| 接近段参考下降速度 | `0.12004 m/s` |
| 接触段参考下降速度 | `0.02895 m/s` |
| 最低参考高度 | `0.0500 m` |
| 最低估计高度 | `0.2181 m` |
| 最低 Ground Truth 机体高度 | `0.2107 m` |
| 起落架最小接触间隙 | `-0.0163 m`，PASS |
| 触地相对垂直速度 | `0.0119 m/s` |
| `TOUCHDOWN_HOLD` 持续时间 | `10.0002 s` |
| 保持阶段 z 目标跨度 | `0.0 m` |
| 保持阶段最大 z 速度命令 | `0.0 m/s` |
| 水平 RMSE | `0.02616 m` |
| 最大水平误差 | `0.04972 m` |
| 最终阶段甲板水平位移 | 约 `0 m` |
| Marker 切换序列 | `[1, 2]` |
| 视觉丢失 / 恢复次数 | `0 / 0` |
| `NAV_LAND / Disarm` | `0 / 0` |
| 最终结果 | **PASS** |

---

## 4. constant02 正向验收

### 4.1 文件

```text
Episode:
results/p6b_bugfix/constant02_terminal_descent_seed1/

Bag:
results/p6b_bugfix/constant02_terminal_descent_seed1/bag

Human-readable evaluation:
results/p6b_bugfix/constant02_terminal_descent_seed1/evaluation.txt

JSON evaluation:
results/p6b_bugfix/constant02_terminal_descent_seed1/evaluation.json
```

评测由单轮运行器自动带入 `--require-moving-deck`。

### 4.2 指标

| 验收项 | 结果 |
| --- | --- |
| 场景 | `constant02`，甲板速度 `0.2 m/s` |
| seed | `1` |
| 相机模型 | `close-range`，near=`0.02 m` |
| 完整状态序列 | `WAIT_FOR_PX4 → OFFBOARD_PRE_STREAM → ARM_AND_TAKEOFF → WAIT_DECK_GNSS → RENDEZVOUS_GNSS → ACQUIRE_ARUCO → VISUAL_HANDOVER → TRACK_TARGET → WAIT_LANDING_WINDOW → DESCEND → TEST_HEIGHT_HOLD → FINAL_DESCENT → TOUCHDOWN_CANDIDATE_HOLD → TOUCHDOWN_HOLD` |
| `FINAL_DESCENT` 首次时间 | `1785341660.0245433 s` |
| `CANDIDATE` 首次时间 | `1785341685.0750594 s` |
| `CONFIRMED` 首次时间 | `1785341685.525053 s` |
| `TOUCHDOWN_HOLD` 首次时间 | `1785341685.575058 s` |
| 候选到确认延迟 | `0.4500 s` |
| 最终下降到确认时间 | `25.5005 s` |
| 接近段参考下降速度 | `0.11989 m/s` |
| 接触段参考下降速度 | `0.02999 m/s` |
| 最低参考高度 | `0.0500 m` |
| 最低估计高度 | `0.2191 m` |
| 最低 Ground Truth 机体高度 | `0.1955 m` |
| 起落架最小接触间隙 | `-0.0315 m`，PASS |
| 触地相对垂直速度 | `0.00985 m/s` |
| `TOUCHDOWN_HOLD` 持续时间 | `10.000 s` |
| 保持阶段 z 目标跨度 | `0.0 m` |
| 保持阶段最大 z 速度命令 | `0.0 m/s` |
| 水平 RMSE | `0.03446 m` |
| 最大水平误差 | `0.06666 m` |
| 最终阶段甲板水平位移 | `7.0921 m`，moving-deck PASS |
| Marker 切换序列 | `[1, 2, 3, 2]` |
| 视觉丢失 / 恢复次数 | `1 / 1` |
| `NAV_LAND / Disarm` | `0 / 0` |
| 最终结果 | **PASS** |

---

## 5. 修补内容

### 5.1 终端落板段

最终下降控制器新增：

- `terminal_descent_entry_height_m=0.20`；
- `minimum_command_height_m=0.05`；
- `maximum_reference_tracking_error_m=0.20`；
- 仅在除低高度外的 landing-window 条件仍安全时进入终端段；
- 终端段可忽略“仅由低高度造成的窗口关闭”，继续低速落板；
- 候选出现后立即冻结下降参考；
- 确认后锁存当前位置保持。

### 5.2 防止悬停误判触地

第一次修补测试发现参考约 `0.12 m` 时可能因估计高度偏差和垂直低速提前形成“终端停滞”候选，但起落架尚未触地。

最终修补增加：

```text
terminal_command_complete
```

只有参考已经到达最低命令 `0.05 m` 后，终端停滞证据才允许生效。新增单元测试明确验证：

```text
reference=0.12 m + low vertical speed → AIRBORNE
reference=0.05 m + physical stall → CANDIDATE / CONFIRMED
```

### 5.3 评测器

评测器新增：

- X500 机体参考点到起落架接触点的几何偏移；
- 起落架 Ground Truth 接触间隙；
- `physical_contact_passed`；
- 保持状态进入后的短暂同周期目标切换去抖；
- 位置保持未发送有限 z 速度前馈时按 `0 m/s` 处理。

---

## 6. 测试结果

```text
230 tests
0 errors
0 failures
0 skipped
```

真实 SITL：

```text
static:     PASS
constant02: PASS
```

P6B 已满足冻结条件。建议用户检查后执行：

```bash
git tag -a baseline-touchdown-v0.1 \
  -m "Freeze static and constant02 touchdown baseline"
```

本次未执行 commit、tag 或 push。

---

## 7. 2026-07-30 再次复验

为确认终端落板逻辑不是单次偶然成功，使用当前工作树重新执行：

```text
static seed 1
constant02 seed 1
P7 smoke: static seeds 101/102/103 + constant02 seeds 201/202/203
```

单轮结果目录：

```text
results/p6b_completion/p6b_terminal_static_seed1/
results/p6b_completion/p6b_terminal_constant02_seed1/
```

3+3 冒烟目录：

```text
results/p7_smoke_terminal_20260730/
```

结果：

```text
static seed 1:     PASS
constant02 seed 1: PASS
P7 smoke:          6/6 PASS
failure count:     0
```

3+3 聚合指标：

| 指标 | 结果 |
| --- | ---: |
| 成功率 | `100%` |
| 平均落地时间 | `24.9006 s` |
| 平均候选到确认延迟 | `0.4584 s` |
| 平均水平 RMSE | `0.02508 m` |
| 平均最大水平误差 | `0.05048 m` |
| 平均触地垂直速度 | `0.00602 m/s` |
| 平均保持时间 | `10.00019 s` |
| 恢复次数 | `0` |

六轮最低参考高度均为 `0.05 m`，全部完成：

```text
FINAL_DESCENT
→ TOUCHDOWN_CANDIDATE_HOLD
→ TOUCHDOWN_HOLD
```

复验期间没有发送 `NAV_LAND` 或 Disarm，所有 PX4、Gazebo、MicroXRCEAgent 和 ROS SITL 子进程均正常清理。
