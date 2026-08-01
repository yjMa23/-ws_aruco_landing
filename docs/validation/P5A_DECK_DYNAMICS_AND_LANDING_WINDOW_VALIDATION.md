# P5A 甲板动态仿真与规则式着陆窗口验收记录

## 1. 阶段范围

P5A 在 P4.7 安全高度移动目标跟踪基础上完成：

- 甲板升沉、横摇/纵摇和组合运动解析轨迹；
- Gazebo 三维线速度与机体系角速度驱动；
- 完整甲板 Ground Truth 位置、姿态、线速度和角速度；
- 基于 Marker 向上法向量的甲板倾角估计；
- 带进入/退出迟滞和连续满足时间的规则式着陆窗口；
- `WAIT_LANDING_WINDOW` 状态与窗口调试话题；
- 五类真实 PX4 SITL 场景验收。

本阶段只判断下降条件，不实现下降、触地、Land 或 Disarm。所有验收飞行继续保持
`5 m` 安全高度。

## 2. 新增甲板运动场景

`MotionProfile` 新增：

```text
S3_HEAVE
S4_ROLL_PITCH
S5_COMBINED
```

新增参数：

```yaml
amplitude_z_m
period_z_s
initial_rpy_deg
amplitude_rpy_deg
period_rpy_s
```

新增输出：

```text
position_enu
rpy_enu
linear_velocity_enu
angular_velocity_body
```

当前场景配置：

| 场景 | 运动参数 |
| --- | --- |
| `heave.yaml` | z 振幅 `0.30 m`，周期 `8 s` |
| `roll_pitch.yaml` | roll `5°/8 s`，pitch `3°/6 s` |
| `combined.yaml` | XY 正弦 + `0.30 m` 升沉 + roll/pitch |

旧 S0/S1/S2 的水平位置和速度公式保持不变。

## 3. Gazebo Ground Truth 验证

独立启动 `moving_deck_sim`，不启动 PX4 控制器，读取
`/simulation/deck/ground_truth`：

### 3.1 升沉

示例样本：

```text
z  = 2.2640 m
vz = 0.1193 m/s
orientation = identity
```

说明 z 和 vz 按解析轨迹变化，甲板保持水平。

### 3.2 横摇/纵摇

示例样本：

```text
position ≈ [0, 0, 2] m
orientation = non-identity quaternion
angular velocity ≈ [0.0299, 0.0042, -0.0003] rad/s
```

说明甲板中心基本不漂移，实际姿态和机体系角速度持续变化。

### 3.3 组合运动

示例样本：

```text
position ≈ [0.795, 0.493, 2.265] m
linear velocity ≈ [0.399, 0.051, 0.106] m/s
angular velocity ≈ [0.0308, 0.0054, -0.0004] rad/s
```

说明 XY、升沉、姿态和角速度能够同时生效。

## 4. Marker 法向量姿态估计

水平甲板的 Marker 完整四元数直接转换欧拉角时，roll 约为 `180°`，原因是 Marker
坐标定义包含固有翻转，不能直接解释为甲板姿态。

P5A 新增：

```text
deck_attitude_estimator.hpp/.cpp
deck_attitude_estimator_test.cpp
```

算法：

```text
Marker +Z 法向量
→ 转换到 local NED
→ 检查法向量朝上
→ 对单位法向量一阶低通
→ 计算 yaw 无关的 roll、pitch 和总倾角
```

调试话题：

```text
/landing/estimated_deck_attitude
```

消息为 `geometry_msgs/msg/Vector3Stamped`：

```text
x = roll rad
y = pitch rad
z = total tilt rad
```

真实 SITL 中姿态总倾角 RMSE：

| 场景 | 倾角 RMSE |
| --- | ---: |
| 静止 | `1.611°` |
| 0.4 m/s 匀速 | `1.459°` |
| 升沉 | `1.468°` |
| 横摇/纵摇 | `1.029°` |
| 组合 | `0.782°` |

## 5. 规则式着陆窗口

新增：

```text
landing_window.hpp/.cpp
landing_window_test.cpp
```

默认参数：

```yaml
landing_window.enter_horizontal_error_m: 0.15
landing_window.exit_horizontal_error_m: 0.25
landing_window.enter_relative_speed_mps: 0.15
landing_window.exit_relative_speed_mps: 0.25
landing_window.enter_max_tilt_deg: 5.0
landing_window.exit_max_tilt_deg: 8.0
landing_window.max_visual_age_s: 0.20
landing_window.minimum_relative_height_m: 0.50
landing_window.maximum_relative_height_m: 6.00
landing_window.required_duration_s: 1.00
```

窗口关闭时使用进入阈值；窗口打开后使用更宽松的退出阈值。视觉、估计、预测、时间
或相对高度等硬有效性条件失败时立即关闭。

拒绝原因位掩码：

```text
visual unavailable
visual too old
estimate invalid
prediction invalid
horizontal error
relative speed
deck tilt
relative height
invalid time
```

调试话题：

```text
/landing/window_open
/landing/window_reject_reasons
/landing/window_satisfied_duration
```

## 6. 状态机行为

主路径新增：

```text
TRACK_TARGET
→ WAIT_LANDING_WINDOW
```

两者使用完全相同的水平跟踪控制。第一条有效跟踪指令后进入
`WAIT_LANDING_WINDOW`，但：

- z 目标继续固定为 `-5 m`；
- 不连接 `DESCEND_WITH_TRACKING`；
- 不发送 `NAV_LAND`；
- TRACK→WAIT 时保留速度前馈和自适应增益历史；
- 离开视觉链路时重置姿态估计和着陆窗口。

## 7. PX4 SITL 验收结果

统一使用：

```bash
python3 scripts/evaluate_p5a_bag.py bags/<bag_name>
```

| 场景 | Bag | 首次开启延迟 | 开启时连续满足时间 | 开启样本比例 | 最大估计倾角 | 结果 |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| 静止 | `p4_static_20260725_002325` | `2.50 s` | `1.00 s` | `97.1%` | `4.50°` | 开启 |
| 0.4 m/s 匀速 | `p4_constant_20260725_003923` | `2.25 s` | `1.00 s` | `97.2%` | `4.44°` | 开启 |
| 升沉 | `p4_heave_20260725_003513` | `2.60 s` | `1.00 s` | `96.9%` | `4.76°` | 开启 |
| 横摇/纵摇 | `p4_rollpitch_20260725_002850` | `5.35 s` | `1.00 s` | `93.4%` | `7.40°` | 迟滞保持 |
| 组合 | `p4_combined_20260725_003159` | 未开启 | — | `0%` | `7.38°` | 安全阻断 |

### 7.1 静止、匀速和升沉

窗口开启前主要由初始水平误差和相对速度阻塞。条件连续满足 `1.00 s` 后窗口开启，
之后未抖动关闭。

### 7.2 横摇/纵摇

窗口开启前出现 `deck_tilt` 拒绝。进入倾角阈值满足后开启；开启后最大估计倾角约
`7.40°`，低于 `8°` 退出阈值，因此窗口由迟滞保持，符合设计。

### 7.3 组合运动

窗口全程保持关闭，主要拒绝原因为：

```text
horizontal_error: 1568 samples
relative_speed: 1328 samples
deck_tilt: 407 samples
```

说明只有在位置、速度和姿态条件同时连续满足时才会开放，不会因单个条件偶然满足而
误触发。

## 8. 安全边界验证

五轮真实 PX4 SITL 中：

```text
target_z_min = -5.0000 m
target_z_max = -5.0000 m
target_z_span = 0.000000 m
```

所有 bag 均未出现：

```text
CENTER_ABOVE_MARKER
DESCEND_WITH_TRACKING
FINAL_LAND
DONE
```

本轮最新日志也未出现：

```text
timestamp synchronization failed
pose history insertion failed
camera-to-NED transform failed
Moving target tracking input unavailable
Rejected Marker attitude
ABORT
```

## 9. 测试结果

完整工作区：

```text
3 packages finished
142 tests
0 errors
0 failures
0 skipped
```

新增测试覆盖：

- S3/S4/S5 解析轨迹和角速度；
- 参数非法与旧场景兼容；
- Marker 固有翻转下水平甲板零倾角；
- 法向量姿态估计、滤波、reset 和异常输入；
- 窗口连续满足时间；
- 进入/退出迟滞；
- 多原因拒绝；
- 视觉超时、相对高度和时间回退；
- 窗口 reset。

## 10. 验收结论

P5A 已完成。项目已经具备：

```text
三维动态甲板仿真
+ 视觉甲板倾角估计
+ 规则式着陆窗口
+ WAIT_LANDING_WINDOW 安全高度状态
```

下一阶段为 P5B：实现相对甲板高度参考和分阶段下降控制。P5B 第一版应只下降到
`0.5 m` 左右的安全测试高度，不实现触地、Land 或 Disarm，并必须在窗口恶化时暂停或
恢复高度。
