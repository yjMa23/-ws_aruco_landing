# Future Twist estimator-confidence replay

## 1. 问题定义

本阶段原计划只回答一个可证伪问题：

> `DeckMotionEstimator` 自身的 acceleration / angular-acceleration uncertainty，是否能够识别会在 `0.5 s` Future Twist 中造成大尾部误差的 acceleration estimate？

但在读取 estimator covariance 和进行任何 Ground Truth correlation 之前，必须先证明固定 Bag 可以离线等价重放 production estimator。实际执行结果是 **replay equivalence 未通过**，因此本阶段按预注册规则停止在 replay 数据流缺口，不进入 confidence correlation、quintile、tail capture，也不实现 production confidence law。

## 2. 固定数据与因果边界

只使用：

```text
results/deck_motion_shadow_planar_marine_refinelm_local7m_20260815
```

覆盖：

```text
static / rollpitch / combined / rigid_body_motion
× seed 1 / 2 / 3
```

12 个 Bag 均可读取。每轮均包含：

```text
/aruco/pose
/aruco/id
/landing/active_marker_id
/landing/state
/landing/deck_motion_shadow/state
/landing/deck_motion_shadow/trajectory
/fmu/out/vehicle_odometry
```

本阶段没有读取 scenario phase、MotionProfile 未来状态或 future Ground Truth 来生成 replay 输入；也没有运行新 SITL、换 seed、删除失败 episode 或修改旧 `evaluation.json`。

Ground Truth correlation 没有执行，因为 replay equivalence gate 已经失败。

## 3. Replay architecture

新增离线 C++ harness：

```text
deck_motion_estimator_replay
```

它直接链接现有 `aruco_landing_math`，复用 production：

```text
DeckMotionEstimator
VehiclePoseHistory
transform_marker_to_uav_centered_ned()
```

Python `scripts/replay_deck_motion_estimator.py` 只负责：

```text
rosbag2 读取
→ controller_config.yaml snapshot 参数恢复
→ causal event 编排
→ C++ harness stdin
→ production trajectory origin equivalence 评分
→ strict JSON
```

没有 Python clone `DeckMotionEstimator`、`VehiclePoseHistory` 或 coordinate transform。

### 3.1 Production estimator update input

真实 update 仍从 causal Bag 数据恢复：

```text
/aruco/pose.header.stamp
/aruco/pose.pose
/fmu/out/vehicle_odometry
/aruco/id
controller_config.yaml camera extrinsic
```

C++ harness 内部执行：

```text
VehiclePoseHistory::lookup_state(image_sample_time)
→ transform_marker_to_uav_centered_ned(...)
→ DeckMotionEstimator::update(...)
```

`/landing/deck_motion_shadow/state` 的 pose/twist 从未作为 estimator update 输入；`/landing/deck_motion_shadow/trajectory` 的 origin 也只用于 equivalence 评分。

## 4. Accepted sample reconstruction 的实际发现

原计划允许把 shadow state header 的唯一变化视为 estimator accepted sample schedule。固定 Bag 证明这一假设不完整。

原因是：

```text
ArUco ≈ 30 Hz
control/shadow publish = 20 Hz
```

两个 control tick 之间可能有不止一帧 ArUco 被 estimator 连续接受，而 shadow state 只在 control tick 发布“最后一个 accepted sample”。例如 `static_s1` 在第一条 shadow state 前可见：

```text
/aruco/pose sample 26.832 s
/aruco/pose sample 26.864 s
shadow state header    26.864 s
```

production estimator 实际先消费 `26.832 s`，再消费 `26.864 s`；若只 replay state header 的唯一变化，quadratic-fit history 会立即分叉。

因此最终 replay 使用：

1. `/landing/state` 只恢复当时是否处于 production `visual_state`；
2. visual state 中的 causal `/aruco/pose` 全部按原 sample stamp 重放；
3. state header 只用于约束每个 control tick 前“最后 accepted sample”及 trajectory 配对；
4. `/aruco/id` 恢复 update 的 marker ID；`/landing/active_marker_id` 只做 Bag presence/count 核对。

这仍不读取 shadow state pose/twist 作为 estimator 输入。

## 5. Publish-time semantics

production 语义保持：

```text
state.header.stamp
= estimator last accepted sample time

trajectory.header.stamp
= publish now

trajectory time_from_start=0
= estimator 从 sample time propagation 到 publish now 后的 causal origin
```

离线 harness 因此对每个真实 `trajectory.header.stamp` 调用：

```text
DeckMotionEstimator::predict(publish_time)
```

并将 `prediction.points[0]` 与 Bag 中同一个 trajectory origin 比较。

## 6. Replay equivalence result

预注册门：

```text
paired origin coverage >= 99%
sample time max abs error <= 1e-6 s

linear velocity:
P95 <= 1e-6 m/s, max <= 1e-5 m/s

linear acceleration:
P95 <= 1e-6 m/s², max <= 1e-5 m/s²

angular velocity:
P95 <= 1e-6 rad/s, max <= 1e-5 rad/s

angular acceleration:
P95 <= 1e-6 rad/s², max <= 1e-5 rad/s²
```

默认、未人为校准 callback receipt time 的全矩阵结果：

```text
paired origin coverage                  7464 / 7464 = 100%
sample-time worst episode P95           0 s
sample-time max                          0 s

linear velocity worst episode P95       0.019300 m/s
linear velocity max                      0.031134 m/s

linear acceleration worst episode P95   0.071471 m/s²
linear acceleration max                 0.155767 m/s²

angular velocity worst episode P95      0.002175 rad/s
angular velocity max                     0.002780 rad/s

angular acceleration worst episode P95  0.003104 rad/s²
angular acceleration max                 0.004240 rad/s²
```

因此：

```text
REPLAY EQUIVALENCE = FAIL
```

### 6.1 Episode 结果

| episode | linear v P95 m/s | linear a P95 m/s² | angular v P95 rad/s | angular a P95 rad/s² |
| --- | ---: | ---: | ---: | ---: |
| static_s1 | 0.007827 | 0.033250 | 0.000842 | 0.001179 |
| static_s2 | 0.008769 | 0.027334 | 0.000861 | 0.001277 |
| static_s3 | 0.007238 | 0.024550 | 0.000776 | 0.001067 |
| rollpitch_s1 | 0.009593 | 0.036668 | 0.000990 | 0.001539 |
| rollpitch_s2 | 0.009179 | 0.038832 | 0.000992 | 0.001472 |
| rollpitch_s3 | 0.007375 | 0.027020 | 0.000762 | 0.001084 |
| combined_s1 | 0.017262 | 0.059320 | 0.001847 | 0.002775 |
| combined_s2 | 0.015838 | 0.056539 | 0.001848 | 0.002479 |
| combined_s3 | 0.019300 | 0.071471 | 0.002175 | 0.003104 |
| rigid_body_motion_s1 | 0.015380 | 0.054068 | 0.001736 | 0.002553 |
| rigid_body_motion_s2 | 0.019277 | 0.064140 | 0.002108 | 0.002881 |
| rigid_body_motion_s3 | 0.015564 | 0.058268 | 0.001765 | 0.002627 |

## 7. 已定位的数据流缺口

固定 Bag 中 `/fmu/out/vehicle_odometry` 的 rosbag timestamp 是 recorder 的 wall-clock receipt time，而 production node 在：

```text
vehicle_odometry_callback()
```

内部使用的是：

```text
get_clock()->now()
```

对应 simulation ROS time。然后该值进入 `update_px4_to_ros_time_offset()` 的低通 offset：

```text
observed_offset
= callback_ros_receipt_time
- px4_sync_timestamp

filtered_offset
← filtered_offset + 0.05 * offset_error
```

最终 `VehiclePoseHistory` 的 sample time 和图像时刻插值都依赖这条 filtered offset history。

但是固定 Bag：

```text
没有 /clock
没有 vehicle_odometry_callback() 内部 get_clock()->now() 的逐帧记录
没有最终写入 VehiclePoseHistory 的 mapped odometry sample time
```

因此离线只能通过 trajectory `header.stamp` 与 rosbag wall receipt 的 epoch 差来近似恢复 callback ROS receipt time。trajectory recorder latency 本身存在约毫秒级抖动，无法恢复 node subscription callback 的逐帧精确时刻。

这不是 `float`/JSON serialization 精度问题，而是 production input dataflow 中一个未记录的时间量。

## 8. Best-effort timing sensitivity check

为了确认剩余误差确实受该缺失时间量主导，在**未读取 covariance、未使用 Ground Truth**的前提下，只对所有 episode 统一施加 odometry ROS receipt-time 常量偏移，扫描：

```text
+1.0 / +1.5 / +2.0 / +2.5 / +3.0 ms
```

全矩阵 worst-episode P95：

| uniform correction | linear v m/s | linear a m/s² | angular v rad/s | angular a rad/s² |
| --- | ---: | ---: | ---: | ---: |
| +1.0 ms | 0.012605 | 0.046176 | 0.001386 | 0.001994 |
| +1.5 ms | 0.009173 | 0.032848 | 0.001013 | 0.001400 |
| +2.0 ms | 0.006022 | 0.020295 | 0.000648 | 0.000889 |
| +2.5 ms | 0.004335 | 0.022957 | 0.000279 | 0.000426 |
| +3.0 ms | 0.006437 | 0.031037 | 0.000583 | 0.000866 |

一个统一毫秒级 shift 能让四类 origin error 同时显著下降，支持“缺失 callback receipt-time history”是当前 replay 分叉主因；但即使选择最有利的 shift，误差仍远高于预注册的 `1e-6` P95 门。

本阶段没有把该 shift 固化成 replay 参数，也没有据此放宽 equivalence tolerance，因为它是从 production output 反推的 nuisance calibration，不能替代真实 causal timing input。

## 9. Covariance source 与停止边界

production 代码已确认 covariance 来源：

```text
translation acceleration covariance
= DeckMotionEstimate.translation_covariance[6:9, 6:9]
```

translation quadratic fit valid 后，`make_estimate()` 会把 `fitted_kinematic_covariance_` 写入 velocity/acceleration block。

rotation angular-acceleration covariance 为：

```text
DeckMotionEstimate.rotation_covariance[6:9, 6:9]
```

但由于 replay equivalence 未通过，本阶段 **没有提取、评分或解释这些 covariance**，也没有生成：

```text
uncertainty vs acceleration-error Pearson/Spearman
uncertainty quintile bins
tail capture
static/dynamic confidence comparison
```

因此 translation confidence hypothesis 与 rotation confidence hypothesis 当前都只能标记：

```text
NOT TESTED — blocked by replay equivalence
```

不能归类为 Confidence-Supported，也不能归类为 Confidence-Rejected。

## 10. 下一最小实验

下一任务不是 covariance-derived confidence A/B，而是先让 production replay input 可观测。

最小方案是在 shadow-only 诊断路径记录 estimator 真正消费的 causal update input / timing，例如标准消息组合记录：

```text
image sample time
relative_deck_pose_ned
image-time uav_velocity_ned
marker_id
VehiclePoseHistory mapped sample timing provenance
```

或至少记录 `/clock` 与 `vehicle_odometry_callback()` 使用的 ROS receipt time / mapped odometry sample time。该诊断不得接入 controller，也不得改变 estimator 算法或参数。

由于 2026-08-15 固定 Bag 已经缺失上述量，不能通过修改离线脚本把它事后无损恢复。只有新的 replay-observable 证据先证明同一 production math 可以达到冻结 equivalence gate，才允许再次进入 covariance confidence scoring。

## 11. 安全边界

本阶段保持：

```text
production DeckMotionEstimator 未修改
px4_aruco_landing_node 未修改
px4_aruco_landing.yaml 未修改
Planar Board / RefineLM / SUBPIX 未修改
shadow hard gates 未修改
controller / descent / contact / NMPC 未修改
```

没有运行新的 SITL。Ground Truth 没有进入 estimator、replay candidate 或 controller。
