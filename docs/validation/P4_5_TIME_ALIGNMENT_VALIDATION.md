# P4.5 实验可复现与视觉时间对齐验收记录

## 1. 阶段范围

P4.5 在不修改 P4 跟踪控制律、不实现下降的前提下，完成：

- P4 rosbag 离线统一评测；
- SITL 节点仿真时钟统一；
- PX4 local NED 无人机位姿历史；
- PX4 时间戳到 ROS 时间域映射；
- 图像采样时刻的无人机位姿插值；
- ArUco 坐标变换时间对齐。

本阶段不实现着陆窗口、垂直下降、触地、MPC 或强化学习。

## 2. P4 离线评测入口

新增：

```text
scripts/evaluate_p4_bag.py
```

使用：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash
python3 scripts/evaluate_p4_bag.py bags/<bag_name>
```

脚本自动输出：

- 稳定统计时长；
- 水平位置 RMSE；
- 相对速度 RMSE；
- 最大水平误差；
- 预测位置 RMSE；
- Marker 丢失次数；
- GNSS 恢复次数；
- 最大水平速度和加速度；
- 最大滚转角和俯仰角。

Ground Truth 只在脚本内用于离线评测。`bags/` 已加入 `.gitignore`。

现有 rosbag 回归示例：

| Bag | 稳定时长 | 水平位置 RMSE | 相对速度 RMSE | 最大水平误差 |
| --- | ---: | ---: | ---: | ---: |
| `p4_static_20260723_210534` | 64.735 s | 0.0289 m | 0.0171 m/s | 0.0752 m |
| `p4_constant_20260723_210830` | 47.069 s | 0.0388 m | 0.0202 m/s | 0.0777 m |
| `p4_sinusoidal_20260723_210256` | 61.675 s | 0.4609 m | 0.4823 m/s | 0.6658 m |

静止和匀速结果与 P4 验收文档对应轮次一致。正弦场景不同 rosbag 轮次的统计值不同，现已可通过脚本明确复现具体轮次，不再依赖手工表格。

## 3. 仿真时钟统一

以下 launch 新增 `use_sim_time` 参数，默认值为 `false`：

```text
src/aruco_detector/launch/aruco_detector.launch.py
src/aruco_precision_landing_cpp/launch/px4_aruco_landing.launch.py
```

`scripts/start_sitl.sh` 在 SITL 中显式传入：

```text
use_sim_time:=true
```

因此：

- SITL 使用 Gazebo `/clock`；
- 真机或普通启动默认继续使用系统时钟；
- 不改变已有话题和控制行为。

## 4. 无人机位姿历史

新增纯 C++ 模块：

```text
include/aruco_precision_landing_cpp/vehicle_pose_history.hpp
src/vehicle_pose_history.cpp
test/vehicle_pose_history_test.cpp
```

功能：

- 保存最近可配置时长的 local NED 机体位姿；
- 位置线性插值；
- 四元数最短路径 Slerp；
- 有限端点保持；
- 严格拒绝重复、乱序和非法输入；
- 自动裁剪过旧历史；
- 支持 reset。

## 5. PX4 与 ROS 时间映射

`VehicleOdometry.timestamp` 用于估计 PX4 时间域到 ROS 时间域的偏移；
`VehicleOdometry.timestamp_sample` 用于得到实际位姿采样时刻，缺失时才回退到
`timestamp`。

偏移更新：

```text
offset_observed = ros_receipt_time - px4_publish_time
offset = offset + gain * (offset_observed - offset)
ros_sample_time = px4_timestamp_sample + offset
```

当 ROS 时间或 PX4 时间回退，或偏移发生超过阈值的跳变时，清空旧位姿历史，避免跨时间域插值。

新增参数：

```yaml
vehicle_pose_history.history_duration_s: 2.0
vehicle_pose_history.max_endpoint_hold_s: 0.03
vehicle_pose_history.clock_offset_filter_gain: 0.05
vehicle_pose_history.max_clock_offset_jump_s: 0.10
```

## 6. ArUco 时间对齐变换

修改前：

```text
图像采样时刻的 T_camera_marker
+
回调时最新 T_local_body
```

修改后：

```text
图像采样时刻
→ 查询 VehiclePoseHistory
→ 插值得到同一时刻 T_local_body
→ T_local_marker = T_local_body * T_body_camera * T_camera_marker
```

图像时间戳为零、位姿历史不足、时间超出端点保持范围时，该帧视觉测量会被拒绝，
不再使用接收时刻或最新里程计代替。

## 7. 测试结果

全工作区：

```text
103 tests
0 errors
0 failures
0 skipped
```

新增位姿历史测试覆盖：

- 精确时间命中；
- 平移插值；
- 四元数 Slerp；
- 相反符号等价四元数；
- 端点保持；
- 超时拒绝；
- 乱序、重复和非法输入；
- 历史裁剪；
- reset；
- 非法参数。

启动级检查：

- 降落控制节点能够加载新参数并进入 `WAIT_FOR_PX4`；
- ArUco 检测节点能够通过新增 `use_sim_time` 参数启动；
- 两个 launch 文件、评测脚本和 `start_sitl.sh` 语法检查通过。

## 8. 完整 PX4 SITL 回归

2026-07-24 在 QGroundControl、PX4 SITL、Gazebo Harmonic 和理想船舶 GNSS 下，
使用 P4.5 新时间对齐实现重新运行四个场景。每轮进入 `TRACK_TARGET` 后丢弃前 5 秒，
其余稳定数据由 `scripts/evaluate_p4_bag.py` 统一评测。

| 场景 | Bag | 稳定时长 | 水平位置 RMSE | 相对速度 RMSE | 最大水平误差 | 预测位置 RMSE |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 静止 | `p4_static_20260724_214044` | 91.596 s | 0.0271 m | 0.0172 m/s | 0.0722 m | 0.0263 m |
| 匀速 0.2 m/s | `p4_constant02_20260724_214815` | 89.214 s | 0.0382 m | 0.0174 m/s | 0.0771 m | 0.0317 m |
| 匀速 0.4 m/s | `p4_constant_20260724_214358` | 86.250 s | 0.0510 m | 0.0168 m/s | 0.1137 m | 0.0412 m |
| XY 正弦 | `p4_sinusoidal_20260724_215104` | 101.324 s | 0.4812 m | 0.5086 m/s | 0.7367 m | 0.1873 m |

飞行器动力学峰值：

| 场景 | 最大水平速度 | 最大水平加速度 | 最大滚转角 | 最大俯仰角 |
| --- | ---: | ---: | ---: | ---: |
| 静止 | 0.0418 m/s | 0.0783 m/s² | 0.423° | 0.409° |
| 匀速 0.2 m/s | 0.2421 m/s | 0.0753 m/s² | 0.472° | 0.422° |
| 匀速 0.4 m/s | 0.4274 m/s | 0.0654 m/s² | 0.375° | 0.578° |
| XY 正弦 | 1.3518 m/s | 1.1699 m/s² | 6.168° | 3.784° |

四轮均满足：

- 正常进入并持续保持 `TRACK_TARGET`；
- Marker 丢失次数为 0；
- GNSS 恢复次数为 0；
- 没有出现 `timestamp synchronization failed`；
- 没有出现 `pose history insertion failed`；
- 没有出现 `full camera-to-NED transform failed`；
- 没有出现非法 PX4 NED 位姿或跟踪输入不可用告警；
- 退出由预设定时 SIGINT 触发，所有子进程和 rosbag 均正常清理。

## 9. 阶段结论

P4.5 代码、单元测试、离线评测和完整 PX4 SITL 回归均已完成。时间对齐实现没有破坏
静止或匀速跟踪，0.4 m/s 场景位置 RMSE 从原验收的 `0.064 m` 降至 `0.051 m`。

XY 正弦场景仍保持约 `0.48 m` 的位置 RMSE，说明主要限制来自常速度预测和前馈参数，
而不是图像与机体位姿时间错位。下一阶段进入 P4.6 正弦运动参数优化，仍不进入 P5 下降。
