# P4.5 实验可复现与视觉时间对齐执行计划

## 1. 阶段目标

在不修改 P4 水平跟踪控制律、不实现 P5 下降的前提下，完成以下两项工作：

1. 冻结 P4 实验资产，使现有 rosbag 能够通过统一入口自动计算验收指标。
2. 修复 ArUco 图像采样时刻与 PX4 无人机位姿时刻不一致的问题，使视觉坐标变换使用图像采样时刻对应的机体位姿。

本阶段完成后，P4 应具备可重复评测能力，视觉测量应具备严格时间对齐基础，并可继续开展正弦运动调参。

## 执行状态（2026-07-24）

- Task 1：已完成，新增离线评测脚本并忽略 `bags/`。
- Task 2：已完成，SITL 启动链显式使用仿真时钟。
- Task 3：已完成，新增 `VehiclePoseHistory` 与单元测试。
- Task 4：已完成，加入 PX4→ROS 时间映射和跳变重置。
- Task 5：已完成，ArUco 变换改用图像采样时刻的插值机体位姿。
- Task 6：已完成，103 项测试、四场景完整 PX4 SITL 回归和日志检查均通过。

详细验收记录：

```text
docs/P4_5_TIME_ALIGNMENT_VALIDATION.md
```

---

## 2. 明确不做

本阶段不实现：

- 着陆窗口；
- 垂直下降；
- 触地检测；
- 自动 Land 或自动 Disarm；
- MPC；
- 强化学习；
- 常加速度预测或其他新控制律；
- 甲板升沉、横摇和纵摇场景。

---

## 3. 执行顺序

### Task 1：冻结 P4 实验资产

修改：

```text
.gitignore
scripts/start_sitl.sh
```

新增：

```text
scripts/evaluate_p4_bag.py
```

要求：

- `bags/` 默认不进入 Git；
- 评测脚本直接读取 ROS 2 bag；
- 自动查找 P4 所需话题；
- 丢弃首次进入 `TRACK_TARGET` 后的可配置过渡时间；
- 自动输出稳定统计时长、水平位置 RMSE、相对速度 RMSE、最大水平误差、预测位置 RMSE、Marker 丢失次数、GNSS 恢复次数、最大水平速度和最大水平加速度；
- Ground Truth 仅用于离线评测；
- 评测脚本不得进入控制节点依赖链；
- 错误输入应给出明确错误信息并返回非零退出码。

验证：

```bash
python3 scripts/evaluate_p4_bag.py --help
python3 scripts/evaluate_p4_bag.py bags/p4_static_predicted_acceptance
```

### Task 2：统一 SITL 仿真时钟

修改：

```text
src/aruco_detector/launch/aruco_detector.launch.py
src/aruco_precision_landing_cpp/launch/px4_aruco_landing.launch.py
scripts/start_sitl.sh
```

要求：

- 两个 launch 均声明 `use_sim_time` 参数；
- SITL 启动脚本显式传入 `use_sim_time:=true`；
- 真机或普通启动时默认仍为 `false`；
- 不改变现有话题名和控制行为。

验证：

```bash
python3 -m py_compile src/aruco_detector/launch/aruco_detector.launch.py
python3 -m py_compile src/aruco_precision_landing_cpp/launch/px4_aruco_landing.launch.py
```

### Task 3：实现无人机位姿历史

新增：

```text
src/aruco_precision_landing_cpp/include/aruco_precision_landing_cpp/vehicle_pose_history.hpp
src/aruco_precision_landing_cpp/src/vehicle_pose_history.cpp
src/aruco_precision_landing_cpp/test/vehicle_pose_history_test.cpp
```

修改：

```text
src/aruco_precision_landing_cpp/CMakeLists.txt
```

功能：

- 保存最近一段时间的 PX4 local NED 机体位姿；
- 输入时间使用统一的 ROS 时间秒；
- 位置使用线性插值；
- 姿态使用四元数 Slerp；
- 拒绝乱序、重复、NaN、Inf、非法四元数和非 NED 位姿；
- 查询时间位于历史范围内时插值；
- 距离端点不超过最大外推时间时允许端点保持；
- 历史不足或时间跨度过大时返回 `std::nullopt`；
- 限制最大历史时长，避免无界增长。

单元测试至少覆盖：

- 精确时间命中；
- 平移插值；
- 四元数 Slerp；
- 四元数符号相反但姿态相同；
- 最大端点保持；
- 超出允许范围拒绝；
- 乱序和重复输入拒绝；
- 非法输入拒绝；
- 历史裁剪；
- reset。

### Task 4：建立 PX4 时间到 ROS 时间映射

修改：

```text
src/aruco_precision_landing_cpp/include/aruco_precision_landing_cpp/px4_aruco_landing_node.hpp
src/aruco_precision_landing_cpp/src/px4_aruco_landing_node.cpp
src/aruco_precision_landing_cpp/config/px4_aruco_landing.yaml
src/aruco_precision_landing_cpp/README.md
```

设计：

- 每次接收 `VehicleOdometry` 时读取 `timestamp_sample`，若无效则退回 `timestamp`；
- 使用接收时 ROS 时钟减去 PX4 时间，估计 PX4→ROS 的时钟偏移；
- 对偏移进行有限幅低通更新，避免单帧抖动；
- 将 odometry 采样时间转换为 ROS 时间并写入 `VehiclePoseHistory`；
- 增加最大历史时长、最大端点保持时间、最大时钟偏移跳变和偏移滤波系数参数；
- PX4 时间戳无效时拒绝写入历史，不使用接收时刻伪装采样时刻。

参数初值：

```yaml
vehicle_pose_history.history_duration_s: 2.0
vehicle_pose_history.max_endpoint_hold_s: 0.03
vehicle_pose_history.clock_offset_filter_gain: 0.05
vehicle_pose_history.max_clock_offset_jump_s: 0.10
```

### Task 5：视觉坐标变换改用图像采样时刻位姿

修改：

```text
src/aruco_precision_landing_cpp/include/aruco_precision_landing_cpp/px4_aruco_landing_node.hpp
src/aruco_precision_landing_cpp/src/px4_aruco_landing_node.cpp
```

要求：

- `compute_marker_pose_ned` 接受图像采样时间；
- 使用 `VehiclePoseHistory` 查询对应机体位姿；
- 不再使用最新 `vehicle_odometry_` 完成视觉坐标变换；
- 图像时间戳为零、历史不足或时间差超限时拒绝该帧视觉测量；
- 估计器继续使用同一图像采样时间；
- 调试输出继续使用接收时刻发布，但数据语义来自图像采样时刻；
- 不修改 P4 跟踪模式和增益。

### Task 6：构建、测试和回归

执行：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash

colcon build --symlink-install \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

colcon test
colcon test-result --verbose
```

验收标准：

- 原有 93 项测试全部通过；
- 新增位姿历史测试全部通过；
- launch Python 文件可编译；
- 离线评测脚本可处理至少一个现有 P4 rosbag；
- P4 控制律、状态机路径和默认高度保持行为不变；
- 控制节点仍不订阅 Ground Truth；
- 工作区不存在意外生成或提交的大体积 bag 文件。

---

## 4. 后续阶段

本计划完成后，下一阶段才进行：

```text
正弦场景参数批量扫描
→ 确定统一默认预测时域和前馈增益
→ 扩展升沉与倾斜甲板仿真
→ P5 着陆窗口
→ P5 相对高度分阶段下降
```
