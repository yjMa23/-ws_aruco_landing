# P1 水平移动甲板仿真验收记录

## 1. 验收结论

验证日期：2026-07-19。

P1 水平移动甲板仿真通过本记录中的构建、轨迹、Ground Truth、重置、仿真时间和
Marker 刚性随动检查。降落控制器和 ArUco 检测器均未订阅
`/simulation/deck/ground_truth`，P1 没有修改静态降落状态机或控制算法。

## 2. 验证环境

| 项目 | 版本 |
| --- | --- |
| 操作系统 | Ubuntu 22.04.5 LTS |
| ROS 2 | Humble，`ros-humble-ros-base 0.10.0-1jammy.20260423.142225` |
| Gazebo Sim | 8.14.0 |
| OpenCV | 4.5.4 |
| PX4 | `v1.18.0-alpha1-371-g6f5be87b4c-dirty` |

PX4 工作树在验收前已经是 dirty 状态；本次验收未修改 PX4 源码或参数。

## 3. 改动范围与 Ground Truth 隔离

P1 相对 `baseline-static-v0.1` 新增 `src/moving_deck_sim/` 并更新根 README，
`src/aruco_precision_landing_cpp/` 无差异。以下检查无输出：

```bash
rg -n --fixed-strings '/simulation/deck/ground_truth' \
  src/aruco_precision_landing_cpp src/aruco_detector
```

公开 Ground Truth 保持为：

```text
/simulation/deck/ground_truth  nav_msgs/msg/Odometry
header.frame_id: world
child_frame_id: moving_deck
```

Gazebo 原始里程计通过内部话题 `/simulation/deck/ground_truth_raw` 进入仿真控制节点。
启动或 reset 成功时立即发布确定的初始状态；随后 0.1 s 内替换 Gazebo 位姿差分在
teleport 后产生的速度尖峰，之后透传原始里程计。该处理不进入降落控制器。

## 4. 构建与自动测试

执行：

```bash
source /opt/ros/humble/setup.bash
if [ -f "$HOME/ws_sensor_combined/install/setup.bash" ]; then
  source "$HOME/ws_sensor_combined/install/setup.bash"
fi

colcon build --symlink-install \
  --packages-select aruco_detector aruco_precision_landing_cpp moving_deck_sim \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

source install/setup.bash
colcon test \
  --packages-select aruco_detector aruco_precision_landing_cpp moving_deck_sim
colcon test-result --verbose
```

结果：三个包构建成功，`6 tests, 0 errors, 0 failures, 0 skipped`。
`moving_deck_sim` 注册了 `motion_profile_test`，包含：

- 静止轨迹。
- 匀速位置和速度。
- 正弦位置和速度。
- `t=0` 与重复 reset 的确定初始状态。
- 未知场景、非法周期、NaN、Inf、非正更新频率和非法时间。

## 5. 三种运动场景

启动模板：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source ~/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh

SHARE=$(ros2 pkg prefix --share moving_deck_sim)
ros2 launch moving_deck_sim moving_deck_sim.launch.py \
  headless:=true config_file:=$SHARE/config/<scenario>.yaml
```

每种场景均运行超过 30 s。

| 场景 | 实际参数 | 测量结果 |
| --- | --- | --- |
| 静止 | `p0=(0,0,2) m` | `x/y/z` 和单位姿态不变，线速度与角速度均为 0 |
| 匀速 | `p0=(0,0,2) m`，`v=(0.4,0) m/s` | `t: 12.10→44.12 s`，`x: 4.8384→17.6464 m`；`Δx=12.808 m=0.4×32.02 s` |
| 正弦 | `A=(1.0,0.5) m`，`T=(10,6) s`，相位为 0 | 32.04 s 外推误差：`x=-4.48 mm`、`y=+1.22 mm`，速度误差均小于 `0.6 mm/s` |

三个场景的 Ground Truth 均稳定在约 50.000 Hz。静止场景时间戳从 15.00 s
增加到 47.04 s；所有记录中时间戳持续递增，姿态保持单位四元数，`z=2.0 m`。

## 6. 重置与仿真时间

正弦 YAML 和随机种子 `1` 下连续执行两次：

```bash
ros2 service call /simulation/episode/reset std_srvs/srv/Trigger '{}'
```

两次服务均返回 `success=True`。预先建立订阅后捕获到的 reset 初始帧为：

```text
position = (0.0, 0.0, 2.0) m
orientation = (0.0, 0.0, 0.0, 1.0)
linear velocity = (0.628318530718, 0.523598775598, 0.0) m/s
```

去除时间戳后，两次初始帧 `diff -u` 无差异。reset 前后连续记录 5 s、212 条消息，
时间戳非递增计数为 0，最大速度分别为 `|vx|=0.628318530718 m/s`、
`|vy|=0.523598775598 m/s`，未再出现 teleport 速度尖峰。

暂停 Gazebo 3 s 墙钟时间，两次 `/world/aruco/stats` 均为：

```text
paused: true
sim_time: 482.432 s
iterations: 120608
```

因此运动相位使用 Gazebo 仿真时间；暂停时仿真时间、迭代次数和甲板轨迹均不推进。

## 7. Marker 刚性随动与 ArUco 接口

Marker visual、甲板 visual 和甲板 collision 均定义在同一个 `moving_deck` link 中，
Marker 相对甲板表面中心的位姿固定为 `(0,0,0.001)`，不存在独立 Marker 模型。

使用 PX4 `gz_x500_mono_cam_down` 下视相机完成 31 s 正弦场景联调。SITL 起飞至默认
2.5 m 后安全降落并自动解锁。测量结果：

| 项目 | 结果 |
| --- | --- |
| 甲板 Ground Truth | 1510 帧，`x=[-0.993618,1.006377] m`，`y=[-0.494929,0.505066] m` |
| `/aruco/visible` | 903 帧 true，0 帧 false |
| `/aruco/pose` | 903 帧，约 30 Hz，位姿随移动甲板连续变化 |
| `/aruco/debug_image` | 约 29 Hz |

原有三个接口保持不变：

```text
/aruco/pose         geometry_msgs/msg/PoseStamped
/aruco/visible      std_msgs/msg/Bool
/aruco/debug_image  sensor_msgs/msg/Image
```

## 8. 已知限制与告警

- P1 只包含静止、水平匀速和零初相位水平正弦运动，不包含升沉、横摇、纵摇、风扰或随机相位。
- 无界面运行时出现 `libGstCameraSystem.so`、EGL 和 PX4 SDF 扩展字段告警；PX4 相机仍稳定发布约 30 Hz，未影响本次视觉验收。
- 公共 Ground Truth 在启动/reset 后 0.1 s 内使用解析速度规避 Gazebo 原生里程计的位姿差分尖峰；位置始终来自 Gazebo。
