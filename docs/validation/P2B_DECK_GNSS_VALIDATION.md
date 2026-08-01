# P2B 船舶 GNSS 传感器仿真验收记录

## 1. 验收范围

本阶段完成甲板 Ground Truth 到船舶 GNSS / 遥测输出的仿真链路：

```text
/simulation/deck/ground_truth  Gazebo world ENU
              ↓
deck_gnss_simulator
  - 固定频率采样
  - WGS84 地理转换
  - 固定种子高斯噪声
  - 固定延迟队列
  - 可配置丢包
  - reset 后确定性复现
              ↓
/deck/gps/fix       NavSatFix
/deck/gps/velocity  TwistStamped, world_enu
```

本阶段没有修改降落控制器，控制器仍未订阅 Ground Truth 或船舶 GNSS。

## 2. 实现文件

```text
src/moving_deck_sim/
├── include/moving_deck_sim/gnss_sensor_model.hpp
├── src/gnss_sensor_model.cpp
├── src/deck_gnss_simulator.cpp
├── config/gnss_ideal.yaml
├── config/gnss_noisy.yaml
├── launch/deck_gnss_sim.launch.py
└── test/gnss_sensor_model_test.cpp
```

同时扩展：

```text
src/moving_deck_sim/src/moving_deck_controller.cpp
src/moving_deck_sim/launch/moving_deck_sim.launch.py
```

`moving_deck_controller` 在每次成功初始化或 reset 后发布瞬态本地
`/simulation/episode/reset_count`。GNSS 节点使用该序号清空延迟队列、重置采样相位并恢复随机数种子。

## 3. 坐标与时间语义

- 输入位置、速度：Gazebo world ENU，单位为米和米每秒。
- GNSS 位置：WGS84 纬度、经度、海拔。
- GNSS 速度：`world_enu`，`x=East`、`y=North`、`z=Up`。
- `NavSatFix.position_covariance`：ENU 顺序，单位为平方米。
- 输出 `header.stamp`：原始 GNSS 采样时刻，不是延迟后的发布时间。
- 地理原点：与 `aruco_moving_deck.sdf` 的 `<spherical_coordinates>` 一致。

地理转换调用 Gazebo `SphericalCoordinates::PositionTransform()`，使用
`LOCAL2 → SPHERICAL`，避免旧 `LOCAL` 包装接口的已知兼容性问题。

## 4. 单元测试

`gnss_sensor_model_test` 覆盖：

1. WGS84 原点以及 East、North、Up 符号方向。
2. 固定频率采样。
3. 固定延迟和样本顺序。
4. 固定随机种子输出一致。
5. reset 后噪声序列重复。
6. 丢包概率边界 `0` 和 `1`。
7. NaN Ground Truth 不生成新测量。
8. reset 清空旧延迟队列。
9. NavSatFix 协方差与配置一致。
10. 非法频率、噪声、延迟、概率和地理原点被拒绝。

构建与测试结果：

```text
moving_deck_sim build: passed
motion_profile_test: 5 passed
gnss_sensor_model_test: 9 passed
```

## 5. 端到端仿真冒烟测试

验证环境：

```text
Gazebo Harmonic
ROS 2 Humble
moving_deck scenario: S1_CONSTANT_XY
velocity_xy: [0.4, 0.0] m/s in world ENU
gnss config: gnss_ideal.yaml
```

启动：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash
source ~/ws_aruco_landing/install/setup.bash
source ~/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh

SHARE=$(ros2 pkg prefix --share moving_deck_sim)
ros2 launch moving_deck_sim moving_deck_sim.launch.py \
  headless:=true \
  gnss_config_file:=$SHARE/config/gnss_ideal.yaml
```

实际确认话题：

```text
/deck/gps/fix
/deck/gps/velocity
/simulation/deck/ground_truth
/simulation/episode/reset_count
```

实测 GNSS 发布频率：

```text
average rate: 5.000 Hz
min period: 0.200 s
max period: 0.200 s
```

匀速场景中读取到：

```text
/deck/gps/velocity
frame_id: world_enu
linear.x: approximately 0.4 m/s
linear.y: 0.0 m/s
linear.z: 0.0 m/s
```

甲板沿 world ENU East 正方向运动时，`/deck/gps/fix.longitude` 随时间增加，符合坐标约定。
理想配置协方差为零，`NavSatFix.status=STATUS_FIX`，`service=SERVICE_GPS`。

调用：

```bash
ros2 service call /simulation/episode/reset std_srvs/srv/Trigger '{}'
```

返回成功，`reset_count` 从 `1` 增加到 `2`，GNSS 节点日志确认重置 episode 2。

## 6. Ground Truth 隔离

以下包仍无 `/simulation/deck/ground_truth` 引用：

```text
src/aruco_precision_landing_cpp
src/aruco_detector
```

只有 `moving_deck_sim` 的仿真传感器节点和未来评测器允许使用 Ground Truth。

## 7. 已知限制

- 当前 GNSS 噪声为独立同分布高斯白噪声，尚未实现慢变偏置或卫星几何变化。
- 丢包为单帧独立伯努利模型。
- 延迟为固定值，不包含抖动。
- 速度话题使用 `TwistStamped`，因此没有速度协方差字段。
- world WGS84 原点通过 YAML 配置，修改 world spherical origin 时必须同步修改 GNSS 配置。
- 本次 headless 验证中 Gazebo 报告本机缺少 `libGstCameraSystem.so`；该插件问题不影响甲板运动和 GNSS 话题，但后续视觉联合验证前需要单独处理。

## 8. 阶段结论

P2B 的代码、纯模型测试和理想配置端到端冒烟测试通过。下一阶段为 P2C：

```text
船舶 GNSS + PX4 local/global reference
→ WGS84 转 PX4 local NED
→ WAIT_DECK_GNSS
→ RENDEZVOUS_GNSS
→ ACQUIRE_ARUCO
```

P2C 只实现安全高度会合和移动甲板上方粗跟踪，不下降，不接入视觉精降。
