# moving_deck_sim

`moving_deck_sim` 为 PX4 SITL + Gazebo Harmonic 提供可重复的水平移动甲板。
甲板包含 `5 m × 5 m` 碰撞面和位于中心的 `DICT_4X4_50 / ID 0 / 0.5 m`
Marker。运动使用 Gazebo 原生 `VelocityControl`，Ground Truth 使用原生
`OdometryPublisher`，控制器不会接收该真值。

## 构建

```bash
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash
colcon build --symlink-install --packages-select moving_deck_sim
source install/setup.bash
```

## 启动

PX4 需要连接由本包启动的 Gazebo world，因此使用 standalone 模式。在第一个终端启动
PX4；它会等待 Gazebo 就绪：

```bash
cd ~/PX4-Autopilot
PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD=aruco \
PX4_GZ_MODEL_POSE=0,0,2.2 \
make px4_sitl gz_x500_mono_cam_down
```

随后在第二个终端立即启动默认的 `0.4 m/s` 匀速场景。Gazebo 进程需要 PX4 的模型、
插件和 ArUco 纹理资源，因此必须先加载 PX4 生成的环境文件：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_aruco_landing/install/setup.bash
source ~/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh
ros2 launch moving_deck_sim moving_deck_sim.launch.py
```

launch 会将 Gazebo Transport 固定到 `GZ_IP=127.0.0.1`，与 PX4 的 `gz_*` make
目标保持一致；P1 默认仅支持同一台主机上的 PX4 与 Gazebo。

无界面运行：

```bash
ros2 launch moving_deck_sim moving_deck_sim.launch.py headless:=true
```

选择静止或水平正弦配置：

```bash
SHARE=$(ros2 pkg prefix --share moving_deck_sim)
ros2 launch moving_deck_sim moving_deck_sim.launch.py \
  config_file:=$SHARE/config/static.yaml
ros2 launch moving_deck_sim moving_deck_sim.launch.py \
  config_file:=$SHARE/config/sinusoidal_xy.yaml
```

主 launch 默认同时启动理想船舶 GNSS。切换为含噪、延迟配置，或临时关闭 GNSS：

```bash
ros2 launch moving_deck_sim moving_deck_sim.launch.py \
  gnss_config_file:=$SHARE/config/gnss_noisy.yaml
ros2 launch moving_deck_sim moving_deck_sim.launch.py enable_gnss:=false
```

只启动 GNSS 传感器节点时使用：

```bash
ros2 launch moving_deck_sim deck_gnss_sim.launch.py \
  config_file:=$SHARE/config/gnss_ideal.yaml
```

world 内部名称仍为 `aruco`，PX4 生成的相机话题继续使用：

```text
/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image
/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info
```

## 场景参数

| 参数 | 类型 | 说明 |
| --- | --- | --- |
| `world_name` | string | Gazebo world 名称，默认 `aruco`。 |
| `model_name` | string | 甲板模型名称，默认 `moving_deck`。 |
| `scenario` | string | `S0_STATIC`、`S1_CONSTANT_XY` 或 `S2_SINUSOIDAL_XY`。 |
| `initial_position_enu` | double[3] | 甲板表面中心初始 ENU 位置，单位为米。 |
| `velocity_xy` | double[2] | 匀速场景的 ENU XY 速度，单位为米每秒。 |
| `amplitude_xy` | double[2] | 正弦场景的 ENU XY 幅值，单位为米。 |
| `period_xy` | double[2] | 正弦场景的 XY 周期，单位为秒且必须大于零。 |
| `update_rate_hz` | double | 速度指令更新频率，必须为有限正数。 |
| `random_seed` | int | Gazebo 随机种子，范围为 `uint32`。 |

三个场景均从 `initial_position_enu` 开始。正弦场景使用：

```text
p_xy(t) = p0_xy + amplitude_xy * sin(2*pi*t/period_xy)
v_xy(t) = amplitude_xy * 2*pi/period_xy * cos(2*pi*t/period_xy)
```

所有数组元素必须为有限数。未知场景、NaN、Inf、非正周期或非正更新频率会使节点
启动失败，避免非法速度进入 Gazebo。

## Ground Truth 与重置

Gazebo 实际甲板状态发布到：

```text
/simulation/deck/ground_truth  nav_msgs/msg/Odometry
```

`header.frame_id` 为 `world`，`child_frame_id` 为 `moving_deck`，位置和速度均采用
Gazebo ENU。启动或重置成功时会立即发布确定的初始状态；随后 `0.1 s` 内使用解析轨迹
速度替换 Gazebo 位姿差分产生的瞬时尖峰，之后透传 Gazebo 原始里程计。该话题只能由
后续评测器使用，禁止接入降落控制器。

重置甲板：

```bash
ros2 service call /simulation/episode/reset std_srvs/srv/Trigger '{}'
```

重置会恢复甲板初始位姿、运动相位和随机种子，不重置 PX4，也不回拨 Gazebo 时钟。
每次成功初始化或重置还会通过瞬态本地话题发布递增序号：

```text
/simulation/episode/reset_count  std_msgs/msg/UInt32
```

GNSS 节点收到新序号后会清空延迟队列、重置采样相位和随机数引擎，使固定种子实验可重复。

## 船舶 GNSS / 遥测仿真

`deck_gnss_simulator` 是仿真传感器层，允许订阅甲板 Ground Truth，并仅向控制器暴露：

```text
/deck/gps/fix       sensor_msgs/msg/NavSatFix
/deck/gps/velocity  geometry_msgs/msg/TwistStamped
```

`NavSatFix` 使用 WGS84，`header.stamp` 保留原始采样时刻，位置协方差按 ENU
`East/North/Up` 排列。速度话题 `frame_id=world_enu`，线速度依次为 East、North、Up。
控制器可以使用这两个传感器话题进行远距离会合，但仍禁止订阅 Ground Truth。

地理转换使用 Gazebo `SphericalCoordinates::LOCAL2 → SPHERICAL`，world 原点参数必须与
`aruco_moving_deck.sdf` 的 `<spherical_coordinates>` 保持一致。

| GNSS 参数 | 说明 |
| --- | --- |
| `publish_rate_hz` | GNSS 采样频率，默认 `5 Hz`。 |
| `horizontal_noise_std_m` | ENU 水平位置高斯噪声标准差。 |
| `vertical_noise_std_m` | Up 方向位置高斯噪声标准差。 |
| `velocity_noise_std_mps` | 三轴 ENU 速度噪声标准差。 |
| `latency_ms` | 固定发布延迟；消息时间戳仍为采样时间。 |
| `packet_drop_probability` | 单帧独立丢包概率，范围 `[0,1]`。 |
| `random_seed` | 噪声和丢包随机种子。 |
| `world_origin.latitude_deg` | Gazebo world WGS84 原点纬度。 |
| `world_origin.longitude_deg` | Gazebo world WGS84 原点经度。 |
| `world_origin.elevation_m` | Gazebo world WGS84 原点海拔。 |

## 验证

```bash
ros2 topic hz /simulation/deck/ground_truth
ros2 topic echo /simulation/deck/ground_truth
ros2 topic hz /deck/gps/fix
ros2 topic echo /deck/gps/fix --qos-reliability best_effort --once
ros2 topic echo /deck/gps/velocity --qos-reliability best_effort --once
colcon test --packages-select moving_deck_sim
colcon test-result --verbose
```

P1/P2B 当前只覆盖水平甲板运动与 GNSS 传感器特性，不包含升沉、横摇、纵摇、风扰、
随机相位或视觉噪声。
