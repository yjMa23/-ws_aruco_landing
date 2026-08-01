# P2C GNSS 会合与移动甲板上方粗跟踪验收记录

验证日期：2026-07-20

## 1. 验收范围

本阶段实现并验证：

```text
WAIT_DECK_GNSS
→ RENDEZVOUS_GNSS
→ ACQUIRE_ARUCO
```

包括：

- 船舶 WGS84 GNSS 转 PX4 local NED。
- 船舶 ENU 速度转 NED。
- 位置和速度独立新鲜度、稳定时间校验。
- GNSS 水平跳变拒绝。
- 会合目标速度和单周期步长限幅。
- 实时船舶 GNSS 中心搜索。
- 稳定 ArUco 后停止搜索并回到 GNSS 中心悬停。
- GNSS 超时后锁定当前水平位置并回到等待状态。
- 状态转换原因日志。

本阶段明确不包含：

- GNSS 到视觉位置控制接管。
- 完整 ArUco 位姿接入控制器。
- 视觉状态估计和预测。
- 下降、触地或自动 Land。

## 2. 新增实现

```text
src/aruco_precision_landing_cpp/
├── include/aruco_precision_landing_cpp/gnss_rendezvous_guidance.hpp
├── src/gnss_rendezvous_guidance.cpp
└── test/gnss_rendezvous_guidance_test.cpp
```

控制节点新增订阅：

```text
/deck/gps/fix       sensor_msgs/msg/NavSatFix
/deck/gps/velocity  geometry_msgs/msg/TwistStamped
```

新增调试输出：

```text
/landing/deck_gnss_pose_ned  geometry_msgs/msg/PoseStamped
/landing/guidance_source     std_msgs/msg/String
```

控制器未新增或使用任何 Gazebo Ground Truth 输入。

## 3. 坐标和时间契约

### 3.1 地理参考

船舶 WGS84 转 local NED 的参考原点来自：

```text
VehicleLocalPosition.ref_lat
VehicleLocalPosition.ref_lon
VehicleLocalPosition.ref_alt
```

且要求：

```text
xy_global = true
z_global = true
```

### 3.2 船舶速度

```text
输入：world_enu = [East, North, Up]
输出：local_ned = [North, East, Down]
```

### 3.3 高度策略

普通船舶 GNSS 高度只进入调试估计，不控制会合高度。P2C 始终使用：

```text
target_z_ned = -rendezvous_altitude_m
```

默认：

```text
rendezvous_altitude_m = 5.0 m
```

### 3.4 新鲜度时间

当前 PX4 时间与 Gazebo `/clock` 尚未统一。P2C 使用控制器回调到达时间判断 GNSS
新鲜度和稳定持续时间，避免跨时间域直接相减。原始采样时间仍保留在输入消息中。

## 4. 单元测试

`gnss_rendezvous_guidance_test` 共 9 项：

1. 缺少 PX4 地理参考时拒绝船舶位置。
2. WGS84、ENU、NED 位置和速度方向正确。
3. 位置和速度必须连续稳定且保持新鲜。
4. 速度晚到时使用独立稳定时间窗口。
5. 新鲜 GNSS 大跳变被拒绝，超时后允许重新建立基准。
6. PX4 地理参考变化后清空旧船舶状态。
7. 目标同时受最大速度和单周期步长限制。
8. 搜索序列为中心、北、东、南、西并循环。
9. NaN、Inf、非法参数和非法时间被拒绝。

包级测试结果：

```text
coordinate_transform_test       10 passed
geodetic_converter_test          8 passed
gnss_rendezvous_guidance_test    9 passed
```

## 5. 合成 PX4 消息状态机冒烟测试

该测试不驱动真实 PX4 动力学，只验证 ROS 接口、状态转换、坐标转换和目标生成。

输入：

- `VehicleStatus`：Offboard、Armed。
- `VehicleOdometry`：单位姿态、`POSE_FRAME_NED`。
- `VehicleLocalPosition`：先 `z=-3 m`，后切换为 `z=-5 m`。
- PX4 WGS84 参考：

```text
lat = 47.397971057728974 deg
lon = 8.546163739800146 deg
alt = 2.2 m
```

- 船舶 GNSS 水平位置位于 PX4 local 原点。
- 船舶速度为 ENU 零速度。

### 5.1 正常状态链

实测：

```text
INIT
→ WAIT_FOR_PX4
→ OFFBOARD_PRE_STREAM
→ ARM_AND_TAKEOFF
→ WAIT_DECK_GNSS
→ RENDEZVOUS_GNSS
→ ACQUIRE_ARUCO
```

状态转换日志均包含原因。

### 5.2 移动中心搜索

在没有 ArUco 时：

```text
state = ACQUIRE_ARUCO
guidance_source = GNSS_SEARCH
target_z = -5.0 m
```

采样时搜索目标位于船舶中心西侧约 `1 m`，符合中心、北、东、南、西循环序列。

### 5.3 稳定 ArUco

持续发布有效 `/aruco/pose` 和 `/aruco/visible=true` 后：

```text
state = ACQUIRE_ARUCO
guidance_source = GNSS_ARUCO_ACQUIRED_HOLD
target = approximately [0.0, 0.0, -5.0] local NED
```

控制器未切换到视觉位置控制，未进入下降状态。

### 5.4 GNSS 超时

停止发布船舶位置和速度，超过配置超时后：

```text
state = WAIT_DECK_GNSS
guidance_source = GNSS_WAIT
target = [current_uav_x, current_uav_y, -5.0]
```

测试输入中无人机水平位置为零，因此实测目标为：

```text
[0.0, 0.0, -5.0]
```

控制器没有继续追踪旧船舶目标。

### 5.5 真实移动甲板 GNSS 联合数据链

启动实际 `moving_deck_sim` 默认 `0.4 m/s` 东向匀速场景和理想 GNSS，PX4 飞行状态仍使用
合成消息，目的是验证：

```text
Gazebo 移动甲板
→ Ground Truth
→ deck_gnss_simulator
→ NavSatFix / ENU velocity
→ GNSS rendezvous guidance
→ local NED target
```

合成无人机固定在 `local_ned = [-5, 0, -5] m`，因此状态保持：

```text
RENDEZVOUS_GNSS
```

两次采样中，船舶 local NED East 和控制目标均持续增加：

```text
deck pose 1: approximately [0.00, 3.21,  0.20] m
deck pose 2: approximately [0.00, 5.61,  0.20] m

target 1:    approximately [-0.72, 2.74, -5.00] m
target 2:    approximately [-0.09, 5.66, -5.00] m
```

结果说明：

- Gazebo ENU 东向运动正确映射为 PX4 local NED East 正方向。
- 控制目标随移动甲板 GNSS 持续更新，而不是固定搜索点。
- 水平目标经过速度和单周期步长限制。
- 垂直目标始终保持 `-5.0 m`，普通 GNSS 高度没有驱动下降。

该测试仍未驱动 PX4 动力学，因此只证明完整数据链和目标生成正确，不证明无人机实际跟踪性能。

## 6. 安全检查

控制器和检测器目录检查：

```bash
grep -R "/simulation/deck/ground_truth" \
  src/aruco_precision_landing_cpp src/aruco_detector
```

期望且实测无输出。

默认参数：

```yaml
enable_auto_land: false
```

当前从 `INIT` 出发的 P2C 主路径只到达 `ACQUIRE_ARUCO`。旧静态下降状态代码暂时保留用于
历史基线参考，但当前主路径没有进入这些状态的转换。

## 7. 尚未完成的真实飞行验证

以下项目尚未由本记录声明通过：

- PX4 SITL 实际动力学下的自动解锁和起飞。
- 无人机真实跟随 `0.2 / 0.4 m/s` 移动甲板。
- XY 正弦甲板实际会合和搜索。
- 含噪、延迟和丢包 GNSS 下的实际飞行稳定性。
- 相机视场内真实 ArUco 捕获率。

原因是本次自动验收使用合成 PX4 状态验证控制逻辑，没有将其冒充为真实飞行结果。
这些项目应在 PX4、QGroundControl/心跳和相机插件环境完整后继续验证。

## 8. 阶段结论

P2C 代码和消息级状态机验收通过：

- 能够使用船舶 GNSS 生成受限 local NED 会合目标。
- 能够围绕移动 GNSS 中心搜索 ArUco。
- 稳定视觉后保持安全高度，不下降。
- GNSS 超时后停止追踪旧目标并回到等待状态。
- Ground Truth 隔离保持成立。

下一阶段为：

```text
P2D：ArUco 完整变换、GNSS—视觉平滑接管和下降前恢复
```
