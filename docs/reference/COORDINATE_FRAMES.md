# 坐标系与时间契约

本文档定义控制器、仿真、视觉和离线评测共同遵守的坐标与时间语义。业务代码不得自行拼接符号转换。

## 1. 坐标系

| 坐标系 | 轴方向 | 用途 |
| --- | --- | --- |
| `camera_optical` | x 右、y 下、z 前 | OpenCV、ArUco PnP。 |
| `base_link_frd` | x 前、y 右、z 下 | PX4 机体坐标。 |
| `local_ned` | x 北、y 东、z 下 | PX4 本地位置、速度和 setpoint。 |
| `uav_centered_ned` | 状态采样时刻位于无人机参考点，三轴平行 `local_ned` | ArUco shadow 当前相对位姿。 |
| `uav_origin_ned` | 轨迹发布时冻结在无人机参考点，三轴平行 `local_ned` | 甲板预测轨迹与未来相对 MPC 输入。 |
| `deck_landing_up` | x 甲板前、y 甲板左、z 甲板上 | 统一 Marker 后的甲板着陆参考系和 6-DoF shadow。 |
| `vessel_body` | marine WAM-V canonical 刚体参考系；与官方 `wamv/base_link` 轴语义对齐 | marine `MotionProfile` 驱动参考点，neutral world z≈0.2 m。 |
| `landing_deck` | x 甲板前、y 甲板左、z 甲板上 | marine Gazebo 固定甲板 frame，原点为着陆平面中心。 |
| Gazebo world ENU | x 东、y 北、z 上 | Gazebo 模型和 Ground Truth。 |
| WGS84 | 纬度、经度、椭球高 | 船舶 GNSS 和 PX4 地理参考。 |

所有距离使用米、速度使用米每秒、角速度使用弧度每秒；只有明确带 `_deg`/`_degps` 的参数和诊断使用度。

## 2. 刚体变换约定

`T_A_B` 表示把 B 坐标中的点变换到 A：

```math
p^A = R_A^B p^B + t_A^B
```

组合顺序为：

```math
T_A_C = T_A_B T_B_C
```

逆变换为：

```math
R_B^A = (R_A^B)^T
t_B^A = -(R_A^B)^T t_A^B
```

四元数统一归一化并检查有限性。范数过小、旋转矩阵非正交或输入含 NaN/Inf 时返回失败，不自动修补。

## 3. Marker 和甲板着陆系到 local NED

ArUco PnP 提供：

```text
T_camera_optical_marker
```

相机安装参数提供：

```text
T_body_frd_camera_optical
```

PX4 `VehicleOdometry` 提供：

```text
T_local_ned_body_frd
```

每个 Marker 的标定提供 `T_marker_deck_landing_up`。当前仿真四个 Marker 共面、方向
一致，检测器只需用各自平移把原点补偿到甲板中心，旋转为单位旋转。最终链路：

```text
T_local_ned_deck_landing_up
= T_local_ned_body_frd
* T_body_frd_camera_optical
* T_camera_optical_marker
* T_marker_deck_landing_up
```

相对 shadow 不使用 `T_local_ned_body_frd` 的平移。定义 `U=uav_centered_ned`，则：

```math
t_U^D=R_N^B t_B^D=p_D^N-p_B^N,
\qquad
R_U^D=R_N^B R_B^D=R_N^D
```

其中 `T_B^D=T_B^C T_C^M T_M^D`。`U` 的原点位于图像采样时刻的无人机参考点，
轴与 `local_ned` 平行，因此当前平移表示 `deck-uav`，不是甲板绝对位置。相邻
样本的原点不同，估计器必须用时间对齐的 PX4 NED 速度积分补偿无人机自运动；禁止
把 PX4 绝对位置加回 shadow。

预测发布时另定义 `O_t=uav_origin_ned`：原点固定在发布时刻 `t` 的无人机参考点，
在该条轨迹的整个 `0–1.0 s` 时域内不再随无人机移动。因此轨迹点平移为
`p_D(t+τ)-p_B(t)`，twist 为甲板自身相对 NED 的 twist。`O_t` 只在单条消息内有
意义，消息 header 时间戳给出其冻结时刻。

实现前必须确认 `VehicleOdometry.pose_frame` 是 NED；不支持的 frame 直接拒绝。相机外参参数表达机体到相机的明确方向，不得根据结果“看起来正确”临时取逆。

## 4. Marker 坐标和甲板法向

`deck_landing_up` 的 +Z 为甲板上方法向。旋转到 local NED 后得到单位向量：

```math
n_{up}^N = R_N^D [0,0,1]^T
```

有效向上法向应满足：

```math
||n_{up}^N|| = 1,
\qquad n_D < 0
```

多尺度 Marker 切换前必须先应用完整 `T_M^D`，使位置、朝向和 yaw 都属于同一甲板
参考系。当前仿真 Marker 朝向相同；若实物安装朝向不同，必须标定旋转，不能只补偿平移。
法向先与上一有效法向统一半球，再低通并归一化；向上分量不足时输出无效。

远距非共面目标中，ID 0 的 `T_D^M` 为单位旋转和平移
`[+0.45,0,0]^T m`；ID 4 为绕 `D` 系 `+Y` 旋转 `+45°`和平移
`[-0.75,0,+0.2852]^T m`；ID 5/6 分别绕 `+X` 旋转 `+45°/-45°`，平移为
`[0,+0.75,+0.2852]^T m` 和 `[0,-0.75,+0.2852]^T m`。ID 4/5/6 的边长
均为 `0.75 m`。板式 PnP 直接输出
`T_C^D`，因此不再追加单 Marker 的中心平移补偿。

## 5. Marine vessel → landing deck

legacy `moving_deck` 的模型原点就是甲板中心，因此仿真 raw GT 与 deck GT 重合。marine 则明确分离：

```text
T_world_vessel
* T_vessel_deck
= T_world_deck
```

Marine M2 固定：

```text
T_vessel_deck.translation = [0, 0, 1.8] m
T_vessel_deck.rotation = I
```

该 offset 来自 VRX WAM-V canonical base geometry 与新增 2.4×2.4 m landing platform：official `top_base` 顶面约 z=1.30 m，固定支撑将 landing plane 提升到 z=1.80 m。scenario neutral deck center 仍定义为 world z=2.0 m，因此 neutral vessel reference 为 z=0.2 m。

若 `r_{VD}^V` 为 vessel body 中的 deck offset，输入线速度在 world ENU、角速度在 vessel body 中，则：

```math
p_D^W=p_V^W+R_W^V r_{VD}^V
```

```math
v_D^W=v_V^W+R_W^V(\omega_V^V\times r_{VD}^V)
```

因此 roll/pitch 会产生 deck center 的 lever-arm 位移和速度；不能只把 vessel 姿态复制到 deck 而保持 deck 位置不动。该转换集中在 `moving_deck_sim/rigid_body_kinematics`，不得散落在 ROS callback 中。marine `/simulation/deck/ground_truth_raw` 表示 vessel raw state，最终 `/simulation/deck/ground_truth` 始终表示 landing deck center。

完整公式、有限性检查和 fixed rotation 语义见 [MARINE_VESSEL_KINEMATICS.md](MARINE_VESSEL_KINEMATICS.md)。

## 6. ENU 与 NED

位置和线速度使用同一线性映射：

```math
\begin{bmatrix}N\\E\\D\end{bmatrix}
=
\begin{bmatrix}
0&1&0\\
1&0&0\\
0&0&-1
\end{bmatrix}
\begin{bmatrix}E\\N\\U\end{bmatrix}
```

即：

```text
N = ENU.y
E = ENU.x
D = -ENU.z
```

姿态必须通过完整旋转矩阵或四元数基变换，禁止只交换 Euler 角或手写 roll/pitch 正负号。

## 7. WGS84 与本地坐标

控制器使用 PX4 `VehicleLocalPosition` 的：

```text
ref_lat
ref_lon
ref_alt
xy_global
z_global
```

建立局部地理参考。转换集中在 `geodetic_converter`：

```text
WGS84 → ECEF → local ENU → local NED
```

输入必须检查：

- 纬度在 `[-90°, 90°]`。
- 经度在 `[-180°, 180°]`。
- 高度和所有中间量有限。
- PX4 地理参考有效。
- 目标在局部线性化允许的范围内。

船舶 GNSS 速度已经是 ENU，不执行 WGS84 转换，只做 ENU/NED 轴变换。

## 8. 相对高度

NED z 向下，因此无人机在甲板上方时：

```math
h_{rel}=z_{deck}^{NED}-z_{uav}^{NED}>0
```

给定相对高度参考 `h_ref`：

```math
z_{sp}^{NED}=z_{deck,pred}^{NED}-h_{ref}
```

该量是世界竖直方向的相对高度。水平或纯升沉甲板上它与法向距离一致；倾斜甲板上只作为现有生产通道，甲板平面与滑橇法向间隙另行诊断，不能混为同一量。

## 9. 甲板平面

甲板参考点为 `p_d^N`，向上法向为 `n^N`。平面方程：

```math
(n^N)^T(p^N-p_d^N)=0
```

任一点的有符号法向间隙：

```math
h(p)=(n^N)^T(p^N-p_d^N)
```

无人机四个滑橇端点从 FRD 通过机体姿态变换到 NED 后分别计算 `h_i`。`min(h_i)` 表示最近接触端点；Ground Truth 平面只允许在 evaluator 中计算误差，控制器使用视觉法向与内部估计甲板位置。

## 10. 时间域

系统同时存在：

- ROS/Gazebo 仿真时间。
- PX4 消息微秒时间戳。
- 图像 Header 时间戳。

控制器维护 PX4→ROS 时间映射，并按 `VehicleOdometry.timestamp_sample` 保存机体位姿历史。处理图像时：

1. 校验图像时间有限且不倒退。
2. 将目标采样时间映射到 PX4/ROS 一致时间域。
3. 在相邻机体位姿间线性插值位置、SLERP 插值姿态。
4. 生产绝对位置链使用完整插值位姿；相对 shadow 使用同一插值姿态和 NED
   速度，将 `base_link_frd` 相对位姿旋转到 `uav_centered_ned` 并补偿观测原点移动。

超出历史范围、时间间隔过大或映射未稳定时拒绝观测，不能退化为使用当前位姿。

## 11. 模块边界

- `coordinate_transform`：生产视觉链的刚体组合、逆变换、ENU/NED 和姿态基变换。
- `moving_deck_sim/rigid_body_kinematics`：仿真 vessel reference 到 landing deck 固定点的 pose/twist 转换和 lever-arm 速度。
- `geodetic_converter`：WGS84、ECEF、ENU 和 NED。
- `VehiclePoseHistory`：时间有序位姿与速度缓存和插值；相对 shadow 消费其姿态和 NED 速度。
- `DeckPlaneGeometry`：视觉甲板平面、滑橇间隙和法向/切向分解。

新增坐标或时间逻辑必须进入上述共享模块，并覆盖单位、方向、异常输入和已知数值例测试。
