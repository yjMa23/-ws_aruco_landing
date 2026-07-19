# PX4 ArUco 精准降落控制公式说明

本文档对应 [`px4_aruco_landing_node.hpp`](../src/aruco_precision_landing_cpp/include/aruco_precision_landing_cpp/px4_aruco_landing_node.hpp) 和 [`px4_aruco_landing_node.cpp`](../src/aruco_precision_landing_cpp/src/px4_aruco_landing_node.cpp)，描述当前状态机真实使用的坐标变换、阈值和设定点更新公式。

## 1. 符号与坐标系

| 符号 | 含义 | 单位/代码对应 |
| --- | --- | --- |
| $(x_c,y_c,z_c)$ | Marker 在相机光学坐标系的位置 | m，`aruco_pose_.pose.position` |
| $(f_b,r_b)$ | Marker 相对机体的前向、右向误差 | m |
| $(e_N,e_E)$ | Marker 在 local NED 平面的 North、East 误差 | m，`error_north/east` |
| $(N,E,z)$ | PX4 当前 local NED 位置 | m，`local_position_.x/y/z` |
| $(N_{sp},E_{sp},z_{sp})$ | 发布给 PX4 的位置设定点 | m，`target_x/y/z_` |
| $\psi$ | 当前 NED 航向角 | rad，`current_yaw_` |
| $\Delta t$ | 本次状态机控制步长 | s，`dt` |
| $s_{max}$ | 每个水平轴的最大目标修正 | m，`max_xy_step_` |
| $v_{down}$ | NED `z` 方向的最大下降设定点速率 | m/s，`max_descent_rate_` |

相机光学轴为 `x` 向右、`y` 向下、`z` 向镜头前方。PX4 local NED 为 `x` 向 North、`y` 向 East、`z` 向 Down，因此离局部原点的高度近似为：

$$
h=-z
$$

例如飞行高度 3 m 对应的设定点约为 $z_{sp}=-3$ m；向地面下降时，$z$ 从负值逐渐增大到 0。

## 2. 参数与实际默认值

默认 launch 会加载 `config/px4_aruco_landing.yaml`，因此正常启动时以 YAML 列为准。

| 参数 | 源码声明默认值 | YAML 运行值 | 公式含义 |
| --- | ---: | ---: | --- |
| `control_rate_hz` | 20.0 | 20.0 | $f_c$ |
| `takeoff_alt` | 3.0 | 3.0 | $h_{takeoff}$ |
| `search_x` | 0.0 | 1.0 | $N_s$ |
| `search_y` | 0.0 | 1.0 | $E_s$ |
| `search_alt` | 3.0 | 3.0 | $h_s$ |
| `abort_hover_alt` | 3.0 | 3.0 | $h_{abort}$ |
| `offboard_prestream_count` | 20 | 20 | $n_{pre}$ |
| `stable_detect_count` | 10 | 10 | $n_{stable}$ |
| `camera_x_to_body_y_sign` | 1.0 | 1.0 | $s_x$ |
| `camera_y_to_body_x_sign` | -1.0 | -1.0 | $s_y$ |
| `max_xy_step` | 0.20 | 0.20 | $s_{max}$ |
| `center_xy_threshold` | 0.15 | 0.15 | $r_{center}$ |
| `max_descent_rate` | 0.20 | 0.20 | $v_{down}$ |
| `final_alt` | 0.30 | 0.30 | $h_{final}$ |
| `marker_lost_timeout` | 1.0 | 1.0 | $t_{lost}$ |
| `aruco_pose_timeout` | 0.5 | 0.5 | $t_{pose}$ |
| `takeoff_z_threshold` | 0.20 | 0.20 | $\epsilon_{takeoff}$ |
| `search_xy_threshold` | 0.25 | 0.25 | $r_s$ |
| `search_z_threshold` | 0.20 | 0.20 | $\epsilon_s$ |
| `command_retry_interval` | 1.0 | 1.0 | $t_{retry}$ |
| `enable_auto_land` | true | true | 是否发送最终 Land 命令 |
| `target_pose_frame_id` | `local_ned` | `local_ned` | 调试 Pose 的 `frame_id` |

参数校验要求所有高度、频率、阈值、限幅和超时为有限正数；两个计数至少为 1；搜索点 $N_s,E_s$ 有限；目标坐标系名称非空；$s_x,s_y\in\{-1,+1\}$；并且：

$$
h_{final}<h_{takeoff},\qquad h_{final}<h_s
$$

## 3. 控制周期与时间处理

定时器理论周期为：

$$
T_c=\frac{1}{f_c}
$$

代码使用 ROS 时钟计算原始间隔：

$$
\Delta t_{raw}=t_k-t_{k-1}
$$

若该值非有限或不为正，则先回退为 $T_c$：

$$
\Delta t_0=
\begin{cases}
T_c,& \Delta t_{raw}\le0\text{ 或非有限}\\
\Delta t_{raw},& \text{其他}
\end{cases}
$$

随后对所有情况统一限制异常长周期：

$$
\Delta t=\min(\Delta t_0,0.5)
$$

在默认 $f_c=20$ Hz 时，$T_c=0.05$ s。ROS 纳秒时间戳发布到 PX4 消息时转换为微秒：

$$
t_{\mu s}=\left\lfloor\frac{t_{ns}}{1000}\right\rfloor
$$

## 4. Marker 误差到 local NED 的变换

### 4.1 相机平面到机体平面

代码不使用 Marker 的相机深度 $z_c$ 或姿态，只使用 $x_c,y_c$：

$$
f_b=s_y y_c
$$

$$
r_b=s_x x_c
$$

默认 $s_y=-1,s_x=+1$，即相机图像向上对应机体前方，图像向右对应机体右方。符号参数保留了相机安装方向的现场校准入口。

### 4.2 机体平面到 NED

当前航向为 $\psi$ 时：

$$
\begin{bmatrix}
e_N\\e_E
\end{bmatrix}
=
\begin{bmatrix}
\cos\psi & -\sin\psi\\
\sin\psi & \cos\psi
\end{bmatrix}
\begin{bmatrix}
f_b\\r_b
\end{bmatrix}
$$

展开后与 `compute_local_marker_error()` 完全一致：

$$
e_N=\cos\psi\,f_b-\sin\psi\,r_b
$$

$$
e_E=\sin\psi\,f_b+\cos\psi\,r_b
$$

例：默认符号下，若 $(x_c,y_c)=(0.2,-0.3)$ m，则 $(f_b,r_b)=(0.3,0.2)$ m。

- $\psi=0$ 时：$(e_N,e_E)=(0.3,0.2)$ m；
- $\psi=\pi/2$ 时：$(e_N,e_E)=(-0.2,0.3)$ m。

## 5. 姿态四元数与航向角

PX4 `VehicleOdometry.q` 在代码中按 $(w,x,y,z)$ 读取。首先检查四个分量有限，并要求：

$$
n_q^2=w^2+x^2+y^2+z^2>10^{-12}
$$

通过后归一化：

$$
(\bar w,\bar x,\bar y,\bar z)=\frac{(w,x,y,z)}{\sqrt{n_q^2}}
$$

航向角为：

$$
\psi=operatorname{atan2}
\left(
2(\bar w\bar z+\bar x\bar y),
1-2(\bar y^2+\bar z^2)
\right)
$$

发布 `/landing/target_pose` 时，仅编码绕 `z` 轴的目标航向。ROS 四元数字段顺序为 $(x,y,z,w)$：

$$
q_{target}=\left(0,0,\sin\frac{\psi_{sp}}2,\cos\frac{\psi_{sp}}2\right)
$$

单位四元数 $(w,x,y,z)=(1,0,0,0)$ 得到 $\psi=0$，对应调试 Pose 的 $(x,y,z,w)=(0,0,0,1)$。

## 6. 可见性、稳定计数与超时

### 6.1 新鲜位姿

设控制器当前时刻为 $t$，最近一次 Pose 和 visible 消息的接收时刻分别为 $t_p,t_v$。`marker_is_fresh()` 的条件为：

$$
fresh=
visible
\land have\_pose
\land(t-t_p\le t_{pose})
\land(t-t_v\le t_{pose})
\land finite(x_c)
\land finite(y_c)
$$

这里使用回调执行时记录的 ROS 时钟，不使用消息 `header.stamp`；也不检查 $z_c$ 和 Marker 姿态。

### 6.2 连续可见计数

每收到一次 `/aruco/visible`：

$$
c_k=
\begin{cases}
\min(c_{k-1}+1,n_{stable}),& visible=true\\
0,& visible=false
\end{cases}
$$

从 `WAIT_ARUCO` 进入对中要求：

$$
fresh\land c_k\ge n_{stable}
$$

### 6.3 丢失与命令重试

最近有效 Marker 时刻为 $t_{seen}$。对中阶段满足
$t-t_{seen}>t_{lost}$ 时退回 `WAIT_ARUCO`；下降阶段满足相同条件时进入 `ABORT`。
Marker 暂时不新鲜但尚未超时时，代码不会更新水平或下降目标，而是继续发布上一目标。

Offboard 或 Arm 尚未成功时，命令重发条件为：

$$
retry=\neg have\_last\_command
\lor(t-t_{command}\ge t_{retry})
$$

## 7. 水平对中控制律

定义逐轴限幅函数：

$$
\operatorname{clamp}(e,-s_{max},s_{max})=
\begin{cases}
-s_{max},&e<-s_{max}\\
e,&|e|\le s_{max}\\
s_{max},&e>s_{max}
\end{cases}
$$

`CENTER_ABOVE_MARKER` 和下降阶段都从实测当前位置生成新的水平目标：

$$
N_{sp}=N+\operatorname{clamp}(e_N,-s_{max},s_{max})
$$

$$
E_{sp}=E+\operatorname{clamp}(e_E,-s_{max},s_{max})
$$

由于两个轴独立限幅：

$$
\sqrt{(N_{sp}-N)^2+(E_{sp}-E)^2}
\le\sqrt{2}\,s_{max}
$$

所以 `max_xy_step=0.20` m 时，单次目标更新的平面长度最多约为 $0.283$ m。该参数限制的是设定点偏移，不是无人机速度。

对中误差半径为：

$$
r_e=\sqrt{e_N^2+e_E^2}
$$

开始下降使用严格小于判据：

$$
r_e<r_{center}
$$

例如 $(e_N,e_E)=(0.50,-0.30)$ m、$s_{max}=0.20$ m 时，设定点修正为 $(+0.20,-0.20)$ m，而不是按向量长度等比例缩放。

## 8. 高度与下降设定点

对中阶段保持实测 NED 高度：

$$
z_{sp}=z
$$

下降阶段从上一周期目标高度积分：

$$
z_{sp,k+1}=\min(z_{sp,k}+v_{down}\Delta t,0)
$$

它使用上一目标 `target_z_`，不是实测 `local_position_.z`。因为 NED `z` 向下为正，增加 $z_{sp}$ 就是在下降。默认 $v_{down}=0.20$ m/s、$\Delta t=0.05$ s 时：

$$
\Delta z_{sp}=0.20\times0.05=0.01\text{ m}
$$

例如目标从 $-3.00$ m 更新到 $-2.99$ m。`min(...,0)` 防止设定点穿过 local NED 原点继续向下。

最终降落判断在本周期 Marker 处理之前执行：

$$
z\ge-h_{final}
$$

等价于：

$$
h=-z\le h_{final}
$$

默认 $h_{final}=0.30$ m，因此实测 $z\ge-0.30$ m 时进入 `FINAL_LAND`。

## 9. 起飞与搜索到达条件

起飞目标为：

$$
(N_{sp},E_{sp},z_{sp},\psi_{sp})
=(N_0,E_0,-h_{takeoff},\psi_0)
$$

进入搜索飞行还要求 PX4 已处于 Offboard、已经解锁，并满足：

$$
|z+h_{takeoff}|\le\epsilon_{takeoff}
$$

搜索目标为：

$$
(N_{sp},E_{sp},z_{sp},\psi_{sp})
=(N_s,E_s,-h_s,\psi_0)
$$

到达搜索区域的两个条件必须同时成立：

$$
d_s=\sqrt{(N-N_s)^2+(E-E_s)^2}\le r_s
$$

$$
|z+h_s|\le\epsilon_s
$$

## 10. 状态机条件汇总

| 状态 | 主要公式/条件 | 下一状态 |
| --- | --- | --- |
| `INIT` | 无条件 | `WAIT_FOR_PX4` |
| `WAIT_FOR_PX4` | 状态、位置、里程计齐全；`xy_valid && z_valid`；位置有限；四元数有效 | `OFFBOARD_PRE_STREAM` |
| `OFFBOARD_PRE_STREAM` | 预发送计数达到 $n_{pre}$，然后发送 Offboard 与 Arm | `ARM_AND_TAKEOFF` |
| `ARM_AND_TAKEOFF` | Offboard、Armed 且 $|z+h_{takeoff}|\le\epsilon_{takeoff}$ | `GOTO_ARUCO_AREA` |
| `GOTO_ARUCO_AREA` | $d_s\le r_s$ 且 $|z+h_s|\le\epsilon_s$ | `WAIT_ARUCO` |
| `WAIT_ARUCO` | $fresh$ 且 $c\ge n_{stable}$ | `CENTER_ABOVE_MARKER` |
| `CENTER_ABOVE_MARKER` | $r_e<r_{center}$ | `DESCEND_WITH_TRACKING` |
| `CENTER_ABOVE_MARKER` | Marker 丢失超过 $t_{lost}$ | `WAIT_ARUCO` |
| `DESCEND_WITH_TRACKING` | $z\ge-h_{final}$ | `FINAL_LAND` |
| `DESCEND_WITH_TRACKING` | Marker 丢失超过 $t_{lost}$ | `ABORT` |
| `FINAL_LAND` | 若启用则发送一次 `VEHICLE_CMD_NAV_LAND`，随后无条件转换 | `DONE` |
| `DONE` | 无自动退出 | 保持 `DONE` |
| `ABORT` | 无自动恢复 | 保持 `ABORT` |

进入 `ABORT` 时设定点为：

$$
(N_{sp},E_{sp},z_{sp},\psi_{sp})
=(N_{abort},E_{abort},-h_{abort},\psi_{abort})
$$

其中 $N_{abort},E_{abort}$ 是进入状态时的实测位置，此后不会继续跟踪 Marker。

## 11. 目标有效性与发布

`set_target()` 只有在四个目标量都有限时才令目标有效：

$$
valid_{target}=finite(N_{sp})\land finite(E_{sp})
\land finite(z_{sp})\land finite(\psi_{sp})
$$

目标无效时，`TrajectorySetpoint.position` 和 `yaw` 发布 NaN；有效时发布位置和航向，而 velocity、acceleration、jerk、yawspeed 始终为 NaN，表示当前节点只启用 PX4 位置控制。

`enable_auto_land=false` 不改变状态转换：节点仍从 `FINAL_LAND` 进入 `DONE`，只是跳过 `VEHICLE_CMD_NAV_LAND`，并继续发布进入最终阶段时锁定的位置目标。

## 12. 公式与代码函数映射

| 公式主题 | 代码函数 |
| --- | --- |
| 控制周期与 $\Delta t$ 限制 | `control_timer_callback()` |
| 四元数有效性 | `quaternion_is_valid()` |
| 四元数转航向角 | `quaternion_to_yaw()` |
| 相机误差到 NED | `compute_local_marker_error()` |
| 位姿新鲜度 | `marker_is_fresh()` |
| 水平限幅、对中和下降 | `run_state_machine()` |
| 状态入口目标 | `transition_to()` |
| 目标有限性 | `set_target()` |
| NED 位置设定点 | `publish_trajectory_setpoint()` |
| 航向四元数 | `publish_target_pose()` |
