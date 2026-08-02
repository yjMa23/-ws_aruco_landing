# P8C 固定倾斜与低频 roll/pitch 甲板降落综述与几何模型

## 0. 状态

```text
阶段：P8C
文档日期：2026-08-01
研究状态：RESEARCH PASS
计划状态：见 docs/plans/P8C_TILTED_DECK_LANDING_PLAN.md
实现状态：NOT STARTED
真实验收状态：NOT STARTED
```

本文件只完成项目现状核对、外部资料调研、坐标与接触几何推导、策略选择、分级实验路线和实现接口设计。本轮没有修改终端下降、`TouchdownDetector`、`TouchdownHoldController`、PX4 姿态接口、P8B MPC、MarkerSelector 或相机生产参数，没有开放 `rollpitch`/`combined` 最终下降。

---

## 1. 文档目的与阶段边界

P8C 的研究问题是：在甲板存在固定小倾角，随后扩展到低频 roll/pitch 时，如何把现有“世界 NED z 差”语义升级为可解释、可单测、可离线评测的甲板平面与 X500 滑橇接触几何，同时不破坏 P6B、P8A、P8B 已冻结的终端链路。

本阶段严格遵循：

```text
几何 shadow mode
→ 固定 2° 安全高度
→ 固定 2° 下降到 0.50 m
→ 固定 2° 真实接触
→ 姿态对齐决策门
→ 低频动态 roll/pitch
```

第一版默认采用：

```text
策略 A：UAV 保持水平
固定 +2° roll / +2° pitch
不发布姿态 setpoint
不修改 PX4 内环
```

只有固定 T1 真实接触出现可重复的单侧冲击、明显滑移、触地确认失败、持续离板、二次撞击、倾覆风险或 `TOUCHDOWN_HOLD` 失败，才允许另立姿态对齐综述与计划。

---

## 2. 当前仓库真实状态

### 2.1 Git、构建与依赖

2026-08-01 实际检查：

```text
branch: main
HEAD:   3b6f513 补充AGENTS.md：每次修改后同步说明文档
remote: main 与 origin/main 同步
worktree before P8C docs: clean
P8B validation commit: 2feeea4
existing tests: 271 tests, 0 errors, 0 failures, 0 skipped
```

P8B 固定依赖仍存在：

```text
~/.local/p8b-mpc/osqp-1.0.0-osqpeigen-0.11.2
```

当前不存在：

```text
docs/research/P8C_TILTED_DECK_LANDING_REVIEW.md（本文件创建前）
docs/plans/P8C_TILTED_DECK_LANDING_PLAN.md（计划创建前）
P8C 几何生产模块
P8C 固定倾角独立配置
P8C evaluator
```

### 2.2 已冻结阶段

```text
P6B: static / constant02 真实触地通过
P7-lite: static / constant02 3+3，6/6 PASS
P8A: H1 3/3、H2 3/3，升沉触地与相对 hold 通过
P8B: 安全高度 15/15、下降 6/6、真实触地 6/6 PASS
```

P8B 在 `FINAL_DESCENT`、`TOUCHDOWN_CANDIDATE_HOLD`、`TOUCHDOWN_HOLD` 使用 `TERMINAL_PHASE_P47`，因此 P8C 终端接触不会由 MPC 水平加速度直接驱动。

### 2.3 当前安全门

`scripts/start_sitl.sh` 允许创建 `rollpitch` 和 `combined` 仿真，但 `--enable-final-descent` 仅允许：

```text
static
constant02
constant
sinusoidal
heave
heave_h1
heave_h2
heave_h3
```

因此现有 `rollpitch`/`combined` 最终下降仍被显式阻断。P8C 必须新增独立固定倾角 T1 白名单，而不是复用动态场景绕过门控。

---

## 3. 现有实现复用点

P8C 可直接复用：

1. PX4 `VehicleOdometry` 的完整 NED 位姿和 FRD 机体角速度；
2. `VehicleLocalPosition` 的 NED 位置与线速度；
3. 图像采样时刻的 `VehiclePoseHistory`；
4. Marker PnP 到 local NED 的完整刚体变换；
5. 四尺度有状态 MarkerSelector 和统一甲板中心平移补偿；
6. `DeckAttitudeEstimator` 输出的甲板向上法向；
7. 视觉甲板位置/速度估计与短时预测；
8. landing window、相对下降、最终下降、触地状态机；
9. P6B/P8A evaluator 的接触、离板和二次接触统计框架；
10. P7 单轮与批量自动化；
11. P8B 自由飞行 MPC 与终端 P4.7 handoff。

P8C 第一轮应新增独立纯数学模块并以 shadow mode 接入，不能在几何输出验证前替换上述生产语义。

---

## 4. 当前能力缺口

当前生产链主要使用：

```text
relative_height = deck_z_ned - uav_z_ned
relative_vertical_velocity = deck_vz_ned - uav_vz_ned
horizontal_error = NED XY 差
horizontal_relative_speed = NED XY 相对速度范数
```

这些量在水平甲板上与甲板法向/切向语义一致，但在倾斜甲板上存在以下缺口：

- 没有显式甲板平面；
- 没有机体参考点到平面的有符号距离；
- 没有 X500 四个滑橇端点到平面的距离；
- 没有首接触端点；
- 没有 `h_min`、`h_max` 和接触不均匀度；
- 没有甲板旋转引起的接触点速度；
- 没有甲板平面内误差/速度；
- `TouchdownDetector` 的低高度和速度证据仍是 z/XY 语义；
- `rotational_movement=true` 会阻断倾斜接触候选；
- `TouchdownHoldController` 只跟随甲板中心 z；
- 当前法向估计约有度级误差，而 T1 真值仅 2°；
- 不同 Marker 的视觉平面相对碰撞平面有 1–4 mm z 偏置；
- 没有视觉甲板角速度或法向变化率估计。

---

## 5. 外部文献与方法综述

### 5.1 代表性来源

| 来源 | 问题与状态量 | 倾斜/运动处理 | 姿态/触地语义 | 可复用点 | 不能直接照搬 |
| --- | --- | --- | --- | --- | --- |
| Vlantis, Marantos, Bechlioulis, Kyriakopoulos, 2015, *Quadrotor landing on an inclined platform of a moving ground vehicle*, IEEE ICRA, DOI `10.1109/ICRA.2015.7139490` | 前视相机观测倾斜移动平台；优化位置、姿态、速度和时间 | 离散非线性 MPC，目标终态是倾斜、非平衡状态 | 显式最小化终端姿态与速度误差 | 证明倾斜终态不是普通悬停平衡点；提醒 FOV 与碰撞约束 | 直接控制姿态且模型复杂，超出 2° T1 最小基线 |
| Baca et al., 2019, *Autonomous landing on a moving vehicle with an unmanned aerial vehicle*, Journal of Field Robotics 36(5), DOI `10.1002/rob.21858` | 单目检测、非线性车辆预测、MPC 参考轨迹、非线性跟踪 | 预测移动目标并分阶段跟踪/降落 | 磁性起落架完成附着 | 状态估计、预测、轨迹与状态机分层 | 平坦车顶且磁性腿改变接触与滑移语义 |
| Paris, Lopez, How, 2020, *Dynamic Landing of an Autonomous Quadrotor on a Moving Platform in Turbulent Wind Conditions*, IEEE, arXiv `1909.11071` | EKF 估计平台位置、姿态、速度；滚动轨迹与鲁棒跟踪 | 估计—规划—控制紧耦合 | 强调快速直接触地和扰动鲁棒性 | 动态平台必须联合估计、规划和控制 | 主要针对平面移动与风扰，不给 X500 滑橇几何 |
| Abujoub, McPhee, Irani, 2020, *Methodologies for landing autonomous aerial vehicles on maritime vessels*, Aerospace Science and Technology 106, 106169, DOI `10.1016/j.ast.2020.106169` | 舰船 6-DOF 运动、短时预测、主动升沉补偿、着陆时机 | SPA/AHC/LPI 选择安全着陆窗口 | 指定触地速度并按阶段下降 | 动态 roll/pitch 前应先有运动预测与窗口 | 仿真舰船方法，不负责多旋翼滑橇点接触 |
| Cho, Choi, Bae, Oh, 2022, *Autonomous ship deck landing of a quadrotor UAV using feed-forward image-based visual servoing*, Aerospace Science and Technology 130, 107869, DOI `10.1016/j.ast.2022.107869` | GPS、视觉和舰船模型融合估计速度；IBVS | 虚拟平面、自适应增益、特征形状补偿和完整状态机 | 以安全流程处理快速、振荡舰船 | 速度前馈、FOV 管理、丢标恢复和多尺度 Marker | 图像控制误差不等于滑橇法向间隙，未直接给出接触几何 |
| Procházka et al., 2024, *Model predictive control-based trajectory generation for agile landing of unmanned aerial vehicle on a moving boat*, Ocean Engineering 313, 119164, DOI `10.1016/j.oceaneng.2024.119164` | 预测 USV 位置、姿态及其一阶导数，在线 MPC 轨迹 | 连续变化倾角下按阶段调整代价 | 终端同步 UAV 与 USV 姿态，降低落板冲击 | 动态阶段必须有甲板角运动估计；姿态同步应是独立高级方案 | 需要姿态接口、完整预测和更高复杂度，不适合首版 shadow/T1 |
| Kooi, Babuška, 2021, *Inclined Quadrotor Landing using Deep Reinforcement Learning*, arXiv `2103.09043` | 直接学习倾斜面终端策略 | 以训练策略处理非平衡终态 | 实机 Crazyflie 倾斜降落 | 再次说明倾斜终态不是普通平衡点 | 本项目当前阶段是可解释传统基线，不采用 DRL |
| Ikura, Miyashita, Ishikawa, 2021, *Stabilization System for UAV Landing on Rough Ground by Adaptive 3D Sensing and High-Speed Landing Gear Adjustment*, Journal of Robotics and Mechatronics 33(1), DOI `10.20965/jrm.2021.p0108` | 直接测量多个起落架预期接触点高度 | 高频调整腿长以实现近同步接触 | 两腿接触时间差被作为稳定性目标 | 支持“逐接触点间隙”而非单一机体高度 | 使用主动可变起落架，X500 固定滑橇不能照搬 |
| Wu, Liang, Li, 2022, *The dynamic analysis of the rotorcraft with robotic landing gear in the landing process on uneven ground*, Advances in Mechanical Engineering 14(6), DOI `10.1177/16878132221107473` | 起落架—机体接触动力学、地面倾角和初始速度 | 分析不平地面下冲击和稳定性 | 接触顺序、冲击和倾覆由速度/坡度共同决定 | 支持记录法向/切向速度、姿态和倾覆风险 | 针对机器人起落架动力学，不是本项目控制器模型 |
| PX4 `VehicleOdometry` / `VehicleLocalPosition` 官方消息文档 | local NED 位姿、FRD 机体量、位置/速度有效性 | 明确坐标与四元数契约 | 不定义滑橇接触点 | 用于输入语义核对 | 不能替代具体 X500 SDF 几何 |

### 5.2 调研结论

外部方法可分为：

1. 保持 UAV 近水平，只追踪移动平台位置/速度；
2. 终端阶段把姿态与平台同步；
3. 全程在平台相对坐标中规划和控制；
4. 使用主动/柔顺起落架吸收不均匀接触。

对当前项目，文献不能替代本机 X500 SDF 和现有触地状态机的几何核对。P8C 第一版应先建立接触点 shadow 指标并验证 2° 水平机体基线；只有真实证据表明固定滑橇无法安全完成 T1，才进入姿态同步或接触动力学研究。

---

## 6. 坐标系、符号与单位

### 6.1 坐标系

```text
N: local_ned，x North，y East，z Down
B: base_link_frd，x Forward，y Right，z Down
D: 甲板刚体坐标，原点位于甲板碰撞表面中心
```

位置单位 m，线速度 m/s，角速度 rad/s，角度在代码中使用 rad。

### 6.2 变量

```text
p_d^N(t)       甲板平面参考点，local NED
p_u^N(t)       PX4 UAV 机体参考点，local NED
R_NB(t)        base_link_frd 到 local NED 的旋转
n^N(t)         指向甲板上方的单位法向，local NED
r_i^B          第 i 个等效滑橇接触点相对 UAV 参考点的 FRD 坐标
p_i^N          第 i 个接触点在 local NED 中的位置
v_d^N, v_u^N   甲板参考点和 UAV 参考点线速度
omega_d^N      甲板角速度，local NED
omega_u^N      UAV 角速度，local NED
```

约束：

```text
||n|| = 1
水平甲板的向上法向 n = [0, 0, -1]^T
```

法向符号不得根据单帧视觉结果任意翻转。输入法向若与上一有效法向点积小于 0，应先统一方向；若向上分量不满足安全门，输出无效而不是静默取反。

---

## 7. 甲板平面几何模型

### 7.1 平面方程

甲板碰撞表面定义为：

```math
\Pi(t):\quad {n^N(t)}^T\left[p^N-p_d^N(t)\right]=0
```

点在甲板上方时定义为正间隙。

UAV 参考点法向间隙：

```math
h_{body}={n^N}^T\left(p_u^N-p_d^N\right)
```

### 7.2 水平甲板退化验证

水平甲板：

```math
n^N=\begin{bmatrix}0&0&-1\end{bmatrix}^T
```

于是：

```math
h_{body}
= [0,0,-1]\left(p_u^N-p_d^N\right)
= -\left(z_{uav,ned}-z_{deck,ned}\right)
= z_{deck,ned}-z_{uav,ned}
```

这与当前生产代码完全一致。

数值例：

```text
p_d = [0, 0, 0] m
p_u = [0, 0, -1] m
n   = [0, 0, -1]
h_body = 1 m
```

因此本模型不是替换水平语义，而是其严格三维推广。

### 7.3 当前 z 高度语义的适用性

| 场景 | `deck_z-uav_z` 是否充分 | 原因 |
| --- | --- | --- |
| 水平静止甲板 | 是，作为机体参考点竖直/法向间隙 | z 轴与甲板法向一致 |
| 水平升沉甲板 | 是，配合 `deck_vz-uav_vz` | 法向固定，甲板只平移 |
| 固定 roll/pitch、UAV 正上方且保持水平 | 近似可用，但不是滑橇间隙 | z 差忽略接触点分布和水平偏差 |
| 固定倾斜且 UAV 有水平偏差 | 不充分 | 平面高度随水平位置变化 |
| 动态 roll/pitch | 不充分 | 法向、平面和局部点速度随时间变化 |
| UAV 姿态变化 | 不充分 | 滑橇点通过 `R_NB r_i` 改变 |
| 单侧滑橇先接触 | 不充分 | 机体中心仍可能远离平面，且无法区分接触点 |

必须区分：

```text
NED z 差
!= 甲板法向距离
!= UAV 参考点到平面距离（仅水平甲板时相等）
!= 滑橇最低点到平面距离
```

---

## 8. X500 滑橇接触几何

### 8.1 真实 SDF

实际读取：

```text
/home/j/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf
```

`base_link` 中最低两条滑橇碰撞盒（Gazebo FLU）：

```text
collision_3 center = [0, -0.132, -0.2195] m
collision_4 center = [0, +0.132, -0.2195] m
size               = [0.25, 0.015, 0.015] m
```

FLU 到 FRD：

```text
x_frd = x_flu
y_frd = -y_flu
z_frd = -z_flu
```

碰撞盒底面中心线相对 `base_link` 的 FRD z 为：

```text
0.2195 + 0.015/2 = 0.2270 m
```

这与 P6B evaluator 已冻结的 `0.227 m` 几何偏移一致。

### 8.2 第一版四端点模型

以两条滑橇底部中心线的前后端点作为等效点：

```text
r_0^B = [-0.125, -0.132, 0.227] m
r_1^B = [+0.125, -0.132, 0.227] m
r_2^B = [-0.125, +0.132, 0.227] m
r_3^B = [+0.125, +0.132, 0.227] m
```

端点在 local NED：

```math
p_i^N=p_u^N+R_{NB}r_i^B
```

有符号间隙：

```math
h_i={n^N}^T\left(p_i^N-p_d^N\right)
```

定义：

```math
h_{min}=\min_i h_i,\qquad
h_{max}=\max_i h_i,\qquad
\Delta h=h_{max}-h_{min}
```

解释：

- `h_min → 0`：至少一个等效端点接近或发生接触；
- `h_max → 0`：全部端点接近共面接触；
- `Δh`：几何接触不均匀度；
- `argmin_i h_i`：预测首接触端点。

### 8.3 连续滑橇与端点近似

对零厚度直线滑橇，点到平面的有符号距离沿线段是仿射函数，因此最小值和最大值严格出现在端点。四端点对两条中心线的平面间隙极值不是经验猜测，而是精确结果。

物理碰撞体是宽厚均为 15 mm 的长方体。四个底部中心线端点忽略：

- 半宽 `7.5 mm`；
- 厚度方向角点；
- ODE 接触压入；
- 支撑杆接触；
- 碰撞盒与真实机架柔性。

长方体精确支持函数可写为：

```math
h_{box,min}=n^T(p_c-p_d)
-|n^TR_{NB}e_x|a
-|n^TR_{NB}e_y|b
-|n^TR_{NB}e_z|c
```

其中 `a=0.125 m`、`b=c=0.0075 m`。第一版 shadow mode 使用四端点，evaluator 同时保留 SDF Ground Truth 接触/压入容差；若端点模型与物理接触次序系统不一致，再升级为盒体支持函数，而不是直接改控制器。

### 8.4 2° 数值例

当前姿态估计器的 yaw 无关法向约定：

```math
n(\phi,\theta)=
\begin{bmatrix}
-\sin\theta\cos\phi\\
\sin\phi\\
-\cos\theta\cos\phi
\end{bmatrix}
```

UAV 水平、甲板固定 `+2° roll`：

```text
n ≈ [0, 0.034899, -0.999391]
滑橇左右跨距 = 0.264 m
Δh ≈ 0.264 sin(2°) = 0.00921 m
首接触时 h_body ≈ 0.23147 m
```

固定 `+2° pitch`：

```text
滑橇前后长度 = 0.250 m
Δh ≈ 0.250 sin(2°) = 0.00872 m
首接触时 h_body ≈ 0.23122 m
```

水平甲板首接触参考高度约为 `0.227 m`。2° 倾角只使机体参考点首接触高度变化约 4–5 mm，但会产生约 9 mm 的滑橇端点先后接触差。因此当前 z 语义可能仍能触发终端停滞，却无法解释单侧接触、冲击、滑移和二次接触。

---

## 9. 甲板参考点与 Marker 平面

### 9.1 甲板碰撞平面

`moving_deck/model.sdf`：

```text
model pose z = 2 m
collision pose z = -0.05 m
box thickness = 0.10 m
```

碰撞盒顶面位于 link/model 原点 z=0，因此甲板模型原点是碰撞表面中心，可作为 `p_d` 的物理定义。

### 9.2 Marker 中心补偿

四个 Marker 在甲板 x 方向的偏移分别为：

```text
ID0 +0.45 m
ID1 -0.18 m
ID2 +0.05 m
ID3  0.00 m
```

`aruco_detector` 使用每个 Marker 坐标中的目标偏移，把发布位置平移到统一甲板中心。位置补偿已实现；法向仍来自当前选中 Marker 的完整旋转。

### 9.3 尚存的 z 与法向一致性问题

Marker visual 平面相对碰撞顶面 z 为：

```text
ID0 1 mm
ID1 2 mm
ID2 3 mm
ID3 4 mm
```

当前目标偏移没有 z 补偿，因此 active Marker 切换可能给甲板参考点 z 引入最多约 3 mm 的确定性偏差。各 Marker 名义姿态共面，但 PnP 噪声、尺度、边缘像素和相机外参会造成法向不一致。

P8C 实现前必须：

1. 明确 `p_d` 是碰撞平面中心，不是 Marker visual 平面；
2. 在 evaluator 中按 active ID 统计 Marker 平面偏置；
3. 固定倾角标定时分别统计四个 ID 的法向误差；
4. 记录切换前后法向跳变；
5. 不通过放宽 landing-window 倾角阈值掩盖偏差。

---

## 10. 法向和平面内相对运动

### 10.1 固定甲板参考点法向速度

定义法向间隙变化率（正值分离，负值接近）：

```math
v_n={n^N}^T\left(v_u^N-v_d^N\right)
```

水平甲板：

```math
v_n=[0,0,-1](v_u-v_d)=v_{deck,z,ned}-v_{uav,z,ned}
```

因此它严格退化为当前 `/landing/relative_vertical_velocity` 的符号与语义。

### 10.2 动态甲板接触点速度

刚体上空间点速度：

```math
v_{u,i}^N=v_u^N+\omega_u^N\times(p_i^N-p_u^N)
```

甲板在同一空间点的局部速度：

```math
v_{d@i}^N=v_d^N+\omega_d^N\times(p_i^N-p_d^N)
```

接触点法向相对速度：

```math
v_{n,i}={n^N}^T\left[
 v_u^N+\omega_u^N\times(p_i^N-p_u^N)
-v_d^N-\omega_d^N\times(p_i^N-p_d^N)
\right]
```

`v_{n,i}<0` 表示第 i 点正在闭合；触地安全判据使用绝对值或闭合方向限值时必须显式说明。

### 10.3 当前可获得量

| 量 | 当前控制链 | 来源/限制 |
| --- | --- | --- |
| UAV 位置 | 有 | `VehicleLocalPosition` / `VehicleOdometry` |
| UAV 线速度 | 有 | local NED；有效性检查 |
| UAV 姿态 | 有 | `VehicleOdometry.q`，FRD→NED |
| UAV 角速度 | 有 | `VehicleOdometry.angular_velocity` 为 body FRD；需旋转到 NED |
| 甲板参考点位置 | 有 | Marker 位置估计，需修正 Marker 平面 z 语义 |
| 甲板线速度 | 有 | 视觉状态估计 |
| 甲板法向 | 有 | Marker +Z 低通 |
| 甲板角速度 | 无可靠生产估计 | 仿真 GT 有，但禁止进入控制器 |
| 法向变化率 | 无 | 可由视觉法向时间序列估计，但必须新增滤波、时间同步与有效性 |

结论：

```text
固定 T1：omega_d = 0，可不新增甲板角速度估计。
动态 roll/pitch：未获得可靠 omega_d 或 dot(n) 前禁止最终下降。
```

### 10.4 平面内位置与速度

切平面投影矩阵：

```math
P_t=I-nn^T
```

平面内位置误差：

```math
e_t=P_t(p_u-p_d)
```

平面内参考点相对速度：

```math
v_t=P_t(v_u-v_d)
```

接触点版本应加入双方角速度项后再投影。

水平甲板时：

```text
P_t = diag(1,1,0)
e_t = 当前 NED XY 位置差（外加零 z 分量）
v_t = 当前 NED XY 相对速度（外加零 z 分量）
```

固定 2° 时继续用现有 NED XY 控制是可接受近似，但诊断应同时记录 `e_t`。仅由 `0.50 m` 法向间隙投影产生的水平差约为：

```text
0.50 sin(2°) ≈ 0.0174 m
```

近接触 `0.227 m` 时约为：

```text
0.227 sin(2°) ≈ 0.0079 m
```

它小于当前 `0.15 m` landing-window 水平阈值，但已达到 P8A/P8B 约 0.03 m RMSE 的可观比例，因此 T1 可保留控制器，却必须在 evaluator 中区分 NED XY 和切平面误差。

---

## 11. 固定倾斜与动态 roll/pitch 的区别

| 项目 | 固定 2° | 低频动态 roll/pitch |
| --- | --- | --- |
| 法向 | 常量，可滤波标定 | 时变，存在相位延迟 |
| 甲板角速度 | 0 | 必须估计或可靠门控 |
| 接触点甲板速度 | 等于参考点平移速度 | 含 `omega_d × r` |
| `TouchdownHold` | 静止固定倾斜时世界 z 可暂用 | 世界 z/甲板中心 z 均不充分 |
| landing window | 当前 5°/8° 可容纳 2°，但估计偏差需解决 | 需预测窗口和角速度限制 |
| 相机 FOV | 静态透视畸变 | 透视、运动模糊和周期出视野 |
| 安全回退 | 保持高度/恢复爬升 | 还需相位/角速度恢复条件 |
| 可开放最终下降 | P8C-1/2 通过后单独白名单 | 角速度、接触速度和 hold 完成前禁止 |

固定 T1 通过只证明小倾角静态几何可行，不证明动态甲板可行。

---

## 12. 当前 TouchdownDetector 适用性

### 12.1 现有证据

现有检测器综合：

```text
PX4 ground_contact
PX4 maybe_landed
PX4 landed
PX4 at_rest
PX4 close_to_ground
视觉低高度
甲板—UAV 相对垂直速度
UAV 世界垂直速度（诊断）
水平相对速度
terminal contact stall
PX4 vertical/horizontal/rotational movement
候选持续时间与确认锁存
```

### 12.2 倾斜甲板适用性

| 证据 | 固定 2° | 动态 roll/pitch | 处理建议 |
| --- | --- | --- | --- |
| PX4 ground/maybe/landed/at_rest | 可保留，多源证据 | 可保留但可能因平台运动不稳定 | 不单独作为真值 |
| close_to_ground | 可保留 | 可保留 | 只作组合证据 |
| 视觉低高度 | z 近似可 shadow 对比 | 不充分 | 最终改为 `h_min` 或明确法向间隙 |
| 相对垂直速度 | 2° 时近似 | 不充分 | 最终改为首接触点 `v_n_i` |
| UAV 世界垂直速度 | 仅诊断 | 仅诊断 | 不恢复为硬门 |
| 水平相对速度 | 2° 时近似 | 不充分 | 最终改为平面内相对速度 |
| terminal stall | 可保留作旧路径对照 | 需法向版本 | shadow 先不改生产行为 |
| rotational_movement | 单侧接触可能短时置真并阻断 | 更易置真 | 必须记录；不能简单忽略或放宽 |

### 12.3 单侧先接触

`h_min≈0` 只说明至少一个端点接触。单侧接触后可能发生：

- 机体绕接触滑橇转动；
- PX4 `rotational_movement=true`；
- 另一滑橇随后落下；
- 弹跳、滑移或二次撞击。

因此不能在单侧首接触瞬间直接锁存 confirmed。T1 第一轮保持现有 detector 不变，只记录：

```text
first_contact_point
h_min / h_max
v_n at first contact
rotational_movement duration
candidate start and repeat count
second-side contact delay
```

生产语义升级时建议：

1. `h_min` 和安全法向速度形成“首接触”诊断；
2. PX4 接触证据或 terminal stall 继续参与候选；
3. 候选期间允许有界、短时的接触转动，但必须定义角度/角速度/持续时间边界；
4. `h_max`、低平面内速度、无持续离板和候选持续时间共同确认稳定落板；
5. 不能通过无条件忽略 `rotational_movement` 获得 PASS。

---

## 13. 当前 TouchdownHoldController 适用性

当前保持初始化：

```text
h_hold_z = deck_z_ned - uav_z_ned
z_target = uav_z_ned
```

后续只在甲板中心 z 速度超过阈值时更新：

```text
z_target → deck_z_ned - h_hold_z
```

### 13.1 固定倾斜

固定倾斜且甲板不平移/不转动时，世界 z 目标恒定，作为 T1 首版可继续使用。它不主动增大压板，也不会因甲板姿态变化产生误跟随，因为姿态不变。

但它只是在固定场景“可暂用”，不代表保持了法向接触或四点共面。

### 13.2 动态 roll/pitch

动态甲板中心 z 可保持不变，但边缘高度会随姿态变化。只跟随中心 z 可能：

- 甲板抬向某滑橇时持续压入；
- 甲板离开某滑橇时离板；
- 单侧接触后继续世界 z 保持导致绕点转动；
- 法向力方向变化造成平面内滑移；
- 另一侧发生二次撞击。

### 13.3 候选保持策略

| 策略 | 定义 | 优点 | 风险/适用性 |
| --- | --- | --- | --- |
| A. 保持世界系位置目标 | 锁存当前 `p_u^N` | 最简单，static/固定倾斜兼容 | 动态 roll/pitch 不跟随局部表面 |
| B. 保持甲板参考点法向距离 | 保持 `n^T(p_u-p_d)=h_hold`，切向目标另行定义 | 连续、可解释，适合无接触或轻接触 | 只约束机体参考点，不保证滑橇接触；法向变化会移动目标 |
| C. 保持最低滑橇间隙/接触几何 | 调整目标使 `h_min` 或活动接触集合保持目标值 | 最接近物理接触 | `min` 非光滑、活动点切换、接触动力学和力约束复杂 |

推荐路线：

```text
固定 T1：保持现有世界位置/世界 z，shadow 记录 B/C 指标。
动态研究前：至少实现 B 的无接触安全高度/下降参考。
真实动态接触前：依据 T1/T动态结果选择 B 或独立研究 C；不能直接用中心 z hold。
```

---

## 14. 策略 A/B/C 比较

| 维度 | A UAV 始终水平 | B 终端有限法向对齐 | C 全程跟随法向 |
| --- | --- | --- | --- |
| 实现复杂度 | 低 | 中高 | 高 |
| PX4 接口 | 继续位置/速度/加速度外环 | 需姿态或姿态约束 setpoint | 全程姿态轨迹与更完整接口 |
| 法向噪声敏感性 | 只影响窗口/几何诊断 | 终端直接进入姿态命令 | 全程敏感，滤波延迟更严重 |
| 控制耦合 | 保持现有架构 | 横向、垂向、姿态耦合 | 六自由度强耦合 |
| 相机 FOV | 机体水平，视场最稳定 | 终端倾斜可能偏移 Marker | 全程倾斜，FOV 管理最难 |
| 滑橇接触均匀性 | 2° 时约 9 mm 先后差 | 可降低先后差 | 理论最好 |
| 单侧冲击 | 可能 | 较低 | 较低，但跟踪误差可反向放大 |
| 滑移风险 | 由坡度、摩擦和切向速度决定 | 可降低法向错位，但姿态过渡也可能滑移 | 依赖全程同步精度 |
| 倾覆风险 | 小倾角低速时预计低，需实测 | 过渡失稳风险 | 法向估计/控制失败时风险最高 |
| 动态甲板适用性 | 只适合低频小倾角且需强门控 | 适合终端同步 | 潜在适用范围最大 |
| P6B/P8A/P8B 兼容 | 最高 | 需新增独立链路 | 会重构现有链路 |
| 回退路径 | 直接回退现有水平基线 | 可回退 A | 回退涉及姿态连续性 |
| 论文表达价值 | 清晰基线和几何消融 | 有明确高级对比价值 | 最高但验证成本大 |
| 真实平台迁移 | 最少接口假设 | 需姿态带宽、外参和安全认证 | 对传感器/内环要求最高 |

### 14.1 第一版选择

选择策略 A，理由：

1. 2° 对 X500 只产生约 9 mm 等效端点差；
2. P6B/P8A 终端接触速度已很低；
3. 当前法向估计误差约 1–2°，直接驱动姿态没有足够信噪比；
4. P8B 已证明终端姿态耦合会影响接触，贸然加入姿态命令风险更高；
5. A 是对现有冻结基线的最小、可证伪扩展；
6. 只有真实失败证据才能证明 B/C 的额外复杂度合理。

---

## 15. 视觉姿态标定问题

P5A 真实倾角 RMSE：

```text
static:      1.611°
constant:    1.459°
heave:       1.468°
roll/pitch:  1.029°
combined:    0.782°
```

静止场景最大估计倾角曾达到约 `4.50°`。T1 真值只有 2°，因此当前未经标定的单帧或原始滤波结果不能可靠区分 0° 与 2°。

潜在来源：

- 相机旋转外参偏差直接映射为法向固定偏差；
- Marker 平面与碰撞平面存在 1–4 mm 高度差；
- 不同物理尺度的角点量化和 PnP 条件数不同；
- Marker 切换时法向可能跳变；
- 视觉畸变、模糊和近距离裁剪；
- 当前一阶低通只降噪，不估计固定偏差；
- 法向先线性混合再归一化会产生相位延迟。

P8C-1 前的法向标定门：

```text
固定 0°、+2° roll、-2° roll、+2° pitch、-2° pitch
每个姿态分别统计 ID0/ID1/ID2/ID3
Ground Truth 只进入 evaluator
```

建议通过阈值：

```text
平均有符号角误差 <= 0.5°
倾角 RMSE <= 1.0°
角误差 P95 <= 1.5°
倾斜符号正确率 >= 95%
Marker 切换法向跳变 <= 1.0°
```

依据：2° 目标必须至少保留约 1° 的可辨识裕量；当前 1.6° static RMSE 不满足，故该门是实现前待解决问题，不是已完成指标。可采用离线固定偏差标定或更严格的几何/相机标定，但禁止通过放宽 landing-window 倾角阈值掩盖误差。

---

## 16. P8C 分级实验路线

### 16.1 P8C-0：几何 shadow mode

新增纯数学模块设计：

```text
deck_plane_geometry.hpp
deck_plane_geometry.cpp
deck_plane_geometry_test.cpp
```

只计算并记录：

```text
normalized upward normal
h_body
h_i[4]
h_min / h_max / delta_h / first_contact_index
v_n_body / v_n_i[4]（有效时）
e_t / v_t
input validity and reason
```

不改变控制输出、状态机、landing window、触地确认或 hold。

### 16.2 P8C-1：固定小倾角安全高度

新增独立场景：

```text
tilt_roll_pos_2deg
tilt_roll_neg_2deg
tilt_pitch_pos_2deg
tilt_pitch_neg_2deg
```

第一轮至少运行 `+2° roll` 和 `+2° pitch`，在 5 m 安全高度检查法向标定、Marker 可见性、水平跟踪、FOV、landing window 和 shadow 指标。

### 16.3 P8C-2：固定 2° 下降到 0.50 m

只有 P8C-1 通过后，开放独立 T1 白名单的相对下降，继续关闭最终下降。比较：

```text
current z difference
h_body
h_min/h_max
NED XY vs e_t
```

验证 landing window 和现有下降参考没有因 2° 近似产生危险偏差。

### 16.4 P8C-3：固定 2° 真实接触

顺序：

```text
+2° roll 单轮
→ 定位问题
→ +2° roll 3 seeds
→ +2° pitch 单轮
→ +2° pitch 3 seeds
→ static 3/3
→ constant02 3/3
→ H1 至少单轮
```

保持：

```text
NAV_LAND = 0
Disarm = 0
UAV 姿态 setpoint = 未新增
```

### 16.5 P8C-4：姿态对齐决策门

保持水平策略满足 T1：

```text
不实现姿态对齐，进入低频动态前置研究。
```

出现以下可重复问题之一：

```text
明显单侧冲击
接触后持续滑移
无法稳定触地确认
持续离板
二次撞击
倾覆或大姿态风险
TOUCHDOWN_HOLD 无法维持
```

则停止 P8C 动态扩展，新建独立姿态对齐综述与计划，比较 B/C 和 PX4 姿态接口。

### 16.6 P8C-5：低频动态 roll/pitch

前置条件：

- 可靠甲板角速度或法向变化率估计；
- 动态接触点 `v_n_i`；
- 低频窗口预测和相位延迟评估；
- 动态接触后保持方案；
- 固定 T1 回归不退化；
- 独立动态场景和恢复策略；
- 不从 T1 直接跳到 `combined`。

---

## 17. T1 指标与验收门槛

### 17.1 必须统计

```text
水平位置 RMSE
水平相对速度 RMSE
估计法向角误差、偏差、P95、符号正确率
参考点法向间隙 h_body
四个滑橇间隙 h_i
h_min / h_max / delta_h
触地时法向相对速度
第一接触端点和另一侧接触延迟
触地候选持续时间和反复次数
TOUCHDOWN_HOLD 持续时间
接触后平面内滑移和切向速度
接触后最大 UAV roll/pitch
二次撞击、持续离板、恢复爬升、GNSS recovery
NAV_LAND / Disarm
Marker ID、切换法向跳变和视觉丢失
```

### 17.2 数值门槛及依据

| 指标 | T1 第一版门槛 | 依据 |
| --- | ---: | --- |
| 稳态水平位置 RMSE | `<= 0.08 m` | P8A/P8B 常见约 0.03–0.04 m；低于 0.15 m landing-window 进入阈值并留 2× 退化裕量 |
| 最大水平误差 | `<= 0.15 m` | 不越过现有 landing-window 进入边界 |
| 水平/切向相对速度 RMSE | `<= 0.10 m/s` | 低于现有 0.15 m/s 触地/窗口硬边界 |
| 法向标定 | 见第 15 节 | 需要从 2° 真值保留可辨识裕量 |
| 触地 `h_min` | `[-0.05, +0.03] m` | 继承 P6B/P8A 碰撞压入与接触容差 |
| 触地 `h_max` | `<= 0.05 m` | 不把另一侧持续离板视为稳定接触；继承 P8A detach 门 |
| 触地 `delta_h` | `<= 0.03 m` | 理论 2° 仅 0.009 m，允许 SDF/姿态/估计裕量但禁止明显单侧悬空 |
| 触地 `|v_n|` | 硬门 `<=0.12 m/s`；目标 `<=0.05 m/s` | 继承 detector 最大相对速度和 evaluator 接触速度 |
| 候选持续 | 保持现有参数 `0.50 s` | 不缩短已验证的候选确认逻辑；20 Hz 记录延迟约 0.40–0.60 s 可接受 |
| hold | `>=10 s` | 继承 P6B/P8A/P8B |
| hold `h_min` | 不低于 `-0.05 m` | 禁止持续过度压入 |
| hold `h_max` | 不高于 `+0.05 m` | 禁止持续离板 |
| 接触后最大切向滑移 | `<=0.10 m` | 大于现有约 0.05–0.08 m 最大水平误差但仍小于 landing window 尺度 |
| hold 切向速度 P95 | `<=0.05 m/s` | 稳定接触目标，显著低于 0.15 m/s 硬门 |
| 接触后 UAV roll/pitch | 各自绝对值 `<=10°` 且无发散趋势 | 高于 2° 几何与 8° deck-window exit 少量裕量，但可在倾覆前停止 |
| detach / secondary impact | `0 / 0` | 继承 P8A |
| recovery / GNSS recovery | `0 / 0` | T1 正向验收不允许恢复掩盖失败 |
| NAV_LAND / Disarm | `0 / 0` | 全局安全边界 |

最低阶段门：

```text
固定 2° 单轮 PASS
固定 2° 3/3 PASS
TOUCHDOWN_HOLD >= 10 s
无明显二次撞击、持续滑移、持续离板或倾覆
NAV_LAND = 0
Disarm = 0
static 3/3
constant02 3/3
H1 至少单轮
```

若 +roll 与 +pitch 的接触次序或误差差异明显，必须补跑负倾角，不得把单一方向结果推广到所有倾斜方向。

---

## 18. 风险、失败模式与回退

| 风险 | 诊断 | 停止/回退 |
| --- | --- | --- |
| 2° 法向不可辨识 | 0°/±2° 分布重叠、符号错误 | 停在 P8C-1，完成标定，不放宽倾角阈值 |
| Marker 切换法向跳变 | active ID 变化时角度突跳 | 固定单 ID 诊断并修正尺度一致性；不改 MarkerSelector 参数 |
| Marker 平面 z 偏置 | active ID 对 h_body 有 1–4 mm 系统差 | evaluator 校正/统一模型；不把偏差当真实甲板运动 |
| 端点模型与 SDF 接触不符 | predicted first point 与 GT 接触顺序不一致 | 升级盒体支持函数，不改控制器 |
| 单侧冲击/大转动 | `h_min=0` 后角速度/姿态峰值 | 停止批量，进入 P8C-4 决策 |
| 滑移 | 切向位移/速度超门 | 停止；核对摩擦、切向速度、姿态，不放宽 detector |
| 触地候选被转动阻断 | `rotational_movement` 与几何接触同时出现 | 只记录 shadow；另立检测语义测试，禁止直接忽略该位 |
| 动态法向延迟 | 估计相位落后、`v_n_i` 错符号 | 禁止 P8C-5 最终下降 |
| hold 压入/离板 | h_min/h_max 越界或二次接触 | 固定 T1 可退回世界位置；动态阶段标记 blocked |
| static/constant02/H1 退化 | 回归失败 | 恢复最近通过路径，保留新几何 shadow，不调旧参数 |

---

## 19. 控制器允许和禁止的数据来源

### 19.1 允许进入控制器/几何 shadow

```text
PX4 VehicleOdometry / VehicleLocalPosition
视觉 Marker 位姿与 active ID
视觉甲板状态估计
YAML 中明确坐标语义的相机与滑橇几何参数
控制器内部滤波、有效性和时间同步状态
```

### 19.2 仅允许 evaluator

```text
/simulation/deck/ground_truth
Gazebo 甲板真实姿态、角速度和运动相位
Gazebo 模型真实接触/碰撞状态
场景真值倾角
```

### 19.3 禁止

```text
用 Ground Truth 法向或角速度驱动控制
按场景名给控制器注入姿态真值
用未来解析运动轨迹驱动下降
通过 rollpitch/combined 名称绕过安全门
```

仓库检查确认 `aruco_precision_landing_cpp` 和 `aruco_detector` 当前没有订阅 `/simulation/deck/ground_truth`。

---

## 20. 项目代码接口设计

### 20.1 纯数学输入

```cpp
struct DeckPlaneGeometryInput
{
  Eigen::Vector3d deck_reference_position_ned_m;
  Eigen::Vector3d upward_normal_ned;
  Eigen::Vector3d uav_reference_position_ned_m;
  Eigen::Quaterniond body_frd_to_ned;
  std::array<Eigen::Vector3d, 4> contact_points_body_frd_m;
  std::optional<Eigen::Vector3d> deck_linear_velocity_ned_mps;
  std::optional<Eigen::Vector3d> uav_linear_velocity_ned_mps;
  std::optional<Eigen::Vector3d> deck_angular_velocity_ned_radps;
  std::optional<Eigen::Vector3d> uav_angular_velocity_ned_radps;
};
```

### 20.2 输出

```cpp
struct DeckPlaneGeometryOutput
{
  Eigen::Vector3d upward_normal_ned;
  double body_normal_gap_m;
  std::array<Eigen::Vector3d, 4> contact_positions_ned_m;
  std::array<double, 4> contact_gaps_m;
  double minimum_contact_gap_m;
  double maximum_contact_gap_m;
  double contact_gap_spread_m;
  std::size_t first_contact_index;
  std::optional<double> body_normal_relative_velocity_mps;
  std::array<std::optional<double>, 4> contact_normal_relative_velocity_mps;
  Eigen::Vector3d tangential_position_error_ned_m;
  std::optional<Eigen::Vector3d> tangential_relative_velocity_ned_mps;
};
```

### 20.3 有效性

无效输入返回 `std::nullopt` 或带原因的显式结果：

```text
NaN/Inf
法向范数过小
法向朝向错误
四元数范数过小
接触点非有限
速度组合不完整
动态速度请求但缺少 omega_d
```

位置几何可在速度输入无效时继续输出；速度字段单独无效，不能因缺角速度而丢弃全部位置诊断。

### 20.4 shadow 诊断建议

```text
/landing/deck_plane_normal
/landing/body_normal_gap
/landing/skid_contact_gaps
/landing/skid_min_gap
/landing/skid_max_gap
/landing/skid_gap_spread
/landing/skid_first_contact_index
/landing/body_normal_relative_velocity
/landing/tangential_position_error
/landing/tangential_relative_speed
/landing/deck_plane_geometry_status
```

第一轮只发布调试，不参与 setpoint、窗口、触地或 hold。

---

## 21. 研究结论

1. 当前 `deck_z-uav_z` 是水平甲板下 `h_body=n^T(p_u-p_d)` 的严格退化，不是错误；它在倾斜甲板上不再代表滑橇最低间隙。
2. X500 两条滑橇底部中心线可由四个 FRD 端点建模，`z=0.227 m` 与 P6B evaluator 一致。
3. 固定 2° roll/pitch 造成约 8.7–9.2 mm 的端点先后接触差；这需要 shadow 与 evaluator，但尚不足以证明必须姿态对齐。
4. 固定 T1 可使用当前线速度、姿态、法向和 `omega_d=0`；动态 roll/pitch 在没有可靠甲板角速度/法向变化率前必须阻断最终下降。
5. 当前触地检测器可在 T1 中保持不变作生产对照，但未来低高度、垂直速度和水平速度语义应分别升级为 `h_min`、接触点法向速度和平面内速度。
6. 当前 hold 在固定静态倾斜时可暂用，在动态姿态下不充分。
7. 当前视觉法向误差与 2° 真值同量级；法向标定是 P8C-1 的硬前置，不得靠放宽倾角阈值解决。
8. 第一版选择策略 A：UAV 始终水平，不发布姿态 setpoint。
9. P8C 必须先完成几何 shadow、安全高度和 0.50 m 下降，再开放独立 T1 真实接触。
10. Ground Truth 只进入 evaluator。

---

## 22. RESEARCH PASS 检查表

- [x] 已验证分支、HEAD、工作区、历史提交、测试和依赖。
- [x] 已完成外部资料调研并记录正式来源、问题、状态量、方法和限制。
- [x] 已定义甲板平面和 NED 向上法向。
- [x] 已推导水平甲板退化为 `deck_z-uav_z`。
- [x] 已核对 X500 真实滑橇碰撞体。
- [x] 已定义四端点模型并说明连续线段与盒体误差。
- [x] 已定义 `h_body`、`h_i`、`h_min`、`h_max` 和 `delta_h`。
- [x] 已定义固定与动态甲板的法向相对速度。
- [x] 已定义平面内位置和速度。
- [x] 已分析现有 `TouchdownDetector` 适用边界。
- [x] 已分析现有 `TouchdownHoldController` 适用边界。
- [x] 已比较 A/B/C 三种策略。
- [x] 已明确第一版 UAV 保持水平。
- [x] 已给出 T1 分级场景、指标与回归矩阵。
- [x] 已明确视觉 2° 标定门和 Marker 一致性检查。
- [x] 已明确 Ground Truth 隔离。
- [x] 已明确实现前测试、失败回退和停止条件。
- [x] 文档没有把未来工作写成已实现成果。
- [x] 公式、坐标符号与当前代码/SDF 一致。
- [x] 无悬空关键问题阻止制定独立执行计划。

## 23. P8C-0 实现回填

本综述冻结的几何与隔离原则已经在 P8C-0 落地：

- `deck_plane_geometry.hpp/.cpp` 实现甲板平面、X500 四滑橇端点、法向/切向相对运动和显式失败语义；
- 水平甲板严格退化为 `deck_z_ned-uav_z_ned`，±2° roll/pitch 的理论间隙差和首接触端点由单元测试冻结；
- 节点只发布 shadow 诊断，未修改 `TrajectorySetpoint`、`VehicleCommand`、状态机、landing window、触地确认或 touchdown hold；
- `evaluate_p8c_tilted_deck.py` 支持 shadow 话题统计、时间同步/非法数据检查和 evaluator-only Ground Truth 法向角误差；
- P8C-1 已用独立 shadow 法向滤波、完整法向角门和真实四尺度历史重放处理视觉法向度级噪声与 Marker 一致性；最终固定 ±2° 安全高度 12/12 PASS，完整法向最差 RMSE/P95 为 `0.702°/1.353°`，切换最大跳变 `0.426°`；甲板角速度缺失仍是后续动态 P8C-5 风险。
- P8C-2 已完成固定正 `+2° roll/pitch` 的 `0.50 m` 安全下降：两方向各 3/3 PASS，并完成 static、constant02 回归，总计 8/8 PASS；最低真实滑橇间隙 `0.210050755 m`，未发生接触、NAV_LAND 或 Disarm。

P8C-3 失败诊断完成时的历史结论（保留作为 P8C-4 设计输入）：

```text
P8C RESEARCH PASS
P8C PLAN PASS
P8C-0 IMPLEMENTATION PASS
P8C-1 VALIDATION PASS
P8C-2 SAFE DESCENT PASS
P8C-3 BLOCKED BY ATTITUDE-ALIGNMENT DECISION GATE
P8C T1 VALIDATION NOT PASSED
```

P8C-3 已完成正倾角启动门、接触 evaluator、自动真实实验和回归。`+2° roll` seed1 通过；seed2 独立重跑以滑移 `0.106767 m > 0.10 m` 失败，归档轮还出现约 `60.97°/55.44°` roll/pitch、离板和恢复。姿态发散先于最长视觉丢失，不能用视觉先失效解释。static 3/3、H1 1/1 通过；constant02 新 P8C 门 3/3、旧 P6B 双评测 2/3。

因此策略 A 不能在固定 T1 被保留。后续已执行 `docs/plans/P8C3_ATTITUDE_ALIGNMENT_DECISION_GATE.md` 与 `docs/plans/P8C4_TERMINAL_CONTACT_STABILIZATION_PLAN.md`，选择 Offboard position 模式内的终端主轴法向整形、接触顺应、状态化锚点、切向阻尼和受限预压，而不是 PX4 attitude setpoint 姿态对齐。最终状态为 `P8C-4 VALIDATION PASS / P8C T1 VALIDATION PASS / P8C-3 DESIGN GATE CLOSED`。负倾角、动态 roll/pitch 与 combined 仍未开放，当前下一阶段为 P9 统一评测。

---

## 24. 参考文献与正式资料

1. P. Vlantis, P. Marantos, C. P. Bechlioulis, K. J. Kyriakopoulos, “Quadrotor landing on an inclined platform of a moving ground vehicle,” IEEE ICRA, 2015, pp. 2202–2207. DOI: <https://doi.org/10.1109/ICRA.2015.7139490>.
2. T. Baca, P. Stepan, V. Spurny, et al., “Autonomous landing on a moving vehicle with an unmanned aerial vehicle,” *Journal of Field Robotics*, 36(5), 2019, pp. 874–891. DOI: <https://doi.org/10.1002/rob.21858>.
3. A. Paris, B. T. Lopez, J. P. How, “Dynamic Landing of an Autonomous Quadrotor on a Moving Platform in Turbulent Wind Conditions,” 2019/2020. <https://arxiv.org/abs/1909.11071>.
4. S. Abujoub, J. McPhee, R. A. Irani, “Methodologies for landing autonomous aerial vehicles on maritime vessels,” *Aerospace Science and Technology*, 106, 106169, 2020. DOI: <https://doi.org/10.1016/j.ast.2020.106169>.
5. G. Cho, J. Choi, G. Bae, H. Oh, “Autonomous ship deck landing of a quadrotor UAV using feed-forward image-based visual servoing,” *Aerospace Science and Technology*, 130, 107869, 2022. DOI: <https://doi.org/10.1016/j.ast.2022.107869>.
6. O. Procházka, F. Novák, T. Báča, P. M. Gupta, R. Pěnička, M. Saska, “Model predictive control-based trajectory generation for agile landing of unmanned aerial vehicle on a moving boat,” *Ocean Engineering*, 313, 119164, 2024. DOI: <https://doi.org/10.1016/j.oceaneng.2024.119164>.
7. J. E. Kooi, R. Babuška, “Inclined Quadrotor Landing using Deep Reinforcement Learning,” 2021. <https://arxiv.org/abs/2103.09043>.
8. M. Ikura, L. Miyashita, M. Ishikawa, “Stabilization System for UAV Landing on Rough Ground by Adaptive 3D Sensing and High-Speed Landing Gear Adjustment,” *Journal of Robotics and Mechatronics*, 33(1), 2021, pp. 108–118. DOI: <https://doi.org/10.20965/jrm.2021.p0108>.
9. L. Wu, Y. Liang, Q. Li, “The dynamic analysis of the rotorcraft with robotic landing gear in the landing process on uneven ground,” *Advances in Mechanical Engineering*, 14(6), 2022. DOI: <https://doi.org/10.1177/16878132221107473>.
10. PX4, `VehicleOdometry` message reference: <https://docs.px4.io/main/en/msg_docs/VehicleOdometry>.
11. PX4, `VehicleLocalPosition` message reference: <https://docs.px4.io/main/en/msg_docs/VehicleLocalPosition>.
12. 本地 PX4 X500 SDF：`/home/j/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf`。
13. 本地下视相机 SDF：`/home/j/PX4-Autopilot/Tools/simulation/gz/models/x500_mono_cam_down/model.sdf`。
14. 项目甲板 SDF：`src/moving_deck_sim/models/moving_deck/model.sdf`。
