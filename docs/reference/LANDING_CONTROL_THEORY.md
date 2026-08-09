# 移动甲板自主降落控制理论

本文档描述当前代码实际使用的估计、预测、控制、接触几何和安全回退。坐标定义见[坐标系与时间契约](COORDINATE_FRAMES.md)。

## 1. 视觉测量

ArUco PnP 位姿通过完整刚体链转换到 local NED。甲板位置观测记为：

```math
z_k=[p_x,p_y,p_z]^T
```

测量必须满足：时间单调、frame 合法、有限、与 GNSS 粗位置一致，并通过位置跳变和创新门。
多尺度 Marker 的完整刚体位姿统一到甲板着陆参考系 `deck_landing_up`。令该参考系为
`D`，Marker 为 `M`，则：

```math
T_N^D=T_N^B T_B^C T_C^M T_M^D
```

原多尺度 ID 0/1/2/3 共面、方向一致，检测器用 `T_M^D` 的平移把位置补偿到甲板
中心。后续实物若改变 Marker 朝向，必须标定完整 `T_M^D`，不能在估计器中按 Marker
ID 临时翻转符号。

远距三维目标由一个 `0.50 m` 主 Marker 和三个 `0.75 m` 副 Marker 组成：ID 0
保持甲板共面，中心 `[+0.45,0,0]^T m`；ID 4 中心
`[-0.75,0,+0.2852]^T m`、绕 `D` 系 `+Y` 旋转 `+45°`；ID 5 中心
`[0,+0.75,+0.2852]^T m`、绕 `+X` 旋转 `+45°`；ID 6 中心
`[0,-0.75,+0.2852]^T m`、绕 `+X` 旋转 `-45°`。三个倾斜 Marker
的最低边约高于甲板 `0.02 m`。同帧检测到 ID 0/4/5/6 时，把十六个角点先通过
各自 `T_D^M` 表达到 `D`，再一次求解：

```math
u_i=\pi\left(K,T_C^D p_i^D\right),\qquad i=1,\ldots,16
```

三组互相独立的倾斜方向同时约束 roll/pitch 深度可观性，不依靠预知甲板运动。
副 Marker 短时漏检时，只要当前可见的已标定角点仍构成非共面集合，就继续联合 PnP；
退化为共面集合时才保留现有单 Marker 位姿用于兼容诊断，且不得把回退解标记为已验收
的三维目标观测。

甲板向上法向由统一甲板系 +Z 旋转到 local NED：

```math
n_{up}^N=R_N^D[0,0,1]^T
```

低通前先统一法向半球，低通后归一化。法向范数过小、向上分量不足或时间无效时不更新。

## 2. 甲板状态估计

三维常速度模型的状态为：

```math
x=[p_x,p_y,p_z,v_x,v_y,v_z]^T
```

对采样间隔 `dt`：

```math
x_{k+1}=F(dt)x_k+w_k
```

```math
F(dt)=
\begin{bmatrix}
I_3 & dt I_3\\
0 & I_3
\end{bmatrix}
```

位置观测矩阵：

```math
H=[I_3\quad 0]
```

更新前检查 `dt`、协方差和创新 Mahalanobis 距离。短时丢帧只预测；超过重初始化间隔后，下一有效观测重置位置并清空不可信速度。

垂直估计器独立维护甲板 z、z 速度、相对高度与相对垂直速度，避免低高度通道被水平逻辑耦合。甲板垂直速度前馈限制为有限幅值。

## 3. 短时预测

常速度预测：

```math
p_{deck,pred}=p_{deck}+\tau v_{deck}
```

预测器限制：

```math
0\le\tau\le\tau_{max}
```

以及单轴/平面最大位移。估计过期、协方差异常或结果非有限时预测无效，不把上次值伪装成新预测。

### 3.1 ArUco 相对 6-DoF shadow 状态

独立 shadow 不替换上述生产估计器。令 `B=base_link_frd`、`C=camera_optical`、
`D=deck_landing_up`，图像采样时刻的直接视觉链为：

```math
T_B^D=T_B^C T_C^M T_M^D
```

定义 `U_k=uav_centered_ned`：其原点位于第 `k` 个图像时刻的无人机参考点，三轴
平行 `local_ned`。只使用同一图像时刻的 PX4 姿态 `R_N^B`，得到：

```math
r^N=t_U^D=R_N^B t_B^D=p_D^N-p_B^N,
\qquad
R_N^D=R_N^B R_B^D
```

shadow 估计：

```math
x_s=(r^N,v_D^N,a_D^N,R_N^D,\omega_D^N,\alpha_D^N)
```

平移量单位依次为 `m`、`m/s`、`m/s²`，角运动量单位依次为 `rad/s`、
`rad/s²`。`r^N` 是当前 `deck-uav` 相对位移，不是甲板绝对位置；`v_D^N/a_D^N`
是甲板自身的 NED 速度和加速度。相对 MPC 在运行时用 `v_D^N-v_B^N` 构造相对
速度，不要求 shadow 猜测无人机未来闭环响应。`R_N^D` 把
`deck_landing_up` 向量旋转到 NED；因为 `U` 的轴不随无人机旋转，角速度和角加速度
仍是甲板相对 NED 的绝对角运动，并在 NED 轴表达。

该链禁止使用 PX4 绝对位置、甲板 GNSS 或 Ground Truth。PX4 绝对位置仍只服务现有
生产链；shadow 只额外使用图像时刻插值后的 PX4 NED 速度补偿相邻观测原点变化。
Ground Truth 只进入离线 evaluator。

平移误差状态为：

```math
\delta x_s=[\delta r^T,\delta v_D^T,\delta a_D^T]^T
```

采用白噪声 jerk 驱动的常加速度模型：

```math
F_{ca}(\Delta t)=
\begin{bmatrix}
I_3&\Delta tI_3&\frac12\Delta t^2I_3\\
0&I_3&\Delta tI_3\\
0&0&I_3
\end{bmatrix}
```

若单轴 jerk 白噪声谱密度为 `q_j`，单轴离散过程噪声为：

```math
Q_{ca}=q_j
\begin{bmatrix}
\frac{\Delta t^5}{20}&\frac{\Delta t^4}{8}&\frac{\Delta t^3}{6}\\
\frac{\Delta t^4}{8}&\frac{\Delta t^3}{3}&\frac{\Delta t^2}{2}\\
\frac{\Delta t^3}{6}&\frac{\Delta t^2}{2}&\Delta t
\end{bmatrix}
\otimes I_3
```

相邻样本间无人机位移使用时间对齐速度的梯形积分：

```math
\Delta p_B^N\approx\frac12(v_{B,k-1}^N+v_{B,k}^N)\Delta t
```

名义平移传播为：

```math
x_{s,k}^-=F_{ca}(\Delta t)x_{s,k-1}^+
-[\Delta p_B^{N,T},0,0]^T
```

补偿量作为已知输入，当前不把 PX4 速度不确定性重复加入 `Q_ca`。相对位置观测
矩阵为 `H_r=[I_3\ 0\ 0]`。对平移和后述旋转误差状态，协方差更新统一为：

```math
P_k^-=F_kP_{k-1}^+F_k^T+Q_k,
\qquad
S_k=H_kP_k^-H_k^T+R_k
```

```math
d_k^2=y_k^TS_k^{-1}y_k,
\qquad
K_k=P_k^-H_k^TS_k^{-1}
```

只有 `d_k^2\le\gamma^2` 时注入误差。协方差使用 Joseph 形式：

```math
P_k^+=(I-K_kH_k)P_k^-(I-K_kH_k)^T+K_kR_kK_k^T
```

PX4 速度无效、frame 不是 NED、创新
非有限、时间不递增或 Mahalanobis 超门时拒绝更新。

为避免单帧差分放大位置噪声，接受观测后另对最近 `0.30 s` 的滤波相对位置加
已积分无人机位移做局部常加速度最小二乘拟合；至少六个样本时才用拟合的
`v_D^N/a_D^N` 发布和外推，否则保留误差状态滤波值。该拟合不使用周期、场景或未来
真值先验。导数协方差由正规矩阵逆与残差方差给出，残差方差不得低于冻结的
位置测量方差；拟合导数与名义位置的交叉协方差保守地置零。

姿态不能直接对 Euler 角做差。名义姿态预测为 `R_{pred}`、观测为 `R_{meas}` 时，
NED 中的最小旋转创新为：

```math
\delta\theta=\operatorname{Log}(R_{meas}R_{pred}^T)
```

旋转误差状态为：

```math
\delta x_R=[\delta\theta^T,\delta\omega^T,\delta\alpha^T]^T
```

其小误差协方差使用与 `F_ca/Q_ca` 同形的常角加速度模型。校正后左乘名义姿态：

```math
R^+=\operatorname{Exp}([\delta\theta]_{\times})R^-
```

误差注入名义姿态后，协方差使用一阶 reset Jacobian
`G_{\theta}=I-\frac12[\delta\theta]_{\times}` 变换，随后再次对称化。四元数只作为
`R` 的数值载体；更新前归一化并统一半球，但不掩盖真实大角度创新。
重复或倒退时间戳不改变已验收状态；普通长 `dt` 分段传播，超过重初始化间隔则
以新观测重建 `r/R`，清空旧的导数和样本窗。Marker 切换先通过 `T_M^D`
归一到同一甲板系，再使用同一创新门，不在估计器内为 ID 切换引入姿态或平移跳变。

### 3.2 冻结无人机原点的甲板轨迹预测

在轨迹发布时刻 `t` 冻结 `O_t=uav_origin_ned` 原点。先用最近 PX4 速度将最后
图像状态传播到 `t`，得到当前 `r(t)`；再只传播甲板运动：

```math
p_{D/O_t}^N(t+\tau)=r^N(t)+v_D^N(t)\tau+\frac12a_D^N(t)\tau^2
```

```math
v_D^N(t+\tau)=v_D^N(t)+a_D^N(t)\tau
```

姿态以 `0.05 s` 步长用中点角速度积分：

```math
\omega_{k+\frac12}=\omega_k+\frac12\alpha_k\Delta t
```

```math
R_{k+1}=\operatorname{Exp}([\omega_{k+\frac12}]_{\times}\Delta t)R_k,
\qquad
\omega_{k+1}=\omega_k+\alpha_k\Delta t
```

当前状态使用 `uav_centered_ned`，预测轨迹使用 `uav_origin_ned`，从发布时刻起按
`0.05 s` 采样并固定发布到 `1.0 s`。每个轨迹点表示甲板相对发布时刻无人机原点的
未来状态；未来 MPC 用自身状态和控制输入形成相对动力学，不得把 shadow 轨迹直接
当成控制指令。设最后有效视觉样本年龄为 `t_age`、轨迹点
相对当前发布时刻为 `\tau`，只有满足：

```math
t_{age}+\tau\le0.5\ \text{s}
```

的点属于可信 shadow；`0.5～1.0 s` 只提供低置信度诊断。时间倒退、非有限状态、
协方差非法、创新离群或视觉年龄超过最大诊断时间时不发布可信轨迹。超过重初始化间隔
后，下一帧有效观测重新初始化，速度和加速度不得沿用旧值。

### 3.3 相对离线误差定义

预测点按目标时间 `t+\tau` 与甲板 Ground Truth 对齐，同时按轨迹 header 时刻 `t`
与无人机 Ground Truth 对齐。Bag 末尾缺少未来甲板真值或 header 时刻无人机真值的
点排除，不记为失败。Gazebo world ENU 位置必须先
通过 PX4 `VehicleLocalPosition.ref_*` 地理参考转换到同一 `local_ned`，不能只交换
ENU/NED 分量。先构造：

```math
p_{D/O_t,gt}^N(t+\tau)=p_{deck,gt}^N(t+\tau)-p_{uav,gt}^N(t),
\qquad
v_{gt}^N=v_{deck,gt}^N(t+\tau)
```

水平和垂直相对位置误差为：

```math
e_{xy}=\lVert(r_{pred}-r_{gt})_{xy}\rVert_2,
\qquad
e_z=|r_{pred,z}-r_{gt,z}|
```

完整旋转、法向和 yaw 误差分别为：

```math
e_R=\cos^{-1}\left(\frac{\operatorname{tr}(R_{pred}^TR_{gt})-1}{2}\right)
```

```math
e_n=\cos^{-1}(\operatorname{clamp}(n_{pred}^Tn_{gt},-1,1))
```

```math
e_\psi=|\operatorname{wrap}_{[-\pi,\pi)}(\psi_{pred}-\psi_{gt})|
```

甲板线速度误差按水平范数和垂直绝对值统计，甲板角速度误差为
`\lVert\omega_{D,pred}^N-\omega_{D,gt}^N\rVert_2`。加速度与角加速度完整报告但
不设本轮硬门。原数值门限保持不变；位置门使用冻结原点的相对位置，twist 门使用
甲板自身 twist。Ground Truth 只能由 evaluator 读取，禁止订阅回控制器。

## 4. 默认水平控制

水平位置目标来自受限预测位置：

```math
p_{sp,xy}=limit(p_{deck,pred,xy})
```

速度前馈为：

```math
v_{ff,xy}=v_{deck,xy}+k_v(v_{deck,xy}-v_{uav,xy})
```

其中 `k_v` 根据估计甲板水平加速度连续调度：低加速度使用较低相对速度阻尼，高加速度逐渐提高阻尼，调度结果再次限幅。

输出同时受：

- 位置目标偏移限制。
- 速度前馈限制。
- 加速度前馈限制。
- 目标变化率限制。

视觉短时丢失时位置目标保持，速度/加速度前馈平滑衰减；长时丢失进入 GNSS 恢复。

## 5. 水平相对 MPC

### 5.1 状态和模型

二维相对状态：

```math
x_k=[e_x,e_y,v_{rel,x},v_{rel,y}]^T
```

定义以代码实际契约为准：

```math
e=p_{deck}-p_{uav},
\qquad
v_{rel}=v_{deck}-v_{uav}
```

控制量 `u` 是 UAV 水平加速度前馈，`d` 是由视觉估计得到的甲板水平加速度。离散双积分模型：

```math
x_{k+1}=Ax_k+Bu_k+Ed_k
```

```math
A=
\begin{bmatrix}
I_2&T_sI_2\\
0&I_2
\end{bmatrix},
\quad
B=
\begin{bmatrix}
-\frac12T_s^2I_2\\
-T_sI_2
\end{bmatrix},
\quad
E=
\begin{bmatrix}
\frac12T_s^2I_2\\
T_sI_2
\end{bmatrix}
```

甲板加速度在有限预测时域内保持当前估计值；不可用时置零并输出诊断，禁止使用仿真轨迹或 Ground Truth。

### 5.2 代价和约束

QP 目标：

```math
\min \sum_{k=0}^{N-1}
(x_k^TQx_k+u_k^TRu_k+\Delta u_k^TS\Delta u_k)
+x_N^TPx_N+\rho\lVert\epsilon\rVert^2
```

硬约束包括水平加速度与控制增量；速度约束可通过松弛变量软化，位置误差不设硬约束以避免会合初期不可行。

固定 QP 稀疏结构只在初始化时建立；每周期更新初始状态、扰动、边界和线性项，并使用 OSQP warm start。

### 5.3 PX4 接口和回退

MPC 只填写：

```text
TrajectorySetpoint.acceleration[0:2]
```

位置目标和垂直通道保持现有控制。以下任一情况立即使用本周期规则式输出：

- 输入过期或非有限。
- 求解状态不是成功。
- 求解超时或违反约束。
- 首个控制量非有限或越界。

进入 `FINAL_DESCENT`、`TOUCHDOWN_CANDIDATE_HOLD` 或 `TOUCHDOWN_HOLD` 后主动切换为 `TERMINAL_RULE_BASED_TRACKING`，避免水平加速度通过姿态耦合影响垂直接触。

## 6. 着陆窗口

窗口输入包括水平误差、水平相对速度、视觉年龄、预测有效性、甲板倾角和相对高度。每个连续量有进入阈值和更宽松的退出阈值。

只有全部条件连续满足 `required_duration` 后窗口才打开；任一硬失效立即关闭。拒绝原因以位掩码发布，便于确定是跟踪、速度、视觉、姿态、高度还是估计问题。

## 7. 相对高度下降

NED 中：

```math
h_{rel}=z_{deck}-z_{uav}
```

下降参考按速率积分：

```math
h_{ref,k+1}=max(h_{min},h_{ref,k}-r(h_{ref})dt)
```

世界 z 目标：

```math
z_{sp}=z_{deck,pred}-h_{ref}
```

垂直速度前馈：

```math
v_{ff,z}=v_{deck,z}-\dot h_{ref}
```

再施加增益和幅值限制。窗口关闭时冻结 `h_ref`；估计严重失效时恢复爬升，并锁止本任务再次下降。

## 8. 最终下降

从 `0.50 m` 开始，最终下降使用分段速率：接近段、近接触段和安全终端段。参考最低降至 `0.05 m`，任何跟踪误差、视觉年龄、窗口或估计硬门失败都请求恢复。

候选触地后立即冻结下降参考，避免候选确认期间继续压低命令。确认后转入接触保持。

## 9. 触地证据

触地不是单一高度阈值。候选证据联合：

- PX4 ground contact / landed 状态。
- 视觉相对高度。
- 相对垂直速度。
- 水平相对速度。
- 视觉滑橇最小间隙。

候选必须连续满足 `candidate_duration` 并具有迟滞。视觉高度只能提供几何接近证据，不能单独确认。确认结果锁存，恢复或任务 reset 才清除。

## 10. 甲板平面和滑橇几何

甲板平面：

```math
(n^N)^T(p^N-p_d^N)=0
```

X500 四个滑橇端点在 FRD 中为：

```text
[-0.125, -0.132, 0.227]
[+0.125, -0.132, 0.227]
[-0.125, +0.132, 0.227]
[+0.125, +0.132, 0.227] m
```

端点变换到 NED 后计算：

```math
h_i=(n^N)^T(p_{skid,i}^N-p_d^N)
```

```math
h_{min}=\min_i h_i,
\qquad
h_{max}=\max_i h_i
```

`h_min` 表示首接触几何，`h_max-h_min` 表示四点相对甲板平面的展开。动态甲板接触点速度还包含角速度项：

```math
v_{deck,c}=v_d+\omega_d\times r_c
```

当前生产控制只使用视觉法向、甲板估计状态和 PX4 odometry；Ground Truth 平面只用于 evaluator 对比。

## 11. 终端接触稳定化

固定正小倾角接触时，从视觉向上法向构造期望有限倾角。为保持 yaw，不直接发送 attitude setpoint，而是在 Offboard position 模式内增加水平加速度偏置。

若期望机体 z 轴为 `b_z^N`，重力为 `g`，小倾角近似下水平偏置为：

```math
a_{bias,xy}\approx-g\frac{[b_{z,N},b_{z,E}]^T}{b_{z,D}}
```

偏置受目标倾角、加速度和 slew rate 限制；法向失效时平滑回零。

候选阶段以当前名义目标建立甲板相对接触锚点。保持阶段：

- 锚点随估计甲板位置移动。
- 在顺应区内允许小范围中心调整。
- 平面内速度阻尼限制滑移。
- 锁存候选结束前最后有效法向。
- 相对高度参考和向下加速度预压均独立限幅。

姿态偏差或角速度连续超过安全门时请求恢复；恢复、Abort 或任务 reset 清空偏置、锚点和预压。

## 12. Marine vessel 刚体几何（仅仿真语义）

Marine 第一版把解析运动参考点从甲板中心改为 `vessel_body`，控制目标仍然是 landing deck。固定变换为 `T_V_D`，其中第一版 `r_{VD}^V=[0,0,2]^T m`、`R_V_D=I`。Gazebo world ENU 中：

```math
p_D^W=p_V^W+R_W^V r_{VD}^V
```

输入线速度在 world 表达、角速度在 vessel body 表达时：

```math
v_D^W=v_V^W+R_W^V(\omega_V^V\times r_{VD}^V)
```

这一步只在 `moving_deck_sim` 中把 vessel raw Ground Truth 转换成 deck-center Ground Truth，确保 roll/pitch 的甲板中心位置包含杠杆臂运动。它**不是**控制器可用的状态来源；生产控制仍只使用 PX4 state、模拟 GNSS、camera/ArUco 与内部 estimator。

Marker 的 `T_deck_marker_i` 保持 legacy 标定不变，marine 只是把整个 landing deck 作为 `vessel_body` 的固定刚体子 frame。完整仿真契约见 [MARINE_VESSEL_KINEMATICS.md](MARINE_VESSEL_KINEMATICS.md)。

## 13. 动态姿态限制

动态 `rollpitch` 和 `combined` 与固定倾角不同：法向随时间变化，存在视觉滤波相位延迟，甲板局部接触点速度还需要可靠角速度。当前尚未完成法向变化率/角速度可观测性验证，因此这些场景只允许约 `5 m` 安全高度 shadow，不允许下降、真实接触或使用 Ground Truth 驱动控制。Marine 环境对所有 scenario 进一步限制为 safe-altitude only，不能借由静态或固定倾角 profile 绕过下降/接触安全门。
