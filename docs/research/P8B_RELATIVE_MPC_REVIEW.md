# P8B 水平相对运动线性 MPC 综述与论文级模型

## 状态

```text
RESEARCH PASS
IMPLEMENTATION PASS
VALIDATION PASS
依赖核验、实现与验收日期：2026-08-01
```

调研、项目模型、候选方案、PX4 接口和回退策略已经明确。初始环境确实没有可供 C++/CMake 使用的 QP 求解器；现已从官方仓库按固定 tag 构建 OSQP `v1.0.0` 与 OsqpEigen `v0.11.2`，安装到独立用户前缀，并完成最小求解示例、生产实现、全工作区测试和严格顺序的真实 PX4 SITL。P8B 已达到 `VALIDATION PASS`，详细结果见 `docs/P8B_RELATIVE_MPC_VALIDATION.md`。

## 1. 研究问题

当前 P4.7 使用“预测甲板位置 + 甲板速度前馈 + 相对速度反馈”的规则式水平控制，已经完成 static、constant02、constant、sinusoidal 和触地基线。P8B 只替换水平相对运动控制，不修改垂直下降、landing window、TouchdownDetector、P8A hold 或状态机。

输入为视觉估计的甲板 NED 位置/速度、PX4 UAV NED 位置/速度和当前水平控制输出；输出为 PX4 `TrajectorySetpoint.acceleration[x,y]`，同时保留位置目标和垂直通道。目标是显式处理水平位置、相对速度、加速度和控制增量约束，并在求解失败时无扰回退 P4.7。

本阶段不解决完整六自由度动力学、姿态控制、垂直 MPC、接触动力学、倾斜甲板和波浪预测。

## 2. 当前项目接入位置

坐标统一为 PX4 `local_ned`：x 北、y 东、z 下。当前节点已具有：

- UAV 位置、速度；
- 视觉甲板估计位置、速度、短时预测；
- P4.7 水平位置目标、速度前馈和加速度限幅；
- 约 20 Hz 的 ROS 控制循环；
- PX4 `TrajectorySetpoint`。

PX4 官方说明：位置 setpoint 非 NaN 时，非 NaN 的速度和加速度字段作为前馈；所有位置、速度、加速度均使用 NED，单位分别为 m、m/s、m/s²。因此第一版 MPC 可保持当前位置 setpoint，使用 MPC 的首个水平加速度作为 acceleration feedforward，避免改动 PX4 内环结构。

## 3. 方法分类

### 3.1 保留规则式 P4.7

优点是已验证、依赖少、回退可靠；缺点是约束和预测时域隐式，参数耦合较强。它必须继续作为默认模式和 MPC fallback。

### 3.2 线性相对状态 MPC

使用二维双积分相对模型，在线解凸 QP。优点是模型、约束、代价和论文表达清晰，计算规模小，与现有外环兼容；缺点是忽略姿态和高阶空气动力学，依赖甲板速度/加速度估计质量。

### 3.3 全状态 NMPC

可直接处理姿态、推力和非线性动力学，代表工作表明可用于高性能四旋翼轨迹跟踪。缺点是建模、求解器和调参复杂度明显更高，会扩大本阶段范围，也不适合在 P4.7 基线上做最小替换。

### 3.4 鲁棒、Tube、学习增强 MPC

适合显式处理有界扰动或复杂空气动力学，但需要扰动集合、观测器或学习模型。当前项目低速移动平台水平跟踪不需要第一版引入这些复杂度。

## 4. 代表性工作与可借鉴内容

1. Alexis、Nikolakopoulos、Tzes，2014，*Trajectory Tracking Model Predictive Control of an Unmanned Quadrotor Helicopter Subject to Aerodynamic Disturbances*，Asian Journal of Control，DOI `10.1002/asjc.587`。说明 MPC 可用于四旋翼约束轨迹跟踪；本项目只借鉴外环轨迹跟踪思想，不照搬完整气动模型。
2. Wang 等，2021，*Efficient Nonlinear Model Predictive Control for Quadrotor Trajectory Tracking: Algorithms and Experiment*，IEEE Transactions on Cybernetics，DOI `10.1109/TCYB.2020.3043361`。证明 NMPC 有实验价值，但第一版线性相对模型更符合最小实现原则。
3. Paris、Lopez、How，2019，*Dynamic Landing of an Autonomous Quadrotor on a Moving Platform in Turbulent Wind Conditions*，arXiv `1909.11071`。采用移动平台估计、在线滚动规划和鲁棒跟踪，说明移动平台降落必须联合估计、规划和控制；其控制链并非本项目要实现的线性 MPC。
4. Wang 等，2022，*Quadrotor Autonomous Landing on Moving Platform*，arXiv `2208.05201`。使用 ArUco、局部规划和状态机完成移动平台降落，与本项目感知和状态机结构相关，但不是 MPC 直接实现来源。
5. Izadi 等，2026，*Fixed-Time Dynamic Landing of Quadrotors using Adaptive Unscented Kalman Filtering and Nonlinear Model Predictive Control*，arXiv `2606.02658`。将移动平台估计、固定触地时间轨迹和 NMPC结合；适合作为后续终端时序优化参考，不纳入第一版水平 MPC。
6. Stephenson、Greeff，2026，*Impact-Aware Model Predictive Control for UAV Landing on a Heaving Platform*，arXiv `2604.21078`。处理升沉接触与反弹，属于接触动力学 MPC；P8A 已用最小相对保持解决当前问题，P8B 禁止扩展到该范围。
7. Torrente 等，2021，*Data-Driven MPC for Quadrotors*，arXiv `2102.05773`。高速度空气动力学学习增强 MPC 可降低跟踪误差，但当前低速移动甲板不需要数据驱动模型。
8. Stellato 等，2020，*OSQP: an operator splitting solver for quadratic programs*，Mathematical Programming Computation，DOI `10.1007/s12532-020-00179-2`。OSQP 支持凸 QP、warm start、不可行检测和嵌入式代码生成，是本项目首选求解器。
9. Ferreau、Bock、Diehl，2008，*An online active set strategy to overcome the limitations of explicit MPC*，International Journal of Robust and Nonlinear Control，DOI `10.1002/rnc.1251`。qpOASES 利用相邻 QP 活跃集变化小进行热启动，是备选方案。
10. Andersson 等，2019，*CasADi — A software framework for nonlinear optimization and optimal control*，Mathematical Programming Computation，DOI `10.1007/s12532-018-0139-4`。适合 NMPC 和符号建模，但对当前固定线性 QP 过重。
11. acados 官方文档及仓库。其面向实时 NMPC/MHE，并可结合 CasADi 生成 C 代码；能力强但集成复杂度高于第一版需要。
12. PX4 ROS 2 Offboard 与 Offboard Mode 官方文档。明确 `TrajectorySetpoint` 的 NED 坐标与 position/velocity/acceleration 前馈语义，是接口映射依据。

## 5. 求解器与依赖比较

| 方案 | 问题类型 | 优点 | 缺点 | 许可证 | 当前机器 |
|---|---|---|---|---|---|
| OSQP + OsqpEigen | 稀疏凸 QP | warm start、不可行检测、固定结构重复求解、C++ 接口清晰 | 一阶法收敛精度与迭代数需测量 | Apache-2.0 / BSD-3-Clause | 已固定安装并通过最小求解 |
| qpOASES | 稠密/参数 QP | 在线 active-set、热启动、严格迭代预算思路成熟 | 老旧集成和稀疏扩展较弱 | LGPL-2.1 | 未安装 |
| acados | OCP/NMPC | 实时迭代、结构化求解、高性能 | 对 4 状态线性 QP 过重 | 以官方仓库 LICENSE 为准 | 未安装 |
| CasADi | 符号优化框架 | 建模灵活、可接多种 NLP/QP solver | 不是单独求解器，C++ 集成更重 | LGPL | 未安装 |
| 手写无约束 LQR | 无约束线性反馈 | 无外部依赖 | 不能表达 MPC 约束和有限时域，不得冒充 MPC | — | 不采用 |

初始检查覆盖 `dpkg`、apt、`pkg-config`、CMake package、`/usr`、`/usr/local`、`/opt`、当前 Python、Conda 和 ROS 2 underlay。系统与当前 Python 均未发现可供本项目使用的 OSQP/OsqpEigen；两个 Conda 环境仅带 Python `osqp` codegen 头文件，不构成系统 C++ 依赖。

最终固定依赖如下：

| 项目 | 固定版本 | 官方 tag commit | 许可证 | 安装前缀 |
|---|---|---|---|---|
| OSQP | `v1.0.0` | `236713ce9a56c182ac3230d52108f952afce1523` | Apache-2.0 | `~/.local/p8b-mpc/osqp-1.0.0-osqpeigen-0.11.2` |
| OsqpEigen | `v0.11.2` | `7587e6994dc194cf22511d909bf4cc5d5e0e4eb2` | BSD-3-Clause | 同上 |

安装方式为官方源码固定 tag 的 Release 构建，不使用 sudo、不修改 `/usr/local`、不复制第三方源码进本仓库。项目通过 `find_package(osqp REQUIRED)` 和 `find_package(OsqpEigen 0.11.2 EXACT REQUIRED)` 查找，生产目标链接 `OsqpEigen::OsqpEigen`；后者再链接 `osqp::osqp`。OSQP `v1.0.0` 官方生成的 CMake version 文件错误地报告 `0.0.0`，因此控制器初始化时额外校验 `osqp_version()=="1.0.0"`，避免误链接其他版本。

最小验证使用官方 OsqpEigen MPC 示例：CMake 配置和链接成功，OSQP 返回 `solved`，25 次迭代，单次求解约 `0.25 ms`，目标值约 `-105.0`。

## 6. 数学模型

定义甲板相对 UAV 的水平误差：

```text
e_k = p_deck,k^NED - p_uav,k^NED ∈ R² [m]
v_rel,k = v_deck,k^NED - v_uav,k^NED ∈ R² [m/s]
x_k = [e_x, e_y, v_rel,x, v_rel,y]^T ∈ R⁴
u_k = a_uav,k^NED ∈ R² [m/s²]
d_k = a_deck,k^NED ∈ R² [m/s²]
```

正号含义：`e_x>0` 表示甲板在 UAV 北侧；`u_x>0` 表示向北加速。连续模型：

```text
d e / dt = v_rel
d v_rel / dt = d - u
```

零阶保持、采样周期 `T_s` 离散化：

```text
x_{k+1} = A x_k + B u_k + E d_k
```

其中：

```text
A = [[I2, T_s I2],
     [02, I2]]                  ∈ R^(4×4)

B = [[-0.5 T_s² I2],
     [-T_s I2]]                 ∈ R^(4×2)

E = [[ 0.5 T_s² I2],
     [ T_s I2]]                 ∈ R^(4×2)
```

第一版令 `d_k` 为当前视觉甲板加速度估计在预测时域内保持常值；估计无效时置零并增加诊断。该项是可测外部输入，不是 Ground Truth。模型假设 PX4 内环能跟踪受限水平加速度，横纵耦合、姿态饱和和气动力作为模型失配由反馈与约束吸收。

## 7. 目标函数与约束

预测时域 `N`，控制增量 `Δu_k=u_k-u_{k-1}`。优化：

```text
min Σ(k=0..N-1) [x_k^T Q x_k + u_k^T R u_k + Δu_k^T S Δu_k]
    + x_N^T P x_N + ρ ||ε||²
```

建议第一版参数范围：`T_s=0.05 s`，`N=20~30`。Q 强调位置误差和相对速度，R 抑制加速度，S 抑制目标跳变，P 为终端权重，ε 只用于必要的软速度约束。

硬约束：

```text
|u_x|, |u_y| <= a_max
|Δu_x|, |Δu_y| <= Δa_max
```

速度约束优先软化：

```text
|v_uav,x|, |v_uav,y| <= v_max + ε,  ε >= 0
```

第一版不对位置误差设硬约束，避免初始大误差导致不可行。所有阈值从当前 P4.7 限幅继承起步，不为单次实验放宽安全边界。

## 8. 候选方案比较

| 方案 | 精度 | 实时性 | 复杂度 | 依赖 | 可解释性 | 兼容性 |
|---|---|---|---|---|---|---|
| P4.7 规则式 | 已验证 | 高 | 低 | 无 | 中 | 最高 |
| 4 状态线性相对 MPC + OSQP | 适合低速水平相对运动 | 高，需实测 P95 | 中 | OSQP/OsqpEigen | 高 | 高 |
| 全状态 NMPC + acados/CasADi | 高模型保真 | 中到高 | 高 | 多 | 中 | 低 |
| Tube/学习增强 MPC | 抗扰潜力高 | 取决于实现 | 很高 | 多 | 中低 | 低 |

## 9. 推荐方案

选择并已实现“4 状态二维相对双积分线性 MPC + OSQP/OsqpEigen”。QP 稀疏结构固定，运行时只更新初始状态、可测甲板加速度扰动、线性项和边界；OSQP 原生状态信息用于发布 iteration/solve time/objective 等诊断，OsqpEigen 提供固定的 CMake/Eigen 集成。该实现直接输出 PX4 接口所需的水平加速度前馈。

P4.7 保持默认；MPC 显式启用。节点并行维护完整 P4.7 控制器，任一 NaN、状态过期、solver 非 solved、超时或输出越界时，本周期回退 P4.7。真实接触验收还表明 MPC 水平加速度在终端阶段会通过姿态耦合影响垂直接触，因此最终实现从 `FINAL_DESCENT` 起使用 `TERMINAL_PHASE_P47` 安全 handoff；该计划内切换不计为 solver failure，不触发 Land/Disarm。

暂不选择 NMPC、acados、CasADi、鲁棒 MPC、学习增强 MPC，因为它们超出第一版问题规模，也会使论文模型与当前代码接口不一致。

## 10. 论文写作映射

- 问题描述：移动甲板水平相对状态和 P4.7 局限；
- 系统模型：上述连续/离散相对双积分模型；
- 控制器设计：有限时域二次代价、硬输入约束、软速度约束；
- 接口框图：视觉估计 → 相对状态 → MPC/P4.7 fallback → PX4 TrajectorySetpoint；
- 实验：P4.7 与 MPC 在 static、constant02、constant、sinusoidal、H1 的 RMSE、相对速度、平滑度、求解时间和成功率对比；
- 消融：无加速度扰动项、无 Δu 代价、关闭 warm start；
- 参数表：T_s、N、Q/R/S/P、速度/加速度/增量约束；
- 曲线：预测轨迹、首控制量、active constraints、solve time、fallback count。

## 11. 参考文献与官方资料

[1] Alexis et al., DOI `10.1002/asjc.587`.
[2] Wang et al., DOI `10.1109/TCYB.2020.3043361`.
[3] Paris et al., arXiv `1909.11071`.
[4] Wang et al., arXiv `2208.05201`.
[5] Izadi et al., arXiv `2606.02658`.
[6] Stephenson and Greeff, arXiv `2604.21078`.
[7] Torrente et al., arXiv `2102.05773`.
[8] Stellato et al., DOI `10.1007/s12532-020-00179-2` and OSQP official documentation.
[9] Ferreau et al., DOI `10.1002/rnc.1251` and qpOASES official repository/wiki.
[10] Andersson et al., DOI `10.1007/s12532-018-0139-4` and CasADi official documentation.
[11] acados official documentation and official GitHub repository.
[12] PX4 ROS 2 Offboard Control and Offboard Mode official documentation.

## 12. 门槛结论

研究问题、模型、约束、候选方案、求解器、PX4 接口、回退和论文映射均已明确，标记 `RESEARCH PASS`。固定依赖、纯 C++ 控制器、显式模式、完整 P4.7 fallback、终端安全 handoff、ROS 2 诊断、启动脚本和评测扩展均已实现，全工作区 `271` 项测试通过，标记 `IMPLEMENTATION PASS`。

已按 static → constant02 → constant → sinusoidal → H1 → 安全下降 → 真实触地的顺序完成真实 PX4 SITL、Bag 和 P4.7 对比。安全高度 15/15、安全下降 6/6、最终代码真实触地 6/6 PASS，所有有效 MPC 轮次均为 0 deadline miss、0 solver failure、0 unexpected fallback。因此标记 `VALIDATION PASS`，验收文档为 `docs/P8B_RELATIVE_MPC_VALIDATION.md`。
