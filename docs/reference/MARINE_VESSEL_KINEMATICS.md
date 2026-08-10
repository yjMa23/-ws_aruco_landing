# Marine Vessel 刚体运动学契约

> 本文保留 Marine M1 的通用刚体公式与安全边界；其中 primitive `landing_vessel`、5×5 m deck、neutral vessel z≈0 和 `r_VD≈[0,0,2] m` 是 M1 fixture 数值。Marine M2 正式 WAM-V 几何改为 `r_VD=[0,0,1.8] m`、neutral vessel z≈0.2 m、2.4×2.4 m landing platform，详见 [`VRX_WAMV_INTEGRATION.md`](VRX_WAMV_INTEGRATION.md)。

本文档定义 marine 环境中 `vessel_body → landing deck` 的通用刚体几何与 Ground Truth 语义。该模型只用于仿真场景、传感器模拟和离线评测，不改变生产控制器的数据边界。

## 1. 问题定义与边界

legacy 环境把 `moving_deck` 模型原点直接定义在甲板中心；marine 环境把运动参考点改为船体参考点：

```text
MotionProfile → vessel_body → fixed T_vessel_deck → landing deck
```

第一版只实现解析刚体运动，不实现浮力、水动力、海浪、RAO、JONSWAP、风、流或 VRX。

## 2. 坐标系、单位与时间

- `W`：Gazebo world ENU，x 东、y 北、z 上。
- `V`：`vessel_body`，原点位于船体中部接近水线/参考点，neutral world z 约为 0 m。
- `D`：landing deck，原点位于 5 m × 5 m 着陆甲板表面中心；第一版与 `V` 轴平行。
- 位置：m；线速度：m/s；角速度：rad/s。
- `MotionProfile` 的位置和线速度在 `W` 中表达；`angular_velocity_body` 在 `V` 中表达。
- 所有状态沿用 ROS/Gazebo 仿真时间，不引入额外时间基准。

## 3. 固定刚体变换

记 `T_W_V=(R_W_V,p_V^W)`，固定 `T_V_D=(R_V_D,r_{VD}^V)`。第一版 marine 使用：

```text
r_vessel_deck = [0, 0, 2] m
R_vessel_deck = I
```

甲板姿态：

```math
R_W_D = R_W_V R_V_D
```

甲板中心位置：

```math
p_D^W = p_V^W + R_W_V r_{VD}^V
```

因此 roll/pitch 不仅改变甲板法向，也会因为 2 m 杠杆臂改变甲板中心位置。

## 4. 速度转换

输入船体线速度 `v_V^W` 在 world ENU 表达，船体角速度 `ω_V^V` 在 vessel body 表达。固定点 D 的 world 线速度为：

```math
v_D^W = v_V^W + R_W_V(\omega_V^V \times r_{VD}^V)
```

若 `R_V_D` 非单位旋转，甲板自身坐标表达的角速度为：

```math
\omega_D^D = R_D_V \omega_V^V = R_V_D^T \omega_V^V
```

第一版 `R_V_D=I`，因此 deck 与 vessel 的 body-frame 角速度数值相同。

## 5. 输入、输出和失败条件

纯数学模块输入：

- vessel world position；
- vessel world orientation quaternion；
- vessel world linear velocity；
- vessel body angular velocity；
- fixed vessel→deck translation；
- fixed vessel→deck rotation quaternion。

输出：landing deck world pose、world linear velocity、deck-body angular velocity。

以下输入必须拒绝并抛出 `std::invalid_argument`：

- 任一位置、速度、角速度、offset 或 quaternion 分量为 NaN/Inf；
- vessel 或 fixed-transform quaternion 范数过小；
- 无法归一化的姿态。

合法 quaternion 在计算前归一化，避免小的数值漂移进入几何结果。

## 6. Ground Truth 数据边界

Gazebo marine raw odometry 可以表示 `landing_vessel/vessel_body`，并继续通过 `/simulation/deck/ground_truth_raw` 进入仿真控制节点；该 raw topic 不得被生产 landing controller 订阅。

最终 `/simulation/deck/ground_truth` 必须始终表示 landing deck center：

```text
vessel raw GT
  → rigid_body_kinematics
  → deck-center GT
  → GNSS sensor simulation / offline evaluator
```

因此 legacy 环境采用零 offset，marine 环境采用约 `[0,0,2] m` offset；下游 GNSS、评测器和历史 topic 名保持不变。

## 7. 适用范围与安全边界

本模型假设 `landing deck` 与 `vessel_body` 构成理想刚体，固定变换不随时间变化。它不描述船体弹性、甲板机构运动、水动力载荷或随机海况。

marine 第一版只允许 GNSS rendezvous、视觉捕获、安全高度跟踪和 deck-motion shadow；禁止相对下降、最终下降、真实接触、`NAV_LAND` 和自动 Disarm。动态 roll/pitch 的既有安全门不得放宽。

## 8. 单元测试验收

必须覆盖：

- zero offset 恒等关系；
- pure translation；
- pure roll / pitch / yaw；
- 非零角速度 + 2 m lever arm；
- fixed transform 旋转；
- NaN/Inf 和非法 quaternion；
- marine neutral `vessel z=0 → deck z≈2 m`。

数值比较使用明确容差，不通过 ROS callback 间接测试几何公式。
