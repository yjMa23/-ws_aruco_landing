# VRX WAM-V Marine 场景集成契约

本文档定义 marine 环境从 primitive `landing_vessel` 切换到 VRX WAM-V 视觉资产后的几何、数据流和安全边界。它只改变仿真场景与 vessel→deck 固定刚体语义，不改变 production landing controller 的观测边界、Future Twist 算法或下降控制器。

## 1. Problem definition

Marine 环境需要同时满足：

1. 船体视觉从方盒子替换为可识别的 WAM-V 双体船；
2. 海面不再是纯蓝色平板，而使用 VRX 风格的 PBR 水面外观；
3. WAM-V 上增加专门的、水平的 UAV landing platform；
4. ID0～ID6 的 deck-local 标定完全保持 legacy / Marine M1 数值；
5. `MotionProfile` 仍是船体唯一运动源；
6. `/simulation/deck/ground_truth` 始终表示 landing deck center；
7. Marine M2 仍只允许 safe-altitude tracking / deck-motion shadow。

M2 不实现 wave-driven vessel dynamics、buoyancy、hydrodynamics、thruster、wind、current、CFD、RAO 或海况驱动船体响应。

## 2. Upstream VRX audit

官方上游：

```text
repository: https://github.com/osrf/vrx
branch: jazzy
commit: 7609d1bd90ce7edb29d040a082f949e8b089c864
commit date: 2026-01-26
license: Apache License 2.0
```

审计时 upstream 默认分支为 `jazzy`。README 明确说明该分支面向 Gazebo Harmonic + ROS 2 Jazzy；需要继续使用 ROS 2 Humble 的完整 VRX 用户应使用 upstream `humble` branch（该分支对应 Garden/Humble 组合）。本项目实际环境是 Ubuntu 22.04.5 + ROS 2 Humble + Gazebo Harmonic，因此 **不引入完整 VRX ROS/competition stack**。

`vrx_gz/CMakeLists.txt` 的 Harmonic 依赖包括 `gz-sim8`、`gz-common5`、`gz-fuel_tools9`、`gz-math7`、`gz-msgs10`、`gz-transport13`、`gz-plugin2`、`gz-rendering8`、`gz-sensors8`、`gz-utils2` 和 `sdformat14`。本机 `gz sim` 为 8.14.0，`gz sdf` 为 14.9.0，与 Harmonic ABI 匹配；ROS 层版本仍保持 Humble。

## 3. Reused upstream assets

M2 只导入以下固定 commit 中的最小视觉资产，不依赖运行时 VRX package：

```text
vrx_urdf/wamv_description/models/WAM-V-Base/mesh/WAM-V-Base.dae
vrx_urdf/wamv_description/models/WAM-V-Base/mesh/WAM-V_Albedo.png
vrx_urdf/wamv_description/models/WAM-V-Base/mesh/WAM-V_Normal.png
vrx_urdf/wamv_description/models/WAM-V-Base/mesh/WAM-V_Roughness.png
vrx_urdf/wamv_description/models/WAM-V-Base/mesh/WAM-V_Metalness.png
vrx_gz/models/coast_waves/materials/textures/wave_normals.dds
```

WAM-V collision geometry依据同一 commit 的：

```text
vrx_urdf/wamv_description/urdf/wamv_base.urdf.xacro
```

其中官方 `wamv/base_link` 为 canonical vessel reference。M2 不复制发动机、螺旋桨、传感器、推进器布局、competition scoring plugin、wind plugin、buoyancy 或 hydrodynamics plugin。

由于上述二进制 PBR 资产总量较大，本仓库采用 **固定 commit + Git partial clone 的构建期导入**，而不是把整个 VRX workspace 或 Fuel cache 纳入运行时依赖。CMake 只从精确 SHA 导出列出的六个视觉文件并校验预期字节数；构建后所有运行时 URI 都指向 `moving_deck_sim` install share 内的本地文件。运行 marine 场景不需要访问 Gazebo Fuel、GitHub、用户 home cache 或 VRX workspace。来源、commit、复制路径和改动记录见模型目录 `THIRD_PARTY.md`。

## 4. WAM-V model hierarchy

Marine 正式模型名：

```text
vrx_wamv_landing
└── vessel_body                 canonical vessel reference
    ├── WAM-V base visual       official VRX mesh + PBR maps
    ├── WAM-V collision set     copied numeric geometry from wamv_base.urdf.xacro
    ├── landing deck supports   project-added, no dynamics source
    ├── landing platform        project-added 2.4 m × 2.4 m
    └── landing_deck frame
        ├── ID0
        ├── ID1
        ├── ID2
        ├── ID3
        ├── ID4
        ├── ID5
        └── ID6
```

M2 不生成 VRX 原始 thruster joints，不加载 WAM-V propulsion controller。

## 5. Canonical vessel reference frame

`vessel_body` 与 VRX `wamv/base_link` 语义对齐：

- 原点：官方 WAM-V base mesh / collision 的 canonical base reference；
- +x：WAM-V forward；
- +y：left；
- +z：up；
- world：Gazebo ENU。

`MotionProfile` 的 position / orientation / velocity 均作用在该 reference 上。

## 6. landing_deck frame and platform geometry

为了让已有非共面 ID4～ID6 以及多尺度 ID0～ID3 都合法安装，同时避免恢复旧的 5×5 m 超宽方板，M2 landing platform 为：

```text
size: 2.4 m × 2.4 m × 0.10 m
landing plane: platform top surface
landing_deck origin: top-surface center
landing_deck +z: upward deck normal
landing_deck +x/+y: parallel to vessel_body +x/+y
```

官方 collision geometry 的 `top_base` center 位于 `z=1.25 m`、厚度 `0.10 m`，因此其顶面约为 `z=1.30 m`。M2 在 top base 与 landing platform 之间增加固定支撑；landing platform underside 为 `z=1.70 m`，top surface 为 `z=1.80 m`。由此固定变换不是沿用 Marine M1 的临时数值，而是由 WAM-V 几何和新增平台高度得到：

```text
r_vessel_deck = [0, 0, 1.80] m
R_vessel_deck = I
```

平台横向 2.4 m 小于官方 collision envelope 约 2.46 m（左右 pontoon center `±1.03 m`、radius `0.20 m`），不会恢复明显比船体更宽的旧 5×5 m deck。

## 7. Fixed transform T_vessel_deck

记：

```text
T_W_V = (R_W_V, p_V^W)
T_V_D = (R_V_D, r_VD^V)
```

M2：

```text
r_VD^V = [0, 0, 1.80]^T m
R_V_D = I
```

则：

```text
p_D^W = p_V^W + R_W_V r_VD^V
R_W_D = R_W_V R_V_D
```

线速度必须包含固定杠杆臂项：

```text
v_D^W = v_V^W + R_W_V (omega_V^V × r_VD^V)
```

如果后续 landing platform 固定旋转不再为单位阵，必须继续通过 `FixedRigidTransform.rotation_wxyz` 处理，不能在 callback 中增加特殊分支。

## 8. Neutral deck world height

现有 scenario YAML 的 neutral `initial_position_enu.z = 2.0 m` 继续解释为 **landing deck center**。Marine launch 只在一个地方转换到 vessel reference：

```text
neutral_deck_height = 2.0 m
deck_offset_body_z = 1.80 m
neutral_vessel_z = 2.0 - 1.80 = 0.20 m
```

因此 neutral landing deck center 仍为 `world z≈2.0 m`，`--rendezvous-altitude 7.0` 继续对应约 5 m deck-relative safe-altitude 语义。禁止在 world、launch 和 controller 多处分别重新硬编码该换算。

## 9. Marker frame hierarchy

ID0～ID6 的以下量必须逐项等于 legacy / Marine M1：

- marker ID；
- plane size；
- `T_deck_marker_i` translation；
- `T_deck_marker_i` rotation；
- texture URI。

M2 只改变 `T_vessel_deck` 和船体视觉，不重新肉眼摆放 Marker。

## 10. MotionProfile input semantics

唯一船体运动链为：

```text
MotionProfile
  → /model/vrx_wamv_landing/cmd_vel
  → vessel_body
  → fixed T_vessel_deck
  → landing deck
```

继续支持既有：`static`、`constant02`、`constant`、`sinusoidal`、`heave*`、`rollpitch`、`combined`、`rigid_body_motion`。

禁止把 WAM-V propulsion、wave physics、buoyancy 或 hydrodynamics 与 `MotionProfile` 同时施加到同一个刚体。

## 11. Ground Truth semantics

```text
/simulation/deck/ground_truth_raw
```

在 marine 中表示 WAM-V `vessel_body` raw state；它可进入仿真状态转换节点，但不得进入 production landing controller。

```text
/simulation/deck/ground_truth
```

始终表示 `landing_deck` center state，包含固定变换产生的 position / orientation / lever-arm velocity。GNSS simulator 和 evaluator 的既有输入语义不改变。

## 12. Ocean / wave visual semantics

Upstream VRX `coast_waves` 使用 `libWaveVisual.so`、Gerstner vertex/fragment shaders、`Wavefield` 参数和专用 water mesh。M2 审计确认该实现的动态可视化是独立 rendering plugin，但完整复用需要额外维护 VRX plugin source、shader、13 MB water mesh 以及 rendering ABI；它不是 WAM-V visual asset 的必要依赖。

M2 因而采用 VRX-style **visual-only PBR water fallback**：

- 大尺度 plane；
- VRX `wave_normals.dds` 作为 normal map；
- 低 roughness / 适度 specular；
- 无 collision；
- 无 buoyancy / hydrodynamics；
- 无 vessel force coupling。

该水面可以具有纹理法线带来的视觉起伏/高光，但 **不宣称动态 Gerstner WaveVisual 已启用**。真正的动态 wave rendering plugin 构建和 GUI 视觉验证留给 Marine M3，除非本阶段本机验证证明可以无额外风险启用。

## 13. Why M2 does not enable hydrodynamics

M2 的目的是替换视觉场景并保持控制实验因果关系。当前 baseline 已将 vessel motion 明确定义为 deterministic `MotionProfile`。如果同时开启 wave/buoyancy/hydrodynamics：

```text
MotionProfile + fluid force → same vessel
```

则船体存在双运动源，Ground Truth 与 profile 之间不再是一一对应，既破坏可重复 smoke，也混淆后续 Future Twist 因果可观测性诊断。因此 M2 只允许 visual ocean。

## 14. Failure conditions

以下任一情况视为 M2 未完成相应验证：

- WAM-V mesh/PBR asset 无法从精确 commit 获取或安装；
- install share 中存在 unresolved model URI；
- Gazebo 报 missing mesh / texture / material；
- neutral deck center 不在 `z≈2.0 m`；
- ID0～ID6 任意标定变化；
- raw GT 或 deck GT 出现 NaN/Inf；
- rigid-body lever-arm 数值关系不成立；
- marine 允许 descent/contact flag；
- production controller 出现 Ground Truth subscription；
- legacy world/model/spawn 发生语义变化；
- GUI 不可用时把视觉验收写成“通过”。

## 15. Safety boundaries

Marine M2 仍为 safe-altitude only。以下必须在启动前失败，并包含：

```text
marine environment currently supports safe-altitude validation only
```

- `--enable-relative-descent`
- `--enable-final-descent`
- `--terminal-contact-stabilization-shadow`
- `--terminal-contact-stabilization-rehearsal`
- `--enable-terminal-contact-stabilization`

运行时继续要求：

```text
descent.enabled=false
final_descent.enabled=false
enable_auto_land=false
terminal_contact_stabilization.enabled=false
NAV_LAND=0
automatic Disarm=0
```

Ground Truth 只能进入仿真传感器、场景转换和离线 evaluator，不能进入 production landing controller。

## 16. Legacy compatibility

以下 legacy 事实必须保持：

```text
default environment = legacy
legacy world = worlds/aruco_moving_deck.sdf
legacy model = moving_deck
legacy PX4_GZ_MODEL_POSE = -4,0,0.2
```

`models/moving_deck/` 和 `worlds/aruco_moving_deck.sdf` 不因 WAM-V 改造而重写。Marine 正式路径切换到 `vrx_wamv_landing`，旧 primitive `landing_vessel` 仅保留为历史/debug fixture，不再由 `--environment marine` 启动。
