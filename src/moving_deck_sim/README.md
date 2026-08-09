# moving_deck_sim

Gazebo Harmonic/ROS 2 移动甲板、marine 船体刚体场景和船舶 GNSS 仿真包。

## 环境

`legacy` 为默认环境，继续使用：

```text
worlds/aruco_moving_deck.sdf
models/moving_deck/
```

其模型原点就是 landing deck center，历史 P9/P10 与正式 baseline 的默认启动路径不变。

`marine` 为显式可选环境，使用：

```text
worlds/aruco_marine_vessel.sdf
models/landing_vessel/
```

marine world 保持 `world=aruco`、ENU、球面坐标和 250 Hz 物理更新，新增约 `300×300 m` 无碰撞视觉海面、`x=-12 m` 附近的静态 UAV launch platform，以及 primitive-only `landing_vessel`。第一版不包含 VRX、外部 mesh、wave、RAO、浮力或水动力。

## 运动场景

| 场景 | 说明 |
| --- | --- |
| `static` | 静止。 |
| `constant02` | 水平 `0.2 m/s` 匀速。 |
| `constant` | 水平 `0.4 m/s` 匀速。 |
| `sinusoidal` | XY 正弦运动。 |
| `heave_h1/h2/h3` | 分级升沉。 |
| `tilt_*_2deg` | 固定正负 `2°` roll/pitch。 |
| `rollpitch` | 低频动态 roll/pitch。 |
| `combined` | XY、升沉和动态姿态组合。 |
| `rigid_body_motion` | `combined` 加小幅周期 yaw，供 6-DoF shadow 评测。 |

同一组 `MotionProfile` 参数同时供两个环境复用。legacy 中轨迹参考点为 deck center；marine 中通过 launch 参数把 neutral 参考点从 z=2 m 下移到 `vessel_body z≈0`，再由固定 `T_vessel_deck` 恢复 deck center。

动态 `rollpitch`、`combined` 与 `rigid_body_motion` 仍只用于安全高度观察和离线评测。除此之外，marine 第一版对所有 scenario 都额外禁止相对下降、最终下降和 terminal-contact stabilization。

## Vessel → Deck 刚体语义

marine 模型包含：

```text
landing_vessel
└── vessel_body
    └── fixed frame: landing_deck, offset [0, 0, 2] m
```

`rigid_body_kinematics` 使用 world ENU 线速度和 vessel body-frame 角速度计算：

```math
p_D^W=p_V^W+R_W^V r_{VD}^V
```

```math
v_D^W=v_V^W+R_W^V(\omega_V^V\times r_{VD}^V)
```

因此 marine roll/pitch 会同时产生 deck orientation 变化与 deck center lever-arm 位移/速度。完整契约见 [`docs/reference/MARINE_VESSEL_KINEMATICS.md`](../../docs/reference/MARINE_VESSEL_KINEMATICS.md)。

## 视觉目标

landing deck 保留 legacy 的 Marker 局部几何：ID 0/1/2/3 多尺度目标，以及远距非共面 ID 4/5/6。marine model 中各 Marker pose 直接相对 `landing_deck` frame 定义，尺寸、ID、偏移和倾斜角不变；纹理继续复用仓库现有资源。

## 输出

```text
/simulation/deck/ground_truth_raw
/simulation/deck/ground_truth
/deck/gps/fix
/deck/gps/velocity
```

legacy raw Ground Truth 表示 `moving_deck`；marine raw Ground Truth 表示 `landing_vessel/vessel_body`。`moving_deck_controller` 统一将 raw state 转换成 landing deck center 后再发布 `/simulation/deck/ground_truth`，所以 GNSS simulator 和 evaluator 的既有语义不变。

Ground Truth 只能进入 GNSS 传感器仿真和离线 evaluator，禁止生产 landing controller 订阅。

## GNSS 模型

支持理想/含噪位置与速度、固定采样频率、固定延迟、丢包概率、固定随机种子，以及 reset 后确定性复现。传感器输入仍是 deck-center Ground Truth，不是 vessel CG/reference。

## 启动

推荐通过工作区统一脚本：

```bash
# 默认 legacy
./scripts/start_sitl.sh --scenario sinusoidal

# marine safe-altitude validation
./scripts/start_sitl.sh --environment marine --scenario static
./scripts/start_sitl.sh --environment marine --scenario rigid_body_motion --rendezvous-altitude 7.0
```

仅启动仿真包：

```bash
ros2 launch moving_deck_sim moving_deck_sim.launch.py \
  environment:=marine \
  config_file:=$(ros2 pkg prefix --share moving_deck_sim)/config/rigid_body_motion.yaml \
  headless:=true
```

launch 默认 `environment:=legacy`。

## 测试

```bash
colcon test --packages-select moving_deck_sim
colcon test-result --verbose
```

普通 C++ 测试覆盖 MotionProfile、GNSS sensor model 以及 rigid-body zero offset、translation、roll/pitch/yaw、lever-arm velocity、固定旋转和非有限输入；Python 测试覆盖 environment 默认值、marine 安全门与关键 SDF 几何。
