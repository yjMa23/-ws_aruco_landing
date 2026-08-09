# moving_deck_sim

Gazebo Harmonic/ROS 2 移动甲板和船舶 GNSS 仿真包。

## 运动场景

| 场景 | 说明 |
| --- | --- |
| `static` | 静止甲板。 |
| `constant02` | 水平 `0.2 m/s` 匀速。 |
| `constant` | 水平 `0.4 m/s` 匀速。 |
| `sinusoidal` | XY 正弦运动。 |
| `heave_h1/h2/h3` | 分级升沉。 |
| `tilt_*_2deg` | 固定正负 `2°` roll/pitch。 |
| `rollpitch` | 低频动态 roll/pitch。 |
| `combined` | XY、升沉和动态姿态组合。 |
| `rigid_body_motion` | `combined` 加小幅周期 yaw，供 6-DoF shadow 评测。 |

`rollpitch`、`combined` 与 `rigid_body_motion` 当前只用于安全高度观察和离线评测，不授权下降或接触。

## 视觉目标

甲板保留 ID 0/1/2/3 多尺度共面 Marker，并增加远距非共面副 Marker：ID 4/5/6
边长均为 `0.75 m`，分别绕甲板 `+Y/+X/+X` 旋转 `+45°/+45°/-45°`，最低边约
高于甲板 `0.02 m`。副 Marker 使用 `120×120` 无损放大纹理，避免斜视时低分辨率
纹理插值破坏码元。该几何只提供相机观测，不包含运动真值或未来轨迹。

## 输出

```text
/simulation/deck/ground_truth
/deck/gps/fix
/deck/gps/velocity
```

Ground Truth 包含甲板位置、姿态、线速度和角速度。它只能进入 GNSS 传感器仿真和离线 evaluator，禁止控制器订阅。

## GNSS 模型

支持：

- 理想或带噪位置/速度。
- 固定采样频率。
- 固定延迟队列。
- 丢包概率。
- 固定随机种子。
- reset 后清空队列并复现采样相位与随机序列。

## 启动

推荐通过工作区统一脚本：

```bash
./scripts/start_sitl.sh --scenario sinusoidal
./scripts/start_sitl.sh --scenario rigid_body_motion
```

仅启动甲板仿真：

```bash
ros2 launch moving_deck_sim moving_deck_sim.launch.py \
  scenario_config:=sinusoidal_xy.yaml
```

## 测试

```bash
colcon test --packages-select moving_deck_sim
colcon test-result --verbose
```

运动模型为普通 C++ 类，测试覆盖场景解析、位置/速度/姿态/角速度、非法参数和确定性 reset。
