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

`rollpitch` 与 `combined` 当前只用于安全高度观察和离线评测，不授权下降或接触。

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
