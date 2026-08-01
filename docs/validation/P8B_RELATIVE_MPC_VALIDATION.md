# P8B 水平相对运动线性 MPC 验收记录

## 1. 阶段结论

```text
RESEARCH PASS
PLAN PASS
IMPLEMENTATION PASS
VALIDATION PASS
```

P8B 已完成固定求解器依赖、4 状态水平相对运动线性 MPC、PX4 接口、P4.7 安全回退、ROS 2 诊断、离线评测、全工作区测试和严格顺序的真实 PX4 SITL 验收。

第一版 MPC 只负责自由飞行和安全下降阶段的水平相对运动。为保持 P6B/P8A 已验收的终端接触行为，进入 `FINAL_DESCENT` 后显式切换到持续并行更新的 P4.7 水平控制；垂直下降、landing window、TouchdownDetector、TouchdownHoldController 和状态机均未修改。

最终真实结果：

```text
安全高度：15/15 PASS
安全下降： 6/6 PASS
真实触地： 6/6 PASS
全工作区：271 tests, 0 failures
NAV_LAND：0
Disarm：  0
```

## 2. 固定依赖

采用官方源码固定 tag，并安装到独立用户前缀：

```text
OSQP      v1.0.0
commit    236713ce9a56c182ac3230d52108f952afce1523
license   Apache-2.0

OsqpEigen v0.11.2
commit    7587e6994dc194cf22511d909bf4cc5d5e0e4eb2
license   BSD-3-Clause

prefix    ~/.local/p8b-mpc/osqp-1.0.0-osqpeigen-0.11.2
```

构建不使用 sudo，不修改 `/usr/local`，不将第三方源码复制进本仓库。构建和运行前使用：

```bash
export P8B_MPC_PREFIX="$HOME/.local/p8b-mpc/osqp-1.0.0-osqpeigen-0.11.2"
export CMAKE_PREFIX_PATH="$P8B_MPC_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$P8B_MPC_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

## 3. 最终控制边界

### 3.1 自由飞行阶段

以下状态显式启用 `RELATIVE_MPC` 时使用 MPC 水平加速度前馈：

```text
TRACK_TARGET
WAIT_LANDING_WINDOW
DESCEND
TEST_HEIGHT_HOLD
```

水平命令组成：

```text
预测甲板位置目标
+ 纯甲板速度运动学前馈
+ MPC 首个水平加速度
```

MPC 状态和输入：

```text
x = [e_x, e_y, v_rel_x, v_rel_y]^T
u = [a_x, a_y]^T

e = p_deck - p_uav
v_rel = v_deck - v_uav
```

只使用视觉估计、PX4 UAV 状态和控制器内部估计。Gazebo Ground Truth、仿真运动相位和未来轨迹没有进入控制器。

### 3.2 终端下降阶段

从以下状态开始停止发布 MPC 水平加速度，并切换到持续并行计算的完整 P4.7 水平指令：

```text
FINAL_DESCENT
TOUCHDOWN_CANDIDATE_HOLD
TOUCHDOWN_HOLD
```

诊断状态为：

```text
TERMINAL_PHASE_P47
```

这是计划内的终端安全 handoff，不计入 solver failure 或 fallback count。该边界避免 MPC 横向加速度经姿态耦合破坏升沉甲板接触保持，同时保留 P8B 在安全高度、会合、对中和下降阶段的水平预测控制能力。

### 3.3 求解失败回退

节点始终并行维护一个完整 P4.7 控制器。OSQP 非严格 `solved`、输入非法、输出非有限或求解器不可用时，本周期：

```text
不发布 MPC acceleration[x,y]
使用完整 P4.7 位置和速度前馈
重置 MPC warm start
累计 fallback 诊断
```

默认模式仍为：

```text
PREDICTED_POSITION_VELOCITY_FF
```

只有显式传入 `RELATIVE_MPC` 才启用 P8B。

## 4. 构建与测试

最终执行：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash
export P8B_MPC_PREFIX="$HOME/.local/p8b-mpc/osqp-1.0.0-osqpeigen-0.11.2"
export CMAKE_PREFIX_PATH="$P8B_MPC_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$P8B_MPC_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

colcon build --symlink-install \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
colcon test
colcon test-result --verbose
```

结果：

```text
3 packages finished
271 tests
0 errors
0 failures
0 skipped
```

同时通过：

```text
git diff --check
bash -n scripts/start_sitl.sh
python3 -m py_compile scripts/*.py
start_sitl.sh --help
run_single_experiment.py --help
evaluate_p4_bag.py --help
evaluate_p6b_touchdown.py --help
evaluate_p8a_heave_touchdown.py --help
```

测试覆盖模型矩阵、离散化、正负误差方向、相对速度、扰动、加速度/速度/增量约束、warm start、不可行、迭代上限、NaN/Inf、模式选择、完整 P4.7 fallback、终端 handoff 和 evaluator 诊断分类。

## 5. 严格顺序 SITL 验收

QGroundControl 未连接时，首轮 static 启动曾被 PX4 正常健康检查阻止，状态停在 `ARM_AND_TAKEOFF`。连接 QGroundControl 后未绕过任何 PX4 安全检查，按计划严格依次执行：

```text
1. static 5 m
2. constant02 5 m
3. constant 5 m
4. sinusoidal 5 m
5. H1 heave 5 m
6. constant02 下降到 0.50 m
7. H1 下降到 0.70 m
8. constant02 真实触地
9. H1 真实触地
```

每个场景先单轮，再补齐三个 seed；出现退化时停止后续场景，完成根因定位、最小修复、全量测试和失败场景复验后才继续。

## 6. 安全高度结果

所有安全高度轮次：

```text
15/15 PASS
MPC deadline miss = 0
solver failure = 0
fallback = 0
GNSS recovery = 0
```

| 场景 | 水平 RMSE 均值 / 范围 | 最大水平误差范围 | 相对速度 RMSE 范围 | solve P95 范围 |
|---|---:|---:|---:|---:|
| static | `0.0219 m` / `0.0210–0.0233 m` | `0.0534–0.0561 m` | `0.0222–0.0279 m/s` | `0.151–0.204 ms` |
| constant02 | `0.0338 m` / `0.0326–0.0355 m` | `0.0672–0.0817 m` | `0.0339–0.0453 m/s` | `0.234–0.242 ms` |
| constant 0.4 m/s | `0.0465 m` / `0.0457–0.0475 m` | `0.0948–0.1170 m` | `0.0303–0.0311 m/s` | `0.130–0.137 ms` |
| sinusoidal | `0.1848 m` / `0.1836–0.1860 m` | `0.2982–0.3154 m` | `0.2212–0.2326 m/s` | `0.242–0.256 ms` |
| H1 heave | `0.0367 m` / `0.0285–0.0410 m` | `0.0842–0.1178 m` | `0.0665–0.1312 m/s` | `0.225–0.235 ms` |

控制周期为 `50 ms`，所有场景的 solve time P95 均小于 `0.26 ms`，实时裕量充足。

同 seed 的 constant02 P4.7 对照水平 RMSE 为 `0.0355 m`；最终 MPC 为 `0.0332 m`。仓库已记录的 P4.7 正弦最佳水平 RMSE 约为 `0.3439 m`，P8B 三轮正弦均值为 `0.1848 m`，说明显式相对状态与约束在换向运动中取得明显改善。

## 7. 安全下降结果

### 7.1 constant02 到 0.50 m

```text
3/3 PASS
TEST_HEIGHT_HOLD = 3/3
RECOVER_CLIMB = 0/3
NAV_LAND = 0
Disarm = 0
MPC fallback = 0
```

| 指标 | 结果 |
|---|---:|
| Ground Truth 高度对参考 RMSE 均值 | `0.0266 m` |
| 估计高度对参考 RMSE 均值 | `0.0114 m` |
| 最低真实相对高度范围 | `0.444–0.465 m` |
| 最低参考高度 | `0.500 m` |
| 最大 z 目标单步变化范围 | `0.0336–0.0345 m` |
| 水平 RMSE 范围 | `0.0369–0.0395 m` |

### 7.2 H1 到 0.70 m

```text
3/3 PASS
TEST_HEIGHT_HOLD = 3/3
RECOVER_CLIMB = 0/3
NAV_LAND = 0
Disarm = 0
MPC fallback = 0
```

| 指标 | 结果 |
|---|---:|
| Ground Truth 高度对参考 RMSE 均值 | `0.0371 m` |
| 估计高度对参考 RMSE 均值 | `0.0188 m` |
| 最低真实相对高度范围 | `0.602–0.622 m` |
| 最低参考高度 | `0.700 m` |
| 最大 z 目标单步变化范围 | `0.0305–0.0433 m` |
| 水平 RMSE 范围 | `0.0266–0.0319 m` |

## 8. 最终代码真实触地结果

`run_single_experiment.py` 已扩展 `--tracking-mode`，所有最终触地轮次都使用自动状态监控，并在 `TOUCHDOWN_HOLD` 连续满 10 秒后停止。没有使用固定 sleep 作为成功判据。

### 8.1 constant02

结果目录：

```text
/tmp/p8b_constant02_touchdown_auto/
```

最终代码：

```text
3/3 PASS
physical contact = 3/3
TOUCHDOWN_HOLD >= 10 s = 3/3
recovery = 0
NAV_LAND = 0
Disarm = 0
solver failure = 0
fallback = 0
```

聚合指标：

| 指标 | 均值 | 范围 |
|---|---:|---:|
| 落地时间 | `24.684 s` | `24.200–25.050 s` |
| hold 时长 | `10.0002 s` | `10.0002–10.0003 s` |
| hold 相对高度跨度 | `0.0302 m` | `0.0227–0.0419 m` |
| 水平 RMSE | `0.0276 m` | `0.0235–0.0331 m` |
| 最大水平误差 | `0.0536 m` | `0.0483–0.0641 m` |
| solve time mean | `0.174 ms` | `0.170–0.180 ms` |
| solve time P95 | `0.244 ms` | `0.233–0.251 ms` |

最终 Bag：

```text
/tmp/p8b_constant02_touchdown_auto/p8b-constant02-mpc-seed1-terminal-fix/bag
/tmp/p8b_constant02_touchdown_auto/p8b-constant02-mpc-seed2-terminal-fix/bag
/tmp/p8b_constant02_touchdown_auto/p8b-constant02-mpc-seed3-terminal-fix/bag
```

### 8.2 H1 heave

结果目录：

```text
/tmp/p8b_h1_touchdown_auto/
```

最终代码：

```text
3/3 PASS
physical contact = 3/3
detach = 0
secondary contact = 0
TOUCHDOWN_HOLD >= 10 s = 3/3
recovery = 0
NAV_LAND = 0
Disarm = 0
solver failure = 0
fallback = 0
```

聚合指标：

| 指标 | 均值 | 范围 |
|---|---:|---:|
| 落地时间 | `26.217 s` | `25.201–27.250 s` |
| hold 时长 | `10.0002 s` | `10.0001–10.0003 s` |
| hold 相对高度跨度 | `0.0418 m` | `0.0341–0.0459 m` |
| hold 相对垂直速度 P95 | — | `0.0185–0.0306 m/s` |
| 水平 RMSE | `0.0276 m` | `0.0259–0.0296 m` |
| 最大水平误差 | `0.0522 m` | `0.0422–0.0620 m` |
| solve time mean | `0.122 ms` | `0.112–0.131 ms` |
| solve time P95 | `0.214 ms` | `0.201–0.232 ms` |

最终 Bag：

```text
/tmp/p8b_h1_touchdown_auto/p8b-h1-mpc-seed1-terminal-fix/bag
/tmp/p8b_h1_touchdown_auto/p8b-h1-mpc-seed2-terminal-fix/bag
/tmp/p8b_h1_touchdown_auto/p8b-h1-mpc-seed3-terminal-fix/bag
```

## 9. 验收中发现并修复的问题

### 9.1 匀速平台稳态误差

初版 nominal MPC 关闭了甲板速度前馈，constant02 水平 RMSE 达到 `0.2105 m`。原因是 PX4 position loop 对移动参考形成稳态跟随误差。

修复为保留纯甲板速度运动学前馈，MPC 只负责相对位置/速度反馈和水平加速度约束。修复后 constant02 seed 1 水平 RMSE 降为 `0.0332 m`。

### 9.2 重复相对速度反馈

第一次修复错误地同时保留了 P4.7 相对速度反馈和 MPC 相对速度反馈，导致水平 RMSE `0.1090 m`、相对速度 RMSE `0.3676 m/s` 和明显姿态振荡。

最终将 nominal MPC 的速度项限定为纯甲板运动学前馈，并增加独立并行 P4.7 控制器，只有 solver failure 或终端 handoff 才选择完整 P4.7 指令。

### 9.3 升沉触地横向—垂直耦合

MPC 持续进入接触阶段时，H1 出现临界离板或接触压入边界。对照同 seed P4.7 后确认，问题来自 MPC 水平加速度引起的姿态变化，经推力方向耦合影响垂直接触保持。

没有放宽接触间隙、穿透、landing window 或 touchdown 阈值。最终进行直接相关的最小边界修复：从 `FINAL_DESCENT` 开始切换到完整 P4.7 水平指令。修复后 H1 最终代码 3/3 PASS，0 detach、0 secondary contact。

## 10. PASS 判定

逐项满足 `docs/plans/P8B_RELATIVE_MPC_PLAN.md` 门槛：

- static、constant02、constant、sinusoidal、H1 安全高度 3/3；
- constant02 下降到 `0.50 m` 3/3；
- H1 下降到 `0.70 m` 3/3；
- constant02 最终代码真实触地 3/3；
- H1 最终代码真实触地 3/3，超过至少单轮要求；
- solve time P95 最大约 `0.256 ms`，远小于 `50 ms` 控制周期；
- 所有有效轮次 deadline miss、solver failure 和 unexpected fallback 均为 0；
- 未产生 NaN、状态机异常或 GNSS recovery；
- `NAV_LAND=0`；
- Disarm=0；
- Ground Truth 仅用于 evaluator；
- 全工作区 271 项测试通过。

因此 P8B 标记为：

```text
VALIDATION PASS
```

允许下一阶段开始 P8C 调研，但不得直接实现倾斜甲板控制。下一步必须先创建并完成：

```text
docs/research/P8C_TILTED_DECK_LANDING_REVIEW.md
```

在 P8C `RESEARCH PASS` 和独立执行计划完成前，不得修改倾斜甲板终端控制生产代码。
