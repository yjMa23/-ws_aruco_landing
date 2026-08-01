# P8B 水平相对运动线性 MPC 执行计划

## 1. 状态

```text
RESEARCH PASS
PLAN PASS
IMPLEMENTATION PASS
VALIDATION PASS
```

本计划严格引用 `docs/research/P8B_RELATIVE_MPC_REVIEW.md` 的最终方案：4 状态二维相对双积分线性 MPC，OSQP 求解，OsqpEigen 作为 C++ 接口，P4.7 为默认与失败回退。固定依赖、生产实现、全量构建、单元测试和严格顺序的真实 SITL 均已完成，验收记录见 `docs/P8B_RELATIVE_MPC_VALIDATION.md`。

## 2. 依赖门槛

依赖门槛已解除。Ubuntu 22.04 当前 apt 源没有满足本项目固定接口的包，因此采用官方源码固定 tag：

```text
OSQP      v1.0.0  236713ce9a56c182ac3230d52108f952afce1523  Apache-2.0
OsqpEigen v0.11.2 7587e6994dc194cf22511d909bf4cc5d5e0e4eb2  BSD-3-Clause
prefix    ~/.local/p8b-mpc/osqp-1.0.0-osqpeigen-0.11.2
```

构建不使用 sudo，不修改 `/usr/local`。CMake target 为 `OsqpEigen::OsqpEigen` 和 `osqp::osqp`；项目固定 `OsqpEigen 0.11.2 EXACT`，并在运行时核对 OSQP `1.0.0`。官方 MPC 示例已返回 `solved`、25 iterations、约 `0.25 ms`。构建前需导出：

```bash
export P8B_MPC_PREFIX="$HOME/.local/p8b-mpc/osqp-1.0.0-osqpeigen-0.11.2"
export CMAKE_PREFIX_PATH="$P8B_MPC_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$P8B_MPC_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

## 3. 文件修改范围

新增：

- `include/aruco_precision_landing_cpp/relative_mpc_controller.hpp`；
- `src/relative_mpc_controller.cpp`；
- `test/relative_mpc_controller_test.cpp`；
- 必要的 solver adapter 测试；
- P8B evaluator 或对现有 P4/P8A evaluator 的复用扩展；
- P8B 场景与批量配置；
- `docs/P8B_RELATIVE_MPC_VALIDATION.md`，仅在真实门槛通过后创建。

修改：

- `CMakeLists.txt` 与 `package.xml`：显式查找 OSQP/OsqpEigen；
- `px4_aruco_landing_node.hpp/.cpp`：模式选择、状态输入、诊断、fallback；
- YAML：MPC 参数声明、校验和默认关闭；
- `scripts/start_sitl.sh`、单轮/批量运行脚本：显式 `RELATIVE_MPC` 模式；
- README、AGENTS 和总路线：只同步真实状态。

不修改：

- RelativeDescentController；
- FinalDescentController；
- TouchdownDetector；
- TouchdownHoldController；
- landing window；
- MarkerSelector；
- GNSS/视觉接管；
- NAV_LAND/Disarm 语义。

## 4. 类接口

`RelativeMpcController` 为可单测的普通 C++ 类，输入：

- `dt_s`；
- `e_x/e_y` [m, NED]；
- `v_rel_x/v_rel_y` [m/s, NED]；
- 可选甲板加速度 `d_x/d_y` [m/s², NED]；
- 上一周期控制；
- 参数与有效性。

输出：

- 第一控制 `a_x/a_y` [m/s², NED]；
- solver status；
- solve time；
- iteration count；
- objective；
- active constraints；
- predicted states；
- 是否需要 fallback 及原因。

禁止输出 NaN/Inf。异常输入直接返回失败，不调用 PX4。

## 5. 测试优先顺序

先写失败测试，再实现：

1. A/B/E 矩阵与维度；
2. 离散化数值；
3. 零状态输出；
4. 正负位置误差方向；
5. 相对速度阻尼方向；
6. 加速度约束；
7. 控制增量约束；
8. 速度软约束；
9. warm start；
10. infeasible；
11. timeout/迭代上限；
12. NaN/Inf 输入；
13. solver failure fallback；
14. 模式切换；
15. P4.7 默认不变；
16. 非法参数拒绝；
17. 输出连续性与斜率限制。

## 6. 最小实现步骤

1. 仅实现模型矩阵和参数校验；
2. 固定稀疏 QP 结构，运行时只更新状态、扰动和边界；
3. 接入 warm start；
4. 输出首个水平加速度；
5. 节点中增加 `tracking.mode=RELATIVE_MPC`，默认仍为 P4.7；
6. 将 MPC 加速度写入 `TrajectorySetpoint.acceleration[x,y]`，保留位置目标和垂直通道；
7. 任一 solver 异常本周期回退 P4.7；
8. 增加诊断话题或现有调试消息字段；
9. 不在第一版加入 NMPC、姿态、垂直或接触控制。

## 7. 构建与静态验证

每次生产修改后执行全工作区：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash
colcon build --symlink-install \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
colcon test
colcon test-result --verbose
bash -n scripts/start_sitl.sh
python3 -m py_compile scripts/*.py
git diff --check
```

同时执行新脚本 `--help`、dry-run 和参数错误用例。

## 8. SITL 验收顺序

严格依次：

1. static 5 m；
2. constant02 5 m；
3. constant 5 m；
4. sinusoidal 5 m；
5. H1 heave 5 m；
6. constant02 下降到 0.50 m；
7. H1 下降到安全高度；
8. constant02 真实触地；
9. H1 真实触地。

每个场景先单轮，再三个 seed。不得直接从单元测试跳到触地。

比较 P4.7 与 MPC：水平 RMSE、最大误差、相对速度 RMSE、控制平滑度、最大加速度、solve time mean/P95、fallback 和成功率。

## 9. PASS 门槛

- static/constant02/sinusoidal 无明显退化；
- solve time P95 小于控制周期；
- fallback 不产生跳变、NaN 或状态机异常；
- constant02 3/3 触地；
- H1 至少单轮触地；
- NAV_LAND=0；
- Disarm=0；
- 全工作区测试通过；
- 保存 Bag、evaluator 输出和验收文档。

## 10. 回退策略

- 默认模式始终为 P4.7；
- solver 未 solved、超时、NaN、状态过期、预测异常或输出越界时，本周期使用 P4.7；
- 输出继续经过现有限幅和变化率限制；
- 连续 fallback 达阈值后锁定 P4.7 并发布原因；
- 不通过放宽 landing window、touchdown 或安全阈值解决 MPC 问题。

## 11. 实现与验收结果

已完成：

- 独立纯 C++ `RelativeMpcController`；
- 固定稀疏 QP、warm start、不可行与非成功状态处理；
- `tracking.mode=RELATIVE_MPC` 显式启用，默认 P4.7 不变；
- 自由飞行阶段只写入 `TrajectorySetpoint.acceleration[x,y]`，垂直输出与状态机不变；
- 独立并行的完整 P4.7 fallback；
- 从 `FINAL_DESCENT` 开始使用 `TERMINAL_PHASE_P47` 安全 handoff，避免横向加速度破坏终端接触；
- 标准 ROS 2 solver/state/prediction 诊断话题；
- `start_sitl.sh`、`run_single_experiment.py` 模式、依赖检查和 rosbag 话题；
- P4 evaluator 的 solve time mean/P95、iteration、fallback、terminal handoff、active constraints 和控制平滑度指标；
- 全工作区 `271` 项测试通过。

QGroundControl 连接后，已严格按 static → constant02 → constant → sinusoidal → H1 → 安全下降 → 真实触地顺序完成真实 PX4 SITL：

```text
安全高度：15/15 PASS
安全下降： 6/6 PASS
真实触地： 6/6 PASS
```

所有有效 MPC 轮次的 deadline miss、solver failure 和 unexpected fallback 均为 0；constant02 与 H1 最终代码真实触地均为 3/3 PASS，`NAV_LAND=0`、Disarm=0。详细 Bag、指标、根因修复和 PASS 判定见：

```text
docs/P8B_RELATIVE_MPC_VALIDATION.md
```

因此 P8B 标记 `VALIDATION PASS`。下一步允许开始 P8C 调研，但必须先完成 `docs/research/P8C_TILTED_DECK_LANDING_REVIEW.md`，不得直接编写倾斜甲板终端控制生产代码。
