# P8A 升沉甲板最终下降与真实接触验收记录

## 1. 阶段结论

```text
阶段：P8A
状态：VALIDATION PASS
日期：2026-07-30
前置基线：P6B VALIDATION PASS，P7-lite 6/6 PASS
后续阶段：允许进入 P8B research；生产实现仍须等待 RESEARCH PASS 和 PLAN PASS
```

P8A 在不引入 Ground Truth 控制、不启用 `NAV_LAND`、不自动 Disarm 的前提下，完成了升沉甲板相对垂直速度语义、真实接触确认以及 `TOUCHDOWN_HOLD` 相对甲板随动保持。

## 2. 实现范围

本阶段完成：

- 将触地垂直速度证据统一为有效的甲板—无人机相对垂直速度；
- 甲板垂直速度估计无效时阻断误确认；
- 保留 UAV 世界系垂直速度作为诊断，不再把共同升沉误判为未稳定；
- 新增 `TouchdownHoldController`，触地确认后保持接触时的甲板相对高度；
- 估计失效时保持最后安全目标，不继续下降；
- 新增 H1/H2/H3 唯一分级配置入口；
- 扩展 P8A evaluator、批量配置和离线指标；
- rollpitch/combined 仍被最终下降入口阻断。

Ground Truth 仅由 evaluator 用于接触间隙、离板和二次接触统计，没有进入控制器、状态估计器、landing window、touchdown detector 或状态机。

## 3. 构建与测试

执行：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash
colcon build --symlink-install \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
colcon test
colcon test-result --verbose
```

结果：

```text
3 packages finished
245 tests
0 errors
0 failures
0 skipped
```

同时通过：

- `bash -n scripts/start_sitl.sh`；
- Python `py_compile`；
- `evaluate_p8a_heave_touchdown.py --help`；
- 单轮/批量运行脚本 `--help`；
- `git diff --check`。

相对 P7-lite 的 230 项基线新增 15 项测试，覆盖触地相对垂直速度有效性、共同升沉、相对速度超限、确认锁存、候选中断、相对甲板 hold、估计失效和 P8A evaluator/批量配置。

## 4. 真实 SITL 结果

### 4.1 H1 三个 seed

结果目录：

```text
results/p8a_h1_smoke_20260730/
```

结果：3/3 PASS，0 failure。

聚合指标：

| 指标 | 结果 |
|---|---:|
| 甲板最终阶段 z 跨度均值 | 0.2000 m |
| hold 时长均值 | 10.0002 s |
| hold 相对高度跨度均值 | 0.0399 m |
| hold 相对垂直速度 P95 均值 | 0.0225 m/s |
| 水平 RMSE 均值 | 0.0278 m |
| 最大水平误差均值 | 0.0583 m |
| 离板次数 | 0 |
| 二次接触次数 | 0 |
| 候选反复次数 | 0 |
| recovery 次数 | 0 |
| NAV_LAND | 0 |
| Disarm | 0 |

### 4.2 H2 三个 seed

最终有效结果目录：

```text
results/p8a_h2_smoke_retry_20260730/
```

结果：3/3 PASS，0 failure。

聚合指标：

| 指标 | 结果 |
|---|---:|
| 甲板最终阶段 z 跨度均值 | 0.4000 m |
| hold 时长均值 | 10.0002 s |
| hold 相对高度跨度均值 | 0.0417 m |
| hold 相对垂直速度 P95 均值 | 0.0615 m/s |
| 水平 RMSE 均值 | 0.0330 m |
| 最大水平误差均值 | 0.0818 m |
| 离板次数 | 0 |
| 二次接触次数 | 0 |
| 候选反复次数 | 0 |
| recovery 次数 | 0 |
| NAV_LAND | 0 |
| Disarm | 0 |

### 4.3 H3 探索性单轮

结果目录：

```text
results/p8a_validation/p8a_h3_single_seed300/
```

H3 单轮进入 `TOUCHDOWN_HOLD` 并保持约 10 s，无离板、无二次接触、无 recovery、无 `NAV_LAND`、无 Disarm。H3 不属于 P8A 最低 PASS 门槛，因此不把单轮结果声明为 H3 批量冻结。

### 4.4 static / constant02 回归

结果目录：

```text
results/p8a_regression_retry/p8a_static_seed302/
results/p8a_regression_retry/p8a_constant02_seed402/
```

两轮均完成真实接触和 10 s hold，水平跟踪通过，无 recovery、无 `NAV_LAND`、无 Disarm。旧 P6B evaluator 的“世界系 z 目标冻结”断言不再适用于 P8A 相对甲板 hold，因此该旧字段不能作为 P8A 回归判据；P8A evaluator 和真实接触指标用于本阶段结论。

## 5. PASS 判定

P8A 最低门槛逐项满足：

- H1 3/3 PASS；
- H2 3/3 PASS，超过“至少单轮 PASS”要求；
- 所有有效轮次进入 `TOUCHDOWN_HOLD`；
- hold 不少于 10 s；
- 触地相对垂直速度满足 evaluator 阈值；
- hold 中无持续离板；
- 无明显二次撞击；
- 无 recovery；
- `NAV_LAND=0`；
- Disarm=0；
- static/constant02 真实回归无功能退化；
- 全工作区 245 项测试通过。

因此 P8A 标记为：

```text
VALIDATION PASS
```

## 6. 后续约束

允许进入 P8B 调研，但不得直接编码 MPC。下一步必须先完成：

```text
docs/research/P8B_RELATIVE_MPC_REVIEW.md
```

只有达到 `RESEARCH PASS`，完成求解器依赖确认并保存独立执行计划后，才允许编写 P8B 生产实现。

本阶段未 commit、未 tag、未 push。
