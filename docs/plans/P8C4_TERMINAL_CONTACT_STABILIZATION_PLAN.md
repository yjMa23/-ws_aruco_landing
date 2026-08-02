# P8C-4 TERMINAL CONTACT STABILIZATION PLAN

## 1. 阶段目标

解决固定正 `+2°` roll/pitch 甲板真实接触时的滑移、单侧支撑、姿态发散、离板、二次接触和 hold 不稳定。本计划已按严格顺序全部执行完成，最终状态为：

```text
P8C-4 VALIDATION PASS
P8C T1 VALIDATION PASS
P8C-3 DESIGN GATE CLOSED
```

## 2. 已冻结方案

```text
B1：Offboard position 模式内受限 terminal acceleration bias
+
A2：状态化接触锚点、锚点中心目标、非积分切向速度阻尼
+
T1 场景轴约束与 TOUCHDOWN_HOLD 法向锁存
+
candidate/HOLD 相对高度与向下加速度预压
+
持续姿态/角速度安全保护
```

控制接口不变：

```text
position=true
attitude=false
TrajectorySetpoint 为唯一 companion 控制输出
vehicle_attitude_setpoint 仍只由 PX4 mc_pos_control 生成
```

## 3. 实现分层

### 3.1 纯数学层

新增：

```text
TerminalDeckNormalStabilizer
TerminalContactComplianceController
TerminalAttitudeSafetyMonitor
```

要求：无 ROS、无 Ground Truth、C++17、参数构造校验、显式 reset、所有输出有限。

### 3.2 ROS 接入层

- 参数前缀统一为 `terminal_contact_stabilization.*`；
- 默认 `enabled=false, shadow_only=true`；
- 场景名仅做正 `+2°` 安全白名单；
- visual shadow 法向为唯一法向输入；
- P8B/跟踪 acceleration、terminal bias、combined acceleration 分开保存和诊断；
- active 输出只在 FINAL_DESCENT、CANDIDATE、HOLD 或显式 rehearsal；
- candidate/hold 只在甲板状态有效时建立和更新锚点；
- 姿态保护触发后立即清零 terminal 控制状态并请求恢复。

### 3.3 启动与实验层

新增启动选项：

```text
--enable-terminal-contact-stabilization
--terminal-contact-stabilization-shadow
--terminal-contact-stabilization-rehearsal
```

白名单：

- shadow：固定正 `+2°`；安全高度可不下降，安全下降要求 relative descent + 0.50 m；
- rehearsal：固定正 `+2°`、relative descent、0.50 m、final descent 关闭；
- active touchdown：固定正 `+2°`、relative descent、0.50 m、final descent 开启。

负倾角、rollpitch、combined、非 0.50 m、缺少 relative descent、普通场景 active 全部在启动前拒绝。

## 4. TDD 顺序

1. 先加入纯数学/状态机测试并保存链接失败证据；
2. 实现纯类并使测试通过；
3. 先加入启动门失败测试并保存证据；
4. 实现脚本参数和白名单；
5. 先加入 evaluator 新字段失败测试并保存证据；
6. 扩展 evaluator，保持 P8C-1/2/3 输出兼容；
7. 全量构建和测试；
8. 依次执行 Stage 0～7，不跳级。

## 5. 冻结验收门

P8C-3 原硬门全部保留。P8C-4 自身门在真实实验前冻结：

```text
command tilt max <= 2.5°
command tilt slew P100 <= 4.5°/s（含离散时间容差）
combined horizontal acceleration <= 1.50 m/s²
rehearsal horizontal drift max <= 0.15 m
actual roll/pitch max <= 6° before contact safety warning
attitude tracking error P95 <= 1.5° during stable active window
NaN/Inf count = 0
real touchdown fallback after activation = 0
real touchdown divergence protection = 0
recovery = 0
```

姿态保护触发不计为 PASS，只用于在灾难性姿态前停止。

## 6. 分级实验

### Stage 0：历史 Bag 离线重放

- roll seed1 PASS；
- roll seed2 slip FAIL；
- roll seed2 catastrophic FAIL。

只验证诊断和指标，不声明控制 PASS。

### Stage 1：shadow 安全高度

roll/pitch seeds 1/2/3，约 5 m，不下降。

### Stage 2：shadow 0.50 m 安全下降

roll/pitch seeds 1/2/3，到 TEST_HEIGHT_HOLD，不接触。

### Stage 3：主动无接触 rehearsal

roll seed1→3/3；pitch seed1→3/3。验证真实姿态响应、漂移、限幅和回零。

### Stage 4～6：真实触地

- roll seed1 单轮；
- roll 1/2/3 同代码同参数 3/3；
- pitch seed1 单轮；
- pitch 1/2/3 同代码同参数 3/3。

### Stage 7：旧路径回归

- static 3/3，P8C + legacy P6B 双 evaluator；
- constant02 3/3，P8C + legacy P6B 双 evaluator；
- H1/H2 至少各 1；
- RELATIVE_MPC safe altitude 或 safe descent 至少 1；
- 旧场景 stabilization active 样本必须为 0。

### 最终执行结果

```text
Stage 0 historical replay: PASS
Stage 1 shadow safe altitude: 6/6 PASS
Stage 2 shadow safe descent: 6/6 PASS
Stage 3 active rehearsal: 6/6 PASS
Stage 4-6 active touchdown: roll 3/3 + pitch 3/3 = 6/6 PASS
Stage 7 legacy regressions: 9/9 PASS
Full workspace: 340 tests, 0 failures, 0 skipped
```

最差主轮指标仍通过冻结门：tracking P95 `0.238131°`、slip `0.059209 m`、HOLD 速度 P95 `0.032226 m/s`、水平最大误差 `0.058881 m`、姿态发散增量 `1.908267°`；fallback、离板、二次接触和 recovery 均为 0。

## 7. 失败处理

任一级失败：保存 Bag、stdout/stderr、参数、Git 状态和 evaluator；先写失败测试或独立根因分析；最小修复；全量测试；从当前级第一轮重新开始。不得改 seed、选成功 Bag、放宽门或使用 Ground Truth 控制。

## 8. 完成状态

只有全部 Stage 和回归通过后才能写：

```text
P8C-4 VALIDATION PASS
P8C T1 VALIDATION PASS
P8C-3 DESIGN GATE CLOSED
```

当前计划状态：

```text
P8C-4 PLAN PASS
P8C-4 IMPLEMENTATION PASS
P8C-4 VALIDATION PASS
P8C T1 VALIDATION PASS
P8C-3 DESIGN GATE CLOSED
```

最终证据：

```text
results/p8c4_validation_20260802/p8c4_final_summary.json
results/p8c4_validation_20260802/p8c4_final_summary.txt
```
