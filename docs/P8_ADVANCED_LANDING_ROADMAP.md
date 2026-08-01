# P8 高级移动甲板降落路线图

## 1. 文档目的

本文档是 P7-lite 开发基线冻结之后的高级传统方法主路线。它只定义阶段依赖、研究门槛、实现边界和验收顺序；各阶段的技术细节必须保存在独立综述、执行计划和验收文档中。

当前统一路线为：

```text
P7-lite 开发基线冻结
→ P8A 升沉甲板最终下降与真实接触
→ P8B 相对运动线性 MPC
→ P8C 固定倾斜及低频 roll/pitch 甲板降落
→ P9 统一批量评测、消融和论文实验
```

阶段依赖必须严格遵守：

```text
P8A
→ P8B research
→ P8B plan
→ P8B implementation
→ P8C research
→ P8C plan
→ P8C implementation
→ P9
```

不得同时实现升沉触地、MPC 和倾斜姿态对齐。当前阶段没有通过真实验收时，后续阶段只能保留为未来计划，不能写成已实现成果。

---

## 2. 已冻结开发基线

### 2.1 P6B

P6B 已完成：

- static 静止甲板最终下降、真实接触、触地确认和 `TOUCHDOWN_HOLD`；
- constant02（0.2 m/s 匀速甲板）最终下降、真实接触、触地确认和 `TOUCHDOWN_HOLD`；
- 四尺度有状态 MarkerSelector；
- 项目内 close-range `near=0.02 m` 相机模型；
- 分段最终下降、终端参考下降到约 `0.05 m`；
- 动态平台相对水平速度触地证据；
- 视觉短时丢帧去抖；
- `FINAL_DESCENT`、`TOUCHDOWN_CANDIDATE_HOLD`、`TOUCHDOWN_HOLD`。

P6B 终端落板参数、Marker 参数和 close-range 相机参数在 P8A 开始时保持冻结。只有出现可重复、可归因的真实失败，才允许进行直接相关的最小修复。

### 2.2 P7-lite

P7 自动化与真实 3+3 冒烟已完成：

```text
static:     3/3 PASS
constant02: 3/3 PASS
total:      6/6 PASS
```

P7-lite 状态定义为：

- 单轮实验自动化完成；
- 顺序批量、seed 展开、resume、失败分类、轻量 Bag、参数快照和聚合完成；
- static/constant02 真实 3+3 冒烟完成；
- 当前结果足够作为后续高级功能的开发基线。

static 20 次 + constant02 20 次不再是进入 P8A 的硬门槛。P7 自动化必须保留，不删除、不废弃；大规模批量实验统一推迟到 P9 论文实验阶段。

---

## 3. 全局硬约束

1. Ground Truth 只能进入离线 evaluator，不得进入控制器、状态估计器、MPC、landing window、touchdown detector 或状态机。
2. 必须保留 GNSS 会合、GNSS—视觉接管、MarkerSelector、视觉状态估计、视觉丢失恢复、landing window、P6B 终端落板门控和稳定 P4.7 控制器。
3. 保持 `NAV_LAND=0`、自动 Disarm=0。
4. 不得通过删除速度证据、大幅放宽倾角阈值、大幅延长视觉超时、绕过 landing window 或无条件开放最终下降来通过实验。
5. 每个阶段必须按以下顺序推进：

```text
现状检查
→ 调研或技术分析
→ 保存文档
→ 明确数学模型和接口
→ 保存独立执行计划
→ 先写测试
→ 最小实现
→ 构建和单元测试
→ 低风险回归
→ 真实 SITL
→ 离线评测
→ 验收文档
→ 更新总路线
```

6. 对需要在论文中详细建模的复杂方法，未完成可验证的综述文档和独立执行计划前，不得编写生产实现。
7. 若真实 SITL 因 QGroundControl、图形界面、人工确认或环境限制无法完成，当前阶段标记 `VALIDATION BLOCKED`，输出准确命令并停止后续阶段。
8. 不自动 commit、tag 或 push，不修改 Git 历史，不删除或覆盖历史阶段文档和验收记录。

---

## 4. P8A：升沉甲板最终下降与真实接触

### 4.1 执行顺序

```text
项目内垂直语义分析
→ docs/P8A_HEAVE_TOUCHDOWN_PLAN.md
→ 测试
→ 最小实现
→ static/constant02 回归
→ H1/H2/H3 分级 SITL
→ docs/P8A_HEAVE_TOUCHDOWN_VALIDATION.md
```

P8A 若只修正已有相对垂直速度语义和 `TOUCHDOWN_HOLD` 随动保持，可先做项目内技术分析。若引入预测控制、扰动观测器、波浪预测器或新接触动力学模型，则必须先完成完整外部综述。

### 4.2 场景分级

- H1：`amplitude_z_m=0.10`，`period_z_s=10.0`；
- H2：`amplitude_z_m=0.20`，`period_z_s=8.0`；
- H3：`amplitude_z_m=0.30`，`period_z_s=8.0`。

第一次只开放 H1 最终下降，H2/H3 逐级开放。rollpitch 和 combined 继续阻断最终下降。

### 4.3 核心问题

- 核对 ENU/NED 垂直正负号；
- 明确 UAV 世界系垂直速度、甲板垂直速度和相对垂直速度；
- 确保低相对速度、非零共同世界速度可以形成候选；
- 确保高相对速度不能因 UAV 世界速度较小而误确认；
- 甲板垂直速度估计无效时不得绕过判据；
- 检查 `TOUCHDOWN_HOLD` 是否冻结世界系 z；
- 必要时实现使用已有 deck state estimate 的甲板相对垂直保持；
- Ground Truth 仅用于 P8A evaluator。

### 4.4 通过门槛

P8A 至少满足：

- H1 3/3 PASS；
- H2 至少单轮 PASS；
- 进入 `TOUCHDOWN_HOLD` 并保持不少于 10 秒；
- 接触时相对垂直速度合格；
- hold 中无持续离板、无明显二次撞击；
- `NAV_LAND=0`、Disarm=0；
- static/constant02 回归通过。

未满足时标记 `P8A VALIDATION BLOCKED`，不得进入 P8B。

---

## 5. P8B：水平相对运动线性 MPC

### 5.1 强制研究门槛

已完成并保存：

```text
docs/research/P8B_RELATIVE_MPC_REVIEW.md
docs/P8B_RELATIVE_MPC_PLAN.md
```

执行顺序：

```text
外部调研
→ 综述
→ 项目问题定义
→ 数学建模
→ 候选方案比较
→ 求解器选型与依赖确认
→ RESEARCH PASS
→ docs/P8B_RELATIVE_MPC_PLAN.md
→ PLAN PASS
→ 测试
→ 最小实现
→ 分级 SITL
→ docs/P8B_RELATIVE_MPC_VALIDATION.md
```

可验证综述、独立计划、固定依赖、生产实现、全量测试和分级真实 SITL 已完成。安全高度 15/15、安全下降 6/6、最终代码真实触地 6/6 PASS，验收见 `docs/P8B_RELATIVE_MPC_VALIDATION.md`。

### 5.2 第一版边界

第一版 MPC 只负责水平相对运动，状态建议为：

```text
x_k = [e_x, e_y, v_rel_x, v_rel_y]^T
u_k = [a_x, a_y]^T
```

必须在综述中重新核对并推导 `A、B、E`、扰动定义、采样周期、离散化、状态来源、PX4 输入映射、目标函数、约束、求解失败回退和模型适用边界。

不得负责最终垂直下降、touchdown detector、landing window、姿态对齐或倾斜甲板法向控制。P4.7 保持默认模式，MPC 显式启用，solver 失败回退 P4.7。

### 5.3 状态标记

- `RESEARCH PASS` / `RESEARCH BLOCKED`；
- `DEPENDENCY BLOCKED`；
- `PLAN PASS`；
- `IMPLEMENTATION PASS`；
- `VALIDATION PASS` / `VALIDATION BLOCKED`。

没有真实 Bag 不得标记 `VALIDATION PASS`。

---

## 6. P8C：固定倾斜与低频 roll/pitch 甲板降落

### 6.1 强制研究门槛

第一步必须新增：

```text
docs/research/P8C_TILTED_DECK_LANDING_REVIEW.md
```

执行顺序：

```text
外部调研
→ 倾斜甲板几何模型
→ 候选策略比较
→ RESEARCH PASS
→ docs/P8C_TILTED_DECK_LANDING_PLAN.md
→ PLAN PASS
→ 固定小倾角 T1
→ 必要时再做法向姿态对齐调研和实现
→ 动态低频 roll/pitch
→ docs/P8C_TILTED_DECK_LANDING_VALIDATION.md
```

### 6.2 第一版策略选择原则

必须先比较：

- A：UAV 保持水平；
- B：仅终端阶段有限法向对齐；
- C：全程跟随甲板法向。

先测试 T1 固定 2° 保持水平策略。只有出现单侧起落架冲击、明显滑移、倾覆风险、触地证据不稳定或无法维持 hold 时，才启动独立的甲板法向姿态对齐综述和计划。

### 6.3 几何与触地语义

必须定义甲板平面、法向、参考点和起落架点到平面距离、法向相对速度、平面内相对位置/速度、姿态误差和接触后相对保持目标。倾斜触地不得继续只用 NED z 高度解释。

T1 未通过，不进入动态 roll/pitch；combined 最后处理。

---

## 7. P9：统一批量评测、消融和论文实验

P9 开始前新增：

```text
docs/P9_UNIFIED_EVALUATION_PLAN.md
```

P9 复用并扩展 P7 自动化，统一处理：

- 方法配置与消融；
- static、constant02、constant、sinusoidal、H1、H2、T1、T3 等场景；
- 每个新组合先 3 次 smoke；
- 根据论文问题设计正式次数，而不是机械地对所有组合运行 50 次；
- `overall`、`by_scenario`、`by_method` 和 failure breakdown；
- 失败轮完整诊断、成功轮轻量 Bag；
- 论文实验矩阵、参数表、曲线和统计结果。

推荐方法集合：

```text
B0：完整 P4.7 规则式基线
B1：关闭额外预测
B2：关闭速度前馈
B3：完整水平相对 MPC
B4：MPC + 升沉处理
B5：若已实现，MPC + 法向姿态对齐
```

P7 的 20+20 配置和自动化继续保留，可在 P9 中作为正式实验子集执行。

---

## 8. 当前状态

截至 2026-08-01：

```text
P6B: VALIDATION PASS
P7-lite: VALIDATION PASS (static/constant02 3+3, 6/6 PASS)
P7 20+20: deferred to P9
P8A: VALIDATION PASS (H1 3/3, H2 3/3)
P8B: RESEARCH PASS / PLAN PASS / IMPLEMENTATION PASS / VALIDATION PASS
P8C: not started
P9: not started
```

P8A 验收见 `docs/P8A_HEAVE_TOUCHDOWN_VALIDATION.md`。P8B 已完成 OSQP `v1.0.0` + OsqpEigen `v0.11.2` 固定依赖、水平相对 MPC、完整 P4.7 fallback、`TERMINAL_PHASE_P47` 安全 handoff、诊断、评测扩展、`271` 项全工作区测试和严格顺序真实 SITL，验收见 `docs/P8B_RELATIVE_MPC_VALIDATION.md`。当前默认下一任务是创建并完成 `docs/research/P8C_TILTED_DECK_LANDING_REVIEW.md`；P8C 未达到 `RESEARCH PASS` 并保存独立计划前，不得直接编写倾斜甲板终端控制生产代码。