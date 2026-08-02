# P8C 固定倾斜与低频 roll/pitch 甲板降落独立执行计划

## 0. 状态

```text
前置研究：P8C RESEARCH PASS
计划状态：PLAN PASS
实现状态：P8C-0 IMPLEMENTATION PASS / P8C-2 WHITELIST IMPLEMENTED / P8C-4 IMPLEMENTATION PASS
真实验收状态：P8C-1 VALIDATION PASS / P8C-2 SAFE DESCENT PASS / P8C-4 VALIDATION PASS / P8C T1 VALIDATION PASS / P8C-3 DESIGN GATE CLOSED
计划日期：2026-08-01
最新同步：2026-08-02
```

本计划以 `docs/research/P8C_TILTED_DECK_LANDING_REVIEW.md` 为几何与分级验证依据。文档前半部分保留原始 P8C-0～P8C-3 计划和失败决策门历史；最终执行已由 `docs/plans/P8C4_TERMINAL_CONTACT_STABILIZATION_PLAN.md` 完成。P8C-4 使用 Offboard position 模式内的终端稳定化、接触顺应和受限预压，不是 PX4 attitude setpoint 姿态对齐。当前下一工程阶段为 P9 统一评测。

---

## 1. 阶段目标

P8C 的目标不是立即让 UAV 跟随甲板法向，而是先把倾斜甲板降落拆成可验证的几何、估计、下降、接触和保持问题：

```text
P8C-0 几何 shadow mode
→ P8C-1 固定 2° 安全高度
→ P8C-2 固定 2° 下降到 0.50 m
→ P8C-3 固定 2° 真实接触
→ P8C-4 终端接触稳定化独立研究、实现与分级验收
→ P8C fixed T1 冻结
→ P9 统一批量评测

负倾角、低频动态 roll/pitch 和 combined touchdown 不属于本次 fixed T1 结论，后续必须另立阶段。
```

第一版策略：

```text
UAV 保持水平
不发布姿态 setpoint
只新增独立 T1 场景白名单
Ground Truth 仅用于 evaluator
```

---

## 2. 当前冻结基线

开始实现前必须重新执行：

```bash
cd /home/j/ws_aruco_landing
git status --short --branch
git log --oneline --decorate -10
git diff --check
colcon test-result --verbose
```

计划编写时基线：

```text
branch: main
HEAD: 3b6f513
P8B validation commit: 2feeea4
271 tests, 0 errors, 0 failures, 0 skipped
P8B safe altitude: 15/15 PASS
P8B safe descent: 6/6 PASS
P8B touchdown: 6/6 PASS
NAV_LAND / Disarm: 0 / 0
```

必须冻结：

- P6B 分段最终下降参数；
- P6B MarkerSelector 参数；
- close-range 相机参数；
- P8A `TouchdownDetector` 与 `TouchdownHoldController` 生产行为；
- P8B MPC 参数和 `TERMINAL_PHASE_P47` handoff；
- landing-window 5°/8° 倾角阈值；
- 默认 `descent.enabled=false`、`final_descent.enabled=false`；
- `NAV_LAND=0`、自动 Disarm=0。

发现未提交用户修改时不得 reset、checkout 覆盖或 stash 后遗忘；先分类并在现有修改上继续。

---

## 3. 非目标

第一轮禁止：

```text
修改 RelativeDescentController 生产算法
修改 FinalDescentController 生产算法
修改 TouchdownDetector 生产行为
修改 TouchdownHoldController 生产行为
修改 PX4 姿态控制接口
新增姿态 setpoint
实现甲板法向对齐
开放动态 rollpitch/combined 最终下降
放宽 landing-window 倾角阈值
调整 MarkerSelector 或相机参数
调整 P8B MPC
启动 P9 大规模实验
```

P8C-3 前允许修改 `scripts/start_sitl.sh` 的唯一目的，是增加独立固定倾角 T1 场景和逐级白名单；不得让历史 `rollpitch`/`combined` 获得最终下降权限。

---

## 4. 安全约束

1. Ground Truth 只进入 evaluator，控制器禁止订阅 `/simulation/deck/ground_truth`。
2. shadow geometry 不得影响 setpoint、landing window、touchdown detector、状态机和 hold。
3. 固定倾角安全高度未通过，不运行下降。
4. 0.50 m 下降未通过，不运行真实接触。
5. 每个真实阶段先单轮，再三个 seed。
6. 任何可重复失败先停止后续场景，保存 Bag、日志和 evaluator 输出。
7. 不通过扩大视觉超时、接触穿透、速度、倾角或水平误差阈值获得 PASS。
8. 不发送 `NAV_LAND`，不自动 Disarm。
9. 不自动 commit、tag、push。

---

## 5. 几何模块接口

### 5.1 新增文件

```text
src/aruco_precision_landing_cpp/include/aruco_precision_landing_cpp/deck_plane_geometry.hpp
src/aruco_precision_landing_cpp/src/deck_plane_geometry.cpp
src/aruco_precision_landing_cpp/test/deck_plane_geometry_test.cpp
```

### 5.2 类职责

`DeckPlaneGeometry` 必须是无 ROS、无参数服务器、无 PX4 发布的纯数学模块，只负责：

```text
法向归一化与方向检查
UAV 参考点到平面的间隙
FRD 接触点到 NED 的转换
四个滑橇端点间隙
h_min / h_max / delta_h / first_contact_index
固定/动态接触点法向相对速度
平面内位置与速度投影
输入有效性和失败原因
```

### 5.3 输入建议

```cpp
struct DeckPlaneGeometryInput
{
  Eigen::Vector3d deck_reference_position_ned_m;
  Eigen::Vector3d upward_normal_ned;
  Eigen::Vector3d uav_reference_position_ned_m;
  Eigen::Quaterniond body_frd_to_ned;
  std::array<Eigen::Vector3d, 4> contact_points_body_frd_m;

  std::optional<Eigen::Vector3d> deck_linear_velocity_ned_mps;
  std::optional<Eigen::Vector3d> uav_linear_velocity_ned_mps;
  std::optional<Eigen::Vector3d> deck_angular_velocity_ned_radps;
  std::optional<Eigen::Vector3d> uav_angular_velocity_ned_radps;
};
```

### 5.4 输出建议

```cpp
struct DeckPlaneGeometryOutput
{
  Eigen::Vector3d upward_normal_ned;
  double body_normal_gap_m;
  std::array<Eigen::Vector3d, 4> contact_positions_ned_m;
  std::array<double, 4> contact_gaps_m;
  double minimum_contact_gap_m;
  double maximum_contact_gap_m;
  double contact_gap_spread_m;
  std::size_t first_contact_index;
  std::optional<double> body_normal_relative_velocity_mps;
  std::array<std::optional<double>, 4> contact_normal_relative_velocity_mps;
  Eigen::Vector3d tangential_position_error_ned_m;
  std::optional<Eigen::Vector3d> tangential_relative_velocity_ned_mps;
};
```

### 5.5 X500 默认接触点

YAML 参数采用明确 FRD 语义：

```yaml
deck_plane_geometry.contact_points_body_frd_m:
  [-0.125, -0.132, 0.227,
    0.125, -0.132, 0.227,
   -0.125,  0.132, 0.227,
    0.125,  0.132, 0.227]
```

参数必须与实际 SDF 版本和校验日期记录在 README/配置注释中。不得在算法中写死本机绝对 PX4 路径。

### 5.6 失败语义

```text
位置/法向/姿态无效：整个输出失败
法向范数过小：失败
法向朝下或向上分量低于安全门：失败
四元数可归一化：归一化并继续
零/极小四元数：失败
速度缺失：位置输出有效，速度字段无效
请求动态接触速度但缺 omega_d：接触速度字段无效并给原因
```

不得用 NaN 伪装有效输出。

---

## 6. 参数设计

第一轮新增参数建议：

```yaml
deck_plane_geometry.enabled: false
deck_plane_geometry.shadow_only: true
deck_plane_geometry.minimum_normal_norm: 1.0e-6
deck_plane_geometry.minimum_upward_component: 0.5
deck_plane_geometry.contact_points_body_frd_m: [...]
deck_plane_geometry.marker_plane_offsets_m: [0.001, 0.002, 0.003, 0.004]
deck_plane_geometry.apply_marker_plane_offset_in_shadow: true
```

原则：

- 默认关闭或 shadow only；
- 不引入控制增益；
- active Marker z 偏置仅用于明确的平面参考修正/对比；
- 任何参数都必须声明、校验、YAML、README 和测试同步；
- 不从场景名推断真实法向。

固定 T1 配置使用仿真侧参数，不给控制器注入真值：

```yaml
initial_rpy_deg: [2.0, 0.0, 0.0]
amplitude_rpy_deg: [0.0, 0.0, 0.0]
```

或 pitch 对应配置。控制器仍只看视觉法向。

---

## 7. 调试话题

shadow mode 建议发布：

```text
/landing/deck_plane_normal                 geometry_msgs/Vector3Stamped
/landing/body_normal_gap                  std_msgs/Float64
/landing/skid_contact_gaps                std_msgs/Float64MultiArray
/landing/skid_min_gap                     std_msgs/Float64
/landing/skid_max_gap                     std_msgs/Float64
/landing/skid_gap_spread                  std_msgs/Float64
/landing/skid_first_contact_index         std_msgs/Int32
/landing/body_normal_relative_velocity    std_msgs/Float64
/landing/skid_normal_relative_velocities  std_msgs/Float64MultiArray
/landing/tangential_position_error        geometry_msgs/Vector3Stamped
/landing/tangential_relative_velocity     geometry_msgs/Vector3Stamped
/landing/deck_plane_geometry_status       std_msgs/String
```

要求：

- 时间戳与控制周期一致；
- frame_id=`local_ned` 或项目冻结的 NED frame；
- 无效字段使用独立 status/valid 标志，不发布误导性零值；
- rosbag 只加入必要轻量话题；
- 所有话题只用于诊断/evaluator。

---

## 8. evaluator 扩展

### 8.1 新增文件

```text
scripts/evaluate_p8c_tilted_deck.py
src/aruco_precision_landing_cpp/test/test_p8c_tilted_deck.py
```

优先复用：

```text
evaluate_p6b_touchdown.py
evaluate_p8a_heave_touchdown.py
```

不得复制整套 rosbag 解析器。必要时先提取纯函数，但不得改变旧 evaluator 输出语义。

### 8.2 Ground Truth 隔离

P8C evaluator 可读取：

```text
/simulation/deck/ground_truth
```

只用于：

```text
真实甲板法向/姿态
真实角速度
真实平面参考点
真实接触几何
法向误差
滑橇间隙与接触顺序
滑移、离板和二次接触
```

控制器和 detector 禁止读取这些数据。

### 8.3 统计字段

至少输出 JSON 和文本：

```text
scenario / seed / git metadata
estimated_normal_error_mean_deg
estimated_normal_error_rmse_deg
estimated_normal_error_p95_deg
estimated_normal_sign_accuracy
normal_jump_by_marker_switch_deg
horizontal_error_rmse_m / max_m
relative_horizontal_speed_rmse_mps
body_normal_gap_min/max/rmse
contact_gap_i_min/max
minimum_contact_gap_m
maximum_contact_gap_m
contact_gap_spread_at_first_contact_m
first_contact_index
second_side_contact_delay_s
touchdown_normal_relative_velocity_mps
tangential_speed_at_touchdown_mps
candidate_duration_s / repeat_count
hold_duration_s
post_contact_tangential_slip_max_m
hold_tangential_speed_p95_mps
post_contact_max_roll_deg / pitch_deg
detach_count / secondary_contact_count
recover_climb_count / gnss_recovery_count
nav_land_count / disarm_count
```

### 8.4 PASS 判据

默认阈值按综述第 17 节实现，命令行可显式覆盖但批量配置必须保存实际值。阈值改变必须有文档与原因，不得在失败后临时放宽并覆盖原结果。

---

## 9. 测试清单

### 9.1 几何单元测试

先写失败测试：

1. 水平甲板退化为 `deck_z-uav_z`；
2. 水平甲板四点间隙等于 `h_body-0.227 m`；
3. `+2° roll` 的 `delta_h≈0.009213 m`；
4. `-2° roll` 首接触侧反转；
5. `+2° pitch` 的 `delta_h≈0.008725 m`；
6. `-2° pitch` 首接触端反转；
7. roll+pitch 组合；
8. 法向量自动归一化；
9. 法向量翻转被拒绝或按明确策略统一；
10. 非法四元数；
11. NaN/Inf 位置、法向、接触点；
12. FRD 到 NED：yaw 0/90/180/-90°；
13. UAV roll/pitch 改变接触点顺序；
14. 甲板参考点水平/垂直偏移；
15. 四点 `h_min/h_max/argmin`；
16. 相等最小值的确定性 tie-break；
17. Marker 平面 1–4 mm z 修正；
18. 水平输出与旧 z 逻辑数值一致。

测试预期：在没有模块时编译失败；只实现位置几何后，速度测试仍失败；不得一次编写全部实现后再补测试。

### 9.2 速度测试

1. 固定水平甲板：法向速度退化为 `deck_vz-uav_vz`；
2. 升沉甲板共同运动：`v_n=0`；
3. 固定倾斜平移甲板；
4. 动态 roll；
5. 动态 pitch；
6. 甲板 `omega_d × r` 项方向；
7. UAV `omega_u × r` 项方向；
8. 两角速度项抵消；
9. 平面内速度投影；
10. 非法速度输入；
11. 缺甲板角速度时动态点速度无效；
12. 位置输出不因速度无效而丢失。

### 9.3 姿态估计/标定测试

不修改估计器参数前，新增离线/消息级测试：

```text
0°、±2° roll、±2° pitch
ID0/ID1/ID2/ID3 法向一致性
Marker 切换跳变
相机外参小角偏差的预期误差方向
法向滤波 reset 和时间回退
```

固定偏差校正若最终需要，必须作为独立、默认关闭且可单测的标定层，不能把场景真值写入 estimator。

### 9.4 脚本测试

```text
四个固定倾角场景参数
start_sitl.sh 默认阻断最终下降
T1 安全高度可启动
P8C-1 未通过时 T1 下降不可启用
rollpitch/combined 始终阻断
参数非法、未知场景、缺配置失败
P8C evaluator --help
缺 topic 明确失败
Ground Truth 只在 evaluator 出现
```

### 9.5 回归测试

```text
static 几何输出与现有 z 逻辑一致
constant02 不退化
H1 不退化
P8B RELATIVE_MPC 自由飞行路径不受影响
TERMINAL_PHASE_P47 不受影响
final descent 默认安全门不受影响
NAV_LAND/Disarm 默认保持关闭
271 项旧测试全部通过
```

---

## 10. P8C-0：shadow mode 执行步骤

### 10.1 Step 0A：测试和接口

修改/新增：

```text
deck_plane_geometry.hpp
deck_plane_geometry_test.cpp
CMakeLists.txt（只注册测试和目标）
```

操作：

1. 写水平退化、±roll、±pitch、非法输入测试；
2. 运行单包测试，确认因未实现而失败；
3. 保存失败输出；
4. 实现最小位置几何；
5. 通过位置测试后再写速度测试。

停止条件：坐标符号无法与 `COORDINATE_FRAMES.md`、SDF 和 P6B `0.227 m` 同时一致。

### 10.2 Step 0B：速度几何

新增动态速度公式，但第一轮运行只要求固定 T1：

1. 实现 `v_n` 和 `v_t`；
2. 实现可选角速度项；
3. 缺 `omega_d` 时接触点动态速度无效；
4. 禁止用 GT 补全缺失量；
5. 完成所有速度测试。

### 10.3 Step 0C：节点 shadow 接入

修改：

```text
px4_aruco_landing_node.hpp/.cpp
px4_aruco_landing.yaml
launch/px4_aruco_landing.launch.py（若参数由 launch 暴露）
CMakeLists.txt
```

只做：

- 构造视觉甲板平面；
- 读取 PX4 UAV 位姿/速度/角速度；
- 计算并发布诊断；
- 记录 active Marker 平面 z 修正；
- 无效时发布明确 status；
- 不把结果写入任何控制输入结构。

必须增加代码级隔离测试或静态检查，证明 `h_min/v_n/e_t` 不进入：

```text
landing_window input
touchdown detector input
relative/final descent input
touchdown hold input
TrajectorySetpoint
```

### 10.4 Step 0D：水平场景 shadow 回归

只运行已有 Bag 离线或低风险 SITL：

```text
static
constant02
H1（安全高度或历史 Bag）
```

要求：

- 水平 `h_body` 与旧 relative height 数值一致；
- 水平 `v_n` 与旧 relative vertical velocity 一致；
- 旧控制输出逐字段无改变；
- 旧状态序列无改变；
- 无新增 recovery。

通过后标记：

```text
P8C-0 SHADOW PASS
```

失败则停在 P8C-0，不创建 T1 下降白名单。

---

## 11. P8C-1：固定 2° 安全高度

### 11.1 新增场景文件

```text
src/moving_deck_sim/config/tilt_roll_pos_2deg.yaml
src/moving_deck_sim/config/tilt_roll_neg_2deg.yaml
src/moving_deck_sim/config/tilt_pitch_pos_2deg.yaml
src/moving_deck_sim/config/tilt_pitch_neg_2deg.yaml
```

固定场景要求：

```text
位置不动
线速度为 0
姿态固定
角速度为 0
无 amplitude_rpy
确定性 reset
```

不要覆盖历史 `roll_pitch.yaml` 或 `combined.yaml`。

### 11.2 启动脚本

`scripts/start_sitl.sh`：

- 增加上述场景名称映射；
- 默认 5 m 安全高度；
- 此阶段拒绝 `--enable-relative-descent` 和 `--enable-final-descent`；
- 原 `rollpitch`/`combined` 阻断测试继续通过。

### 11.3 法向标定

运行 0° 与 ±2° 静态姿态，至少每个姿态覆盖可用 Marker ID。统计：

```text
平均有符号误差 <=0.5°
RMSE <=1.0°
P95 <=1.5°
符号正确率 >=95%
Marker 切换跳变 <=1.0°
```

若失败：

```text
P8C-1 CALIBRATION BLOCKED
```

先核对外参、Marker 共面性和尺度 PnP；禁止放宽 5°/8° landing-window 阈值。

### 11.4 安全高度验收

顺序：

```text
+2° roll 单轮
+2° pitch 单轮
必要时 -2° 单轮
通过后各 3 seeds
```

检查：

- 5 m 目标不下降；
- Marker 可见率/FOV；
- landing window 拒绝原因；
- NED XY 与 `e_t`；
- 理论/测量 `delta_h`；
- 控制输出与状态机不变；
- `NAV_LAND/Disarm=0/0`。

通过后才允许 P8C-2。

---

## 12. P8C-2：固定 2° 安全下降

### 12.1 白名单

增加独立下降门：

```text
只允许 tilt_roll_pos_2deg / tilt_pitch_pos_2deg
只允许下降到 0.50 m
final_descent 仍关闭
```

负倾角在正倾角通过前不开放；历史动态 `rollpitch`/`combined` 继续阻断。

### 12.2 单轮验证

先运行 `+2° roll`：

```text
WAIT_LANDING_WINDOW
→ DESCEND
→ TEST_HEIGHT_HOLD
```

比较：

```text
relative_height_z
h_body
h_min/h_max
height reference
NED XY error/e_t
```

要求：

- 最低真实滑橇间隙保持安全；
- 无 `FINAL_DESCENT`；
- 无接触；
- 无恢复爬升；
- 估计法向满足标定门；
- 水平 RMSE <=0.08 m；
- 最大误差 <=0.15 m。

### 12.3 三 seed

`+2° roll` 3/3 后再运行 `+2° pitch` 单轮和 3 seeds。任一方向失败先停止，不把另一方向成功作为总 PASS。

通过后标记：

```text
P8C-2 SAFE DESCENT PASS
```

---

## 13. P8C-3：固定 2° 真实触地

### 13.1 最终下降白名单

只有 P8C-2 通过后，`start_sitl.sh` 才允许固定 T1 使用：

```text
--enable-relative-descent
--test-height 0.50
--enable-final-descent
```

仍禁止：

```text
rollpitch
combined
任意未验收倾角幅值
```

生产触地检测和 hold 第一轮保持不变；几何只作 shadow/evaluator。

### 13.2 第一轮

运行 `+2° roll` 单轮：

1. 自动记录完整 Bag；
2. 到 `TOUCHDOWN_HOLD` 连续 10 s 后停止；
3. evaluator 统计首接触、另一侧延迟、`h_min/h_max`、法向速度、滑移、角度和 PX4 movement bits；
4. 不通过固定 sleep 声明成功；
5. 失败时不继续 seeds。

### 13.3 问题定位顺序

若失败，依次分类：

```text
法向估计/Marker 切换
平面参考点 z 偏置
下降参考与 h_min 不一致
单侧冲击/rotational_movement
触地候选持续不足
滑移
离板/二次接触
hold 语义
PX4 状态或仿真环境
```

第一轮只允许修复 shadow/evaluator/场景问题。若必须改 detector、hold 或姿态控制，停止并进入独立计划，不在 P8C-3 内临时修改。

### 13.4 批量与回归

固定方向通过后：

```text
+2° roll 3/3
+2° pitch 单轮 → 3/3
static 3/3
constant02 3/3
H1 至少单轮
```

若实际结果显示方向对称性不足，补跑 `-2° roll` 和 `-2° pitch`。

### 13.5 PASS 门槛

按综述冻结：

```text
水平 RMSE <=0.08 m
最大水平误差 <=0.15 m
相对/切向速度 RMSE <=0.10 m/s
h_min at touchdown ∈ [-0.05, +0.03] m
h_max at touchdown <=+0.05 m
delta_h at touchdown <=0.03 m
|v_n| <=0.12 m/s，目标 <=0.05 m/s
候选参数保持 0.50 s
TOUCHDOWN_HOLD >=10 s
切向滑移 <=0.10 m
hold 切向速度 P95 <=0.05 m/s
post-contact |roll|,|pitch| <=10° 且不发散
detach=0
secondary contact=0
recovery=0
NAV_LAND=0
Disarm=0
```

完成后创建：

```text
docs/validation/P8C_TILTED_DECK_LANDING_VALIDATION.md
```

只有真实 Bag 和 evaluator 全部满足才可写 `P8C T1 VALIDATION PASS`。

---

## 14. 姿态对齐决策门

### 14.1 不启动姿态对齐

固定 T1 保持水平 3/3、回归通过且无以下问题时：

```text
单侧冲击明显
持续滑移
触地确认不稳定
持续离板
二次撞击
倾覆风险
hold 失败
```

结论：

```text
策略 A 保留
不实现 B/C
进入 P8C-5 动态前置研究
```

### 14.2 启动独立研究

任一问题在至少两个 seed 可重复且已排除视觉/场景/evaluator 错误时：

```text
停止 P8C 动态场景
新建姿态对齐综述
核对 PX4 姿态/推力接口
比较终端有限对齐与全程法向跟随
设计姿态连续切换、限幅、FOV 和回退
```

不得在本计划中直接插入姿态 setpoint 实现。

---

## 15. P8C-5：低频动态 roll/pitch 前置条件

固定 T1 通过不自动授权动态最终下降。必须另行完成：

1. 视觉甲板角速度或法向变化率估计；
2. 与 Ground Truth 的幅值、相位和 P95 误差评测；
3. 动态接触点 `v_n_i`；
4. 角速度/相位 landing window；
5. 低频预测与视觉延迟分析；
6. 动态 `TOUCHDOWN_HOLD` 候选方案 B/C；
7. 动态场景独立安全高度和 0.50 m 下降；
8. 姿态对齐决策门结果；
9. static/constant02/H1/T1 全回归；
10. `combined` 最后开放。

建议动态分级：

```text
T2: 单轴、幅值 2°、周期 >=12 s
T3: 单轴、幅值 3°、周期 >=10 s
T4: 双轴低频
combined: 最后
```

具体周期和阈值必须在 P8C-5 独立计划中由估计带宽和真实 shadow 数据确定，当前不冻结为已批准参数。

---

## 16. 回归矩阵

| 阶段 | static | constant02 | H1 | T1 roll | T1 pitch | dynamic |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| P8C-0 geometry unit | 数学退化 | 数学退化 | 速度退化 | 合成 | 合成 | 合成速度 |
| P8C-0 shadow | 单轮/历史 Bag | 单轮/历史 Bag | 单轮/历史 Bag | — | — | — |
| P8C-1 5 m | 回归可选 | 回归可选 | — | 3 seeds | 3 seeds | 禁止 |
| P8C-2 0.50 m | 单轮 | 单轮 | — | 3 seeds | 3 seeds | 禁止 |
| P8C-3 contact | 3/3 | 3/3 | 至少单轮 | 3/3 | 3/3 | 禁止 |
| P8C-5 | 3/3 | 3/3 | 至少单轮 | 3/3 | 3/3 | 分级 |

P8B MPC：安全高度/下降阶段可显式测试；终端继续 `TERMINAL_PHASE_P47`，不得因 P8C 改变 handoff。

---

## 17. 构建、静态检查与执行命令

每个实现批次执行：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash

export P8B_MPC_PREFIX="$HOME/.local/p8b-mpc/osqp-1.0.0-osqpeigen-0.11.2"
export CMAKE_PREFIX_PATH="$P8B_MPC_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$P8B_MPC_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

colcon build --symlink-install \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

source install/setup.bash
colcon test
colcon test-result --verbose

bash -n scripts/start_sitl.sh
python3 -m py_compile scripts/*.py
python3 scripts/evaluate_p8c_tilted_deck.py --help
git diff --check
```

Ground Truth 隔离：

```bash
grep -R "/simulation/deck/ground_truth" \
  src/aruco_precision_landing_cpp src/aruco_detector
```

期望没有控制器订阅代码。

脚本级 dry-run/错误路径：

```text
未知 T1 场景拒绝
T1 安全高度阶段拒绝下降开关
P8C-2 拒绝 final descent
rollpitch/combined 始终拒绝 final descent
缺依赖/缺 topic 给出非零返回码
```

---

## 18. 风险和回退

### 18.1 几何风险

- 符号错：停在单元测试，禁止接入节点；
- SDF 版本变化：重新提取滑橇点并更新参数/测试；
- 四端点与盒体不符：升级支持函数，不改触地阈值；
- Marker z 偏置：在平面参考层显式修正，不调相机 near。

### 18.2 视觉风险

- 2° 不可辨识：标记 `CALIBRATION BLOCKED`；
- Marker 切换跳变：按 ID 统计并修正一致性；
- 外参固定偏差：独立标定，不放宽 landing window；
- 视觉短时丢失：保留 P6B 去抖，不延长全局超时。

### 18.3 接触风险

- `rotational_movement` 阻断：保存证据，不直接忽略；
- 单侧冲击/滑移：停止 seeds，进入决策门；
- hold 压入/离板：不在本轮改 hold，标记 blocked；
- 二次接触：失败，不通过增大接触容差解决。

### 18.4 回归风险

static、constant02、H1 或 P8B terminal handoff 退化时：

1. 停止 T1；
2. 比较 shadow 接入前后控制话题；
3. 确认 geometry 未进入 setpoint；
4. 做直接相关最小修复；
5. 重跑全量测试与失败回归；
6. 仍失败则回退生产接入，保留纯模块和失败证据。

---

## 19. 文件级修改清单

### 19.1 P8C-0 新增

```text
src/aruco_precision_landing_cpp/include/aruco_precision_landing_cpp/deck_plane_geometry.hpp
src/aruco_precision_landing_cpp/src/deck_plane_geometry.cpp
src/aruco_precision_landing_cpp/test/deck_plane_geometry_test.cpp
scripts/evaluate_p8c_tilted_deck.py
src/aruco_precision_landing_cpp/test/test_p8c_tilted_deck.py
```

### 19.2 P8C-0 修改

```text
src/aruco_precision_landing_cpp/CMakeLists.txt
src/aruco_precision_landing_cpp/include/aruco_precision_landing_cpp/px4_aruco_landing_node.hpp
src/aruco_precision_landing_cpp/src/px4_aruco_landing_node.cpp
src/aruco_precision_landing_cpp/config/px4_aruco_landing.yaml
src/aruco_precision_landing_cpp/launch/px4_aruco_landing.launch.py（仅需要时）
scripts/run_single_experiment.py（仅诊断 topic/evaluator 参数）
```

节点修改必须只增加 shadow 计算和发布。

### 19.3 P8C-1 新增

```text
src/moving_deck_sim/config/tilt_roll_pos_2deg.yaml
src/moving_deck_sim/config/tilt_roll_neg_2deg.yaml
src/moving_deck_sim/config/tilt_pitch_pos_2deg.yaml
src/moving_deck_sim/config/tilt_pitch_neg_2deg.yaml
```

### 19.4 P8C-1/2/3 修改

```text
scripts/start_sitl.sh
scripts/run_single_experiment.py
scripts/run_batch_experiments.py（若批量入口需要）
config/experiments/p8c_*.yaml（按现有实验配置结构）
```

不得删除或覆盖历史场景与批量配置。

### 19.5 文档

实现时最小同步：

```text
AGENTS.md
README.md
docs/README.md
docs/plans/NEXT_DEVELOPMENT_PLAN.md
docs/plans/P8_ADVANCED_LANDING_ROADMAP.md
docs/plans/TRADITIONAL_BASELINE_PLAN.md
docs/reference/SYSTEM_OVERVIEW.md
```

真实接触通过后新增：

```text
docs/validation/P8C_TILTED_DECK_LANDING_VALIDATION.md
```

状态必须区分 `SHADOW PASS`、`SAFE ALTITUDE PASS`、`SAFE DESCENT PASS` 和 `VALIDATION PASS`。

---

## 20. 分提交建议

本计划不自动提交。用户决定提交时建议：

```text
1. test: define P8C deck plane and skid geometry
2. feat: add pure deck plane geometry module
3. feat: publish P8C geometry shadow diagnostics
4. test: add fixed 2 degree deck scenarios and evaluator
5. feat: enable P8C fixed-tilt safe-altitude validation
6. feat: enable P8C fixed-tilt safe descent
7. feat: gate P8C fixed-tilt touchdown scenario
8. docs: record P8C T1 validation and decision gate
```

每个提交前必须全量测试。不要把姿态对齐混入上述提交。

---

## 21. 执行停止条件

遇到以下任一项立即停止当前后续阶段：

```text
无法确认 PX4/SDF 参考点
四端点符号与水平退化不一致
2° 法向标定不通过
P8C-0 改变任何控制输出
P8C-1 发生下降
P8C-2 发生接触
单轮 T1 出现倾覆、持续滑移、二次撞击或持续离板
static/constant02/H1 回归失败
Ground Truth 进入控制器
NAV_LAND 或 Disarm 非零
```

停止时输出：

```text
失败阶段
可复现命令
Bag/日志/evaluator 路径
预期与实际
是否为环境问题
允许的下一项最小修复
```

不得用“任务较长”作为停止原因。

---

## 22. PLAN PASS 检查表

- [x] 已引用通过的 P8C 综述和冻结模型。
- [x] 已定义阶段目标、非目标和安全约束。
- [x] 已给出纯数学模块输入、输出、有效性和参数。
- [x] 已给出 X500 四接触点坐标。
- [x] 已列出调试话题与 Ground Truth 隔离。
- [x] 已设计 P8C evaluator 和输出指标。
- [x] 已列出几何、速度、姿态、脚本和回归测试。
- [x] 已明确先写失败测试、最小实现和构建命令。
- [x] 已定义 P8C-0 shadow mode。
- [x] 已定义 P8C-1 固定 2° 安全高度。
- [x] 已定义 P8C-2 固定 2° 安全下降。
- [x] 已定义 P8C-3 固定 2° 真实触地。
- [x] 已定义姿态对齐决策门。
- [x] 已定义低频动态 roll/pitch 前置条件。
- [x] 已给出回归矩阵和数值验收门槛。
- [x] 已给出文件级修改清单与分提交建议。
- [x] 已给出失败停止点和回退策略。
- [x] 第一轮没有计划直接实现姿态对齐。
- [x] 没有把未来实现或验证写成已完成。

## 23. P8C-0 实施记录

已完成：

- 新增独立 C++17 `DeckPlaneGeometry`，输入输出全部使用明确的 `local_ned`、`base_link_frd`、米、米每秒和弧度每秒语义；
- 从真实 X500 SDF 冻结四个等效接触点 `[-0.125,-0.132,0.227]`、`[0.125,-0.132,0.227]`、`[-0.125,0.132,0.227]`、`[0.125,0.132,0.227] m`；
- 测试先因未定义实现失败，再完成水平退化、±2° roll/pitch、组合姿态、四元数、法向、速度和角速度单元测试；
- 以 `deck_plane_geometry.shadow_only=true` 接入节点，几何计算位于 `TrajectorySetpoint` 发布之后，不被状态机、landing window、下降、触地或 hold 读取；
- 发布 `/landing/deck_plane/*` 几何、速度、状态和 Marker 法向标定诊断；甲板角速度不可用时不填零冒充，四端点动态速度保持无效；
- 新增 `scripts/evaluate_p8c_tilted_deck.py` 和纯函数测试，支持缺失/非法/时间不同步检测、JSON/文本输出和 evaluator-only Ground Truth 法向对照；
- rosbag 轻量列表已包含全部 P8C-0 shadow 话题；控制器与检测器未新增 Ground Truth 订阅；
- 未开放固定倾角下降、最终下降、触地白名单、姿态 setpoint、NAV_LAND 或自动 Disarm。

结论：

```text
P8C RESEARCH PASS
P8C PLAN PASS
P8C-0 IMPLEMENTATION PASS
P8C-1 VALIDATION PASS
P8C-2 SAFE DESCENT PASS
P8C-3 BLOCKED BY ATTITUDE-ALIGNMENT DECISION GATE
P8C T1 VALIDATION NOT PASSED
```

## 24. P8C-1 实施与验收记录

已完成：

- 新增固定 `±2° roll / ±2° pitch` 四个独立静态场景，保持位置、线速度和角速度为零；
- `scripts/start_sitl.sh` 在启动进程前拒绝四个固定倾角场景的 relative descent 和 final descent；
- production `deck_attitude.filter_gain=0.20` 保持不变，新增独立 shadow 法向滤波，最终 `deck_plane_geometry.normal_filter_gain=0.08`；
- evaluator 使用完整法向夹角计算 RMSE/P95，同时保留有符号主轴均值和符号正确率；
- gain `0.12` 在真实 `tilt_pitch_neg_2deg seed2` 暴露 P95 `1.629°` 失败，保存证据后只调整 shadow gain，没有放宽阈值或修改控制路径；
- 最终固定倾角 `12/12 PASS`、static 对照 `1/1 PASS`，完整法向最差 RMSE/P95 为 `0.702°/1.353°`，最差符号率 `100%`；
- 使用已验收 P6B Bag 重放 ID0/1/2/3，5 次真实切换最大法向跳变 `0.426°`；
- constant02、H1、RELATIVE_MPC 三条安全高度回归全部 PASS；
- 全工作区 `294 tests, 0 failures`，所有真实轮次 `NAV_LAND / Disarm = 0 / 0`；
- 完整验收见 `docs/validation/P8C_TILTED_DECK_LANDING_VALIDATION.md`。

## 25. P8C-2 实施与验收记录

已完成：

- `scripts/start_sitl.sh` 只对白名单 `tilt_roll_pos_2deg / tilt_pitch_pos_2deg` 允许 `--enable-relative-descent --descent-test-height 0.50`；
- 正倾角非 `0.50 m`、任意固定倾角 final descent、负倾角 relative descent、`rollpitch/combined` final descent 均在进程启动前拒绝；
- 默认 YAML 继续关闭下降、final descent 和 auto land，P8C shadow-only 几何及 `normal_filter_gain=0.08` 保持不变；
- evaluator 新增 P8C-2 严格模式，区分估计量、离线 Ground Truth、硬 PASS 门和 observation-only 指标；
- 在真实实验前根据 `0.50 m` 目标、X500 四接触点、`2°` 理论 spread 和 P8C-1 最坏几何误差冻结 `0.090 m` Ground Truth 最低滑橇间隙门；
- TDD 红阶段、启动白名单错误路径、状态路径、接触/穿透、水平误差、法向误差、同步、NaN/Inf、NAV_LAND 和 Disarm 纯函数测试均已覆盖；
- 首轮自动化暴露残留进程检测误判祖先命令行，完成最小修复、回归测试、重新构建和全量测试后从干净 seed1 重跑；
- `tilt_roll_pos_2deg` 3/3、`tilt_pitch_pos_2deg` 3/3、static 1/1、constant02 1/1，共 8/8 PASS；
- 所有轮次到达 `TEST_HEIGHT_HOLD`，最低 hold `44.947 s`；
- 最差水平 RMSE/max `0.020931/0.068704 m`，最低真实滑橇间隙 `0.210051 m`，最差完整法向 RMSE/P95 `0.317644°/0.562188°`，最差符号正确率 `99.9168%`，Marker 最大跳变 `0.427306°`；
- 所有轮次接触、穿透、恢复、final descent、touchdown 状态、time sync failure、NaN/Inf、NAV_LAND 和 Disarm 均为 0；
- 全工作区 `294 tests, 0 errors, 0 failures, 0 skipped`；
- 完整验收见 `docs/validation/P8C_TILTED_DECK_LANDING_VALIDATION.md`，结果见 `results/p8c2_validation_20260802/`。

当前结论：

```text
P8C RESEARCH PASS
P8C PLAN PASS
P8C-0 IMPLEMENTATION PASS
P8C-1 VALIDATION PASS
P8C-2 SAFE DESCENT PASS
P8C-3 BLOCKED BY ATTITUDE-ALIGNMENT DECISION GATE
P8C T1 VALIDATION NOT PASSED
```

P8C-3 已完成正 `+2° roll/pitch` 显式白名单、严格 evaluator、无人值守状态监控和真实 Bag。`+2° roll` seed1 通过；seed2 独立重跑以滑移 `0.106767 m > 0.10 m` 失败，归档轮还出现最大 roll/pitch `60.967996°/55.439155°`、离板和恢复。按失败停止规则，roll seed3 和 pitch 未继续。

回归结果：static 3/3、H1 1/1 PASS；constant02 的新 P8C 门 3/3，但旧 P6B 双评测 2/3，seed3 接触穿透比旧 `-0.05 m` 门多 `0.000144709 m`。全部运行保持 Ground Truth evaluator-only，NAV_LAND / Disarm 为 0 / 0。

上述设计门已由 P8C-4 独立研究、TDD、实现和分级验证关闭。当前最终状态为：

```text
P8C-3 FAILURE EVIDENCE PRESERVED
P8C-4 VALIDATION PASS
P8C T1 VALIDATION PASS
P8C-3 DESIGN GATE CLOSED
```

不得继续通过重跑挑选成功结果；不得放宽冻结门；不得开放负倾角、动态 roll/pitch/combined final descent 或 Ground Truth 控制。fixed T1 之后进入 P9 统一评测，不继续扩大 P8C 安全边界。
