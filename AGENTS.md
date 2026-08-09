# AGENTS.md

## 项目目标

本项目基于 PX4 SITL、Gazebo Harmonic 和 ROS 2 Humble，实现无人机在移动船舶甲板上的自主降落传统基线。当前实现与验证结论以 `docs/reference/SYSTEM_OVERVIEW.md` 为唯一事实源，未完成工作只写入 `docs/plans/NEXT_DEVELOPMENT_PLAN.md`。

完整链路为：

```text
船舶 GNSS 粗引导
→ 移动甲板上方会合
→ ArUco 捕获与视觉接管
→ 坐标和时间对齐
→ 甲板状态估计与预测
→ 水平相对跟踪
→ 着陆窗口判断
→ 相对高度下降
→ 最终下降与触地确认
→ 接触后保持或安全恢复
→ 批量实验与统一统计
```

## 当前能力边界

- 支持静止、水平匀速、水平正弦、升沉、固定小倾角和组合运动仿真。
- 支持 GNSS 会合、视觉接管、状态估计、规则式跟踪、可选水平相对 MPC、相对下降、多源触地确认和接触后保持。
- 已验证静止、水平运动、升沉以及固定正 `+2° roll/pitch` 的真实 SITL 接触路径。
- 负固定倾角、动态 `rollpitch`、`combined` 的下降和真实接触仍禁止。
- 默认不下降，不发送 `NAV_LAND`，不自动 Disarm。
- Ground Truth 只能进入仿真传感器或离线评测，禁止进入控制器。

## 实现约束

### 控制器允许使用

- PX4 飞行状态、局部/全局定位参考和无人机估计状态。
- 经过传感器模型处理的船舶 GNSS 位置和速度。
- 相机图像、内参、外参、ArUco 位姿与可见性。
- 控制器内部估计状态和 YAML 参数。

普通 GNSS 只用于远距离会合、视觉捕获和下降前恢复，不参与最终精确下降。

### 控制器禁止使用

- Gazebo 甲板 Ground Truth。
- 仿真器内部运动相位或预知未来轨迹。
- 评测器结果。

Ground Truth 只能用于离线误差统计和传感器仿真。新增订阅、参数或数据流时必须检查这一边界。

## 坐标与时间契约

- `camera_optical`：右、下、前。
- `base_link_frd`：前、右、下。
- `local_ned`：北、东、下。

Marker 位姿统一按下式转换：

```text
T_local_ned_marker
= T_local_ned_body_frd
* T_body_frd_camera_optical
* T_camera_optical_marker
```

- 相机到机体外参必须明确方向。
- PX4 `VehicleOdometry.pose_frame` 必须校验。
- 船舶 WGS84 转 local NED 使用 `VehicleLocalPosition.ref_lat/ref_lon/ref_alt`。
- 图像观测必须使用图像采样时刻插值后的无人机位姿。
- 刚体转换集中在 `coordinate_transform`，地理转换集中在 `geodetic_converter`，禁止在业务代码散落符号翻转。
- 所有转换必须有普通 C++ 单元测试。

## 控制原则

水平跟踪采用：

```text
预测甲板位置
+ 甲板速度前馈
+ 水平位置反馈
+ 相对速度阻尼
```

相对下降采用：

```text
relative_height = deck_z_ned - uav_z_ned
position_sp_z = predicted_deck_z_ned - height_ref
```

只有 Marker 新鲜、水平误差和相对速度满足门槛、甲板倾角安全且估计有效时才允许下降。移动甲板必须保持 Offboard 水平跟踪直至触地确认，不能在高处切换 PX4 自动降落。

推荐状态机：

```text
INIT → WAIT_FOR_PX4 → OFFBOARD_PRE_STREAM → ARM_AND_TAKEOFF
→ WAIT_DECK_GNSS → RENDEZVOUS_GNSS → ACQUIRE_ARUCO
→ VISUAL_HANDOVER → TRACK_TARGET → WAIT_LANDING_WINDOW
→ DESCEND → FINAL_DESCENT → TOUCHDOWN_CANDIDATE_HOLD
→ TOUCHDOWN_HOLD → DONE
```

恢复路径包括 `RECOVER_TO_GNSS`、`RECOVER_CLIMB` 和 `ABORT`。每次转换必须记录原状态、新状态、原因、时间戳、当前误差和阈值。

## 模型与算法前置文档

- 实现任何新模型或算法前，必须先输出精确的理论文档和构建说明，文档未完成不得开始编码。
- 理论文档必须明确问题定义、假设、符号、坐标系、时间基准、单位、输入输出、公式及推导、适用范围、失效条件和安全边界。
- 构建说明必须明确数据流、依赖关系、模块落点、参数来源、实现步骤、测试方法和可量化验收标准，使他人能够据此复现实现与验证过程。

## 编码规则

- 使用 C++17，保持项目现有风格。
- 明确假设；发现矛盾或高影响歧义时先停止并澄清。
- 使用最少代码解决当前问题，不建立一次性抽象、兼容层或推测性配置。
- 先搜索现有实现，再考虑新增帮助类或依赖。
- 只修改与当前任务直接相关的文件，不顺手重构相邻模块。
- 修复 Bug 时先写可复现检查，再在所有调用者共享的根因位置修复。
- 不新增自定义消息包，除非标准消息无法表达稳定接口。
- 参数变更必须同步声明、校验、YAML、文档和测试。
- 禁止硬编码用户目录、绝对路径和 PX4 版本后缀话题。
- 自动 Arm、Land、Disarm 和真实接触能力必须有显式参数与启动安全门。
- NaN、Inf、非法四元数、过期观测或无效 PX4 状态不得进入有效运动目标。

## 依赖与构建可复现

- 每次新增或升级依赖、修改构建方式时，必须在同一变更中更新依赖声明和说明文档。
- 文档必须记录系统与工具链前提、依赖名称、版本、来源、安装命令、配置项、环境变量、完整构建命令、预期产物和验证方法。
- 交付前必须在干净环境或等效隔离环境中严格按文档完成一次构建与验证；不得依赖文档未记录的全局依赖、本地缓存、用户绝对路径或手工操作。无法仅凭文档复现的任务不得视为完成。

## C++ 注释

新增公开类和函数使用中文 Doxygen，说明功能、主要思路、坐标系、单位、有效范围、返回值、失败条件和重要副作用。复杂坐标转换、滤波、预测、状态切换和控制公式添加说明“为什么”的行内注释；直观代码不写重复注释。

## 测试要求

新增数学逻辑优先实现为不依赖 ROS 的普通 C++ 类并使用 GTest。按改动覆盖：

- 坐标与时间转换。
- 估计器预测、更新、异常间隔、离群点和观测超时。
- 着陆窗口迟滞和状态机正常/恢复/中止路径。
- 控制限幅、求解失败回退和非法参数。
- 启动脚本 dry-run、安全门和批量实验配置。

交付前执行：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash

colcon build --symlink-install \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
colcon test
colcon test-result --verbose
```

测试代码不得触发实机自动解锁或绕过 SITL 确认。

## 文档同步规则

- `docs/reference/SYSTEM_OVERVIEW.md` 只描述当前已经实现并验证的事实。
- `docs/plans/NEXT_DEVELOPMENT_PLAN.md` 只描述尚未完成的下一步工作。
- 任何改变功能、配置、公开接口、安全边界、验证结论或后续优先级的变更，必须在同一变更中同步更新上述两个文件。
- 计划完成后，将结论移入系统总览并从下一步计划删除；计划文件不得积累完成历史。
- 纯格式调整或不影响能力、接口、边界和优先级的内部改动无需制造计划更新。
- 新文件、接口、结果字段和文档禁止使用项目生命周期阶段代号命名；使用能力或用途名称。统计术语 `P90/P95/P100`、实验方法编号 `B0～B5` 和文件格式标识不受此规则影响。
- 修改功能后至少检查根 README、包 README、操作指南、系统总览和下一步计划。

## 默认下一任务

默认只执行 `docs/plans/NEXT_DEVELOPMENT_PLAN.md` 中的动态甲板姿态研究与约 `5 m` 安全高度 shadow 验证。未经新计划和分级验收，不开放动态姿态下降、真实接触、`NAV_LAND` 或自动 Disarm。
