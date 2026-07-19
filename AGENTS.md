# AGENTS.md

## 项目目标

本项目基于 PX4 SITL、Gazebo Harmonic 和 ROS 2 Humble，实现无人机在移动船舶甲板上的自主降落传统基线。

最终传统基线必须完成：

```text
ArUco 视觉定位
→ 完整坐标变换
→ 甲板状态估计
→ 短时运动预测
→ 移动目标水平跟踪
→ 规则式着陆窗口判断
→ 相对高度下降
→ 持续跟踪直至触地
→ 安全恢复或中止
→ 批量实验和指标统计
```

详细需求、目录结构、状态机、实验场景和验收指标见：

```text
docs/TRADITIONAL_BASELINE_PLAN.md
```

---

## 当前实现

现有工作区包含：

* `aruco_detector`

  * 输出 `/aruco/pose`
  * 输出 `/aruco/visible`
  * 输出 `/aruco/debug_image`

* `aruco_precision_landing_cpp`

  * 订阅 PX4 状态和 ArUco 位姿。
  * 通过 PX4 Offboard 位置设定点完成起飞、搜索、对中、下降和降落。
  * 当前属于静态 Marker 降落 V0。

当前缺少：

* 移动甲板仿真。
* 完整相机外参和坐标变换。
* 甲板速度估计。
* 运动预测和速度前馈。
* 规则式着陆窗口。
* 移动甲板持续跟踪触地。
* 触地检测、恢复策略和批量评测。

---

## 实现顺序

Codex 必须按以下顺序推进，除非用户明确改变优先级：

1. `P0`：修复仓库结构并冻结静态基线。
2. `P1`：实现水平移动甲板仿真。
3. `P2`：实现完整坐标变换。
4. `P3`：实现甲板状态估计和短时预测。
5. `P4`：实现移动甲板水平跟踪。
6. `P5`：实现规则式着陆窗口和分阶段下降。
7. `P6`：实现触地确认、恢复和安全中止。
8. `P7`：实现批量评测。
9. `P8`：完成传统方法消融实验。

未完成 `P0~P7` 前，不实现强化学习或 MPC。

---

## 核心约束

### 控制器允许使用

* PX4 飞行状态和无人机估计状态。
* 相机图像和相机内参。
* 相机外参。
* ArUco 位姿和可见性。
* 控制器内部估计状态。
* YAML 配置参数。

### 控制器禁止使用

* Gazebo 甲板 Ground Truth。
* 仿真器内部运动相位。
* 预先知道的甲板未来轨迹。
* 评测器计算结果。

Ground Truth 只能用于评测和误差统计。

---

## 坐标系

统一使用：

* `camera_optical`：右、下、前。
* `base_link_frd`：前、右、下。
* `local_ned`：北、东、下。

Marker 位姿转换统一为：

```text
T_local_ned_marker
=
T_local_ned_body
*
T_body_camera
*
T_camera_marker
```

要求：

* `T_camera_marker` 来自 ArUco PnP。
* `T_body_camera` 来自参数或静态 TF。
* `T_local_ned_body` 来自 PX4 里程计。
* 禁止在业务代码中散落手写符号转换。
* 所有坐标转换必须集中在 `coordinate_transform` 模块。
* 坐标转换必须有单元测试。

---

## 控制原则

移动目标跟踪采用：

```text
预测甲板位置目标
+
甲板速度前馈
+
水平位置和相对速度反馈
```

下降采用相对甲板高度，不使用固定世界高度：

```text
relative_height = deck_z_ned - uav_z_ned
position_sp_z = predicted_deck_z_ned - height_ref
```

只有满足以下条件并持续一定时间才允许下降：

* Marker 可见且数据新鲜。
* 水平误差小于阈值。
* 水平相对速度小于阈值。
* 甲板横摇和纵摇小于阈值。
* 状态估计有效。

移动甲板场景下，默认保持 Offboard 水平跟踪直至确认触地，不得在较高高度直接切换 PX4 `NAV_LAND`。

---

## 推荐状态机

```text
INIT
WAIT_FOR_PX4
OFFBOARD_PRE_STREAM
ARM_AND_TAKEOFF
RENDEZVOUS
ACQUIRE_TARGET
TRACK_TARGET
WAIT_LANDING_WINDOW
DESCEND
FINAL_DESCENT
TOUCHDOWN_CONFIRM
DONE
RECOVER
ABORT
```

每次状态转换必须记录：

* 原状态。
* 新状态。
* 转换原因。
* 时间戳。
* 当前误差和对应阈值。

---

## 代码修改规则

* 每次只完成一个明确任务。
* 只修改与当前任务直接相关的文件。
* 不顺手重构相邻模块。
* 不为一次性需求创建复杂抽象。
* 优先复用现有节点、参数和 launch。
* 第一版估计器、预测器和控制器作为独立 C++ 类保留在同一个 ROS 节点内，不急于拆成多个节点。
* 不新增自定义消息包，除非标准消息已无法清晰表达稳定接口。
* 所有参数必须同时更新声明、校验、YAML、文档和测试。
* 禁止硬编码用户目录、绝对路径和 PX4 版本后缀话题。

---

## C++ 注释规则

使用 C++17。

所有新增公开函数必须使用 Doxygen 中文注释，说明：

* 函数实现的功能。
* 输入参数的坐标系、单位和有效范围。
* 返回值或输出内容。
* 失败条件。

复杂坐标变换、滤波更新、运动预测、状态转换和控制公式必须添加中文注释。

不要添加无意义的逐行注释。

示例：

```cpp
/**
 * @brief 将相机坐标系下的 Marker 位姿转换到 PX4 local NED 坐标系。
 *
 * @param camera_marker_pose Marker 在 camera_optical 坐标系下的位姿，位置单位为米。
 * @param ned_body_pose 无人机机体在 local_ned 坐标系下的位姿。
 * @return Marker 在 local_ned 坐标系下的位姿；输入无效时返回 std::nullopt。
 */
std::optional<Pose3d> transform_marker_to_local_ned(...);
```

---

## 测试要求

新增数学模块优先写成不依赖 ROS 的普通 C++ 类，并使用 GTest 测试。

至少覆盖：

* 坐标变换。
* Kalman Filter 预测和更新。
* 异常时间间隔。
* 观测超时。
* 着陆窗口迟滞。
* 正常状态路径。
* 丢失目标恢复路径。
* ABORT 路径。
* 参数非法输入。

每个阶段完成后执行：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash

colcon build --symlink-install \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

colcon test
colcon test-result --verbose
```

---

## 安全要求

* 默认配置只用于 SITL。
* 自动 Arm、Land 和 Disarm 必须由显式参数控制。
* PX4 状态无效时禁止发布有效运动目标。
* 观测过期时禁止继续正常速度下降。
* NaN、Inf 和非法四元数不得进入 PX4 setpoint。
* 位置、速度、加速度和目标变化率必须限幅。
* 测试代码不能误触发实机自动解锁。

---

## 完成定义

传统基线完成必须同时满足：

1. 普通 clone 后可以构建整个工作区。
2. 静态 ArUco 降落功能保持可用。
3. 控制器不使用仿真 Ground Truth。
4. 能够稳定跟踪水平移动甲板。
5. 能够估计甲板速度并短时预测。
6. 能够判断规则式着陆窗口。
7. 能够持续跟踪移动甲板直至触地。
8. 丢失目标时能够减速、恢复或中止。
9. 能够批量运行实验并输出统一指标。
10. 能够完成 B0 到 B4 的传统方法消融对比。

---

## 默认下一任务

没有额外指令时，从 `P0` 开始：

```text
检查并修复 src/aruco_detector 的仓库结构，
将其作为普通 ROS 2 包纳入当前工作区，
确保根目录执行 colcon build 能同时构建
aruco_detector 和 aruco_precision_landing_cpp，
且不改变现有 ROS 话题和运行行为。
```
