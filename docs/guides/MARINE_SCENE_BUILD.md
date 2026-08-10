# Marine Scene 第一版构建与验证说明

> 本文记录 Marine M1 primitive fixture 的构建基线。Marine M2 的正式 `--environment marine` 已切换到 VRX WAM-V、`r_VD=[0,0,1.8] m` 和 VRX-style visual-only ocean；当前构建与验证请以 [`VRX_WAMV_BUILD.md`](VRX_WAMV_BUILD.md) 为准。本文中的 `landing_vessel` / `2 m` 只描述保留的 M1 debug fixture，不再是正式 marine runtime。

## 1. 数据流与模块落点

```text
config/*.yaml MotionProfile
  → moving_deck_controller
  → /model/landing_vessel/cmd_vel
  → Gazebo landing_vessel/vessel_body raw odometry
  → rigid_body_kinematics
  → /simulation/deck/ground_truth
  → deck_gnss_simulator / offline evaluator
```

legacy 路径保持 `moving_deck` + `aruco_moving_deck.sdf`；marine 路径使用 `landing_vessel` + `aruco_marine_vessel.sdf`。`scripts/start_sitl.sh --environment legacy|marine` 负责选择 world、模型和 marine 参数覆盖，默认必须为 `legacy`。

## 2. 依赖

不新增第三方依赖。沿用项目现有：

- Ubuntu + ROS 2 Humble；
- Gazebo Harmonic / `gz-sim`；
- `gz-math7`、`gz-msgs10`、`gz-transport13`；
- PX4 SITL 与现有 `x500_mono_cam_down`；
- `ros_gz_sim`、`ros_gz_bridge`。

禁止为本阶段安装 VRX、Fuel 模型、Wavefield、Hydrodynamics、Buoyancy、RAO 或随机海况库。

## 3. 实现步骤

1. 在 `moving_deck_sim` 增加无 ROS 依赖的 `rigid_body_kinematics` 库及 GTest。
2. 新增 primitive-only `models/landing_vessel`：船体参考点约 z=0，固定 landing deck offset 约 +2 m，复制 legacy Marker 的局部几何语义。
3. 新增 `worlds/aruco_marine_vessel.sdf`：保留 world `aruco`、ENU、球面坐标、光照与 250 Hz 物理更新；加入无碰撞 ocean visual、静态 UAV 起飞平台和 landing vessel。
4. `moving_deck_controller` 增加固定 vessel→deck transform 参数；raw odometry 通过纯数学模块转换后再发布 deck Ground Truth。
5. launch 允许显式覆盖 world、model、初始参考位置和 fixed transform；默认参数保持 legacy。
6. `start_sitl.sh` 增加 `--environment legacy|marine`，默认 legacy；marine 使用独立 UAV spawn，并在参数检查阶段拒绝下降/接触相关能力。
7. 增加脚本安全门/默认值回归测试与静态 SDF 检查。

## 4. 构建

```bash
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash

colcon build --symlink-install \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

预期 `moving_deck_sim` 产生：

- `libmoving_deck_motion`；
- `libdeck_gnss_model`；
- `librigid_body_kinematics`；
- `moving_deck_controller`；
- `deck_gnss_simulator`。

## 5. 自动测试

```bash
colcon test
colcon test-result --verbose
git diff --check
```

验收要求：0 errors、0 failures、0 skipped；刚体测试覆盖 neutral height、roll/pitch lever arm 和非有限输入；启动测试覆盖 legacy 默认与 marine safety gate。

## 6. SDF 静态检查

使用本机实际可用命令，先查看：

```bash
gz sdf --help
gz sim --help
```

本机 `gz sdf` 静态解析 `model://` include 时使用 `SDF_PATH`；可复现检查为：

```bash
gz sdf -k src/moving_deck_sim/models/landing_vessel/model.sdf
SDF_PATH="$PWD/src/moving_deck_sim/models" \
  gz sdf -k src/moving_deck_sim/worlds/aruco_marine_vessel.sdf
```

两项都应输出 `Valid.`。运行 Gazebo 时仍由 launch 设置 `GZ_SIM_RESOURCE_PATH`，两者用途不要混淆。确认：

- world name 为 `aruco`；
- `landing_vessel` 资源可由 package share models 路径找到；
- Marker URI 可解析；
- 无重复 entity name 或明显 SDF error。

## 7. 有限 SITL smoke

仅运行各一轮：

```bash
./scripts/start_sitl.sh --environment marine --scenario static --headless --auto-confirm-controller
./scripts/start_sitl.sh --environment marine --scenario rigid_body_motion --headless --auto-confirm-controller --rendezvous-altitude 7.0
```

检查 static：Gazebo/PX4 启动、deck GT finite、neutral deck z≈2 m、ArUco 可捕获、无下降/NAV_LAND/自动 Disarm。

检查 rigid-body：vessel 6DoF、deck 随刚体运动、roll/pitch 产生位置 lever-arm、shadow topic 正常、无 non-finite、无下降/接触。

若当前 shell 无 DISPLAY，只记录 headless smoke 结果；GUI 人工视觉验收由用户在可用桌面会话执行，不为 GUI 改代码。可直接运行：

```bash
./scripts/start_sitl.sh --environment marine --scenario static \
  --rendezvous-altitude 7.0
```

确认 ocean、完整 vessel、5×5 m landing deck、ID 0/4/5/6 等 ArUco 和独立 UAV launch platform 的相对位置与外观合理。
