# VRX WAM-V Marine 构建与验证说明

本文档给出 marine WAM-V 场景的上游来源、依赖、资产导入、构建、SDF 检查、SITL smoke 和 GUI 验收步骤。理论与坐标契约见 [`VRX_WAMV_INTEGRATION.md`](../reference/VRX_WAMV_INTEGRATION.md)。

## 1. Upstream source

```text
repository: https://github.com/osrf/vrx
branch: jazzy
commit: 7609d1bd90ce7edb29d040a082f949e8b089c864
license: Apache License 2.0
```

审计时 upstream default branch 为 `jazzy`。该 commit 的 VRX README 将 Gazebo Harmonic + ROS 2 Jazzy 作为推荐组合；本项目保持 ROS 2 Humble，因此不构建 `vrx_ros`、competition stack 或 WAM-V propulsion stack。

## 2. Verified local toolchain

审计机器：

```text
Ubuntu 22.04.5 LTS
ROS 2 Humble
Gazebo Sim 8.14.0 (Harmonic)
sdformat 14.9.0
```

验证命令：

```bash
lsb_release -a
printenv ROS_DISTRO
gz sim --versions || gz sim --version
gz sdf --versions || true
ros2 --version || true
```

不得为了 VRX 把系统 ROS/Gazebo 升级到 Jazzy 或其它 ABI。

## 3. Dependencies

### 3.1 Existing project/runtime dependencies

继续使用项目已有：

```text
ROS 2 Humble
ros_gz_sim / ros_gz_bridge
Gazebo Harmonic gz-sim8
sdformat14
gz-math7
gz-msgs10
gz-transport13
PX4 SITL
MicroXRCEAgent
```

Marine asset / WaveVisual configure 还要求系统 `git`（审计机 `git version 2.34.1`）。Ubuntu 22.04 缺失时安装：

```bash
sudo apt update
sudo apt install git
```

验证：`git --version`。Git 只在 configure/build 阶段访问固定 upstream；built runtime 不需要 GitHub。

如果运行 RELATIVE_MPC，仍按现有项目文档设置 `RELATIVE_MPC_PREFIX`；WAM-V 本身不新增 OSQP 依赖。

### 3.2 New source asset dependency

Marine 不安装完整 VRX runtime package，而是在 CMake configure/build 阶段通过 Git partial clone 从固定 commit 导入最小 WAM-V / dynamic-ocean 内容：

```text
WAM-V-Base.dae
WAM-V_Albedo.png
WAM-V_Normal.png
WAM-V_Roughness.png
WAM-V_Metalness.png
waterlow.dae
GerstnerWaves_vs_330.glsl
GerstnerWaves_fs_330.glsl
wave_normals.dds
skybox_lowres.dds
WaveVisual.cc / WaveVisual.hh
Wavefield.cc / Wavefield.hh
```

`WaveVisual.cc` 与 `Wavefield.cc` 从同一 pinned commit 导入 build tree 后编译为本项目安装目录中的 `libWaveVisual.so`；不会安装 VRX competition、propulsion、buoyancy 或 hydrodynamics plugin。动态 ocean plugin 依赖本机 Gazebo Harmonic 的 `gz-sim8`、`gz-common5`、`gz-plugin2`、`gz-rendering8`、`gz-utils2`、`sdformat14`、现有 `gz-math7/gz-msgs10/gz-transport13` 与 `Eigen3`。
来源仓库与 revision 固定为：

```text
https://github.com/osrf/vrx.git
branch: jazzy
commit: 7609d1bd90ce7edb29d040a082f949e8b089c864
```

CMake 使用 `--filter=blob:none --no-checkout --depth 1`，随后只通过 `git show <commit>:<path>` 导出本项目需要的 WAM-V / ocean 资源与 WaveVisual/Wavefield 源文件，并逐项校验预期字节数。构建期需要访问 GitHub；**运行时不需要网络**。构建后的 asset / plugin 安装到：

```text
install/moving_deck_sim/lib/libWaveVisual.so
install/moving_deck_sim/share/moving_deck_sim/models/vrx_wamv_landing/meshes/
install/moving_deck_sim/share/moving_deck_sim/models/vrx_wamv_landing/materials/textures/
install/moving_deck_sim/share/moving_deck_sim/models/vrx_ocean_visual/meshes/
install/moving_deck_sim/share/moving_deck_sim/models/vrx_ocean_visual/materials/programs/
install/moving_deck_sim/share/moving_deck_sim/models/vrx_ocean_visual/materials/textures/
```

模型目录 `THIRD_PARTY.md` 记录每个上游文件、commit、license 和本项目改动。

> 选择构建期固定 commit 导入而不是整个 VRX workspace 的原因：本项目只需要 WAM-V base visual 与 dynamic-ocean 的最小 rendering 子集。该策略避免 ROS Jazzy/competition、propulsion、buoyancy 等无关 plugin 变成 runtime dependency，也避免依赖 Gazebo Fuel 或用户 cache。

## 4. Asset import / vendor strategy

CMake 使用一个最小 asset import helper：

1. 在 build tree 创建 `vrx_upstream` partial-clone staging directory；
2. 校验 staging checkout 的 HEAD 必须等于精确 commit；
3. 在 build tree 创建 `vrx_assets` staging directory；
4. 只从精确 commit 导出指定 blob，并校验预期字节数；
5. clone/blob 导入失败则 configure/build 失败，不回退到 Fuel；
6. 安装到 `moving_deck_sim` share 下固定模型目录；
7. SDF 只使用 `model://vrx_wamv_landing/...` 和 `model://vrx_ocean_visual/...`；
8. 不读取 `~/.gz/fuel`、`~/.ignition/fuel` 或任意用户绝对 asset 路径。

如果未来决定将二进制资产直接 vendor 到 Git，则应保留相同 `THIRD_PARTY.md`、commit 和 Apache-2.0 attribution，并删除构建期下载逻辑，不能同时维护两套来源。

## 5. Model/resource layout

```text
src/moving_deck_sim/models/
├── vrx_wamv_landing/
│   ├── model.config
│   ├── model.sdf
│   ├── THIRD_PARTY.md
│   ├── LICENSE-VRX.txt
│   ├── meshes/                 # build/install 后含官方 WAM-V mesh + PBR maps
│   └── materials/textures/
└── vrx_ocean_visual/
    ├── model.config
    ├── model.sdf               # water mesh + vrx::WaveVisual + CWR 参数
    ├── THIRD_PARTY.md
    ├── meshes/                 # build/install 后含 waterlow.dae
    ├── materials/programs/     # Gerstner vertex / fragment shaders
    └── materials/textures/     # wave normals + skybox cubemap
```

`models/landing_vessel/` 可以保留为历史/debug fixture，但 marine 正式 world 不再 include 它。

## 6. Environment variables

正常 launch 会把 package share 的 `models` 目录加入 `GZ_SIM_RESOURCE_PATH`，并把 package `lib/` 加入 `GZ_SIM_SYSTEM_PLUGIN_PATH`，保证 `libWaveVisual.so` 可由 Gazebo 解析。
手工检查 source/install SDF 时可使用：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash
source install/setup.bash

share="$(ros2 pkg prefix --share moving_deck_sim)"
prefix="$(ros2 pkg prefix moving_deck_sim)"
export GZ_SIM_RESOURCE_PATH="$share/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$prefix/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
```

禁止把 VRX clone、Fuel cache 或某个用户 home 下的 mesh 路径写进 SDF。

## 7. Build

先加载既有依赖：

```bash
cd /home/j/ws_aruco_landing
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash
```

如果项目当前 tracking 配置需要 OSQP，按根 README / AGENTS 记录设置 `RELATIVE_MPC_PREFIX`。然后：

```bash
colcon build --symlink-install \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

预期：

- `moving_deck_sim` 正常构建；
- `libWaveVisual.so` 安装到 package lib；
- WAM-V mesh/PBR maps、water mesh、Gerstner shaders、normal texture 和 skybox texture 出现在 install share；
- 不需要安装完整 `vrx_gz`、`vrx_ros`、`wamv_gazebo` 或 `wamv_description` package。

## 8. Gazebo CLI capability check

先确认本机命令：

```bash
gz sdf --help
gz sim --help
```

然后验证安装后的模型与 world：

```bash
source install/setup.bash
share="$(ros2 pkg prefix --share moving_deck_sim)"
prefix="$(ros2 pkg prefix moving_deck_sim)"
export GZ_SIM_RESOURCE_PATH="$share/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$prefix/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"

gz sdf -k "$share/models/vrx_wamv_landing/model.sdf"
gz sdf -k "$share/models/vrx_ocean_visual/model.sdf"
gz sdf -k "$share/worlds/aruco_marine_vessel.sdf"
```

`gz sdf -k` 只证明 SDF schema/URI 字符串可解析，**不能单独证明 mesh/material 可加载**；还必须执行真实 `gz sim` / SITL smoke 并检查 stderr。

## 9. Asset validation

构建后检查：

```bash
share="$(ros2 pkg prefix --share moving_deck_sim)"
prefix="$(ros2 pkg prefix moving_deck_sim)"
test -f "$prefix/lib/libWaveVisual.so"
find -L "$share/models/vrx_wamv_landing" -maxdepth 4 -type f -print
find -L "$share/models/vrx_ocean_visual" -maxdepth 4 -type f -print
```

必须存在：

```text
libWaveVisual.so
WAM-V-Base.dae
WAM-V_Albedo.png
WAM-V_Normal.png
WAM-V_Roughness.png
WAM-V_Metalness.png
waterlow.dae
GerstnerWaves_vs_330.glsl
GerstnerWaves_fs_330.glsl
wave_normals.dds
skybox_lowres.dds
THIRD_PARTY.md
LICENSE-VRX.txt
```

正式 model/world 禁止出现：

```text
https://fuel.gazebosim.org
fuel://
/home/<user>/...
~/.gz/fuel
~/.ignition/fuel
```

## 10. Full test

```bash
colcon test
colcon test-result --verbose
git diff --check
```

验收目标：

```text
0 errors
0 failures
0 skipped
```

Marine tests 应覆盖：

- upstream asset metadata / installed asset existence；
- WAM-V model、landing deck、landing platform、raw GT plugin；
- VRX-style ocean visual；
- ID0～ID6 pose/size/orientation/texture regression；
- legacy default/world/model/PX4 spawn；
- 全部 marine descent/contact safety flags；
- rigid-body neutral/roll/pitch/yaw/fixed rotation/lever-arm velocity/NaN/Inf/quaternion normalization；
- production source 不订阅 `/simulation/deck/ground_truth`。

## 11. Headless Gazebo model/world smoke

在完整 PX4 SITL 前，允许先用 Gazebo server 快速确认 asset：

```bash
source install/setup.bash
share="$(ros2 pkg prefix --share moving_deck_sim)"
export GZ_SIM_RESOURCE_PATH="$share/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

gz sim -s -r "$share/worlds/aruco_marine_vessel.sdf"
```

检查输出不存在：

```text
missing mesh
missing texture
missing material
URI not found
Unable to find file
Failed to load
```

该 smoke 不替代 PX4 SITL。

## 12. Finite SITL smoke

只运行各一轮：

```bash
./scripts/start_sitl.sh \
  --environment marine \
  --scenario static \
  --headless \
  --auto-confirm-controller \
  --rendezvous-altitude 7.0
```

以及：

```bash
./scripts/start_sitl.sh \
  --environment marine \
  --scenario rigid_body_motion \
  --headless \
  --auto-confirm-controller \
  --rendezvous-altitude 7.0
```

每轮必须检查：

- `vrx_wamv_landing` entity 存在；
- `vrx_ocean_visual` entity 存在；
- `uav_launch_platform` 独立存在；
- PX4 正常；
- `/simulation/deck/ground_truth_raw` finite；
- `/simulation/deck/ground_truth` finite；
- neutral deck `z≈2.0 m`；
- 4-marker noncoplanar PnP 可工作；
- state 到达 safe tracking / `WAIT_LANDING_WINDOW`；
- deck-motion shadow finite；
- 不进入 DESCEND / FINAL_DESCENT / touchdown；
- `NAV_LAND=0`；
- automatic Disarm=0。

## 13. Lever-arm numerical check

在 `rigid_body_motion` smoke 中抓取同时间戳 raw vessel GT 与 deck GT。使用固定：

```text
r_VD = [0, 0, 1.80] m
```

验证：

```text
position_error = p_D - p_V - R_W_V r_VD
velocity_error = v_D - v_V - R_W_V (omega_V × r_VD)
```

报告 position error 和 velocity error 的最大范数；不能只报告“看起来一致”。

## 14. GUI validation

当前 shell 只有在 `DISPLAY` 或 `WAYLAND_DISPLAY` 可用时才能真正执行：

```bash
./scripts/start_sitl.sh \
  --environment marine \
  --scenario static \
  --rendezvous-altitude 7.0
```

人工检查：

1. 海面存在连续可见的波峰/波谷与法线纹理运动，不再是一张静止平板；
2. 暂停 Gazebo 时海面动画同步暂停，继续仿真后恢复；
3. 波幅不会明显吞没 WAM-V 或造成大范围穿模；
4. WAM-V 双体船 mesh 完整，hull / pontoon 比例正确；
5. 2.4×2.4 m landing platform 明确安装在船上，ID0～ID6 位于 platform；
6. UAV launch platform 与船分离；
7. UAV 下视相机能够看到 deck；
8. 没有粉色/黑色 missing-material 模型。

若 `DISPLAY` / `WAYLAND_DISPLAY` 为空，最终报告必须写“GUI 未验证”，并给出上述命令和检查项，不能把 headless smoke 替代 GUI 验收。

## 15. Dynamic wave plugin status

VRX upstream 的动态视觉海面来自 `WaveVisual` + `Wavefield` + Gerstner shaders + coast water mesh。项目仍不直接构建完整 `vrx_gz`：那会额外引入 competition/scoring、propulsion、buoyancy 等当前不需要的依赖。当前实现只从同一 pinned commit 导入 `WaveVisual.cc/.hh`、`Wavefield.cc/.hh` 和海面 mesh/shader/texture，并在 `moving_deck_sim` 内编译单独的 `libWaveVisual.so`。

因此当前 ocean 是：

```text
waterlow.dae
+ libWaveVisual.so
+ CWR 3-component Gerstner vertex displacement
+ animated wave-normal bump map
```

仍然不是：

```text
wave-driven vessel dynamics
buoyancy / hydrodynamics
RAO / wind / current
```

server smoke 已确认 `vrx::WaveVisual` 能被 Gazebo Harmonic 8.14.0 加载，CWR 参数被正确解析，且没有 missing plugin/mesh/shader/texture。由于当前验证 shell 没有 `DISPLAY` / `WAYLAND_DISPLAY`，最终“海面肉眼连续运动”的 GUI 验收仍需在用户图形会话中执行。

## 16. M2 实际验证记录

本阶段实际环境与结果：

```text
Ubuntu: 22.04.5 LTS
ROS: Humble
Gazebo Sim: 8.14.0
sdformat: 14.9.0
VRX branch: jazzy
VRX commit: 7609d1bd90ce7edb29d040a082f949e8b089c864
build: 3 packages finished
full test: 374 tests, 0 errors, 0 failures, 0 skipped
```

第一次使用 `raw.githubusercontent.com` 的 CMake 下载方案在 `WAM-V-Base.dae` 上出现 inactivity timeout，因此最终实现改为 Git HTTP/1.1 partial clone + lazy blob fetch；相同机制现已用于 WAM-V 资产、dynamic-ocean mesh/shader/texture 和 WaveVisual/Wavefield 源码，并完成全仓构建。该失败是构建策略审计证据，不是最终 runtime dependency。

WAM-V / ocean 单模型 `gz sdf -k` 均为 `Valid.`。完整 world 的 `gz sdf -k` 会因为 standalone sdformat CLI 没有设置 `model://` find callback 而报告 include URI unresolved；实际同时设置 package `GZ_SIM_RESOURCE_PATH` 与 `GZ_SIM_SYSTEM_PLUGIN_PATH` 后执行 `gz sim -s -r` smoke，成功加载 `vrx::WaveVisual`、`vrx_wamv_landing` VelocityControl / OdometryPublisher，正确解析 `amplitude=0.06 m`、`period=4.0 s`、`steepness=0.02`，且没有 missing plugin/mesh/shader/texture。

有限 SITL：

- `static`：GNSS rendezvous → noncoplanar 4-marker PnP → `WAIT_LANDING_WINDOW`，随后持续稳定检测；没有 missing asset 或 NaN/Inf 日志；PX4 ULog `NAV_LAND(21)=0`、Disarm command=0、结束时 landed=false。
- `rigid_body_motion`：GNSS rendezvous → noncoplanar 3/4-marker PnP → `WAIT_LANDING_WINDOW`；`/landing/deck_motion_shadow/status=UPDATED:TRUSTED`、terminal stabilization=false、touchdown_confirmed=false；PX4 ULog 同样 `NAV_LAND(21)=0`、Disarm command=0、结束时 landed=false。
- rigid-body 同步采样 `3578` 组：position relation 最大误差 `8.95e-16 m`，velocity lever-arm relation 最大误差 `1.57e-16 m/s`，全部 raw/deck GT finite；最早 deck z≈2.00 m，随后 6-DoF profile 中 deck z 范围约 `1.453–2.274 m`。

当前 shell：

```text
DISPLAY=
WAYLAND_DISPLAY=
```

因此 GUI 人工验收没有执行；必须在有图形会话的终端按第 14 节命令检查 WAM-V、海面是否持续运动、landing platform、Marker、launch dock 和材质。
