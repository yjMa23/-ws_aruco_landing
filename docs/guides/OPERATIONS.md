# 安装、构建、启动与实验操作指南

本文档集中说明 `ws_aruco_landing` 的依赖安装、环境加载、构建、SITL 启动、实验记录、状态监控和故障排查。系统架构、阶段计划与验收结果分别见[系统总览](../reference/SYSTEM_OVERVIEW.md)、[计划目录](../plans/)和[验收目录](../validation/)。

> 本项目默认只用于 SITL。启动控制器会发送 Offboard 和 Arm 命令；相对下降与最终下降默认关闭，系统不发送 `NAV_LAND`，也不自动 Disarm。

## 1. 环境安装与构建

### 1.1 前置环境

开始前应已安装 Ubuntu 22.04、ROS 2 Humble 和 Gazebo Harmonic。本节继续安装 PX4 SITL、MicroXRCEAgent、与 PX4 匹配的 `px4_msgs`，以及 P8B MPC 使用的 OSQP/OsqpEigen。

先安装本项目构建所需的通用工具和 ROS 依赖：

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git libeigen3-dev libopencv-dev \
  python3-colcon-common-extensions python3-rosdep \
  ros-humble-ros-gz-bridge
```

本项目当前真实 SITL 验收使用以下固定版本：

| 依赖 | 固定版本或提交 |
| --- | --- |
| PX4-Autopilot | `6f5be87b4cb764d7d2d7956ac4d052e4e4e94fa2` |
| MicroXRCEAgent | `v2.4.3`（`73622810d984349b80bbac0ef55fc0b694d62222`） |
| `px4_msgs` | `f7d9fcb65e2cdf4cf556f658bde55682403dcc8c` |
| OSQP | `v1.0.0`（`236713ce9a56c182ac3230d52108f952afce1523`） |
| OsqpEigen | `v0.11.2`（`7587e6994dc194cf22511d909bf4cc5d5e0e4eb2`） |

默认目录：

```text
PX4：~/PX4-Autopilot
工作空间：~/ws_aruco_landing
px4_msgs underlay：~/ws_sensor_combined
MPC 依赖：~/.local/p8b-mpc/osqp-1.0.0-osqpeigen-0.11.2
```

路径不同时，通过 `PX4_DIR`、`PX4_MSGS_WS`、`P8B_MPC_PREFIX` 或 `ROS_SETUP` 覆盖。所有终端应使用相同的 `ROS_DOMAIN_ID`。

### 1.2 安装 PX4 SITL

克隆并切换到本项目已验收的 PX4 提交，然后安装仅用于 SITL 的工具链：

```bash
git clone --recursive https://github.com/PX4/PX4-Autopilot.git "$HOME/PX4-Autopilot"
git -C "$HOME/PX4-Autopilot" checkout 6f5be87b4cb764d7d2d7956ac4d052e4e4e94fa2
git -C "$HOME/PX4-Autopilot" submodule update --init --recursive

bash "$HOME/PX4-Autopilot/Tools/setup/ubuntu.sh" --no-nuttx
```

安装脚本结束后重启计算机，再验证 PX4 SITL 可以构建：

```bash
make -C "$HOME/PX4-Autopilot" px4_sitl_default
```

项目启动脚本会自动构建并运行实际使用的 `gz_x500_mono_cam_down` 模型，无需在安装阶段单独启动 Gazebo。PX4 的 Ubuntu 环境脚本和 SITL 构建命令见 [PX4 官方 Ubuntu 开发环境](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html)与[构建文档](https://docs.px4.io/main/en/dev_setup/building_px4.html)。

### 1.3 安装 MicroXRCEAgent

本项目验收环境固定使用 `v2.4.3`，并将可执行文件安装到 `/usr/local/bin/MicroXRCEAgent`：

```bash
git clone --branch v2.4.3 --depth 1 \
  https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
  "$HOME/Micro-XRCE-DDS-Agent"

cmake \
  -S "$HOME/Micro-XRCE-DDS-Agent" \
  -B "$HOME/Micro-XRCE-DDS-Agent/build" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "$HOME/Micro-XRCE-DDS-Agent/build" --parallel
sudo cmake --install "$HOME/Micro-XRCE-DDS-Agent/build"
sudo ldconfig
```

确认启动脚本能够找到 Agent：

```bash
command -v MicroXRCEAgent
```

预期输出 `/usr/local/bin/MicroXRCEAgent`。本项目启动时会自动执行 `MicroXRCEAgent udp4 -p 8888`；源码安装方式和 PX4/ROS 2 通信说明见 [PX4 uXRCE-DDS 文档](https://docs.px4.io/main/en/middleware/uxrce_dds.html)。

### 1.4 安装匹配的 `px4_msgs`

`px4_msgs` 的消息定义必须与 PX4 版本匹配。以下提交与上面的 PX4 提交共同通过了本项目验收：

```bash
mkdir -p "$HOME/ws_sensor_combined/src"
git clone https://github.com/PX4/px4_msgs.git \
  "$HOME/ws_sensor_combined/src/px4_msgs"
git -C "$HOME/ws_sensor_combined/src/px4_msgs" \
  checkout f7d9fcb65e2cdf4cf556f658bde55682403dcc8c

source /opt/ros/humble/setup.bash
cd "$HOME/ws_sensor_combined"
colcon build --symlink-install
source install/setup.bash
ros2 pkg prefix px4_msgs
```

最后一条命令应输出 `~/ws_sensor_combined/install/px4_msgs` 对应的绝对路径。若改用其他 PX4 版本，应同时按照 [PX4 uXRCE-DDS 文档](https://docs.px4.io/main/en/middleware/uxrce_dds.html)切换到匹配的 [`px4_msgs`](https://github.com/PX4/px4_msgs) 分支或提交，不要只升级其中一侧。

### 1.5 安装 OSQP 与 OsqpEigen

P8B MPC 固定使用 OSQP `v1.0.0` 和 OsqpEigen `v0.11.2`。两者安装到同一个用户目录，不覆盖系统库：

```bash
export P8B_MPC_PREFIX="$HOME/.local/p8b-mpc/osqp-1.0.0-osqpeigen-0.11.2"
mkdir -p "$HOME/src/p8b-mpc"

git clone --branch v1.0.0 --depth 1 \
  https://github.com/osqp/osqp.git \
  "$HOME/src/p8b-mpc/osqp"
cmake \
  -S "$HOME/src/p8b-mpc/osqp" \
  -B "$HOME/src/p8b-mpc/osqp/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$P8B_MPC_PREFIX" \
  -DOSQP_BUILD_UNITTESTS=OFF
cmake --build "$HOME/src/p8b-mpc/osqp/build" --parallel
cmake --install "$HOME/src/p8b-mpc/osqp/build"

git clone --branch v0.11.2 --depth 1 \
  https://github.com/gbionics/osqp-eigen.git \
  "$HOME/src/p8b-mpc/osqp-eigen"
cmake \
  -S "$HOME/src/p8b-mpc/osqp-eigen" \
  -B "$HOME/src/p8b-mpc/osqp-eigen/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$P8B_MPC_PREFIX" \
  -DCMAKE_PREFIX_PATH="$P8B_MPC_PREFIX" \
  -DBUILD_TESTING=OFF
cmake --build "$HOME/src/p8b-mpc/osqp-eigen/build" --parallel
cmake --install "$HOME/src/p8b-mpc/osqp-eigen/build"
```

确认两个 CMake 包都已安装：

```bash
test -f "$P8B_MPC_PREFIX/lib/cmake/osqp/osqp-config.cmake"
test -f "$P8B_MPC_PREFIX/lib/cmake/OsqpEigen/OsqpEigenConfig.cmake"
```

源码构建方式见 [OSQP 官方文档](https://osqp.org/docs/get_started/sources.html)和 [OsqpEigen `v0.11.2`](https://github.com/gbionics/osqp-eigen/tree/v0.11.2)。项目构建与运行时都必须保留下面的 CMake 和动态库路径：

```bash
export P8B_MPC_PREFIX="$HOME/.local/p8b-mpc/osqp-1.0.0-osqpeigen-0.11.2"
export CMAKE_PREFIX_PATH="$P8B_MPC_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$P8B_MPC_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

### 1.6 克隆和构建本项目

```bash
git clone https://github.com/yjMa23/-ws_aruco_landing.git "$HOME/ws_aruco_landing"
cd "$HOME/ws_aruco_landing"
```

如果当前目录已经是本仓库，跳过以上两条命令。加载 ROS 2、`px4_msgs` 和 MPC 依赖后构建：

```bash
cd ~/ws_aruco_landing
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
```

当前完整工作区基线为：

```text
3 packages finished
271 tests
0 errors
0 failures
0 skipped
```

## 2. 一键启动 SITL

### 2.1 安全默认场景

```bash
./scripts/start_sitl.sh
```

脚本默认启动静止甲板和 `close-range` 相机模型，并依次启动：

```text
MicroXRCEAgent
→ PX4 SITL
→ Gazebo 与移动甲板
→ 相机桥接
→ ArUco 检测器
→ 人工安全确认
→ 降落控制器
```

确认 PX4/QGroundControl 状态正常后按回车，控制器才会切换 Offboard 并 Arm。默认停在安全高度跟踪，不进行相对下降或最终下降。`Ctrl-C` 会统一停止本轮脚本启动的进程。

无界面并记录轻量 Bag：

```bash
./scripts/start_sitl.sh --scenario constant02 --headless --record
```

查看全部可用参数：

```bash
./scripts/start_sitl.sh --help
```

### 2.2 场景与相机

| 场景 | 说明 |
| --- | --- |
| `static` | 静止甲板，默认场景。 |
| `constant02` | East 方向 `0.2 m/s` 匀速。 |
| `constant` | East 方向 `0.4 m/s` 匀速。 |
| `sinusoidal` | XY 正弦运动。 |
| `heave` | 通用升沉场景。 |
| `heave_h1/h2/h3` | P8A 分级升沉场景。 |
| `tilt_roll_pos_2deg / tilt_pitch_pos_2deg` | P8C 正 `2°` 固定倾角；允许安全高度、安全下降，或在固定白名单和显式终端稳定化下执行 P8C-4 touchdown。 |
| `tilt_roll_neg_2deg / tilt_pitch_neg_2deg` | P8C 负 `2°` 固定倾角；仍仅允许 P8C-1 安全高度 shadow。 |
| `rollpitch` | 低频横摇/纵摇，仅用于当前研究和安全高度测试。 |
| `combined` | 水平、升沉和姿态组合运动。 |

相机配置：

```bash
./scripts/start_sitl.sh --camera-model close-range
./scripts/start_sitl.sh --camera-model px4-default
```

`close-range` 为默认配置，near clip 为 `0.02 m`；`px4-default` 的 near clip 为 `0.10 m`，仅用于 A/B 对照。

P8C-1 固定倾角安全高度示例：

```bash
./scripts/start_sitl.sh \
  --scenario tilt_roll_pos_2deg \
  --headless \
  --record
```

P8C-2 正倾角安全下降示例：

```bash
./scripts/start_sitl.sh \
  --scenario tilt_roll_pos_2deg \
  --headless \
  --seed 1 \
  --auto-confirm-controller \
  --camera-model close-range \
  --tracking-mode PREDICTED_POSITION_VELOCITY_FF \
  --enable-relative-descent \
  --descent-test-height 0.50 \
  --bag-output results/p8c2_validation_YYYYMMDD/tilt_roll_pos_2deg_seed1/bag
```

只有 `tilt_roll_pos_2deg / tilt_pitch_pos_2deg` 可在显式 relative descent 且测试高度严格为 `0.50 m` 时进入 P8C-2。P8C-2 本身仍不构成真实接触许可；真实 fixed T1 touchdown 只能通过 P8C-4 的显式 active 终端稳定化白名单启动。正倾角非 `0.50 m`、负倾角 relative descent 或 touchdown、动态 `rollpitch/combined` final descent 均会在启动 PX4/Gazebo/ROS 前返回非零。

普通 `--record` 不记录大体积图像。需要原始图像、`camera_info` 和 `/aruco/debug_image` 时使用：

```bash
./scripts/start_sitl.sh --record-camera-debug
```

### 2.3 跟踪模式

默认 P4.7：

```bash
./scripts/start_sitl.sh \
  --scenario sinusoidal \
  --tracking-mode PREDICTED_POSITION_VELOCITY_FF
```

显式启用 P8B 水平相对 MPC：

```bash
./scripts/start_sitl.sh \
  --scenario constant02 \
  --tracking-mode RELATIVE_MPC \
  --headless \
  --record
```

MPC 求解异常时当前周期自动回退 P4.7；终端下降阶段固定交回 P4.7，不由 MPC 直接控制接触段。

### 2.4 相对下降和最终下降

只下降到 `0.50 m` 安全测试高度：

```bash
./scripts/start_sitl.sh \
  --scenario static \
  --headless \
  --record \
  --enable-relative-descent
```

最终下降必须同时显式授权：

```bash
./scripts/start_sitl.sh \
  --scenario static \
  --headless \
  --record \
  --enable-relative-descent \
  --enable-final-descent
```

当前脚本允许 `static`、纯水平运动场景、`heave_h1/h2/h3` 启用最终下降；固定正 `+2° roll/pitch` 只有在 P8C-4 active 终端稳定化、relative descent、严格 `0.50 m` 和 final descent 同时显式启用时开放。`heave`、负倾角、`rollpitch` 和 `combined` final descent 继续拒绝。P8A 升沉触地示例：

```bash
./scripts/start_sitl.sh \
  --scenario heave_h1 \
  --headless \
  --record \
  --enable-relative-descent \
  --enable-final-descent
```

最终下降默认从 `0.50 m` 开始，接近段速率为 `0.12 m/s`，`0.25 m` 以下切换为 `0.03 m/s`，安全终端参考最低为 `0.05 m`。所有覆盖参数和合法范围以 `./scripts/start_sitl.sh --help` 及脚本校验为准。

## 3. 批量实验

P7-lite 默认支持 `static` 和 `constant02`，使用轻量 Bag、顺序执行、seed 展开、失败分类和 resume。成功判据为进入 `TOUCHDOWN_HOLD` 并连续保持至少 10 秒，不使用固定 sleep 代替状态判断。

单轮：

```bash
python3 scripts/run_single_experiment.py \
  --scenario static \
  --seed 101 \
  --episode-timeout 600 \
  --startup-timeout 120 \
  --touchdown-hold 10 \
  --output-directory results/manual
```

3+3 冒烟和 20+20 配置：

```bash
python3 scripts/run_batch_experiments.py config/experiments/p7_smoke.yaml
python3 scripts/run_batch_experiments.py config/experiments/p7_baseline.yaml
```

Dry-run：

```bash
python3 scripts/run_batch_experiments.py \
  config/experiments/p7_smoke.yaml \
  --dry-run
```

恢复已有批次：

```bash
python3 scripts/run_batch_experiments.py \
  config/experiments/p7_smoke.yaml \
  --resume \
  --batch-id <EXISTING_BATCH_ID>
```

重新聚合：

```bash
python3 scripts/aggregate_results.py results/<BATCH_ID>
```

结果目录：

```text
results/<batch_id>/
├── batch_manifest.json
├── episodes.csv
├── summary.json
├── summary.csv
├── failures.csv
└── <episode_id>/
    ├── manifest.json
    ├── controller_config.yaml
    ├── scenario_config.yaml
    ├── evaluation.json
    ├── evaluation.txt
    ├── run.log
    └── bag/
```

P7-lite 真实 3+3 冒烟已完成 6/6 PASS。P9 统一评测第一版已完成：smoke `20/27`，正式 baseline `40/40`，正式消融 `60/60`；正式运行提交为 `71af1cc`，所有 7 个失败均为 smoke `SAFETY_GATE_FAILURE`。复现实验命令为：

```bash
python3 scripts/run_batch_experiments.py \
  config/experiments/p9_baseline_20x20.yaml \
  --batch-id p9_baseline_20x20_<YYYYMMDD>_<shortsha>

python3 scripts/run_batch_experiments.py \
  config/experiments/p9_ablation.yaml \
  --batch-id p9_ablation_<YYYYMMDD>_<shortsha>
```

正式 baseline 为 static/constant02 `20+20`，实际目录为 `results/p9_baseline_20x20_20260804_71af1cc/`，40/40 PASS。正式消融目录为 `results/p9_ablation_20260804_71af1cc/`，只执行 B0/B1/B3 constant02、B0/B3 sinusoidal 和 B5 roll `+2°`，60/60 PASS；B2 constant02、B4 heave_h1 和 B5 pitch `+2°` 的 `30` 个计划槽位记为 `NOT_APPLICABLE`，没有启动也不进入失败分母。聚合命令为 `python3 scripts/aggregate_results.py <batch_dir>`，每个 batch 输出 `summary.json`、CSV、`P9_RESULTS_SUMMARY.md` 和图表。`results/p9_baseline_20x20_20260803/` 是旧提交上的 interrupted pre-freeze batch，仅 `4/40`，必须保留但排除在最终统计之外。

P9 执行中还保留了以下排除证据：编排污染的 `results/p9_baseline_20x20_20260803_a9d011d/`、时钟修复前的完整 baseline `results/p9_baseline_20x20_20260803_a9d011d_clean1/`，以及暴露 SYSTEM_TIME/ROS_TIME 混用缺陷的 `results/p9_ablation_20260804_a9d011d/`。不得将这些目录 resume 或混入正式统计。

详细设计见 [P7 批量评测计划](../plans/P7_BATCH_EVALUATION_PLAN.md) 与 [P9 统一评测计划](../plans/P9_UNIFIED_EVALUATION_PLAN.md)。

### 3.6 P10 论文结果定稿与证据归档

P10 不运行新的 SITL，只使用三个显式冻结目录的结构化证据：

```bash
python3 scripts/finalize_p9_paper_results.py \
  --smoke results/p9_smoke_20260803 \
  --baseline results/p9_baseline_20x20_20260804_71af1cc \
  --ablation results/p9_ablation_20260804_71af1cc \
  --output results/p9_paper_results_v0.1
```

脚本会硬检查 smoke `20/27`、baseline `40/40`、ablation `60/60`、三个关闭组合、30 个 `NOT_APPLICABLE` 槽位、formal 仿真提交 `71af1cc897136265a999c83dd6034bf156a32a50`、dirty 状态、JSON 完整性以及 `NAV_LAND / Disarm = 0 / 0`。历史 interrupted、污染或时钟修复前批次不得作为输入。

输出目录包含：

```text
paper_summary.json
success_rate_confidence_intervals.csv
continuous_metric_confidence_intervals.csv
method_comparisons.csv
data_provenance.json
DATA_MANIFEST.sha256
P9_PAPER_RESULTS.md
tables/*.csv|md|tex
plots/*.png|pdf|svg
```

成功率使用 Wilson 95% 区间；连续指标均值与独立样本方法差异使用固定 seed `20260804`、10000 次确定性非参数 percentile bootstrap。`NOT_APPLICABLE` 不进入分母，smoke 失败不混入正式成功率。PNG 为 300 dpi，PDF/SVG 为矢量输出。完整说明见 [P10 计划](../plans/P10_PAPER_RESULTS_FINALIZATION_PLAN.md)、[论文结果](../results/P9_PAPER_RESULTS.md) 与 [数据 provenance](../results/P9_DATA_PROVENANCE.md)。

`results/` 不提交 Git；原始 Bag 外部归档与远端同步需用户执行或授权。只提交统计代码、测试、仓库内摘要及小型最终 PDF/SVG 图表。

## 4. 手动多终端启动

一键脚本应作为普通入口；以下流程仅用于逐项调试。控制器必须最后启动。

### 4.1 终端 1：MicroXRCEAgent

```bash
source /opt/ros/humble/setup.bash
MicroXRCEAgent udp4 -p 8888
```

### 4.2 终端 2：PX4 SITL

移动甲板 world 由 `moving_deck_sim` 启动，因此 PX4 使用 standalone 模式：

```bash
cd ~/PX4-Autopilot

PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD=aruco \
PX4_GZ_MODEL_POSE=-4,0,0.2 \
make px4_sitl gz_x500_mono_cam_down
```

同时打开 QGroundControl，确认飞行器连接、Local Position 有效且没有持续的严重 EKF/GPS 错误。

### 4.3 终端 3：移动甲板与船舶 GNSS

```bash
cd ~/ws_aruco_landing
source /opt/ros/humble/setup.bash
source install/setup.bash
source ~/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh

ros2 launch moving_deck_sim moving_deck_sim.launch.py
```

静止或正弦配置：

```bash
SHARE=$(ros2 pkg prefix --share moving_deck_sim)

ros2 launch moving_deck_sim moving_deck_sim.launch.py \
  config_file:=$SHARE/config/static.yaml \
  gnss_config_file:=$SHARE/config/gnss_ideal.yaml
```

```bash
SHARE=$(ros2 pkg prefix --share moving_deck_sim)

ros2 launch moving_deck_sim moving_deck_sim.launch.py \
  config_file:=$SHARE/config/sinusoidal_xy.yaml \
  gnss_config_file:=$SHARE/config/gnss_ideal.yaml
```

### 4.4 终端 4：相机桥接

```bash
source /opt/ros/humble/setup.bash
source ~/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh

ros2 run ros_gz_bridge parameter_bridge \
  '/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
```

检查图像：

```bash
ros2 topic hz \
  /world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image
```

正常频率约为 `30 Hz`。

### 4.5 终端 5：ArUco 检测器

```bash
cd ~/ws_aruco_landing
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch aruco_detector aruco_detector.launch.py
```

```bash
ros2 topic echo /aruco/visible
ros2 topic echo /aruco/pose
```

`/aruco/pose` 数值语义固定为 OpenCV `camera_optical`：x 向右、y 向下、z 向镜头前方。

### 4.6 终端 6：控制器

```bash
cd ~/ws_aruco_landing
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash
source install/setup.bash

export P8B_MPC_PREFIX="$HOME/.local/p8b-mpc/osqp-1.0.0-osqpeigen-0.11.2"
export LD_LIBRARY_PATH="$P8B_MPC_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

ros2 launch aruco_precision_landing_cpp px4_aruco_landing.launch.py
```

默认 PX4 v1.18 话题 remap：

```text
/fmu/out/vehicle_status → /fmu/out/vehicle_status_v4
/fmu/out/vehicle_local_position → /fmu/out/vehicle_local_position_v1
```

PX4 消息版本不同时可覆盖：

```bash
ros2 launch aruco_precision_landing_cpp px4_aruco_landing.launch.py \
  vehicle_status_topic:=/fmu/out/vehicle_status \
  vehicle_local_position_topic:=/fmu/out/vehicle_local_position
```

## 5. 状态机与跟踪模式

默认安全高度链路：

```text
INIT
→ WAIT_FOR_PX4
→ OFFBOARD_PRE_STREAM
→ ARM_AND_TAKEOFF
→ WAIT_DECK_GNSS
→ RENDEZVOUS_GNSS
→ ACQUIRE_ARUCO
→ VISUAL_HANDOVER
→ TRACK_TARGET
→ WAIT_LANDING_WINDOW
```

显式启用下降后：

```text
WAIT_LANDING_WINDOW
→ DESCEND
→ TEST_HEIGHT_HOLD
→ FINAL_DESCENT
→ TOUCHDOWN_CANDIDATE_HOLD
→ TOUCHDOWN_HOLD
```

视觉、窗口或估计严重失效时进入 `RECOVER_CLIMB` 或 `RECOVER_TO_GNSS`。恢复后锁止再次自动下降，必须重新完成视觉接管或重启任务才能解除。

支持的水平跟踪模式：

| 模式 | 位置目标 | 前馈/控制 | 说明 |
| --- | --- | --- | --- |
| `RAW_VISUAL_POSITION` | 原始 Marker | 无 | P2D 消融基线。 |
| `ESTIMATED_POSITION` | Kalman 估计位置 | 无 | 估计消融。 |
| `ESTIMATED_POSITION_VELOCITY_FF` | 估计位置 | 甲板速度 | 速度前馈消融。 |
| `PREDICTED_POSITION_VELOCITY_FF` | 短时预测位置 | 甲板速度与自适应相对速度阻尼 | 默认 P4.7。 |
| `RELATIVE_MPC` | 预测位置 | MPC 水平加速度，失败回退 P4.7 | P8B 显式模式。 |

默认配置位于：

```text
src/aruco_precision_landing_cpp/config/px4_aruco_landing.yaml
```

不要在飞行过程中动态修改模式；每轮实验应停止控制器后重新启动。

## 6. 监控与离线评测

常用监控：

```bash
ros2 topic echo /landing/state
ros2 topic echo /landing/guidance_source
ros2 topic echo /landing/target_pose
ros2 topic echo /landing/estimated_deck_odometry
ros2 topic echo /landing/relative_height
ros2 topic echo /landing/window_open
ros2 topic echo /landing/touchdown_status
ros2 topic echo /landing/touchdown_confirmed
ros2 topic echo /landing/relative_mpc/status
ros2 topic echo /landing/relative_mpc/solve_time_ms
ros2 topic echo /landing/deck_plane/status
ros2 topic echo /landing/deck_plane/upward_normal_ned
ros2 topic echo /landing/deck_plane/skid_clearances
ros2 topic echo /landing/deck_plane/clearance_spread
```

手动录制最小调试 Bag：

```bash
ros2 bag record \
  /landing/state \
  /landing/guidance_source \
  /landing/target_pose \
  /landing/estimated_deck_odometry \
  /landing/relative_height \
  /landing/touchdown_status \
  /landing/touchdown_confirmed \
  /landing/relative_mpc/status \
  /simulation/deck/ground_truth \
  /aruco/visible \
  /fmu/out/vehicle_local_position_v1 \
  /fmu/in/trajectory_setpoint
```

Ground Truth 只能进入离线评测，不得进入控制器。`bags/` 默认被 Git 忽略。P8C 完整验收记录和固定阈值见 `docs/validation/P8C_TILTED_DECK_LANDING_VALIDATION.md`。

阶段评测入口：

```bash
python3 scripts/evaluate_p4_bag.py bags/<bag_name>
python3 scripts/evaluate_p5a_bag.py bags/<bag_name>
python3 scripts/evaluate_p5b_bag.py bags/<bag_name>
python3 scripts/evaluate_p5c_vertical_estimation.py bags/<bag_name>
python3 scripts/evaluate_p6a_touchdown.py bags/<bag_name>
python3 scripts/evaluate_p6b_touchdown.py bags/<bag_name>
python3 scripts/evaluate_p8a_heave_touchdown.py bags/<bag_name>
python3 scripts/evaluate_p8c_tilted_deck.py \
  bags/<bag_name> \
  --scenario tilt_roll_pos_2deg \
  --seed 1 \
  --output-json results/p8c_evaluation.json
```

## 7. 常见问题

检查 PX4 数据：

```bash
ros2 topic echo /fmu/out/vehicle_status_v4 \
  --once \
  --qos-reliability best_effort \
  --qos-durability transient_local

ros2 topic echo /fmu/out/vehicle_local_position_v1 \
  --once \
  --qos-reliability best_effort \
  --qos-durability transient_local
```

`xy_valid`、`z_valid`、`v_xy_valid`、`xy_global` 和 `z_global` 应为 `true`。

检查船舶 GNSS：

```bash
ros2 topic hz /deck/gps/fix
ros2 topic echo /deck/gps/fix --once --qos-reliability best_effort
ros2 topic echo /deck/gps/velocity --once --qos-reliability best_effort
```

理想 GNSS 默认约为 `5 Hz`。

检查重复发布者：

```bash
ros2 topic info /deck/gps/fix -v
ros2 topic info /aruco/pose -v
```

检查残留进程：

```bash
pgrep -af 'MicroXRCEAgent|px4|gz sim|moving_deck|deck_gnss|aruco_detector|px4_aruco'
```

`start_sitl.sh` 检测到同类残留进程时会拒绝启动新一轮实验。

## 8. P8C fixed T1 无人值守固定倾角触地

只允许正 `+2° roll/pitch`，并且必须同时显式给出 relative descent、严格 `0.50 m` 和 final descent：

```bash
python3 scripts/run_single_experiment.py \
  --scenario tilt_roll_pos_2deg \
  --seed 1 \
  --episode-timeout 240 \
  --startup-timeout 120 \
  --touchdown-hold 10 \
  --output-directory results/p8c3_validation_20260802 \
  --batch-id p8c3-final \
  --episode-id tilt_roll_pos_2deg_seed1 \
  --camera-model close-range \
  --tracking-mode PREDICTED_POSITION_VELOCITY_FF \
  --p8c3-touchdown
```

实验器自动等待状态、录 Bag、要求 `TOUCHDOWN_HOLD` 完整维持 10 秒、额外录制 1 秒停机隔离余量、运行 evaluator 并清理本轮进程。发生 terminal recovery 或 evaluator 失败时返回非零，不会自动用二次降落覆盖首次失败。

上述 P8C-3 命令用于保留水平机体历史方案与失败证据。当前固定 T1 的最终状态为：

```text
P8C-3 FAILURE EVIDENCE PRESERVED
P8C-4 VALIDATION PASS
P8C T1 VALIDATION PASS
P8C-3 DESIGN GATE CLOSED
```

P8C-4 使用 Offboard position 模式内的终端主轴法向整形、接触锚点顺应、切向阻尼和受限预压，不是 PX4 attitude setpoint 姿态对齐。负倾角、动态 `rollpitch/combined`、dynamic attitude final descent、Ground Truth 控制、`NAV_LAND` 和自动 Disarm 继续关闭。历史失败证据见 `results/p8c3_validation_20260802/`，最终成功证据见 `results/p8c4_validation_20260802/`。

## 9. 停止与安全边界

一键启动时使用 `Ctrl-C`，脚本会依次发送 `INT`、`TERM`，必要时再终止本轮子进程。

手动启动时建议：

1. 在 QGroundControl 中 Land 并确认无人机已落地；
2. 关闭控制器；
3. 关闭 ArUco 检测器和相机桥接；
4. 关闭移动甲板与 Gazebo；
5. 关闭 PX4 SITL 和 MicroXRCEAgent。

不要在飞行过程中只调用甲板 reset，因为该服务不会同步重置 PX4 飞行器状态。

实机测试前必须重新验证相机内外参、坐标方向、时间同步、控制限幅、Offboard failsafe、人工接管和紧急停机流程。当前 SITL 验收不能直接视为实机自动触地安全证明。
