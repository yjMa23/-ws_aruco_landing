#!/usr/bin/env bash

set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage: ./scripts/start_sitl.sh [options]

Options:
  --scenario static|constant02|constant|sinusoidal|heave|rollpitch|combined
                                         Deck scenario (default: static)
                                         constant02 = 0.2 m/s, constant = 0.4 m/s
  --headless                             Run Gazebo without GUI
  --record                               Record evaluation and ArUco diagnostics topics
  --record-camera-debug                  Record a bag and additionally include raw camera topics
  --bag-output PATH                      Write rosbag to this exact directory (implies --record)
  --seed SEED                            Deterministic deck/GNSS seed (default: 1)
  --auto-confirm-controller              Skip the interactive SITL controller confirmation
  --camera-model px4-default|close-range Select camera near clip profile (default: close-range)
  --tracking-mode MODE                   Override tracking.mode
  --prediction-horizon SEC               Override additional prediction horizon
  --velocity-ff-gain GAIN                Override deck velocity feedforward gain
  --relative-velocity-gain GAIN          Override fixed relative velocity damping gain
  --adaptive-relative-gain               Enable acceleration-aware gain scheduling (default)
  --fixed-relative-gain                  Disable scheduling and use the fixed gain
  --adaptive-gain-min GAIN               Scheduled minimum gain (default: 0.25)
  --adaptive-gain-max GAIN               Scheduled maximum gain (default: 1.2)
  --adaptive-accel-low VALUE             Low acceleration threshold (default: 0.05)
  --adaptive-accel-high VALUE            High acceleration threshold (default: 0.35)
  --adaptive-max-accel VALUE             Acceleration clamp (default: 1.50)
  --adaptive-filter-gain GAIN            Acceleration low-pass gain (default: 0.20)
  --enable-vertical-ff                   Enable P5C deck vertical-velocity feedforward (default)
  --disable-vertical-ff                  Disable vertical feedforward for ablation
  --vertical-ff-gain GAIN                Deck vertical-velocity gain (default: 1.0)
  --vertical-ff-max VALUE                Absolute vertical feedforward limit (default: 0.60)
  --enable-relative-descent              Enable P5B relative-height descent
  --descent-test-height HEIGHT           Override P5B minimum test height (default: 0.50)
  --descent-fast-rate RATE               P5B high-altitude rate (default: 0.50)
  --descent-medium-rate RATE             P5B middle-altitude rate (default: 0.30)
  --descent-slow-rate RATE               P5B pre-final rate (default: 0.12)
  --landing-window-min-height HEIGHT     Minimum valid estimated height (default: 0.08)
  --enable-final-descent                 Enable P6B on static or horizontal-motion decks
  --final-descent-approach-rate RATE     0.50 m to slowdown-height rate (default: 0.12)
  --final-descent-contact-rate RATE      Near-contact rate (default: 0.03)
  --final-descent-rate RATE              Alias for --final-descent-contact-rate
  --final-descent-slowdown-height HEIGHT Switch to contact rate here (default: 0.25)
  --final-descent-terminal-height HEIGHT Begin safe terminal touchdown here (default: 0.20)
  --final-descent-min-height HEIGHT      Physical-contact command clamp (default: 0.05)
  -h, --help                             Show this help

Environment:
  PX4_DIR       PX4-Autopilot directory (default: $HOME/PX4-Autopilot)
  PX4_MSGS_WS   px4_msgs workspace (default: $HOME/ws_sensor_combined)
  ROS_SETUP     ROS setup file (default: /opt/ros/${ROS_DISTRO:-humble}/setup.bash)
EOF
}

die() {
  echo "Error: $*" >&2
  exit 1
}

scenario="static"
headless="false"
record="false"
record_camera_debug="false"
bag_output=""
random_seed="1"
auto_confirm_controller="false"
camera_model_profile="close-range"
tracking_mode="PREDICTED_POSITION_VELOCITY_FF"
prediction_horizon_s="0.10"
velocity_feedforward_gain="1.0"
relative_velocity_gain="0.25"
adaptive_relative_velocity_gain_enabled="true"
adaptive_gain_min="0.25"
adaptive_gain_max="1.2"
adaptive_acceleration_low_threshold_mps2="0.05"
adaptive_acceleration_high_threshold_mps2="0.35"
adaptive_max_acceleration_mps2="1.50"
adaptive_acceleration_filter_gain="0.20"
vertical_velocity_feedforward_enabled="true"
vertical_velocity_feedforward_gain="1.0"
vertical_velocity_feedforward_max_mps="0.60"
relative_descent_enabled="false"
descent_minimum_test_height_m="0.50"
descent_fast_rate_mps="0.50"
descent_medium_rate_mps="0.30"
descent_slow_rate_mps="0.12"
landing_window_minimum_relative_height_m="0.08"
final_descent_enabled="false"
final_descent_approach_rate_mps="0.12"
final_descent_contact_rate_mps="0.03"
final_descent_contact_slowdown_height_m="0.25"
final_descent_terminal_entry_height_m="0.20"
final_descent_minimum_command_height_m="0.05"
final_descent_max_reference_tracking_error_m="0.20"
tuning_override="false"

while (($#)); do
  case "$1" in
    --scenario)
      (($# >= 2)) || die "--scenario requires a value"
      scenario="$2"
      shift 2
      ;;
    --headless)
      headless="true"
      shift
      ;;
    --record)
      record="true"
      shift
      ;;
    --record-camera-debug)
      record="true"
      record_camera_debug="true"
      shift
      ;;
    --bag-output)
      (($# >= 2)) || die "--bag-output requires a value"
      record="true"
      bag_output="$2"
      shift 2
      ;;
    --seed)
      (($# >= 2)) || die "--seed requires a value"
      random_seed="$2"
      shift 2
      ;;
    --auto-confirm-controller)
      auto_confirm_controller="true"
      shift
      ;;
    --camera-model)
      (($# >= 2)) || die "--camera-model requires a value"
      camera_model_profile="$2"
      shift 2
      ;;
    --tracking-mode)
      (($# >= 2)) || die "--tracking-mode requires a value"
      tracking_mode="$2"
      tuning_override="true"
      shift 2
      ;;
    --prediction-horizon)
      (($# >= 2)) || die "--prediction-horizon requires a value"
      prediction_horizon_s="$2"
      tuning_override="true"
      shift 2
      ;;
    --velocity-ff-gain)
      (($# >= 2)) || die "--velocity-ff-gain requires a value"
      velocity_feedforward_gain="$2"
      tuning_override="true"
      shift 2
      ;;
    --relative-velocity-gain)
      (($# >= 2)) || die "--relative-velocity-gain requires a value"
      relative_velocity_gain="$2"
      tuning_override="true"
      shift 2
      ;;
    --adaptive-relative-gain)
      adaptive_relative_velocity_gain_enabled="true"
      tuning_override="true"
      shift
      ;;
    --fixed-relative-gain)
      adaptive_relative_velocity_gain_enabled="false"
      tuning_override="true"
      shift
      ;;
    --adaptive-gain-min)
      (($# >= 2)) || die "--adaptive-gain-min requires a value"
      adaptive_gain_min="$2"
      tuning_override="true"
      shift 2
      ;;
    --adaptive-gain-max)
      (($# >= 2)) || die "--adaptive-gain-max requires a value"
      adaptive_gain_max="$2"
      tuning_override="true"
      shift 2
      ;;
    --adaptive-accel-low)
      (($# >= 2)) || die "--adaptive-accel-low requires a value"
      adaptive_acceleration_low_threshold_mps2="$2"
      tuning_override="true"
      shift 2
      ;;
    --adaptive-accel-high)
      (($# >= 2)) || die "--adaptive-accel-high requires a value"
      adaptive_acceleration_high_threshold_mps2="$2"
      tuning_override="true"
      shift 2
      ;;
    --adaptive-max-accel)
      (($# >= 2)) || die "--adaptive-max-accel requires a value"
      adaptive_max_acceleration_mps2="$2"
      tuning_override="true"
      shift 2
      ;;
    --adaptive-filter-gain)
      (($# >= 2)) || die "--adaptive-filter-gain requires a value"
      adaptive_acceleration_filter_gain="$2"
      tuning_override="true"
      shift 2
      ;;
    --enable-vertical-ff)
      vertical_velocity_feedforward_enabled="true"
      tuning_override="true"
      shift
      ;;
    --disable-vertical-ff)
      vertical_velocity_feedforward_enabled="false"
      tuning_override="true"
      shift
      ;;
    --vertical-ff-gain)
      (($# >= 2)) || die "--vertical-ff-gain requires a value"
      vertical_velocity_feedforward_gain="$2"
      tuning_override="true"
      shift 2
      ;;
    --vertical-ff-max)
      (($# >= 2)) || die "--vertical-ff-max requires a value"
      vertical_velocity_feedforward_max_mps="$2"
      tuning_override="true"
      shift 2
      ;;
    --enable-relative-descent)
      relative_descent_enabled="true"
      shift
      ;;
    --descent-test-height)
      (($# >= 2)) || die "--descent-test-height requires a value"
      descent_minimum_test_height_m="$2"
      tuning_override="true"
      shift 2
      ;;
    --descent-fast-rate)
      (($# >= 2)) || die "--descent-fast-rate requires a value"
      descent_fast_rate_mps="$2"
      tuning_override="true"
      shift 2
      ;;
    --descent-medium-rate)
      (($# >= 2)) || die "--descent-medium-rate requires a value"
      descent_medium_rate_mps="$2"
      tuning_override="true"
      shift 2
      ;;
    --descent-slow-rate)
      (($# >= 2)) || die "--descent-slow-rate requires a value"
      descent_slow_rate_mps="$2"
      tuning_override="true"
      shift 2
      ;;
    --landing-window-min-height)
      (($# >= 2)) || die "--landing-window-min-height requires a value"
      landing_window_minimum_relative_height_m="$2"
      tuning_override="true"
      shift 2
      ;;
    --enable-final-descent)
      final_descent_enabled="true"
      shift
      ;;
    --final-descent-approach-rate)
      (($# >= 2)) || die "--final-descent-approach-rate requires a value"
      final_descent_approach_rate_mps="$2"
      tuning_override="true"
      shift 2
      ;;
    --final-descent-contact-rate | --final-descent-rate)
      (($# >= 2)) || die "$1 requires a value"
      final_descent_contact_rate_mps="$2"
      tuning_override="true"
      shift 2
      ;;
    --final-descent-slowdown-height)
      (($# >= 2)) || die "--final-descent-slowdown-height requires a value"
      final_descent_contact_slowdown_height_m="$2"
      tuning_override="true"
      shift 2
      ;;
    --final-descent-terminal-height)
      (($# >= 2)) || die "--final-descent-terminal-height requires a value"
      final_descent_terminal_entry_height_m="$2"
      tuning_override="true"
      shift 2
      ;;
    --final-descent-min-height)
      (($# >= 2)) || die "--final-descent-min-height requires a value"
      final_descent_minimum_command_height_m="$2"
      tuning_override="true"
      shift 2
      ;;
    -h | --help)
      usage
      exit 0
      ;;
    *)
      die "unknown option: $1"
      ;;
  esac
done

case "$scenario" in
  static) scenario_config="static.yaml" ;;
  constant02) scenario_config="constant_velocity_0p2.yaml" ;;
  constant) scenario_config="constant_velocity.yaml" ;;
  sinusoidal) scenario_config="sinusoidal_xy.yaml" ;;
  heave) scenario_config="heave.yaml" ;;
  rollpitch) scenario_config="roll_pitch.yaml" ;;
  combined) scenario_config="combined.yaml" ;;
  *) die "invalid scenario '$scenario' (expected static, constant02, constant, sinusoidal, heave, rollpitch, or combined)" ;;
esac

case "$camera_model_profile" in
  px4-default | close-range) ;;
  *) die "invalid camera model '$camera_model_profile' (expected px4-default or close-range)" ;;
esac

case "$tracking_mode" in
  RAW_VISUAL_POSITION) tracking_mode_slug="raw" ;;
  ESTIMATED_POSITION) tracking_mode_slug="estimated" ;;
  ESTIMATED_POSITION_VELOCITY_FF) tracking_mode_slug="estff" ;;
  PREDICTED_POSITION_VELOCITY_FF) tracking_mode_slug="predff" ;;
  *) die "invalid tracking mode '$tracking_mode'" ;;
esac

is_nonnegative_number() {
  [[ "$1" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]
}

[[ "$random_seed" =~ ^[0-9]+$ ]] || die "random_seed must be an unsigned integer"
awk -v value="$random_seed" 'BEGIN {exit !(value >= 0 && value <= 4294967295)}' ||
  die "random_seed must fit in uint32"

for value_name in \
  prediction_horizon_s \
  velocity_feedforward_gain \
  relative_velocity_gain \
  adaptive_gain_min \
  adaptive_gain_max \
  adaptive_acceleration_low_threshold_mps2 \
  adaptive_acceleration_high_threshold_mps2 \
  adaptive_max_acceleration_mps2 \
  adaptive_acceleration_filter_gain \
  vertical_velocity_feedforward_gain \
  vertical_velocity_feedforward_max_mps \
  descent_minimum_test_height_m \
  descent_fast_rate_mps \
  descent_medium_rate_mps \
  descent_slow_rate_mps \
  landing_window_minimum_relative_height_m \
  final_descent_approach_rate_mps \
  final_descent_contact_rate_mps \
  final_descent_contact_slowdown_height_m \
  final_descent_terminal_entry_height_m \
  final_descent_minimum_command_height_m \
  final_descent_max_reference_tracking_error_m; do
  value="${!value_name}"
  is_nonnegative_number "$value" || die "$value_name must be a non-negative decimal number"
done

awk -v value="$prediction_horizon_s" 'BEGIN {exit !(value <= 0.50)}' ||
  die "prediction_horizon_s must not exceed 0.50"
awk -v value="$velocity_feedforward_gain" 'BEGIN {exit !(value <= 5.0)}' ||
  die "velocity_feedforward_gain must not exceed 5.0"
awk -v value="$relative_velocity_gain" 'BEGIN {exit !(value <= 5.0)}' ||
  die "relative_velocity_gain must not exceed 5.0"
awk -v min="$adaptive_gain_min" -v max="$adaptive_gain_max" \
  'BEGIN {exit !(min <= max && max <= 5.0)}' ||
  die "adaptive gains must satisfy 0 <= min <= max <= 5.0"
awk -v low="$adaptive_acceleration_low_threshold_mps2" \
  -v high="$adaptive_acceleration_high_threshold_mps2" \
  -v max="$adaptive_max_acceleration_mps2" \
  'BEGIN {exit !(low < high && high <= max && max > 0.0)}' ||
  die "adaptive acceleration values must satisfy 0 <= low < high <= max"
awk -v value="$adaptive_acceleration_filter_gain" \
  'BEGIN {exit !(value > 0.0 && value <= 1.0)}' ||
  die "adaptive_acceleration_filter_gain must be within (0, 1]"
awk -v value="$vertical_velocity_feedforward_gain" \
  'BEGIN {exit !(value >= 0.0 && value <= 3.0)}' ||
  die "vertical_velocity_feedforward_gain must be within [0, 3]"
awk -v value="$vertical_velocity_feedforward_max_mps" \
  'BEGIN {exit !(value > 0.0 && value <= 2.0)}' ||
  die "vertical_velocity_feedforward_max_mps must be within (0, 2]"
awk -v value="$descent_minimum_test_height_m" \
  'BEGIN {exit !(value >= 0.50 && value < 0.80)}' ||
  die "descent_minimum_test_height_m must be within [0.50, 0.80)"
awk -v fast="$descent_fast_rate_mps" \
  -v medium="$descent_medium_rate_mps" \
  -v slow="$descent_slow_rate_mps" \
  'BEGIN {exit !(slow > 0.0 && slow <= medium && medium <= fast && fast <= 0.60)}' ||
  die "descent rates must satisfy 0 < slow <= medium <= fast <= 0.60"
awk -v value="$landing_window_minimum_relative_height_m" \
  'BEGIN {exit !(value > 0.0 && value < 6.0)}' ||
  die "landing_window_minimum_relative_height_m must be within (0, 6)"
awk -v approach="$final_descent_approach_rate_mps" \
  -v contact="$final_descent_contact_rate_mps" \
  'BEGIN {exit !(contact > 0.0 && contact <= approach && approach <= 0.20 && contact <= 0.05)}' ||
  die "final descent rates must satisfy 0 < contact <= approach <= 0.20 and contact <= 0.05"
awk -v minimum="$final_descent_minimum_command_height_m" \
  -v terminal="$final_descent_terminal_entry_height_m" \
  -v slowdown="$final_descent_contact_slowdown_height_m" \
  -v entry="$descent_minimum_test_height_m" \
  'BEGIN {exit !(minimum >= 0.02 && minimum < terminal && terminal < slowdown && slowdown < entry)}' ||
  die "final heights must satisfy 0.02 <= minimum < terminal < slowdown < descent test height"
awk -v value="$final_descent_max_reference_tracking_error_m" \
  'BEGIN {exit !(value > 0.0 && value <= 0.30)}' ||
  die "final_descent_max_reference_tracking_error_m must be within (0, 0.30]"
if [[ "$final_descent_enabled" == "true" ]]; then
  [[ "$relative_descent_enabled" == "true" ]] ||
    die "--enable-final-descent requires --enable-relative-descent"
  case "$scenario" in
    static | constant02 | constant | sinusoidal) ;;
    *) die "--enable-final-descent currently supports static and horizontal-motion scenarios only" ;;
  esac
  awk -v value="$descent_minimum_test_height_m" \
    'BEGIN {exit !(value == 0.50)}' ||
    die "P6B final descent requires --descent-test-height 0.50"
  awk -v window_min="$landing_window_minimum_relative_height_m" \
    -v terminal="$final_descent_terminal_entry_height_m" \
    'BEGIN {exit !(window_min < terminal)}' ||
    die "final descent requires landing-window minimum height below terminal-entry height"
fi

sanitize_number() {
  local value="$1"
  value="${value//./p}"
  printf '%s' "$value"
}

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd -- "$script_dir/.." && pwd)"
ros_setup="${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-humble}/setup.bash}"
px4_dir="${PX4_DIR:-${HOME}/PX4-Autopilot}"
px4_msgs_ws="${PX4_MSGS_WS:-${HOME}/ws_sensor_combined}"
workspace_setup="$workspace_dir/install/setup.bash"
px4_msgs_setup="$px4_msgs_ws/install/setup.bash"
px4_gz_env="$px4_dir/build/px4_sitl_default/rootfs/gz_env.sh"

for file in "$ros_setup" "$px4_msgs_setup" "$workspace_setup" "$px4_gz_env"; do
  [[ -f "$file" ]] || die "required environment file not found: $file"
done

# ROS/PX4 生成的环境脚本会读取未定义变量，加载期间暂时关闭 nounset。
set +u
# shellcheck disable=SC1090
source "$ros_setup"
# shellcheck disable=SC1090
source "$px4_msgs_setup"
# shellcheck disable=SC1090
source "$workspace_setup"
# shellcheck disable=SC1090
source "$px4_gz_env"
set -u

for command in ros2 python3 MicroXRCEAgent make setsid pgrep tail grep awk realpath; do
  command -v "$command" >/dev/null || die "required command not found: $command"
done

deck_share="$(ros2 pkg prefix --share moving_deck_sim)" ||
  die "moving_deck_sim is not built; run colcon build first"
scenario_path="$deck_share/config/$scenario_config"
gnss_config_path="$deck_share/config/gnss_ideal.yaml"
[[ -f "$scenario_path" ]] || die "scenario config not found: $scenario_path"
[[ -f "$gnss_config_path" ]] || die "GNSS config not found: $gnss_config_path"

px4_camera_model_path="$px4_dir/Tools/simulation/gz/models/mono_cam/model.sdf"
project_camera_models_dir="$deck_share/models"
project_camera_model_path="$project_camera_models_dir/mono_cam/model.sdf"
case "$camera_model_profile" in
  px4-default)
    camera_model_path="$px4_camera_model_path"
    export GZ_SIM_RESOURCE_PATH="$PX4_GZ_MODELS:$PX4_GZ_WORLDS${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
    ;;
  close-range)
    camera_model_path="$project_camera_model_path"
    export GZ_SIM_RESOURCE_PATH="$project_camera_models_dir:$PX4_GZ_MODELS:$PX4_GZ_WORLDS${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
    ;;
esac
[[ -f "$camera_model_path" ]] || die "camera model not found: $camera_model_path"
camera_near_clip="$(awk -F'[<>]' '/<near>/{gsub(/[[:space:]]/, "", $3); print $3; exit}' "$camera_model_path")"
[[ -n "$camera_near_clip" ]] || die "camera near clip not found in: $camera_model_path"

if [[ "$headless" != "true" ]]; then
  export QT_QPA_PLATFORM="${GAZEBO_QT_QPA_PLATFORM:-${QT_QPA_PLATFORM:-xcb}}"
  if [[ "$QT_QPA_PLATFORM" == "xcb" ]]; then
    export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
  fi
fi

echo "Camera model profile: $camera_model_profile"
echo "Camera model path: $camera_model_path"
echo "Camera near clip: $camera_near_clip m"
echo "Gazebo model priority: ${GZ_SIM_RESOURCE_PATH%%:*}"
[[ "$headless" == "true" ]] || echo "Gazebo Qt platform: $QT_QPA_PLATFORM"

stale_pattern='MicroXRCEAgent|(^|/)px4( |$)|gz sim|moving_deck_controller|deck_gnss_simulator|parameter_bridge.*world/aruco|aruco_detector_node|px4_aruco_landing_node'
if pgrep -f "$stale_pattern" >/dev/null; then
  echo "Error: existing SITL processes detected; stop them before starting a new run:" >&2
  pgrep -af "$stale_pattern" >&2
  exit 1
fi

declare -a child_pids=()

groups_alive() {
  local pid
  for pid in "${child_pids[@]}"; do
    if kill -0 -- "-$pid" 2>/dev/null; then
      return 0
    fi
  done
  return 1
}

signal_groups() {
  local signal="$1"
  local pid
  for pid in "${child_pids[@]}"; do
    kill -"$signal" -- "-$pid" 2>/dev/null || true
  done
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if ((${#child_pids[@]})); then
    echo
    echo "Stopping SITL processes..."
    signal_groups INT
    for _ in {1..25}; do
      groups_alive || break
      sleep 0.2
    done
    if groups_alive; then
      signal_groups TERM
      for _ in {1..10}; do
        groups_alive || break
        sleep 0.2
      done
    fi
    groups_alive && signal_groups KILL
    wait "${child_pids[@]}" 2>/dev/null || true
  fi
  exit "$status"
}

trap cleanup EXIT
trap 'exit 130' INT TERM

start_process() {
  local label="$1"
  shift
  echo "Starting $label..."
  setsid "$@" </dev/null &
  child_pids+=("$!")
}

start_px4() {
  echo "Starting PX4 SITL..."
  # PX4 控制台在 stdin 为 EOF 时会循环刷新提示符，使用不产生数据的常开管道避免占用用户终端。
  setsid bash -c 'exec "$@" < <(tail -f /dev/null)' bash \
    env \
    PX4_GZ_STANDALONE=1 \
    PX4_GZ_WORLD=aruco \
    PX4_GZ_MODEL_POSE=-4,0,0.2 \
    make -C "$px4_dir" px4_sitl gz_x500_mono_cam_down &
  child_pids+=("$!")
}

start_process "MicroXRCEAgent" MicroXRCEAgent udp4 -p 8888
start_px4
start_process "moving deck ($scenario)" ros2 launch \
  moving_deck_sim moving_deck_sim.launch.py \
  "config_file:=$scenario_path" \
  "gnss_config_file:=$gnss_config_path" \
  "headless:=$headless" \
  "random_seed:=$random_seed"
start_process "camera bridge" ros2 run ros_gz_bridge parameter_bridge \
  '/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
start_process "ArUco detector" ros2 launch aruco_detector aruco_detector.launch.py \
  use_sim_time:=true

if [[ "$auto_confirm_controller" != "true" ]]; then
  [[ -t 0 ]] || die "an interactive terminal is required for the controller safety confirmation; P7 automation must pass --auto-confirm-controller explicitly"
fi
echo "Waiting for PX4 ROS topics..."
until python3 "$script_dir/check_ros_topic.py" /fmu/out/vehicle_status_v4; do
  for pid in "${child_pids[@]}"; do
    kill -0 "$pid" 2>/dev/null || die "a startup process exited while waiting for PX4"
  done
  sleep 1
done
echo
echo "The controller will switch to Offboard and arm automatically."
if [[ "$auto_confirm_controller" == "true" ]]; then
  echo "Controller safety confirmation: explicitly auto-confirmed for automated SITL."
else
  printf 'Confirm PX4/QGroundControl is healthy, then press Enter to start the controller... '
  while true; do
    read_status=0
    read -r -t 1 || read_status=$?
    ((read_status == 0)) && break
    ((read_status > 128)) || die "controller confirmation cancelled"
    for pid in "${child_pids[@]}"; do
      kill -0 "$pid" 2>/dev/null || die "a startup process exited before controller confirmation"
    done
  done
fi

if [[ "$record" == "true" ]]; then
  bag_prefix="p4_${scenario}"
  if [[ "$relative_descent_enabled" == "true" ]]; then
    bag_prefix="p5b_${scenario}_descent"
    if [[ "$final_descent_enabled" == "true" ]]; then
      bag_prefix="p6b_${scenario}_final_descent"
    fi
    if [[ "$vertical_velocity_feedforward_enabled" == "true" ]]; then
      bag_prefix+="_zff$(sanitize_number "$vertical_velocity_feedforward_gain")"
    else
      bag_prefix+="_zffoff"
    fi
  fi
  if [[ "$tuning_override" == "true" ]]; then
    bag_prefix+="_${tracking_mode_slug}"
    bag_prefix+="_h$(sanitize_number "$prediction_horizon_s")"
    bag_prefix+="_vff$(sanitize_number "$velocity_feedforward_gain")"
    bag_prefix+="_rvg$(sanitize_number "$relative_velocity_gain")"
    if [[ "$adaptive_relative_velocity_gain_enabled" == "true" ]]; then
      bag_prefix+="_adapt"
      bag_prefix+="_g$(sanitize_number "$adaptive_gain_min")-$(sanitize_number "$adaptive_gain_max")"
      bag_prefix+="_a$(sanitize_number "$adaptive_acceleration_low_threshold_mps2")-$(sanitize_number "$adaptive_acceleration_high_threshold_mps2")"
      bag_prefix+="_f$(sanitize_number "$adaptive_acceleration_filter_gain")"
    fi
    if [[ "$descent_minimum_test_height_m" != "0.50" ]]; then
      bag_prefix+="_hmin$(sanitize_number "$descent_minimum_test_height_m")"
    fi
    if [[ "$landing_window_minimum_relative_height_m" != "0.08" ]]; then
      bag_prefix+="_winmin$(sanitize_number "$landing_window_minimum_relative_height_m")"
    fi
    if [[ "$descent_fast_rate_mps" != "0.50" ||
      "$descent_medium_rate_mps" != "0.30" ||
      "$descent_slow_rate_mps" != "0.12" ]]
    then
      bag_prefix+="_dr$(sanitize_number "$descent_fast_rate_mps")"
      bag_prefix+="-$(sanitize_number "$descent_medium_rate_mps")"
      bag_prefix+="-$(sanitize_number "$descent_slow_rate_mps")"
    fi
    if [[ "$final_descent_enabled" == "true" ]]; then
      bag_prefix+="_fa$(sanitize_number "$final_descent_approach_rate_mps")"
      bag_prefix+="_fc$(sanitize_number "$final_descent_contact_rate_mps")"
      bag_prefix+="_fh$(sanitize_number "$final_descent_contact_slowdown_height_m")"
      bag_prefix+="_ft$(sanitize_number "$final_descent_terminal_entry_height_m")"
      bag_prefix+="_fmin$(sanitize_number "$final_descent_minimum_command_height_m")"
    fi
  fi
  if [[ "$record_camera_debug" == "true" ]]; then
    bag_prefix+="_camdebug_near$(sanitize_number "$camera_near_clip")"
  fi
  if [[ -n "$bag_output" ]]; then
    bag_path="$(realpath -m "$bag_output")"
    [[ ! -e "$bag_path" ]] || die "bag output already exists: $bag_path"
    mkdir -p "$(dirname "$bag_path")"
  else
    bag_path="$workspace_dir/bags/${bag_prefix}_$(date +%Y%m%d_%H%M%S)"
    mkdir -p "$workspace_dir/bags"
  fi
  echo "BAG_PATH=$bag_path"
  declare -a bag_topics=(
    /landing/state
    /landing/guidance_source
    /landing/target_pose
    /landing/deck_gnss_pose_ned
    /landing/marker_pose_ned
    /landing/active_marker_id
    /landing/estimated_deck_odometry
    /landing/vertical_state
    /landing/raw_relative_height
    /landing/relative_vertical_velocity
    /landing/touchdown_status
    /landing/touchdown_evidence
    /landing/touchdown_candidate_duration
    /landing/touchdown_confirmed
    /landing/final_descent_phase
    /landing/predicted_deck_pose
    /landing/tracking_velocity_setpoint
    /landing/effective_relative_velocity_gain
    /landing/estimated_deck_acceleration
    /landing/estimated_deck_attitude
    /landing/window_open
    /landing/window_reject_reasons
    /landing/window_satisfied_duration
    /landing/relative_height
    /landing/relative_height_reference
    /landing/descent_phase
    /simulation/deck/ground_truth
    /aruco/visible
    /aruco/id
    /aruco/pose
    /aruco/active_marker_id
    /aruco/selected_corner_area_px2
    /aruco/selected_border_margin_px
    /aruco/selection_reason
    /fmu/out/vehicle_local_position_v1
    /fmu/out/vehicle_land_detected
    /fmu/out/vehicle_odometry
    /fmu/in/trajectory_setpoint
    /fmu/in/vehicle_command
  )
  if [[ "$record_camera_debug" == "true" ]]; then
    bag_topics+=(
      /world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image
      /world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info
      /aruco/debug_image
    )
  fi
  start_process "rosbag ($bag_path)" ros2 bag record -o "$bag_path" "${bag_topics[@]}"
fi

start_process "landing controller" ros2 launch \
  aruco_precision_landing_cpp px4_aruco_landing.launch.py \
  use_sim_time:=true \
  "tracking_mode:=$tracking_mode" \
  "prediction_horizon_s:=$prediction_horizon_s" \
  "velocity_feedforward_gain:=$velocity_feedforward_gain" \
  "relative_velocity_gain:=$relative_velocity_gain" \
  "adaptive_relative_velocity_gain_enabled:=$adaptive_relative_velocity_gain_enabled" \
  "adaptive_gain_min:=$adaptive_gain_min" \
  "adaptive_gain_max:=$adaptive_gain_max" \
  "adaptive_acceleration_low_threshold_mps2:=$adaptive_acceleration_low_threshold_mps2" \
  "adaptive_acceleration_high_threshold_mps2:=$adaptive_acceleration_high_threshold_mps2" \
  "adaptive_max_acceleration_mps2:=$adaptive_max_acceleration_mps2" \
  "adaptive_acceleration_filter_gain:=$adaptive_acceleration_filter_gain" \
  "vertical_velocity_feedforward_enabled:=$vertical_velocity_feedforward_enabled" \
  "vertical_velocity_feedforward_gain:=$vertical_velocity_feedforward_gain" \
  "vertical_velocity_feedforward_max_mps:=$vertical_velocity_feedforward_max_mps" \
  "final_descent_enabled:=$final_descent_enabled" \
  "final_descent_entry_height_m:=$descent_minimum_test_height_m" \
  "final_descent_approach_rate_mps:=$final_descent_approach_rate_mps" \
  "final_descent_contact_rate_mps:=$final_descent_contact_rate_mps" \
  "final_descent_contact_slowdown_height_m:=$final_descent_contact_slowdown_height_m" \
  "final_descent_terminal_entry_height_m:=$final_descent_terminal_entry_height_m" \
  "final_descent_minimum_command_height_m:=$final_descent_minimum_command_height_m" \
  "final_descent_max_reference_tracking_error_m:=$final_descent_max_reference_tracking_error_m" \
  "relative_descent_enabled:=$relative_descent_enabled" \
  "descent_minimum_test_height_m:=$descent_minimum_test_height_m" \
  "descent_fast_rate_mps:=$descent_fast_rate_mps" \
  "descent_medium_rate_mps:=$descent_medium_rate_mps" \
  "descent_slow_rate_mps:=$descent_slow_rate_mps" \
  "landing_window_minimum_relative_height_m:=$landing_window_minimum_relative_height_m"

echo "SITL is running. Press Ctrl-C to stop all processes."
set +e
wait -n "${child_pids[@]}"
wait_status=$?
set -e
echo "A SITL process exited (status $wait_status); shutting down the stack." >&2
exit 1
