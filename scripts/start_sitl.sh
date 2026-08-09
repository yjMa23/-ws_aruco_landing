#!/usr/bin/env bash

set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage: ./scripts/start_sitl.sh [options]

Options:
  --environment legacy|marine             Simulation environment (default: legacy)
  --scenario static|constant02|constant|sinusoidal|heave|heave_h1|heave_h2|heave_h3|rollpitch|combined|rigid_body_motion
             |tilt_roll_pos_2deg|tilt_roll_neg_2deg|tilt_pitch_pos_2deg|tilt_pitch_neg_2deg
                                         Deck scenario (default: static)
                                         constant02 = 0.2 m/s, constant = 0.4 m/s
                                         heave_h1/h2/h3 = graded heave touchdown profiles
                                         negative tilt_*_2deg = safe-altitude shadow only
                                         positive tilt_*_2deg = safe altitude, 0.50 m safe descent,
                                         or explicitly enabled touchdown
  --headless                             Run Gazebo without GUI
  --dry-run                              Validate arguments and safety gates without starting processes
  --record                               Record evaluation and ArUco diagnostics topics
  --record-camera-debug                  Record a bag and additionally include raw camera topics
  --bag-output PATH                      Write rosbag to this exact directory (implies --record)
  --seed SEED                            Deterministic deck/GNSS seed (default: 1)
  --rendezvous-altitude METERS           PX4 local altitude target (default: 5.0; 7.0 gives about 5 m above this deck)
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
  --enable-vertical-ff                   Enable deck vertical-velocity feedforward (default)
  --disable-vertical-ff                  Disable vertical feedforward for ablation
  --vertical-ff-gain GAIN                Deck vertical-velocity gain (default: 1.0)
  --vertical-ff-max VALUE                Absolute vertical feedforward limit (default: 0.60)
  --enable-relative-descent              Enable relative-height descent
  --descent-test-height HEIGHT           Override relative descent minimum test height (default: 0.50)
  --descent-fast-rate RATE               relative descent high-altitude rate (default: 0.50)
  --descent-medium-rate RATE             relative descent middle-altitude rate (default: 0.30)
  --descent-slow-rate RATE               relative descent pre-final rate (default: 0.12)
  --landing-window-min-height HEIGHT     Minimum valid estimated height (default: 0.08)
  --enable-final-descent                 Enable final descent/heave touchdown, or fixed-tilt touchdown on positive fixed +2° profiles
  --final-descent-approach-rate RATE     0.50 m to slowdown-height rate (default: 0.12)
  --final-descent-contact-rate RATE      Near-contact rate (default: 0.03)
  --final-descent-rate RATE              Alias for --final-descent-contact-rate
  --final-descent-slowdown-height HEIGHT Switch to contact rate here (default: 0.25)
  --final-descent-terminal-height HEIGHT Begin safe terminal touchdown here (default: 0.20)
  --final-descent-min-height HEIGHT      Physical-contact command clamp (default: 0.05)
  --terminal-contact-stabilization-shadow
                                         terminal contact stabilization diagnostics only; no control output
  --terminal-contact-stabilization-rehearsal
                                         terminal contact stabilization bounded active rehearsal at 0.50 m; no contact
  --enable-terminal-contact-stabilization
                                         terminal contact stabilization active terminal output for positive +2° touchdown
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

environment="legacy"
scenario="static"
headless="false"
dry_run="false"
record="false"
record_camera_debug="false"
bag_output=""
random_seed="1"
rendezvous_altitude_m="5.0"
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
terminal_contact_stabilization_mode="disabled"
terminal_contact_stabilization_enabled="false"
terminal_contact_stabilization_shadow_only="true"
terminal_contact_stabilization_rehearsal_enabled="false"
tuning_override="false"

while (($#)); do
  case "$1" in
    --environment)
      (($# >= 2)) || die "--environment requires a value"
      environment="$2"
      shift 2
      ;;
    --scenario)
      (($# >= 2)) || die "--scenario requires a value"
      scenario="$2"
      shift 2
      ;;
    --headless)
      headless="true"
      shift
      ;;
    --dry-run)
      dry_run="true"
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
    --rendezvous-altitude)
      (($# >= 2)) || die "--rendezvous-altitude requires a value"
      rendezvous_altitude_m="$2"
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
    --terminal-contact-stabilization-shadow)
      [[ "$terminal_contact_stabilization_mode" == "disabled" ]] ||
        die "terminal contact stabilization mode flags are mutually exclusive"
      terminal_contact_stabilization_mode="shadow"
      terminal_contact_stabilization_enabled="true"
      terminal_contact_stabilization_shadow_only="true"
      terminal_contact_stabilization_rehearsal_enabled="false"
      shift
      ;;
    --terminal-contact-stabilization-rehearsal)
      [[ "$terminal_contact_stabilization_mode" == "disabled" ]] ||
        die "terminal contact stabilization mode flags are mutually exclusive"
      terminal_contact_stabilization_mode="rehearsal"
      terminal_contact_stabilization_enabled="true"
      terminal_contact_stabilization_shadow_only="false"
      terminal_contact_stabilization_rehearsal_enabled="true"
      shift
      ;;
    --enable-terminal-contact-stabilization)
      [[ "$terminal_contact_stabilization_mode" == "disabled" ]] ||
        die "terminal contact stabilization mode flags are mutually exclusive"
      terminal_contact_stabilization_mode="active"
      terminal_contact_stabilization_enabled="true"
      terminal_contact_stabilization_shadow_only="false"
      terminal_contact_stabilization_rehearsal_enabled="false"
      shift
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

case "$environment" in
  legacy | marine) ;;
  *) die "invalid environment '$environment' (expected legacy or marine)" ;;
esac

case "$scenario" in
  static) scenario_config="static.yaml" ;;
  constant02) scenario_config="constant_velocity_0p2.yaml" ;;
  constant) scenario_config="constant_velocity.yaml" ;;
  sinusoidal) scenario_config="sinusoidal_xy.yaml" ;;
  heave) scenario_config="heave.yaml" ;;
  heave_h1) scenario_config="heave_h1.yaml" ;;
  heave_h2) scenario_config="heave_h2.yaml" ;;
  heave_h3) scenario_config="heave_h3.yaml" ;;
  rollpitch) scenario_config="roll_pitch.yaml" ;;
  combined) scenario_config="combined.yaml" ;;
  rigid_body_motion) scenario_config="rigid_body_motion.yaml" ;;
  tilt_roll_pos_2deg) scenario_config="tilt_roll_pos_2deg.yaml" ;;
  tilt_roll_neg_2deg) scenario_config="tilt_roll_neg_2deg.yaml" ;;
  tilt_pitch_pos_2deg) scenario_config="tilt_pitch_pos_2deg.yaml" ;;
  tilt_pitch_neg_2deg) scenario_config="tilt_pitch_neg_2deg.yaml" ;;
  *) die "invalid scenario '$scenario' (expected static, constant02, constant, sinusoidal, heave, heave_h1, heave_h2, heave_h3, rollpitch, combined, rigid_body_motion, or a tilted-deck fixed tilt_*_2deg profile)" ;;
esac

if [[ "$environment" == "marine" ]]; then
  if [[ "$relative_descent_enabled" == "true" || "$final_descent_enabled" == "true" ||
    "$terminal_contact_stabilization_mode" != "disabled" ]]
  then
    die "marine environment currently supports safe-altitude validation only; descent, final descent, terminal contact stabilization, NAV_LAND and automatic disarm are forbidden"
  fi
fi

if [[ "$scenario" == "rollpitch" || "$scenario" == "combined" || "$scenario" == "rigid_body_motion" ]]; then
  [[ "$relative_descent_enabled" == "false" ]] ||
    die "dynamic roll/pitch and combined scenarios are restricted to safe-altitude shadow validation"
  [[ "$final_descent_enabled" == "false" ]] ||
    die "dynamic attitude scenarios forbid final descent"
  [[ "$terminal_contact_stabilization_mode" == "disabled" || "$terminal_contact_stabilization_mode" == "shadow" ]] ||
    die "dynamic attitude scenarios forbid active terminal contact stabilization"
fi

tilted_deck_gate_profile="not_tilted_deck"
case "$scenario" in
  tilt_roll_pos_2deg | tilt_pitch_pos_2deg)
    if [[ "$final_descent_enabled" == "true" ]]; then
      [[ "$relative_descent_enabled" == "true" ]] ||
        die "positive fixed-tilt touchdown requires --enable-relative-descent"
      awk -v value="$descent_minimum_test_height_m" \
        'BEGIN {exit !(value == 0.50)}' ||
        die "positive fixed-tilt touchdown requires exactly 0.50 m test height"
      tilted_deck_gate_profile="positive fixed-tilt touchdown"
    elif [[ "$relative_descent_enabled" == "true" ]]; then
      awk -v value="$descent_minimum_test_height_m" \
        'BEGIN {exit !(value == 0.50)}' ||
        die "positive fixed-tilt safe descent requires exactly 0.50 m test height"
      tilted_deck_gate_profile="positive fixed-tilt safe descent"
    else
      tilted_deck_gate_profile="fixed-tilt safe altitude"
    fi
    ;;
  tilt_roll_neg_2deg | tilt_pitch_neg_2deg)
    if [[ "$final_descent_enabled" == "true" ]]; then
      die "negative fixed-tilt final descent and real contact are not open"
    fi
    if [[ "$relative_descent_enabled" == "true" ]]; then
      die "fixed-tilt safe altitude negative fixed-tilt profiles remain safe-altitude shadow only"
    fi
    tilted_deck_gate_profile="fixed-tilt safe altitude"
    ;;
esac

if [[ "$terminal_contact_stabilization_enabled" == "true" ]]; then
  case "$scenario" in
    tilt_roll_pos_2deg | tilt_pitch_pos_2deg) ;;
    *)
      die "terminal contact stabilization is restricted to positive fixed +2 degree roll/pitch scenarios"
      ;;
  esac

  case "$terminal_contact_stabilization_mode" in
    shadow)
      [[ "$final_descent_enabled" == "false" ]] ||
        die "terminal contact stabilization shadow validation requires final descent disabled"
      if [[ "$relative_descent_enabled" == "true" ]]; then
        awk -v value="$descent_minimum_test_height_m" \
          'BEGIN {exit !(value == 0.50)}' ||
          die "terminal contact stabilization shadow safe descent requires exactly 0.50 m test height"
      fi
      ;;
    rehearsal)
      [[ "$relative_descent_enabled" == "true" ]] ||
        die "terminal contact stabilization rehearsal requires relative descent"
      awk -v value="$descent_minimum_test_height_m" \
        'BEGIN {exit !(value == 0.50)}' ||
        die "terminal contact stabilization rehearsal requires exactly 0.50 m test height"
      [[ "$final_descent_enabled" == "false" ]] ||
        die "terminal contact stabilization rehearsal requires final descent disabled"
      ;;
    active)
      [[ "$relative_descent_enabled" == "true" ]] ||
        die "terminal contact stabilization active touchdown requires relative descent"
      awk -v value="$descent_minimum_test_height_m" \
        'BEGIN {exit !(value == 0.50)}' ||
        die "terminal contact stabilization active touchdown requires exactly 0.50 m test height"
      [[ "$final_descent_enabled" == "true" ]] ||
        die "terminal contact stabilization active touchdown requires final descent"
      ;;
    *) die "invalid terminal contact stabilization mode" ;;
  esac
fi

case "$camera_model_profile" in
  px4-default | close-range) ;;
  *) die "invalid camera model '$camera_model_profile' (expected px4-default or close-range)" ;;
esac

case "$tracking_mode" in
  RAW_VISUAL_POSITION) tracking_mode_slug="raw" ;;
  ESTIMATED_POSITION) tracking_mode_slug="estimated" ;;
  ESTIMATED_POSITION_VELOCITY_FF) tracking_mode_slug="estff" ;;
  PREDICTED_POSITION_VELOCITY_FF) tracking_mode_slug="predff" ;;
  RELATIVE_MPC) tracking_mode_slug="mpc" ;;
  *) die "invalid tracking mode '$tracking_mode'" ;;
esac

if [[ "$tracking_mode" == "RELATIVE_MPC" ]]; then
  relative_mpc_prefix="${RELATIVE_MPC_PREFIX:-$HOME/.local/relative-mpc/osqp-1.0.0-osqpeigen-0.11.2}"
  [[ -f "$relative_mpc_prefix/lib/cmake/osqp/osqp-config.cmake" ]] ||
    die "OSQP CMake package not found under $relative_mpc_prefix"
  [[ -f "$relative_mpc_prefix/lib/cmake/OsqpEigen/OsqpEigenConfig.cmake" ]] ||
    die "OsqpEigen CMake package not found under $relative_mpc_prefix"
  export LD_LIBRARY_PATH="$relative_mpc_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

is_nonnegative_number() {
  [[ "$1" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]
}

[[ "$random_seed" =~ ^[0-9]+$ ]] || die "random_seed must be an unsigned integer"
awk -v value="$random_seed" 'BEGIN {exit !(value >= 0 && value <= 4294967295)}' ||
  die "random_seed must fit in uint32"

for value_name in \
  prediction_horizon_s \
  rendezvous_altitude_m \
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

awk -v value="$rendezvous_altitude_m" \
  'BEGIN {exit !(value == 3.0 || value == 5.0 || value == 7.0)}' ||
  die "rendezvous_altitude_m must be exactly 3.0, 5.0, or 7.0"

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
    static | constant02 | constant | sinusoidal | heave_h1 | heave_h2 | heave_h3 | tilt_roll_pos_2deg | tilt_pitch_pos_2deg) ;;
    *) die "--enable-final-descent currently supports static, horizontal-motion, and heave profiles only, plus fixed-tilt touchdown positive fixed +2 degree profiles" ;;
  esac
  awk -v value="$descent_minimum_test_height_m" \
    'BEGIN {exit !(value == 0.50)}' ||
    die "final descent requires --descent-test-height 0.50"
  awk -v window_min="$landing_window_minimum_relative_height_m" \
    -v terminal="$final_descent_terminal_entry_height_m" \
    'BEGIN {exit !(window_min < terminal)}' ||
    die "final descent requires landing-window minimum height below terminal-entry height"
fi

if [[ "$dry_run" == "true" ]]; then
  echo "DRY_RUN validation passed"
  echo "environment=$environment"
  echo "scenario=$scenario"
  echo "rendezvous_altitude_m=$rendezvous_altitude_m"
  echo "relative_descent_enabled=$relative_descent_enabled"
  echo "descent_minimum_test_height_m=$descent_minimum_test_height_m"
  echo "final_descent_enabled=$final_descent_enabled"
  echo "terminal_contact_stabilization_mode=$terminal_contact_stabilization_mode"
  echo "terminal_contact_stabilization_enabled=$terminal_contact_stabilization_enabled"
  echo "terminal_contact_stabilization_shadow_only=$terminal_contact_stabilization_shadow_only"
  echo "terminal_contact_stabilization_rehearsal_enabled=$terminal_contact_stabilization_rehearsal_enabled"
  if [[ "$tilted_deck_gate_profile" != "not_tilted_deck" ]]; then
    echo "TILTED_DECK_GATE=$tilted_deck_gate_profile"
  fi
  exit 0
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

for command in ros2 python3 MicroXRCEAgent make setsid pgrep ps tail grep awk realpath; do
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

stale_pattern='MicroXRCEAgent|(^|/)px4( |$)|gz sim|moving_deck_controller|deck_gnss_simulator|parameter_bridge|aruco_detector_node|px4_aruco_landing_node|mavlink_gcs_heartbeat.py'
# Automated runners often mention process names in their own command line. Exclude
# this script and its complete ancestor chain so only independent stale SITL
# processes block startup.
ancestor_pids=" $$ "
ancestor_pid="$PPID"
while [[ "$ancestor_pid" =~ ^[0-9]+$ ]] && ((ancestor_pid > 1)); do
  ancestor_pids+="$ancestor_pid "
  ancestor_pid="$(ps -o ppid= -p "$ancestor_pid" | awk '{print $1}')"
done
stale_processes="$(
  { pgrep -af "$stale_pattern" || true; } | while read -r process_pid process_command; do
    case "$ancestor_pids" in
      *" $process_pid "*) ;;
      *) printf '%s %s\n' "$process_pid" "$process_command" ;;
    esac
  done
)"
if [[ -n "$stale_processes" ]]; then
  echo "Error: existing SITL processes detected; stop them before starting a new run:" >&2
  printf '%s\n' "$stale_processes" >&2
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

px4_spawn_pose="-4,0,0.2"
if [[ "$environment" == "marine" ]]; then
  px4_spawn_pose="-12,0,0.4"
fi

start_px4() {
  echo "Starting PX4 SITL..."
  # PX4 控制台在 stdin 为 EOF 时会循环刷新提示符，使用不产生数据的常开管道避免占用用户终端。
  setsid bash -c 'exec "$@" < <(tail -f /dev/null)' bash \
    env \
    PX4_GZ_STANDALONE=1 \
    PX4_GZ_WORLD=aruco \
    PX4_GZ_MODEL_POSE="$px4_spawn_pose" \
    make -C "$px4_dir" px4_sitl gz_x500_mono_cam_down &
  child_pids+=("$!")
}

heartbeat_script="$script_dir/mavlink_gcs_heartbeat.py"
[[ -f "$heartbeat_script" ]] || die "local GCS heartbeat script not found: $heartbeat_script"
python3 -c 'import pymavlink' >/dev/null 2>&1 ||
  die "pymavlink is required for unattended PX4 SITL GCS heartbeat"

start_process "MicroXRCEAgent" MicroXRCEAgent udp4 -p 8888
start_px4
# PX4 keeps NAV_DLL_ACT active in SITL. Own a local, non-controlling MAV_TYPE_GCS
# heartbeat so automated runs do not depend on an accidentally running QGroundControl.
start_process "local GCS heartbeat" python3 "$heartbeat_script" \
  --host 127.0.0.1 --port 18570 --rate-hz 1.0
start_process "moving deck ($environment/$scenario)" ros2 launch \
  moving_deck_sim moving_deck_sim.launch.py \
  "environment:=$environment" \
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
  [[ -t 0 ]] || die "an interactive terminal is required for the controller safety confirmation; automation must pass --auto-confirm-controller explicitly"
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
  bag_prefix="horizontal_tracking_${scenario}"
  case "$scenario" in
    tilt_roll_pos_2deg | tilt_roll_neg_2deg | tilt_pitch_pos_2deg | tilt_pitch_neg_2deg)
      bag_prefix="fixed_tilt_safe_altitude_${scenario}"
      ;;
  esac
  if [[ "$relative_descent_enabled" == "true" ]]; then
    case "$scenario" in
      tilt_roll_pos_2deg | tilt_pitch_pos_2deg)
        bag_prefix="fixed_tilt_safe_descent_${scenario}"
        ;;
      *)
        bag_prefix="relative_descent_${scenario}_descent"
        ;;
    esac
    if [[ "$final_descent_enabled" == "true" ]]; then
      case "$scenario" in
        tilt_roll_pos_2deg | tilt_pitch_pos_2deg)
          bag_prefix="tilted_deck_touchdown_${scenario}"
          ;;
        *)
          bag_prefix="final_descent_${scenario}"
          ;;
      esac
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
    /landing/uav_vertical_velocity
    /landing/touchdown_hold_relative_height_reference
    /landing/touchdown_hold_vertical_target
    /landing/touchdown_hold_mode
    /landing/touchdown_hold_reason
    /landing/touchdown_status
    /landing/touchdown_evidence
    /landing/touchdown_candidate_duration
    /landing/touchdown_confirmed
    /landing/final_descent_phase
    /landing/predicted_deck_pose
    /landing/tracking_velocity_setpoint
    /landing/relative_mpc/status
    /landing/relative_mpc/solve_time_ms
    /landing/relative_mpc/iteration_count
    /landing/relative_mpc/objective
    /landing/relative_mpc/fallback_count
    /landing/relative_mpc/first_control
    /landing/relative_mpc/active_constraints
    /landing/relative_mpc/current_state
    /landing/relative_mpc/predicted_path
    /landing/effective_relative_velocity_gain
    /landing/estimated_deck_acceleration
    /landing/estimated_deck_attitude
    /landing/deck_motion_shadow/state
    /landing/deck_motion_shadow/trajectory
    /landing/deck_motion_shadow/status
    /landing/deck_motion_shadow/trusted_horizon_s
    /landing/deck_plane/upward_normal_ned
    /landing/deck_plane/body_clearance
    /landing/deck_plane/skid_clearances
    /landing/deck_plane/minimum_skid_clearance
    /landing/deck_plane/maximum_skid_clearance
    /landing/deck_plane/clearance_spread
    /landing/deck_plane/first_contact_point_index
    /landing/deck_plane/normal_relative_velocity
    /landing/deck_plane/skid_normal_relative_velocities
    /landing/deck_plane/tangential_position_error
    /landing/deck_plane/tangential_relative_velocity
    /landing/deck_plane/status
    /landing/deck_plane/normal_rate_degps
    /landing/deck_plane/marker_switch_normal_jump_deg
    /landing/deck_plane/marker_normals_by_id
    /landing/deck_plane/marker_normal_valid_mask
    /landing/terminal_stabilization/enabled
    /landing/terminal_stabilization/mode
    /landing/terminal_stabilization/reason
    /landing/terminal_stabilization/desired_normal
    /landing/terminal_stabilization/desired_roll_pitch
    /landing/terminal_stabilization/actual_roll_pitch
    /landing/terminal_stabilization/attitude_error
    /landing/terminal_stabilization/acceleration_bias_ned
    /landing/terminal_stabilization/combined_acceleration_ff_ned
    /landing/terminal_stabilization/contact_anchor
    /landing/terminal_stabilization/compliant_target
    /landing/terminal_stabilization/divergence_status
    /landing/window_open
    /landing/window_reject_reasons
    /landing/window_satisfied_duration
    /landing/relative_height
    /landing/relative_height_reference
    /landing/descent_phase
    /simulation/deck/ground_truth
    /simulation/uav/ground_truth_pose
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
  "rendezvous_altitude_m:=$rendezvous_altitude_m" \
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
  "landing_window_minimum_relative_height_m:=$landing_window_minimum_relative_height_m" \
  "terminal_contact_stabilization_enabled:=$terminal_contact_stabilization_enabled" \
  "terminal_contact_stabilization_shadow_only:=$terminal_contact_stabilization_shadow_only" \
  "terminal_contact_stabilization_rehearsal_enabled:=$terminal_contact_stabilization_rehearsal_enabled" \
  "terminal_contact_stabilization_scenario:=$scenario"

echo "SITL is running. Press Ctrl-C to stop all processes."
set +e
wait -n "${child_pids[@]}"
wait_status=$?
set -e
echo "A SITL process exited (status $wait_status); shutting down the stack." >&2
exit 1
