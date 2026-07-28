// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/px4_aruco_landing_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace aruco_precision_landing_cpp
{

namespace
{

constexpr double kMinimumQuaternionNorm = 1.0e-6;
constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;

bool is_positive_finite(double value)
{
  return std::isfinite(value) && value > 0.0;
}

template<std::size_t Size>
std::array<double, Size> to_array(
  const std::vector<double> & values,
  const std::string & parameter_name)
{
  if (values.size() != Size) {
    throw std::invalid_argument(
            parameter_name + " must contain " + std::to_string(Size) + " values");
  }

  std::array<double, Size> result{};
  for (std::size_t index = 0; index < Size; ++index) {
    result[index] = values[index];
  }
  return result;
}

}  // namespace

Px4ArucoLandingNode::Px4ArucoLandingNode()
: Node("px4_aruco_landing_node")
{
  declare_and_load_parameters();
  validate_parameters();

  GnssRendezvousParameters gnss_parameters;
  gnss_parameters.fix_timeout_s = gnss_fix_timeout_s_;
  gnss_parameters.velocity_timeout_s = gnss_velocity_timeout_s_;
  gnss_parameters.stable_duration_s = gnss_stable_duration_s_;
  gnss_parameters.max_fix_jump_m = max_gnss_jump_m_;
  gnss_parameters.max_target_step_m = max_target_step_m_;
  gnss_parameters.max_target_speed_mps = max_rendezvous_speed_mps_;
  gnss_parameters.search_offset_m = search_offset_m_;
  gnss_parameters.search_point_hold_s = search_point_hold_s_;
  gnss_parameters.max_geodetic_range_m = gnss_max_geodetic_range_m_;
  gnss_guidance_ = std::make_unique<GnssRendezvousGuidance>(gnss_parameters);

  VisualHandoverParameters visual_parameters;
  visual_parameters.handover_duration_s = visual_handover_duration_s_;
  visual_parameters.max_gnss_visual_difference_m =
    handover_max_horizontal_difference_m_;
  visual_parameters.max_visual_measurement_jump_m = max_visual_measurement_jump_m_;
  visual_parameters.visual_loss_short_timeout_s = visual_loss_short_timeout_s_;
  visual_parameters.visual_loss_long_timeout_s = visual_loss_long_timeout_s_;
  visual_parameters.max_target_speed_mps = max_rendezvous_speed_mps_;
  visual_parameters.max_target_step_m = max_target_step_m_;
  visual_guidance_ = std::make_unique<VisualHandoverGuidance>(visual_parameters);

  TargetStateEstimatorParameters estimator_parameters;
  estimator_parameters.process_acceleration_std_mps2 =
    estimator_process_acceleration_std_mps2_;
  estimator_parameters.measurement_horizontal_std_m =
    estimator_measurement_horizontal_std_m_;
  estimator_parameters.measurement_vertical_std_m =
    estimator_measurement_vertical_std_m_;
  estimator_parameters.initial_position_std_m = estimator_initial_position_std_m_;
  estimator_parameters.initial_velocity_std_mps = estimator_initial_velocity_std_mps_;
  estimator_parameters.minimum_sample_dt_s = estimator_minimum_sample_dt_s_;
  estimator_parameters.maximum_sample_dt_s = estimator_maximum_sample_dt_s_;
  estimator_parameters.reinitialize_gap_s = estimator_reinitialize_gap_s_;
  estimator_parameters.innovation_gate_mahalanobis =
    estimator_innovation_gate_mahalanobis_;
  target_state_estimator_ =
    std::make_unique<TargetStateEstimator>(estimator_parameters);

  VerticalStateEstimatorParameters vertical_parameters;
  vertical_parameters.process_acceleration_std_mps2 =
    vertical_process_acceleration_std_mps2_;
  vertical_parameters.measurement_std_m = vertical_measurement_std_m_;
  vertical_parameters.measurement_bias_m = vertical_measurement_bias_m_;
  vertical_parameters.initial_position_std_m = vertical_initial_position_std_m_;
  vertical_parameters.initial_velocity_std_mps = vertical_initial_velocity_std_mps_;
  vertical_parameters.minimum_sample_dt_s = vertical_minimum_sample_dt_s_;
  vertical_parameters.maximum_sample_dt_s = vertical_maximum_sample_dt_s_;
  vertical_parameters.reinitialize_gap_s = vertical_reinitialize_gap_s_;
  vertical_parameters.innovation_gate_mahalanobis =
    vertical_innovation_gate_mahalanobis_;
  vertical_state_estimator_ =
    std::make_unique<VerticalStateEstimator>(vertical_parameters);

  MotionPredictorParameters predictor_parameters;
  predictor_parameters.additional_prediction_horizon_s =
    additional_prediction_horizon_s_;
  predictor_parameters.max_prediction_horizon_s = max_prediction_horizon_s_;
  motion_predictor_ = std::make_unique<MotionPredictor>(predictor_parameters);

  const auto tracking_mode = tracking_control_mode_from_string(tracking_mode_string_);
  if (!tracking_mode.has_value()) {
    throw std::invalid_argument("Unsupported tracking.mode: " + tracking_mode_string_);
  }
  MovingTargetTrackingParameters tracking_parameters;
  tracking_parameters.mode = *tracking_mode;
  tracking_parameters.max_position_target_speed_mps =
    tracking_max_position_target_speed_mps_;
  tracking_parameters.max_position_target_step_m =
    tracking_max_position_target_step_m_;
  tracking_parameters.velocity_feedforward_gain =
    tracking_velocity_feedforward_gain_;
  tracking_parameters.relative_velocity_gain = tracking_relative_velocity_gain_;
  tracking_parameters.adaptive_relative_velocity_gain_enabled =
    tracking_adaptive_relative_velocity_gain_enabled_;
  tracking_parameters.adaptive_relative_velocity_gain_parameters.min_gain =
    tracking_adaptive_relative_velocity_gain_min_;
  tracking_parameters.adaptive_relative_velocity_gain_parameters.max_gain =
    tracking_adaptive_relative_velocity_gain_max_;
  tracking_parameters.adaptive_relative_velocity_gain_parameters.
  acceleration_low_threshold_mps2 =
    tracking_adaptive_acceleration_low_threshold_mps2_;
  tracking_parameters.adaptive_relative_velocity_gain_parameters.
  acceleration_high_threshold_mps2 =
    tracking_adaptive_acceleration_high_threshold_mps2_;
  tracking_parameters.adaptive_relative_velocity_gain_parameters.max_acceleration_mps2 =
    tracking_adaptive_max_acceleration_mps2_;
  tracking_parameters.adaptive_relative_velocity_gain_parameters.acceleration_filter_gain =
    tracking_adaptive_acceleration_filter_gain_;
  tracking_parameters.max_velocity_feedforward_mps =
    tracking_max_velocity_feedforward_mps_;
  tracking_parameters.max_velocity_feedforward_acceleration_mps2 =
    tracking_max_velocity_feedforward_acceleration_mps2_;
  tracking_parameters.max_prediction_age_s = tracking_max_prediction_age_s_;
  tracking_controller_ =
    std::make_unique<MovingTargetTrackingController>(tracking_parameters);

  VehiclePoseHistoryParameters pose_history_parameters;
  pose_history_parameters.history_duration_s = vehicle_pose_history_duration_s_;
  pose_history_parameters.max_endpoint_hold_s =
    vehicle_pose_history_max_endpoint_hold_s_;
  vehicle_pose_history_ =
    std::make_unique<VehiclePoseHistory>(pose_history_parameters);

  DeckAttitudeEstimatorParameters attitude_parameters;
  attitude_parameters.filter_gain = deck_attitude_filter_gain_;
  attitude_parameters.minimum_upward_normal_component =
    deck_attitude_minimum_upward_normal_component_;
  deck_attitude_estimator_ =
    std::make_unique<DeckAttitudeEstimator>(attitude_parameters);

  LandingWindowParameters landing_window_parameters;
  landing_window_parameters.enter_horizontal_error_m =
    landing_window_enter_horizontal_error_m_;
  landing_window_parameters.exit_horizontal_error_m =
    landing_window_exit_horizontal_error_m_;
  landing_window_parameters.enter_relative_speed_mps =
    landing_window_enter_relative_speed_mps_;
  landing_window_parameters.exit_relative_speed_mps =
    landing_window_exit_relative_speed_mps_;
  landing_window_parameters.enter_max_tilt_rad =
    landing_window_enter_max_tilt_deg_ * kDegreesToRadians;
  landing_window_parameters.exit_max_tilt_rad =
    landing_window_exit_max_tilt_deg_ * kDegreesToRadians;
  landing_window_parameters.max_visual_age_s = landing_window_max_visual_age_s_;
  landing_window_parameters.minimum_relative_height_m =
    landing_window_minimum_relative_height_m_;
  landing_window_parameters.maximum_relative_height_m =
    landing_window_maximum_relative_height_m_;
  landing_window_parameters.required_duration_s = landing_window_required_duration_s_;
  landing_window_ = std::make_unique<LandingWindow>(landing_window_parameters);

  RelativeDescentParameters descent_parameters;
  descent_parameters.minimum_test_height_m = descent_minimum_test_height_m_;
  descent_parameters.fast_height_threshold_m = descent_fast_height_threshold_m_;
  descent_parameters.slow_height_threshold_m = descent_slow_height_threshold_m_;
  descent_parameters.fast_rate_mps = descent_fast_rate_mps_;
  descent_parameters.medium_rate_mps = descent_medium_rate_mps_;
  descent_parameters.slow_rate_mps = descent_slow_rate_mps_;
  descent_parameters.recovery_height_m = descent_recovery_height_m_;
  descent_parameters.recovery_rate_mps = descent_recovery_rate_mps_;
  descent_parameters.max_reference_tracking_error_m =
    descent_max_reference_tracking_error_m_;
  relative_descent_controller_ =
    std::make_unique<RelativeDescentController>(descent_parameters);

  TouchdownDetectorParameters touchdown_parameters;
  touchdown_parameters.px4_status_timeout_s = touchdown_px4_status_timeout_s_;
  touchdown_parameters.visual_timeout_s = touchdown_visual_timeout_s_;
  touchdown_parameters.low_height_enter_m = touchdown_low_height_enter_m_;
  touchdown_parameters.low_height_exit_m = touchdown_low_height_exit_m_;
  touchdown_parameters.max_relative_vertical_speed_mps =
    touchdown_max_relative_vertical_speed_mps_;
  touchdown_parameters.max_uav_vertical_speed_mps =
    touchdown_max_uav_vertical_speed_mps_;
  touchdown_parameters.max_relative_horizontal_speed_mps =
    touchdown_max_relative_horizontal_speed_mps_;
  touchdown_parameters.candidate_required_duration_s =
    touchdown_candidate_required_duration_s_;
  touchdown_detector_ = std::make_unique<TouchdownDetector>(touchdown_parameters);

  FinalDescentParameters final_descent_parameters;
  final_descent_parameters.entry_height_m = final_descent_entry_height_m_;
  final_descent_parameters.approach_rate_mps = final_descent_approach_rate_mps_;
  final_descent_parameters.contact_rate_mps = final_descent_contact_rate_mps_;
  final_descent_parameters.contact_slowdown_height_m =
    final_descent_contact_slowdown_height_m_;
  final_descent_parameters.minimum_command_height_m =
    final_descent_minimum_command_height_m_;
  final_descent_parameters.maximum_reference_tracking_error_m =
    final_descent_max_reference_tracking_error_m_;
  final_descent_controller_ =
    std::make_unique<FinalDescentController>(final_descent_parameters);

  body_camera_pose_.translation = Eigen::Vector3d{
    camera_translation_frd_m_[0],
    camera_translation_frd_m_[1],
    camera_translation_frd_m_[2]};
  body_camera_pose_.rotation = Eigen::Quaterniond{
    camera_rotation_wxyz_[0],
    camera_rotation_wxyz_[1],
    camera_rotation_wxyz_[2],
    camera_rotation_wxyz_[3]};
  body_camera_pose_.rotation.normalize();

  create_ros_interfaces();

  last_control_time_ = get_clock()->now();
  const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / control_rate_hz_));
  control_timer_ = create_wall_timer(
    timer_period,
    std::bind(&Px4ArucoLandingNode::control_timer_callback, this));

  RCLCPP_INFO(
    get_logger(),
    "PX4 landing controller started at %.1f Hz; GNSS rendezvous altitude %.2f m, "
    "radius %.2f m; tracking mode %s",
    control_rate_hz_,
    rendezvous_altitude_m_,
    rendezvous_radius_m_,
    tracking_control_mode_name(tracking_controller_->mode()));
}

void Px4ArucoLandingNode::declare_and_load_parameters()
{
  control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);
  takeoff_alt_ = declare_parameter<double>("takeoff_alt", 3.0);
  search_x_ = declare_parameter<double>("search_x", 0.0);
  search_y_ = declare_parameter<double>("search_y", 0.0);
  search_alt_ = declare_parameter<double>("search_alt", 3.0);
  abort_hover_alt_ = declare_parameter<double>("abort_hover_alt", 3.0);
  rendezvous_altitude_m_ = declare_parameter<double>("rendezvous_altitude_m", 5.0);
  rendezvous_radius_m_ = declare_parameter<double>("rendezvous_radius_m", 2.0);
  gnss_fix_timeout_s_ = declare_parameter<double>("gnss_fix_timeout_s", 1.0);
  gnss_velocity_timeout_s_ = declare_parameter<double>("gnss_velocity_timeout_s", 1.0);
  gnss_stable_duration_s_ = declare_parameter<double>("gnss_stable_duration_s", 1.0);
  max_gnss_jump_m_ = declare_parameter<double>("max_gnss_jump_m", 5.0);
  max_rendezvous_speed_mps_ =
    declare_parameter<double>("max_rendezvous_speed_mps", 2.0);
  max_target_step_m_ = declare_parameter<double>("max_target_step_m", 0.20);
  search_offset_m_ = declare_parameter<double>("search_offset_m", 1.0);
  search_point_hold_s_ = declare_parameter<double>("search_point_hold_s", 1.0);
  aruco_acquire_duration_s_ =
    declare_parameter<double>("aruco_acquire_duration_s", 0.5);
  gnss_max_geodetic_range_m_ =
    declare_parameter<double>("gnss_max_geodetic_range_m", 10000.0);
  visual_handover_duration_s_ =
    declare_parameter<double>("visual_handover_duration_s", 0.5);
  handover_max_horizontal_difference_m_ = declare_parameter<double>(
    "handover_max_horizontal_difference_m", 3.0);
  max_visual_measurement_jump_m_ =
    declare_parameter<double>("max_visual_measurement_jump_m", 0.5);
  visual_loss_short_timeout_s_ =
    declare_parameter<double>("visual_loss_short_timeout_s", 0.5);
  visual_loss_long_timeout_s_ =
    declare_parameter<double>("visual_loss_long_timeout_s", 2.0);
  estimator_process_acceleration_std_mps2_ = declare_parameter<double>(
    "target_state_estimator.process_acceleration_std_mps2", 1.0);
  estimator_measurement_horizontal_std_m_ = declare_parameter<double>(
    "target_state_estimator.measurement_horizontal_std_m", 0.08);
  estimator_measurement_vertical_std_m_ = declare_parameter<double>(
    "target_state_estimator.measurement_vertical_std_m", 0.12);
  estimator_initial_position_std_m_ = declare_parameter<double>(
    "target_state_estimator.initial_position_std_m", 0.20);
  estimator_initial_velocity_std_mps_ = declare_parameter<double>(
    "target_state_estimator.initial_velocity_std_mps", 1.0);
  estimator_minimum_sample_dt_s_ = declare_parameter<double>(
    "target_state_estimator.minimum_sample_dt_s", 0.001);
  estimator_maximum_sample_dt_s_ = declare_parameter<double>(
    "target_state_estimator.maximum_sample_dt_s", 0.50);
  estimator_reinitialize_gap_s_ = declare_parameter<double>(
    "target_state_estimator.reinitialize_gap_s", 2.0);
  estimator_innovation_gate_mahalanobis_ = declare_parameter<double>(
    "target_state_estimator.innovation_gate_mahalanobis", 5.0);
  vertical_state_estimator_enabled_ = declare_parameter<bool>(
    "vertical_state_estimator.enabled", true);
  vertical_process_acceleration_std_mps2_ = declare_parameter<double>(
    "vertical_state_estimator.process_acceleration_std_mps2", 0.40);
  vertical_measurement_std_m_ = declare_parameter<double>(
    "vertical_state_estimator.measurement_std_m", 0.05);
  vertical_measurement_bias_m_ = declare_parameter<double>(
    "vertical_state_estimator.measurement_bias_m", 0.0);
  vertical_initial_position_std_m_ = declare_parameter<double>(
    "vertical_state_estimator.initial_position_std_m", 0.10);
  vertical_initial_velocity_std_mps_ = declare_parameter<double>(
    "vertical_state_estimator.initial_velocity_std_mps", 0.50);
  vertical_minimum_sample_dt_s_ = declare_parameter<double>(
    "vertical_state_estimator.minimum_sample_dt_s", 0.001);
  vertical_maximum_sample_dt_s_ = declare_parameter<double>(
    "vertical_state_estimator.maximum_sample_dt_s", 0.25);
  vertical_reinitialize_gap_s_ = declare_parameter<double>(
    "vertical_state_estimator.reinitialize_gap_s", 2.0);
  vertical_innovation_gate_mahalanobis_ = declare_parameter<double>(
    "vertical_state_estimator.innovation_gate_mahalanobis", 5.0);
  vertical_prediction_horizon_s_ = declare_parameter<double>(
    "vertical_state_estimator.prediction_horizon_s", 0.10);
  vertical_velocity_feedforward_enabled_ = declare_parameter<bool>(
    "vertical_velocity_feedforward.enabled", true);
  vertical_velocity_feedforward_gain_ = declare_parameter<double>(
    "vertical_velocity_feedforward.deck_velocity_gain", 1.0);
  vertical_velocity_feedforward_max_mps_ = declare_parameter<double>(
    "vertical_velocity_feedforward.max_abs_mps", 0.60);
  touchdown_detector_enabled_ = declare_parameter<bool>(
    "touchdown_detector.enabled", true);
  touchdown_px4_status_timeout_s_ = declare_parameter<double>(
    "touchdown_detector.px4_status_timeout_s", 0.20);
  touchdown_visual_timeout_s_ = declare_parameter<double>(
    "touchdown_detector.visual_timeout_s", 0.20);
  touchdown_low_height_enter_m_ = declare_parameter<double>(
    "touchdown_detector.low_height_enter_m", 0.18);
  touchdown_low_height_exit_m_ = declare_parameter<double>(
    "touchdown_detector.low_height_exit_m", 0.28);
  touchdown_max_relative_vertical_speed_mps_ = declare_parameter<double>(
    "touchdown_detector.max_relative_vertical_speed_mps", 0.12);
  touchdown_max_uav_vertical_speed_mps_ = declare_parameter<double>(
    "touchdown_detector.max_uav_vertical_speed_mps", 0.15);
  touchdown_max_relative_horizontal_speed_mps_ = declare_parameter<double>(
    "touchdown_detector.max_relative_horizontal_speed_mps", 0.15);
  touchdown_candidate_required_duration_s_ = declare_parameter<double>(
    "touchdown_detector.candidate_required_duration_s", 0.50);
  final_descent_enabled_ = declare_parameter<bool>(
    "final_descent.enabled", false);
  final_descent_entry_height_m_ = declare_parameter<double>(
    "final_descent.entry_height_m", 0.50);
  final_descent_approach_rate_mps_ = declare_parameter<double>(
    "final_descent.approach_rate_mps", 0.12);
  final_descent_contact_rate_mps_ = declare_parameter<double>(
    "final_descent.contact_rate_mps", 0.03);
  final_descent_contact_slowdown_height_m_ = declare_parameter<double>(
    "final_descent.contact_slowdown_height_m", 0.25);
  final_descent_minimum_command_height_m_ = declare_parameter<double>(
    "final_descent.minimum_command_height_m", 0.15);
  final_descent_max_reference_tracking_error_m_ = declare_parameter<double>(
    "final_descent.max_reference_tracking_error_m", 0.20);
  additional_prediction_horizon_s_ = declare_parameter<double>(
    "motion_predictor.additional_prediction_horizon_s", 0.10);
  max_prediction_horizon_s_ = declare_parameter<double>(
    "motion_predictor.max_prediction_horizon_s", 0.50);
  estimator_output_timeout_s_ =
    declare_parameter<double>("estimator_output_timeout_s", 2.0);
  tracking_mode_string_ = declare_parameter<std::string>(
    "tracking.mode", "PREDICTED_POSITION_VELOCITY_FF");
  tracking_max_position_target_speed_mps_ = declare_parameter<double>(
    "tracking.max_position_target_speed_mps", 2.0);
  tracking_max_position_target_step_m_ = declare_parameter<double>(
    "tracking.max_position_target_step_m", 0.20);
  tracking_velocity_feedforward_gain_ = declare_parameter<double>(
    "tracking.velocity_feedforward_gain", 1.0);
  tracking_relative_velocity_gain_ = declare_parameter<double>(
    "tracking.relative_velocity_gain", 0.25);
  tracking_adaptive_relative_velocity_gain_enabled_ = declare_parameter<bool>(
    "tracking.adaptive_relative_velocity_gain.enabled", true);
  tracking_adaptive_relative_velocity_gain_min_ = declare_parameter<double>(
    "tracking.adaptive_relative_velocity_gain.min_gain", 0.25);
  tracking_adaptive_relative_velocity_gain_max_ = declare_parameter<double>(
    "tracking.adaptive_relative_velocity_gain.max_gain", 1.2);
  tracking_adaptive_acceleration_low_threshold_mps2_ = declare_parameter<double>(
    "tracking.adaptive_relative_velocity_gain.acceleration_low_threshold_mps2", 0.05);
  tracking_adaptive_acceleration_high_threshold_mps2_ = declare_parameter<double>(
    "tracking.adaptive_relative_velocity_gain.acceleration_high_threshold_mps2", 0.35);
  tracking_adaptive_max_acceleration_mps2_ = declare_parameter<double>(
    "tracking.adaptive_relative_velocity_gain.max_acceleration_mps2", 1.50);
  tracking_adaptive_acceleration_filter_gain_ = declare_parameter<double>(
    "tracking.adaptive_relative_velocity_gain.acceleration_filter_gain", 0.20);
  tracking_max_velocity_feedforward_mps_ = declare_parameter<double>(
    "tracking.max_velocity_feedforward_mps", 1.5);
  tracking_max_velocity_feedforward_acceleration_mps2_ = declare_parameter<double>(
    "tracking.max_velocity_feedforward_acceleration_mps2", 1.0);
  tracking_max_prediction_age_s_ = declare_parameter<double>(
    "tracking.max_prediction_age_s", 0.75);
  deck_attitude_filter_gain_ = declare_parameter<double>(
    "deck_attitude.filter_gain", 0.20);
  deck_attitude_minimum_upward_normal_component_ = declare_parameter<double>(
    "deck_attitude.minimum_upward_normal_component", 0.50);
  landing_window_enter_horizontal_error_m_ = declare_parameter<double>(
    "landing_window.enter_horizontal_error_m", 0.15);
  landing_window_exit_horizontal_error_m_ = declare_parameter<double>(
    "landing_window.exit_horizontal_error_m", 0.25);
  landing_window_enter_relative_speed_mps_ = declare_parameter<double>(
    "landing_window.enter_relative_speed_mps", 0.15);
  landing_window_exit_relative_speed_mps_ = declare_parameter<double>(
    "landing_window.exit_relative_speed_mps", 0.25);
  landing_window_enter_max_tilt_deg_ = declare_parameter<double>(
    "landing_window.enter_max_tilt_deg", 5.0);
  landing_window_exit_max_tilt_deg_ = declare_parameter<double>(
    "landing_window.exit_max_tilt_deg", 8.0);
  landing_window_max_visual_age_s_ = declare_parameter<double>(
    "landing_window.max_visual_age_s", 0.20);
  landing_window_minimum_relative_height_m_ = declare_parameter<double>(
    "landing_window.minimum_relative_height_m", 0.08);
  landing_window_maximum_relative_height_m_ = declare_parameter<double>(
    "landing_window.maximum_relative_height_m", 6.00);
  landing_window_required_duration_s_ = declare_parameter<double>(
    "landing_window.required_duration_s", 1.00);
  relative_descent_enabled_ = declare_parameter<bool>("descent.enabled", false);
  descent_minimum_test_height_m_ = declare_parameter<double>(
    "descent.minimum_test_height_m", 0.50);
  descent_fast_height_threshold_m_ = declare_parameter<double>(
    "descent.fast_height_threshold_m", 2.00);
  descent_slow_height_threshold_m_ = declare_parameter<double>(
    "descent.slow_height_threshold_m", 0.80);
  descent_fast_rate_mps_ = declare_parameter<double>("descent.fast_rate_mps", 0.50);
  descent_medium_rate_mps_ = declare_parameter<double>("descent.medium_rate_mps", 0.30);
  descent_slow_rate_mps_ = declare_parameter<double>("descent.slow_rate_mps", 0.12);
  descent_recovery_height_m_ = declare_parameter<double>(
    "descent.recovery_height_m", 2.00);
  descent_recovery_rate_mps_ = declare_parameter<double>(
    "descent.recovery_rate_mps", 0.30);
  descent_max_reference_tracking_error_m_ = declare_parameter<double>(
    "descent.max_reference_tracking_error_m", 0.50);
  vehicle_pose_history_duration_s_ = declare_parameter<double>(
    "vehicle_pose_history.history_duration_s", 2.0);
  vehicle_pose_history_max_endpoint_hold_s_ = declare_parameter<double>(
    "vehicle_pose_history.max_endpoint_hold_s", 0.03);
  px4_clock_offset_filter_gain_ = declare_parameter<double>(
    "vehicle_pose_history.clock_offset_filter_gain", 0.05);
  px4_clock_offset_max_jump_s_ = declare_parameter<double>(
    "vehicle_pose_history.max_clock_offset_jump_s", 0.10);
  offboard_prestream_count_ = declare_parameter<int>("offboard_prestream_count", 20);
  stable_detect_count_ = declare_parameter<int>("stable_detect_count", 10);
  camera_x_to_body_y_sign_ =
    declare_parameter<double>("camera_x_to_body_y_sign", 1.0);
  camera_y_to_body_x_sign_ =
    declare_parameter<double>("camera_y_to_body_x_sign", -1.0);
  max_xy_step_ = declare_parameter<double>("max_xy_step", 0.20);
  center_xy_threshold_ = declare_parameter<double>("center_xy_threshold", 0.15);
  max_descent_rate_ = declare_parameter<double>("max_descent_rate", 0.20);
  final_alt_ = declare_parameter<double>("final_alt", 0.30);
  marker_lost_timeout_ = declare_parameter<double>("marker_lost_timeout", 1.0);
  aruco_pose_timeout_ = declare_parameter<double>("aruco_pose_timeout", 0.5);
  takeoff_z_threshold_ = declare_parameter<double>("takeoff_z_threshold", 0.20);
  search_xy_threshold_ = declare_parameter<double>("search_xy_threshold", 0.25);
  search_z_threshold_ = declare_parameter<double>("search_z_threshold", 0.20);
  command_retry_interval_ = declare_parameter<double>("command_retry_interval", 1.0);
  enable_auto_land_ = declare_parameter<bool>("enable_auto_land", false);
  target_pose_frame_id_ =
    declare_parameter<std::string>("target_pose_frame_id", "local_ned");
  deck_gnss_velocity_frame_id_ =
    declare_parameter<std::string>("deck_gnss_velocity_frame_id", "world_enu");
  expected_aruco_pose_frame_id_ =
    declare_parameter<std::string>("expected_aruco_pose_frame_id", "camera_link");
  estimated_deck_child_frame_id_ = declare_parameter<std::string>(
    "estimated_deck_child_frame_id", "estimated_deck");
  camera_translation_frd_m_ = to_array<3>(
    declare_parameter<std::vector<double>>(
      "camera_extrinsic.translation_frd_m", {0.0, 0.0, 0.14}),
    "camera_extrinsic.translation_frd_m");
  camera_rotation_wxyz_ = to_array<4>(
    declare_parameter<std::vector<double>>(
      "camera_extrinsic.rotation_wxyz", {0.70710678, 0.0, 0.0, 0.70710678}),
    "camera_extrinsic.rotation_wxyz");
}

void Px4ArucoLandingNode::validate_parameters() const
{
  const auto require_positive = [this](const char * name, double value) {
      if (!is_positive_finite(value)) {
        throw std::invalid_argument(
                std::string("Parameter '") + name + "' must be finite and positive");
      }
    };

  require_positive("control_rate_hz", control_rate_hz_);
  require_positive("takeoff_alt", takeoff_alt_);
  require_positive("search_alt", search_alt_);
  require_positive("abort_hover_alt", abort_hover_alt_);
  require_positive("rendezvous_altitude_m", rendezvous_altitude_m_);
  require_positive("rendezvous_radius_m", rendezvous_radius_m_);
  require_positive("gnss_fix_timeout_s", gnss_fix_timeout_s_);
  require_positive("gnss_velocity_timeout_s", gnss_velocity_timeout_s_);
  require_positive("max_gnss_jump_m", max_gnss_jump_m_);
  require_positive("max_rendezvous_speed_mps", max_rendezvous_speed_mps_);
  require_positive("max_target_step_m", max_target_step_m_);
  require_positive("search_point_hold_s", search_point_hold_s_);
  require_positive("gnss_max_geodetic_range_m", gnss_max_geodetic_range_m_);
  require_positive("visual_handover_duration_s", visual_handover_duration_s_);
  require_positive(
    "handover_max_horizontal_difference_m", handover_max_horizontal_difference_m_);
  require_positive("max_visual_measurement_jump_m", max_visual_measurement_jump_m_);
  require_positive("visual_loss_short_timeout_s", visual_loss_short_timeout_s_);
  require_positive("visual_loss_long_timeout_s", visual_loss_long_timeout_s_);
  require_positive(
    "target_state_estimator.process_acceleration_std_mps2",
    estimator_process_acceleration_std_mps2_);
  require_positive(
    "target_state_estimator.measurement_horizontal_std_m",
    estimator_measurement_horizontal_std_m_);
  require_positive(
    "target_state_estimator.measurement_vertical_std_m",
    estimator_measurement_vertical_std_m_);
  require_positive(
    "target_state_estimator.initial_position_std_m",
    estimator_initial_position_std_m_);
  require_positive(
    "target_state_estimator.initial_velocity_std_mps",
    estimator_initial_velocity_std_mps_);
  require_positive(
    "target_state_estimator.minimum_sample_dt_s",
    estimator_minimum_sample_dt_s_);
  require_positive(
    "target_state_estimator.maximum_sample_dt_s",
    estimator_maximum_sample_dt_s_);
  require_positive(
    "target_state_estimator.reinitialize_gap_s",
    estimator_reinitialize_gap_s_);
  require_positive(
    "target_state_estimator.innovation_gate_mahalanobis",
    estimator_innovation_gate_mahalanobis_);
  require_positive(
    "vertical_state_estimator.process_acceleration_std_mps2",
    vertical_process_acceleration_std_mps2_);
  require_positive(
    "vertical_state_estimator.measurement_std_m", vertical_measurement_std_m_);
  require_positive(
    "vertical_state_estimator.initial_position_std_m",
    vertical_initial_position_std_m_);
  require_positive(
    "vertical_state_estimator.initial_velocity_std_mps",
    vertical_initial_velocity_std_mps_);
  require_positive(
    "vertical_state_estimator.minimum_sample_dt_s",
    vertical_minimum_sample_dt_s_);
  require_positive(
    "vertical_state_estimator.maximum_sample_dt_s",
    vertical_maximum_sample_dt_s_);
  require_positive(
    "vertical_state_estimator.reinitialize_gap_s",
    vertical_reinitialize_gap_s_);
  require_positive(
    "vertical_state_estimator.innovation_gate_mahalanobis",
    vertical_innovation_gate_mahalanobis_);
  require_positive("motion_predictor.max_prediction_horizon_s", max_prediction_horizon_s_);
  require_positive("estimator_output_timeout_s", estimator_output_timeout_s_);
  require_positive(
    "tracking.max_position_target_speed_mps",
    tracking_max_position_target_speed_mps_);
  require_positive(
    "tracking.max_position_target_step_m",
    tracking_max_position_target_step_m_);
  require_positive(
    "tracking.max_velocity_feedforward_mps",
    tracking_max_velocity_feedforward_mps_);
  require_positive(
    "tracking.max_velocity_feedforward_acceleration_mps2",
    tracking_max_velocity_feedforward_acceleration_mps2_);
  require_positive("tracking.max_prediction_age_s", tracking_max_prediction_age_s_);
  require_positive(
    "vehicle_pose_history.history_duration_s", vehicle_pose_history_duration_s_);
  require_positive(
    "vehicle_pose_history.max_clock_offset_jump_s", px4_clock_offset_max_jump_s_);
  require_positive("max_xy_step", max_xy_step_);
  require_positive("center_xy_threshold", center_xy_threshold_);
  require_positive("max_descent_rate", max_descent_rate_);
  require_positive("final_alt", final_alt_);
  require_positive("marker_lost_timeout", marker_lost_timeout_);
  require_positive("aruco_pose_timeout", aruco_pose_timeout_);
  require_positive("takeoff_z_threshold", takeoff_z_threshold_);
  require_positive("search_xy_threshold", search_xy_threshold_);
  require_positive("search_z_threshold", search_z_threshold_);
  require_positive("command_retry_interval", command_retry_interval_);

  if (!std::isfinite(search_x_) || !std::isfinite(search_y_)) {
    throw std::invalid_argument("Parameters 'search_x' and 'search_y' must be finite");
  }
  if (!std::isfinite(gnss_stable_duration_s_) || gnss_stable_duration_s_ < 0.0) {
    throw std::invalid_argument(
            "Parameter 'gnss_stable_duration_s' must be finite and non-negative");
  }
  if (!std::isfinite(search_offset_m_) || search_offset_m_ < 0.0) {
    throw std::invalid_argument(
            "Parameter 'search_offset_m' must be finite and non-negative");
  }
  if (!std::isfinite(aruco_acquire_duration_s_) || aruco_acquire_duration_s_ < 0.0) {
    throw std::invalid_argument(
            "Parameter 'aruco_acquire_duration_s' must be finite and non-negative");
  }
  if (offboard_prestream_count_ < 1) {
    throw std::invalid_argument("Parameter 'offboard_prestream_count' must be at least 1");
  }
  if (stable_detect_count_ < 1) {
    throw std::invalid_argument("Parameter 'stable_detect_count' must be at least 1");
  }
  if (std::abs(std::abs(camera_x_to_body_y_sign_) - 1.0) > 1.0e-6 ||
    std::abs(std::abs(camera_y_to_body_x_sign_) - 1.0) > 1.0e-6)
  {
    throw std::invalid_argument(
            "Camera-to-body sign parameters must each be either -1.0 or 1.0");
  }
  if (final_alt_ >= takeoff_alt_ || final_alt_ >= search_alt_) {
    throw std::invalid_argument(
            "Parameter 'final_alt' must be lower than takeoff_alt and search_alt");
  }
  if (visual_loss_long_timeout_s_ <= visual_loss_short_timeout_s_) {
    throw std::invalid_argument(
            "visual_loss_long_timeout_s must be greater than visual_loss_short_timeout_s");
  }
  if (estimator_maximum_sample_dt_s_ < estimator_minimum_sample_dt_s_) {
    throw std::invalid_argument(
            "target_state_estimator.maximum_sample_dt_s must not be smaller than minimum_sample_dt_s");
  }
  if (estimator_reinitialize_gap_s_ <= estimator_maximum_sample_dt_s_) {
    throw std::invalid_argument(
            "target_state_estimator.reinitialize_gap_s must be greater than maximum_sample_dt_s");
  }
  if (!std::isfinite(additional_prediction_horizon_s_) ||
    additional_prediction_horizon_s_ < 0.0 ||
    additional_prediction_horizon_s_ > max_prediction_horizon_s_)
  {
    throw std::invalid_argument(
            "motion_predictor.additional_prediction_horizon_s must be finite and within the maximum horizon");
  }
  if (!std::isfinite(tracking_velocity_feedforward_gain_) ||
    tracking_velocity_feedforward_gain_ < 0.0 ||
    !std::isfinite(tracking_relative_velocity_gain_) ||
    tracking_relative_velocity_gain_ < 0.0)
  {
    throw std::invalid_argument("Tracking velocity gains must be finite and non-negative");
  }
  AdaptiveRelativeVelocityGainParameters adaptive_gain_parameters;
  adaptive_gain_parameters.min_gain = tracking_adaptive_relative_velocity_gain_min_;
  adaptive_gain_parameters.max_gain = tracking_adaptive_relative_velocity_gain_max_;
  adaptive_gain_parameters.acceleration_low_threshold_mps2 =
    tracking_adaptive_acceleration_low_threshold_mps2_;
  adaptive_gain_parameters.acceleration_high_threshold_mps2 =
    tracking_adaptive_acceleration_high_threshold_mps2_;
  adaptive_gain_parameters.max_acceleration_mps2 =
    tracking_adaptive_max_acceleration_mps2_;
  adaptive_gain_parameters.acceleration_filter_gain =
    tracking_adaptive_acceleration_filter_gain_;
  static_cast<void>(AdaptiveRelativeVelocityGain(adaptive_gain_parameters));

  DeckAttitudeEstimatorParameters attitude_parameters;
  attitude_parameters.filter_gain = deck_attitude_filter_gain_;
  attitude_parameters.minimum_upward_normal_component =
    deck_attitude_minimum_upward_normal_component_;
  static_cast<void>(DeckAttitudeEstimator(attitude_parameters));

  LandingWindowParameters landing_window_parameters;
  landing_window_parameters.enter_horizontal_error_m =
    landing_window_enter_horizontal_error_m_;
  landing_window_parameters.exit_horizontal_error_m =
    landing_window_exit_horizontal_error_m_;
  landing_window_parameters.enter_relative_speed_mps =
    landing_window_enter_relative_speed_mps_;
  landing_window_parameters.exit_relative_speed_mps =
    landing_window_exit_relative_speed_mps_;
  landing_window_parameters.enter_max_tilt_rad =
    landing_window_enter_max_tilt_deg_ * kDegreesToRadians;
  landing_window_parameters.exit_max_tilt_rad =
    landing_window_exit_max_tilt_deg_ * kDegreesToRadians;
  landing_window_parameters.max_visual_age_s = landing_window_max_visual_age_s_;
  landing_window_parameters.minimum_relative_height_m =
    landing_window_minimum_relative_height_m_;
  landing_window_parameters.maximum_relative_height_m =
    landing_window_maximum_relative_height_m_;
  landing_window_parameters.required_duration_s = landing_window_required_duration_s_;
  static_cast<void>(LandingWindow(landing_window_parameters));

  VerticalStateEstimatorParameters vertical_parameters;
  vertical_parameters.process_acceleration_std_mps2 =
    vertical_process_acceleration_std_mps2_;
  vertical_parameters.measurement_std_m = vertical_measurement_std_m_;
  vertical_parameters.measurement_bias_m = vertical_measurement_bias_m_;
  vertical_parameters.initial_position_std_m = vertical_initial_position_std_m_;
  vertical_parameters.initial_velocity_std_mps = vertical_initial_velocity_std_mps_;
  vertical_parameters.minimum_sample_dt_s = vertical_minimum_sample_dt_s_;
  vertical_parameters.maximum_sample_dt_s = vertical_maximum_sample_dt_s_;
  vertical_parameters.reinitialize_gap_s = vertical_reinitialize_gap_s_;
  vertical_parameters.innovation_gate_mahalanobis =
    vertical_innovation_gate_mahalanobis_;
  static_cast<void>(VerticalStateEstimator(vertical_parameters));
  if (!std::isfinite(vertical_prediction_horizon_s_) ||
    vertical_prediction_horizon_s_ < 0.0 || vertical_prediction_horizon_s_ > 0.50)
  {
    throw std::invalid_argument(
            "vertical_state_estimator.prediction_horizon_s must be within [0, 0.50]");
  }
  if (!std::isfinite(vertical_velocity_feedforward_gain_) ||
    vertical_velocity_feedforward_gain_ < 0.0)
  {
    throw std::invalid_argument(
            "vertical_velocity_feedforward.deck_velocity_gain must be finite and non-negative");
  }
  require_positive(
    "vertical_velocity_feedforward.max_abs_mps",
    vertical_velocity_feedforward_max_mps_);

  TouchdownDetectorParameters touchdown_parameters;
  touchdown_parameters.px4_status_timeout_s = touchdown_px4_status_timeout_s_;
  touchdown_parameters.visual_timeout_s = touchdown_visual_timeout_s_;
  touchdown_parameters.low_height_enter_m = touchdown_low_height_enter_m_;
  touchdown_parameters.low_height_exit_m = touchdown_low_height_exit_m_;
  touchdown_parameters.max_relative_vertical_speed_mps =
    touchdown_max_relative_vertical_speed_mps_;
  touchdown_parameters.max_uav_vertical_speed_mps =
    touchdown_max_uav_vertical_speed_mps_;
  touchdown_parameters.max_relative_horizontal_speed_mps =
    touchdown_max_relative_horizontal_speed_mps_;
  touchdown_parameters.candidate_required_duration_s =
    touchdown_candidate_required_duration_s_;
  static_cast<void>(TouchdownDetector(touchdown_parameters));

  FinalDescentParameters final_descent_parameters;
  final_descent_parameters.entry_height_m = final_descent_entry_height_m_;
  final_descent_parameters.approach_rate_mps = final_descent_approach_rate_mps_;
  final_descent_parameters.contact_rate_mps = final_descent_contact_rate_mps_;
  final_descent_parameters.contact_slowdown_height_m =
    final_descent_contact_slowdown_height_m_;
  final_descent_parameters.minimum_command_height_m =
    final_descent_minimum_command_height_m_;
  final_descent_parameters.maximum_reference_tracking_error_m =
    final_descent_max_reference_tracking_error_m_;
  static_cast<void>(FinalDescentController(final_descent_parameters));
  if (final_descent_enabled_ && !relative_descent_enabled_) {
    throw std::invalid_argument(
            "final_descent.enabled requires descent.enabled to be true");
  }

  RelativeDescentParameters descent_parameters;
  descent_parameters.minimum_test_height_m = descent_minimum_test_height_m_;
  descent_parameters.fast_height_threshold_m = descent_fast_height_threshold_m_;
  descent_parameters.slow_height_threshold_m = descent_slow_height_threshold_m_;
  descent_parameters.fast_rate_mps = descent_fast_rate_mps_;
  descent_parameters.medium_rate_mps = descent_medium_rate_mps_;
  descent_parameters.slow_rate_mps = descent_slow_rate_mps_;
  descent_parameters.recovery_height_m = descent_recovery_height_m_;
  descent_parameters.recovery_rate_mps = descent_recovery_rate_mps_;
  descent_parameters.max_reference_tracking_error_m =
    descent_max_reference_tracking_error_m_;
  static_cast<void>(RelativeDescentController(descent_parameters));

  if (!std::isfinite(vehicle_pose_history_max_endpoint_hold_s_) ||
    vehicle_pose_history_max_endpoint_hold_s_ < 0.0 ||
    vehicle_pose_history_max_endpoint_hold_s_ > vehicle_pose_history_duration_s_)
  {
    throw std::invalid_argument(
            "vehicle_pose_history.max_endpoint_hold_s must be finite, non-negative, and not exceed history_duration_s");
  }
  if (!std::isfinite(px4_clock_offset_filter_gain_) ||
    px4_clock_offset_filter_gain_ <= 0.0 || px4_clock_offset_filter_gain_ > 1.0)
  {
    throw std::invalid_argument(
            "vehicle_pose_history.clock_offset_filter_gain must be within (0, 1]");
  }
  if (!tracking_control_mode_from_string(tracking_mode_string_).has_value()) {
    throw std::invalid_argument("Parameter 'tracking.mode' is unsupported");
  }
  if (tracking_max_prediction_age_s_ > visual_loss_long_timeout_s_) {
    throw std::invalid_argument(
            "tracking.max_prediction_age_s must not exceed visual_loss_long_timeout_s");
  }
  if (target_pose_frame_id_.empty()) {
    throw std::invalid_argument("Parameter 'target_pose_frame_id' must not be empty");
  }
  if (expected_aruco_pose_frame_id_.empty()) {
    throw std::invalid_argument(
            "Parameter 'expected_aruco_pose_frame_id' must not be empty");
  }
  if (estimated_deck_child_frame_id_.empty()) {
    throw std::invalid_argument(
            "Parameter 'estimated_deck_child_frame_id' must not be empty");
  }
  if (deck_gnss_velocity_frame_id_.empty()) {
    throw std::invalid_argument(
            "Parameter 'deck_gnss_velocity_frame_id' must not be empty");
  }

  const Pose3d body_camera_pose{
    Eigen::Vector3d{
      camera_translation_frd_m_[0],
      camera_translation_frd_m_[1],
      camera_translation_frd_m_[2]},
    Eigen::Quaterniond{
      camera_rotation_wxyz_[0],
      camera_rotation_wxyz_[1],
      camera_rotation_wxyz_[2],
      camera_rotation_wxyz_[3]}};
  if (!make_isometry(body_camera_pose.translation, body_camera_pose.rotation).has_value()) {
    throw std::invalid_argument("Camera extrinsic contains invalid translation or quaternion");
  }
}

void Px4ArucoLandingNode::create_ros_interfaces()
{
  const auto aruco_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  const auto deck_gnss_qos = rclcpp::SensorDataQoS();
  const auto px4_qos =
    rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();

  aruco_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/aruco/pose",
    aruco_qos,
    std::bind(
      &Px4ArucoLandingNode::aruco_pose_callback,
      this,
      std::placeholders::_1));
  aruco_visible_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/aruco/visible",
    aruco_qos,
    std::bind(
      &Px4ArucoLandingNode::aruco_visible_callback,
      this,
      std::placeholders::_1));
  aruco_id_sub_ = create_subscription<std_msgs::msg::Int32>(
    "/aruco/id",
    aruco_qos,
    std::bind(
      &Px4ArucoLandingNode::aruco_id_callback,
      this,
      std::placeholders::_1));
  deck_gps_fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    "/deck/gps/fix",
    deck_gnss_qos,
    std::bind(
      &Px4ArucoLandingNode::deck_gps_fix_callback,
      this,
      std::placeholders::_1));
  deck_gps_velocity_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "/deck/gps/velocity",
    deck_gnss_qos,
    std::bind(
      &Px4ArucoLandingNode::deck_gps_velocity_callback,
      this,
      std::placeholders::_1));
  vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status",
    px4_qos,
    std::bind(
      &Px4ArucoLandingNode::vehicle_status_callback,
      this,
      std::placeholders::_1));
  vehicle_land_detected_sub_ =
    create_subscription<px4_msgs::msg::VehicleLandDetected>(
    "/fmu/out/vehicle_land_detected",
    px4_qos,
    std::bind(
      &Px4ArucoLandingNode::vehicle_land_detected_callback,
      this,
      std::placeholders::_1));
  vehicle_local_position_sub_ =
    create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    "/fmu/out/vehicle_local_position",
    px4_qos,
    std::bind(
      &Px4ArucoLandingNode::vehicle_local_position_callback,
      this,
      std::placeholders::_1));
  vehicle_odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
    "/fmu/out/vehicle_odometry",
    px4_qos,
    std::bind(
      &Px4ArucoLandingNode::vehicle_odometry_callback,
      this,
      std::placeholders::_1));

  offboard_control_mode_pub_ =
    create_publisher<px4_msgs::msg::OffboardControlMode>(
    "/fmu/in/offboard_control_mode",
    px4_qos);
  trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    "/fmu/in/trajectory_setpoint",
    px4_qos);
  vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command",
    px4_qos);
  landing_state_pub_ = create_publisher<std_msgs::msg::String>(
    "/landing/state",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  target_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "/landing/target_pose",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  deck_gnss_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "/landing/deck_gnss_pose_ned",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  marker_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "/landing/marker_pose_ned",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  active_marker_id_pub_ = create_publisher<std_msgs::msg::Int32>(
    "/landing/active_marker_id",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  estimated_deck_odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(
    "/landing/estimated_deck_odometry",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  predicted_deck_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "/landing/predicted_deck_pose",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  tracking_velocity_setpoint_pub_ =
    create_publisher<geometry_msgs::msg::TwistStamped>(
    "/landing/tracking_velocity_setpoint",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  effective_relative_velocity_gain_pub_ = create_publisher<std_msgs::msg::Float64>(
    "/landing/effective_relative_velocity_gain",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  estimated_deck_acceleration_pub_ =
    create_publisher<geometry_msgs::msg::TwistStamped>(
    "/landing/estimated_deck_acceleration",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  estimated_deck_attitude_pub_ =
    create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "/landing/estimated_deck_attitude",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  landing_window_open_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/landing/window_open",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  landing_window_reject_reasons_pub_ = create_publisher<std_msgs::msg::UInt32>(
    "/landing/window_reject_reasons",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  landing_window_satisfied_duration_pub_ = create_publisher<std_msgs::msg::Float64>(
    "/landing/window_satisfied_duration",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  relative_height_pub_ = create_publisher<std_msgs::msg::Float64>(
    "/landing/relative_height",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  relative_height_reference_pub_ = create_publisher<std_msgs::msg::Float64>(
    "/landing/relative_height_reference",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  descent_phase_pub_ = create_publisher<std_msgs::msg::String>(
    "/landing/descent_phase",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  vertical_state_pub_ = create_publisher<nav_msgs::msg::Odometry>(
    "/landing/vertical_state",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  raw_relative_height_pub_ = create_publisher<std_msgs::msg::Float64>(
    "/landing/raw_relative_height",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  relative_vertical_velocity_pub_ = create_publisher<std_msgs::msg::Float64>(
    "/landing/relative_vertical_velocity",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  touchdown_status_pub_ = create_publisher<std_msgs::msg::String>(
    "/landing/touchdown_status",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  touchdown_evidence_pub_ = create_publisher<std_msgs::msg::UInt32>(
    "/landing/touchdown_evidence",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  touchdown_candidate_duration_pub_ = create_publisher<std_msgs::msg::Float64>(
    "/landing/touchdown_candidate_duration",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  touchdown_confirmed_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/landing/touchdown_confirmed",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  final_descent_phase_pub_ = create_publisher<std_msgs::msg::String>(
    "/landing/final_descent_phase",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  guidance_source_pub_ = create_publisher<std_msgs::msg::String>(
    "/landing/guidance_source",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
}

void Px4ArucoLandingNode::aruco_pose_callback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  const rclcpp::Time receipt_time = get_clock()->now();

  if (msg->header.frame_id != expected_aruco_pose_frame_id_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Rejected ArUco frame '%s'; expected '%s' with camera_optical numeric semantics",
      msg->header.frame_id.c_str(), expected_aruco_pose_frame_id_.c_str());
    return;
  }

  const int64_t sample_stamp_ns =
    static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
    static_cast<int64_t>(msg->header.stamp.nanosec);
  if (sample_stamp_ns <= 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Rejected ArUco pose because the image sample timestamp is invalid");
    return;
  }
  if (have_last_aruco_sample_stamp_ && sample_stamp_ns <= last_aruco_sample_stamp_ns_)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Rejected repeated or out-of-order ArUco sample");
    return;
  }

  const double sample_time_s = static_cast<double>(sample_stamp_ns) * 1.0e-9;
  aruco_pose_ = *msg;
  have_aruco_pose_ = true;

  Pose3d marker_pose_ned;
  if (!compute_marker_pose_ned(sample_time_s, marker_pose_ned)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Rejected ArUco pose because full camera-to-NED transform failed");
    return;
  }

  const bool visual_state =
    state_ == LandingState::ACQUIRE_ARUCO ||
    state_ == LandingState::VISUAL_HANDOVER ||
    state_ == LandingState::TRACK_TARGET ||
    state_ == LandingState::WAIT_LANDING_WINDOW ||
    state_ == LandingState::DESCEND ||
    state_ == LandingState::TEST_HEIGHT_HOLD ||
    state_ == LandingState::FINAL_DESCENT ||
    state_ == LandingState::TOUCHDOWN_CANDIDATE_HOLD ||
    state_ == LandingState::TOUCHDOWN_HOLD ||
    state_ == LandingState::RECOVER_CLIMB;
  if (!visual_state) {
    return;
  }

  if (state_ != LandingState::TRACK_TARGET &&
    state_ != LandingState::WAIT_LANDING_WINDOW &&
    state_ != LandingState::DESCEND &&
    state_ != LandingState::TEST_HEIGHT_HOLD &&
    state_ != LandingState::FINAL_DESCENT &&
    state_ != LandingState::TOUCHDOWN_CANDIDATE_HOLD &&
    state_ != LandingState::TOUCHDOWN_HOLD &&
    state_ != LandingState::RECOVER_CLIMB)
  {
    const auto gnss_estimate = gnss_guidance_->estimate(receipt_time.seconds());
    if (!gnss_estimate.has_value() ||
      !visual_guidance_->consistent_with_gnss(
        marker_pose_ned.translation, gnss_estimate->position_ned))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Rejected visual candidate inconsistent with deck GNSS during handover");
      return;
    }
  }

  if (!visual_guidance_->update_visual_position(
      marker_pose_ned.translation, receipt_time.seconds()))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Rejected visual candidate due to invalid time or excessive measurement jump");
    return;
  }

  update_estimated_deck_attitude(marker_pose_ned.rotation, receipt_time);

  const TargetStateUpdateResult estimator_result =
    target_state_estimator_->update(marker_pose_ned.translation, sample_time_s);
  switch (estimator_result.status) {
    case TargetStateUpdateStatus::kInitialized:
    case TargetStateUpdateStatus::kUpdated:
    case TargetStateUpdateStatus::kReinitialized:
      last_estimator_measurement_receipt_time_s_ = receipt_time.seconds();
      last_estimator_state_receipt_time_s_ = receipt_time.seconds();
      have_estimator_measurement_receipt_time_ = true;
      break;
    case TargetStateUpdateStatus::kRejectedOutlier:
      last_estimator_state_receipt_time_s_ = receipt_time.seconds();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Rejected visual state-estimator outlier with NIS %.3f",
        estimator_result.normalized_innovation_squared);
      break;
    case TargetStateUpdateStatus::kRejectedInvalidInput:
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Rejected invalid visual state-estimator input");
      break;
    case TargetStateUpdateStatus::kRejectedNonMonotonicTime:
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Rejected non-monotonic visual state-estimator sample time");
      break;
  }

  if (vertical_state_estimator_enabled_) {
    const VerticalStateUpdateResult vertical_result =
      vertical_state_estimator_->update(marker_pose_ned.translation.z(), sample_time_s);
    switch (vertical_result.status) {
      case VerticalStateUpdateStatus::kInitialized:
      case VerticalStateUpdateStatus::kUpdated:
      case VerticalStateUpdateStatus::kReinitialized:
        last_vertical_state_measurement_receipt_time_s_ = receipt_time.seconds();
        vertical_state_measurement_valid_ = true;
        break;
      case VerticalStateUpdateStatus::kRejectedOutlier:
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Rejected vertical state-estimator outlier with NIS %.3f",
          vertical_result.normalized_innovation_squared);
        break;
      case VerticalStateUpdateStatus::kRejectedInvalidInput:
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Rejected invalid vertical state-estimator input");
        break;
      case VerticalStateUpdateStatus::kRejectedNonMonotonicTime:
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Rejected non-monotonic vertical state-estimator sample time");
        break;
    }
  }

  const auto sample_body_pose = vehicle_pose_history_->lookup(sample_time_s);
  if (sample_body_pose.has_value()) {
    raw_relative_height_m_ =
      marker_pose_ned.translation.z() - sample_body_pose->translation.z();
    raw_relative_height_valid_ = std::isfinite(raw_relative_height_m_);
  } else {
    raw_relative_height_valid_ = false;
  }

  marker_pose_ned_.header.stamp = receipt_time;
  marker_pose_ned_.header.frame_id = target_pose_frame_id_;
  marker_pose_ned_.pose.position.x = marker_pose_ned.translation.x();
  marker_pose_ned_.pose.position.y = marker_pose_ned.translation.y();
  marker_pose_ned_.pose.position.z = marker_pose_ned.translation.z();
  marker_pose_ned_.pose.orientation.w = marker_pose_ned.rotation.w();
  marker_pose_ned_.pose.orientation.x = marker_pose_ned.rotation.x();
  marker_pose_ned_.pose.orientation.y = marker_pose_ned.rotation.y();
  marker_pose_ned_.pose.orientation.z = marker_pose_ned.rotation.z();
  have_marker_pose_ned_ = true;
  last_aruco_pose_time_ = receipt_time;

  last_aruco_sample_stamp_ns_ = sample_stamp_ns;
  have_last_aruco_sample_stamp_ = true;
}

void Px4ArucoLandingNode::aruco_visible_callback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  const bool was_visible = aruco_visible_;
  aruco_visible_ = msg->data;
  last_aruco_visible_time_ = get_clock()->now();

  if (aruco_visible_) {
    if (!was_visible || !have_aruco_visible_since_) {
      aruco_visible_since_ = last_aruco_visible_time_;
      have_aruco_visible_since_ = true;
    }
    if (stable_visible_count_ < stable_detect_count_) {
      ++stable_visible_count_;
    }
  } else {
    stable_visible_count_ = 0;
    have_aruco_visible_since_ = false;
  }
}

void Px4ArucoLandingNode::aruco_id_callback(
  const std_msgs::msg::Int32::SharedPtr msg)
{
  aruco_id_ = msg->data;
  have_aruco_id_ = true;
}

void Px4ArucoLandingNode::deck_gps_fix_callback(
  const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
    return;
  }

  const Wgs84Position fix{msg->latitude, msg->longitude, msg->altitude};
  if (!gnss_guidance_->ingest_fix(fix, get_clock()->now().seconds())) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Rejected deck GNSS fix: invalid, out of range, or excessive jump");
  }
}

void Px4ArucoLandingNode::deck_gps_velocity_callback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  if (!msg->header.frame_id.empty() &&
    msg->header.frame_id != deck_gnss_velocity_frame_id_)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Rejected deck GNSS velocity frame '%s'; expected '%s'",
      msg->header.frame_id.c_str(), deck_gnss_velocity_frame_id_.c_str());
    return;
  }

  const Eigen::Vector3d velocity_enu{
    msg->twist.linear.x,
    msg->twist.linear.y,
    msg->twist.linear.z};
  if (!gnss_guidance_->ingest_velocity_enu(
      velocity_enu, get_clock()->now().seconds()))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Rejected invalid deck GNSS velocity");
  }
}

void Px4ArucoLandingNode::vehicle_status_callback(
  const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  vehicle_status_ = *msg;
  have_vehicle_status_ = true;
}

void Px4ArucoLandingNode::vehicle_land_detected_callback(
  const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
  vehicle_land_detected_ = *msg;
  have_vehicle_land_detected_ = true;
  last_vehicle_land_detected_receipt_time_s_ = get_clock()->now().seconds();
}

void Px4ArucoLandingNode::vehicle_local_position_callback(
  const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  local_position_ = *msg;
  have_local_position_ = true;

  if (local_position_.xy_global && local_position_.z_global &&
    std::isfinite(local_position_.ref_lat) &&
    std::isfinite(local_position_.ref_lon) &&
    std::isfinite(local_position_.ref_alt))
  {
    const Wgs84Position reference{
      local_position_.ref_lat,
      local_position_.ref_lon,
      local_position_.ref_alt};
    if (!gnss_guidance_->set_local_reference(reference)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Rejected invalid PX4 local geodetic reference");
    }
  }
}

void Px4ArucoLandingNode::vehicle_odometry_callback(
  const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  const rclcpp::Time receipt_time = get_clock()->now();
  vehicle_odometry_ = *msg;
  have_vehicle_odometry_ = true;

  const bool position_valid =
    std::isfinite(vehicle_odometry_.position[0]) &&
    std::isfinite(vehicle_odometry_.position[1]) &&
    std::isfinite(vehicle_odometry_.position[2]);
  if (vehicle_odometry_.pose_frame !=
      px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED ||
    !position_valid || !quaternion_is_valid(vehicle_odometry_.q.data()))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Rejected PX4 odometry sample with invalid NED pose");
    return;
  }

  current_yaw_ = quaternion_to_yaw(vehicle_odometry_.q.data());
  const auto sample_time_s = update_px4_to_ros_time_offset(*msg, receipt_time);
  if (!sample_time_s.has_value()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Rejected PX4 odometry sample because timestamp synchronization failed");
    return;
  }

  const Pose3d local_body_pose{
    Eigen::Vector3d{
      vehicle_odometry_.position[0],
      vehicle_odometry_.position[1],
      vehicle_odometry_.position[2]},
    Eigen::Quaterniond{
      vehicle_odometry_.q[0],
      vehicle_odometry_.q[1],
      vehicle_odometry_.q[2],
      vehicle_odometry_.q[3]}};
  if (!vehicle_pose_history_->add_sample(local_body_pose, *sample_time_s)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Rejected PX4 odometry sample because pose history insertion failed");
  }
}

void Px4ArucoLandingNode::control_timer_callback()
{
  const auto now = get_clock()->now();
  double dt = (now - last_control_time_).seconds();
  last_control_time_ = now;

  if (!std::isfinite(dt) || dt <= 0.0) {
    dt = 1.0 / control_rate_hz_;
  }
  dt = std::min(dt, 0.5);

  clear_velocity_feedforward();
  landing_window_result_valid_ = false;
  relative_descent_debug_valid_ = false;
  final_descent_debug_valid_ = false;
  publish_offboard_control_mode();
  run_state_machine(now, dt);
  update_touchdown_detection(now);
  publish_trajectory_setpoint();
  publish_landing_state();
  publish_target_pose();
  publish_deck_gnss_pose(now);
  publish_marker_pose(now);
  publish_active_marker_id();
  publish_estimated_deck_odometry(now);
  publish_predicted_deck_pose(now);
  publish_tracking_velocity_setpoint(now);
  publish_effective_relative_velocity_gain();
  publish_estimated_deck_acceleration(now);
  publish_estimated_deck_attitude();
  publish_landing_window_debug();
  publish_relative_descent_debug();
  publish_vertical_state(now);
  publish_raw_relative_height(now);
  publish_relative_vertical_velocity(now);
  publish_final_descent_debug();
  publish_touchdown_debug();
  publish_guidance_source();
}

void Px4ArucoLandingNode::run_state_machine(const rclcpp::Time & now, double dt)
{
  switch (state_) {
    case LandingState::INIT:
      transition_to(LandingState::WAIT_FOR_PX4, "controller initialized");
      break;

    case LandingState::WAIT_FOR_PX4:
      if (px4_data_ready()) {
        takeoff_start_x_ = local_position_.x;
        takeoff_start_y_ = local_position_.y;
        initial_yaw_ = current_yaw_;
        transition_to(
          LandingState::OFFBOARD_PRE_STREAM,
          "PX4 status, local position, and attitude are valid");
      }
      break;

    case LandingState::OFFBOARD_PRE_STREAM:
      if (prestream_setpoint_count_ >= offboard_prestream_count_) {
        publish_vehicle_command(
          px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
          1.0F,
          6.0F);
        publish_vehicle_command(
          px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
          1.0F);
        last_command_time_ = now;
        have_last_command_time_ = true;
        transition_to(
          LandingState::ARM_AND_TAKEOFF,
          "offboard pre-stream complete; mode and arm commands sent");
      } else {
        ++prestream_setpoint_count_;
      }
      break;

    case LandingState::ARM_AND_TAKEOFF:
      {
        const bool in_offboard =
          vehicle_status_.nav_state ==
          px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
        const bool armed =
          vehicle_status_.arming_state ==
          px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;

        if ((!in_offboard || !armed) && should_retry_command(now)) {
          if (!in_offboard) {
            publish_vehicle_command(
              px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
              1.0F,
              6.0F);
          }
          if (!armed) {
            publish_vehicle_command(
              px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
              1.0F);
          }
          last_command_time_ = now;
          have_last_command_time_ = true;
        }

        if (in_offboard && armed &&
          std::abs(local_position_.z + takeoff_alt_) <= takeoff_z_threshold_)
        {
          transition_to(
            LandingState::WAIT_DECK_GNSS,
            "takeoff complete; wait for stable deck GNSS");
        }
        break;
      }

    case LandingState::WAIT_DECK_GNSS:
      if (gnss_guidance_->ready(now.seconds())) {
        transition_to(
          LandingState::RENDEZVOUS_GNSS,
          "deck GNSS and PX4 geodetic reference are stable");
      }
      break;

    case LandingState::RENDEZVOUS_GNSS:
      {
        if (!gnss_guidance_->ready(now.seconds())) {
          transition_to(
            LandingState::WAIT_DECK_GNSS,
            "deck GNSS became stale or unstable during rendezvous");
          break;
        }

        const auto estimate = gnss_guidance_->estimate(now.seconds());
        if (!estimate.has_value()) {
          transition_to(
            LandingState::WAIT_DECK_GNSS,
            "deck GNSS estimate unavailable during rendezvous");
          break;
        }

        const Eigen::Vector2d current_target = target_valid_ ?
          Eigen::Vector2d{target_x_, target_y_} :
          Eigen::Vector2d{local_position_.x, local_position_.y};
        const Eigen::Vector2d desired_target = estimate->position_ned.head<2>();
        const auto limited_target = gnss_guidance_->limit_target_xy(
          current_target, desired_target, dt);
        if (!limited_target.has_value()) {
          transition_to(LandingState::ABORT, "invalid GNSS rendezvous target");
          break;
        }

        set_target(
          limited_target->x(),
          limited_target->y(),
          -rendezvous_altitude_m_,
          current_yaw_);

        const double horizontal_distance = std::hypot(
          local_position_.x - estimate->position_ned.x(),
          local_position_.y - estimate->position_ned.y());
        const double vertical_error =
          std::abs(local_position_.z + rendezvous_altitude_m_);
        if (horizontal_distance <= rendezvous_radius_m_ &&
          vertical_error <= search_z_threshold_)
        {
          transition_to(
            LandingState::ACQUIRE_ARUCO,
            "UAV reached GNSS rendezvous region above moving deck");
        }
        break;
      }

    case LandingState::ACQUIRE_ARUCO:
      {
        if (!gnss_guidance_->ready(now.seconds())) {
          transition_to(
            LandingState::WAIT_DECK_GNSS,
            "deck GNSS became stale while acquiring ArUco");
          break;
        }

        const auto estimate = gnss_guidance_->estimate(now.seconds());
        if (!estimate.has_value()) {
          transition_to(
            LandingState::WAIT_DECK_GNSS,
            "deck GNSS estimate unavailable while acquiring ArUco");
          break;
        }

        const auto visual_position = visual_guidance_->visual_position(now.seconds());
        if (marker_is_stably_visible(now) && visual_position.has_value() &&
          visual_guidance_->consistent_with_gnss(
            *visual_position, estimate->position_ned))
        {
          transition_to(
            LandingState::VISUAL_HANDOVER,
            "stable full-transform ArUco pose is consistent with deck GNSS");
          break;
        }

        Eigen::Vector2d desired_target = estimate->position_ned.head<2>();
        if (!have_search_pattern_start_time_) {
          search_pattern_start_time_ = now;
          have_search_pattern_start_time_ = true;
        }
        const double elapsed_s = (now - search_pattern_start_time_).seconds();
        const auto offset = gnss_guidance_->search_offset(elapsed_s);
        if (!offset.has_value()) {
          transition_to(LandingState::ABORT, "invalid GNSS-centered search offset");
          break;
        }
        desired_target += *offset;

        const Eigen::Vector2d current_target = target_valid_ ?
          Eigen::Vector2d{target_x_, target_y_} :
          Eigen::Vector2d{local_position_.x, local_position_.y};
        const auto limited_target = gnss_guidance_->limit_target_xy(
          current_target, desired_target, dt);
        if (!limited_target.has_value()) {
          transition_to(LandingState::ABORT, "invalid ArUco acquisition target");
          break;
        }

        set_target(
          limited_target->x(),
          limited_target->y(),
          -rendezvous_altitude_m_,
          current_yaw_);
        break;
      }

    case LandingState::VISUAL_HANDOVER:
      {
        if (!gnss_guidance_->ready(now.seconds())) {
          transition_to(
            LandingState::WAIT_DECK_GNSS,
            "deck GNSS became stale during GNSS-to-visual handover");
          break;
        }

        const auto estimate = gnss_guidance_->estimate(now.seconds());
        if (!estimate.has_value()) {
          transition_to(
            LandingState::WAIT_DECK_GNSS,
            "deck GNSS estimate unavailable during GNSS-to-visual handover");
          break;
        }

        const auto visual_position = visual_guidance_->visual_position(now.seconds());
        const bool visual_valid =
          visual_position.has_value() &&
          marker_is_fresh(now) &&
          visual_guidance_->consistent_with_gnss(
            *visual_position, estimate->position_ned);
        const VisualLossState loss_state =
          visual_guidance_->loss_state(visual_valid, now.seconds());

        if (loss_state == VisualLossState::kLongLoss) {
          transition_to(
            LandingState::RECOVER_TO_GNSS,
            "visual pose was lost or inconsistent for the long timeout during handover");
          break;
        }
        if (!visual_valid) {
          set_target(
            target_valid_ ? target_x_ : local_position_.x,
            target_valid_ ? target_y_ : local_position_.y,
            -rendezvous_altitude_m_,
            current_yaw_);
          break;
        }

        handover_progress_s_ += dt;
        const auto blended_target = visual_guidance_->blended_target_xy(
          estimate->position_ned.head<2>(),
          visual_position->head<2>(),
          handover_progress_s_);
        const Eigen::Vector2d current_target = target_valid_ ?
          Eigen::Vector2d{target_x_, target_y_} :
          Eigen::Vector2d{local_position_.x, local_position_.y};
        const auto limited_target = blended_target.has_value() ?
          visual_guidance_->limit_target_xy(current_target, *blended_target, dt) :
          std::nullopt;
        if (!limited_target.has_value()) {
          transition_to(LandingState::ABORT, "invalid blended visual handover target");
          break;
        }

        set_target(
          limited_target->x(),
          limited_target->y(),
          -rendezvous_altitude_m_,
          current_yaw_);

        const auto alpha = visual_guidance_->handover_alpha(handover_progress_s_);
        if (alpha.has_value() && *alpha >= 1.0) {
          transition_to(
            LandingState::TRACK_TARGET,
            "GNSS-to-visual handover weight reached one");
        }
        break;
      }

    case LandingState::TRACK_TARGET:
    case LandingState::WAIT_LANDING_WINDOW:
    case LandingState::DESCEND:
    case LandingState::TEST_HEIGHT_HOLD:
    case LandingState::FINAL_DESCENT:
    case LandingState::TOUCHDOWN_CANDIDATE_HOLD:
    case LandingState::TOUCHDOWN_HOLD:
    case LandingState::RECOVER_CLIMB:
      {
        const auto visual_position = visual_guidance_->visual_position(now.seconds());
        const bool visual_valid = visual_position.has_value() && marker_is_fresh(now);
        const VisualLossState loss_state =
          visual_guidance_->loss_state(visual_valid, now.seconds());

        if (loss_state == VisualLossState::kLongLoss) {
          transition_to(
            LandingState::RECOVER_TO_GNSS,
            "visual pose was lost for the long timeout during visual guidance");
          break;
        }

        const auto estimate = target_state_estimator_->estimate();
        const double valid_measurement_age_s =
          estimate.has_value() && have_estimator_measurement_receipt_time_ ?
          now.seconds() - last_estimator_measurement_receipt_time_s_ :
          std::numeric_limits<double>::infinity();
        const double estimator_state_age_s =
          estimate.has_value() && have_estimator_measurement_receipt_time_ ?
          now.seconds() - last_estimator_state_receipt_time_s_ :
          std::numeric_limits<double>::infinity();
        std::optional<Eigen::Vector3d> predicted_position_ned;
        std::optional<Eigen::Vector2d> predicted_position_xy;
        if (estimate.has_value() &&
          std::isfinite(estimator_state_age_s) && estimator_state_age_s >= 0.0)
        {
          const auto prediction = motion_predictor_->predict(
            *estimate, estimator_state_age_s);
          if (prediction.has_value()) {
            predicted_position_ned = prediction->position_ned;
            predicted_position_xy = prediction->position_ned.head<2>();
          }
        }

        MovingTargetTrackingInput tracking_input;
        tracking_input.current_target_xy = target_valid_ ?
          Eigen::Vector2d{target_x_, target_y_} :
          Eigen::Vector2d{local_position_.x, local_position_.y};
        if (visual_position.has_value()) {
          tracking_input.raw_visual_position_xy = visual_position->head<2>();
        }
        tracking_input.estimated_state = estimate;
        tracking_input.predicted_position_xy = predicted_position_xy;
        tracking_input.uav_position_xy =
          Eigen::Vector2d{local_position_.x, local_position_.y};
        std::optional<Eigen::Vector2d> uav_velocity_xy;
        if (local_position_.v_xy_valid &&
          std::isfinite(local_position_.vx) &&
          std::isfinite(local_position_.vy))
        {
          uav_velocity_xy = Eigen::Vector2d{local_position_.vx, local_position_.vy};
          tracking_input.uav_velocity_xy = uav_velocity_xy;
        }
        tracking_input.visual_fresh = visual_valid;
        tracking_input.estimate_age_s = valid_measurement_age_s;
        tracking_input.dt_s = dt;

        if (state_ != LandingState::TRACK_TARGET) {
          update_landing_window(
            now,
            visual_valid,
            valid_measurement_age_s,
            estimate,
            predicted_position_xy,
            uav_velocity_xy);
        }

        const auto command = tracking_controller_->compute(tracking_input);
        if (!command.has_value()) {
          tracking_controller_->reset();
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Moving target tracking input unavailable; holding the latest safe position target");
          const bool relative_descent_state =
            state_ == LandingState::DESCEND ||
            state_ == LandingState::TEST_HEIGHT_HOLD ||
            state_ == LandingState::FINAL_DESCENT ||
            state_ == LandingState::TOUCHDOWN_CANDIDATE_HOLD ||
            state_ == LandingState::TOUCHDOWN_HOLD ||
            state_ == LandingState::RECOVER_CLIMB ||
            (state_ == LandingState::WAIT_LANDING_WINDOW &&
            relative_descent_enabled_ && relative_descent_controller_->initialized());
          set_target(
            target_valid_ ? target_x_ : local_position_.x,
            target_valid_ ? target_y_ : local_position_.y,
            relative_descent_state && target_valid_ ? target_z_ : -rendezvous_altitude_m_,
            current_yaw_);
          break;
        }

        double vertical_target_z = -rendezvous_altitude_m_;
        std::optional<RelativeDescentOutput> descent_output;
        std::optional<FinalDescentOutput> final_descent_output;
        const bool descent_state =
          state_ == LandingState::DESCEND ||
          state_ == LandingState::TEST_HEIGHT_HOLD ||
          state_ == LandingState::RECOVER_CLIMB;
        const bool final_descent_state =
          state_ == LandingState::FINAL_DESCENT ||
          state_ == LandingState::TOUCHDOWN_CANDIDATE_HOLD ||
          state_ == LandingState::TOUCHDOWN_HOLD;
        const bool may_start_descent =
          state_ == LandingState::WAIT_LANDING_WINDOW &&
          relative_descent_enabled_ &&
          !descent_reentry_locked_ &&
          landing_window_result_valid_ &&
          landing_window_result_.window_open;
        const bool continue_relative_height_hold =
          state_ == LandingState::WAIT_LANDING_WINDOW &&
          relative_descent_enabled_ &&
          relative_descent_controller_->initialized();
        if (descent_state || may_start_descent || continue_relative_height_hold) {
          descent_output = update_relative_descent(
            estimate, predicted_position_ned, visual_valid, loss_state, dt);
          if (descent_output.has_value() && predicted_position_ned.has_value()) {
            vertical_target_z =
              predicted_position_ned->z() - descent_output->height_reference_m;
          } else if (target_valid_) {
            vertical_target_z = target_z_;
          }
        } else if (final_descent_state) {
          final_descent_output = update_final_descent(estimate, visual_valid, dt);
          if (state_ == LandingState::TOUCHDOWN_HOLD && touchdown_hold_target_valid_) {
            vertical_target_z = touchdown_hold_target_z_;
          } else if (final_descent_output.has_value() && predicted_position_ned.has_value()) {
            vertical_target_z =
              predicted_position_ned->z() -
              final_descent_output->relative_height_reference_m;
          } else if (target_valid_) {
            vertical_target_z = target_z_;
          }
        }

        std::optional<double> vertical_reference_velocity_ned_mps;
        if (descent_output.has_value()) {
          vertical_reference_velocity_ned_mps =
            descent_output->vertical_reference_velocity_ned_mps;
        } else if (final_descent_output.has_value()) {
          vertical_reference_velocity_ned_mps =
            final_descent_output->vertical_reference_velocity_ned_mps;
        }
        if (vertical_velocity_feedforward_enabled_ &&
          vertical_reference_velocity_ned_mps.has_value() &&
          state_ != LandingState::TOUCHDOWN_HOLD &&
          vertical_state_estimator_enabled_ && vertical_state_measurement_valid_)
        {
          const double vertical_observation_age_s =
            now.seconds() - last_vertical_state_measurement_receipt_time_s_;
          const auto vertical_estimate = vertical_state_estimator_->estimate();
          if (vertical_estimate.has_value() &&
            std::isfinite(vertical_observation_age_s) &&
            vertical_observation_age_s >= 0.0 &&
            vertical_observation_age_s <= estimator_output_timeout_s_)
          {
            const double requested_down_velocity_mps =
              vertical_velocity_feedforward_gain_ *
              vertical_estimate->deck_vertical_velocity_ned_mps +
              *vertical_reference_velocity_ned_mps;
            set_vertical_velocity_feedforward(std::clamp(
              requested_down_velocity_mps,
              -vertical_velocity_feedforward_max_mps_,
              vertical_velocity_feedforward_max_mps_));
          }
        }

        set_target(
          command->position_target_xy.x(),
          command->position_target_xy.y(),
          vertical_target_z,
          current_yaw_);
        if (command->velocity_feedforward_xy.has_value()) {
          set_velocity_feedforward(
            command->velocity_feedforward_xy->x(),
            command->velocity_feedforward_xy->y());
        }
        set_adaptive_tracking_debug(
          command->effective_relative_velocity_gain,
          command->estimated_deck_acceleration_xy);

        if (state_ == LandingState::TRACK_TARGET) {
          transition_to(
            LandingState::WAIT_LANDING_WINDOW,
            "visual tracking command is valid; begin landing-window evaluation");
        } else if (may_start_descent && descent_output.has_value()) {
          transition_to(
            LandingState::DESCEND,
            "landing window opened and relative descent is explicitly enabled");
        } else if (state_ == LandingState::DESCEND && descent_output.has_value()) {
          if (descent_output->phase == RelativeDescentPhase::kTestHeightHold) {
            transition_to(
              LandingState::TEST_HEIGHT_HOLD,
              "minimum P5B test height reached");
          } else if (descent_output->phase == RelativeDescentPhase::kRecovering) {
            transition_to(
              LandingState::RECOVER_CLIMB,
              "relative descent controller requested recovery climb");
          }
        } else if (state_ == LandingState::TEST_HEIGHT_HOLD &&
          final_descent_enabled_ &&
          descent_output.has_value() &&
          descent_output->phase == RelativeDescentPhase::kTestHeightHold &&
          landing_window_result_valid_ && landing_window_result_.window_open)
        {
          transition_to(
            LandingState::FINAL_DESCENT,
            "P6B final descent is explicitly enabled at the P5B test height");
        } else if (state_ == LandingState::TEST_HEIGHT_HOLD &&
          descent_output.has_value() &&
          descent_output->phase == RelativeDescentPhase::kRecovering)
        {
          transition_to(
            LandingState::RECOVER_CLIMB,
            "landing window became unsafe at the test height");
        } else if (state_ == LandingState::FINAL_DESCENT &&
          final_descent_output.has_value())
        {
          if (final_descent_output->phase == FinalDescentPhase::kCandidateHold) {
            transition_to(
              LandingState::TOUCHDOWN_CANDIDATE_HOLD,
              "P6A touchdown candidate detected; freeze final descent reference");
          } else if (
            final_descent_output->phase == FinalDescentPhase::kTouchdownHold)
          {
            transition_to(
              LandingState::TOUCHDOWN_HOLD,
              "P6A touchdown confirmation latched");
          } else if (
            final_descent_output->phase == FinalDescentPhase::kRecoveryRequested)
          {
            transition_to(
              LandingState::RECOVER_CLIMB,
              "P6B final descent requested recovery");
          }
        } else if (state_ == LandingState::TOUCHDOWN_CANDIDATE_HOLD &&
          final_descent_output.has_value())
        {
          if (final_descent_output->phase == FinalDescentPhase::kTouchdownHold) {
            transition_to(
              LandingState::TOUCHDOWN_HOLD,
              "touchdown candidate satisfied confirmation duration");
          } else if (final_descent_output->phase == FinalDescentPhase::kDescending) {
            transition_to(
              LandingState::FINAL_DESCENT,
              "touchdown candidate cleared; resume final descent");
          } else if (
            final_descent_output->phase == FinalDescentPhase::kRecoveryRequested)
          {
            transition_to(
              LandingState::RECOVER_CLIMB,
              "touchdown candidate became unsafe; recover climb");
          }
        } else if (state_ == LandingState::RECOVER_CLIMB &&
          descent_output.has_value() &&
          descent_output->phase == RelativeDescentPhase::kPaused &&
          descent_output->height_reference_m >= descent_recovery_height_m_)
        {
          transition_to(
            LandingState::WAIT_LANDING_WINDOW,
            "recovery height reached; wait for a new landing window");
        }
        break;
      }

    case LandingState::RECOVER_TO_GNSS:
      {
        if (!gnss_guidance_->ready(now.seconds())) {
          transition_to(
            LandingState::WAIT_DECK_GNSS,
            "deck GNSS is unavailable during visual recovery");
          break;
        }

        const auto estimate = gnss_guidance_->estimate(now.seconds());
        if (!estimate.has_value()) {
          transition_to(
            LandingState::WAIT_DECK_GNSS,
            "deck GNSS estimate unavailable during visual recovery");
          break;
        }

        const Eigen::Vector2d current_target = target_valid_ ?
          Eigen::Vector2d{target_x_, target_y_} :
          Eigen::Vector2d{local_position_.x, local_position_.y};
        const auto limited_target = gnss_guidance_->limit_target_xy(
          current_target, estimate->position_ned.head<2>(), dt);
        if (!limited_target.has_value()) {
          transition_to(LandingState::ABORT, "invalid GNSS recovery target");
          break;
        }
        set_target(
          limited_target->x(),
          limited_target->y(),
          -rendezvous_altitude_m_,
          current_yaw_);

        const double horizontal_distance = std::hypot(
          local_position_.x - estimate->position_ned.x(),
          local_position_.y - estimate->position_ned.y());
        const double vertical_error =
          std::abs(local_position_.z + rendezvous_altitude_m_);
        transition_to(
          horizontal_distance <= rendezvous_radius_m_ &&
          vertical_error <= search_z_threshold_ ?
          LandingState::ACQUIRE_ARUCO : LandingState::RENDEZVOUS_GNSS,
          "GNSS recovery target is valid; resume coarse guidance");
        break;
      }

    case LandingState::GOTO_ARUCO_AREA:
      {
        const double dx = local_position_.x - search_x_;
        const double dy = local_position_.y - search_y_;
        const double horizontal_distance = std::hypot(dx, dy);
        const double vertical_error = std::abs(local_position_.z + search_alt_);

        if (horizontal_distance <= search_xy_threshold_ &&
          vertical_error <= search_z_threshold_)
        {
          transition_to(
            LandingState::WAIT_ARUCO,
            "legacy fixed search point reached");
        }
        break;
      }

    case LandingState::WAIT_ARUCO:
      if (marker_is_fresh(now) && stable_visible_count_ >= stable_detect_count_) {
        last_marker_seen_time_ = now;
        have_last_marker_seen_time_ = true;
        transition_to(
          LandingState::CENTER_ABOVE_MARKER,
          "legacy ArUco visibility became stable");
      }
      break;

    case LandingState::CENTER_ABOVE_MARKER:
      if (marker_is_fresh(now)) {
        last_marker_seen_time_ = now;
        have_last_marker_seen_time_ = true;

        double error_north = 0.0;
        double error_east = 0.0;
        if (compute_local_marker_error(error_north, error_east)) {
          set_target(
            local_position_.x +
            std::clamp(error_north, -max_xy_step_, max_xy_step_),
            local_position_.y +
            std::clamp(error_east, -max_xy_step_, max_xy_step_),
            local_position_.z,
            current_yaw_);

          if (std::hypot(error_north, error_east) < center_xy_threshold_) {
            transition_to(
              LandingState::DESCEND_WITH_TRACKING,
              "legacy horizontal centering threshold satisfied");
          }
        }
      } else {
        if (
          have_last_marker_seen_time_ &&
          (now - last_marker_seen_time_).seconds() > marker_lost_timeout_)
        {
          transition_to(
            LandingState::WAIT_ARUCO,
            "legacy ArUco lost during centering");
        }
      }
      break;

    case LandingState::DESCEND_WITH_TRACKING:
      if (local_position_.z >= -final_alt_) {
        transition_to(
          LandingState::FINAL_LAND,
          "legacy final altitude reached");
        break;
      }

      if (marker_is_fresh(now)) {
        last_marker_seen_time_ = now;
        have_last_marker_seen_time_ = true;

        double error_north = 0.0;
        double error_east = 0.0;
        if (compute_local_marker_error(error_north, error_east)) {
          const double next_z = std::min(
            target_z_ + max_descent_rate_ * dt,
            0.0);
          set_target(
            local_position_.x +
            std::clamp(error_north, -max_xy_step_, max_xy_step_),
            local_position_.y +
            std::clamp(error_east, -max_xy_step_, max_xy_step_),
            next_z,
            current_yaw_);
        }
      } else {
        if (
          have_last_marker_seen_time_ &&
          (now - last_marker_seen_time_).seconds() > marker_lost_timeout_)
        {
          transition_to(
            LandingState::ABORT,
            "legacy ArUco lost during descent");
        }
      }
      break;

    case LandingState::FINAL_LAND:
      if (enable_auto_land_ && !final_land_command_sent_) {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
        final_land_command_sent_ = true;
      }
      transition_to(
        LandingState::DONE,
        "legacy final land action completed");
      break;

    case LandingState::DONE:
    case LandingState::ABORT:
      break;
  }
}

void Px4ArucoLandingNode::transition_to(
  LandingState new_state,
  const char * reason)
{
  if (state_ == new_state) {
    return;
  }

  const LandingState previous_state = state_;
  RCLCPP_INFO(
    get_logger(),
    "Landing state: %s -> %s; reason: %s",
    state_name(previous_state),
    state_name(new_state),
    reason);
  state_ = new_state;

  const auto is_visual_tracking_state = [](LandingState state) {
      return state == LandingState::TRACK_TARGET ||
             state == LandingState::WAIT_LANDING_WINDOW ||
             state == LandingState::DESCEND ||
             state == LandingState::TEST_HEIGHT_HOLD ||
             state == LandingState::FINAL_DESCENT ||
             state == LandingState::TOUCHDOWN_CANDIDATE_HOLD ||
             state == LandingState::TOUCHDOWN_HOLD ||
             state == LandingState::RECOVER_CLIMB;
    };
  const auto is_final_descent_state = [](LandingState state) {
      return state == LandingState::FINAL_DESCENT ||
             state == LandingState::TOUCHDOWN_CANDIDATE_HOLD ||
             state == LandingState::TOUCHDOWN_HOLD;
    };
  const bool preserve_visual_tracking_history =
    is_visual_tracking_state(previous_state) && is_visual_tracking_state(new_state);
  if (!preserve_visual_tracking_history) {
    clear_velocity_feedforward();
    tracking_controller_->reset();
    relative_descent_controller_->reset();
    relative_descent_debug_valid_ = false;
  }
  if (is_final_descent_state(previous_state) && !is_final_descent_state(new_state)) {
    final_descent_controller_->reset();
    final_descent_debug_valid_ = false;
    touchdown_hold_target_valid_ = false;
  }

  switch (state_) {
    case LandingState::OFFBOARD_PRE_STREAM:
      prestream_setpoint_count_ = 0;
      set_target(
        takeoff_start_x_,
        takeoff_start_y_,
        local_position_.z,
        initial_yaw_);
      break;

    case LandingState::ARM_AND_TAKEOFF:
      set_target(
        takeoff_start_x_,
        takeoff_start_y_,
        -takeoff_alt_,
        initial_yaw_);
      break;

    case LandingState::WAIT_DECK_GNSS:
      stable_visible_count_ = 0;
      have_aruco_visible_since_ = false;
      have_search_pattern_start_time_ = false;
      have_marker_pose_ned_ = false;
      have_last_aruco_sample_stamp_ = false;
      have_estimator_measurement_receipt_time_ = false;
      vertical_state_measurement_valid_ = false;
      raw_relative_height_valid_ = false;
      have_estimated_deck_attitude_ = false;
      visual_guidance_->reset();
      target_state_estimator_->reset();
      vertical_state_estimator_->reset();
      touchdown_detector_->reset();
      touchdown_result_valid_ = false;
      final_descent_controller_->reset();
      final_descent_debug_valid_ = false;
      touchdown_hold_target_valid_ = false;
      deck_attitude_estimator_->reset();
      landing_window_->reset();
      landing_window_result_valid_ = false;
      set_target(
        local_position_.x,
        local_position_.y,
        -rendezvous_altitude_m_,
        current_yaw_);
      break;

    case LandingState::RENDEZVOUS_GNSS:
      set_target(
        target_valid_ ? target_x_ : local_position_.x,
        target_valid_ ? target_y_ : local_position_.y,
        -rendezvous_altitude_m_,
        current_yaw_);
      break;

    case LandingState::ACQUIRE_ARUCO:
      search_pattern_start_time_ = get_clock()->now();
      have_search_pattern_start_time_ = true;
      stable_visible_count_ = 0;
      have_aruco_visible_since_ = false;
      have_marker_pose_ned_ = false;
      have_last_aruco_sample_stamp_ = false;
      have_estimator_measurement_receipt_time_ = false;
      vertical_state_measurement_valid_ = false;
      raw_relative_height_valid_ = false;
      have_estimated_deck_attitude_ = false;
      visual_guidance_->reset();
      target_state_estimator_->reset();
      vertical_state_estimator_->reset();
      touchdown_detector_->reset();
      touchdown_result_valid_ = false;
      final_descent_controller_->reset();
      final_descent_debug_valid_ = false;
      touchdown_hold_target_valid_ = false;
      deck_attitude_estimator_->reset();
      landing_window_->reset();
      landing_window_result_valid_ = false;
      set_target(
        target_valid_ ? target_x_ : local_position_.x,
        target_valid_ ? target_y_ : local_position_.y,
        -rendezvous_altitude_m_,
        current_yaw_);
      break;

    case LandingState::VISUAL_HANDOVER:
      handover_progress_s_ = 0.0;
      set_target(
        target_valid_ ? target_x_ : local_position_.x,
        target_valid_ ? target_y_ : local_position_.y,
        -rendezvous_altitude_m_,
        current_yaw_);
      break;

    case LandingState::TRACK_TARGET:
      landing_window_->reset();
      landing_window_result_valid_ = false;
      descent_reentry_locked_ = false;
      set_target(
        target_valid_ ? target_x_ : local_position_.x,
        target_valid_ ? target_y_ : local_position_.y,
        -rendezvous_altitude_m_,
        current_yaw_);
      break;

    case LandingState::WAIT_LANDING_WINDOW:
      landing_window_->reset();
      if (previous_state == LandingState::RECOVER_CLIMB) {
        descent_reentry_locked_ = true;
        RCLCPP_WARN(
          get_logger(),
          "Relative descent re-entry is locked after recovery; a new visual handover is required");
      } else {
        relative_descent_controller_->reset();
        relative_descent_debug_valid_ = false;
      }
      landing_window_result_valid_ = false;
      set_target(
        target_valid_ ? target_x_ : local_position_.x,
        target_valid_ ? target_y_ : local_position_.y,
        previous_state == LandingState::RECOVER_CLIMB && target_valid_ ?
        target_z_ : -rendezvous_altitude_m_,
        current_yaw_);
      break;

    case LandingState::DESCEND:
    case LandingState::TEST_HEIGHT_HOLD:
      break;

    case LandingState::FINAL_DESCENT:
      if (previous_state != LandingState::TOUCHDOWN_CANDIDATE_HOLD) {
        final_descent_controller_->reset();
        final_descent_debug_valid_ = false;
      }
      touchdown_hold_target_valid_ = false;
      break;

    case LandingState::TOUCHDOWN_CANDIDATE_HOLD:
      break;

    case LandingState::TOUCHDOWN_HOLD:
      touchdown_hold_target_z_ = local_position_.z;
      touchdown_hold_target_valid_ = std::isfinite(touchdown_hold_target_z_);
      clear_velocity_feedforward();
      break;

    case LandingState::RECOVER_CLIMB:
      break;

    case LandingState::RECOVER_TO_GNSS:
      stable_visible_count_ = 0;
      have_aruco_visible_since_ = false;
      have_marker_pose_ned_ = false;
      have_last_aruco_sample_stamp_ = false;
      have_estimator_measurement_receipt_time_ = false;
      vertical_state_measurement_valid_ = false;
      raw_relative_height_valid_ = false;
      have_estimated_deck_attitude_ = false;
      visual_guidance_->reset();
      target_state_estimator_->reset();
      vertical_state_estimator_->reset();
      deck_attitude_estimator_->reset();
      landing_window_->reset();
      landing_window_result_valid_ = false;
      set_target(
        target_valid_ ? target_x_ : local_position_.x,
        target_valid_ ? target_y_ : local_position_.y,
        -rendezvous_altitude_m_,
        current_yaw_);
      break;

    case LandingState::GOTO_ARUCO_AREA:
      set_target(search_x_, search_y_, -search_alt_, initial_yaw_);
      break;

    case LandingState::WAIT_ARUCO:
      stable_visible_count_ = 0;
      set_target(search_x_, search_y_, -search_alt_, current_yaw_);
      break;

    case LandingState::CENTER_ABOVE_MARKER:
    case LandingState::DESCEND_WITH_TRACKING:
      last_marker_seen_time_ = get_clock()->now();
      have_last_marker_seen_time_ = true;
      break;

    case LandingState::FINAL_LAND:
      final_land_command_sent_ = false;
      set_target(
        local_position_.x,
        local_position_.y,
        local_position_.z,
        current_yaw_);
      break;

    case LandingState::ABORT:
      have_estimator_measurement_receipt_time_ = false;
      vertical_state_measurement_valid_ = false;
      raw_relative_height_valid_ = false;
      have_estimated_deck_attitude_ = false;
      target_state_estimator_->reset();
      vertical_state_estimator_->reset();
      deck_attitude_estimator_->reset();
      landing_window_->reset();
      landing_window_result_valid_ = false;
      set_target(
        local_position_.x,
        local_position_.y,
        -abort_hover_alt_,
        current_yaw_);
      break;

    case LandingState::INIT:
    case LandingState::WAIT_FOR_PX4:
    case LandingState::DONE:
      break;
  }
}

bool Px4ArucoLandingNode::px4_data_ready() const
{
  return have_vehicle_status_ &&
         have_local_position_ &&
         have_vehicle_odometry_ &&
         local_position_.xy_valid &&
         local_position_.z_valid &&
         std::isfinite(local_position_.x) &&
         std::isfinite(local_position_.y) &&
         std::isfinite(local_position_.z) &&
         vehicle_odometry_.pose_frame ==
         px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED &&
         quaternion_is_valid(vehicle_odometry_.q.data());
}

bool Px4ArucoLandingNode::marker_is_fresh(const rclcpp::Time & now) const
{
  if (!aruco_visible_ || !have_aruco_pose_) {
    return false;
  }

  return (now - last_aruco_pose_time_).seconds() <= aruco_pose_timeout_ &&
         (now - last_aruco_visible_time_).seconds() <= aruco_pose_timeout_ &&
         std::isfinite(aruco_pose_.pose.position.x) &&
         std::isfinite(aruco_pose_.pose.position.y);
}

bool Px4ArucoLandingNode::marker_is_stably_visible(const rclcpp::Time & now) const
{
  return marker_is_fresh(now) &&
         have_aruco_visible_since_ &&
         now >= aruco_visible_since_ &&
         (now - aruco_visible_since_).seconds() >= aruco_acquire_duration_s_;
}

bool Px4ArucoLandingNode::should_retry_command(const rclcpp::Time & now) const
{
  return !have_last_command_time_ ||
         (now - last_command_time_).seconds() >= command_retry_interval_;
}

std::optional<double> Px4ArucoLandingNode::update_px4_to_ros_time_offset(
  const px4_msgs::msg::VehicleOdometry & odometry,
  const rclcpp::Time & receipt_time)
{
  const uint64_t sample_timestamp_us =
    odometry.timestamp_sample > 0U ? odometry.timestamp_sample : odometry.timestamp;
  const uint64_t sync_timestamp_us =
    odometry.timestamp > 0U ? odometry.timestamp : sample_timestamp_us;
  const double receipt_time_s = receipt_time.seconds();
  if (sample_timestamp_us == 0U || sync_timestamp_us == 0U ||
    !std::isfinite(receipt_time_s) || receipt_time_s <= 0.0)
  {
    return std::nullopt;
  }

  if (have_last_time_sync_observation_) {
    const bool ros_time_reset = receipt_time_s < last_time_sync_receipt_s_;
    const bool px4_time_reset = sync_timestamp_us < last_px4_sync_timestamp_us_;
    if (ros_time_reset || px4_time_reset) {
      vehicle_pose_history_->reset();
      have_px4_to_ros_time_offset_ = false;
      have_last_time_sync_observation_ = false;
    } else if (sync_timestamp_us == last_px4_sync_timestamp_us_) {
      return std::nullopt;
    }
  }

  const double observed_offset_s =
    receipt_time_s - static_cast<double>(sync_timestamp_us) * 1.0e-6;
  if (!std::isfinite(observed_offset_s)) {
    return std::nullopt;
  }

  if (!have_px4_to_ros_time_offset_) {
    px4_to_ros_time_offset_s_ = observed_offset_s;
    have_px4_to_ros_time_offset_ = true;
  } else {
    const double offset_error_s = observed_offset_s - px4_to_ros_time_offset_s_;
    if (std::abs(offset_error_s) > px4_clock_offset_max_jump_s_) {
      // 时钟发生明显跳变时清空旧历史，以免跨时间域插值。
      vehicle_pose_history_->reset();
      px4_to_ros_time_offset_s_ = observed_offset_s;
    } else {
      px4_to_ros_time_offset_s_ +=
        px4_clock_offset_filter_gain_ * offset_error_s;
    }
  }

  last_time_sync_receipt_s_ = receipt_time_s;
  last_px4_sync_timestamp_us_ = sync_timestamp_us;
  have_last_time_sync_observation_ = true;

  const double sample_time_s =
    static_cast<double>(sample_timestamp_us) * 1.0e-6 + px4_to_ros_time_offset_s_;
  return std::isfinite(sample_time_s) && sample_time_s > 0.0 ?
         std::optional<double>(sample_time_s) : std::nullopt;
}

bool Px4ArucoLandingNode::compute_marker_pose_ned(
  double image_sample_time_s,
  Pose3d & marker_pose_ned) const
{
  if (!have_aruco_pose_ || !std::isfinite(image_sample_time_s)) {
    return false;
  }

  const auto local_body_pose = vehicle_pose_history_->lookup(image_sample_time_s);
  if (!local_body_pose.has_value()) {
    return false;
  }

  const Pose3d camera_marker_pose{
    Eigen::Vector3d{
      aruco_pose_.pose.position.x,
      aruco_pose_.pose.position.y,
      aruco_pose_.pose.position.z},
    Eigen::Quaterniond{
      aruco_pose_.pose.orientation.w,
      aruco_pose_.pose.orientation.x,
      aruco_pose_.pose.orientation.y,
      aruco_pose_.pose.orientation.z}};

  const auto transformed = transform_marker_to_local_ned(
    *local_body_pose,
    PoseReferenceFrame::kLocalNed,
    body_camera_pose_,
    camera_marker_pose);
  if (!transformed.has_value()) {
    return false;
  }
  marker_pose_ned = *transformed;
  return true;
}

bool Px4ArucoLandingNode::compute_local_marker_error(
  double & error_north,
  double & error_east) const
{
  if (!have_aruco_pose_ || !quaternion_is_valid(vehicle_odometry_.q.data())) {
    return false;
  }

  const double body_forward =
    camera_y_to_body_x_sign_ * aruco_pose_.pose.position.y;
  const double body_right =
    camera_x_to_body_y_sign_ * aruco_pose_.pose.position.x;
  const double cos_yaw = std::cos(current_yaw_);
  const double sin_yaw = std::sin(current_yaw_);

  error_north = cos_yaw * body_forward - sin_yaw * body_right;
  error_east = sin_yaw * body_forward + cos_yaw * body_right;
  return std::isfinite(error_north) && std::isfinite(error_east);
}

void Px4ArucoLandingNode::update_estimated_deck_attitude(
  const Eigen::Quaterniond & marker_to_ned_rotation,
  const rclcpp::Time & sample_time)
{
  const auto estimate = deck_attitude_estimator_->update(marker_to_ned_rotation);
  if (!estimate.has_value()) {
    have_estimated_deck_attitude_ = false;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Rejected Marker attitude because its upward normal is invalid or flipped");
    return;
  }

  estimated_deck_attitude_ = *estimate;
  last_estimated_deck_attitude_time_ = sample_time;
  have_estimated_deck_attitude_ = true;
}

void Px4ArucoLandingNode::update_landing_window(
  const rclcpp::Time & now,
  bool visual_valid,
  double visual_age_s,
  const std::optional<TargetStateEstimate> & estimate,
  const std::optional<Eigen::Vector2d> & predicted_position_xy,
  const std::optional<Eigen::Vector2d> & uav_velocity_xy)
{
  LandingWindowInput input;
  input.visual_fresh = visual_valid;
  input.visual_age_s = visual_age_s;
  input.prediction_valid =
    predicted_position_xy.has_value() && predicted_position_xy->allFinite();
  input.now_s = now.seconds();

  const bool estimate_valid =
    estimate.has_value() &&
    estimate->position_ned.allFinite() &&
    estimate->velocity_ned.allFinite() &&
    estimate->covariance.allFinite() &&
    std::isfinite(estimate->sample_time_s);
  input.estimate_valid = estimate_valid;

  if (estimate_valid) {
    input.horizontal_error_m =
      (estimate->position_ned.head<2>() -
      Eigen::Vector2d{local_position_.x, local_position_.y}).norm();
    input.relative_height_m = estimate->position_ned.z() - local_position_.z;
    if (uav_velocity_xy.has_value() && uav_velocity_xy->allFinite()) {
      input.horizontal_relative_speed_mps =
        (estimate->velocity_ned.head<2>() - *uav_velocity_xy).norm();
    } else {
      input.horizontal_relative_speed_mps = std::numeric_limits<double>::infinity();
    }
  } else {
    input.horizontal_error_m = std::numeric_limits<double>::infinity();
    input.horizontal_relative_speed_mps = std::numeric_limits<double>::infinity();
    input.relative_height_m = std::numeric_limits<double>::quiet_NaN();
  }

  const bool attitude_fresh =
    have_estimated_deck_attitude_ &&
    now >= last_estimated_deck_attitude_time_ &&
    (now - last_estimated_deck_attitude_time_).seconds() <=
    landing_window_max_visual_age_s_;
  if (attitude_fresh) {
    input.deck_roll_rad = estimated_deck_attitude_.roll_rad;
    input.deck_pitch_rad = estimated_deck_attitude_.pitch_rad;
  } else {
    input.deck_roll_rad = std::numeric_limits<double>::quiet_NaN();
    input.deck_pitch_rad = std::numeric_limits<double>::quiet_NaN();
  }

  landing_window_result_ = landing_window_->update(input);
  landing_window_result_valid_ = true;
}

std::optional<RelativeDescentOutput> Px4ArucoLandingNode::update_relative_descent(
  const std::optional<TargetStateEstimate> & estimate,
  const std::optional<Eigen::Vector3d> & predicted_position_ned,
  bool visual_valid,
  VisualLossState visual_loss_state,
  double dt)
{
  const bool estimate_valid =
    estimate.has_value() &&
    estimate->position_ned.allFinite() &&
    estimate->velocity_ned.allFinite();
  const bool prediction_valid =
    predicted_position_ned.has_value() && predicted_position_ned->allFinite();

  RelativeDescentInput input;
  input.vertical_reference_valid =
    estimate_valid && prediction_valid && std::isfinite(local_position_.z);
  if (estimate_valid && std::isfinite(local_position_.z)) {
    input.current_relative_height_m = estimate->position_ned.z() - local_position_.z;
  } else {
    input.current_relative_height_m = std::numeric_limits<double>::quiet_NaN();
  }
  input.window_open =
    landing_window_result_valid_ && landing_window_result_.window_open;

  const std::uint32_t hard_failure_mask =
    landing_window_reason_mask(LandingWindowRejectReason::kEstimateInvalid) |
    landing_window_reason_mask(LandingWindowRejectReason::kPredictionInvalid) |
    landing_window_reason_mask(LandingWindowRejectReason::kRelativeHeight) |
    landing_window_reason_mask(LandingWindowRejectReason::kInvalidTime);
  const bool hard_window_failure =
    !landing_window_result_valid_ ||
    (landing_window_result_.reject_reasons & hard_failure_mask) != 0U;
  const bool sustained_visual_loss =
    !visual_valid && visual_loss_state == VisualLossState::kShortLoss;
  input.severe_failure =
    state_ == LandingState::RECOVER_CLIMB ||
    sustained_visual_loss || hard_window_failure;
  input.dt_s = dt;

  const auto output = relative_descent_controller_->update(input);
  if (!output.has_value()) {
    relative_descent_debug_valid_ = false;
    return std::nullopt;
  }

  relative_height_m_ = input.current_relative_height_m;
  relative_height_reference_m_ = output->height_reference_m;
  relative_descent_phase_ = output->phase;
  relative_descent_debug_valid_ = true;
  return output;
}

std::optional<FinalDescentOutput> Px4ArucoLandingNode::update_final_descent(
  const std::optional<TargetStateEstimate> & estimate,
  bool visual_valid,
  double dt)
{
  const bool estimate_valid =
    estimate.has_value() &&
    estimate->position_ned.allFinite() &&
    estimate->velocity_ned.allFinite() &&
    std::isfinite(local_position_.z);

  FinalDescentInput input;
  input.current_relative_height_m = estimate_valid ?
    estimate->position_ned.z() - local_position_.z :
    std::numeric_limits<double>::quiet_NaN();
  input.current_reference_height_m = final_descent_controller_->initialized() ?
    final_descent_output_.relative_height_reference_m :
    relative_height_reference_m_;
  input.final_descent_authorized = final_descent_enabled_;
  input.vertical_reference_valid = estimate_valid && visual_valid;
  input.landing_window_open =
    landing_window_result_valid_ && landing_window_result_.window_open;
  input.touchdown_status = touchdown_result_valid_ ?
    touchdown_result_.status : TouchdownStatus::kInsufficientEvidence;
  input.dt_s = dt;

  const auto output = final_descent_controller_->update(input);
  if (!output.has_value()) {
    final_descent_debug_valid_ = false;
    return std::nullopt;
  }

  final_descent_output_ = *output;
  final_descent_debug_valid_ = true;
  relative_height_m_ = input.current_relative_height_m;
  relative_height_reference_m_ = output->relative_height_reference_m;
  return output;
}

void Px4ArucoLandingNode::update_touchdown_detection(const rclcpp::Time & now)
{
  if (!touchdown_detector_enabled_) {
    touchdown_detector_->reset();
    touchdown_result_valid_ = false;
    return;
  }

  TouchdownDetectorInput input;
  input.sample_time_s = now.seconds();
  input.state_allows_touchdown_detection =
    state_ == LandingState::TEST_HEIGHT_HOLD ||
    state_ == LandingState::FINAL_DESCENT ||
    state_ == LandingState::TOUCHDOWN_CANDIDATE_HOLD ||
    state_ == LandingState::TOUCHDOWN_HOLD;
  input.px4_land_status_valid = have_vehicle_land_detected_;
  input.px4_land_status_age_s = have_vehicle_land_detected_ ?
    now.seconds() - last_vehicle_land_detected_receipt_time_s_ :
    std::numeric_limits<double>::infinity();
  input.freefall = vehicle_land_detected_.freefall;
  input.ground_contact = vehicle_land_detected_.ground_contact;
  input.maybe_landed = vehicle_land_detected_.maybe_landed;
  input.landed = vehicle_land_detected_.landed;
  input.at_rest = vehicle_land_detected_.at_rest;
  input.has_low_throttle = vehicle_land_detected_.has_low_throttle;
  input.vertical_movement = vehicle_land_detected_.vertical_movement;
  input.horizontal_movement = vehicle_land_detected_.horizontal_movement;
  input.rotational_movement = vehicle_land_detected_.rotational_movement;
  input.close_to_ground =
    vehicle_land_detected_.close_to_ground_or_skipped_check;

  const double visual_age_s = have_estimator_measurement_receipt_time_ ?
    now.seconds() - last_estimator_measurement_receipt_time_s_ :
    std::numeric_limits<double>::infinity();
  const auto deck_estimate = target_state_estimator_->estimate();
  input.visual_height_valid =
    deck_estimate.has_value() &&
    deck_estimate->position_ned.allFinite() &&
    have_estimator_measurement_receipt_time_ &&
    std::isfinite(local_position_.z);
  input.visual_height_age_s = visual_age_s;
  input.relative_height_m = input.visual_height_valid ?
    deck_estimate->position_ned.z() - local_position_.z :
    std::numeric_limits<double>::quiet_NaN();
  input.relative_horizontal_speed_valid =
    deck_estimate.has_value() &&
    deck_estimate->velocity_ned.allFinite() &&
    std::isfinite(local_position_.vx) &&
    std::isfinite(local_position_.vy);
  input.relative_horizontal_speed_mps = input.relative_horizontal_speed_valid ?
    std::hypot(
    deck_estimate->velocity_ned.x() - local_position_.vx,
    deck_estimate->velocity_ned.y() - local_position_.vy) :
    std::numeric_limits<double>::quiet_NaN();
  input.uav_vertical_velocity_mps =
    std::isfinite(local_position_.vz) ?
    static_cast<double>(local_position_.vz) :
    std::numeric_limits<double>::quiet_NaN();

  input.relative_vertical_velocity_mps =
    std::numeric_limits<double>::quiet_NaN();
  if (vertical_state_estimator_enabled_ && vertical_state_measurement_valid_ &&
    std::isfinite(local_position_.vz))
  {
    const auto vertical_estimate = vertical_state_estimator_->estimate();
    if (vertical_estimate.has_value() &&
      std::isfinite(vertical_estimate->deck_vertical_velocity_ned_mps))
    {
      input.relative_vertical_velocity_mps =
        vertical_estimate->deck_vertical_velocity_ned_mps - local_position_.vz;
    }
  }

  touchdown_result_ = touchdown_detector_->update(input);
  touchdown_result_valid_ = true;
}

void Px4ArucoLandingNode::set_target(double x, double y, double z, double yaw)
{
  target_x_ = x;
  target_y_ = y;
  target_z_ = z;
  target_yaw_ = yaw;
  target_valid_ =
    std::isfinite(target_x_) &&
    std::isfinite(target_y_) &&
    std::isfinite(target_z_) &&
    std::isfinite(target_yaw_);
}

void Px4ArucoLandingNode::set_velocity_feedforward(
  double north_mps,
  double east_mps)
{
  velocity_feedforward_north_mps_ = north_mps;
  velocity_feedforward_east_mps_ = east_mps;
  velocity_feedforward_valid_ =
    std::isfinite(velocity_feedforward_north_mps_) &&
    std::isfinite(velocity_feedforward_east_mps_);
}

void Px4ArucoLandingNode::set_vertical_velocity_feedforward(double down_mps)
{
  vertical_velocity_feedforward_down_mps_ = down_mps;
  vertical_velocity_feedforward_valid_ =
    std::isfinite(vertical_velocity_feedforward_down_mps_);
}

void Px4ArucoLandingNode::set_adaptive_tracking_debug(
  const std::optional<double> & effective_gain,
  const std::optional<Eigen::Vector2d> & estimated_deck_acceleration_xy)
{
  effective_relative_velocity_gain_valid_ =
    effective_gain.has_value() && std::isfinite(*effective_gain);
  if (effective_relative_velocity_gain_valid_) {
    effective_relative_velocity_gain_ = *effective_gain;
  }

  estimated_deck_acceleration_valid_ =
    estimated_deck_acceleration_xy.has_value() &&
    estimated_deck_acceleration_xy->allFinite();
  if (estimated_deck_acceleration_valid_) {
    estimated_deck_acceleration_xy_ = *estimated_deck_acceleration_xy;
  }
}

void Px4ArucoLandingNode::clear_velocity_feedforward()
{
  velocity_feedforward_north_mps_ = 0.0;
  velocity_feedforward_east_mps_ = 0.0;
  velocity_feedforward_valid_ = false;
  vertical_velocity_feedforward_down_mps_ = 0.0;
  vertical_velocity_feedforward_valid_ = false;
  effective_relative_velocity_gain_ = 0.0;
  effective_relative_velocity_gain_valid_ = false;
  estimated_deck_acceleration_xy_.setZero();
  estimated_deck_acceleration_valid_ = false;
}

void Px4ArucoLandingNode::publish_offboard_control_mode()
{
  px4_msgs::msg::OffboardControlMode msg{};
  msg.timestamp = static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  msg.position = true;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.thrust_and_torque = false;
  msg.direct_actuator = false;
  offboard_control_mode_pub_->publish(msg);
}

void Px4ArucoLandingNode::publish_trajectory_setpoint()
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  px4_msgs::msg::TrajectorySetpoint msg{};
  msg.timestamp = static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  msg.position = target_valid_ ?
    std::array<float, 3>{
    static_cast<float>(target_x_),
    static_cast<float>(target_y_),
    static_cast<float>(target_z_)} :
  std::array<float, 3>{nan, nan, nan};
  msg.velocity = target_valid_ ?
    std::array<float, 3>{
    velocity_feedforward_valid_ ?
    static_cast<float>(velocity_feedforward_north_mps_) : nan,
    velocity_feedforward_valid_ ?
    static_cast<float>(velocity_feedforward_east_mps_) : nan,
    vertical_velocity_feedforward_valid_ ?
    static_cast<float>(vertical_velocity_feedforward_down_mps_) : nan} :
  std::array<float, 3>{nan, nan, nan};
  msg.acceleration = {nan, nan, nan};
  msg.jerk = {nan, nan, nan};
  msg.yaw = target_valid_ ? static_cast<float>(target_yaw_) : nan;
  msg.yawspeed = nan;
  trajectory_setpoint_pub_->publish(msg);
}

void Px4ArucoLandingNode::publish_vehicle_command(
  uint32_t command,
  float param1,
  float param2,
  float param3,
  float param4,
  double param5,
  double param6,
  float param7)
{
  px4_msgs::msg::VehicleCommand msg{};
  msg.timestamp = static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  msg.param1 = param1;
  msg.param2 = param2;
  msg.param3 = param3;
  msg.param4 = param4;
  msg.param5 = param5;
  msg.param6 = param6;
  msg.param7 = param7;
  msg.command = command;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.confirmation = 0;
  msg.from_external = true;
  vehicle_command_pub_->publish(msg);
}

void Px4ArucoLandingNode::publish_landing_state()
{
  std_msgs::msg::String msg;
  msg.data = state_name(state_);
  landing_state_pub_->publish(msg);
}

void Px4ArucoLandingNode::publish_target_pose()
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = get_clock()->now();
  msg.header.frame_id = target_pose_frame_id_;
  msg.pose.position.x = target_valid_ ? target_x_ : nan;
  msg.pose.position.y = target_valid_ ? target_y_ : nan;
  msg.pose.position.z = target_valid_ ? target_z_ : nan;

  if (target_valid_) {
    msg.pose.orientation.z = std::sin(target_yaw_ * 0.5);
    msg.pose.orientation.w = std::cos(target_yaw_ * 0.5);
  } else {
    msg.pose.orientation.w = 1.0;
  }

  target_pose_pub_->publish(msg);
}

void Px4ArucoLandingNode::publish_deck_gnss_pose(const rclcpp::Time & now)
{
  const auto estimate = gnss_guidance_->estimate(now.seconds());
  if (!estimate.has_value()) {
    return;
  }

  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = now;
  msg.header.frame_id = target_pose_frame_id_;
  msg.pose.position.x = estimate->position_ned.x();
  msg.pose.position.y = estimate->position_ned.y();
  msg.pose.position.z = estimate->position_ned.z();
  msg.pose.orientation.w = 1.0;
  deck_gnss_pose_pub_->publish(msg);
}

void Px4ArucoLandingNode::publish_marker_pose(const rclcpp::Time & now)
{
  if (!have_marker_pose_ned_ ||
    !visual_guidance_->visual_position(now.seconds()).has_value())
  {
    return;
  }

  geometry_msgs::msg::PoseStamped msg = marker_pose_ned_;
  msg.header.stamp = now;
  marker_pose_pub_->publish(msg);
}

void Px4ArucoLandingNode::publish_active_marker_id()
{
  if (!have_aruco_id_) {
    return;
  }

  std_msgs::msg::Int32 msg;
  msg.data = aruco_id_;
  active_marker_id_pub_->publish(msg);
}

void Px4ArucoLandingNode::publish_estimated_deck_odometry(const rclcpp::Time & now)
{
  if (!have_estimator_measurement_receipt_time_) {
    return;
  }

  const double observation_age_s =
    now.seconds() - last_estimator_measurement_receipt_time_s_;
  if (!std::isfinite(observation_age_s) ||
    observation_age_s < 0.0 ||
    observation_age_s > estimator_output_timeout_s_)
  {
    return;
  }

  const auto estimate = target_state_estimator_->estimate();
  if (!estimate.has_value()) {
    return;
  }

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = now;
  msg.header.frame_id = target_pose_frame_id_;
  msg.child_frame_id = estimated_deck_child_frame_id_;
  msg.pose.pose.position.x = estimate->position_ned.x();
  msg.pose.pose.position.y = estimate->position_ned.y();
  msg.pose.pose.position.z = estimate->position_ned.z();
  msg.pose.pose.orientation.w = 1.0;
  msg.twist.twist.linear.x = estimate->velocity_ned.x();
  msg.twist.twist.linear.y = estimate->velocity_ned.y();
  msg.twist.twist.linear.z = estimate->velocity_ned.z();

  msg.pose.covariance.fill(0.0);
  msg.twist.covariance.fill(0.0);
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      msg.pose.covariance[row * 6 + column] =
        estimate->covariance(row, column);
      msg.twist.covariance[row * 6 + column] =
        estimate->covariance(row + 3, column + 3);
    }
  }

  constexpr double kUnestimatedVariance = 1.0e6;
  msg.pose.covariance[21] = kUnestimatedVariance;
  msg.pose.covariance[28] = kUnestimatedVariance;
  msg.pose.covariance[35] = kUnestimatedVariance;
  msg.twist.covariance[21] = kUnestimatedVariance;
  msg.twist.covariance[28] = kUnestimatedVariance;
  msg.twist.covariance[35] = kUnestimatedVariance;
  estimated_deck_odometry_pub_->publish(msg);
}

void Px4ArucoLandingNode::publish_predicted_deck_pose(const rclcpp::Time & now)
{
  if (!have_estimator_measurement_receipt_time_) {
    return;
  }

  const double valid_measurement_age_s =
    now.seconds() - last_estimator_measurement_receipt_time_s_;
  const double estimator_state_age_s =
    now.seconds() - last_estimator_state_receipt_time_s_;
  if (!std::isfinite(valid_measurement_age_s) ||
    !std::isfinite(estimator_state_age_s) ||
    valid_measurement_age_s < 0.0 ||
    estimator_state_age_s < 0.0 ||
    valid_measurement_age_s > estimator_output_timeout_s_)
  {
    return;
  }

  const auto estimate = target_state_estimator_->estimate();
  if (!estimate.has_value()) {
    return;
  }
  const auto prediction = motion_predictor_->predict(*estimate, estimator_state_age_s);
  if (!prediction.has_value()) {
    return;
  }

  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = now;
  msg.header.frame_id = target_pose_frame_id_;
  msg.pose.position.x = prediction->position_ned.x();
  msg.pose.position.y = prediction->position_ned.y();
  msg.pose.position.z = prediction->position_ned.z();
  msg.pose.orientation.w = 1.0;
  predicted_deck_pose_pub_->publish(msg);
}

void Px4ArucoLandingNode::publish_tracking_velocity_setpoint(
  const rclcpp::Time & now)
{
  if (!velocity_feedforward_valid_ && !vertical_velocity_feedforward_valid_) {
    return;
  }

  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = now;
  msg.header.frame_id = target_pose_frame_id_;
  msg.twist.linear.x = velocity_feedforward_valid_ ?
    velocity_feedforward_north_mps_ : 0.0;
  msg.twist.linear.y = velocity_feedforward_valid_ ?
    velocity_feedforward_east_mps_ : 0.0;
  msg.twist.linear.z = vertical_velocity_feedforward_valid_ ?
    vertical_velocity_feedforward_down_mps_ : 0.0;
  tracking_velocity_setpoint_pub_->publish(msg);
}

void Px4ArucoLandingNode::publish_effective_relative_velocity_gain()
{
  if (!effective_relative_velocity_gain_valid_) {
    return;
  }

  std_msgs::msg::Float64 msg;
  msg.data = effective_relative_velocity_gain_;
  effective_relative_velocity_gain_pub_->publish(msg);
}

void Px4ArucoLandingNode::publish_estimated_deck_acceleration(
  const rclcpp::Time & now)
{
  if (!estimated_deck_acceleration_valid_) {
    return;
  }

  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = now;
  msg.header.frame_id = target_pose_frame_id_;
  msg.twist.linear.x = estimated_deck_acceleration_xy_.x();
  msg.twist.linear.y = estimated_deck_acceleration_xy_.y();
  msg.twist.linear.z = 0.0;
  estimated_deck_acceleration_pub_->publish(msg);
}

void Px4ArucoLandingNode::publish_estimated_deck_attitude()
{
  if (!have_estimated_deck_attitude_) {
    return;
  }

  geometry_msgs::msg::Vector3Stamped msg;
  msg.header.stamp = last_estimated_deck_attitude_time_;
  msg.header.frame_id = target_pose_frame_id_;
  msg.vector.x = estimated_deck_attitude_.roll_rad;
  msg.vector.y = estimated_deck_attitude_.pitch_rad;
  msg.vector.z = estimated_deck_attitude_.tilt_rad;
  estimated_deck_attitude_pub_->publish(msg);
}

void Px4ArucoLandingNode::publish_landing_window_debug()
{
  if (!landing_window_result_valid_) {
    return;
  }

  std_msgs::msg::Bool open_msg;
  open_msg.data = landing_window_result_.window_open;
  landing_window_open_pub_->publish(open_msg);

  std_msgs::msg::UInt32 reasons_msg;
  reasons_msg.data = landing_window_result_.reject_reasons;
  landing_window_reject_reasons_pub_->publish(reasons_msg);

  std_msgs::msg::Float64 duration_msg;
  duration_msg.data = landing_window_result_.satisfied_duration_s;
  landing_window_satisfied_duration_pub_->publish(duration_msg);
}

void Px4ArucoLandingNode::publish_relative_descent_debug()
{
  if (!relative_descent_debug_valid_ && !final_descent_debug_valid_) {
    return;
  }

  std_msgs::msg::Float64 height_msg;
  height_msg.data = relative_height_m_;
  relative_height_pub_->publish(height_msg);

  std_msgs::msg::Float64 reference_msg;
  reference_msg.data = relative_height_reference_m_;
  relative_height_reference_pub_->publish(reference_msg);

  if (relative_descent_debug_valid_) {
    std_msgs::msg::String phase_msg;
    phase_msg.data = relative_descent_phase_name(relative_descent_phase_);
    descent_phase_pub_->publish(phase_msg);
  }
}

void Px4ArucoLandingNode::publish_vertical_state(const rclcpp::Time & now)
{
  if (!vertical_state_estimator_enabled_ || !vertical_state_measurement_valid_) {
    return;
  }

  const double observation_age_s =
    now.seconds() - last_vertical_state_measurement_receipt_time_s_;
  if (!std::isfinite(observation_age_s) || observation_age_s < 0.0 ||
    observation_age_s > estimator_output_timeout_s_)
  {
    return;
  }

  const auto estimate = vertical_state_estimator_->estimate();
  if (!estimate.has_value()) {
    return;
  }

  const double state_age_s = now.seconds() - estimate->sample_time_s;
  if (!std::isfinite(state_age_s) || state_age_s < 0.0) {
    return;
  }
  const double prediction_horizon_s = std::clamp(
    state_age_s + vertical_prediction_horizon_s_, 0.0, 0.50);
  const double predicted_z_ned_m =
    estimate->deck_z_ned_m +
    estimate->deck_vertical_velocity_ned_mps * prediction_horizon_s;
  if (!std::isfinite(predicted_z_ned_m)) {
    return;
  }

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = now;
  msg.header.frame_id = target_pose_frame_id_;
  msg.child_frame_id = "vertical_estimated_deck";
  msg.pose.pose.position.z = predicted_z_ned_m;
  msg.pose.pose.orientation.w = 1.0;
  msg.twist.twist.linear.z = estimate->deck_vertical_velocity_ned_mps;
  msg.pose.covariance.fill(0.0);
  msg.twist.covariance.fill(0.0);
  constexpr double kUnestimatedVariance = 1.0e6;
  msg.pose.covariance[0] = kUnestimatedVariance;
  msg.pose.covariance[7] = kUnestimatedVariance;
  msg.pose.covariance[14] = estimate->covariance(0, 0);
  msg.pose.covariance[21] = kUnestimatedVariance;
  msg.pose.covariance[28] = kUnestimatedVariance;
  msg.pose.covariance[35] = kUnestimatedVariance;
  msg.twist.covariance[0] = kUnestimatedVariance;
  msg.twist.covariance[7] = kUnestimatedVariance;
  msg.twist.covariance[14] = estimate->covariance(1, 1);
  msg.twist.covariance[21] = kUnestimatedVariance;
  msg.twist.covariance[28] = kUnestimatedVariance;
  msg.twist.covariance[35] = kUnestimatedVariance;
  vertical_state_pub_->publish(msg);
}

void Px4ArucoLandingNode::publish_raw_relative_height(const rclcpp::Time & now)
{
  const double observation_age_s =
    now.seconds() - last_vertical_state_measurement_receipt_time_s_;
  if (!raw_relative_height_valid_ || !std::isfinite(observation_age_s) ||
    observation_age_s < 0.0 || observation_age_s > estimator_output_timeout_s_)
  {
    return;
  }

  std_msgs::msg::Float64 msg;
  msg.data = raw_relative_height_m_;
  raw_relative_height_pub_->publish(msg);
}

void Px4ArucoLandingNode::publish_relative_vertical_velocity(
  const rclcpp::Time & now)
{
  const double observation_age_s =
    now.seconds() - last_vertical_state_measurement_receipt_time_s_;
  if (!vertical_state_estimator_enabled_ || !vertical_state_measurement_valid_ ||
    !std::isfinite(observation_age_s) || observation_age_s < 0.0 ||
    observation_age_s > estimator_output_timeout_s_ ||
    !local_position_.v_z_valid || !std::isfinite(local_position_.vz))
  {
    return;
  }

  const auto estimate = vertical_state_estimator_->estimate();
  if (!estimate.has_value()) {
    return;
  }
  const double relative_vertical_velocity_mps =
    estimate->deck_vertical_velocity_ned_mps - local_position_.vz;
  if (!std::isfinite(relative_vertical_velocity_mps)) {
    return;
  }

  std_msgs::msg::Float64 msg;
  msg.data = relative_vertical_velocity_mps;
  relative_vertical_velocity_pub_->publish(msg);
}

void Px4ArucoLandingNode::publish_final_descent_debug()
{
  if (!final_descent_debug_valid_) {
    return;
  }

  std_msgs::msg::String msg;
  msg.data = final_descent_phase_name(final_descent_output_.phase);
  final_descent_phase_pub_->publish(msg);
}

void Px4ArucoLandingNode::publish_touchdown_debug()
{
  if (!touchdown_result_valid_) {
    return;
  }

  std_msgs::msg::String status_msg;
  status_msg.data = touchdown_status_name(touchdown_result_.status);
  touchdown_status_pub_->publish(status_msg);

  std_msgs::msg::UInt32 evidence_msg;
  evidence_msg.data = touchdown_result_.evidence_mask;
  touchdown_evidence_pub_->publish(evidence_msg);

  std_msgs::msg::Float64 duration_msg;
  duration_msg.data = touchdown_result_.candidate_duration_s;
  touchdown_candidate_duration_pub_->publish(duration_msg);

  std_msgs::msg::Bool confirmed_msg;
  confirmed_msg.data = touchdown_result_.confirmed_latched;
  touchdown_confirmed_pub_->publish(confirmed_msg);
}

void Px4ArucoLandingNode::publish_guidance_source()
{
  std_msgs::msg::String msg;
  switch (state_) {
    case LandingState::WAIT_DECK_GNSS:
      msg.data = "GNSS_WAIT";
      break;
    case LandingState::RENDEZVOUS_GNSS:
      msg.data = "GNSS_RENDEZVOUS";
      break;
    case LandingState::ACQUIRE_ARUCO:
      msg.data = "GNSS_SEARCH";
      break;
    case LandingState::VISUAL_HANDOVER:
      msg.data = "BLENDING";
      break;
    case LandingState::TRACK_TARGET:
      switch (tracking_controller_->mode()) {
        case TrackingControlMode::kRawVisualPosition:
          msg.data = "VISION_RAW";
          break;
        case TrackingControlMode::kEstimatedPosition:
          msg.data = "VISION_ESTIMATED";
          break;
        case TrackingControlMode::kEstimatedPositionVelocityFeedforward:
          msg.data = "VISION_ESTIMATED_FF";
          break;
        case TrackingControlMode::kPredictedPositionVelocityFeedforward:
          msg.data = "VISION_PREDICTED_FF";
          break;
      }
      break;
    case LandingState::WAIT_LANDING_WINDOW:
      if (descent_reentry_locked_) {
        msg.data = "LANDING_WINDOW_REAUTH_REQUIRED";
      } else {
        msg.data = landing_window_result_valid_ && landing_window_result_.window_open ?
          "LANDING_WINDOW_OPEN" : "LANDING_WINDOW_WAIT";
      }
      break;
    case LandingState::DESCEND:
      msg.data = "RELATIVE_DESCENT";
      break;
    case LandingState::TEST_HEIGHT_HOLD:
      msg.data = "TEST_HEIGHT_HOLD";
      break;
    case LandingState::FINAL_DESCENT:
      msg.data = "FINAL_DESCENT";
      break;
    case LandingState::TOUCHDOWN_CANDIDATE_HOLD:
      msg.data = "TOUCHDOWN_CANDIDATE_HOLD";
      break;
    case LandingState::TOUCHDOWN_HOLD:
      msg.data = "TOUCHDOWN_HOLD";
      break;
    case LandingState::RECOVER_CLIMB:
      msg.data = "RECOVER_CLIMB";
      break;
    case LandingState::RECOVER_TO_GNSS:
      msg.data = "GNSS_RECOVERY";
      break;
    case LandingState::CENTER_ABOVE_MARKER:
    case LandingState::DESCEND_WITH_TRACKING:
      msg.data = "LEGACY_VISION";
      break;
    default:
      msg.data = "NONE";
      break;
  }
  guidance_source_pub_->publish(msg);
}

const char * Px4ArucoLandingNode::state_name(LandingState state)
{
  switch (state) {
    case LandingState::INIT:
      return "INIT";
    case LandingState::WAIT_FOR_PX4:
      return "WAIT_FOR_PX4";
    case LandingState::OFFBOARD_PRE_STREAM:
      return "OFFBOARD_PRE_STREAM";
    case LandingState::ARM_AND_TAKEOFF:
      return "ARM_AND_TAKEOFF";
    case LandingState::WAIT_DECK_GNSS:
      return "WAIT_DECK_GNSS";
    case LandingState::RENDEZVOUS_GNSS:
      return "RENDEZVOUS_GNSS";
    case LandingState::ACQUIRE_ARUCO:
      return "ACQUIRE_ARUCO";
    case LandingState::VISUAL_HANDOVER:
      return "VISUAL_HANDOVER";
    case LandingState::TRACK_TARGET:
      return "TRACK_TARGET";
    case LandingState::WAIT_LANDING_WINDOW:
      return "WAIT_LANDING_WINDOW";
    case LandingState::DESCEND:
      return "DESCEND";
    case LandingState::TEST_HEIGHT_HOLD:
      return "TEST_HEIGHT_HOLD";
    case LandingState::FINAL_DESCENT:
      return "FINAL_DESCENT";
    case LandingState::TOUCHDOWN_CANDIDATE_HOLD:
      return "TOUCHDOWN_CANDIDATE_HOLD";
    case LandingState::TOUCHDOWN_HOLD:
      return "TOUCHDOWN_HOLD";
    case LandingState::RECOVER_CLIMB:
      return "RECOVER_CLIMB";
    case LandingState::RECOVER_TO_GNSS:
      return "RECOVER_TO_GNSS";
    case LandingState::GOTO_ARUCO_AREA:
      return "GOTO_ARUCO_AREA";
    case LandingState::WAIT_ARUCO:
      return "WAIT_ARUCO";
    case LandingState::CENTER_ABOVE_MARKER:
      return "CENTER_ABOVE_MARKER";
    case LandingState::DESCEND_WITH_TRACKING:
      return "DESCEND_WITH_TRACKING";
    case LandingState::FINAL_LAND:
      return "FINAL_LAND";
    case LandingState::DONE:
      return "DONE";
    case LandingState::ABORT:
      return "ABORT";
  }
  return "UNKNOWN";
}

const char * Px4ArucoLandingNode::relative_descent_phase_name(
  RelativeDescentPhase phase)
{
  switch (phase) {
    case RelativeDescentPhase::kWaitingWindow:
      return "WAITING_WINDOW";
    case RelativeDescentPhase::kDescending:
      return "DESCENDING";
    case RelativeDescentPhase::kPaused:
      return "PAUSED";
    case RelativeDescentPhase::kRecovering:
      return "RECOVERING";
    case RelativeDescentPhase::kTestHeightHold:
      return "TEST_HEIGHT_HOLD";
  }
  return "UNKNOWN";
}

const char * Px4ArucoLandingNode::final_descent_phase_name(FinalDescentPhase phase)
{
  switch (phase) {
    case FinalDescentPhase::kWaitingAuthorization:
      return "WAITING_AUTHORIZATION";
    case FinalDescentPhase::kDescending:
      return "DESCENDING";
    case FinalDescentPhase::kCandidateHold:
      return "CANDIDATE_HOLD";
    case FinalDescentPhase::kTouchdownHold:
      return "TOUCHDOWN_HOLD";
    case FinalDescentPhase::kPaused:
      return "PAUSED";
    case FinalDescentPhase::kRecoveryRequested:
      return "RECOVERY_REQUESTED";
  }
  return "UNKNOWN";
}

const char * Px4ArucoLandingNode::touchdown_status_name(TouchdownStatus status)
{
  switch (status) {
    case TouchdownStatus::kInsufficientEvidence:
      return "INSUFFICIENT_EVIDENCE";
    case TouchdownStatus::kAirborne:
      return "AIRBORNE";
    case TouchdownStatus::kCandidate:
      return "CANDIDATE";
    case TouchdownStatus::kConfirmed:
      return "CONFIRMED";
    case TouchdownStatus::kRejectedUnsafe:
      return "REJECTED_UNSAFE";
  }
  return "UNKNOWN";
}

double Px4ArucoLandingNode::quaternion_to_yaw(const float q[4])
{
  const double norm = std::sqrt(
    static_cast<double>(q[0]) * q[0] +
    static_cast<double>(q[1]) * q[1] +
    static_cast<double>(q[2]) * q[2] +
    static_cast<double>(q[3]) * q[3]);
  const double w = q[0] / norm;
  const double x = q[1] / norm;
  const double y = q[2] / norm;
  const double z = q[3] / norm;
  return std::atan2(
    2.0 * (w * z + x * y),
    1.0 - 2.0 * (y * y + z * z));
}

bool Px4ArucoLandingNode::quaternion_is_valid(const float q[4])
{
  if (!std::isfinite(q[0]) ||
    !std::isfinite(q[1]) ||
    !std::isfinite(q[2]) ||
    !std::isfinite(q[3]))
  {
    return false;
  }

  const double norm_squared =
    static_cast<double>(q[0]) * q[0] +
    static_cast<double>(q[1]) * q[1] +
    static_cast<double>(q[2]) * q[2] +
    static_cast<double>(q[3]) * q[3];
  return norm_squared > kMinimumQuaternionNorm * kMinimumQuaternionNorm;
}

}  // namespace aruco_precision_landing_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<aruco_precision_landing_cpp::Px4ArucoLandingNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("px4_aruco_landing_node"),
      "Failed to start PX4 ArUco landing controller: %s",
      exception.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
