// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCO_PRECISION_LANDING_CPP__PX4_ARUCO_LANDING_NODE_HPP_
#define ARUCO_PRECISION_LANDING_CPP__PX4_ARUCO_LANDING_NODE_HPP_

#include "aruco_precision_landing_cpp/coordinate_transform.hpp"
#include "aruco_precision_landing_cpp/deck_attitude_estimator.hpp"
#include "aruco_precision_landing_cpp/deck_plane_geometry.hpp"
#include "aruco_precision_landing_cpp/final_descent_controller.hpp"
#include "aruco_precision_landing_cpp/gnss_rendezvous_guidance.hpp"
#include "aruco_precision_landing_cpp/landing_window.hpp"
#include "aruco_precision_landing_cpp/motion_predictor.hpp"
#include "aruco_precision_landing_cpp/moving_target_tracking_controller.hpp"
#include "aruco_precision_landing_cpp/relative_descent_controller.hpp"
#include "aruco_precision_landing_cpp/relative_mpc_controller.hpp"
#include "aruco_precision_landing_cpp/target_state_estimator.hpp"
#include "aruco_precision_landing_cpp/terminal_contact_stabilization.hpp"
#include "aruco_precision_landing_cpp/touchdown_detector.hpp"
#include "aruco_precision_landing_cpp/touchdown_hold_controller.hpp"
#include "aruco_precision_landing_cpp/vehicle_pose_history.hpp"
#include "aruco_precision_landing_cpp/vertical_state_estimator.hpp"
#include "aruco_precision_landing_cpp/visual_handover_guidance.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>

namespace aruco_precision_landing_cpp
{

enum class LandingState
{
  INIT,
  WAIT_FOR_PX4,
  OFFBOARD_PRE_STREAM,
  ARM_AND_TAKEOFF,
  WAIT_DECK_GNSS,
  RENDEZVOUS_GNSS,
  ACQUIRE_ARUCO,
  VISUAL_HANDOVER,
  TRACK_TARGET,
  WAIT_LANDING_WINDOW,
  DESCEND,
  TEST_HEIGHT_HOLD,
  FINAL_DESCENT,
  TOUCHDOWN_CANDIDATE_HOLD,
  TOUCHDOWN_HOLD,
  RECOVER_CLIMB,
  RECOVER_TO_GNSS,
  GOTO_ARUCO_AREA,
  WAIT_ARUCO,
  CENTER_ABOVE_MARKER,
  DESCEND_WITH_TRACKING,
  FINAL_LAND,
  DONE,
  ABORT
};

class Px4ArucoLandingNode : public rclcpp::Node
{
public:
  Px4ArucoLandingNode();

private:
  void declare_and_load_parameters();
  void validate_parameters() const;
  void create_ros_interfaces();

  void aruco_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void aruco_visible_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void aruco_id_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void deck_gps_fix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void deck_gps_velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void vehicle_land_detected_callback(
    const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);
  void vehicle_local_position_callback(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

  void control_timer_callback();
  void run_state_machine(const rclcpp::Time & now, double dt);
  void transition_to(LandingState new_state, const char * reason = "unspecified");

  bool px4_data_ready() const;
  bool marker_is_fresh(const rclcpp::Time & now) const;
  bool marker_is_stably_visible(const rclcpp::Time & now) const;
  bool should_retry_command(const rclcpp::Time & now) const;

  /**
   * @brief 更新 PX4→ROS 时钟偏移，并返回里程计位姿的 ROS 采样时间。
   *
   * @param odometry PX4 local NED 里程计；`timestamp` 和 `timestamp_sample` 单位为微秒。
   * @param receipt_time ROS 回调接收时间。
   * @return 映射到 ROS 时间域的位姿采样时间，单位为秒；时间戳无效时返回
   *         `std::nullopt`。检测到时钟回退或大幅跳变时会清空旧位姿历史。
   */
  std::optional<double> update_px4_to_ros_time_offset(
    const px4_msgs::msg::VehicleOdometry & odometry,
    const rclcpp::Time & receipt_time);

  /**
   * @brief 使用图像采样时刻的机体位姿将 Marker 转换到 PX4 local NED。
   *
   * @param image_sample_time_s 图像 header 中的 ROS 采样时间，单位为秒。
   * @param marker_pose_ned 输出 Marker 在 local NED 中的位姿，位置单位为米。
   * @return 位姿历史可插值且完整相机变换有效时返回 true，否则返回 false。
   */
  bool compute_marker_pose_ned(
    double image_sample_time_s,
    Pose3d & marker_pose_ned) const;
  bool compute_local_marker_error(double & error_north, double & error_east) const;
  void update_estimated_deck_attitude(
    const Eigen::Quaterniond & marker_to_ned_rotation,
    const rclcpp::Time & sample_time);
  void update_deck_plane_geometry_shadow(const rclcpp::Time & now);
  void update_landing_window(
    const rclcpp::Time & now,
    bool visual_valid,
    double visual_age_s,
    const std::optional<TargetStateEstimate> & estimate,
    const std::optional<Eigen::Vector2d> & predicted_position_xy,
    const std::optional<Eigen::Vector2d> & uav_velocity_xy);
  std::optional<RelativeDescentOutput> update_relative_descent(
    const std::optional<TargetStateEstimate> & estimate,
    const std::optional<Eigen::Vector3d> & predicted_position_ned,
    bool visual_valid,
    VisualLossState visual_loss_state,
    double dt);
  std::optional<FinalDescentOutput> update_final_descent(
    const std::optional<TargetStateEstimate> & estimate,
    bool visual_valid,
    double dt);
  std::optional<VerticalStateEstimate> predicted_vertical_state(
    const rclcpp::Time & now,
    bool include_additional_prediction_horizon = true) const;
  std::optional<TouchdownHoldOutput> update_touchdown_hold(
    const rclcpp::Time & now,
    double dt);
  void update_touchdown_detection(const rclcpp::Time & now);
  TerminalStabilizationPhase terminal_stabilization_phase() const;
  bool update_terminal_contact_stabilization(
    const rclcpp::Time & now,
    double dt,
    const std::optional<TargetStateEstimate> & estimate,
    const std::optional<Eigen::Vector2d> & uav_velocity_xy,
    Eigen::Vector2d & horizontal_target_xy);

  void set_target(double x, double y, double z, double yaw);
  void set_velocity_feedforward(double north_mps, double east_mps);
  void set_horizontal_acceleration_feedforward(double north_mps2, double east_mps2);
  void set_vertical_acceleration_feedforward(double down_mps2);
  void set_vertical_velocity_feedforward(double down_mps);
  void set_adaptive_tracking_debug(
    const std::optional<double> & effective_gain,
    const std::optional<Eigen::Vector2d> & estimated_deck_acceleration_xy);
  void clear_velocity_feedforward();
  void publish_offboard_control_mode();
  void publish_trajectory_setpoint();
  void publish_vehicle_command(
    uint32_t command,
    float param1 = 0.0F,
    float param2 = 0.0F,
    float param3 = 0.0F,
    float param4 = 0.0F,
    double param5 = 0.0,
    double param6 = 0.0,
    float param7 = 0.0F);
  void publish_landing_state();
  void publish_target_pose();
  void publish_deck_gnss_pose(const rclcpp::Time & now);
  void publish_marker_pose(const rclcpp::Time & now);
  void publish_active_marker_id();
  void publish_estimated_deck_odometry(const rclcpp::Time & now);
  void publish_predicted_deck_pose(const rclcpp::Time & now);
  void publish_tracking_velocity_setpoint(const rclcpp::Time & now);
  void publish_relative_mpc_debug(const rclcpp::Time & now);
  void publish_effective_relative_velocity_gain();
  void publish_estimated_deck_acceleration(const rclcpp::Time & now);
  void publish_estimated_deck_attitude();
  void publish_deck_plane_geometry_shadow(const rclcpp::Time & now);
  void publish_deck_normal_calibration_debug();
  void publish_terminal_stabilization_debug(const rclcpp::Time & now);
  void publish_landing_window_debug();
  void publish_relative_descent_debug();
  void publish_vertical_state(const rclcpp::Time & now);
  void publish_raw_relative_height(const rclcpp::Time & now);
  void publish_relative_vertical_velocity(const rclcpp::Time & now);
  void publish_uav_vertical_velocity();
  void publish_touchdown_hold_debug();
  void publish_final_descent_debug();
  void publish_touchdown_debug();
  void publish_guidance_source();

  static const char * state_name(LandingState state);
  static const char * relative_descent_phase_name(RelativeDescentPhase phase);
  static const char * final_descent_phase_name(FinalDescentPhase phase);
  static const char * touchdown_status_name(TouchdownStatus status);
  static const char * touchdown_hold_mode_name(TouchdownHoldMode mode);
  static const char * touchdown_hold_reason_name(TouchdownHoldReason reason);
  static double quaternion_to_yaw(const float q[4]);
  static Eigen::Vector2d quaternion_to_roll_pitch(const float q[4]);
  static bool quaternion_is_valid(const float q[4]);

  double control_rate_hz_{20.0};
  double takeoff_alt_{3.0};
  double search_x_{0.0};
  double search_y_{0.0};
  double search_alt_{3.0};
  double abort_hover_alt_{3.0};
  double rendezvous_altitude_m_{5.0};
  double rendezvous_radius_m_{2.0};
  double gnss_fix_timeout_s_{1.0};
  double gnss_velocity_timeout_s_{1.0};
  double gnss_stable_duration_s_{1.0};
  double max_gnss_jump_m_{5.0};
  double max_rendezvous_speed_mps_{2.0};
  double max_target_step_m_{0.20};
  double search_offset_m_{1.0};
  double search_point_hold_s_{1.0};
  double aruco_acquire_duration_s_{0.5};
  double gnss_max_geodetic_range_m_{10000.0};
  double visual_handover_duration_s_{0.5};
  double handover_max_horizontal_difference_m_{3.0};
  double max_visual_measurement_jump_m_{0.5};
  double visual_loss_short_timeout_s_{0.5};
  double visual_loss_long_timeout_s_{2.0};
  double estimator_process_acceleration_std_mps2_{1.0};
  double estimator_measurement_horizontal_std_m_{0.08};
  double estimator_measurement_vertical_std_m_{0.12};
  double estimator_initial_position_std_m_{0.20};
  double estimator_initial_velocity_std_mps_{1.0};
  double estimator_minimum_sample_dt_s_{0.001};
  double estimator_maximum_sample_dt_s_{0.50};
  double estimator_reinitialize_gap_s_{2.0};
  double estimator_innovation_gate_mahalanobis_{5.0};
  bool vertical_state_estimator_enabled_{true};
  double vertical_process_acceleration_std_mps2_{0.40};
  double vertical_measurement_std_m_{0.05};
  double vertical_measurement_bias_m_{0.0};
  double vertical_initial_position_std_m_{0.10};
  double vertical_initial_velocity_std_mps_{0.50};
  double vertical_minimum_sample_dt_s_{0.001};
  double vertical_maximum_sample_dt_s_{0.25};
  double vertical_reinitialize_gap_s_{2.0};
  double vertical_innovation_gate_mahalanobis_{5.0};
  double vertical_prediction_horizon_s_{0.10};
  bool vertical_velocity_feedforward_enabled_{true};
  double vertical_velocity_feedforward_gain_{1.0};
  double vertical_velocity_feedforward_max_mps_{0.60};
  bool touchdown_detector_enabled_{true};
  double touchdown_px4_status_timeout_s_{0.20};
  double touchdown_visual_timeout_s_{0.20};
  double touchdown_low_height_enter_m_{0.18};
  double touchdown_low_height_exit_m_{0.28};
  double touchdown_max_relative_vertical_speed_mps_{0.12};
  double touchdown_max_uav_vertical_speed_mps_{0.15};
  double touchdown_max_relative_horizontal_speed_mps_{0.15};
  double touchdown_terminal_contact_max_height_m_{0.24};
  double touchdown_terminal_contact_min_reference_error_m_{0.10};
  double touchdown_terminal_contact_max_geometry_gap_m_{0.03};
  double touchdown_terminal_contact_max_vertical_speed_mps_{0.05};
  double touchdown_terminal_contact_px4_status_timeout_s_{2.0};
  double touchdown_candidate_required_duration_s_{0.50};
  double touchdown_hold_max_target_rate_mps_{0.60};
  double touchdown_hold_max_reference_preload_rate_mps_{0.05};
  double touchdown_hold_motion_enter_speed_mps_{0.04};
  double touchdown_hold_motion_exit_speed_mps_{0.02};
  bool final_descent_enabled_{false};
  double final_descent_entry_height_m_{0.50};
  double final_descent_approach_rate_mps_{0.12};
  double final_descent_contact_rate_mps_{0.03};
  double final_descent_contact_slowdown_height_m_{0.25};
  double final_descent_terminal_entry_height_m_{0.20};
  double final_descent_minimum_command_height_m_{0.05};
  double final_descent_max_reference_tracking_error_m_{0.20};
  double additional_prediction_horizon_s_{0.10};
  double max_prediction_horizon_s_{0.50};
  double estimator_output_timeout_s_{2.0};
  double tracking_max_position_target_speed_mps_{2.0};
  double tracking_max_position_target_step_m_{0.20};
  double tracking_velocity_feedforward_gain_{1.0};
  double tracking_relative_velocity_gain_{0.25};
  bool tracking_adaptive_relative_velocity_gain_enabled_{true};
  double tracking_adaptive_relative_velocity_gain_min_{0.25};
  double tracking_adaptive_relative_velocity_gain_max_{1.2};
  double tracking_adaptive_acceleration_low_threshold_mps2_{0.05};
  double tracking_adaptive_acceleration_high_threshold_mps2_{0.35};
  double tracking_adaptive_max_acceleration_mps2_{1.50};
  double tracking_adaptive_acceleration_filter_gain_{0.20};
  double tracking_max_velocity_feedforward_mps_{1.5};
  double tracking_max_velocity_feedforward_acceleration_mps2_{1.0};
  double tracking_max_prediction_age_s_{0.75};
  double relative_mpc_sample_period_s_{0.05};
  int relative_mpc_horizon_steps_{20};
  std::array<double, 4> relative_mpc_state_weights_{{8.0, 8.0, 2.0, 2.0}};
  std::array<double, 4> relative_mpc_terminal_state_weights_{{16.0, 16.0, 4.0, 4.0}};
  std::array<double, 2> relative_mpc_control_weights_{{0.20, 0.20}};
  std::array<double, 2> relative_mpc_control_increment_weights_{{1.0, 1.0}};
  double relative_mpc_speed_slack_weight_{1000.0};
  double relative_mpc_maximum_uav_speed_mps_{2.0};
  double relative_mpc_maximum_acceleration_mps2_{1.5};
  double relative_mpc_maximum_acceleration_increment_mps2_{0.25};
  double relative_mpc_maximum_speed_slack_mps_{2.0};
  int relative_mpc_maximum_iterations_{1000};
  double relative_mpc_absolute_tolerance_{1.0e-4};
  double relative_mpc_relative_tolerance_{1.0e-4};
  double relative_mpc_time_limit_s_{0.02};
  bool relative_mpc_warm_start_enabled_{true};
  double relative_mpc_active_constraint_tolerance_{1.0e-3};
  double deck_attitude_filter_gain_{0.20};
  double deck_attitude_minimum_upward_normal_component_{0.50};
  bool deck_plane_geometry_enabled_{true};
  double deck_plane_geometry_normal_filter_gain_{0.08};
  bool deck_plane_geometry_shadow_only_{true};
  double deck_plane_geometry_minimum_normal_norm_{1.0e-6};
  double deck_plane_geometry_minimum_upward_component_{0.50};
  bool deck_plane_geometry_apply_marker_plane_offset_{true};
  std::array<double, 12> deck_plane_geometry_contact_points_body_frd_m_{{
    -0.125, -0.132, 0.227,
    0.125, -0.132, 0.227,
    -0.125, 0.132, 0.227,
    0.125, 0.132, 0.227}};
  std::array<double, 4> deck_plane_geometry_marker_plane_offsets_m_{{
    0.001, 0.002, 0.003, 0.004}};
  bool terminal_contact_stabilization_enabled_{false};
  bool terminal_contact_stabilization_shadow_only_{true};
  bool terminal_contact_stabilization_rehearsal_enabled_{false};
  std::string terminal_contact_stabilization_scenario_{"none"};
  double terminal_contact_maximum_target_tilt_deg_{2.5};
  double terminal_contact_normal_freshness_timeout_s_{0.20};
  double terminal_contact_short_loss_hold_s_{0.10};
  double terminal_contact_marker_switch_jump_gate_deg_{1.0};
  double terminal_contact_tilt_slew_rate_degps_{4.0};
  double terminal_contact_acceleration_bias_limit_mps2_{0.45};
  double terminal_contact_acceleration_bias_slew_rate_mps3_{0.80};
  double terminal_contact_activation_duration_s_{0.50};
  double terminal_contact_deactivation_duration_s_{0.30};
  double terminal_contact_total_acceleration_limit_mps2_{1.50};
  double terminal_contact_rehearsal_max_duration_s_{1.0};
  double terminal_contact_preload_relative_height_m_{0.20};
  double terminal_contact_preload_acceleration_mps2_{1.0};
  double terminal_contact_preload_acceleration_slew_mps3_{1.0};
  bool terminal_contact_compliance_enabled_{true};
  double terminal_contact_compliance_deadband_m_{0.015};
  double terminal_contact_compliance_maximum_allowance_m_{0.040};
  double terminal_contact_compliance_target_rate_mps_{0.10};
  double terminal_contact_compliance_anchor_correction_rate_mps_{0.05};
  double terminal_contact_compliance_deck_velocity_deadband_mps_{0.035};
  double terminal_contact_compliance_velocity_damping_s_{0.12};
  double terminal_contact_compliance_maximum_damping_offset_m_{0.020};
  bool terminal_contact_attitude_safety_enabled_{true};
  double terminal_contact_attitude_trigger_deg_{6.0};
  double terminal_contact_attitude_clear_deg_{4.0};
  double terminal_contact_angular_rate_trigger_degps_{45.0};
  double terminal_contact_safety_required_duration_s_{0.20};
  double terminal_contact_safety_clear_duration_s_{0.30};
  double landing_window_enter_horizontal_error_m_{0.15};
  double landing_window_exit_horizontal_error_m_{0.25};
  double landing_window_enter_relative_speed_mps_{0.15};
  double landing_window_exit_relative_speed_mps_{0.25};
  double landing_window_enter_max_tilt_deg_{5.0};
  double landing_window_exit_max_tilt_deg_{8.0};
  double landing_window_max_visual_age_s_{0.20};
  double landing_window_minimum_relative_height_m_{0.08};
  double landing_window_maximum_relative_height_m_{6.00};
  double landing_window_required_duration_s_{1.00};
  bool relative_descent_enabled_{false};
  double descent_minimum_test_height_m_{0.50};
  double descent_fast_height_threshold_m_{2.00};
  double descent_slow_height_threshold_m_{0.80};
  double descent_fast_rate_mps_{0.50};
  double descent_medium_rate_mps_{0.30};
  double descent_slow_rate_mps_{0.12};
  double descent_recovery_height_m_{2.00};
  double descent_recovery_rate_mps_{0.30};
  double descent_max_reference_tracking_error_m_{0.50};
  double vehicle_pose_history_duration_s_{2.0};
  double vehicle_pose_history_max_endpoint_hold_s_{0.03};
  double px4_clock_offset_filter_gain_{0.05};
  double px4_clock_offset_max_jump_s_{0.10};
  int offboard_prestream_count_{20};
  int stable_detect_count_{10};
  double camera_x_to_body_y_sign_{1.0};
  double camera_y_to_body_x_sign_{-1.0};
  double max_xy_step_{0.20};
  double center_xy_threshold_{0.15};
  double max_descent_rate_{0.20};
  double final_alt_{0.30};
  double marker_lost_timeout_{1.0};
  double aruco_pose_timeout_{0.5};
  double takeoff_z_threshold_{0.20};
  double search_xy_threshold_{0.25};
  double search_z_threshold_{0.20};
  double command_retry_interval_{1.0};
  bool enable_auto_land_{false};
  std::string target_pose_frame_id_{"local_ned"};
  std::string deck_gnss_velocity_frame_id_{"world_enu"};
  std::string expected_aruco_pose_frame_id_{"camera_link"};
  std::string estimated_deck_child_frame_id_{"estimated_deck"};
  std::string tracking_mode_string_{"PREDICTED_POSITION_VELOCITY_FF"};
  std::array<double, 3> camera_translation_frd_m_{{0.0, 0.0, 0.14}};
  std::array<double, 4> camera_rotation_wxyz_{{0.70710678, 0.0, 0.0, 0.70710678}};

  LandingState state_{LandingState::INIT};

  bool have_vehicle_status_{false};
  bool have_local_position_{false};
  bool have_vehicle_odometry_{false};
  bool have_vehicle_land_detected_{false};
  bool have_aruco_pose_{false};
  bool aruco_visible_{false};
  bool have_aruco_id_{false};
  bool have_aruco_visible_since_{false};
  bool have_search_pattern_start_time_{false};
  bool have_marker_pose_ned_{false};
  bool have_last_aruco_sample_stamp_{false};
  bool have_estimator_measurement_receipt_time_{false};
  bool have_estimated_deck_attitude_{false};
  bool have_deck_plane_geometry_sample_{false};
  bool have_shadow_deck_attitude_{false};
  bool have_previous_deck_normal_{false};
  bool deck_normal_rate_valid_{false};
  bool marker_switch_normal_jump_valid_{false};
  bool landing_window_result_valid_{false};
  bool relative_descent_debug_valid_{false};
  bool vertical_state_measurement_valid_{false};
  bool raw_relative_height_valid_{false};
  bool touchdown_result_valid_{false};
  bool final_descent_debug_valid_{false};
  bool touchdown_hold_debug_valid_{false};
  bool terminal_stabilization_debug_valid_{false};
  bool terminal_stabilization_applied_{false};
  bool terminal_combined_acceleration_valid_{false};
  bool descent_reentry_locked_{false};
  bool have_px4_to_ros_time_offset_{false};
  bool have_last_time_sync_observation_{false};
  int32_t aruco_id_{-1};
  int64_t last_aruco_sample_stamp_ns_{0};
  uint64_t last_px4_sync_timestamp_us_{0};

  px4_msgs::msg::VehicleStatus vehicle_status_{};
  px4_msgs::msg::VehicleLocalPosition local_position_{};
  px4_msgs::msg::VehicleOdometry vehicle_odometry_{};
  px4_msgs::msg::VehicleLandDetected vehicle_land_detected_{};
  geometry_msgs::msg::PoseStamped aruco_pose_{};
  geometry_msgs::msg::PoseStamped marker_pose_ned_{};
  Pose3d body_camera_pose_{};
  Pose3d deck_plane_geometry_sample_body_pose_{};
  Eigen::Vector3d deck_plane_geometry_sample_deck_position_ned_m_{Eigen::Vector3d::Zero()};

  rclcpp::Time last_aruco_pose_time_;
  rclcpp::Time last_aruco_visible_time_;
  rclcpp::Time aruco_visible_since_;
  rclcpp::Time search_pattern_start_time_;
  rclcpp::Time last_marker_seen_time_;
  rclcpp::Time last_command_time_;
  rclcpp::Time last_control_time_;
  rclcpp::Time last_estimated_deck_attitude_time_;
  rclcpp::Time last_deck_plane_geometry_sample_time_;
  rclcpp::Time previous_deck_normal_time_;
  bool have_last_marker_seen_time_{false};
  bool have_last_command_time_{false};

  int stable_visible_count_{0};
  int prestream_setpoint_count_{0};
  bool final_land_command_sent_{false};
  double handover_progress_s_{0.0};
  double last_estimator_measurement_receipt_time_s_{0.0};
  double last_estimator_state_receipt_time_s_{0.0};
  double last_vertical_state_measurement_receipt_time_s_{0.0};
  double last_vehicle_land_detected_receipt_time_s_{0.0};
  double raw_relative_height_m_{0.0};
  double deck_normal_rate_degps_{0.0};
  double marker_switch_normal_jump_deg_{0.0};
  double px4_to_ros_time_offset_s_{0.0};
  double last_time_sync_receipt_s_{0.0};

  double takeoff_start_x_{0.0};
  double takeoff_start_y_{0.0};
  double initial_yaw_{0.0};
  double current_yaw_{0.0};

  bool target_valid_{false};
  double target_x_{0.0};
  double target_y_{0.0};
  double target_z_{0.0};
  double target_yaw_{0.0};
  bool velocity_feedforward_valid_{false};
  double velocity_feedforward_north_mps_{0.0};
  double velocity_feedforward_east_mps_{0.0};
  bool horizontal_acceleration_feedforward_valid_{false};
  double horizontal_acceleration_feedforward_north_mps2_{0.0};
  double horizontal_acceleration_feedforward_east_mps2_{0.0};
  bool vertical_acceleration_feedforward_valid_{false};
  double vertical_acceleration_feedforward_down_mps2_{0.0};
  bool vertical_velocity_feedforward_valid_{false};
  double vertical_velocity_feedforward_down_mps_{0.0};
  bool effective_relative_velocity_gain_valid_{false};
  bool estimated_deck_acceleration_valid_{false};
  double effective_relative_velocity_gain_{0.0};
  Eigen::Vector2d estimated_deck_acceleration_xy_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d terminal_base_acceleration_ff_xy_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d terminal_combined_acceleration_ff_xy_{Eigen::Vector2d::Zero()};
  double terminal_vertical_preload_acceleration_mps2_{0.0};
  double terminal_rehearsal_elapsed_s_{0.0};
  std::uint32_t terminal_fallback_count_{0U};
  std::uint32_t terminal_divergence_protection_count_{0U};
  Eigen::Vector3d previous_deck_normal_ned_{Eigen::Vector3d{0.0, 0.0, -1.0}};
  std::array<Eigen::Vector3d, 4> marker_normal_ned_by_id_{{
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()}};
  std::uint32_t marker_normal_valid_mask_{0U};
  int32_t previous_deck_normal_marker_id_{-1};
  int32_t deck_plane_geometry_sample_marker_id_{-1};
  DeckAttitudeEstimate estimated_deck_attitude_{};
  DeckAttitudeEstimate shadow_deck_attitude_{};
  DeckPlaneGeometryResult deck_plane_geometry_result_{};
  std::string deck_plane_geometry_status_{"not updated"};
  LandingWindowResult landing_window_result_{};
  double relative_height_m_{0.0};
  double relative_height_reference_m_{0.0};
  RelativeDescentPhase relative_descent_phase_{RelativeDescentPhase::kWaitingWindow};
  FinalDescentOutput final_descent_output_{};
  TouchdownDetectorOutput touchdown_result_{};
  TouchdownHoldOutput touchdown_hold_output_{};
  TerminalDeckNormalOutput terminal_normal_output_{};
  TerminalContactComplianceOutput terminal_compliance_output_{};
  TerminalAttitudeSafetyOutput terminal_attitude_safety_output_{};
  RelativeMpcResult relative_mpc_result_{};
  bool relative_mpc_debug_valid_{false};
  std::uint32_t relative_mpc_fallback_count_{0U};
  Eigen::Vector2d last_relative_mpc_control_xy_{Eigen::Vector2d::Zero()};

  std::unique_ptr<GnssRendezvousGuidance> gnss_guidance_;
  std::unique_ptr<VisualHandoverGuidance> visual_guidance_;
  std::unique_ptr<TargetStateEstimator> target_state_estimator_;
  std::unique_ptr<VerticalStateEstimator> vertical_state_estimator_;
  std::unique_ptr<MotionPredictor> motion_predictor_;
  std::unique_ptr<MovingTargetTrackingController> tracking_controller_;
  std::unique_ptr<MovingTargetTrackingController> rule_based_fallback_controller_;
  std::unique_ptr<RelativeMpcController> relative_mpc_controller_;
  std::unique_ptr<VehiclePoseHistory> vehicle_pose_history_;
  std::unique_ptr<DeckAttitudeEstimator> deck_attitude_estimator_;
  std::unique_ptr<DeckAttitudeEstimator> deck_plane_shadow_attitude_estimator_;
  std::unique_ptr<LandingWindow> landing_window_;
  std::unique_ptr<RelativeDescentController> relative_descent_controller_;
  std::unique_ptr<TouchdownDetector> touchdown_detector_;
  std::unique_ptr<FinalDescentController> final_descent_controller_;
  std::unique_ptr<TouchdownHoldController> touchdown_hold_controller_;
  std::unique_ptr<TerminalDeckNormalStabilizer> terminal_normal_stabilizer_;
  std::unique_ptr<TerminalContactComplianceController> terminal_compliance_controller_;
  std::unique_ptr<TerminalAttitudeSafetyMonitor> terminal_attitude_safety_monitor_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr aruco_visible_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr aruco_id_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr deck_gps_fix_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr deck_gps_velocity_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr
    vehicle_land_detected_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
    vehicle_local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
    offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr landing_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr deck_gnss_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr marker_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr active_marker_id_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr estimated_deck_odometry_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr predicted_deck_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
    tracking_velocity_setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr relative_mpc_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr relative_mpc_solve_time_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr relative_mpc_iteration_count_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr relative_mpc_objective_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr relative_mpc_fallback_count_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    relative_mpc_first_control_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr relative_mpc_active_constraints_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr relative_mpc_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr relative_mpc_predicted_path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
    effective_relative_velocity_gain_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
    estimated_deck_acceleration_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    estimated_deck_attitude_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    deck_plane_upward_normal_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr deck_plane_body_clearance_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    deck_plane_skid_clearances_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
    deck_plane_minimum_skid_clearance_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
    deck_plane_maximum_skid_clearance_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr deck_plane_clearance_spread_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr
    deck_plane_first_contact_point_index_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
    deck_plane_normal_relative_velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    deck_plane_skid_normal_relative_velocities_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    deck_plane_tangential_position_error_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    deck_plane_tangential_relative_velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr deck_plane_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr deck_normal_rate_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr marker_switch_normal_jump_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr marker_normals_by_id_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr marker_normal_valid_mask_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr terminal_stabilization_enabled_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr terminal_stabilization_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr terminal_stabilization_reason_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    terminal_desired_normal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    terminal_desired_roll_pitch_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    terminal_actual_roll_pitch_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    terminal_attitude_error_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    terminal_acceleration_bias_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    terminal_combined_acceleration_ff_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    terminal_contact_anchor_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    terminal_compliant_target_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr terminal_divergence_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr landing_window_open_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr landing_window_reject_reasons_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
    landing_window_satisfied_duration_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr relative_height_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr relative_height_reference_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr descent_phase_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vertical_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr raw_relative_height_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
    relative_vertical_velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr uav_vertical_velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
    touchdown_hold_relative_height_reference_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
    touchdown_hold_vertical_target_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr touchdown_hold_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr touchdown_hold_reason_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr touchdown_status_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr touchdown_evidence_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
    touchdown_candidate_duration_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr touchdown_confirmed_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr final_descent_phase_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr guidance_source_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace aruco_precision_landing_cpp

#endif  // ARUCO_PRECISION_LANDING_CPP__PX4_ARUCO_LANDING_NODE_HPP_
