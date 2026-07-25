// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCO_PRECISION_LANDING_CPP__PX4_ARUCO_LANDING_NODE_HPP_
#define ARUCO_PRECISION_LANDING_CPP__PX4_ARUCO_LANDING_NODE_HPP_

#include "aruco_precision_landing_cpp/coordinate_transform.hpp"
#include "aruco_precision_landing_cpp/deck_attitude_estimator.hpp"
#include "aruco_precision_landing_cpp/gnss_rendezvous_guidance.hpp"
#include "aruco_precision_landing_cpp/landing_window.hpp"
#include "aruco_precision_landing_cpp/motion_predictor.hpp"
#include "aruco_precision_landing_cpp/moving_target_tracking_controller.hpp"
#include "aruco_precision_landing_cpp/relative_descent_controller.hpp"
#include "aruco_precision_landing_cpp/target_state_estimator.hpp"
#include "aruco_precision_landing_cpp/vehicle_pose_history.hpp"
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
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
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
    double dt);

  void set_target(double x, double y, double z, double yaw);
  void set_velocity_feedforward(double north_mps, double east_mps);
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
  void publish_estimated_deck_odometry(const rclcpp::Time & now);
  void publish_predicted_deck_pose(const rclcpp::Time & now);
  void publish_tracking_velocity_setpoint(const rclcpp::Time & now);
  void publish_effective_relative_velocity_gain();
  void publish_estimated_deck_acceleration(const rclcpp::Time & now);
  void publish_estimated_deck_attitude();
  void publish_landing_window_debug();
  void publish_relative_descent_debug();
  void publish_guidance_source();

  static const char * state_name(LandingState state);
  static const char * relative_descent_phase_name(RelativeDescentPhase phase);
  static double quaternion_to_yaw(const float q[4]);
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
  double deck_attitude_filter_gain_{0.20};
  double deck_attitude_minimum_upward_normal_component_{0.50};
  double landing_window_enter_horizontal_error_m_{0.15};
  double landing_window_exit_horizontal_error_m_{0.25};
  double landing_window_enter_relative_speed_mps_{0.15};
  double landing_window_exit_relative_speed_mps_{0.25};
  double landing_window_enter_max_tilt_deg_{5.0};
  double landing_window_exit_max_tilt_deg_{8.0};
  double landing_window_max_visual_age_s_{0.20};
  double landing_window_minimum_relative_height_m_{0.20};
  double landing_window_maximum_relative_height_m_{6.00};
  double landing_window_required_duration_s_{1.00};
  bool relative_descent_enabled_{false};
  double descent_minimum_test_height_m_{0.50};
  double descent_fast_height_threshold_m_{2.00};
  double descent_slow_height_threshold_m_{0.80};
  double descent_fast_rate_mps_{0.30};
  double descent_medium_rate_mps_{0.15};
  double descent_slow_rate_mps_{0.05};
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
  std::array<double, 3> camera_translation_frd_m_{{0.0, 0.0, -0.10}};
  std::array<double, 4> camera_rotation_wxyz_{{0.70710678, 0.0, 0.0, 0.70710678}};

  LandingState state_{LandingState::INIT};

  bool have_vehicle_status_{false};
  bool have_local_position_{false};
  bool have_vehicle_odometry_{false};
  bool have_aruco_pose_{false};
  bool aruco_visible_{false};
  bool have_aruco_id_{false};
  bool have_aruco_visible_since_{false};
  bool have_search_pattern_start_time_{false};
  bool have_marker_pose_ned_{false};
  bool have_last_aruco_sample_stamp_{false};
  bool have_estimator_measurement_receipt_time_{false};
  bool have_estimated_deck_attitude_{false};
  bool landing_window_result_valid_{false};
  bool relative_descent_debug_valid_{false};
  bool descent_reentry_locked_{false};
  bool have_px4_to_ros_time_offset_{false};
  bool have_last_time_sync_observation_{false};
  int32_t aruco_id_{-1};
  int64_t last_aruco_sample_stamp_ns_{0};
  uint64_t last_px4_sync_timestamp_us_{0};

  px4_msgs::msg::VehicleStatus vehicle_status_{};
  px4_msgs::msg::VehicleLocalPosition local_position_{};
  px4_msgs::msg::VehicleOdometry vehicle_odometry_{};
  geometry_msgs::msg::PoseStamped aruco_pose_{};
  geometry_msgs::msg::PoseStamped marker_pose_ned_{};
  Pose3d body_camera_pose_{};

  rclcpp::Time last_aruco_pose_time_;
  rclcpp::Time last_aruco_visible_time_;
  rclcpp::Time aruco_visible_since_;
  rclcpp::Time search_pattern_start_time_;
  rclcpp::Time last_marker_seen_time_;
  rclcpp::Time last_command_time_;
  rclcpp::Time last_control_time_;
  rclcpp::Time last_estimated_deck_attitude_time_;
  bool have_last_marker_seen_time_{false};
  bool have_last_command_time_{false};

  int stable_visible_count_{0};
  int prestream_setpoint_count_{0};
  bool final_land_command_sent_{false};
  double handover_progress_s_{0.0};
  double last_estimator_measurement_receipt_time_s_{0.0};
  double last_estimator_state_receipt_time_s_{0.0};
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
  bool effective_relative_velocity_gain_valid_{false};
  bool estimated_deck_acceleration_valid_{false};
  double effective_relative_velocity_gain_{0.0};
  Eigen::Vector2d estimated_deck_acceleration_xy_{Eigen::Vector2d::Zero()};
  DeckAttitudeEstimate estimated_deck_attitude_{};
  LandingWindowResult landing_window_result_{};
  double relative_height_m_{0.0};
  double relative_height_reference_m_{0.0};
  RelativeDescentPhase relative_descent_phase_{RelativeDescentPhase::kWaitingWindow};

  std::unique_ptr<GnssRendezvousGuidance> gnss_guidance_;
  std::unique_ptr<VisualHandoverGuidance> visual_guidance_;
  std::unique_ptr<TargetStateEstimator> target_state_estimator_;
  std::unique_ptr<MotionPredictor> motion_predictor_;
  std::unique_ptr<MovingTargetTrackingController> tracking_controller_;
  std::unique_ptr<VehiclePoseHistory> vehicle_pose_history_;
  std::unique_ptr<DeckAttitudeEstimator> deck_attitude_estimator_;
  std::unique_ptr<LandingWindow> landing_window_;
  std::unique_ptr<RelativeDescentController> relative_descent_controller_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr aruco_visible_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr aruco_id_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr deck_gps_fix_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr deck_gps_velocity_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
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
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr estimated_deck_odometry_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr predicted_deck_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
    tracking_velocity_setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
    effective_relative_velocity_gain_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
    estimated_deck_acceleration_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    estimated_deck_attitude_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr landing_window_open_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr landing_window_reject_reasons_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
    landing_window_satisfied_duration_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr relative_height_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr relative_height_reference_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr descent_phase_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr guidance_source_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace aruco_precision_landing_cpp

#endif  // ARUCO_PRECISION_LANDING_CPP__PX4_ARUCO_LANDING_NODE_HPP_
