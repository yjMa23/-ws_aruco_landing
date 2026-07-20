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

namespace aruco_precision_landing_cpp
{

namespace
{

constexpr double kMinimumQuaternionNorm = 1.0e-6;

bool is_positive_finite(double value)
{
  return std::isfinite(value) && value > 0.0;
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
    "radius %.2f m",
    control_rate_hz_,
    rendezvous_altitude_m_,
    rendezvous_radius_m_);
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
  if (target_pose_frame_id_.empty()) {
    throw std::invalid_argument("Parameter 'target_pose_frame_id' must not be empty");
  }
  if (deck_gnss_velocity_frame_id_.empty()) {
    throw std::invalid_argument(
            "Parameter 'deck_gnss_velocity_frame_id' must not be empty");
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
  guidance_source_pub_ = create_publisher<std_msgs::msg::String>(
    "/landing/guidance_source",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
}

void Px4ArucoLandingNode::aruco_pose_callback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  aruco_pose_ = *msg;
  have_aruco_pose_ = true;
  last_aruco_pose_time_ = get_clock()->now();
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
  vehicle_odometry_ = *msg;
  have_vehicle_odometry_ = true;

  if (quaternion_is_valid(vehicle_odometry_.q.data())) {
    current_yaw_ = quaternion_to_yaw(vehicle_odometry_.q.data());
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

  publish_offboard_control_mode();
  run_state_machine(now, dt);
  publish_trajectory_setpoint();
  publish_landing_state();
  publish_target_pose();
  publish_deck_gnss_pose(now);
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

        Eigen::Vector2d desired_target = estimate->position_ned.head<2>();
        if (marker_is_stably_visible(now)) {
          if (!aruco_acquired_hold_) {
            RCLCPP_INFO(
              get_logger(),
              "ArUco acquired at safe altitude; holding GNSS center until P2D handover");
          }
          aruco_acquired_hold_ = true;
        } else {
          if (aruco_acquired_hold_ || !have_search_pattern_start_time_) {
            search_pattern_start_time_ = now;
            have_search_pattern_start_time_ = true;
          }
          aruco_acquired_hold_ = false;
          const double elapsed_s =
            (now - search_pattern_start_time_).seconds();
          const auto offset = gnss_guidance_->search_offset(elapsed_s);
          if (!offset.has_value()) {
            transition_to(LandingState::ABORT, "invalid GNSS-centered search offset");
            break;
          }
          desired_target += *offset;
        }

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

  RCLCPP_INFO(
    get_logger(),
    "Landing state: %s -> %s; reason: %s",
    state_name(state_),
    state_name(new_state),
    reason);
  state_ = new_state;

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
      aruco_acquired_hold_ = false;
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
      aruco_acquired_hold_ = false;
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
  msg.velocity = {nan, nan, nan};
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
      msg.data = aruco_acquired_hold_ ? "GNSS_ARUCO_ACQUIRED_HOLD" : "GNSS_SEARCH";
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
