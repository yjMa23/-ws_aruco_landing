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
  create_ros_interfaces();

  last_control_time_ = get_clock()->now();
  const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / control_rate_hz_));
  control_timer_ = create_wall_timer(
    timer_period,
    std::bind(&Px4ArucoLandingNode::control_timer_callback, this));

  RCLCPP_INFO(
    get_logger(),
    "PX4 ArUco landing controller started at %.1f Hz; search target NED "
    "(%.2f, %.2f, %.2f)",
    control_rate_hz_,
    search_x_,
    search_y_,
    -search_alt_);
}

void Px4ArucoLandingNode::declare_and_load_parameters()
{
  control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);
  takeoff_alt_ = declare_parameter<double>("takeoff_alt", 3.0);
  search_x_ = declare_parameter<double>("search_x", 0.0);
  search_y_ = declare_parameter<double>("search_y", 0.0);
  search_alt_ = declare_parameter<double>("search_alt", 3.0);
  abort_hover_alt_ = declare_parameter<double>("abort_hover_alt", 3.0);
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
  enable_auto_land_ = declare_parameter<bool>("enable_auto_land", true);
  target_pose_frame_id_ =
    declare_parameter<std::string>("target_pose_frame_id", "local_ned");
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
}

void Px4ArucoLandingNode::create_ros_interfaces()
{
  const auto aruco_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
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
  aruco_visible_ = msg->data;
  last_aruco_visible_time_ = get_clock()->now();

  if (aruco_visible_) {
    if (stable_visible_count_ < stable_detect_count_) {
      ++stable_visible_count_;
    }
  } else {
    stable_visible_count_ = 0;
  }
}

void Px4ArucoLandingNode::aruco_id_callback(
  const std_msgs::msg::Int32::SharedPtr msg)
{
  aruco_id_ = msg->data;
  have_aruco_id_ = true;
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
}

void Px4ArucoLandingNode::run_state_machine(const rclcpp::Time & now, double dt)
{
  switch (state_) {
    case LandingState::INIT:
      transition_to(LandingState::WAIT_FOR_PX4);
      break;

    case LandingState::WAIT_FOR_PX4:
      if (px4_data_ready()) {
        takeoff_start_x_ = local_position_.x;
        takeoff_start_y_ = local_position_.y;
        initial_yaw_ = current_yaw_;
        transition_to(LandingState::OFFBOARD_PRE_STREAM);
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
        transition_to(LandingState::ARM_AND_TAKEOFF);
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
          transition_to(LandingState::GOTO_ARUCO_AREA);
        }
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
          transition_to(LandingState::WAIT_ARUCO);
        }
        break;
      }

    case LandingState::WAIT_ARUCO:
      if (marker_is_fresh(now) && stable_visible_count_ >= stable_detect_count_) {
        last_marker_seen_time_ = now;
        have_last_marker_seen_time_ = true;
        transition_to(LandingState::CENTER_ABOVE_MARKER);
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
            transition_to(LandingState::DESCEND_WITH_TRACKING);
          }
        }
      } else {
        if (
          have_last_marker_seen_time_ &&
          (now - last_marker_seen_time_).seconds() > marker_lost_timeout_)
        {
          transition_to(LandingState::WAIT_ARUCO);
        }
      }
      break;

    case LandingState::DESCEND_WITH_TRACKING:
      if (local_position_.z >= -final_alt_) {
        transition_to(LandingState::FINAL_LAND);
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
          transition_to(LandingState::ABORT);
        }
      }
      break;

    case LandingState::FINAL_LAND:
      if (enable_auto_land_ && !final_land_command_sent_) {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
        final_land_command_sent_ = true;
      }
      transition_to(LandingState::DONE);
      break;

    case LandingState::DONE:
    case LandingState::ABORT:
      break;
  }
}

void Px4ArucoLandingNode::transition_to(LandingState new_state)
{
  if (state_ == new_state) {
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "Landing state: %s -> %s",
    state_name(state_),
    state_name(new_state));
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
