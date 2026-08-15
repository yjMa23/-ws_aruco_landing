// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/coordinate_transform.hpp"
#include "aruco_precision_landing_cpp/deck_motion_estimator.hpp"
#include "aruco_precision_landing_cpp/vehicle_pose_history.hpp"

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace aruco_precision_landing_cpp
{
namespace
{

struct ReplayConfig
{
  VehiclePoseHistoryParameters history;
  double clock_offset_filter_gain{0.05};
  double max_clock_offset_jump_s{0.10};
  Pose3d body_camera_pose;
  DeckMotionEstimatorParameters estimator;
};

struct ReplayState
{
  std::unique_ptr<VehiclePoseHistory> history;
  std::unique_ptr<DeckMotionEstimator> estimator;
  ReplayConfig config;
  double px4_to_ros_time_offset_s{0.0};
  double last_time_sync_receipt_s{0.0};
  std::uint64_t last_px4_sync_timestamp_us{0U};
  bool have_px4_to_ros_time_offset{false};
  bool have_last_time_sync_observation{false};
};

bool finite_quaternion(const Eigen::Quaterniond & q)
{
  return std::isfinite(q.w()) && std::isfinite(q.x()) &&
         std::isfinite(q.y()) && std::isfinite(q.z()) && q.norm() > 1.0e-9;
}

bool read_config(std::istringstream & stream, ReplayConfig & config)
{
  auto & p = config.estimator;
  double tx = 0.0;
  double ty = 0.0;
  double tz = 0.0;
  double qw = 1.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  if (!(stream >> config.history.history_duration_s >> config.history.max_endpoint_hold_s >>
    config.clock_offset_filter_gain >> config.max_clock_offset_jump_s >>
    tx >> ty >> tz >> qw >> qx >> qy >> qz >>
    p.linear_jerk_std_mps3 >> p.angular_jerk_std_radps3 >>
    p.measurement_horizontal_std_m >> p.measurement_vertical_std_m >>
    p.measurement_orientation_std_rad >> p.initial_position_std_m >>
    p.initial_velocity_std_mps >> p.initial_acceleration_std_mps2 >>
    p.initial_orientation_std_rad >> p.initial_angular_velocity_std_radps >>
    p.initial_angular_acceleration_std_radps2 >> p.minimum_sample_dt_s >>
    p.maximum_sample_dt_s >> p.reinitialize_gap_s >>
    p.position_innovation_gate_mahalanobis >>
    p.orientation_innovation_gate_mahalanobis >>
    p.minimum_upward_normal_component >> p.prediction_sample_period_s >>
    p.trusted_prediction_horizon_s >> p.maximum_prediction_horizon_s >>
    p.kinematic_fit_window_s))
  {
    return false;
  }
  config.body_camera_pose.translation = Eigen::Vector3d{tx, ty, tz};
  config.body_camera_pose.rotation = Eigen::Quaterniond{qw, qx, qy, qz};
  return finite_quaternion(config.body_camera_pose.rotation) &&
         std::isfinite(config.clock_offset_filter_gain) &&
         config.clock_offset_filter_gain > 0.0 && config.clock_offset_filter_gain <= 1.0 &&
         std::isfinite(config.max_clock_offset_jump_s) &&
         config.max_clock_offset_jump_s > 0.0;
}

std::optional<double> update_px4_to_ros_time_offset(
  ReplayState & state,
  std::uint64_t sample_timestamp_us,
  std::uint64_t sync_timestamp_us,
  double receipt_time_s)
{
  if (sample_timestamp_us == 0U || sync_timestamp_us == 0U ||
    !std::isfinite(receipt_time_s) || receipt_time_s <= 0.0)
  {
    return std::nullopt;
  }

  if (state.have_last_time_sync_observation) {
    const bool ros_time_reset = receipt_time_s < state.last_time_sync_receipt_s;
    const bool px4_time_reset = sync_timestamp_us < state.last_px4_sync_timestamp_us;
    if (ros_time_reset || px4_time_reset) {
      state.history->reset();
      state.have_px4_to_ros_time_offset = false;
      state.have_last_time_sync_observation = false;
    } else if (sync_timestamp_us == state.last_px4_sync_timestamp_us) {
      return std::nullopt;
    }
  }

  const double observed_offset_s =
    receipt_time_s - static_cast<double>(sync_timestamp_us) * 1.0e-6;
  if (!std::isfinite(observed_offset_s)) {
    return std::nullopt;
  }

  if (!state.have_px4_to_ros_time_offset) {
    state.px4_to_ros_time_offset_s = observed_offset_s;
    state.have_px4_to_ros_time_offset = true;
  } else {
    const double error_s = observed_offset_s - state.px4_to_ros_time_offset_s;
    if (std::abs(error_s) > state.config.max_clock_offset_jump_s) {
      state.history->reset();
      state.px4_to_ros_time_offset_s = observed_offset_s;
    } else {
      state.px4_to_ros_time_offset_s +=
        state.config.clock_offset_filter_gain * error_s;
    }
  }

  state.last_time_sync_receipt_s = receipt_time_s;
  state.last_px4_sync_timestamp_us = sync_timestamp_us;
  state.have_last_time_sync_observation = true;
  const double sample_time_s =
    static_cast<double>(sample_timestamp_us) * 1.0e-6 + state.px4_to_ros_time_offset_s;
  return std::isfinite(sample_time_s) && sample_time_s > 0.0 ?
         std::optional<double>(sample_time_s) : std::nullopt;
}

void emit_prediction(std::size_t index, double publish_time_s, const ReplayState & state)
{
  const auto estimate = state.estimator->estimate();
  const auto prediction = state.estimator->predict(publish_time_s);
  if (!estimate.has_value() || !prediction.has_value() || prediction->points.empty()) {
    std::cout << "P\t" << index << "\t0\n";
    return;
  }

  const auto & point = prediction->points.front();
  std::cout << std::setprecision(17)
            << "P\t" << index << "\t1\t" << estimate->sample_time_s;
  for (const double value : {
      point.velocity_ned_mps.x(), point.velocity_ned_mps.y(), point.velocity_ned_mps.z(),
      point.acceleration_ned_mps2.x(), point.acceleration_ned_mps2.y(),
      point.acceleration_ned_mps2.z(),
      point.angular_velocity_ned_radps.x(), point.angular_velocity_ned_radps.y(),
      point.angular_velocity_ned_radps.z(),
      point.angular_acceleration_ned_radps2.x(), point.angular_acceleration_ned_radps2.y(),
      point.angular_acceleration_ned_radps2.z()})
  {
    std::cout << '\t' << value;
  }
  std::cout << '\n';
}

}  // namespace
}  // namespace aruco_precision_landing_cpp

int main()
{
  using namespace aruco_precision_landing_cpp;
  ReplayState state;
  bool configured = false;
  std::string line;
  while (std::getline(std::cin, line)) {
    if (line.empty() || line.front() == '#') {
      continue;
    }
    std::istringstream stream(line);
    std::string event;
    stream >> event;
    if (event == "CONFIG") {
      if (configured || !read_config(stream, state.config)) {
        std::cerr << "invalid CONFIG line\n";
        return 2;
      }
      try {
        state.history = std::make_unique<VehiclePoseHistory>(state.config.history);
        state.estimator = std::make_unique<DeckMotionEstimator>(state.config.estimator);
      } catch (const std::exception & exception) {
        std::cerr << "invalid replay configuration: " << exception.what() << '\n';
        return 2;
      }
      configured = true;
      continue;
    }
    if (!configured) {
      std::cerr << "CONFIG must precede replay events\n";
      return 2;
    }

    if (event == "ODOM") {
      double receipt_time_s = 0.0;
      std::uint64_t sample_timestamp_us = 0U;
      std::uint64_t sync_timestamp_us = 0U;
      double px = 0.0;
      double py = 0.0;
      double pz = 0.0;
      double qw = 1.0;
      double qx = 0.0;
      double qy = 0.0;
      double qz = 0.0;
      double vx = 0.0;
      double vy = 0.0;
      double vz = 0.0;
      int velocity_valid = 0;
      if (!(stream >> receipt_time_s >> sample_timestamp_us >> sync_timestamp_us >>
        px >> py >> pz >> qw >> qx >> qy >> qz >> vx >> vy >> vz >> velocity_valid))
      {
        std::cerr << "invalid ODOM line\n";
        return 2;
      }
      const Pose3d pose{
        Eigen::Vector3d{px, py, pz}, Eigen::Quaterniond{qw, qx, qy, qz}};
      // 与 production callback 一致：非法 NED pose/quaternion 在进入 clock sync 前就拒绝。
      if (!finite_quaternion(pose.rotation) || !pose.translation.allFinite()) {
        continue;
      }
      const auto sample_time_s = update_px4_to_ros_time_offset(
        state, sample_timestamp_us, sync_timestamp_us, receipt_time_s);
      if (sample_time_s.has_value()) {
        VehicleKinematicState kinematic{
          pose, Eigen::Vector3d{vx, vy, vz}, velocity_valid != 0};
        state.history->add_sample(kinematic, *sample_time_s);
      }
      continue;
    }

    if (event == "ARUCO") {
      double sample_time_s = 0.0;
      std::int32_t marker_id = -1;
      double px = 0.0;
      double py = 0.0;
      double pz = 0.0;
      double qw = 1.0;
      double qx = 0.0;
      double qy = 0.0;
      double qz = 0.0;
      if (!(stream >> sample_time_s >> marker_id >>
        px >> py >> pz >> qw >> qx >> qy >> qz))
      {
        std::cerr << "invalid ARUCO line\n";
        return 2;
      }
      const auto vehicle = state.history->lookup_state(sample_time_s);
      if (!vehicle.has_value() || !vehicle->velocity_valid) {
        std::cout << std::setprecision(17) << "U\t" << sample_time_s << "\tNO_HISTORY\n";
        continue;
      }
      const Pose3d camera_marker_pose{
        Eigen::Vector3d{px, py, pz}, Eigen::Quaterniond{qw, qx, qy, qz}};
      const auto transformed = transform_marker_to_uav_centered_ned(
        vehicle->pose.rotation, state.config.body_camera_pose, camera_marker_pose);
      if (!transformed.has_value()) {
        std::cout << std::setprecision(17) << "U\t" << sample_time_s << "\tTRANSFORM_FAILED\n";
        continue;
      }
      const auto result = state.estimator->update(
        *transformed, vehicle->velocity_ned_mps, marker_id, sample_time_s);
      std::cout << std::setprecision(17) << "U\t" << sample_time_s << "\t"
                << static_cast<int>(result.status) << '\n';
      continue;
    }

    if (event == "PREDICT") {
      std::size_t index = 0U;
      double publish_time_s = 0.0;
      if (!(stream >> index >> publish_time_s)) {
        std::cerr << "invalid PREDICT line\n";
        return 2;
      }
      emit_prediction(index, publish_time_s, state);
      continue;
    }

    std::cerr << "unknown replay event: " << event << '\n';
    return 2;
  }
  return configured ? 0 : 2;
}
