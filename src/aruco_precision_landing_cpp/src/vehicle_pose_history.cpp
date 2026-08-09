// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/vehicle_pose_history.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aruco_precision_landing_cpp
{

VehiclePoseHistory::VehiclePoseHistory(
  const VehiclePoseHistoryParameters & parameters)
: parameters_(parameters)
{
  if (!std::isfinite(parameters_.history_duration_s) ||
    parameters_.history_duration_s <= 0.0)
  {
    throw std::invalid_argument("history_duration_s must be finite and positive");
  }
  if (!std::isfinite(parameters_.max_endpoint_hold_s) ||
    parameters_.max_endpoint_hold_s < 0.0)
  {
    throw std::invalid_argument("max_endpoint_hold_s must be finite and non-negative");
  }
}

bool VehiclePoseHistory::add_sample(const Pose3d & pose, double sample_time_s)
{
  return add_sample(
    VehicleKinematicState{pose, Eigen::Vector3d::Zero(), true}, sample_time_s);
}

bool VehiclePoseHistory::add_sample(
  const VehicleKinematicState & state, double sample_time_s)
{
  if (!std::isfinite(sample_time_s) ||
    (state.velocity_valid && !state.velocity_ned_mps.allFinite()))
  {
    return false;
  }
  const auto normalized_transform = make_isometry(
    state.pose.translation, state.pose.rotation);
  if (!normalized_transform.has_value()) {
    return false;
  }
  if (!samples_.empty() && sample_time_s <= samples_.back().time_s) {
    return false;
  }

  VehicleKinematicState normalized_state = state;
  normalized_state.pose.translation = normalized_transform->translation();
  normalized_state.pose.rotation = Eigen::Quaterniond(normalized_transform->linear());
  normalized_state.pose.rotation.normalize();
  samples_.push_back(Sample{sample_time_s, normalized_state});

  const double minimum_time_s = sample_time_s - parameters_.history_duration_s;
  while (!samples_.empty() && samples_.front().time_s < minimum_time_s) {
    samples_.pop_front();
  }
  return true;
}

std::optional<Pose3d> VehiclePoseHistory::lookup(double query_time_s) const
{
  const auto state = lookup_state(query_time_s);
  return state.has_value() ? std::optional<Pose3d>(state->pose) : std::nullopt;
}

std::optional<VehicleKinematicState> VehiclePoseHistory::lookup_state(
  double query_time_s) const
{
  if (!std::isfinite(query_time_s) || samples_.empty()) {
    return std::nullopt;
  }

  const Sample & first = samples_.front();
  const Sample & last = samples_.back();
  if (query_time_s <= first.time_s) {
    if (first.time_s - query_time_s <= parameters_.max_endpoint_hold_s) {
      return first.state;
    }
    return std::nullopt;
  }
  if (query_time_s >= last.time_s) {
    if (query_time_s - last.time_s <= parameters_.max_endpoint_hold_s) {
      return last.state;
    }
    return std::nullopt;
  }

  const auto upper = std::lower_bound(
    samples_.begin(), samples_.end(), query_time_s,
    [](const Sample & sample, double time_s) {
      return sample.time_s < time_s;
    });
  if (upper == samples_.end()) {
    return std::nullopt;
  }
  if (upper->time_s == query_time_s) {
    return upper->state;
  }

  const auto lower = std::prev(upper);
  const double duration_s = upper->time_s - lower->time_s;
  if (!std::isfinite(duration_s) || duration_s <= 0.0) {
    return std::nullopt;
  }
  const double alpha = (query_time_s - lower->time_s) / duration_s;
  if (!std::isfinite(alpha) || alpha < 0.0 || alpha > 1.0) {
    return std::nullopt;
  }

  Eigen::Quaterniond upper_rotation = upper->state.pose.rotation;
  if (lower->state.pose.rotation.dot(upper_rotation) < 0.0) {
    upper_rotation.coeffs() *= -1.0;
  }

  VehicleKinematicState interpolated;
  interpolated.pose.translation =
    lower->state.pose.translation +
    alpha * (upper->state.pose.translation - lower->state.pose.translation);
  interpolated.pose.rotation = lower->state.pose.rotation.slerp(alpha, upper_rotation);
  interpolated.pose.rotation.normalize();
  interpolated.velocity_ned_mps =
    lower->state.velocity_ned_mps;
  interpolated.velocity_valid =
    lower->state.velocity_valid && upper->state.velocity_valid;
  if (interpolated.velocity_valid) {
    interpolated.velocity_ned_mps +=
      alpha * (upper->state.velocity_ned_mps - lower->state.velocity_ned_mps);
  }
  if (!make_isometry(
      interpolated.pose.translation, interpolated.pose.rotation).has_value() ||
    (interpolated.velocity_valid && !interpolated.velocity_ned_mps.allFinite()))
  {
    return std::nullopt;
  }
  return interpolated;
}

void VehiclePoseHistory::reset()
{
  samples_.clear();
}

std::size_t VehiclePoseHistory::size() const
{
  return samples_.size();
}

}  // namespace aruco_precision_landing_cpp
