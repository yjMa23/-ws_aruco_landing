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
  if (!std::isfinite(sample_time_s)) {
    return false;
  }
  const auto normalized_transform = make_isometry(pose.translation, pose.rotation);
  if (!normalized_transform.has_value()) {
    return false;
  }
  if (!samples_.empty() && sample_time_s <= samples_.back().time_s) {
    return false;
  }

  Pose3d normalized_pose;
  normalized_pose.translation = normalized_transform->translation();
  normalized_pose.rotation = Eigen::Quaterniond(normalized_transform->linear());
  normalized_pose.rotation.normalize();
  samples_.push_back(Sample{sample_time_s, normalized_pose});

  const double minimum_time_s = sample_time_s - parameters_.history_duration_s;
  while (!samples_.empty() && samples_.front().time_s < minimum_time_s) {
    samples_.pop_front();
  }
  return true;
}

std::optional<Pose3d> VehiclePoseHistory::lookup(double query_time_s) const
{
  if (!std::isfinite(query_time_s) || samples_.empty()) {
    return std::nullopt;
  }

  const Sample & first = samples_.front();
  const Sample & last = samples_.back();
  if (query_time_s <= first.time_s) {
    if (first.time_s - query_time_s <= parameters_.max_endpoint_hold_s) {
      return first.pose;
    }
    return std::nullopt;
  }
  if (query_time_s >= last.time_s) {
    if (query_time_s - last.time_s <= parameters_.max_endpoint_hold_s) {
      return last.pose;
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
    return upper->pose;
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

  Eigen::Quaterniond upper_rotation = upper->pose.rotation;
  if (lower->pose.rotation.dot(upper_rotation) < 0.0) {
    upper_rotation.coeffs() *= -1.0;
  }

  Pose3d interpolated;
  interpolated.translation =
    lower->pose.translation + alpha * (upper->pose.translation - lower->pose.translation);
  interpolated.rotation = lower->pose.rotation.slerp(alpha, upper_rotation);
  interpolated.rotation.normalize();
  if (!make_isometry(interpolated.translation, interpolated.rotation).has_value()) {
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
