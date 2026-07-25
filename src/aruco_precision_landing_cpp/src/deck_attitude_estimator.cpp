// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/deck_attitude_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aruco_precision_landing_cpp
{
namespace
{

constexpr double kMinimumQuaternionNorm = 1.0e-9;
constexpr double kMinimumNormalNorm = 1.0e-9;

bool quaternion_is_finite(const Eigen::Quaterniond & quaternion)
{
  return std::isfinite(quaternion.w()) &&
         std::isfinite(quaternion.x()) &&
         std::isfinite(quaternion.y()) &&
         std::isfinite(quaternion.z());
}

}  // namespace

DeckAttitudeEstimator::DeckAttitudeEstimator(
  const DeckAttitudeEstimatorParameters & parameters)
: parameters_(parameters)
{
  if (!std::isfinite(parameters_.filter_gain) ||
    parameters_.filter_gain <= 0.0 || parameters_.filter_gain > 1.0)
  {
    throw std::invalid_argument("deck attitude filter_gain must be within (0, 1]");
  }
  if (!std::isfinite(parameters_.minimum_upward_normal_component) ||
    parameters_.minimum_upward_normal_component <= 0.0 ||
    parameters_.minimum_upward_normal_component > 1.0)
  {
    throw std::invalid_argument(
            "minimum_upward_normal_component must be within (0, 1]");
  }
}

std::optional<DeckAttitudeEstimate> DeckAttitudeEstimator::update(
  const Eigen::Quaterniond & marker_to_ned_rotation)
{
  if (!quaternion_is_finite(marker_to_ned_rotation) ||
    marker_to_ned_rotation.norm() <= kMinimumQuaternionNorm)
  {
    return std::nullopt;
  }

  const Eigen::Quaterniond normalized = marker_to_ned_rotation.normalized();
  Eigen::Vector3d normal_ned = normalized * Eigen::Vector3d::UnitZ();
  if (!normal_ned.allFinite() || normal_ned.norm() <= kMinimumNormalNorm) {
    return std::nullopt;
  }
  normal_ned.normalize();

  const double upward_component = -normal_ned.z();
  if (!std::isfinite(upward_component) ||
    upward_component < parameters_.minimum_upward_normal_component)
  {
    return std::nullopt;
  }

  if (!initialized_) {
    filtered_normal_ned_ = normal_ned;
    initialized_ = true;
  } else {
    const Eigen::Vector3d blended =
      (1.0 - parameters_.filter_gain) * filtered_normal_ned_ +
      parameters_.filter_gain * normal_ned;
    if (!blended.allFinite() || blended.norm() <= kMinimumNormalNorm) {
      return std::nullopt;
    }
    filtered_normal_ned_ = blended.normalized();
  }

  return estimate_from_normal(filtered_normal_ned_);
}

void DeckAttitudeEstimator::reset()
{
  filtered_normal_ned_ = Eigen::Vector3d{0.0, 0.0, -1.0};
  initialized_ = false;
}

DeckAttitudeEstimate DeckAttitudeEstimator::estimate_from_normal(
  const Eigen::Vector3d & normal_ned) const
{
  DeckAttitudeEstimate estimate;
  estimate.upward_normal_ned = normal_ned;

  const double clamped_east_component = std::clamp(normal_ned.y(), -1.0, 1.0);
  estimate.roll_rad = std::asin(clamped_east_component);
  estimate.pitch_rad = std::atan2(-normal_ned.x(), -normal_ned.z());
  estimate.tilt_rad = std::acos(std::clamp(-normal_ned.z(), -1.0, 1.0));
  return estimate;
}

}  // namespace aruco_precision_landing_cpp
