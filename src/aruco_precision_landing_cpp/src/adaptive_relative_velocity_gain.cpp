// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/adaptive_relative_velocity_gain.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aruco_precision_landing_cpp
{

AdaptiveRelativeVelocityGain::AdaptiveRelativeVelocityGain(
  const AdaptiveRelativeVelocityGainParameters & parameters)
: parameters_(parameters)
{
  if (!std::isfinite(parameters_.min_gain) || parameters_.min_gain < 0.0 ||
    !std::isfinite(parameters_.max_gain) || parameters_.max_gain < parameters_.min_gain)
  {
    throw std::invalid_argument("adaptive gain limits must be finite and ordered");
  }
  if (!std::isfinite(parameters_.acceleration_low_threshold_mps2) ||
    parameters_.acceleration_low_threshold_mps2 < 0.0 ||
    !std::isfinite(parameters_.acceleration_high_threshold_mps2) ||
    parameters_.acceleration_high_threshold_mps2 <=
    parameters_.acceleration_low_threshold_mps2)
  {
    throw std::invalid_argument("adaptive acceleration thresholds must be finite and ordered");
  }
  if (!std::isfinite(parameters_.max_acceleration_mps2) ||
    parameters_.max_acceleration_mps2 <= 0.0 ||
    parameters_.max_acceleration_mps2 < parameters_.acceleration_high_threshold_mps2)
  {
    throw std::invalid_argument(
            "adaptive maximum acceleration must be finite and cover the high threshold");
  }
  if (!std::isfinite(parameters_.acceleration_filter_gain) ||
    parameters_.acceleration_filter_gain <= 0.0 ||
    parameters_.acceleration_filter_gain > 1.0)
  {
    throw std::invalid_argument("adaptive acceleration filter gain must be within (0, 1]");
  }
}

std::optional<AdaptiveRelativeVelocityGainOutput>
AdaptiveRelativeVelocityGain::update(
  const Eigen::Vector2d & deck_velocity_xy,
  double dt_s)
{
  if (!deck_velocity_xy.allFinite() || !std::isfinite(dt_s) || dt_s <= 0.0) {
    return std::nullopt;
  }

  if (!initialized_) {
    previous_velocity_xy_ = deck_velocity_xy;
    filtered_acceleration_xy_.setZero();
    initialized_ = true;
    return make_output();
  }

  Eigen::Vector2d raw_acceleration_xy =
    (deck_velocity_xy - previous_velocity_xy_) / dt_s;
  const double raw_acceleration_norm = raw_acceleration_xy.norm();
  if (!std::isfinite(raw_acceleration_norm)) {
    return std::nullopt;
  }
  if (raw_acceleration_norm > parameters_.max_acceleration_mps2) {
    raw_acceleration_xy *= parameters_.max_acceleration_mps2 / raw_acceleration_norm;
  }

  previous_velocity_xy_ = deck_velocity_xy;
  filtered_acceleration_xy_ += parameters_.acceleration_filter_gain *
    (raw_acceleration_xy - filtered_acceleration_xy_);
  if (!filtered_acceleration_xy_.allFinite()) {
    return std::nullopt;
  }
  return make_output();
}

void AdaptiveRelativeVelocityGain::reset()
{
  previous_velocity_xy_.setZero();
  filtered_acceleration_xy_.setZero();
  initialized_ = false;
}

AdaptiveRelativeVelocityGainOutput AdaptiveRelativeVelocityGain::make_output() const
{
  const double acceleration_norm = filtered_acceleration_xy_.norm();
  const double linear_alpha = std::clamp(
    (acceleration_norm - parameters_.acceleration_low_threshold_mps2) /
    (parameters_.acceleration_high_threshold_mps2 -
    parameters_.acceleration_low_threshold_mps2),
    0.0,
    1.0);
  const double smooth_alpha =
    linear_alpha * linear_alpha * (3.0 - 2.0 * linear_alpha);

  AdaptiveRelativeVelocityGainOutput output;
  output.gain = parameters_.min_gain +
    smooth_alpha * (parameters_.max_gain - parameters_.min_gain);
  output.filtered_acceleration_xy = filtered_acceleration_xy_;
  return output;
}

}  // namespace aruco_precision_landing_cpp
