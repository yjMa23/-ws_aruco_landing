// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/vertical_state_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aruco_precision_landing_cpp
{

namespace
{

bool is_positive_finite(double value)
{
  return std::isfinite(value) && value > 0.0;
}

bool estimate_is_finite(const VerticalStateEstimate & estimate)
{
  return std::isfinite(estimate.deck_z_ned_m) &&
         std::isfinite(estimate.deck_vertical_velocity_ned_mps) &&
         estimate.covariance.allFinite() &&
         std::isfinite(estimate.sample_time_s);
}

}  // namespace

VerticalStateEstimator::VerticalStateEstimator(
  const VerticalStateEstimatorParameters & parameters)
: parameters_(parameters)
{
  if (!is_positive_finite(parameters_.process_acceleration_std_mps2) ||
    !is_positive_finite(parameters_.measurement_std_m) ||
    !std::isfinite(parameters_.measurement_bias_m) ||
    !is_positive_finite(parameters_.initial_position_std_m) ||
    !is_positive_finite(parameters_.initial_velocity_std_mps) ||
    !is_positive_finite(parameters_.minimum_sample_dt_s) ||
    !is_positive_finite(parameters_.maximum_sample_dt_s) ||
    !is_positive_finite(parameters_.reinitialize_gap_s) ||
    !is_positive_finite(parameters_.innovation_gate_mahalanobis))
  {
    throw std::invalid_argument(
            "Vertical state estimator parameters are invalid");
  }
  if (parameters_.maximum_sample_dt_s < parameters_.minimum_sample_dt_s) {
    throw std::invalid_argument(
            "maximum_sample_dt_s must not be smaller than minimum_sample_dt_s");
  }
  if (parameters_.reinitialize_gap_s <= parameters_.maximum_sample_dt_s) {
    throw std::invalid_argument(
            "reinitialize_gap_s must be greater than maximum_sample_dt_s");
  }
}

VerticalStateUpdateResult VerticalStateEstimator::update(
  double measurement_z_ned_m,
  double sample_time_s)
{
  VerticalStateUpdateResult result;
  if (!std::isfinite(measurement_z_ned_m) || !std::isfinite(sample_time_s)) {
    result.status = VerticalStateUpdateStatus::kRejectedInvalidInput;
    result.estimate = estimate();
    return result;
  }

  const double corrected_measurement_z_ned_m =
    measurement_z_ned_m - parameters_.measurement_bias_m;
  if (!std::isfinite(corrected_measurement_z_ned_m)) {
    result.status = VerticalStateUpdateStatus::kRejectedInvalidInput;
    result.estimate = estimate();
    return result;
  }

  if (!initialized_) {
    initialize(corrected_measurement_z_ned_m, sample_time_s);
    result.status = VerticalStateUpdateStatus::kInitialized;
    result.estimate = make_estimate();
    return result;
  }

  const double dt_s = sample_time_s - state_time_s_;
  if (!std::isfinite(dt_s) || dt_s < parameters_.minimum_sample_dt_s) {
    result.status = VerticalStateUpdateStatus::kRejectedNonMonotonicTime;
    result.estimate = make_estimate();
    return result;
  }

  if (dt_s > parameters_.reinitialize_gap_s) {
    initialize(corrected_measurement_z_ned_m, sample_time_s);
    result.status = VerticalStateUpdateStatus::kReinitialized;
    result.estimate = make_estimate();
    return result;
  }

  double remaining_dt_s = dt_s;
  while (remaining_dt_s > 0.0) {
    const double step_dt_s = std::min(
      remaining_dt_s, parameters_.maximum_sample_dt_s);
    predict_in_place(step_dt_s);
    remaining_dt_s = std::max(0.0, remaining_dt_s - step_dt_s);
  }
  state_time_s_ = sample_time_s;

  const double innovation = corrected_measurement_z_ned_m - state_.x();
  const double measurement_variance =
    parameters_.measurement_std_m * parameters_.measurement_std_m;
  const double innovation_variance = covariance_(0, 0) + measurement_variance;
  if (!std::isfinite(innovation_variance) || innovation_variance <= 0.0) {
    result.status = VerticalStateUpdateStatus::kRejectedInvalidInput;
    result.estimate = make_estimate();
    return result;
  }

  result.normalized_innovation_squared =
    innovation * innovation / innovation_variance;
  const double gate_squared =
    parameters_.innovation_gate_mahalanobis *
    parameters_.innovation_gate_mahalanobis;
  if (!std::isfinite(result.normalized_innovation_squared) ||
    result.normalized_innovation_squared > gate_squared)
  {
    result.status = VerticalStateUpdateStatus::kRejectedOutlier;
    result.estimate = make_estimate();
    return result;
  }

  const Eigen::Vector2d kalman_gain =
    covariance_.col(0) / innovation_variance;
  if (!kalman_gain.allFinite()) {
    result.status = VerticalStateUpdateStatus::kRejectedInvalidInput;
    result.estimate = make_estimate();
    return result;
  }

  state_ += kalman_gain * innovation;

  Eigen::RowVector2d observation;
  observation << 1.0, 0.0;
  const Eigen::Matrix2d identity = Eigen::Matrix2d::Identity();
  const Eigen::Matrix2d correction = identity - kalman_gain * observation;
  covariance_ =
    correction * covariance_ * correction.transpose() +
    kalman_gain * measurement_variance * kalman_gain.transpose();
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());

  const VerticalStateEstimate current_estimate = make_estimate();
  if (!estimate_is_finite(current_estimate)) {
    reset();
    result.status = VerticalStateUpdateStatus::kRejectedInvalidInput;
    result.estimate = std::nullopt;
    return result;
  }

  result.status = VerticalStateUpdateStatus::kUpdated;
  result.estimate = current_estimate;
  return result;
}

std::optional<VerticalStateEstimate> VerticalStateEstimator::estimate() const
{
  if (!initialized_) {
    return std::nullopt;
  }
  const VerticalStateEstimate current_estimate = make_estimate();
  return estimate_is_finite(current_estimate) ?
         std::optional<VerticalStateEstimate>(current_estimate) : std::nullopt;
}

void VerticalStateEstimator::reset()
{
  state_.setZero();
  covariance_.setZero();
  state_time_s_ = 0.0;
  initialized_ = false;
}

void VerticalStateEstimator::initialize(
  double corrected_measurement_z_ned_m,
  double sample_time_s)
{
  state_ << corrected_measurement_z_ned_m, 0.0;
  covariance_.setZero();
  covariance_(0, 0) =
    parameters_.initial_position_std_m * parameters_.initial_position_std_m;
  covariance_(1, 1) =
    parameters_.initial_velocity_std_mps * parameters_.initial_velocity_std_mps;
  state_time_s_ = sample_time_s;
  initialized_ = true;
}

void VerticalStateEstimator::predict_in_place(double dt_s)
{
  Eigen::Matrix2d transition;
  transition <<
    1.0, dt_s,
    0.0, 1.0;

  const double acceleration_variance =
    parameters_.process_acceleration_std_mps2 *
    parameters_.process_acceleration_std_mps2;
  const double position_gain = 0.5 * dt_s * dt_s;
  const double velocity_gain = dt_s;
  Eigen::Matrix2d process_covariance;
  process_covariance <<
    acceleration_variance * position_gain * position_gain,
    acceleration_variance * position_gain * velocity_gain,
    acceleration_variance * position_gain * velocity_gain,
    acceleration_variance * velocity_gain * velocity_gain;

  state_ = transition * state_;
  covariance_ =
    transition * covariance_ * transition.transpose() + process_covariance;
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
}

VerticalStateEstimate VerticalStateEstimator::make_estimate() const
{
  VerticalStateEstimate estimate;
  estimate.deck_z_ned_m = state_.x();
  estimate.deck_vertical_velocity_ned_mps = state_.y();
  estimate.covariance = covariance_;
  estimate.sample_time_s = state_time_s_;
  return estimate;
}

}  // namespace aruco_precision_landing_cpp
