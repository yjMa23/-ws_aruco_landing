// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/target_state_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Cholesky>

namespace aruco_precision_landing_cpp
{

namespace
{

bool is_positive_finite(double value)
{
  return std::isfinite(value) && value > 0.0;
}

bool estimate_is_finite(const TargetStateEstimate & estimate)
{
  return estimate.position_ned.allFinite() &&
         estimate.velocity_ned.allFinite() &&
         estimate.covariance.allFinite() &&
         std::isfinite(estimate.sample_time_s);
}

}  // namespace

TargetStateEstimator::TargetStateEstimator(
  const TargetStateEstimatorParameters & parameters)
: parameters_(parameters)
{
  if (!is_positive_finite(parameters_.process_acceleration_std_mps2) ||
    !is_positive_finite(parameters_.measurement_horizontal_std_m) ||
    !is_positive_finite(parameters_.measurement_vertical_std_m) ||
    !is_positive_finite(parameters_.initial_position_std_m) ||
    !is_positive_finite(parameters_.initial_velocity_std_mps) ||
    !is_positive_finite(parameters_.minimum_sample_dt_s) ||
    !is_positive_finite(parameters_.maximum_sample_dt_s) ||
    !is_positive_finite(parameters_.reinitialize_gap_s) ||
    !is_positive_finite(parameters_.innovation_gate_mahalanobis))
  {
    throw std::invalid_argument("Target state estimator parameters must be finite and positive");
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

TargetStateUpdateResult TargetStateEstimator::update(
  const Eigen::Vector3d & position_ned,
  double sample_time_s)
{
  TargetStateUpdateResult result;

  if (!position_ned.allFinite() || !std::isfinite(sample_time_s)) {
    result.status = TargetStateUpdateStatus::kRejectedInvalidInput;
    result.estimate = estimate();
    return result;
  }

  if (!initialized_) {
    initialize(position_ned, sample_time_s);
    result.status = TargetStateUpdateStatus::kInitialized;
    result.estimate = make_estimate();
    return result;
  }

  const double dt_s = sample_time_s - state_time_s_;
  if (!std::isfinite(dt_s) || dt_s < parameters_.minimum_sample_dt_s) {
    result.status = TargetStateUpdateStatus::kRejectedNonMonotonicTime;
    result.estimate = make_estimate();
    return result;
  }

  if (dt_s > parameters_.reinitialize_gap_s) {
    initialize(position_ned, sample_time_s);
    result.status = TargetStateUpdateStatus::kReinitialized;
    result.estimate = make_estimate();
    return result;
  }

  double remaining_dt_s = dt_s;
  while (remaining_dt_s > 0.0) {
    const double step_dt_s = std::min(remaining_dt_s, parameters_.maximum_sample_dt_s);
    predict_in_place(step_dt_s);
    remaining_dt_s = std::max(0.0, remaining_dt_s - step_dt_s);
  }
  state_time_s_ = sample_time_s;

  Eigen::Matrix<double, 3, 6> observation_matrix =
    Eigen::Matrix<double, 3, 6>::Zero();
  observation_matrix.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

  Eigen::Matrix3d measurement_covariance = Eigen::Matrix3d::Zero();
  const double horizontal_variance =
    parameters_.measurement_horizontal_std_m *
    parameters_.measurement_horizontal_std_m;
  const double vertical_variance =
    parameters_.measurement_vertical_std_m *
    parameters_.measurement_vertical_std_m;
  measurement_covariance.diagonal() <<
    horizontal_variance,
    horizontal_variance,
    vertical_variance;

  const Eigen::Vector3d innovation = position_ned - observation_matrix * state_;
  const Eigen::Matrix3d innovation_covariance =
    observation_matrix * covariance_ * observation_matrix.transpose() +
    measurement_covariance;
  const Eigen::LDLT<Eigen::Matrix3d> innovation_solver(innovation_covariance);
  if (innovation_solver.info() != Eigen::Success || !innovation_covariance.allFinite()) {
    result.status = TargetStateUpdateStatus::kRejectedInvalidInput;
    result.estimate = make_estimate();
    return result;
  }

  const Eigen::Vector3d normalized_innovation = innovation_solver.solve(innovation);
  if (innovation_solver.info() != Eigen::Success || !normalized_innovation.allFinite()) {
    result.status = TargetStateUpdateStatus::kRejectedInvalidInput;
    result.estimate = make_estimate();
    return result;
  }

  result.normalized_innovation_squared = innovation.dot(normalized_innovation);
  const double gate_squared =
    parameters_.innovation_gate_mahalanobis *
    parameters_.innovation_gate_mahalanobis;
  if (!std::isfinite(result.normalized_innovation_squared) ||
    result.normalized_innovation_squared > gate_squared)
  {
    result.status = TargetStateUpdateStatus::kRejectedOutlier;
    result.estimate = make_estimate();
    return result;
  }

  const Eigen::Matrix<double, 6, 3> kalman_gain =
    covariance_ * observation_matrix.transpose() *
    innovation_solver.solve(Eigen::Matrix3d::Identity());
  if (!kalman_gain.allFinite()) {
    result.status = TargetStateUpdateStatus::kRejectedInvalidInput;
    result.estimate = make_estimate();
    return result;
  }

  state_ += kalman_gain * innovation;

  const Eigen::Matrix<double, 6, 6> identity =
    Eigen::Matrix<double, 6, 6>::Identity();
  const Eigen::Matrix<double, 6, 6> correction =
    identity - kalman_gain * observation_matrix;
  covariance_ =
    correction * covariance_ * correction.transpose() +
    kalman_gain * measurement_covariance * kalman_gain.transpose();
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());

  const TargetStateEstimate current_estimate = make_estimate();
  if (!estimate_is_finite(current_estimate)) {
    reset();
    result.status = TargetStateUpdateStatus::kRejectedInvalidInput;
    result.estimate = std::nullopt;
    return result;
  }

  result.status = TargetStateUpdateStatus::kUpdated;
  result.estimate = current_estimate;
  return result;
}

std::optional<TargetStateEstimate> TargetStateEstimator::estimate() const
{
  if (!initialized_) {
    return std::nullopt;
  }

  const TargetStateEstimate current_estimate = make_estimate();
  if (!estimate_is_finite(current_estimate)) {
    return std::nullopt;
  }
  return current_estimate;
}

void TargetStateEstimator::reset()
{
  state_.setZero();
  covariance_.setZero();
  state_time_s_ = 0.0;
  initialized_ = false;
}

void TargetStateEstimator::initialize(
  const Eigen::Vector3d & position_ned,
  double sample_time_s)
{
  state_.setZero();
  state_.head<3>() = position_ned;

  covariance_.setZero();
  const double position_variance =
    parameters_.initial_position_std_m *
    parameters_.initial_position_std_m;
  const double velocity_variance =
    parameters_.initial_velocity_std_mps *
    parameters_.initial_velocity_std_mps;
  covariance_.diagonal() <<
    position_variance,
    position_variance,
    position_variance,
    velocity_variance,
    velocity_variance,
    velocity_variance;

  state_time_s_ = sample_time_s;
  initialized_ = true;
}

void TargetStateEstimator::predict_in_place(double dt_s)
{
  Eigen::Matrix<double, 6, 6> transition =
    Eigen::Matrix<double, 6, 6>::Identity();
  transition.block<3, 3>(0, 3) = dt_s * Eigen::Matrix3d::Identity();

  Eigen::Matrix<double, 6, 6> process_covariance =
    Eigen::Matrix<double, 6, 6>::Zero();
  const double acceleration_variance =
    parameters_.process_acceleration_std_mps2 *
    parameters_.process_acceleration_std_mps2;
  const double position_gain = 0.5 * dt_s * dt_s;
  const double velocity_gain = dt_s;

  for (int axis = 0; axis < 3; ++axis) {
    process_covariance(axis, axis) =
      acceleration_variance * position_gain * position_gain;
    process_covariance(axis, axis + 3) =
      acceleration_variance * position_gain * velocity_gain;
    process_covariance(axis + 3, axis) =
      process_covariance(axis, axis + 3);
    process_covariance(axis + 3, axis + 3) =
      acceleration_variance * velocity_gain * velocity_gain;
  }

  state_ = transition * state_;
  covariance_ = transition * covariance_ * transition.transpose() + process_covariance;
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
}

TargetStateEstimate TargetStateEstimator::make_estimate() const
{
  TargetStateEstimate estimate;
  estimate.position_ned = state_.head<3>();
  estimate.velocity_ned = state_.tail<3>();
  estimate.covariance = covariance_;
  estimate.sample_time_s = state_time_s_;
  return estimate;
}

}  // namespace aruco_precision_landing_cpp
