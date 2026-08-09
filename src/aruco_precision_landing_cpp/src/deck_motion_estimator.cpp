// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/deck_motion_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Cholesky>
#include <Eigen/QR>

namespace aruco_precision_landing_cpp
{
namespace
{

constexpr double kMinimumQuaternionNorm = 1.0e-9;
constexpr double kSmallRotationRad = 1.0e-10;

bool positive_finite(double value)
{
  return std::isfinite(value) && value > 0.0;
}

bool quaternion_finite(const Eigen::Quaterniond & quaternion)
{
  return std::isfinite(quaternion.w()) &&
         std::isfinite(quaternion.x()) &&
         std::isfinite(quaternion.y()) &&
         std::isfinite(quaternion.z());
}

Eigen::Quaterniond rotation_exp(const Eigen::Vector3d & rotation_vector)
{
  const double angle = rotation_vector.norm();
  if (angle < kSmallRotationRad) {
    return Eigen::Quaterniond{
      1.0, 0.5 * rotation_vector.x(), 0.5 * rotation_vector.y(),
      0.5 * rotation_vector.z()}.normalized();
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotation_vector / angle));
}

Eigen::Vector3d rotation_log(const Eigen::Quaterniond & input)
{
  Eigen::Quaterniond quaternion = input.normalized();
  if (quaternion.w() < 0.0) {
    quaternion.coeffs() *= -1.0;
  }
  const double vector_norm = quaternion.vec().norm();
  if (vector_norm < kSmallRotationRad) {
    return 2.0 * quaternion.vec();
  }
  const double angle = 2.0 * std::atan2(vector_norm, quaternion.w());
  return angle * quaternion.vec() / vector_norm;
}

Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d & vector)
{
  Eigen::Matrix3d matrix;
  matrix <<
    0.0, -vector.z(), vector.y(),
    vector.z(), 0.0, -vector.x(),
    -vector.y(), vector.x(), 0.0;
  return matrix;
}

Eigen::Matrix<double, 9, 9> constant_acceleration_transition(double dt_s)
{
  Eigen::Matrix<double, 9, 9> transition =
    Eigen::Matrix<double, 9, 9>::Identity();
  transition.block<3, 3>(0, 3) = dt_s * Eigen::Matrix3d::Identity();
  transition.block<3, 3>(0, 6) =
    0.5 * dt_s * dt_s * Eigen::Matrix3d::Identity();
  transition.block<3, 3>(3, 6) = dt_s * Eigen::Matrix3d::Identity();
  return transition;
}

Eigen::Matrix<double, 9, 9> constant_acceleration_process_covariance(
  double dt_s,
  double jerk_std)
{
  Eigen::Matrix<double, 9, 9> covariance =
    Eigen::Matrix<double, 9, 9>::Zero();
  const double q = jerk_std * jerk_std;
  const double dt2 = dt_s * dt_s;
  const double dt3 = dt2 * dt_s;
  const double dt4 = dt3 * dt_s;
  const double dt5 = dt4 * dt_s;
  const double block[3][3] = {
    {dt5 / 20.0, dt4 / 8.0, dt3 / 6.0},
    {dt4 / 8.0, dt3 / 3.0, dt2 / 2.0},
    {dt3 / 6.0, dt2 / 2.0, dt_s}};
  for (int row_block = 0; row_block < 3; ++row_block) {
    for (int column_block = 0; column_block < 3; ++column_block) {
      covariance.block<3, 3>(3 * row_block, 3 * column_block) =
        q * block[row_block][column_block] * Eigen::Matrix3d::Identity();
    }
  }
  return covariance;
}

bool covariance_finite(const Eigen::Matrix<double, 9, 9> & covariance)
{
  return covariance.allFinite() &&
         (covariance.diagonal().array() >= 0.0).all();
}

bool estimate_finite(const DeckMotionEstimate & estimate)
{
  return estimate.position_ned_m.allFinite() &&
         estimate.velocity_ned_mps.allFinite() &&
         estimate.acceleration_ned_mps2.allFinite() &&
         quaternion_finite(estimate.orientation_deck_to_ned) &&
         estimate.orientation_deck_to_ned.norm() > kMinimumQuaternionNorm &&
         estimate.angular_velocity_ned_radps.allFinite() &&
         estimate.angular_acceleration_ned_radps2.allFinite() &&
         covariance_finite(estimate.translation_covariance) &&
         covariance_finite(estimate.rotation_covariance) &&
         std::isfinite(estimate.sample_time_s);
}

}  // namespace

DeckMotionEstimator::DeckMotionEstimator(
  const DeckMotionEstimatorParameters & parameters)
: parameters_(parameters)
{
  const bool scalars_valid =
    positive_finite(parameters_.linear_jerk_std_mps3) &&
    positive_finite(parameters_.angular_jerk_std_radps3) &&
    positive_finite(parameters_.measurement_horizontal_std_m) &&
    positive_finite(parameters_.measurement_vertical_std_m) &&
    positive_finite(parameters_.measurement_orientation_std_rad) &&
    positive_finite(parameters_.initial_position_std_m) &&
    positive_finite(parameters_.initial_velocity_std_mps) &&
    positive_finite(parameters_.initial_acceleration_std_mps2) &&
    positive_finite(parameters_.initial_orientation_std_rad) &&
    positive_finite(parameters_.initial_angular_velocity_std_radps) &&
    positive_finite(parameters_.initial_angular_acceleration_std_radps2) &&
    positive_finite(parameters_.minimum_sample_dt_s) &&
    positive_finite(parameters_.maximum_sample_dt_s) &&
    positive_finite(parameters_.reinitialize_gap_s) &&
    positive_finite(parameters_.position_innovation_gate_mahalanobis) &&
    positive_finite(parameters_.orientation_innovation_gate_mahalanobis) &&
    positive_finite(parameters_.minimum_upward_normal_component) &&
    positive_finite(parameters_.prediction_sample_period_s) &&
    positive_finite(parameters_.trusted_prediction_horizon_s) &&
    positive_finite(parameters_.maximum_prediction_horizon_s) &&
    positive_finite(parameters_.kinematic_fit_window_s);
  if (!scalars_valid) {
    throw std::invalid_argument(
            "deck motion estimator parameters must be finite and positive");
  }
  if (parameters_.minimum_upward_normal_component > 1.0 ||
    parameters_.maximum_sample_dt_s < parameters_.minimum_sample_dt_s ||
    parameters_.reinitialize_gap_s <= parameters_.maximum_sample_dt_s ||
    parameters_.trusted_prediction_horizon_s >
    parameters_.maximum_prediction_horizon_s ||
    parameters_.prediction_sample_period_s >
    parameters_.maximum_prediction_horizon_s ||
    parameters_.kinematic_fit_window_s >= parameters_.reinitialize_gap_s)
  {
    throw std::invalid_argument("invalid deck motion estimator parameter relationship");
  }
}

DeckMotionUpdateResult DeckMotionEstimator::update(
  const Pose3d & pose_ned,
  const Eigen::Vector3d & uav_velocity_ned_mps,
  std::int32_t marker_id,
  double sample_time_s)
{
  DeckMotionUpdateResult result;
  if (!pose_ned.translation.allFinite() ||
    !uav_velocity_ned_mps.allFinite() ||
    !quaternion_finite(pose_ned.rotation) ||
    pose_ned.rotation.norm() <= kMinimumQuaternionNorm ||
    !std::isfinite(sample_time_s))
  {
    last_update_accepted_ = false;
    result.estimate = estimate();
    return result;
  }

  Eigen::Quaterniond measurement_orientation = pose_ned.rotation.normalized();
  const Eigen::Vector3d upward_normal_ned =
    measurement_orientation * Eigen::Vector3d::UnitZ();
  if (!upward_normal_ned.allFinite() ||
    upward_normal_ned.z() > -parameters_.minimum_upward_normal_component)
  {
    last_update_accepted_ = false;
    result.estimate = estimate();
    return result;
  }

  if (!initialized_) {
    initialize(
      Pose3d{pose_ned.translation, measurement_orientation},
      uav_velocity_ned_mps, marker_id, sample_time_s);
    result.status = DeckMotionUpdateStatus::kInitialized;
    result.estimate = make_estimate();
    return result;
  }

  const double dt_s = sample_time_s - state_time_s_;
  if (!std::isfinite(dt_s) || dt_s < parameters_.minimum_sample_dt_s) {
    last_update_accepted_ = false;
    result.status = DeckMotionUpdateStatus::kRejectedNonMonotonicTime;
    result.estimate = make_estimate();
    return result;
  }
  if (dt_s > parameters_.reinitialize_gap_s) {
    initialize(
      Pose3d{pose_ned.translation, measurement_orientation},
      uav_velocity_ned_mps, marker_id, sample_time_s);
    result.status = DeckMotionUpdateStatus::kReinitialized;
    result.estimate = make_estimate();
    return result;
  }

  const auto previous_translation_state = translation_state_;
  const auto previous_orientation = orientation_deck_to_ned_;
  const auto previous_angular_velocity = angular_velocity_ned_radps_;
  const auto previous_angular_acceleration = angular_acceleration_ned_radps2_;
  const auto previous_translation_covariance = translation_covariance_;
  const auto previous_rotation_covariance = rotation_covariance_;
  const double previous_state_time_s = state_time_s_;
  const auto restore_previous_state = [this, &previous_translation_state,
      &previous_orientation, &previous_angular_velocity,
      &previous_angular_acceleration, &previous_translation_covariance,
      &previous_rotation_covariance, previous_state_time_s]() {
      translation_state_ = previous_translation_state;
      orientation_deck_to_ned_ = previous_orientation;
      angular_velocity_ned_radps_ = previous_angular_velocity;
      angular_acceleration_ned_radps2_ = previous_angular_acceleration;
      translation_covariance_ = previous_translation_covariance;
      rotation_covariance_ = previous_rotation_covariance;
      state_time_s_ = previous_state_time_s;
    };

  double remaining_dt_s = dt_s;
  while (remaining_dt_s > 0.0) {
    const double step_dt_s =
      std::min(remaining_dt_s, parameters_.maximum_sample_dt_s);
    predict_in_place(step_dt_s);
    remaining_dt_s = std::max(0.0, remaining_dt_s - step_dt_s);
  }
  // 状态速度属于甲板自身；位置观测原点随无人机移动，需扣除原点位移。
  const Eigen::Vector3d uav_displacement_delta =
    0.5 * (uav_velocity_ned_mps_ + uav_velocity_ned_mps) * dt_s;
  translation_state_.head<3>() -= uav_displacement_delta;
  state_time_s_ = sample_time_s;

  if (orientation_deck_to_ned_.coeffs().dot(measurement_orientation.coeffs()) < 0.0) {
    measurement_orientation.coeffs() *= -1.0;
  }

  Eigen::Matrix<double, 3, 9> observation =
    Eigen::Matrix<double, 3, 9>::Zero();
  observation.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

  Eigen::Matrix3d position_measurement_covariance = Eigen::Matrix3d::Zero();
  position_measurement_covariance.diagonal() <<
    parameters_.measurement_horizontal_std_m *
    parameters_.measurement_horizontal_std_m,
    parameters_.measurement_horizontal_std_m *
    parameters_.measurement_horizontal_std_m,
    parameters_.measurement_vertical_std_m *
    parameters_.measurement_vertical_std_m;
  const Eigen::Vector3d position_innovation =
    pose_ned.translation - translation_state_.head<3>();
  const Eigen::Matrix3d position_innovation_covariance =
    observation * translation_covariance_ * observation.transpose() +
    position_measurement_covariance;
  const Eigen::LDLT<Eigen::Matrix3d> position_solver(
    position_innovation_covariance);

  const Eigen::Vector3d orientation_innovation = rotation_log(
    measurement_orientation * orientation_deck_to_ned_.conjugate());
  const Eigen::Matrix3d orientation_measurement_covariance =
    parameters_.measurement_orientation_std_rad *
    parameters_.measurement_orientation_std_rad *
    Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d orientation_innovation_covariance =
    observation * rotation_covariance_ * observation.transpose() +
    orientation_measurement_covariance;
  const Eigen::LDLT<Eigen::Matrix3d> orientation_solver(
    orientation_innovation_covariance);

  if (position_solver.info() != Eigen::Success ||
    orientation_solver.info() != Eigen::Success ||
    !position_innovation_covariance.allFinite() ||
    !orientation_innovation_covariance.allFinite())
  {
    restore_previous_state();
    last_update_accepted_ = false;
    result.status = DeckMotionUpdateStatus::kRejectedInvalidInput;
    result.estimate = make_estimate();
    return result;
  }

  const Eigen::Vector3d normalized_position =
    position_solver.solve(position_innovation);
  const Eigen::Vector3d normalized_orientation =
    orientation_solver.solve(orientation_innovation);
  result.position_normalized_innovation_squared =
    position_innovation.dot(normalized_position);
  result.orientation_normalized_innovation_squared =
    orientation_innovation.dot(normalized_orientation);
  const double position_gate_squared =
    parameters_.position_innovation_gate_mahalanobis *
    parameters_.position_innovation_gate_mahalanobis;
  const double orientation_gate_squared =
    parameters_.orientation_innovation_gate_mahalanobis *
    parameters_.orientation_innovation_gate_mahalanobis;
  if (!std::isfinite(result.position_normalized_innovation_squared) ||
    !std::isfinite(result.orientation_normalized_innovation_squared) ||
    result.position_normalized_innovation_squared > position_gate_squared ||
    result.orientation_normalized_innovation_squared > orientation_gate_squared)
  {
    restore_previous_state();
    last_update_accepted_ = false;
    result.status = DeckMotionUpdateStatus::kRejectedOutlier;
    result.estimate = make_estimate();
    return result;
  }

  const Eigen::Matrix<double, 9, 3> position_gain =
    translation_covariance_ * observation.transpose() *
    position_solver.solve(Eigen::Matrix3d::Identity());
  const Eigen::Matrix<double, 9, 3> orientation_gain =
    rotation_covariance_ * observation.transpose() *
    orientation_solver.solve(Eigen::Matrix3d::Identity());
  if (!position_gain.allFinite() || !orientation_gain.allFinite()) {
    restore_previous_state();
    last_update_accepted_ = false;
    result.status = DeckMotionUpdateStatus::kRejectedInvalidInput;
    result.estimate = make_estimate();
    return result;
  }

  translation_state_ += position_gain * position_innovation;
  const Eigen::Matrix<double, 9, 1> rotation_correction =
    orientation_gain * orientation_innovation;
  orientation_deck_to_ned_ =
    (rotation_exp(rotation_correction.head<3>()) *
    orientation_deck_to_ned_).normalized();
  angular_velocity_ned_radps_ += rotation_correction.segment<3>(3);
  angular_acceleration_ned_radps2_ += rotation_correction.tail<3>();

  const Eigen::Matrix<double, 9, 9> identity =
    Eigen::Matrix<double, 9, 9>::Identity();
  const Eigen::Matrix<double, 9, 9> position_correction =
    identity - position_gain * observation;
  translation_covariance_ =
    position_correction * translation_covariance_ * position_correction.transpose() +
    position_gain * position_measurement_covariance * position_gain.transpose();
  const Eigen::Matrix<double, 9, 9> orientation_correction =
    identity - orientation_gain * observation;
  rotation_covariance_ =
    orientation_correction * rotation_covariance_ * orientation_correction.transpose() +
    orientation_gain * orientation_measurement_covariance * orientation_gain.transpose();
  Eigen::Matrix<double, 9, 9> rotation_reset_jacobian = identity;
  rotation_reset_jacobian.block<3, 3>(0, 0) -=
    0.5 * skew_symmetric(rotation_correction.head<3>());
  rotation_covariance_ =
    rotation_reset_jacobian * rotation_covariance_ * rotation_reset_jacobian.transpose();
  translation_covariance_ =
    0.5 * (translation_covariance_ + translation_covariance_.transpose());
  rotation_covariance_ =
    0.5 * (rotation_covariance_ + rotation_covariance_.transpose());
  marker_id_ = marker_id;
  update_translation_kinematic_fit(
    translation_state_.head<3>(),
    uav_displacement_ned_m_ + uav_displacement_delta,
    sample_time_s);

  const DeckMotionEstimate current_estimate = make_estimate();
  if (!estimate_finite(current_estimate)) {
    restore_previous_state();
    last_update_accepted_ = false;
    result.status = DeckMotionUpdateStatus::kRejectedInvalidInput;
    result.estimate = make_estimate();
    return result;
  }
  result.status = DeckMotionUpdateStatus::kUpdated;
  last_update_accepted_ = true;
  uav_velocity_ned_mps_ = uav_velocity_ned_mps;
  uav_displacement_ned_m_ += uav_displacement_delta;
  result.estimate = current_estimate;
  return result;
}

std::optional<DeckMotionEstimate> DeckMotionEstimator::estimate() const
{
  if (!initialized_) {
    return std::nullopt;
  }
  const DeckMotionEstimate current_estimate = make_estimate();
  return estimate_finite(current_estimate) ?
         std::optional<DeckMotionEstimate>(current_estimate) : std::nullopt;
}

std::optional<DeckMotionPrediction> DeckMotionEstimator::predict(double now_s) const
{
  const auto current = estimate();
  if (!current.has_value() || !std::isfinite(now_s)) {
    return std::nullopt;
  }
  const double observation_age_s = now_s - current->sample_time_s;
  if (!std::isfinite(observation_age_s) || observation_age_s < 0.0 ||
    observation_age_s > parameters_.maximum_prediction_horizon_s)
  {
    return std::nullopt;
  }

  DeckMotionPrediction prediction;
  prediction.observation_age_s = observation_age_s;
  prediction.trusted_horizon_s = last_update_accepted_ ? std::max(
    0.0, parameters_.trusted_prediction_horizon_s - observation_age_s)
    : 0.0;
  const std::size_t point_count =
    static_cast<std::size_t>(
    std::floor(
      parameters_.maximum_prediction_horizon_s /
      parameters_.prediction_sample_period_s + 1.0e-9)) +
    1U;
  prediction.points.reserve(point_count);

  DeckMotionPredictionPoint point;
  point.position_ned_m =
    current->position_ned_m +
    observation_age_s * (current->velocity_ned_mps - uav_velocity_ned_mps_) +
    0.5 * observation_age_s * observation_age_s *
    current->acceleration_ned_mps2;
  point.velocity_ned_mps =
    current->velocity_ned_mps +
    observation_age_s * current->acceleration_ned_mps2;
  point.acceleration_ned_mps2 = current->acceleration_ned_mps2;
  point.orientation_deck_to_ned = current->orientation_deck_to_ned;
  point.angular_velocity_ned_radps = current->angular_velocity_ned_radps;
  point.angular_acceleration_ned_radps2 =
    current->angular_acceleration_ned_radps2;

  auto propagate_rotation = [](DeckMotionPredictionPoint & state, double dt_s) {
      const Eigen::Vector3d midpoint_angular_velocity =
        state.angular_velocity_ned_radps +
        0.5 * dt_s * state.angular_acceleration_ned_radps2;
      state.orientation_deck_to_ned =
        (rotation_exp(midpoint_angular_velocity * dt_s) *
        state.orientation_deck_to_ned).normalized();
      state.angular_velocity_ned_radps +=
        dt_s * state.angular_acceleration_ned_radps2;
    };
  double remaining_age_s = observation_age_s;
  while (remaining_age_s > 0.0) {
    const double step_s =
      std::min(remaining_age_s, parameters_.prediction_sample_period_s);
    propagate_rotation(point, step_s);
    remaining_age_s = std::max(0.0, remaining_age_s - step_s);
  }

  for (std::size_t index = 0; index < point_count; ++index) {
    const double relative_time_s =
      static_cast<double>(index) * parameters_.prediction_sample_period_s;
    if (index > 0U) {
      const double dt_s = parameters_.prediction_sample_period_s;
      point.position_ned_m +=
        dt_s * point.velocity_ned_mps +
        0.5 * dt_s * dt_s * point.acceleration_ned_mps2;
      point.velocity_ned_mps += dt_s * point.acceleration_ned_mps2;
      propagate_rotation(point, dt_s);
    }
    point.relative_time_s = relative_time_s;
    point.trusted = last_update_accepted_ &&
      observation_age_s + relative_time_s <=
      parameters_.trusted_prediction_horizon_s + 1.0e-9;
    if (!point.position_ned_m.allFinite() ||
      !point.velocity_ned_mps.allFinite() ||
      !point.acceleration_ned_mps2.allFinite() ||
      !quaternion_finite(point.orientation_deck_to_ned) ||
      !point.angular_velocity_ned_radps.allFinite() ||
      !point.angular_acceleration_ned_radps2.allFinite())
    {
      return std::nullopt;
    }
    prediction.points.push_back(point);
  }
  return prediction;
}

void DeckMotionEstimator::reset()
{
  translation_state_.setZero();
  orientation_deck_to_ned_.setIdentity();
  angular_velocity_ned_radps_.setZero();
  angular_acceleration_ned_radps2_.setZero();
  translation_covariance_.setZero();
  rotation_covariance_.setZero();
  state_time_s_ = 0.0;
  uav_velocity_ned_mps_.setZero();
  uav_displacement_ned_m_.setZero();
  translation_samples_.clear();
  fitted_deck_velocity_ned_mps_.setZero();
  fitted_deck_acceleration_ned_mps2_.setZero();
  fitted_kinematic_covariance_.setZero();
  translation_fit_valid_ = false;
  marker_id_ = -1;
  last_update_accepted_ = false;
  initialized_ = false;
}

void DeckMotionEstimator::initialize(
  const Pose3d & pose_ned,
  const Eigen::Vector3d & uav_velocity_ned_mps,
  std::int32_t marker_id,
  double sample_time_s)
{
  translation_state_.setZero();
  translation_state_.head<3>() = pose_ned.translation;
  translation_state_.segment<3>(3) = uav_velocity_ned_mps;
  orientation_deck_to_ned_ = pose_ned.rotation.normalized();
  angular_velocity_ned_radps_.setZero();
  angular_acceleration_ned_radps2_.setZero();

  translation_covariance_.setZero();
  rotation_covariance_.setZero();
  const double position_variance =
    parameters_.initial_position_std_m * parameters_.initial_position_std_m;
  const double velocity_variance =
    parameters_.initial_velocity_std_mps * parameters_.initial_velocity_std_mps;
  const double acceleration_variance =
    parameters_.initial_acceleration_std_mps2 *
    parameters_.initial_acceleration_std_mps2;
  translation_covariance_.diagonal() <<
    position_variance, position_variance, position_variance,
    velocity_variance, velocity_variance, velocity_variance,
    acceleration_variance, acceleration_variance, acceleration_variance;

  const double orientation_variance =
    parameters_.initial_orientation_std_rad * parameters_.initial_orientation_std_rad;
  const double angular_velocity_variance =
    parameters_.initial_angular_velocity_std_radps *
    parameters_.initial_angular_velocity_std_radps;
  const double angular_acceleration_variance =
    parameters_.initial_angular_acceleration_std_radps2 *
    parameters_.initial_angular_acceleration_std_radps2;
  rotation_covariance_.diagonal() <<
    orientation_variance, orientation_variance, orientation_variance,
    angular_velocity_variance, angular_velocity_variance, angular_velocity_variance,
    angular_acceleration_variance, angular_acceleration_variance,
    angular_acceleration_variance;

  state_time_s_ = sample_time_s;
  uav_velocity_ned_mps_ = uav_velocity_ned_mps;
  uav_displacement_ned_m_.setZero();
  translation_samples_.clear();
  translation_samples_.push_back(
    TranslationSample{sample_time_s, pose_ned.translation});
  fitted_deck_velocity_ned_mps_ = uav_velocity_ned_mps;
  fitted_deck_acceleration_ned_mps2_.setZero();
  fitted_kinematic_covariance_.setZero();
  translation_fit_valid_ = false;
  marker_id_ = marker_id;
  last_update_accepted_ = true;
  initialized_ = true;
}

void DeckMotionEstimator::update_translation_kinematic_fit(
  const Eigen::Vector3d & relative_position_ned_m,
  const Eigen::Vector3d & uav_displacement_ned_m,
  double sample_time_s)
{
  translation_samples_.push_back(
    TranslationSample{
      sample_time_s, relative_position_ned_m + uav_displacement_ned_m});
  const double minimum_time_s = sample_time_s - parameters_.kinematic_fit_window_s;
  while (!translation_samples_.empty() &&
    translation_samples_.front().time_s < minimum_time_s)
  {
    translation_samples_.pop_front();
  }
  constexpr std::size_t kMinimumFitSamples = 6U;
  if (translation_samples_.size() < kMinimumFitSamples) {
    return;
  }

  Eigen::MatrixXd design(translation_samples_.size(), 3);
  Eigen::MatrixXd measurements(translation_samples_.size(), 3);
  for (std::size_t index = 0; index < translation_samples_.size(); ++index) {
    const double relative_time_s =
      translation_samples_[index].time_s - sample_time_s;
    design.row(index) <<
      1.0, relative_time_s, 0.5 * relative_time_s * relative_time_s;
    measurements.row(index) =
      translation_samples_[index].deck_position_ned_m.transpose();
  }
  const Eigen::Matrix<double, 3, 3> coefficients =
    design.colPivHouseholderQr().solve(measurements);
  const Eigen::Matrix3d normal_matrix = design.transpose() * design;
  const Eigen::LDLT<Eigen::Matrix3d> normal_solver(normal_matrix);
  if (!coefficients.allFinite() || normal_solver.info() != Eigen::Success) {
    return;
  }
  const Eigen::Matrix3d coefficient_scale =
    normal_solver.solve(Eigen::Matrix3d::Identity());
  if (!coefficient_scale.allFinite()) {
    return;
  }

  const Eigen::MatrixXd residuals = measurements - design * coefficients;
  Eigen::Matrix<double, 6, 6> kinematic_covariance =
    Eigen::Matrix<double, 6, 6>::Zero();
  const double degrees_of_freedom =
    static_cast<double>(translation_samples_.size() - 3U);
  for (int axis = 0; axis < 3; ++axis) {
    const double measurement_std = axis < 2 ?
      parameters_.measurement_horizontal_std_m :
      parameters_.measurement_vertical_std_m;
    const double residual_variance =
      residuals.col(axis).squaredNorm() / degrees_of_freedom;
    const double sample_variance = std::max(
      residual_variance, measurement_std * measurement_std);
    kinematic_covariance(axis, axis) =
      sample_variance * coefficient_scale(1, 1);
    kinematic_covariance(axis, axis + 3) =
      sample_variance * coefficient_scale(1, 2);
    kinematic_covariance(axis + 3, axis) =
      kinematic_covariance(axis, axis + 3);
    kinematic_covariance(axis + 3, axis + 3) =
      sample_variance * coefficient_scale(2, 2);
  }
  if (!kinematic_covariance.allFinite()) {
    return;
  }
  fitted_deck_velocity_ned_mps_ = coefficients.row(1).transpose();
  fitted_deck_acceleration_ned_mps2_ = coefficients.row(2).transpose();
  fitted_kinematic_covariance_ = kinematic_covariance;
  translation_fit_valid_ = true;
}

void DeckMotionEstimator::predict_in_place(double dt_s)
{
  const Eigen::Matrix<double, 9, 9> transition =
    constant_acceleration_transition(dt_s);
  translation_state_ = transition * translation_state_;
  translation_covariance_ =
    transition * translation_covariance_ * transition.transpose() +
    constant_acceleration_process_covariance(
    dt_s, parameters_.linear_jerk_std_mps3);

  const Eigen::Vector3d midpoint_angular_velocity =
    angular_velocity_ned_radps_ +
    0.5 * dt_s * angular_acceleration_ned_radps2_;
  orientation_deck_to_ned_ =
    (rotation_exp(midpoint_angular_velocity * dt_s) *
    orientation_deck_to_ned_).normalized();
  angular_velocity_ned_radps_ += dt_s * angular_acceleration_ned_radps2_;
  rotation_covariance_ =
    transition * rotation_covariance_ * transition.transpose() +
    constant_acceleration_process_covariance(
    dt_s, parameters_.angular_jerk_std_radps3);

  translation_covariance_ =
    0.5 * (translation_covariance_ + translation_covariance_.transpose());
  rotation_covariance_ =
    0.5 * (rotation_covariance_ + rotation_covariance_.transpose());
}

DeckMotionEstimate DeckMotionEstimator::make_estimate() const
{
  DeckMotionEstimate estimate;
  estimate.position_ned_m = translation_state_.segment<3>(0);
  estimate.velocity_ned_mps = translation_state_.segment<3>(3);
  estimate.acceleration_ned_mps2 = translation_state_.segment<3>(6);
  estimate.translation_covariance = translation_covariance_;
  if (translation_fit_valid_) {
    estimate.velocity_ned_mps = fitted_deck_velocity_ned_mps_;
    estimate.acceleration_ned_mps2 = fitted_deck_acceleration_ned_mps2_;
    estimate.translation_covariance.block<3, 6>(0, 3).setZero();
    estimate.translation_covariance.block<6, 3>(3, 0).setZero();
    estimate.translation_covariance.block<6, 6>(3, 3) =
      fitted_kinematic_covariance_;
  }
  estimate.orientation_deck_to_ned = orientation_deck_to_ned_;
  estimate.angular_velocity_ned_radps = angular_velocity_ned_radps_;
  estimate.angular_acceleration_ned_radps2 =
    angular_acceleration_ned_radps2_;
  estimate.rotation_covariance = rotation_covariance_;
  estimate.sample_time_s = state_time_s_;
  estimate.marker_id = marker_id_;
  return estimate;
}

}  // namespace aruco_precision_landing_cpp
