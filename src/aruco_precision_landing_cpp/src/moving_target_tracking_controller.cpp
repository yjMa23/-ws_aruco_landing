// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/moving_target_tracking_controller.hpp"

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

bool is_non_negative_finite(double value)
{
  return std::isfinite(value) && value >= 0.0;
}

bool is_supported_mode(TrackingControlMode mode)
{
  switch (mode) {
    case TrackingControlMode::kRawVisualPosition:
    case TrackingControlMode::kEstimatedPosition:
    case TrackingControlMode::kEstimatedPositionVelocityFeedforward:
    case TrackingControlMode::kPredictedPositionVelocityFeedforward:
      return true;
  }
  return false;
}

bool estimate_is_finite(const TargetStateEstimate & estimate)
{
  return estimate.position_ned.allFinite() &&
         estimate.velocity_ned.allFinite() &&
         estimate.covariance.allFinite() &&
         std::isfinite(estimate.sample_time_s);
}

Eigen::Vector2d clamp_vector_norm(
  const Eigen::Vector2d & value,
  double maximum_norm)
{
  const double norm = value.norm();
  if (norm <= maximum_norm || norm <= 0.0) {
    return value;
  }
  return value * (maximum_norm / norm);
}

}  // namespace

std::optional<TrackingControlMode> tracking_control_mode_from_string(
  const std::string & value)
{
  if (value == "RAW_VISUAL_POSITION") {
    return TrackingControlMode::kRawVisualPosition;
  }
  if (value == "ESTIMATED_POSITION") {
    return TrackingControlMode::kEstimatedPosition;
  }
  if (value == "ESTIMATED_POSITION_VELOCITY_FF") {
    return TrackingControlMode::kEstimatedPositionVelocityFeedforward;
  }
  if (value == "PREDICTED_POSITION_VELOCITY_FF") {
    return TrackingControlMode::kPredictedPositionVelocityFeedforward;
  }
  return std::nullopt;
}

const char * tracking_control_mode_name(TrackingControlMode mode)
{
  switch (mode) {
    case TrackingControlMode::kRawVisualPosition:
      return "RAW_VISUAL_POSITION";
    case TrackingControlMode::kEstimatedPosition:
      return "ESTIMATED_POSITION";
    case TrackingControlMode::kEstimatedPositionVelocityFeedforward:
      return "ESTIMATED_POSITION_VELOCITY_FF";
    case TrackingControlMode::kPredictedPositionVelocityFeedforward:
      return "PREDICTED_POSITION_VELOCITY_FF";
  }
  return "UNKNOWN";
}

MovingTargetTrackingController::MovingTargetTrackingController(
  const MovingTargetTrackingParameters & parameters)
: parameters_(parameters)
{
  if (!is_supported_mode(parameters_.mode)) {
    throw std::invalid_argument("Unsupported moving target tracking mode");
  }
  if (!is_positive_finite(parameters_.max_position_target_speed_mps) ||
    !is_positive_finite(parameters_.max_position_target_step_m) ||
    !is_positive_finite(parameters_.max_velocity_feedforward_mps) ||
    !is_positive_finite(parameters_.max_velocity_feedforward_acceleration_mps2) ||
    !is_positive_finite(parameters_.max_prediction_age_s))
  {
    throw std::invalid_argument(
            "Tracking speed, step, acceleration, and prediction age limits must be finite and positive");
  }
  if (!is_non_negative_finite(parameters_.velocity_feedforward_gain) ||
    !is_non_negative_finite(parameters_.relative_velocity_gain))
  {
    throw std::invalid_argument("Tracking velocity gains must be finite and non-negative");
  }
  adaptive_gain_scheduler_ = std::make_unique<AdaptiveRelativeVelocityGain>(
    parameters_.adaptive_relative_velocity_gain_parameters);
}

std::optional<MovingTargetTrackingCommand> MovingTargetTrackingController::compute(
  const MovingTargetTrackingInput & input)
{
  if (!input.current_target_xy.allFinite() ||
    !input.uav_position_xy.allFinite() ||
    !is_positive_finite(input.dt_s))
  {
    return std::nullopt;
  }
  if (parameters_.mode != TrackingControlMode::kRawVisualPosition &&
    !is_non_negative_finite(input.estimate_age_s))
  {
    return std::nullopt;
  }

  bool used_prediction = false;
  const auto desired_position = select_position_reference(input, used_prediction);
  if (!desired_position.has_value()) {
    return std::nullopt;
  }

  const auto limited_position = limit_position_target(
    input.current_target_xy, *desired_position, input.dt_s);
  if (!limited_position.has_value()) {
    return std::nullopt;
  }

  const bool velocity_feedforward_mode =
    parameters_.mode == TrackingControlMode::kEstimatedPositionVelocityFeedforward ||
    parameters_.mode == TrackingControlMode::kPredictedPositionVelocityFeedforward;

  std::optional<Eigen::Vector2d> velocity_feedforward;
  std::optional<double> effective_relative_velocity_gain;
  std::optional<Eigen::Vector2d> estimated_deck_acceleration_xy;
  if (velocity_feedforward_mode) {
    double effective_gain = parameters_.relative_velocity_gain;
    velocity_feedforward = compute_velocity_feedforward(
      input, effective_gain, estimated_deck_acceleration_xy);
    if (!velocity_feedforward.has_value()) {
      return std::nullopt;
    }
    effective_relative_velocity_gain = effective_gain;
  } else {
    have_last_velocity_feedforward_ = false;
    last_velocity_feedforward_xy_.setZero();
    adaptive_gain_scheduler_->reset();
    last_adaptive_gain_output_.reset();
    have_last_adaptive_estimate_sample_time_ = false;
  }

  MovingTargetTrackingCommand command;
  command.position_target_xy = *limited_position;
  command.velocity_feedforward_xy = velocity_feedforward;
  command.effective_relative_velocity_gain = effective_relative_velocity_gain;
  command.estimated_deck_acceleration_xy = estimated_deck_acceleration_xy;
  command.mode = parameters_.mode;
  command.used_prediction = used_prediction;
  command.used_short_loss_prediction = used_prediction && !input.visual_fresh;
  return command;
}

void MovingTargetTrackingController::reset()
{
  last_velocity_feedforward_xy_.setZero();
  have_last_velocity_feedforward_ = false;
  adaptive_gain_scheduler_->reset();
  last_adaptive_gain_output_.reset();
  last_adaptive_estimate_sample_time_s_ = 0.0;
  have_last_adaptive_estimate_sample_time_ = false;
}

TrackingControlMode MovingTargetTrackingController::mode() const
{
  return parameters_.mode;
}

std::optional<Eigen::Vector2d>
MovingTargetTrackingController::select_position_reference(
  const MovingTargetTrackingInput & input,
  bool & used_prediction) const
{
  used_prediction = false;

  switch (parameters_.mode) {
    case TrackingControlMode::kRawVisualPosition:
      if (!input.visual_fresh || !input.raw_visual_position_xy.has_value() ||
        !input.raw_visual_position_xy->allFinite())
      {
        return std::nullopt;
      }
      return input.raw_visual_position_xy;

    case TrackingControlMode::kEstimatedPosition:
    case TrackingControlMode::kEstimatedPositionVelocityFeedforward:
      if (input.estimate_age_s > parameters_.max_prediction_age_s ||
        !input.estimated_state.has_value() ||
        !estimate_is_finite(*input.estimated_state))
      {
        return std::nullopt;
      }
      return input.estimated_state->position_ned.head<2>();

    case TrackingControlMode::kPredictedPositionVelocityFeedforward:
      if (input.estimate_age_s > parameters_.max_prediction_age_s ||
        !input.estimated_state.has_value() ||
        !estimate_is_finite(*input.estimated_state) ||
        !input.predicted_position_xy.has_value() ||
        !input.predicted_position_xy->allFinite())
      {
        return std::nullopt;
      }
      used_prediction = true;
      return input.predicted_position_xy;
  }

  return std::nullopt;
}

std::optional<Eigen::Vector2d>
MovingTargetTrackingController::compute_velocity_feedforward(
  const MovingTargetTrackingInput & input,
  double & effective_relative_velocity_gain,
  std::optional<Eigen::Vector2d> & estimated_deck_acceleration_xy)
{
  if (!input.estimated_state.has_value() ||
    !estimate_is_finite(*input.estimated_state))
  {
    return std::nullopt;
  }

  const Eigen::Vector2d deck_velocity_xy =
    input.estimated_state->velocity_ned.head<2>();
  effective_relative_velocity_gain = parameters_.relative_velocity_gain;
  estimated_deck_acceleration_xy.reset();
  if (parameters_.adaptive_relative_velocity_gain_enabled) {
    const auto adaptive_output = update_adaptive_gain(*input.estimated_state, input.dt_s);
    if (!adaptive_output.has_value()) {
      return std::nullopt;
    }
    effective_relative_velocity_gain = adaptive_output->gain;
    estimated_deck_acceleration_xy = adaptive_output->filtered_acceleration_xy;
  }

  Eigen::Vector2d desired_velocity =
    parameters_.velocity_feedforward_gain * deck_velocity_xy;

  if (effective_relative_velocity_gain > 0.0) {
    if (!input.uav_velocity_xy.has_value() || !input.uav_velocity_xy->allFinite()) {
      return std::nullopt;
    }
    desired_velocity += effective_relative_velocity_gain *
      (deck_velocity_xy - *input.uav_velocity_xy);
  }

  if (!input.visual_fresh) {
    const double loss_scale = std::clamp(
      1.0 - input.estimate_age_s / parameters_.max_prediction_age_s,
      0.0,
      1.0);
    desired_velocity *= loss_scale;
  }

  desired_velocity = clamp_vector_norm(
    desired_velocity, parameters_.max_velocity_feedforward_mps);

  const Eigen::Vector2d previous_velocity = have_last_velocity_feedforward_ ?
    last_velocity_feedforward_xy_ : Eigen::Vector2d::Zero();
  const Eigen::Vector2d velocity_delta = desired_velocity - previous_velocity;
  const double maximum_delta =
    parameters_.max_velocity_feedforward_acceleration_mps2 * input.dt_s;
  const Eigen::Vector2d limited_delta = clamp_vector_norm(velocity_delta, maximum_delta);
  const Eigen::Vector2d limited_velocity = previous_velocity + limited_delta;

  if (!limited_velocity.allFinite()) {
    return std::nullopt;
  }

  last_velocity_feedforward_xy_ = limited_velocity;
  have_last_velocity_feedforward_ = true;
  return limited_velocity;
}

std::optional<AdaptiveRelativeVelocityGainOutput>
MovingTargetTrackingController::update_adaptive_gain(
  const TargetStateEstimate & estimate,
  double fallback_dt_s)
{
  if (!estimate_is_finite(estimate) || !is_positive_finite(fallback_dt_s)) {
    return std::nullopt;
  }

  const Eigen::Vector2d deck_velocity_xy = estimate.velocity_ned.head<2>();
  if (!have_last_adaptive_estimate_sample_time_) {
    const auto output = adaptive_gain_scheduler_->update(deck_velocity_xy, fallback_dt_s);
    if (!output.has_value()) {
      return std::nullopt;
    }
    last_adaptive_estimate_sample_time_s_ = estimate.sample_time_s;
    have_last_adaptive_estimate_sample_time_ = true;
    last_adaptive_gain_output_ = output;
    return output;
  }

  const double estimate_dt_s =
    estimate.sample_time_s - last_adaptive_estimate_sample_time_s_;
  if (estimate_dt_s < 0.0) {
    adaptive_gain_scheduler_->reset();
    last_adaptive_gain_output_.reset();
    have_last_adaptive_estimate_sample_time_ = false;
    return update_adaptive_gain(estimate, fallback_dt_s);
  }
  if (estimate_dt_s == 0.0) {
    return last_adaptive_gain_output_;
  }

  const auto output = adaptive_gain_scheduler_->update(deck_velocity_xy, estimate_dt_s);
  if (!output.has_value()) {
    return std::nullopt;
  }
  last_adaptive_estimate_sample_time_s_ = estimate.sample_time_s;
  last_adaptive_gain_output_ = output;
  return output;
}

std::optional<Eigen::Vector2d>
MovingTargetTrackingController::limit_position_target(
  const Eigen::Vector2d & current_target_xy,
  const Eigen::Vector2d & desired_target_xy,
  double dt_s) const
{
  if (!current_target_xy.allFinite() ||
    !desired_target_xy.allFinite() ||
    !is_positive_finite(dt_s))
  {
    return std::nullopt;
  }

  const double maximum_delta = std::min(
    parameters_.max_position_target_speed_mps * dt_s,
    parameters_.max_position_target_step_m);
  const Eigen::Vector2d delta = desired_target_xy - current_target_xy;
  return current_target_xy + clamp_vector_norm(delta, maximum_delta);
}

}  // namespace aruco_precision_landing_cpp
