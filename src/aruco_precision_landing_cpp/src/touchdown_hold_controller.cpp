// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/touchdown_hold_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aruco_precision_landing_cpp
{

TouchdownHoldController::TouchdownHoldController(
  const TouchdownHoldParameters & parameters)
: parameters_(parameters)
{
  if (!std::isfinite(parameters_.max_target_rate_mps) ||
    parameters_.max_target_rate_mps <= 0.0 ||
    !std::isfinite(parameters_.max_reference_preload_rate_mps) ||
    parameters_.max_reference_preload_rate_mps <= 0.0)
  {
    throw std::invalid_argument(
            "touchdown hold target rates must be finite and positive");
  }
  if (!std::isfinite(parameters_.motion_enter_speed_mps) ||
    !std::isfinite(parameters_.motion_exit_speed_mps) ||
    parameters_.motion_enter_speed_mps <= 0.0 ||
    parameters_.motion_exit_speed_mps < 0.0 ||
    parameters_.motion_exit_speed_mps >= parameters_.motion_enter_speed_mps)
  {
    throw std::invalid_argument(
            "touchdown hold motion speed thresholds must be finite with "
            "0 <= exit < enter");
  }
}

std::optional<TouchdownHoldOutput> TouchdownHoldController::update(
  const TouchdownHoldInput & input)
{
  if (!std::isfinite(input.dt_s) || input.dt_s <= 0.0 ||
    !std::isfinite(input.uav_z_ned_m) ||
    (input.relative_height_target_m.has_value() &&
    (!std::isfinite(*input.relative_height_target_m) ||
    *input.relative_height_target_m < 0.0)))
  {
    return std::nullopt;
  }

  if (!input.deck_state_valid) {
    if (!initialized_) {
      return std::nullopt;
    }
    return make_output(
      TouchdownHoldMode::kHoldLastTarget,
      TouchdownHoldReason::kDeckStateInvalid,
      std::nullopt);
  }

  if (!std::isfinite(input.deck_z_ned_m) ||
    !std::isfinite(input.deck_vertical_velocity_ned_mps))
  {
    return std::nullopt;
  }

  const double absolute_deck_speed_mps =
    std::abs(input.deck_vertical_velocity_ned_mps);
  if (!initialized_) {
    relative_height_reference_m_ = input.deck_z_ned_m - input.uav_z_ned_m;
    vertical_target_z_ned_m_ = input.uav_z_ned_m;
    if (!std::isfinite(relative_height_reference_m_) ||
      !std::isfinite(vertical_target_z_ned_m_))
    {
      reset();
      return std::nullopt;
    }
    deck_motion_active_ =
      absolute_deck_speed_mps >= parameters_.motion_enter_speed_mps;
    initialized_ = true;
  } else {
    if (deck_motion_active_) {
      if (absolute_deck_speed_mps <= parameters_.motion_exit_speed_mps) {
        deck_motion_active_ = false;
      }
    } else if (absolute_deck_speed_mps >= parameters_.motion_enter_speed_mps) {
      deck_motion_active_ = true;
    }
  }

  const bool preload_active = input.relative_height_target_m.has_value();
  if (preload_active) {
    const double maximum_reference_step_m =
      parameters_.max_reference_preload_rate_mps * input.dt_s;
    const double reference_error_m =
      *input.relative_height_target_m - relative_height_reference_m_;
    relative_height_reference_m_ += std::clamp(
      reference_error_m, -maximum_reference_step_m, maximum_reference_step_m);
  }

  if (deck_motion_active_ || preload_active) {
    const double desired_target_z_ned_m =
      input.deck_z_ned_m - relative_height_reference_m_;
    if (!std::isfinite(desired_target_z_ned_m)) {
      return std::nullopt;
    }
    const double maximum_step_m = parameters_.max_target_rate_mps * input.dt_s;
    const double target_error_m = desired_target_z_ned_m - vertical_target_z_ned_m_;
    vertical_target_z_ned_m_ += std::clamp(
      target_error_m, -maximum_step_m, maximum_step_m);
  }

  if (!deck_motion_active_) {
    return make_output(
      TouchdownHoldMode::kStationaryDeckHold,
      TouchdownHoldReason::kDeckMotionBelowThreshold,
      std::nullopt);
  }
  return make_output(
    TouchdownHoldMode::kRelativeDeckHold,
    TouchdownHoldReason::kTrackingDeck,
    input.deck_vertical_velocity_ned_mps);
}

void TouchdownHoldController::reset()
{
  relative_height_reference_m_ = 0.0;
  vertical_target_z_ned_m_ = 0.0;
  initialized_ = false;
  deck_motion_active_ = false;
}

bool TouchdownHoldController::initialized() const
{
  return initialized_;
}

TouchdownHoldOutput TouchdownHoldController::make_output(
  TouchdownHoldMode mode,
  TouchdownHoldReason reason,
  std::optional<double> deck_vertical_velocity_ned_mps) const
{
  TouchdownHoldOutput output;
  output.relative_height_reference_m = relative_height_reference_m_;
  output.vertical_target_z_ned_m = vertical_target_z_ned_m_;
  output.deck_vertical_velocity_ned_mps = deck_vertical_velocity_ned_mps;
  output.mode = mode;
  output.reason = reason;
  return output;
}

}  // namespace aruco_precision_landing_cpp
