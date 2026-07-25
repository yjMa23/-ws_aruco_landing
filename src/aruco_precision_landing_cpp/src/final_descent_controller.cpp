// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/final_descent_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aruco_precision_landing_cpp
{
namespace
{

constexpr double kReferenceChangeTolerance = 1.0e-12;

bool positive_finite(double value)
{
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

FinalDescentController::FinalDescentController(
  const FinalDescentParameters & parameters)
: parameters_(parameters)
{
  if (!positive_finite(parameters_.entry_height_m) ||
    !positive_finite(parameters_.final_descent_rate_mps) ||
    !positive_finite(parameters_.minimum_command_height_m) ||
    !positive_finite(parameters_.maximum_reference_tracking_error_m))
  {
    throw std::invalid_argument(
            "final-descent heights, rate, and tracking error must be finite and positive");
  }
  if (parameters_.minimum_command_height_m >= parameters_.entry_height_m) {
    throw std::invalid_argument(
            "minimum_command_height_m must be smaller than entry_height_m");
  }
}

std::optional<FinalDescentOutput> FinalDescentController::update(
  const FinalDescentInput & input)
{
  if (!input.vertical_reference_valid ||
    !std::isfinite(input.current_relative_height_m) ||
    input.current_relative_height_m < 0.0 ||
    !std::isfinite(input.current_reference_height_m) ||
    input.current_reference_height_m < 0.0 ||
    !positive_finite(input.dt_s))
  {
    return std::nullopt;
  }

  if (!initialized_) {
    relative_height_reference_m_ = std::clamp(
      input.current_reference_height_m,
      parameters_.minimum_command_height_m,
      parameters_.entry_height_m);
    initialized_ = true;
    return make_output(
      relative_height_reference_m_,
      0.0,
      input.final_descent_authorized ?
      FinalDescentPhase::kPaused :
      FinalDescentPhase::kWaitingAuthorization);
  }

  const double previous_reference_m = relative_height_reference_m_;

  if (touchdown_confirmed_latched_ ||
    input.touchdown_status == TouchdownStatus::kConfirmed)
  {
    touchdown_confirmed_latched_ = true;
    return make_output(
      previous_reference_m,
      0.0,
      FinalDescentPhase::kTouchdownHold);
  }

  if (input.touchdown_status == TouchdownStatus::kRejectedUnsafe) {
    return make_output(
      previous_reference_m,
      0.0,
      FinalDescentPhase::kRecoveryRequested);
  }

  if (input.touchdown_status == TouchdownStatus::kCandidate) {
    return make_output(
      previous_reference_m,
      0.0,
      FinalDescentPhase::kCandidateHold);
  }

  if (!input.final_descent_authorized) {
    return make_output(
      previous_reference_m,
      0.0,
      FinalDescentPhase::kWaitingAuthorization);
  }

  if (!input.landing_window_open ||
    input.touchdown_status == TouchdownStatus::kInsufficientEvidence)
  {
    return make_output(previous_reference_m, 0.0, FinalDescentPhase::kPaused);
  }

  const double tracking_error_m =
    std::abs(input.current_relative_height_m - relative_height_reference_m_);
  if (tracking_error_m > parameters_.maximum_reference_tracking_error_m) {
    return make_output(previous_reference_m, 0.0, FinalDescentPhase::kPaused);
  }

  if (relative_height_reference_m_ <= parameters_.minimum_command_height_m) {
    relative_height_reference_m_ = parameters_.minimum_command_height_m;
    return make_output(previous_reference_m, 0.0, FinalDescentPhase::kPaused);
  }

  relative_height_reference_m_ = std::max(
    parameters_.minimum_command_height_m,
    relative_height_reference_m_ -
    parameters_.final_descent_rate_mps * input.dt_s);

  const double velocity_ned_mps =
    relative_height_reference_m_ < previous_reference_m ?
    parameters_.final_descent_rate_mps : 0.0;
  return make_output(
    previous_reference_m,
    velocity_ned_mps,
    FinalDescentPhase::kDescending);
}

void FinalDescentController::reset()
{
  relative_height_reference_m_ = 0.0;
  initialized_ = false;
  touchdown_confirmed_latched_ = false;
}

bool FinalDescentController::initialized() const
{
  return initialized_;
}

FinalDescentOutput FinalDescentController::make_output(
  double previous_reference_m,
  double vertical_reference_velocity_ned_mps,
  FinalDescentPhase phase) const
{
  FinalDescentOutput output;
  output.relative_height_reference_m = relative_height_reference_m_;
  output.vertical_reference_velocity_ned_mps = vertical_reference_velocity_ned_mps;
  output.phase = phase;
  output.reference_changed =
    std::abs(relative_height_reference_m_ - previous_reference_m) >
    kReferenceChangeTolerance;
  output.touchdown_candidate_hold =
    phase == FinalDescentPhase::kCandidateHold;
  output.touchdown_confirmed_hold =
    phase == FinalDescentPhase::kTouchdownHold;
  output.recovery_requested =
    phase == FinalDescentPhase::kRecoveryRequested;
  return output;
}

}  // namespace aruco_precision_landing_cpp
