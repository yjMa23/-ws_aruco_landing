// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/relative_descent_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aruco_precision_landing_cpp
{
namespace
{

constexpr double kReferenceChangeTolerance = 1.0e-12;

bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

RelativeDescentController::RelativeDescentController(
  const RelativeDescentParameters & parameters)
: parameters_(parameters)
{
  if (!finite_positive(parameters_.minimum_test_height_m) ||
    !finite_positive(parameters_.fast_height_threshold_m) ||
    !finite_positive(parameters_.slow_height_threshold_m) ||
    !finite_positive(parameters_.fast_rate_mps) ||
    !finite_positive(parameters_.medium_rate_mps) ||
    !finite_positive(parameters_.slow_rate_mps) ||
    !finite_positive(parameters_.recovery_height_m) ||
    !finite_positive(parameters_.recovery_rate_mps) ||
    !finite_positive(parameters_.max_reference_tracking_error_m))
  {
    throw std::invalid_argument(
            "relative-descent heights, rates, and tracking error must be finite and positive");
  }
  if (parameters_.minimum_test_height_m >= parameters_.slow_height_threshold_m ||
    parameters_.slow_height_threshold_m >= parameters_.fast_height_threshold_m)
  {
    throw std::invalid_argument(
            "height thresholds must satisfy minimum < slow < fast");
  }
  if (parameters_.recovery_height_m <= parameters_.minimum_test_height_m) {
    throw std::invalid_argument(
            "recovery_height_m must be greater than minimum_test_height_m");
  }
}

std::optional<RelativeDescentOutput> RelativeDescentController::update(
  const RelativeDescentInput & input)
{
  if (!input.vertical_reference_valid ||
    !std::isfinite(input.current_relative_height_m) ||
    input.current_relative_height_m < 0.0 ||
    !finite_positive(input.dt_s))
  {
    return std::nullopt;
  }

  if (!initialized_) {
    height_reference_m_ = std::max(
      input.current_relative_height_m, parameters_.minimum_test_height_m);
    initialized_ = true;
    const RelativeDescentPhase phase = input.severe_failure ?
      RelativeDescentPhase::kRecovering :
      (input.window_open ? RelativeDescentPhase::kDescending :
      RelativeDescentPhase::kWaitingWindow);
    return make_output(height_reference_m_, phase);
  }

  const double previous_reference_m = height_reference_m_;

  if (input.severe_failure) {
    if (height_reference_m_ < parameters_.recovery_height_m) {
      height_reference_m_ = std::min(
        parameters_.recovery_height_m,
        height_reference_m_ + parameters_.recovery_rate_mps * input.dt_s);
      descent_started_ = true;
      return make_output(previous_reference_m, RelativeDescentPhase::kRecovering);
    }
    return make_output(previous_reference_m, RelativeDescentPhase::kPaused);
  }

  if (!input.window_open) {
    const RelativeDescentPhase phase = descent_started_ ?
      RelativeDescentPhase::kPaused : RelativeDescentPhase::kWaitingWindow;
    return make_output(previous_reference_m, phase);
  }

  if (height_reference_m_ <= parameters_.minimum_test_height_m) {
    height_reference_m_ = parameters_.minimum_test_height_m;
    return make_output(previous_reference_m, RelativeDescentPhase::kTestHeightHold);
  }

  const double tracking_error_m =
    std::abs(input.current_relative_height_m - height_reference_m_);
  if (tracking_error_m > parameters_.max_reference_tracking_error_m) {
    return make_output(previous_reference_m, RelativeDescentPhase::kPaused);
  }

  const double rate_mps = descent_rate(height_reference_m_);
  height_reference_m_ = std::max(
    parameters_.minimum_test_height_m,
    height_reference_m_ - rate_mps * input.dt_s);
  descent_started_ = true;

  const RelativeDescentPhase phase =
    height_reference_m_ <= parameters_.minimum_test_height_m ?
    RelativeDescentPhase::kTestHeightHold : RelativeDescentPhase::kDescending;
  return make_output(previous_reference_m, phase);
}

void RelativeDescentController::reset()
{
  height_reference_m_ = 0.0;
  initialized_ = false;
  descent_started_ = false;
}

bool RelativeDescentController::initialized() const
{
  return initialized_;
}

double RelativeDescentController::descent_rate(double height_reference_m) const
{
  if (height_reference_m > parameters_.fast_height_threshold_m) {
    return parameters_.fast_rate_mps;
  }
  if (height_reference_m > parameters_.slow_height_threshold_m) {
    return parameters_.medium_rate_mps;
  }
  return parameters_.slow_rate_mps;
}

RelativeDescentOutput RelativeDescentController::make_output(
  double previous_reference_m,
  RelativeDescentPhase phase) const
{
  RelativeDescentOutput output;
  output.height_reference_m = height_reference_m_;
  output.phase = phase;
  output.reference_changed =
    std::abs(height_reference_m_ - previous_reference_m) > kReferenceChangeTolerance;
  output.reached_test_height =
    height_reference_m_ <= parameters_.minimum_test_height_m;
  return output;
}

}  // namespace aruco_precision_landing_cpp
