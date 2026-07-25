// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/landing_window.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aruco_precision_landing_cpp
{
namespace
{

bool finite_non_negative(double value)
{
  return std::isfinite(value) && value >= 0.0;
}

void add_reason(std::uint32_t & reasons, LandingWindowRejectReason reason)
{
  reasons |= landing_window_reason_mask(reason);
}

}  // namespace

LandingWindow::LandingWindow(const LandingWindowParameters & parameters)
: parameters_(parameters)
{
  const bool scalar_parameters_valid =
    finite_non_negative(parameters_.enter_horizontal_error_m) &&
    finite_non_negative(parameters_.exit_horizontal_error_m) &&
    finite_non_negative(parameters_.enter_relative_speed_mps) &&
    finite_non_negative(parameters_.exit_relative_speed_mps) &&
    finite_non_negative(parameters_.enter_max_tilt_rad) &&
    finite_non_negative(parameters_.exit_max_tilt_rad) &&
    finite_non_negative(parameters_.max_visual_age_s) &&
    finite_non_negative(parameters_.minimum_relative_height_m) &&
    finite_non_negative(parameters_.maximum_relative_height_m) &&
    finite_non_negative(parameters_.required_duration_s);
  if (!scalar_parameters_valid) {
    throw std::invalid_argument("landing-window parameters must be finite and non-negative");
  }
  if (parameters_.enter_horizontal_error_m >= parameters_.exit_horizontal_error_m) {
    throw std::invalid_argument(
            "enter_horizontal_error_m must be smaller than exit_horizontal_error_m");
  }
  if (parameters_.enter_relative_speed_mps >= parameters_.exit_relative_speed_mps) {
    throw std::invalid_argument(
            "enter_relative_speed_mps must be smaller than exit_relative_speed_mps");
  }
  if (parameters_.enter_max_tilt_rad >= parameters_.exit_max_tilt_rad) {
    throw std::invalid_argument(
            "enter_max_tilt_rad must be smaller than exit_max_tilt_rad");
  }
  if (parameters_.minimum_relative_height_m >= parameters_.maximum_relative_height_m) {
    throw std::invalid_argument(
            "minimum_relative_height_m must be smaller than maximum_relative_height_m");
  }
}

LandingWindowResult LandingWindow::update(const LandingWindowInput & input)
{
  LandingWindowResult result;

  if (!std::isfinite(input.now_s) || input.now_s < 0.0 ||
    (have_last_update_time_ && input.now_s < last_update_time_s_))
  {
    reset();
    result.reject_reasons = landing_window_reason_mask(
      LandingWindowRejectReason::kInvalidTime);
    return result;
  }

  last_update_time_s_ = input.now_s;
  have_last_update_time_ = true;

  const std::uint32_t reject_reasons = evaluate_reject_reasons(input, window_open_);
  if (reject_reasons != landing_window_reason_mask(LandingWindowRejectReason::kNone)) {
    window_open_ = false;
    have_satisfied_since_ = false;
    satisfied_since_s_ = 0.0;
    result.reject_reasons = reject_reasons;
    return result;
  }

  if (!have_satisfied_since_) {
    satisfied_since_s_ = input.now_s;
    have_satisfied_since_ = true;
  }

  const double satisfied_duration_s = std::max(0.0, input.now_s - satisfied_since_s_);
  if (!window_open_ && satisfied_duration_s >= parameters_.required_duration_s) {
    window_open_ = true;
  }

  result.window_open = window_open_;
  result.conditions_currently_satisfied = true;
  result.satisfied_duration_s = satisfied_duration_s;
  result.reject_reasons = landing_window_reason_mask(LandingWindowRejectReason::kNone);
  return result;
}

void LandingWindow::reset()
{
  window_open_ = false;
  have_satisfied_since_ = false;
  have_last_update_time_ = false;
  satisfied_since_s_ = 0.0;
  last_update_time_s_ = 0.0;
}

std::uint32_t LandingWindow::evaluate_reject_reasons(
  const LandingWindowInput & input,
  bool use_exit_thresholds) const
{
  std::uint32_t reasons = landing_window_reason_mask(LandingWindowRejectReason::kNone);

  if (!input.visual_fresh) {
    add_reason(reasons, LandingWindowRejectReason::kVisualUnavailable);
  }
  if (!finite_non_negative(input.visual_age_s) ||
    input.visual_age_s > parameters_.max_visual_age_s)
  {
    add_reason(reasons, LandingWindowRejectReason::kVisualTooOld);
  }
  if (!input.estimate_valid) {
    add_reason(reasons, LandingWindowRejectReason::kEstimateInvalid);
  }
  if (!input.prediction_valid) {
    add_reason(reasons, LandingWindowRejectReason::kPredictionInvalid);
  }

  const double horizontal_error_limit = use_exit_thresholds ?
    parameters_.exit_horizontal_error_m : parameters_.enter_horizontal_error_m;
  if (!finite_non_negative(input.horizontal_error_m) ||
    input.horizontal_error_m > horizontal_error_limit)
  {
    add_reason(reasons, LandingWindowRejectReason::kHorizontalError);
  }

  const double relative_speed_limit = use_exit_thresholds ?
    parameters_.exit_relative_speed_mps : parameters_.enter_relative_speed_mps;
  if (!finite_non_negative(input.horizontal_relative_speed_mps) ||
    input.horizontal_relative_speed_mps > relative_speed_limit)
  {
    add_reason(reasons, LandingWindowRejectReason::kRelativeSpeed);
  }

  const double tilt_limit = use_exit_thresholds ?
    parameters_.exit_max_tilt_rad : parameters_.enter_max_tilt_rad;
  const double cosine_tilt =
    std::cos(input.deck_roll_rad) * std::cos(input.deck_pitch_rad);
  const double maximum_tilt = std::acos(std::clamp(cosine_tilt, -1.0, 1.0));
  if (!std::isfinite(input.deck_roll_rad) ||
    !std::isfinite(input.deck_pitch_rad) ||
    !std::isfinite(maximum_tilt) || maximum_tilt > tilt_limit)
  {
    add_reason(reasons, LandingWindowRejectReason::kDeckTilt);
  }

  if (!std::isfinite(input.relative_height_m) ||
    input.relative_height_m < parameters_.minimum_relative_height_m ||
    input.relative_height_m > parameters_.maximum_relative_height_m)
  {
    add_reason(reasons, LandingWindowRejectReason::kRelativeHeight);
  }

  return reasons;
}

}  // namespace aruco_precision_landing_cpp
