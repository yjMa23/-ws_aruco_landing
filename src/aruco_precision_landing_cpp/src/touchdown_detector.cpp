// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/touchdown_detector.hpp"

#include <cmath>
#include <stdexcept>

namespace aruco_precision_landing_cpp
{
namespace
{

bool positive_finite(double value)
{
  return std::isfinite(value) && value > 0.0;
}

void add_evidence(std::uint32_t & mask, bool condition, TouchdownEvidence evidence)
{
  if (condition) {
    mask |= touchdown_evidence_mask(evidence);
  }
}

}  // namespace

TouchdownDetector::TouchdownDetector(const TouchdownDetectorParameters & parameters)
: parameters_(parameters)
{
  if (!positive_finite(parameters_.px4_status_timeout_s) ||
    !positive_finite(parameters_.visual_timeout_s) ||
    !positive_finite(parameters_.low_height_enter_m) ||
    !positive_finite(parameters_.low_height_exit_m) ||
    !positive_finite(parameters_.max_relative_vertical_speed_mps) ||
    !positive_finite(parameters_.max_uav_vertical_speed_mps) ||
    !positive_finite(parameters_.max_relative_horizontal_speed_mps) ||
    !positive_finite(parameters_.candidate_required_duration_s))
  {
    throw std::invalid_argument("touchdown detector parameters must be finite and positive");
  }
  if (parameters_.low_height_exit_m <= parameters_.low_height_enter_m) {
    throw std::invalid_argument(
            "low_height_exit_m must be greater than low_height_enter_m");
  }
}

TouchdownDetectorOutput TouchdownDetector::update(const TouchdownDetectorInput & input)
{
  if (confirmed_latched_) {
    return make_output(TouchdownStatus::kConfirmed, 0U);
  }

  if (!std::isfinite(input.sample_time_s)) {
    clear_candidate();
    have_last_sample_time_ = false;
    return make_output(TouchdownStatus::kRejectedUnsafe, 0U);
  }

  double dt_s = 0.0;
  if (have_last_sample_time_) {
    dt_s = input.sample_time_s - last_sample_time_s_;
    if (!std::isfinite(dt_s) || dt_s <= 0.0) {
      clear_candidate();
      visual_low_height_latched_ = false;
      last_sample_time_s_ = input.sample_time_s;
      return make_output(TouchdownStatus::kRejectedUnsafe, 0U);
    }
  }
  last_sample_time_s_ = input.sample_time_s;
  have_last_sample_time_ = true;

  if (!input.state_allows_touchdown_detection) {
    clear_candidate();
    visual_low_height_latched_ = false;
    return make_output(TouchdownStatus::kAirborne, 0U);
  }

  const bool px4_status_fresh =
    input.px4_land_status_valid &&
    std::isfinite(input.px4_land_status_age_s) &&
    input.px4_land_status_age_s >= 0.0 &&
    input.px4_land_status_age_s <= parameters_.px4_status_timeout_s;
  if (!px4_status_fresh) {
    clear_candidate();
    return make_output(TouchdownStatus::kInsufficientEvidence, 0U);
  }

  std::uint32_t evidence_mask = 0U;
  add_evidence(
    evidence_mask, input.ground_contact, TouchdownEvidence::kGroundContact);
  add_evidence(
    evidence_mask, input.maybe_landed, TouchdownEvidence::kMaybeLanded);
  add_evidence(evidence_mask, input.landed, TouchdownEvidence::kLanded);
  add_evidence(evidence_mask, input.at_rest, TouchdownEvidence::kAtRest);
  add_evidence(
    evidence_mask, input.has_low_throttle, TouchdownEvidence::kLowThrottle);
  add_evidence(
    evidence_mask, input.close_to_ground, TouchdownEvidence::kCloseToGround);

  if (input.freefall) {
    clear_candidate();
    visual_low_height_latched_ = false;
    return make_output(TouchdownStatus::kRejectedUnsafe, evidence_mask);
  }

  const bool visual_fresh =
    input.visual_height_valid &&
    std::isfinite(input.visual_height_age_s) &&
    input.visual_height_age_s >= 0.0 &&
    input.visual_height_age_s <= parameters_.visual_timeout_s &&
    std::isfinite(input.relative_height_m) && input.relative_height_m >= 0.0;
  if (visual_fresh) {
    if (visual_low_height_latched_) {
      visual_low_height_latched_ =
        input.relative_height_m <= parameters_.low_height_exit_m;
    } else {
      visual_low_height_latched_ =
        input.relative_height_m <= parameters_.low_height_enter_m;
    }
  } else {
    visual_low_height_latched_ = false;
  }
  add_evidence(
    evidence_mask,
    visual_low_height_latched_,
    TouchdownEvidence::kVisualLowHeight);

  const bool relative_speed_valid =
    std::isfinite(input.relative_vertical_velocity_mps);
  const bool uav_speed_valid = std::isfinite(input.uav_vertical_velocity_mps);
  const bool relative_horizontal_speed_valid =
    input.relative_horizontal_speed_valid &&
    std::isfinite(input.relative_horizontal_speed_mps) &&
    input.relative_horizontal_speed_mps >= 0.0;
  const bool low_relative_speed =
    relative_speed_valid &&
    std::abs(input.relative_vertical_velocity_mps) <=
    parameters_.max_relative_vertical_speed_mps;
  const bool low_uav_speed =
    uav_speed_valid &&
    std::abs(input.uav_vertical_velocity_mps) <=
    parameters_.max_uav_vertical_speed_mps;
  const bool low_relative_horizontal_speed =
    relative_horizontal_speed_valid &&
    input.relative_horizontal_speed_mps <=
    parameters_.max_relative_horizontal_speed_mps;
  const bool no_reported_movement =
    !input.vertical_movement &&
    !input.horizontal_movement &&
    !input.rotational_movement;
  const bool horizontal_motion_compatible =
    !input.horizontal_movement || low_relative_horizontal_speed;
  const bool movement_compatible =
    !input.vertical_movement &&
    !input.rotational_movement &&
    horizontal_motion_compatible;

  add_evidence(
    evidence_mask,
    low_relative_speed,
    TouchdownEvidence::kLowRelativeVerticalSpeed);
  add_evidence(
    evidence_mask,
    low_uav_speed,
    TouchdownEvidence::kLowUavVerticalSpeed);
  add_evidence(
    evidence_mask,
    no_reported_movement,
    TouchdownEvidence::kNoReportedMovement);
  add_evidence(
    evidence_mask,
    low_relative_horizontal_speed,
    TouchdownEvidence::kLowRelativeHorizontalSpeed);

  const bool strong_touchdown_evidence =
    input.landed && input.at_rest && movement_compatible;
  const bool contact_evidence =
    input.ground_contact || input.maybe_landed || input.landed;
  const bool normal_touchdown_evidence =
    contact_evidence && visual_fresh && visual_low_height_latched_ &&
    low_relative_speed && low_uav_speed && movement_compatible;

  if (!strong_touchdown_evidence && !normal_touchdown_evidence) {
    clear_candidate();
    if (!visual_fresh && !strong_touchdown_evidence) {
      return make_output(TouchdownStatus::kInsufficientEvidence, evidence_mask);
    }
    if (!relative_speed_valid || !uav_speed_valid ||
      (input.horizontal_movement && !relative_horizontal_speed_valid))
    {
      return make_output(TouchdownStatus::kInsufficientEvidence, evidence_mask);
    }
    return make_output(TouchdownStatus::kAirborne, evidence_mask);
  }

  candidate_duration_s_ += dt_s;
  if (candidate_duration_s_ >= parameters_.candidate_required_duration_s) {
    confirmed_latched_ = true;
    return make_output(TouchdownStatus::kConfirmed, evidence_mask);
  }
  return make_output(TouchdownStatus::kCandidate, evidence_mask);
}

void TouchdownDetector::reset()
{
  last_sample_time_s_ = 0.0;
  candidate_duration_s_ = 0.0;
  have_last_sample_time_ = false;
  visual_low_height_latched_ = false;
  confirmed_latched_ = false;
}

TouchdownDetectorOutput TouchdownDetector::make_output(
  TouchdownStatus status,
  std::uint32_t evidence_mask) const
{
  TouchdownDetectorOutput output;
  output.status = status;
  output.evidence_mask = evidence_mask;
  output.candidate_duration_s = candidate_duration_s_;
  output.confirmed_latched = confirmed_latched_;
  return output;
}

void TouchdownDetector::clear_candidate()
{
  candidate_duration_s_ = 0.0;
}

}  // namespace aruco_precision_landing_cpp
