// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/visual_handover_guidance.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace aruco_precision_landing_cpp
{
namespace
{

bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

bool finite_vector2(const Eigen::Vector2d & value)
{
  return value.array().isFinite().all();
}

bool finite_vector3(const Eigen::Vector3d & value)
{
  return value.array().isFinite().all();
}

}  // namespace

VisualHandoverGuidance::VisualHandoverGuidance(
  const VisualHandoverParameters & parameters)
: parameters_(parameters)
{
  const auto require_positive = [](const char * name, double value) {
      if (!finite_positive(value)) {
        throw std::invalid_argument(
                std::string("Parameter '") + name + "' must be finite and positive");
      }
    };

  require_positive("handover_duration_s", parameters_.handover_duration_s);
  require_positive(
    "max_gnss_visual_difference_m", parameters_.max_gnss_visual_difference_m);
  require_positive(
    "max_visual_measurement_jump_m", parameters_.max_visual_measurement_jump_m);
  require_positive(
    "visual_loss_short_timeout_s", parameters_.visual_loss_short_timeout_s);
  require_positive(
    "visual_loss_long_timeout_s", parameters_.visual_loss_long_timeout_s);
  require_positive("max_target_speed_mps", parameters_.max_target_speed_mps);
  require_positive("max_target_step_m", parameters_.max_target_step_m);

  if (parameters_.visual_loss_long_timeout_s <=
    parameters_.visual_loss_short_timeout_s)
  {
    throw std::invalid_argument(
            "visual_loss_long_timeout_s must be greater than short timeout");
  }
}

bool VisualHandoverGuidance::update_visual_position(
  const Eigen::Vector3d & position_ned,
  double receipt_time_s)
{
  if (!finite_vector3(position_ned) || !std::isfinite(receipt_time_s) || receipt_time_s < 0.0) {
    return false;
  }

  if (have_visual_position_) {
    if (receipt_time_s < last_visual_time_s_) {
      return false;
    }

    const double age_s = receipt_time_s - last_visual_time_s_;
    const double horizontal_jump_m =
      (position_ned.head<2>() - last_visual_position_ned_.head<2>()).norm();
    if (age_s <= parameters_.visual_loss_long_timeout_s &&
      horizontal_jump_m > parameters_.max_visual_measurement_jump_m)
    {
      return false;
    }
  }

  last_visual_position_ned_ = position_ned;
  last_visual_time_s_ = receipt_time_s;
  have_visual_position_ = true;
  return true;
}

std::optional<Eigen::Vector3d> VisualHandoverGuidance::visual_position(double now_s) const
{
  if (!have_visual_position_ || !std::isfinite(now_s) || now_s < last_visual_time_s_) {
    return std::nullopt;
  }

  if ((now_s - last_visual_time_s_) > parameters_.visual_loss_short_timeout_s) {
    return std::nullopt;
  }
  return last_visual_position_ned_;
}

bool VisualHandoverGuidance::consistent_with_gnss(
  const Eigen::Vector3d & visual_position_ned,
  const Eigen::Vector3d & gnss_position_ned) const
{
  if (!finite_vector3(visual_position_ned) || !finite_vector3(gnss_position_ned)) {
    return false;
  }
  return (visual_position_ned.head<2>() - gnss_position_ned.head<2>()).norm() <=
         parameters_.max_gnss_visual_difference_m;
}

VisualLossState VisualHandoverGuidance::loss_state(
  bool currently_valid,
  double now_s) const
{
  if (currently_valid) {
    return VisualLossState::kFresh;
  }
  if (!have_visual_position_ || !std::isfinite(now_s) || now_s < last_visual_time_s_) {
    return VisualLossState::kLongLoss;
  }
  return (now_s - last_visual_time_s_) >= parameters_.visual_loss_long_timeout_s ?
         VisualLossState::kLongLoss : VisualLossState::kShortLoss;
}

std::optional<double> VisualHandoverGuidance::handover_alpha(double elapsed_s) const
{
  if (!std::isfinite(elapsed_s) || elapsed_s < 0.0) {
    return std::nullopt;
  }
  return std::clamp(elapsed_s / parameters_.handover_duration_s, 0.0, 1.0);
}

std::optional<Eigen::Vector2d> VisualHandoverGuidance::blended_target_xy(
  const Eigen::Vector2d & gnss_target_xy,
  const Eigen::Vector2d & visual_target_xy,
  double elapsed_s) const
{
  if (!finite_vector2(gnss_target_xy) || !finite_vector2(visual_target_xy)) {
    return std::nullopt;
  }
  const auto alpha = handover_alpha(elapsed_s);
  if (!alpha.has_value()) {
    return std::nullopt;
  }
  return (1.0 - *alpha) * gnss_target_xy + *alpha * visual_target_xy;
}

std::optional<Eigen::Vector2d> VisualHandoverGuidance::limit_target_xy(
  const Eigen::Vector2d & current_target_xy,
  const Eigen::Vector2d & desired_target_xy,
  double dt_s) const
{
  if (!finite_vector2(current_target_xy) || !finite_vector2(desired_target_xy) ||
    !finite_positive(dt_s))
  {
    return std::nullopt;
  }

  const Eigen::Vector2d delta = desired_target_xy - current_target_xy;
  const double distance = delta.norm();
  if (distance == 0.0) {
    return current_target_xy;
  }

  const double max_distance = std::min(
    parameters_.max_target_step_m,
    parameters_.max_target_speed_mps * dt_s);
  if (distance <= max_distance) {
    return desired_target_xy;
  }
  return current_target_xy + delta * (max_distance / distance);
}

void VisualHandoverGuidance::reset()
{
  last_visual_position_ned_.setZero();
  last_visual_time_s_ = 0.0;
  have_visual_position_ = false;
}

}  // namespace aruco_precision_landing_cpp
