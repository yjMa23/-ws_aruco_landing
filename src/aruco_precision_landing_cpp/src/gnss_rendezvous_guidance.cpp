// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/gnss_rendezvous_guidance.hpp"

#include "aruco_precision_landing_cpp/coordinate_transform.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace aruco_precision_landing_cpp
{
namespace
{

bool finite_vector(const Eigen::Vector3d & value)
{
  return value.allFinite();
}

bool finite_vector(const Eigen::Vector2d & value)
{
  return value.allFinite();
}

bool positive_finite(double value)
{
  return std::isfinite(value) && value > 0.0;
}

bool same_reference(const Wgs84Position & lhs, const Wgs84Position & rhs)
{
  constexpr double kLatitudeLongitudeToleranceDeg = 1.0e-10;
  constexpr double kAltitudeToleranceM = 1.0e-4;
  return std::abs(lhs.latitude_deg - rhs.latitude_deg) <=
         kLatitudeLongitudeToleranceDeg &&
         std::abs(lhs.longitude_deg - rhs.longitude_deg) <=
         kLatitudeLongitudeToleranceDeg &&
         std::abs(lhs.altitude_m - rhs.altitude_m) <= kAltitudeToleranceM;
}

}  // namespace

GnssRendezvousGuidance::GnssRendezvousGuidance(
  const GnssRendezvousParameters & parameters)
: parameters_(parameters)
{
  if (!positive_finite(parameters_.fix_timeout_s) ||
    !positive_finite(parameters_.velocity_timeout_s) ||
    !std::isfinite(parameters_.stable_duration_s) ||
    parameters_.stable_duration_s < 0.0 ||
    !positive_finite(parameters_.max_fix_jump_m) ||
    !positive_finite(parameters_.max_target_step_m) ||
    !positive_finite(parameters_.max_target_speed_mps) ||
    !std::isfinite(parameters_.search_offset_m) ||
    parameters_.search_offset_m < 0.0 ||
    !positive_finite(parameters_.search_point_hold_s) ||
    !positive_finite(parameters_.max_geodetic_range_m))
  {
    throw std::invalid_argument("Invalid GNSS rendezvous parameters");
  }
}

bool GnssRendezvousGuidance::set_local_reference(const Wgs84Position & reference)
{
  const auto candidate = GeodeticConverter::create(
    reference, parameters_.max_geodetic_range_m);
  if (!candidate.has_value()) {
    return false;
  }
  if (reference_.has_value() && same_reference(*reference_, reference)) {
    return true;
  }

  reference_ = reference;
  converter_ = *candidate;
  reset_measurements();
  return true;
}

bool GnssRendezvousGuidance::ingest_fix(
  const Wgs84Position & fix,
  double receive_time_s)
{
  if (!converter_.has_value() || !std::isfinite(receive_time_s)) {
    return false;
  }

  const auto position_enu = converter_->wgs84_to_local_enu(fix);
  if (!position_enu.has_value()) {
    return false;
  }

  const auto candidate_ned = enu_to_ned(*position_enu);
  if (!candidate_ned.has_value() || !finite_vector(*candidate_ned)) {
    return false;
  }

  const bool previous_fix_stale =
    position_ned_.has_value() &&
    receive_time_s >= last_fix_receive_time_s_ &&
    receive_time_s - last_fix_receive_time_s_ > parameters_.fix_timeout_s;

  if (position_ned_.has_value() && !previous_fix_stale) {
    const double horizontal_jump =
      (candidate_ned->head<2>() - position_ned_->head<2>()).norm();
    if (!std::isfinite(horizontal_jump) || horizontal_jump > parameters_.max_fix_jump_m) {
      return false;
    }
  }

  if (!position_ned_.has_value() || previous_fix_stale ||
    receive_time_s < last_fix_receive_time_s_)
  {
    fix_stable_since_s_ = receive_time_s;
    have_fix_stable_since_ = true;
  }

  position_ned_ = *candidate_ned;
  last_fix_receive_time_s_ = receive_time_s;
  return true;
}

bool GnssRendezvousGuidance::ingest_velocity_enu(
  const Eigen::Vector3d & velocity_enu,
  double receive_time_s)
{
  if (!finite_vector(velocity_enu) || !std::isfinite(receive_time_s)) {
    return false;
  }

  const auto velocity_ned = enu_to_ned(velocity_enu);
  if (!velocity_ned.has_value()) {
    return false;
  }

  const bool previous_velocity_stale =
    velocity_ned_.has_value() &&
    receive_time_s >= last_velocity_receive_time_s_ &&
    receive_time_s - last_velocity_receive_time_s_ > parameters_.velocity_timeout_s;
  if (!velocity_ned_.has_value() || previous_velocity_stale ||
    receive_time_s < last_velocity_receive_time_s_)
  {
    velocity_stable_since_s_ = receive_time_s;
    have_velocity_stable_since_ = true;
  }

  velocity_ned_ = *velocity_ned;
  last_velocity_receive_time_s_ = receive_time_s;
  return true;
}

void GnssRendezvousGuidance::reset_measurements()
{
  position_ned_.reset();
  velocity_ned_.reset();
  last_fix_receive_time_s_ = 0.0;
  last_velocity_receive_time_s_ = 0.0;
  fix_stable_since_s_ = 0.0;
  velocity_stable_since_s_ = 0.0;
  have_fix_stable_since_ = false;
  have_velocity_stable_since_ = false;
}

bool GnssRendezvousGuidance::ready(double now_s) const
{
  if (!std::isfinite(now_s) ||
    !have_fix_stable_since_ || !have_velocity_stable_since_)
  {
    return false;
  }

  return fix_is_fresh(now_s) &&
         velocity_is_fresh(now_s) &&
         now_s >= fix_stable_since_s_ &&
         now_s >= velocity_stable_since_s_ &&
         now_s - fix_stable_since_s_ >= parameters_.stable_duration_s &&
         now_s - velocity_stable_since_s_ >= parameters_.stable_duration_s;
}

std::optional<DeckGnssEstimate> GnssRendezvousGuidance::estimate(double now_s) const
{
  if (!fix_is_fresh(now_s) || !velocity_is_fresh(now_s) ||
    !position_ned_.has_value() || !velocity_ned_.has_value())
  {
    return std::nullopt;
  }

  DeckGnssEstimate result;
  result.position_ned = *position_ned_;
  result.velocity_ned = *velocity_ned_;
  result.fix_receive_time_s = last_fix_receive_time_s_;
  result.velocity_receive_time_s = last_velocity_receive_time_s_;
  return result;
}

std::optional<Eigen::Vector2d> GnssRendezvousGuidance::limit_target_xy(
  const Eigen::Vector2d & current_target_xy,
  const Eigen::Vector2d & desired_target_xy,
  double dt_s) const
{
  if (!finite_vector(current_target_xy) ||
    !finite_vector(desired_target_xy) ||
    !positive_finite(dt_s))
  {
    return std::nullopt;
  }

  const Eigen::Vector2d delta = desired_target_xy - current_target_xy;
  const double distance = delta.norm();
  if (!std::isfinite(distance)) {
    return std::nullopt;
  }
  if (distance <= 0.0) {
    return current_target_xy;
  }

  const double max_step = std::min(
    parameters_.max_target_step_m,
    parameters_.max_target_speed_mps * dt_s);
  if (distance <= max_step) {
    return desired_target_xy;
  }

  return current_target_xy + delta * (max_step / distance);
}

std::optional<Eigen::Vector2d> GnssRendezvousGuidance::search_offset(
  double elapsed_s) const
{
  if (!std::isfinite(elapsed_s) || elapsed_s < 0.0) {
    return std::nullopt;
  }

  const std::array<Eigen::Vector2d, 5> offsets{
    Eigen::Vector2d{0.0, 0.0},
    Eigen::Vector2d{parameters_.search_offset_m, 0.0},
    Eigen::Vector2d{0.0, parameters_.search_offset_m},
    Eigen::Vector2d{-parameters_.search_offset_m, 0.0},
    Eigen::Vector2d{0.0, -parameters_.search_offset_m}};
  const auto index = static_cast<std::size_t>(
    std::floor(elapsed_s / parameters_.search_point_hold_s)) % offsets.size();
  return offsets[index];
}

bool GnssRendezvousGuidance::has_local_reference() const
{
  return converter_.has_value();
}

bool GnssRendezvousGuidance::fix_is_fresh(double now_s) const
{
  return position_ned_.has_value() &&
         std::isfinite(now_s) &&
         now_s >= last_fix_receive_time_s_ &&
         now_s - last_fix_receive_time_s_ <= parameters_.fix_timeout_s;
}

bool GnssRendezvousGuidance::velocity_is_fresh(double now_s) const
{
  return velocity_ned_.has_value() &&
         std::isfinite(now_s) &&
         now_s >= last_velocity_receive_time_s_ &&
         now_s - last_velocity_receive_time_s_ <= parameters_.velocity_timeout_s;
}

}  // namespace aruco_precision_landing_cpp
