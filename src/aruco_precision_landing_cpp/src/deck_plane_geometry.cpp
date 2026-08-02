// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/deck_plane_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace aruco_precision_landing_cpp
{
namespace
{

constexpr double kMinimumQuaternionNorm = 1.0e-6;

bool quaternion_is_finite(const Eigen::Quaterniond & quaternion)
{
  return std::isfinite(quaternion.w()) &&
         std::isfinite(quaternion.x()) &&
         std::isfinite(quaternion.y()) &&
         std::isfinite(quaternion.z());
}

DeckPlaneGeometryResult invalid_result(const std::string & reason)
{
  DeckPlaneGeometryResult result;
  result.valid = false;
  result.failure_reason = reason;
  return result;
}

}  // namespace

std::array<Eigen::Vector3d, 4>
DeckPlaneGeometry::x500_default_contact_points_body_frd_m()
{
  return {{
    Eigen::Vector3d{-0.125, -0.132, 0.227},
    Eigen::Vector3d{0.125, -0.132, 0.227},
    Eigen::Vector3d{-0.125, 0.132, 0.227},
    Eigen::Vector3d{0.125, 0.132, 0.227}}};
}

DeckPlaneGeometryResult DeckPlaneGeometry::compute(
  const DeckPlaneGeometryInput & input,
  const DeckPlaneGeometryParameters & parameters)
{
  if (!std::isfinite(parameters.minimum_normal_norm) ||
    parameters.minimum_normal_norm <= 0.0)
  {
    return invalid_result("minimum_normal_norm must be finite and positive");
  }
  if (!std::isfinite(parameters.minimum_upward_component) ||
    parameters.minimum_upward_component <= 0.0 ||
    parameters.minimum_upward_component > 1.0)
  {
    return invalid_result("minimum_upward_component must be within (0, 1]");
  }
  if (!input.deck_reference_position_ned_m.allFinite()) {
    return invalid_result("deck reference position contains NaN or Inf");
  }
  if (!input.uav_reference_position_ned_m.allFinite()) {
    return invalid_result("UAV reference position contains NaN or Inf");
  }
  if (!input.upward_normal_ned.allFinite()) {
    return invalid_result("deck upward normal contains NaN or Inf");
  }

  const double normal_norm = input.upward_normal_ned.norm();
  if (!std::isfinite(normal_norm) || normal_norm <= parameters.minimum_normal_norm) {
    return invalid_result("deck upward normal norm is too small");
  }
  const Eigen::Vector3d upward_normal_ned = input.upward_normal_ned / normal_norm;
  if (-upward_normal_ned.z() < parameters.minimum_upward_component) {
    return invalid_result("deck normal does not point sufficiently toward NED up");
  }

  if (!quaternion_is_finite(input.body_frd_to_ned)) {
    return invalid_result("body_frd_to_ned quaternion contains NaN or Inf");
  }
  const double quaternion_norm = input.body_frd_to_ned.norm();
  if (!std::isfinite(quaternion_norm) || quaternion_norm <= kMinimumQuaternionNorm) {
    return invalid_result("body_frd_to_ned quaternion norm is too small");
  }
  const Eigen::Quaterniond body_frd_to_ned = input.body_frd_to_ned.normalized();

  for (const auto & contact_point : input.contact_points_body_frd_m) {
    if (!contact_point.allFinite()) {
      return invalid_result("contact point contains NaN or Inf");
    }
  }

  DeckPlaneGeometryResult result;
  result.valid = true;
  auto & output = result.output;
  output.upward_normal_ned = upward_normal_ned;

  const Eigen::Vector3d body_offset_ned =
    input.uav_reference_position_ned_m - input.deck_reference_position_ned_m;
  output.body_normal_gap_m = upward_normal_ned.dot(body_offset_ned);
  if (!std::isfinite(output.body_normal_gap_m)) {
    return invalid_result("computed body normal gap is not finite");
  }

  const Eigen::Matrix3d tangential_projection =
    Eigen::Matrix3d::Identity() - upward_normal_ned * upward_normal_ned.transpose();
  output.tangential_position_error_ned_m = tangential_projection * body_offset_ned;

  output.minimum_contact_gap_m = std::numeric_limits<double>::infinity();
  output.maximum_contact_gap_m = -std::numeric_limits<double>::infinity();
  output.first_contact_index = 0U;
  for (std::size_t index = 0; index < input.contact_points_body_frd_m.size(); ++index) {
    const Eigen::Vector3d contact_arm_ned =
      body_frd_to_ned * input.contact_points_body_frd_m[index];
    output.contact_positions_ned_m[index] =
      input.uav_reference_position_ned_m + contact_arm_ned;
    output.contact_gaps_m[index] = upward_normal_ned.dot(
      output.contact_positions_ned_m[index] - input.deck_reference_position_ned_m);

    if (!output.contact_positions_ned_m[index].allFinite() ||
      !std::isfinite(output.contact_gaps_m[index]))
    {
      return invalid_result("computed contact geometry is not finite");
    }
    if (output.contact_gaps_m[index] < output.minimum_contact_gap_m) {
      output.minimum_contact_gap_m = output.contact_gaps_m[index];
      output.first_contact_index = index;
    }
    output.maximum_contact_gap_m = std::max(
      output.maximum_contact_gap_m, output.contact_gaps_m[index]);
  }
  output.contact_gap_spread_m =
    output.maximum_contact_gap_m - output.minimum_contact_gap_m;

  if (!input.uav_linear_velocity_ned_mps.has_value() ||
    !input.deck_linear_velocity_ned_mps.has_value())
  {
    output.velocity_status = "UAV and deck linear velocities are both required";
    return result;
  }
  if (!input.uav_linear_velocity_ned_mps->allFinite() ||
    !input.deck_linear_velocity_ned_mps->allFinite())
  {
    output.velocity_status = "non-finite linear velocity input";
    return result;
  }

  const Eigen::Vector3d relative_linear_velocity_ned =
    *input.uav_linear_velocity_ned_mps - *input.deck_linear_velocity_ned_mps;
  output.body_normal_relative_velocity_mps =
    upward_normal_ned.dot(relative_linear_velocity_ned);
  output.tangential_relative_velocity_ned_mps =
    tangential_projection * relative_linear_velocity_ned;

  if (!input.uav_angular_velocity_body_frd_radps.has_value()) {
    output.velocity_status =
      "UAV angular velocity is unavailable; contact velocities are invalid";
    return result;
  }
  if (!input.deck_angular_velocity_ned_radps.has_value()) {
    output.velocity_status =
      "deck angular velocity is unavailable; contact velocities are invalid";
    return result;
  }
  if (!input.uav_angular_velocity_body_frd_radps->allFinite() ||
    !input.deck_angular_velocity_ned_radps->allFinite())
  {
    output.velocity_status =
      "non-finite angular velocity input; contact velocities are invalid";
    return result;
  }

  const Eigen::Vector3d uav_angular_velocity_ned =
    body_frd_to_ned * *input.uav_angular_velocity_body_frd_radps;
  for (std::size_t index = 0; index < output.contact_positions_ned_m.size(); ++index) {
    const Eigen::Vector3d uav_arm_ned =
      output.contact_positions_ned_m[index] - input.uav_reference_position_ned_m;
    const Eigen::Vector3d deck_arm_ned =
      output.contact_positions_ned_m[index] - input.deck_reference_position_ned_m;
    const Eigen::Vector3d contact_relative_velocity_ned =
      *input.uav_linear_velocity_ned_mps + uav_angular_velocity_ned.cross(uav_arm_ned) -
      *input.deck_linear_velocity_ned_mps -
      input.deck_angular_velocity_ned_radps->cross(deck_arm_ned);
    const double normal_relative_velocity =
      upward_normal_ned.dot(contact_relative_velocity_ned);
    if (!std::isfinite(normal_relative_velocity)) {
      output.contact_normal_relative_velocity_mps = {};
      output.velocity_status = "computed contact velocity is not finite";
      return result;
    }
    output.contact_normal_relative_velocity_mps[index] = normal_relative_velocity;
  }
  output.velocity_status = "all velocity diagnostics valid";
  return result;
}

}  // namespace aruco_precision_landing_cpp
