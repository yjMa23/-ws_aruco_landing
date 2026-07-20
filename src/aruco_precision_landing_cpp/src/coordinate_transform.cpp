// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/coordinate_transform.hpp"

#include <cmath>

namespace aruco_precision_landing_cpp
{
namespace
{

constexpr double kMinimumQuaternionNorm = 1.0e-6;

bool vector_is_finite(const Eigen::Vector3d & vector)
{
  return vector.allFinite();
}

bool quaternion_is_finite(const Eigen::Quaterniond & quaternion)
{
  return std::isfinite(quaternion.w()) &&
         std::isfinite(quaternion.x()) &&
         std::isfinite(quaternion.y()) &&
         std::isfinite(quaternion.z());
}

}  // namespace

std::optional<Eigen::Isometry3d> make_isometry(
  const Eigen::Vector3d & translation,
  const Eigen::Quaterniond & rotation_from_child_to_parent)
{
  if (!vector_is_finite(translation) || !quaternion_is_finite(rotation_from_child_to_parent)) {
    return std::nullopt;
  }

  const double quaternion_norm = rotation_from_child_to_parent.norm();
  if (!std::isfinite(quaternion_norm) || quaternion_norm <= kMinimumQuaternionNorm) {
    return std::nullopt;
  }

  const Eigen::Quaterniond normalized_rotation = rotation_from_child_to_parent.normalized();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = normalized_rotation.toRotationMatrix();
  transform.translation() = translation;
  return transform;
}

std::optional<Pose3d> transform_marker_to_local_ned(
  const Pose3d & local_body_pose,
  PoseReferenceFrame local_pose_frame,
  const Pose3d & body_camera_pose,
  const Pose3d & camera_marker_pose)
{
  if (local_pose_frame != PoseReferenceFrame::kLocalNed) {
    return std::nullopt;
  }

  const auto local_body_transform =
    make_isometry(local_body_pose.translation, local_body_pose.rotation);
  const auto body_camera_transform =
    make_isometry(body_camera_pose.translation, body_camera_pose.rotation);
  const auto camera_marker_transform =
    make_isometry(camera_marker_pose.translation, camera_marker_pose.rotation);

  if (!local_body_transform || !body_camera_transform || !camera_marker_transform) {
    return std::nullopt;
  }

  const Eigen::Isometry3d local_marker_transform =
    *local_body_transform * *body_camera_transform * *camera_marker_transform;

  Pose3d local_marker_pose;
  local_marker_pose.translation = local_marker_transform.translation();
  local_marker_pose.rotation = Eigen::Quaterniond(local_marker_transform.rotation()).normalized();

  if (!vector_is_finite(local_marker_pose.translation) ||
    !quaternion_is_finite(local_marker_pose.rotation))
  {
    return std::nullopt;
  }

  return local_marker_pose;
}

std::optional<Eigen::Vector3d> enu_to_ned(const Eigen::Vector3d & vector_enu)
{
  if (!vector_is_finite(vector_enu)) {
    return std::nullopt;
  }

  return Eigen::Vector3d(vector_enu.y(), vector_enu.x(), -vector_enu.z());
}

std::optional<Eigen::Vector3d> ned_to_enu(const Eigen::Vector3d & vector_ned)
{
  if (!vector_is_finite(vector_ned)) {
    return std::nullopt;
  }

  return Eigen::Vector3d(vector_ned.y(), vector_ned.x(), -vector_ned.z());
}

}  // namespace aruco_precision_landing_cpp
