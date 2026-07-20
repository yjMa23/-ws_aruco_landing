// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/coordinate_transform.hpp"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

namespace aruco_precision_landing_cpp
{
namespace
{

constexpr double kTolerance = 1.0e-9;
constexpr double kPi = 3.14159265358979323846;

void expect_vector_near(
  const Eigen::Vector3d & actual,
  const Eigen::Vector3d & expected,
  double tolerance = kTolerance)
{
  EXPECT_NEAR(actual.x(), expected.x(), tolerance);
  EXPECT_NEAR(actual.y(), expected.y(), tolerance);
  EXPECT_NEAR(actual.z(), expected.z(), tolerance);
}

void expect_same_rotation(
  const Eigen::Quaterniond & actual,
  const Eigen::Quaterniond & expected,
  double tolerance = kTolerance)
{
  EXPECT_NEAR(std::abs(actual.normalized().dot(expected.normalized())), 1.0, tolerance);
}

TEST(CoordinateTransformTest, IdentityTransformsKeepMarkerPose)
{
  Pose3d camera_marker;
  camera_marker.translation = Eigen::Vector3d(1.0, -2.0, 3.0);
  camera_marker.rotation = Eigen::Quaterniond(
    Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));

  const auto result = transform_marker_to_local_ned(
    Pose3d{}, PoseReferenceFrame::kLocalNed, Pose3d{}, camera_marker);

  ASSERT_TRUE(result.has_value());
  expect_vector_near(result->translation, camera_marker.translation);
  expect_same_rotation(result->rotation, camera_marker.rotation);
}

TEST(CoordinateTransformTest, CameraTranslationIsIncluded)
{
  Pose3d body_camera;
  body_camera.translation = Eigen::Vector3d(0.2, -0.1, 0.3);

  Pose3d camera_marker;
  camera_marker.translation = Eigen::Vector3d(1.0, 2.0, 3.0);

  const auto result = transform_marker_to_local_ned(
    Pose3d{}, PoseReferenceFrame::kLocalNed, body_camera, camera_marker);

  ASSERT_TRUE(result.has_value());
  expect_vector_near(result->translation, Eigen::Vector3d(1.2, 1.9, 3.3));
}

TEST(CoordinateTransformTest, NominalDownwardCameraMapsOpticalAxesToFrd)
{
  Pose3d body_camera;
  body_camera.translation = Eigen::Vector3d(0.0, 0.0, -0.1);
  body_camera.rotation = Eigen::Quaterniond(
    Eigen::AngleAxisd(kPi / 2.0, Eigen::Vector3d::UnitZ()));

  Pose3d camera_marker;
  camera_marker.translation = Eigen::Vector3d(1.0, 2.0, 3.0);

  const auto result = transform_marker_to_local_ned(
    Pose3d{}, PoseReferenceFrame::kLocalNed, body_camera, camera_marker);

  ASSERT_TRUE(result.has_value());
  expect_vector_near(result->translation, Eigen::Vector3d(-2.0, 1.0, 2.9));
}

TEST(CoordinateTransformTest, VehicleYawRotatesBodyForwardIntoNed)
{
  Pose3d camera_marker;
  camera_marker.translation = Eigen::Vector3d(1.0, 0.0, 0.0);

  const struct
  {
    double yaw_rad;
    Eigen::Vector3d expected;
  } cases[] = {
    {0.0, Eigen::Vector3d(1.0, 0.0, 0.0)},
    {kPi / 2.0, Eigen::Vector3d(0.0, 1.0, 0.0)},
    {kPi, Eigen::Vector3d(-1.0, 0.0, 0.0)},
    {-kPi / 2.0, Eigen::Vector3d(0.0, -1.0, 0.0)},
  };

  for (const auto & test_case : cases) {
    Pose3d local_body;
    local_body.rotation = Eigen::Quaterniond(
      Eigen::AngleAxisd(test_case.yaw_rad, Eigen::Vector3d::UnitZ()));

    const auto result = transform_marker_to_local_ned(
      local_body, PoseReferenceFrame::kLocalNed, Pose3d{}, camera_marker);

    ASSERT_TRUE(result.has_value());
    expect_vector_near(result->translation, test_case.expected, 1.0e-8);
  }
}

TEST(CoordinateTransformTest, VehicleRollAndPitchRotateBodyAxesInNed)
{
  Pose3d body_right_marker;
  body_right_marker.translation = Eigen::Vector3d(0.0, 1.0, 0.0);

  Pose3d rolled_body;
  rolled_body.rotation = Eigen::Quaterniond(
    Eigen::AngleAxisd(kPi / 2.0, Eigen::Vector3d::UnitX()));
  const auto roll_result = transform_marker_to_local_ned(
    rolled_body, PoseReferenceFrame::kLocalNed, Pose3d{}, body_right_marker);
  ASSERT_TRUE(roll_result.has_value());
  expect_vector_near(roll_result->translation, Eigen::Vector3d(0.0, 0.0, 1.0), 1.0e-8);

  Pose3d body_forward_marker;
  body_forward_marker.translation = Eigen::Vector3d(1.0, 0.0, 0.0);

  Pose3d pitched_body;
  pitched_body.rotation = Eigen::Quaterniond(
    Eigen::AngleAxisd(kPi / 2.0, Eigen::Vector3d::UnitY()));
  const auto pitch_result = transform_marker_to_local_ned(
    pitched_body, PoseReferenceFrame::kLocalNed, Pose3d{}, body_forward_marker);
  ASSERT_TRUE(pitch_result.has_value());
  expect_vector_near(pitch_result->translation, Eigen::Vector3d(0.0, 0.0, -1.0), 1.0e-8);
}

TEST(CoordinateTransformTest, RollPitchYawAndMarkerRotationAreComposed)
{
  Pose3d local_body;
  local_body.translation = Eigen::Vector3d(10.0, -4.0, 2.0);
  local_body.rotation =
    Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX());

  Pose3d body_camera;
  body_camera.translation = Eigen::Vector3d(0.1, 0.2, -0.3);
  body_camera.rotation = Eigen::Quaterniond(
    Eigen::AngleAxisd(kPi / 2.0, Eigen::Vector3d::UnitZ()));

  Pose3d camera_marker;
  camera_marker.translation = Eigen::Vector3d(0.5, -0.4, 2.5);
  camera_marker.rotation = Eigen::Quaterniond(
    Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitY()));

  const auto result = transform_marker_to_local_ned(
    local_body, PoseReferenceFrame::kLocalNed, body_camera, camera_marker);

  ASSERT_TRUE(result.has_value());
  const Eigen::Vector3d expected_translation = local_body.translation +
    local_body.rotation *
    (body_camera.translation + body_camera.rotation * camera_marker.translation);
  const Eigen::Quaterniond expected_rotation =
    local_body.rotation * body_camera.rotation * camera_marker.rotation;
  expect_vector_near(result->translation, expected_translation, 1.0e-8);
  expect_same_rotation(result->rotation, expected_rotation, 1.0e-8);
}

TEST(CoordinateTransformTest, FiniteUnnormalizedQuaternionIsNormalized)
{
  Pose3d local_body;
  local_body.rotation = Eigen::Quaterniond(2.0, 0.0, 0.0, 0.0);

  Pose3d camera_marker;
  camera_marker.translation = Eigen::Vector3d(1.0, 2.0, 3.0);

  const auto result = transform_marker_to_local_ned(
    local_body, PoseReferenceFrame::kLocalNed, Pose3d{}, camera_marker);

  ASSERT_TRUE(result.has_value());
  expect_vector_near(result->translation, camera_marker.translation);
  EXPECT_NEAR(result->rotation.norm(), 1.0, kTolerance);
}

TEST(CoordinateTransformTest, RejectsInvalidTranslationAndQuaternion)
{
  Pose3d invalid_translation;
  invalid_translation.translation.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(transform_marker_to_local_ned(
    invalid_translation, PoseReferenceFrame::kLocalNed, Pose3d{}, Pose3d{}).has_value());

  Pose3d infinite_translation;
  infinite_translation.translation.z() = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(transform_marker_to_local_ned(
    Pose3d{}, PoseReferenceFrame::kLocalNed, Pose3d{}, infinite_translation).has_value());

  Pose3d zero_quaternion;
  zero_quaternion.rotation = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
  EXPECT_FALSE(transform_marker_to_local_ned(
    Pose3d{}, PoseReferenceFrame::kLocalNed, zero_quaternion, Pose3d{}).has_value());

  Pose3d tiny_quaternion;
  tiny_quaternion.rotation = Eigen::Quaterniond(1.0e-9, 0.0, 0.0, 0.0);
  EXPECT_FALSE(transform_marker_to_local_ned(
    Pose3d{}, PoseReferenceFrame::kLocalNed, Pose3d{}, tiny_quaternion).has_value());

  Pose3d nan_quaternion;
  nan_quaternion.rotation = Eigen::Quaterniond(
    std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0);
  EXPECT_FALSE(transform_marker_to_local_ned(
    Pose3d{}, PoseReferenceFrame::kLocalNed, Pose3d{}, nan_quaternion).has_value());
}

TEST(CoordinateTransformTest, RejectsNonNedVehiclePoseFrame)
{
  EXPECT_FALSE(transform_marker_to_local_ned(
    Pose3d{}, PoseReferenceFrame::kUnknown, Pose3d{}, Pose3d{}).has_value());
  EXPECT_FALSE(transform_marker_to_local_ned(
    Pose3d{}, PoseReferenceFrame::kLocalFrd, Pose3d{}, Pose3d{}).has_value());
}

TEST(CoordinateTransformTest, EnuAndNedConversionsAreInverse)
{
  const Eigen::Vector3d enu(2.0, 3.0, 4.0);
  const auto ned = enu_to_ned(enu);
  ASSERT_TRUE(ned.has_value());
  expect_vector_near(*ned, Eigen::Vector3d(3.0, 2.0, -4.0));

  const auto round_trip = ned_to_enu(*ned);
  ASSERT_TRUE(round_trip.has_value());
  expect_vector_near(*round_trip, enu);

  Eigen::Vector3d invalid = enu;
  invalid.y() = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(enu_to_ned(invalid).has_value());
  EXPECT_FALSE(ned_to_enu(invalid).has_value());
}

}  // namespace
}  // namespace aruco_precision_landing_cpp
