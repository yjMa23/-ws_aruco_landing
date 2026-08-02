// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/deck_plane_geometry.hpp"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

namespace aruco_precision_landing_cpp
{
namespace
{

constexpr double kTolerance = 1.0e-9;
constexpr double kTwoDegrees = 2.0 * 3.14159265358979323846 / 180.0;

Eigen::Vector3d normal_from_roll_pitch(double roll_rad, double pitch_rad)
{
  return Eigen::Vector3d{
    -std::sin(pitch_rad) * std::cos(roll_rad),
    std::sin(roll_rad),
    -std::cos(pitch_rad) * std::cos(roll_rad)};
}

DeckPlaneGeometryInput make_horizontal_input()
{
  DeckPlaneGeometryInput input;
  input.deck_reference_position_ned_m = Eigen::Vector3d::Zero();
  input.upward_normal_ned = Eigen::Vector3d{0.0, 0.0, -1.0};
  input.uav_reference_position_ned_m = Eigen::Vector3d{0.0, 0.0, -1.0};
  input.body_frd_to_ned = Eigen::Quaterniond::Identity();
  input.contact_points_body_frd_m =
    DeckPlaneGeometry::x500_default_contact_points_body_frd_m();
  return input;
}

TEST(DeckPlaneGeometryTest, FreezesX500EquivalentContactPointsFromSdf)
{
  const auto points = DeckPlaneGeometry::x500_default_contact_points_body_frd_m();

  EXPECT_TRUE(points[0].isApprox(Eigen::Vector3d{-0.125, -0.132, 0.227}, kTolerance));
  EXPECT_TRUE(points[1].isApprox(Eigen::Vector3d{0.125, -0.132, 0.227}, kTolerance));
  EXPECT_TRUE(points[2].isApprox(Eigen::Vector3d{-0.125, 0.132, 0.227}, kTolerance));
  EXPECT_TRUE(points[3].isApprox(Eigen::Vector3d{0.125, 0.132, 0.227}, kTolerance));
}

TEST(DeckPlaneGeometryTest, HorizontalPlaneDegeneratesToDeckZMinusUavZ)
{
  auto input = make_horizontal_input();
  input.deck_reference_position_ned_m.z() = 2.0;
  input.uav_reference_position_ned_m.z() = -3.0;

  const auto result = DeckPlaneGeometry::compute(input);

  ASSERT_TRUE(result.valid) << result.failure_reason;
  EXPECT_TRUE(result.output.upward_normal_ned.isApprox(
      Eigen::Vector3d{0.0, 0.0, -1.0}, kTolerance));
  EXPECT_NEAR(result.output.body_normal_gap_m, 5.0, kTolerance);
  for (double gap : result.output.contact_gaps_m) {
    EXPECT_NEAR(gap, 5.0 - 0.227, kTolerance);
  }
  EXPECT_NEAR(result.output.contact_gap_spread_m, 0.0, kTolerance);
  EXPECT_EQ(result.output.first_contact_index, 0U);
}

TEST(DeckPlaneGeometryTest, NormalizesFiniteNonUnitUpwardNormal)
{
  auto input = make_horizontal_input();
  input.upward_normal_ned = Eigen::Vector3d{0.0, 0.0, -4.0};

  const auto result = DeckPlaneGeometry::compute(input);

  ASSERT_TRUE(result.valid) << result.failure_reason;
  EXPECT_TRUE(result.output.upward_normal_ned.isApprox(
      Eigen::Vector3d{0.0, 0.0, -1.0}, kTolerance));
  EXPECT_NEAR(result.output.body_normal_gap_m, 1.0, kTolerance);
}

TEST(DeckPlaneGeometryTest, RejectsDownwardZeroAndNonFiniteNormals)
{
  auto input = make_horizontal_input();
  input.upward_normal_ned = Eigen::Vector3d{0.0, 0.0, 1.0};
  EXPECT_FALSE(DeckPlaneGeometry::compute(input).valid);

  input.upward_normal_ned = Eigen::Vector3d::Zero();
  EXPECT_FALSE(DeckPlaneGeometry::compute(input).valid);

  input.upward_normal_ned = Eigen::Vector3d{
    std::numeric_limits<double>::quiet_NaN(), 0.0, -1.0};
  EXPECT_FALSE(DeckPlaneGeometry::compute(input).valid);

  input.upward_normal_ned = Eigen::Vector3d{
    0.0, std::numeric_limits<double>::infinity(), -1.0};
  EXPECT_FALSE(DeckPlaneGeometry::compute(input).valid);
}

TEST(DeckPlaneGeometryTest, RejectsInvalidPositionsContactPointsAndParameters)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  auto input = make_horizontal_input();
  input.deck_reference_position_ned_m.x() = nan;
  EXPECT_FALSE(DeckPlaneGeometry::compute(input).valid);

  input = make_horizontal_input();
  input.uav_reference_position_ned_m.y() = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(DeckPlaneGeometry::compute(input).valid);

  input = make_horizontal_input();
  input.contact_points_body_frd_m[2].z() = nan;
  EXPECT_FALSE(DeckPlaneGeometry::compute(input).valid);

  input = make_horizontal_input();
  EXPECT_FALSE(DeckPlaneGeometry::compute(input, {0.0, 0.5}).valid);
  EXPECT_FALSE(DeckPlaneGeometry::compute(input, {1.0e-6, 0.0}).valid);
  EXPECT_FALSE(DeckPlaneGeometry::compute(input, {1.0e-6, 1.1}).valid);
}

TEST(DeckPlaneGeometryTest, RecoversNonUnitQuaternionAndRejectsInvalidQuaternion)
{
  auto input = make_horizontal_input();
  input.body_frd_to_ned = Eigen::Quaterniond{2.0, 0.0, 0.0, 0.0};
  EXPECT_TRUE(DeckPlaneGeometry::compute(input).valid);

  input.body_frd_to_ned = Eigen::Quaterniond{0.0, 0.0, 0.0, 0.0};
  EXPECT_FALSE(DeckPlaneGeometry::compute(input).valid);

  input.body_frd_to_ned = Eigen::Quaterniond{
    std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 1.0};
  EXPECT_FALSE(DeckPlaneGeometry::compute(input).valid);
}

TEST(DeckPlaneGeometryTest, ConvertsBodyContactPointsThroughYawNinetyDegrees)
{
  auto input = make_horizontal_input();
  input.uav_reference_position_ned_m = Eigen::Vector3d{1.0, 2.0, -1.0};
  input.body_frd_to_ned = Eigen::Quaterniond(
    Eigen::AngleAxisd(3.14159265358979323846 / 2.0, Eigen::Vector3d::UnitZ()));

  const auto result = DeckPlaneGeometry::compute(input);

  ASSERT_TRUE(result.valid) << result.failure_reason;
  EXPECT_TRUE(result.output.contact_positions_ned_m[0].isApprox(
      Eigen::Vector3d{1.132, 1.875, -0.773}, 1.0e-12));
  EXPECT_TRUE(result.output.contact_positions_ned_m[3].isApprox(
      Eigen::Vector3d{0.868, 2.125, -0.773}, 1.0e-12));
}

TEST(DeckPlaneGeometryTest, BodyRollPitchAndCombinedAttitudeChangeContactGeometry)
{
  auto input = make_horizontal_input();
  const auto level = DeckPlaneGeometry::compute(input);
  ASSERT_TRUE(level.valid);

  input.body_frd_to_ned = Eigen::Quaterniond(
    Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()));
  const auto roll = DeckPlaneGeometry::compute(input);
  ASSERT_TRUE(roll.valid);
  EXPECT_GT(roll.output.contact_gap_spread_m, 0.0);

  input.body_frd_to_ned = Eigen::Quaterniond(
    Eigen::AngleAxisd(-0.08, Eigen::Vector3d::UnitY()));
  const auto pitch = DeckPlaneGeometry::compute(input);
  ASSERT_TRUE(pitch.valid);
  EXPECT_GT(pitch.output.contact_gap_spread_m, 0.0);

  input.body_frd_to_ned =
    Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(-0.08, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX());
  const auto combined = DeckPlaneGeometry::compute(input);
  ASSERT_TRUE(combined.valid);
  EXPECT_GT(combined.output.contact_gap_spread_m, 0.0);
  EXPECT_FALSE(combined.output.contact_positions_ned_m[0].isApprox(
      level.output.contact_positions_ned_m[0], 1.0e-6));
}

TEST(DeckPlaneGeometryTest, TwoDegreeRollHasExpectedSpreadAndContactOrder)
{
  auto input = make_horizontal_input();
  input.upward_normal_ned = normal_from_roll_pitch(kTwoDegrees, 0.0);
  const auto positive = DeckPlaneGeometry::compute(input);
  ASSERT_TRUE(positive.valid) << positive.failure_reason;
  EXPECT_NEAR(positive.output.contact_gap_spread_m, 0.264 * std::sin(kTwoDegrees), 1.0e-9);
  EXPECT_EQ(positive.output.first_contact_index, 0U);
  EXPECT_NEAR(positive.output.contact_gaps_m[0], positive.output.contact_gaps_m[1], kTolerance);
  EXPECT_LT(positive.output.contact_gaps_m[0], positive.output.contact_gaps_m[2]);

  input.upward_normal_ned = normal_from_roll_pitch(-kTwoDegrees, 0.0);
  const auto negative = DeckPlaneGeometry::compute(input);
  ASSERT_TRUE(negative.valid) << negative.failure_reason;
  EXPECT_NEAR(negative.output.contact_gap_spread_m, 0.264 * std::sin(kTwoDegrees), 1.0e-9);
  EXPECT_EQ(negative.output.first_contact_index, 2U);
  EXPECT_LT(negative.output.contact_gaps_m[2], negative.output.contact_gaps_m[0]);
}

TEST(DeckPlaneGeometryTest, TwoDegreePitchHasExpectedSpreadAndContactOrder)
{
  auto input = make_horizontal_input();
  input.upward_normal_ned = normal_from_roll_pitch(0.0, kTwoDegrees);
  const auto positive = DeckPlaneGeometry::compute(input);
  ASSERT_TRUE(positive.valid) << positive.failure_reason;
  EXPECT_NEAR(positive.output.contact_gap_spread_m, 0.250 * std::sin(kTwoDegrees), 1.0e-9);
  EXPECT_EQ(positive.output.first_contact_index, 1U);
  EXPECT_LT(positive.output.contact_gaps_m[1], positive.output.contact_gaps_m[0]);

  input.upward_normal_ned = normal_from_roll_pitch(0.0, -kTwoDegrees);
  const auto negative = DeckPlaneGeometry::compute(input);
  ASSERT_TRUE(negative.valid) << negative.failure_reason;
  EXPECT_NEAR(negative.output.contact_gap_spread_m, 0.250 * std::sin(kTwoDegrees), 1.0e-9);
  EXPECT_EQ(negative.output.first_contact_index, 0U);
  EXPECT_LT(negative.output.contact_gaps_m[0], negative.output.contact_gaps_m[1]);
}

TEST(DeckPlaneGeometryTest, CombinedDeckTiltProducesFiniteOrderedClearances)
{
  auto input = make_horizontal_input();
  input.upward_normal_ned = normal_from_roll_pitch(kTwoDegrees, -kTwoDegrees);

  const auto result = DeckPlaneGeometry::compute(input);

  ASSERT_TRUE(result.valid) << result.failure_reason;
  EXPECT_TRUE(std::isfinite(result.output.minimum_contact_gap_m));
  EXPECT_TRUE(std::isfinite(result.output.maximum_contact_gap_m));
  EXPECT_LE(result.output.minimum_contact_gap_m, result.output.maximum_contact_gap_m);
  EXPECT_NEAR(
    result.output.contact_gap_spread_m,
    result.output.maximum_contact_gap_m - result.output.minimum_contact_gap_m,
    kTolerance);
}

TEST(DeckPlaneGeometryTest, TranslatingUavParallelToFixedPlanePreservesSpread)
{
  auto input = make_horizontal_input();
  input.upward_normal_ned = normal_from_roll_pitch(kTwoDegrees, kTwoDegrees);
  const auto original = DeckPlaneGeometry::compute(input);
  ASSERT_TRUE(original.valid);

  input.uav_reference_position_ned_m += Eigen::Vector3d{2.0, -3.0, 0.4};
  const auto translated = DeckPlaneGeometry::compute(input);
  ASSERT_TRUE(translated.valid);

  EXPECT_NE(original.output.body_normal_gap_m, translated.output.body_normal_gap_m);
  EXPECT_NEAR(
    original.output.contact_gap_spread_m,
    translated.output.contact_gap_spread_m,
    kTolerance);
}

TEST(DeckPlaneGeometryTest, HorizontalTangentialPositionMatchesNedXyError)
{
  auto input = make_horizontal_input();
  input.uav_reference_position_ned_m = Eigen::Vector3d{1.2, -0.7, -2.0};
  input.deck_reference_position_ned_m = Eigen::Vector3d{0.2, 0.3, 1.0};

  const auto result = DeckPlaneGeometry::compute(input);

  ASSERT_TRUE(result.valid) << result.failure_reason;
  EXPECT_TRUE(result.output.tangential_position_error_ned_m.isApprox(
      Eigen::Vector3d{1.0, -1.0, 0.0}, kTolerance));
  EXPECT_NEAR(result.output.body_normal_gap_m, 3.0, kTolerance);
}

TEST(DeckPlaneGeometryTest, ComputesHorizontalAndHeaveNormalRelativeVelocity)
{
  auto input = make_horizontal_input();
  input.uav_linear_velocity_ned_mps = Eigen::Vector3d{0.0, 0.0, 0.2};
  input.deck_linear_velocity_ned_mps = Eigen::Vector3d{0.0, 0.0, 0.1};
  input.uav_angular_velocity_body_frd_radps = Eigen::Vector3d::Zero();
  input.deck_angular_velocity_ned_radps = Eigen::Vector3d::Zero();

  const auto result = DeckPlaneGeometry::compute(input);

  ASSERT_TRUE(result.valid) << result.failure_reason;
  ASSERT_TRUE(result.output.body_normal_relative_velocity_mps.has_value());
  EXPECT_NEAR(*result.output.body_normal_relative_velocity_mps, -0.1, kTolerance);
  ASSERT_TRUE(result.output.tangential_relative_velocity_ned_mps.has_value());
  EXPECT_TRUE(result.output.tangential_relative_velocity_ned_mps->isApprox(
      Eigen::Vector3d::Zero(), kTolerance));
  for (const auto & velocity : result.output.contact_normal_relative_velocity_mps) {
    ASSERT_TRUE(velocity.has_value());
    EXPECT_NEAR(*velocity, -0.1, kTolerance);
  }
}

TEST(DeckPlaneGeometryTest, CommonLinearMotionHasZeroRelativeVelocity)
{
  auto input = make_horizontal_input();
  const Eigen::Vector3d common_velocity{0.4, -0.2, 0.15};
  input.uav_linear_velocity_ned_mps = common_velocity;
  input.deck_linear_velocity_ned_mps = common_velocity;
  input.uav_angular_velocity_body_frd_radps = Eigen::Vector3d::Zero();
  input.deck_angular_velocity_ned_radps = Eigen::Vector3d::Zero();

  const auto result = DeckPlaneGeometry::compute(input);

  ASSERT_TRUE(result.valid) << result.failure_reason;
  EXPECT_NEAR(*result.output.body_normal_relative_velocity_mps, 0.0, kTolerance);
  EXPECT_TRUE(result.output.tangential_relative_velocity_ned_mps->isApprox(
      Eigen::Vector3d::Zero(), kTolerance));
  for (const auto & velocity : result.output.contact_normal_relative_velocity_mps) {
    EXPECT_NEAR(*velocity, 0.0, kTolerance);
  }
}

TEST(DeckPlaneGeometryTest, ProjectsFixedTiltTranslationIntoNormalAndTangentialParts)
{
  auto input = make_horizontal_input();
  input.upward_normal_ned = normal_from_roll_pitch(kTwoDegrees, -kTwoDegrees);
  input.uav_linear_velocity_ned_mps = Eigen::Vector3d{0.3, -0.2, 0.1};
  input.deck_linear_velocity_ned_mps = Eigen::Vector3d{-0.1, 0.05, -0.2};

  const auto result = DeckPlaneGeometry::compute(input);

  ASSERT_TRUE(result.valid) << result.failure_reason;
  const Eigen::Vector3d relative_velocity =
    *input.uav_linear_velocity_ned_mps - *input.deck_linear_velocity_ned_mps;
  const double expected_normal = result.output.upward_normal_ned.dot(relative_velocity);
  const Eigen::Vector3d expected_tangential =
    relative_velocity - expected_normal * result.output.upward_normal_ned;
  EXPECT_NEAR(*result.output.body_normal_relative_velocity_mps, expected_normal, kTolerance);
  EXPECT_TRUE(result.output.tangential_relative_velocity_ned_mps->isApprox(
      expected_tangential, kTolerance));
}

TEST(DeckPlaneGeometryTest, ConvertsUavBodyAngularVelocityToNedForContactVelocity)
{
  auto input = make_horizontal_input();
  input.body_frd_to_ned = Eigen::Quaterniond(
    Eigen::AngleAxisd(3.14159265358979323846 / 2.0, Eigen::Vector3d::UnitZ()));
  input.uav_linear_velocity_ned_mps = Eigen::Vector3d::Zero();
  input.deck_linear_velocity_ned_mps = Eigen::Vector3d::Zero();
  input.uav_angular_velocity_body_frd_radps = Eigen::Vector3d{1.0, 0.0, 0.0};
  input.deck_angular_velocity_ned_radps = Eigen::Vector3d::Zero();

  const auto result = DeckPlaneGeometry::compute(input);

  ASSERT_TRUE(result.valid) << result.failure_reason;
  ASSERT_TRUE(result.output.contact_normal_relative_velocity_mps[0].has_value());
  const Eigen::Vector3d omega_uav_ned{0.0, 1.0, 0.0};
  const Eigen::Vector3d arm =
    result.output.contact_positions_ned_m[0] - input.uav_reference_position_ned_m;
  const double expected = result.output.upward_normal_ned.dot(omega_uav_ned.cross(arm));
  EXPECT_NEAR(*result.output.contact_normal_relative_velocity_mps[0], expected, kTolerance);
}

TEST(DeckPlaneGeometryTest, IncludesDeckAngularVelocityAtEachContactPoint)
{
  auto input = make_horizontal_input();
  input.uav_linear_velocity_ned_mps = Eigen::Vector3d::Zero();
  input.deck_linear_velocity_ned_mps = Eigen::Vector3d::Zero();
  input.uav_angular_velocity_body_frd_radps = Eigen::Vector3d::Zero();
  input.deck_angular_velocity_ned_radps = Eigen::Vector3d{1.0, 0.0, 0.0};

  const auto result = DeckPlaneGeometry::compute(input);

  ASSERT_TRUE(result.valid) << result.failure_reason;
  for (std::size_t index = 0; index < 4; ++index) {
    const Eigen::Vector3d deck_arm =
      result.output.contact_positions_ned_m[index] - input.deck_reference_position_ned_m;
    const double expected = result.output.upward_normal_ned.dot(
      -input.deck_angular_velocity_ned_radps->cross(deck_arm));
    EXPECT_NEAR(
      *result.output.contact_normal_relative_velocity_mps[index], expected, kTolerance);
  }
  EXPECT_NE(
    *result.output.contact_normal_relative_velocity_mps[0],
    *result.output.contact_normal_relative_velocity_mps[2]);
}

TEST(DeckPlaneGeometryTest, MissingDeckAngularVelocityDoesNotInventContactVelocity)
{
  auto input = make_horizontal_input();
  input.uav_linear_velocity_ned_mps = Eigen::Vector3d::Zero();
  input.deck_linear_velocity_ned_mps = Eigen::Vector3d::Zero();
  input.uav_angular_velocity_body_frd_radps = Eigen::Vector3d::Zero();

  const auto result = DeckPlaneGeometry::compute(input);

  ASSERT_TRUE(result.valid) << result.failure_reason;
  EXPECT_TRUE(result.output.body_normal_relative_velocity_mps.has_value());
  EXPECT_TRUE(result.output.tangential_relative_velocity_ned_mps.has_value());
  for (const auto & velocity : result.output.contact_normal_relative_velocity_mps) {
    EXPECT_FALSE(velocity.has_value());
  }
  EXPECT_NE(result.output.velocity_status.find("deck angular velocity"), std::string::npos);
}

TEST(DeckPlaneGeometryTest, InvalidVelocityOnlyClearsVelocityDiagnostics)
{
  auto input = make_horizontal_input();
  input.uav_linear_velocity_ned_mps = Eigen::Vector3d{
    std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0};
  input.deck_linear_velocity_ned_mps = Eigen::Vector3d::Zero();
  input.uav_angular_velocity_body_frd_radps = Eigen::Vector3d::Zero();
  input.deck_angular_velocity_ned_radps = Eigen::Vector3d::Zero();

  const auto result = DeckPlaneGeometry::compute(input);

  ASSERT_TRUE(result.valid) << result.failure_reason;
  EXPECT_FALSE(result.output.body_normal_relative_velocity_mps.has_value());
  EXPECT_FALSE(result.output.tangential_relative_velocity_ned_mps.has_value());
  for (const auto & velocity : result.output.contact_normal_relative_velocity_mps) {
    EXPECT_FALSE(velocity.has_value());
  }
  EXPECT_NE(result.output.velocity_status.find("non-finite linear velocity"), std::string::npos);
}

TEST(DeckPlaneGeometryTest, InvalidAngularVelocityKeepsReferencePointVelocity)
{
  auto input = make_horizontal_input();
  input.uav_linear_velocity_ned_mps = Eigen::Vector3d::Zero();
  input.deck_linear_velocity_ned_mps = Eigen::Vector3d::Zero();
  input.uav_angular_velocity_body_frd_radps = Eigen::Vector3d{
    0.0, std::numeric_limits<double>::infinity(), 0.0};
  input.deck_angular_velocity_ned_radps = Eigen::Vector3d::Zero();

  const auto result = DeckPlaneGeometry::compute(input);

  ASSERT_TRUE(result.valid) << result.failure_reason;
  EXPECT_TRUE(result.output.body_normal_relative_velocity_mps.has_value());
  EXPECT_TRUE(result.output.tangential_relative_velocity_ned_mps.has_value());
  for (const auto & velocity : result.output.contact_normal_relative_velocity_mps) {
    EXPECT_FALSE(velocity.has_value());
  }
  EXPECT_NE(result.output.velocity_status.find("non-finite angular velocity"), std::string::npos);
}

}  // namespace
}  // namespace aruco_precision_landing_cpp
